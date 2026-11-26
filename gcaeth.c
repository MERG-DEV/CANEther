/*
 Rocrail - Model Railroad Software

 Copyright (C) Rob Versluis <r.j.versluis@rocrail.net>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 as published by the Free Software Foundation; either version 3
 of the License, or (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <string.h>

#include <TCPIP Stack/TCPIP.h>

#include "project.h"
#include "gcaeth.h"
#include "canbus.h"
#include "commands.h"
#include "utils.h"
#include "io.h"
#include "cangc1e.h"
#include "cbusdefs8j.h"
#include "TCPIP Stack/Announce.h"


#define TCP_SOCKET_COUNT	(5)
#define HBEATS_INTERVAL     (20)    // Number of half sec heartbeats for announce and bridge connection retrys

/*
 * CBUSETH Connection Info - one for each connection.
 */
typedef struct _CBUSETH_INFO {
  TCP_SOCKET socket;
  unsigned char idle;
  BYTE rxLen;                           // characters of the frame being received; 0 = waiting for ':'
  BYTE rxBuf[MAX_CBUSETH_CMD_LEN];      // the frame being received, which may arrive in pieces
} CBUSETH_INFO;
typedef BYTE CBUSETH_HANDLE;

#define ALL_CONNECTIONS ((BYTE) ((1 << TCP_SOCKET_COUNT) - 1))   // one bit per connection

#ifdef WITHCAN
#define CAN_TX_RESERVE 2    // CAN frames one received frame can cause: itself and a reply (QNN causes both)
#endif

static CBUSETH_INFO HCB[TCP_SOCKET_COUNT];

BYTE nrClients = 0;
BYTE hbeatCounter = 0;
BYTE bridgeSocketIndex = 0xFE;
BYTE ethDropCount = 0;      // frames a connection did not get because its send buffer stayed full
BYTE gcRejectCount = 0;     // malformed or overlong frames received and discarded

static BYTE pendingFlush = 0;   // connections given frames since the last CBusEthFlush
static BYTE firstConn = 0;      // connection read first, taken in turn so all get a fair share


static BYTE CBusEthProcess(void);



/* Frame ASCII format
 * :ShhhhNd0d1d2d3d4d5d6d7d; :XhhhhhhhhNd0d1d2d3d4d5d6d7d; :ShhhhR; :SB020N;
 * :S    -> S=Standard X=extended start CAN Frame
 * hhhh  -> SIDH<bit7,6,5,4=Prio bit3,2,1,0=high 4 part of ID> SIDL<bit7,6,5=low 3 part of ID>
 * Nd    -> N=normal R=RTR
 * 0d    -> OPC 2 byte HEXA
 * 1d-7d -> data 2 byte HEXA
 * ;     -> end of frame
 */

static char hexa[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

// Frame to GridConnect text, at most 28 characters. Returns its length.
// The data length of a CAN frame comes from its DLC. The CANEther's own
// status frames (Y) keep the length their opcode implies, as before.

static byte msg2ASCII(CANMsg* msg, char* s) {
  BYTE len;
  BYTE i;
  BYTE idx;
  BOOL extended = (msg->b[sidl] & 0x08) != 0;
  BOOL eth = !extended && (msg->b[dlc] & 0x80);

  s[0] = ':';
  s[1] = (extended ? 'X' : (eth ? 'Y' : 'S'));
  s[2] = hexa[msg->b[sidh] >> 4];
  s[3] = hexa[msg->b[sidh] & 0x0F];
  s[4] = hexa[msg->b[sidl] >> 4];
  s[5] = hexa[msg->b[sidl] & 0x0F];
  idx = 6;
  if (extended) {
    s[6] = hexa[msg->b[eidh] >> 4];
    s[7] = hexa[msg->b[eidh] & 0x0F];
    s[8] = hexa[msg->b[eidl] >> 4];
    s[9] = hexa[msg->b[eidl] & 0x0F];
    idx = 10;
  }
  s[idx++] = ((msg->b[dlc] & 0x40) ? 'R' : 'N');

  if (eth)
    len = getDataLen(msg->b[d0], TRUE) + 1;
  else
    len = canDataLength(msg->b[dlc]);

  for (i = 0; i < len; i++) {
    s[idx++] = hexa[msg->b[d0 + i] >> 4];
    s[idx++] = hexa[msg->b[d0 + i] & 0x0F];
  }
  s[idx++] = ';';
  return idx;
}

// Value of a hex digit, either case, or 0xFF if c is not one.

static BYTE hexDigit(BYTE c) {
  if (c >= '0' && c <= '9')
    return c - '0';
  if (c >= 'A' && c <= 'F')
    return c - 'A' + 10;
  if (c >= 'a' && c <= 'f')
    return c - 'a' + 10;
  return 0xFF;
}

// Two hex digits to a byte. FALSE if either is not a hex digit.

static BOOL hexByte(BYTE* s, BYTE* val) {
  BYTE hi = hexDigit(s[0]);
  BYTE lo = hexDigit(s[1]);

  if (hi == 0xFF || lo == 0xFF)
    return FALSE;
  *val = (hi << 4) | lo;
  return TRUE;
}

// GridConnect text to a frame. s holds one whole frame, ':' to ';'.
// Returns the frame type, or 0 if the text is not a well-formed frame.
// The data length is the number of data bytes given (0 to 8).

static byte ASCII2Msg(BYTE* s, byte inlen, CANMsg* msg) {
  BYTE type;
  BYTE idBytes;
  BYTE pos;
  BYTE len;
  BYTE i;

  switch (s[1]) {
    case 'S': type = CAN_FRAME; idBytes = 2; break;
    case 'Y': type = ETH_FRAME; idBytes = 2; break;
    case 'X': type = EXT_FRAME; idBytes = 4; break;
    default:  return 0;
  }
  pos = 2 + 2 * idBytes;                // where N or R is
  if (inlen < pos + 2)
    return 0;
  len = inlen - pos - 2;                // hex digits of data
  if ((len & 1) || len > 16)
    return 0;
  len >>= 1;

  msg->b[con] = 0;
  msg->b[eidh] = 0;
  msg->b[eidl] = 0;
  for (i = 0; i < idBytes; i++)
    if (!hexByte(&s[2 + 2 * i], &msg->b[sidh + i]))
      return 0;
  if (s[pos] != 'N' && s[pos] != 'R')
    return 0;
  for (i = 0; i < len; i++)
    if (!hexByte(&s[pos + 1 + 2 * i], &msg->b[d0 + i]))
      return 0;

  if (type == EXT_FRAME)
    msg->b[sidl] |= 0x08;               // EXIDE: extended identifier
  else if (len == 0)
    return 0;                           // a CBUS frame starts with an opcode
  else
    msg->b[sidl] &= 0xF7;               // standard identifier

  msg->b[dlc] = len;
  if (s[pos] == 'R')
    msg->b[dlc] |= 0x40;
  return type;
}

// Add a character received on a connection to the frame being assembled.
// Returns the length of a complete frame, ':' to ';', now in rxBuf, or 0.
// A ':' always starts a new frame, so after a malformed or overlong frame
// reception picks up again at the next one.

static BYTE assembleFrame(CBUSETH_INFO* ph, BYTE c) {
  BYTE n;

  if (c == ':') {
    ph->rxBuf[0] = c;
    ph->rxLen = 1;
    return 0;
  }
  if (ph->rxLen == 0)
    return 0;                           // between frames
  if (ph->rxLen == MAX_CBUSETH_CMD_LEN) {
    ph->rxLen = 0;                      // too long to be a frame: drop it
    gcRejectCount++;
    return 0;
  }
  ph->rxBuf[ph->rxLen++] = c;
  if (c != ';')
    return 0;
  n = ph->rxLen;
  ph->rxLen = 0;
  return n;
}

// Act on a frame received on connection conn.

static void processFrame(BYTE* s, BYTE len, BYTE conn) {
  CANMsg canmsg;
  BYTE type;

  led2timer = 2;
  LED2 = LED_ON;
  type = ASCII2Msg(s, len, &canmsg);
  if (type == 0) {
    gcRejectCount++;
    return;
  }
  if (parseCmdEth(&canmsg, type)) {
    CBusEthBroadcast(&canmsg, conn);  // Send message to other connected Ethernet nodes and CAN
  }
}

// TRUE if a frame received from Ethernet can be taken now: the CAN transmit
// FIFO has room for what it may send. While the CAN controller is at error
// warning level, frames are taken anyway (and dropped if the FIFO is full),
// so a CAN bus fault does not hold up Ethernet.

static BOOL canTakeFrame(void) {
#ifdef WITHCAN
  return canbusTxFree() >= CAN_TX_RESERVE || canbusTxErrorWarning();
#else
  return TRUE;
#endif
}


void CBusEthInit(void) {
  BYTE i;


  // Stack has to define number of client and server sockets, so we have to reserve one for the bridge client, make this the first one in our array
       
  HCB[0].socket = 0xFE;
  openBridge();

  for (i = 1; i < TCP_SOCKET_COUNT; i++) {
    // HCB[i].socket = TCPListen(CBUSETH_PORT);
          
    HCB[i].socket = TCPOpen(0, TCP_OPEN_SERVER, eeReadShort(EE_PORTNUM), TCP_PURPOSE_GENERIC_TCP_SERVER);
    HCB[i].idle = 0;
    HCB[i].rxLen = 0;
  }
}

BOOL isBridgeConnected( void )
{
    if (bridgeSocketIndex < TCP_SOCKET_COUNT)
        return TCPIsConnected(HCB[bridgeSocketIndex].socket);
    else
        return FALSE;
}


void openBridge( void )

{
    DWORD_VAL  remoteIP;
    BYTE  conn;
    BYTE  i;

  
    if (!isBridgeConnected() && (eeRead(EE_REMOTEIP) != 0))
    {
        // Find a free socket

        if (bridgeSocketIndex < TCP_SOCKET_COUNT)
            conn = bridgeSocketIndex;
        else
        {    
            conn = 0;

            while ((conn < TCP_SOCKET_COUNT) && TCPIsConnected(HCB[conn].socket))
                conn++;
        }    


        if (conn < TCP_SOCKET_COUNT)    // do nothing if all sockets in use
        {
            if (HCB[conn].socket < TCP_SOCKET_COUNT)
                TCPClose(HCB[conn].socket); // If socket has been initialised, need to close it first

            for (i=0; i<4; i++)
              remoteIP.v[i] = eeRead(EE_REMOTEIP + i);

            HCB[conn].socket = TCPOpen( remoteIP.Val, TCP_OPEN_IP_ADDRESS, eeReadShort(EE_REMOTEPORT), TCP_PURPOSE_GENERIC_TCP_CLIENT);
            HCB[conn].idle = 0;
            HCB[conn].rxLen = 0;
            bridgeSocketIndex = conn;
        }
    }
}



void CBusEthServer(void) {
  BYTE i;
  char idx = -1;
  byte nrconn = 0;

  nrconn = CBusEthProcess();

  if (nrconn != nrClients) {
    CANMsg canmsg;
    canmsg.b[sidh] = (CANID >> 3);
    canmsg.b[sidl] = (CANID << 5);
    canmsg.b[d0] = 1;
    canmsg.b[d1] = 0;
    canmsg.b[d2] = nrconn;
    canmsg.b[d3] = maxcanq;
    canmsg.b[d4] = maxethq;
    canmsg.b[dlc] = 0x80 + 5;
    CBusEthBroadcast(&canmsg,0xFF);
    maxethq = 0;
    maxcanq = 0;
    nrClients = nrconn;
  }
}

static byte CBusEthProcess(void) {
  byte i;
  byte conn;
  byte nrconn = 0;
  BYTE c;
  BYTE len;

  for (i = 0; i < TCP_SOCKET_COUNT; i++) {
    CBUSETH_INFO* ph;

    conn = firstConn + i;
    if (conn >= TCP_SOCKET_COUNT)
      conn -= TCP_SOCKET_COUNT;
    ph = &HCB[conn];

    if (!TCPIsConnected(ph->socket)) {
      ph->idle = 0;
      ph->rxLen = 0;
      continue;
    }
    nrconn++;

    if (TCPIsGetReady(ph->socket)) {
      ph->idle = 0;

      // Take characters only while a frame can be acted on. Otherwise they
      // wait with TCP, whose receive window then makes the sender wait.
      while (canTakeFrame() && TCPGet(ph->socket, &c)) {
        len = assembleFrame(ph, c);
        if (len) {
          processFrame(ph->rxBuf, len, conn);
        }
      }
    } else if (ph->idle > IdleTime) {
      TCPDisconnect(ph->socket);
      ph->idle = 0;
      if (NV1 & CFG_POWEROFF_ATIDLE) {
        CANMsg canmsg;
        canmsg.b[sidh] = (CANID >> 3);
        canmsg.b[sidl] = (CANID << 5);
        canmsg.b[d0] = OPC_RTOF;
        canmsg.b[dlc] = 1;
#ifdef WITHCAN        
        canbusSend(&canmsg);
#endif        
      }
    }
  }
  if (++firstConn == TCP_SOCKET_COUNT)
    firstConn = 0;
  return nrconn;
}

// Send a frame to each connection in connMask (bit n for connection n).
// Returns the connections it could not be sent to because their send
// buffer was full.

BYTE CBusEthSendTo(CANMsg *msg, BYTE connMask)
{
  BYTE conn;
  BYTE bit;
  char s[32];
  BYTE len;
  BYTE failed;

  failed = 0;
  len = msg2ASCII(msg, s);

  for (conn = 0, bit = 1; conn < TCP_SOCKET_COUNT; conn++, bit <<= 1)
  {
    if (connMask & bit) {
        CBUSETH_INFO* ph = &HCB[conn];
        if (TCPIsConnected(ph->socket)) {
          if (TCPIsPutReady(ph->socket) >= len) {
            TCPPutArray(ph->socket, (byte*) s, len);
            pendingFlush |= bit;
            led2timer = 2;
            LED2 = LED_ON;
          } else {
            LED3 = LED_OFF;
            led3timer = 100;
            failed |= bit;
          }
        }
    }
  }
  return failed;
}

// Send CBUS message to each connected TCP socket.
// Modified to to always send to CAN as well as Ethernet and to
// pass a parameter of an index into the connection table to exlude
// so can be used to send message to all other sockets except the one that it was received on
// To send to all connected sockets, set excludeConn to 0xFF
// TO send to everywhere except CAN, set excludeConn to 0xFE - PNB 1/1/13
// Returns the connections that could not take the frame (see CBusEthSendTo).

BYTE CBusEthBroadcast(CANMsg *msg, BYTE excludeConn) 
{
  BYTE connMask;
  BYTE failed;

  connMask = ALL_CONNECTIONS;
  if (excludeConn < TCP_SOCKET_COUNT)
    connMask &= (BYTE) ~(1 << excludeConn);

  failed = CBusEthSendTo(msg, connMask);

#ifdef WITHCAN
  if ((excludeConn != 0xFE) && !(msg->b[dlc] & 0x80))
    canbusSend(msg); //  Send to CAN as well as Ethernet except for Y frames
#endif
  
  return failed;
}

// Send now what each connection has been given since the last call, rather
// than leaving it for the stack's auto-transmit (up to 40 ms later). Frames
// given in one pass of the main loop share a TCP segment.

void CBusEthFlush(void)
{
  BYTE conn;
  BYTE bit;

  for (conn = 0, bit = 1; conn < TCP_SOCKET_COUNT; conn++, bit <<= 1)
  {
    if (pendingFlush & bit)
      TCPFlush(HCB[conn].socket);
  }
  pendingFlush = 0;
}

/* called every 500ms */
void CBusEthTick(void) {
  byte conn;

  if (NV1 & CFG_IDLE_TIMEOUT) {
    for (conn = 0; conn < TCP_SOCKET_COUNT; conn++) {
      CBUSETH_INFO* ph = &HCB[conn];
      ph->idle++; // count always
    }
  }

  if (++hbeatCounter == HBEATS_INTERVAL)
  {
      hbeatCounter = 0;

      AnnounceIP();  // Send announcement so software can find our IP address
      // If remote IP is set for bridge mode, try and connect if not already connected

      if (eeRead(EE_REMOTEIP) != 0)
      {
          openBridge();
      }      
  }    

  
}
