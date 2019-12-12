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
#include "cbusdefs8r.h"
#include "TCPIP Stack/Announce.h"


#define TCP_SOCKET_COUNT	(5)
#define HBEATS_INTERVAL     (20)    // Number of half sec heartbeats for announce and bridge connection retrys

/*
 * CBUSETH Connection Info - one for each connection.
 */
typedef struct _CBUSETH_INFO {
  TCP_SOCKET socket;
  unsigned char idle;
} CBUSETH_INFO;
typedef BYTE CBUSETH_HANDLE;


static CBUSETH_INFO HCB[TCP_SOCKET_COUNT];

BYTE nrClients = 0;
BYTE hbeatCounter = 0;
BYTE bridgeSocketIndex = 0xFE;


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

static byte msg2ASCII(CANMsg* msg, char* s) {

  if (msg->b[sidl] & 0x08) {
    /* Extended Frame */
    byte len = msg->b[dlc] & 0x0F;
    byte i;
    s[0] = ':';
    s[1] = 'X';
    s[2] = hexa[msg->b[sidh] >> 4];
    s[3] = hexa[msg->b[sidh] & 0x0F];
    s[4] = hexa[msg->b[sidl] >> 4];
    s[5] = hexa[msg->b[sidl] & 0x0F];
    s[6] = hexa[msg->b[eidh] >> 4];
    s[7] = hexa[msg->b[eidh] & 0x0F];
    s[8] = hexa[msg->b[eidl] >> 4];
    s[9] = hexa[msg->b[eidl] & 0x0F];
    s[10] = ((msg->b[dlc] & 0x40) ? 'R' : 'N');
    for (i = 0; i < len; i++) {
      s[11 + i * 2] = hexa[msg->b[d0 + i] >> 4];
      s[11 + i * 2 + 1] = hexa[msg->b[d0 + i] & 0x0F];
    }
    s[11 + i * 2] = ';';
    return 11 + i * 2 + 1;
  } else {
    /* Standard Frame */
    byte len = getDataLen(msg->b[d0], ((msg->b[dlc]&0x80) ? TRUE : FALSE));
    byte i;
    byte idx = 9;
    s[0] = ':';
    s[1] = ((msg->b[dlc] & 0x80) ? 'Y' : 'S');
    s[2] = hexa[msg->b[sidh] >> 4];
    s[3] = hexa[msg->b[sidh] & 0x0F];
    s[4] = hexa[msg->b[sidl] >> 4];
    s[5] = hexa[msg->b[sidl] & 0x0F];
    s[6] = ((msg->b[dlc] & 0x40) ? 'R' : 'N');
    s[7] = hexa[msg->b[d0] >> 4];
    s[8] = hexa[msg->b[d0] & 0x0F];
    for (i = 0; i < len; i++) {
      s[idx + i * 2] = hexa[msg->b[d1 + i] >> 4];
      s[idx + i * 2 + 1] = hexa[msg->b[d1 + i] & 0x0F];
    }
    s[idx + i * 2] = ';';
    return idx + i * 2 + 1;
  }
}

/*  StrOp.fmtb( frame+1, ":%c%02X%02XN%02X;", eth?'Y':'S', (0x80 + (prio << 5) + (cid >> 3)) &0xFF, (cid << 5) & 0xFF, cmd[0] );*/
static char hexb[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 0, 0, 0, 0, 0, 0, 10, 11, 12, 13, 14, 15};

static byte ASCII2Msg(unsigned char* ins, byte inlen, CANMsg* msg) {
  byte len = 0;
  byte i;
  byte type = CAN_FRAME;

  unsigned char* s = ins;
  for (i = 0; i < inlen; i++) {
    if (s[i] == ':' && (s[i + 1] == 'S' || s[i + 1] == 'Y')) {
      if (s[i + 1] == 'Y') {
        type = ETH_FRAME;
      }
      s += i;
      break;
    } else if (s[i] == ':' && s[i + 1] == 'X') {
      type = EXT_FRAME;
      s += i;
      break;
    }
  }
  if (i == inlen) {
    return 0;
  }

  if (type == EXT_FRAME) {
    msg->b[sidh] = (hexb[s[2] - 0x30] << 4) + hexb[s[3] - 0x30];
    msg->b[sidl] = (hexb[s[4] - 0x30] << 4) + hexb[s[5] - 0x30];
    msg->b[eidh] = (hexb[s[6] - 0x30] << 4) + hexb[s[7] - 0x30];
    msg->b[eidl] = (hexb[s[8] - 0x30] << 4) + hexb[s[9] - 0x30];
    // copying all data bytes:
    for (i = 0; i < 8 && s[11 + 2 * i] != ';'; i++) {
      msg->b[d0 + i] = (hexb[s[11 + 2 * i] - 0x30] << 4) + hexb[s[11 + 2 * i + 1] - 0x30];
    }
    msg->b[dlc] = i;
    if (s[10] == 'R')
      msg->b[dlc] |= 0x40;
  } else {
    byte l_sidh, l_sidl, canid;
    l_sidh = (hexb[s[2] - 0x30] << 4) + hexb[s[3] - 0x30];
    l_sidl = (hexb[s[4] - 0x30] << 4) + hexb[s[5] - 0x30];
    canid = (l_sidl >> 5) + ((l_sidh & 0x0F) << 3);

    if (CANID != canid) {
      // Use own CANID
      //CANID = canid;
    }
    msg->b[sidh] = l_sidh;
    msg->b[sidl] = l_sidl;

    msg->b[d0] = (hexb[s[7] - 0x30] << 4) + hexb[s[8] - 0x30];
    len = getDataLen(msg->b[d0], FALSE);
    for (i = 0; i < len; i++) {
      msg->b[d1 + i] = (hexb[s[9 + 2 * i] - 0x30] << 4) + hexb[s[9 + 2 * i + 1] - 0x30];
    }
    msg->b[dlc] = len + 1;
    if (s[6] == 'R')
      msg->b[dlc] |= 0x40;
  }
  return type;
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
  byte conn;
  byte nrconn = 0;

  for (conn = 0; conn < TCP_SOCKET_COUNT; conn++) {
    BYTE cbusData[MAX_CBUSETH_CMD_LEN + 1];
    CBUSETH_INFO* ph = &HCB[conn];

    if (!TCPIsConnected(ph->socket)) {
      ph->idle = 0;
      continue;
    }
    nrconn++;

    if (TCPIsGetReady(ph->socket)) {
      BYTE len = 0;
      BYTE type;
      CANMsg canmsg;
      ph->idle = 0;

      while (len < MAX_CBUSETH_CMD_LEN && TCPGet(ph->socket, &cbusData[len])) {
        if (cbusData[len++] == ';') {
          cbusData[len] = '\0';
          if (cbusData[0] == ':') {
            led2timer = 2;
            LED2 = LED_ON;
            type = ASCII2Msg(cbusData, len, &canmsg);
            if (type > 0) {
              if (parseCmdEth(&canmsg, type)) {
                CBusEthBroadcast(&canmsg, conn);  // Send message to other connected Ethernet nodes
              }
            }
          }
          len = 0;
        }
      }
      TCPDiscard(ph->socket);
    } else if (ph->idle > IdleTime) {
      TCPDisconnect(ph->socket);
      ph->idle = 0;
      if (NV1 & CFG_POWEROFF_ATIDLE) {
        CANMsg canmsg;
        canmsg.b[d0] = OPC_RTOF;
        canmsg.b[dlc] = 1;
#ifdef WITHCAN        
        canbusSend(&canmsg);
#endif        
      }
    }
  }
  return nrconn;
}

// Send CBUS message to each connected TCP socket.
// Modified to to always send to CAN as well as Ethernet and to
// pass a parameter of an index into the connection table to exlude
// so can be used to send message to all other sockets except the one that it was received on
// To send to all connected sockets, set excludeConn to 0xFF
// TO send to everywhere except CAN, set excludeConn to 0xFE - PNB 1/1/13

BYTE CBusEthBroadcast(CANMsg *msg, BYTE excludeConn) 
{

  BYTE conn;
  char s[32];
  BYTE len;
  BYTE err;

  
  err = FALSE;
  len = msg2ASCII(msg, s);

  for (conn = 0; conn < TCP_SOCKET_COUNT; conn++) 
  {
    if (conn != excludeConn) {
        CBUSETH_INFO* ph = &HCB[conn];
        if (TCPIsConnected(ph->socket)) {
          if (TCPIsPutReady(ph->socket) > len) {
            TCPPutArray(ph->socket, (byte*) s, len);
            led2timer = 2;
            LED2 = LED_ON;
          } else {
            LED3 = LED_OFF;
            led3timer = 100;
            err++;
          }
        }
    }
  }

#ifdef WITHCAN
  if ((excludeConn != 0xFE) && !(msg->b[dlc] & 0x80))
    canbusSend(msg); //  Send to CAN as well as Ethernet except for Y frames
#endif
  
  return err;
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
