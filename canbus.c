/*
 Copyright (C) MERG CBUS

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

#include <stddef.h>
#include <string.h>
#include "project.h"
#include "cangc1e.h"
#include "io.h"
#include "canbus.h"
#include "gcaeth.h"

#define SW_FIFO 16
#define SW_FIFO_MAX 32

#define CAN_TX_TIMEOUT_TICKS 2      // half-second ticks before a frame the bus will not take is abandoned

typedef struct {
  BYTE msg[16];
} CAN_PACKET;

#pragma udata VARS_FIFO_0

far CAN_PACKET Fifo0[SW_FIFO];

#pragma udata VARS_FIFO_1

far CAN_PACKET Fifo1[SW_FIFO];

#pragma udata VARS_TXFIFO

far CAN_PACKET TxFifo[CAN_TX_FIFO_SIZE];

#pragma udata

far BYTE FifoIdxW = 0;
far BYTE FifoIdxR = 0;

// Software transmit FIFO, in front of the one transmit buffer used (TXB0).
// It is shared by the main loop and the low priority interrupt: the main loop
// changes it only with low priority interrupts disabled.
static BYTE txHead;                         // next entry to fill
static BYTE txTail;                         // next entry to send
static volatile BYTE txCount;               // entries waiting

static volatile BYTE canTransmitTimeout;    // half-second ticks left for the frame in TXB0; 0 = none
static volatile BOOL canTxTimedOut;         // set by the timer interrupt when they run out
static BOOL txAbortPending;                 // abort requested; TXB0 not free until it is confirmed

BYTE  txErrCount;                           // CAN error interrupts (error state changes, overflow)
BYTE  txTimeoutCount;                       // frames abandoned because the bus would not take them
BYTE  txDropCount;                          // frames not queued because the FIFO was full
BYTE  maxCanTxFifo;                         // most frames ever waiting in the FIFO


static BYTE* _PointBuffer(BYTE b);
static void txStartNext(void);

void cbusSetup(void) {

  txErrCount = 0;
  txTimeoutCount = 0;
  txDropCount = 0;
  maxCanTxFifo = 0;
  txHead = 0;
  txTail = 0;
  txCount = 0;
  canTransmitTimeout = 0;
  canTxTimedOut = FALSE;
  txAbortPending = FALSE;
  
  // Put module into Configuration mode.
  CANCON = 0b10000000;
  // Wait for config mode
  while (CANSTATbits.OPMODE2 == 0);

  /*
    In PICs ist 125Kbps mit folgender Einstellung bereitzustellen:

  16MHz:
  -------------
  BRGCON1: 0x03
  BRGCON2: 0xDE
  BRGCON3: 0x03
   *
   * 125KHz @ 16MHz - BRGCON1=0x7 BRGCON2=0xC9 BRGCON3=0x02
   * 125KHz @ 40MHz - BRGCON1=0xF BRGCON2=0xD1 BRGCON3=0x03
   */
  /*
   * Baud rate setting.
   * Sync segment is fixed at 1 Tq
   * Total bit time is Sync + prop + phase 1 + phase 2
   * or 16 * Tq in our case
   * So, for 125kbits/s, bit time = 8us, we need Tq = 500ns
   * Fosc is 32MHz (8MHz + PLL), Tosc is 31.25ns, so we need 1:16 prescaler
   */

  // prescaler Tq = 16/Fosc
  BRGCON1 = 0b00000111; // 8MHz resonator
  //BRGCON1 = 0b00000011; // 4MHz resonator
  // freely programmable, sample once, phase 1 = 4xTq, prop time = 7xTq
  BRGCON2 = 0b10011110;
  // Wake-up enabled, wake-up filter not used, phase 2 = 4xTq
  BRGCON3 = 0b00000011;
  // TX drives Vdd when recessive, CAN capture to CCP1 disabled
  CIOCON = 0b00100000;

  // Want ECAN mode 2 with FIFO
  ECANCON = 0b10010000; // FIFOWM = 0 (four entry left)
  // EWIN default
  BSEL0 = 0; // All buffers for receive
  // FIFO is 8 deep

  RXM0SIDL = 0; // set all mask register 0 to
  RXM0SIDH = 0; // receive all valid messages
  RXM1SIDL = 0;
  RXM1SIDH = 0;

  RXM0EIDL = 0;
  RXM0EIDH = 0;
  RXM1EIDL = 0;
  RXM1EIDH = 0;

  //    RXFCON0 = 0;
  //    RXFCON1 = 0;

  //    RXFBCON0 = 0;
  //    RXFBCON1 = 0;
  //    RXFBCON2 = 0;
  //    RXFBCON3 = 0;
  //    RXFBCON4 = 0;
  //    RXFBCON5 = 0;
  //    RXFBCON6 = 0;
  //    RXFBCON7 = 0;

  RXB0CON = 0b00000000; // receive valid messages
  RXB1CON = 0b00000000; // receive valid messages
  B0CON = 0b00000000; // receive valid messages
  B1CON = 0b00000000; // receive valid messages
  B2CON = 0b00000000; // receive valid messages
  B3CON = 0b00000000; // receive valid messages
  B4CON = 0b00000000; // receive valid messages
  B5CON = 0b00000000; // receive valid messages

  BIE0 = 0; // No Rx buffer interrupts
  TXBIEbits.TXB0IE = 1; // Tx buffer interrupts from buffer 0 only
  TXBIEbits.TXB1IE = 0;
  TXBIEbits.TXB2IE = 0;
  CANCON = 0;
  PIE3bits.FIFOWMIE = 1; // Fifo 4 left int
  PIE3bits.TXBnIE = 1;   // Transmit buffer has sent its frame
  PIE3bits.ERRIE = 1;
}

// Number of data bytes in a frame, from its DLC register value.
// A DLC field of 9 to 15 is valid on the bus and means 8 bytes.

BYTE canDataLength(BYTE dlcReg)
{
    BYTE len = dlcReg & 0x0F;

    return (len > 8 ? 8 : len);
}

// Queue a frame to send to CAN, without waiting.
// Returns FALSE, and counts the frame as dropped, if the FIFO is full.
// Callers that must not drop frames check canbusTxFree() first.
// The transmit buffers do not work as a FIFO, so one buffer (TXB0) is used,
// fed from a software FIFO, which keeps frames in order.

BOOL canbusSend(CANMsg *msg)
{
  BYTE len;
  BOOL queued;

  len = canDataLength(msg->b[dlc]);

  INTCONbits.GIEL = 0;                      // FIFO is shared with the low priority interrupt
  if (txCount < CAN_TX_FIFO_SIZE)
  {
    memcpy((void *) TxFifo[txHead].msg, (void *) msg->b, len + 6);
    TxFifo[txHead].msg[con] = 0;
    TxFifo[txHead].msg[dlc] = len;          // data frame, never RTR

    if (++txHead == CAN_TX_FIFO_SIZE)
      txHead = 0;
    txCount++;
    if (txCount > maxCanTxFifo)
      maxCanTxFifo = txCount;

    txStartNext();
    queued = TRUE;
  }
  else
  {
    txDropCount++;
    queued = FALSE;
  }
  INTCONbits.GIEL = 1;

  if (!queued)
  {
    led3timer = 20;
    LED3 = LED_OFF;
  }
  return queued;
}

// Free entries in the transmit FIFO.

BYTE canbusTxFree(void)
{
  return CAN_TX_FIFO_SIZE - txCount;
}

// TRUE while the CAN controller is at error warning level or worse, for
// example when no other node acknowledges its frames.

BOOL canbusTxErrorWarning(void)
{
  return COMSTATbits.TXWARN;
}

// If the transmit buffer is idle and a frame is waiting, start sending it.
// Call only from the low priority interrupt, or with it disabled.

static void txStartNext(void)
{
  BYTE* ptr;

  if (TXB0CONbits.TXREQ)
    return;                                 // still sending

  // After an abort request, a frame already on the bus is still sent in
  // full (data sheet 23.6.2), so the buffer is free only once the abort is
  // confirmed (TXABT) or the frame has gone (canTxDone).
  if (txAbortPending)
  {
    if (!TXB0CONbits.TXABT)
      return;
    txAbortPending = FALSE;
  }

  canTransmitTimeout = 0;

  if (txCount == 0)
  {
    TXB0CON = 0;                            // nothing waiting: clear the buffer's status
    return;
  }

  ptr = (BYTE*) & TXB0CON;
  memcpy(ptr, (void *) TxFifo[txTail].msg, TxFifo[txTail].msg[dlc] + 6);
  if (++txTail == CAN_TX_FIFO_SIZE)
    txTail = 0;
  txCount--;

  canTxTimedOut = FALSE;
  canTransmitTimeout = CAN_TX_TIMEOUT_TICKS;
  TXB0CONbits.TXREQ = 1;                    // Initiate transmission
}

// Called by the low priority interrupt when the transmit buffer has sent its frame.

void canTxDone(void)
{
  txAbortPending = FALSE;
  txStartNext();
}

// Called by the high priority interrupt every half second.

void checkCANTimeout(void)
{
  if (canTransmitTimeout != 0)
    if (--canTransmitTimeout == 0)
      canTxTimedOut = TRUE;
}

// Called from the main loop on every pass.
// Abandons a frame the bus has not taken within the time-out (0.5 to 1 s),
// and starts the next queued frame whenever the transmitter is idle. An
// abandoned frame raises no interrupt, so without this the FIFO would wait.

void canbusService(void)
{
  INTCONbits.GIEL = 0;
  if (canTxTimedOut)
  {
    canTxTimedOut = FALSE;
    if (TXB0CONbits.TXREQ)
    {
      TXB0CONbits.TXREQ = 0;                // request abort
      txAbortPending = TRUE;
      txTimeoutCount++;
    }
  }
  txStartNext();
  INTCONbits.GIEL = 1;
}

// Called by the low priority interrupt on a CAN error interrupt.
// That is raised when the controller's error state changes or the receive
// FIFO overflows (PIC18F2585 data sheet, 23.15.6), not by lost arbitration.
// The controller retries a frame by itself, and canbusService abandons a
// frame the bus will not take, so here the event is only counted.

void canTxError(void)
{
  txErrCount++;
}


// This firmware version does not do self enumeration
// The RTR packet and self enumeration replies (canid but no payload) should not be
// transmitted between CBUS segments, and have no use on Ethernet,
// So they are ignored at present and not transferred

// Called by main loop to check for cbus messages received

BOOL canbusRecv(CANMsg *msg)
{

    CANMsg *ptr;
    BOOL   msgFound;
    BYTE   len;
  
    // First check for any messages in the software fifo, which the ISR will have filled if there has been a high watermark interrupt

    PIE3bits.FIFOWMIE = 0;  // Disable high watermark interrupt so ISR canot fiddle with FIFOs

    if (FifoIdxR != FifoIdxW)
    {
      if (FifoIdxR >= SW_FIFO)  
        memcpy(msg->b, &Fifo1[FifoIdxR - SW_FIFO].msg, Fifo1[FifoIdxR - SW_FIFO].msg[dlc] + 6);
      else
        memcpy(msg->b, &Fifo0[FifoIdxR].msg, Fifo0[FifoIdxR].msg[dlc] + 6);
      
      if (++FifoIdxR >= SW_FIFO_MAX)
      {
        FifoIdxR = 0;
      }
      PIE3bits.FIFOWMIE = 1;
      msgFound = TRUE;
    }
    else
    {
        msgFound = FALSE;

        while (!msgFound && COMSTATbits.FIFOEMPTY)
        {
            ptr = (CANMsg*) _PointBuffer(CANCON & 0x07);
            PIR3bits.RXBnIF = 0;
//            if (COMSTATbits.RXBnOVFL) {
//              maxcanq++; // Buffer Overflow
//              led3timer = 5;
//              LED3 = LED_OFF;
//              COMSTATbits.RXBnOVFL = 0;
//            }

            len = canDataLength(ptr->b[dlc]);
            if ( msgFound = ( !(ptr->b[dlc] & 0x40 ) && len ) )   // Check not RTR and not zero payload
            {
                memcpy(msg->b, (void*) ptr, len + 6);
                msg->b[dlc] = len;
            }

            // Record and Clear any previous invalid message bit flag.
            if (PIR3bits.IRXIF) {
              PIR3bits.IRXIF = 0;
            }
            // Mark that this buffer is read and empty.
            *ptr->b &= 0x7f;    // !!kchk

            led1timer = 2;
            LED1 = LED_ON;
        }   // while !msgFound
    }

    PIE3bits.FIFOWMIE = 1;
    return msgFound;
}

// Called from isr when high water mark interrupt received
// Clears ECAN fifo into software FIFO(s)

void canbusFifo(void)
{
  CANMsg *ptr;
  BYTE  waiting;
  BYTE  len;
  BYTE  *entry;


  while (COMSTATbits.FIFOEMPTY)
  {

    ptr = (CANMsg*) _PointBuffer(CANCON & 0x07);
    PIR3bits.RXBnIF = 0;
    if (COMSTATbits.RXBnOVFL) {
      maxcanq++; // Buffer Overflow
      led3timer = 5;
      LED3 = LED_OFF;
      COMSTATbits.RXBnOVFL = 0;
    }
    
    len = canDataLength(ptr->b[dlc]);
    if (!(ptr->b[dlc] & 0x40 ) && len)       // Check not RTR and not zero payload
    {
        if (FifoIdxW >= SW_FIFO)  
          entry = Fifo1[FifoIdxW - SW_FIFO].msg;
        else
          entry = Fifo0[FifoIdxW].msg;
        memcpy((void*) entry, (void*) ptr, len + 6);
        entry[dlc] = len;
        
        FifoIdxW++;
        if (FifoIdxW >= SW_FIFO_MAX)
        {
          FifoIdxW = 0;
        }
        
        if (FifoIdxW == FifoIdxR)
        {
          maxethq++; // Buffer Overflow
          if (FifoIdxW == 0) {
            FifoIdxW = SW_FIFO_MAX - 1;     // On overflow, received packets overwrite last received packet
          } else {
            FifoIdxW--;
          }

          led3timer = 5;
          LED3 = LED_OFF;
        }        
    }

    // Record and Clear any previous invalid message bit flag.
    if (PIR3bits.IRXIF) {
      PIR3bits.IRXIF = 0;
    }
    // Mark that this buffer is read and empty.
    *ptr->b &= 0x7f;
 
    led1timer = 2;
    LED1 = LED_ON;

    waiting = ( FifoIdxW < FifoIdxR ? FifoIdxW + SW_FIFO_MAX : FifoIdxW) - FifoIdxR;
    if (waiting > maxCANFifo )
        maxCANFifo = waiting;

  }  // While hardware FIFO not empty
}


static BYTE* _PointBuffer(BYTE b) {
  BYTE* pt;

  switch (b) {
    case 0:
      pt = (BYTE*) & RXB0CON;
      break;
    case 1:
      pt = (BYTE*) & RXB1CON;
      break;
    case 2:
      pt = (BYTE*) & B0CON;
      break;
    case 3:
      pt = (BYTE*) & B1CON;
      break;
    case 4:
      pt = (BYTE*) & B2CON;
      break;
    case 5:
      pt = (BYTE*) & B3CON;
      break;
    case 6:
      pt = (BYTE*) & B4CON;
      break;
    default:
      pt = (BYTE*) & B5CON;
      break;
  }
  return (pt);
}
