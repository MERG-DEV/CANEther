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

#define LARB_RETRIES 10;

typedef struct {
  BYTE msg[16];
} CAN_PACKET;

#pragma udata VARS_FIFO_0

far CAN_PACKET Fifo0[SW_FIFO];

#pragma udata VARS_FIFO_1

far CAN_PACKET Fifo1[SW_FIFO];

#pragma udata VARS_TXFIFO

far CAN_PACKET TxFifo[SW_FIFO];

#pragma udata

far BYTE FifoIdxW = 0;
far BYTE FifoIdxR = 0;
far BYTE FifoTxIdxW = 0;
far BYTE FifoTxIdxR = 0;

BYTE  larbRetryCount;
BYTE  canTransmitTimeout;
BOOL  canTransmitFailed;
BYTE  larbCount;
BYTE  txErrCount;
BYTE  txTimeoutCount;
BYTE  maxCanTxFifo;



static BYTE* _PointBuffer(BYTE b);

void cbusSetup(void) {

  larbCount = 0;
  txErrCount = 0;
  txTimeoutCount = 0;
  canTransmitFailed = FALSE;
  canTransmitTimeout = 0;
  maxCanTxFifo = 0;
  
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
  PIE3bits.ERRIE = 1;
}

#define buffers 3

BOOL canSend(CANMsg *msg) {

  BYTE* ptr;
  BOOL  fullUp;
  BYTE hiIndex;

  msg->b[con] = 0;
  msg->b[dlc] &= 0x0F;  // Ensure not RTR 
  
  if (msg->b[dlc] > 8)
      msg->b[dlc] = 8;

  PIE3bits.TXBnIE = 0;  // Disable transmit buffer interrupt whilst we fiddle with registers and fifo

  // On chip Transmit buffers do not work as a FIFO, so use just one buffer and implement a software fifo

  if (((FifoTxIdxR == FifoTxIdxW) || canTransmitFailed) && (!TXB0CONbits.TXREQ))  // check if software fifo empty and transmit buffer ready
  {
     ptr = (BYTE*) & TXB0CON;
     memcpy(ptr, (void *) msg->b, msg->b[dlc] + 6);

     larbRetryCount = LARB_RETRIES;
     canTransmitTimeout = 2;	// half second intervals
     canTransmitFailed = FALSE;

     TXB0CONbits.TXREQ = 1;    // Initiate transmission
     fullUp = FALSE;
     PIE3bits.TXBnIE = 1;
  }
  else  // load it into software fifo
  {
      if (!(fullUp = (FifoTxIdxW == 0xFF)))
      {
        memcpy( &TxFifo[FifoTxIdxW].msg, msg->b, msg->b[dlc] + 6);
  
        if (++FifoTxIdxW == SW_FIFO )
            FifoTxIdxW = 0;

        if (FifoTxIdxR == FifoTxIdxW) // check if fifo now full
            FifoTxIdxW = 0xFF;
      }

      hiIndex = ( FifoTxIdxW < FifoTxIdxR ? FifoTxIdxW + SW_FIFO : FifoTxIdxW);
      if ((hiIndex - FifoIdxR) > maxCanTxFifo )
        maxCanTxFifo = hiIndex - FifoTxIdxR;
  }

  PIE3bits.TXBnIE = 1;  // Enable transmit buffer interrupt
 
  return !fullUp;
}



// Called by ISR to handle tx buffer interrupt

void canTxFifo( void )
{
    BYTE* ptr;

    canTransmitFailed = FALSE;   
    
    if (!TXB0CONbits.TXREQ)
    {
        canTransmitTimeout = 0;
        
        if (FifoTxIdxR != FifoTxIdxW)   // If data waiting in software fifo, and buffer ready
        {
            ptr = (BYTE*) & TXB0CON;
            memcpy(ptr, &TxFifo[FifoTxIdxR].msg, TxFifo[FifoTxIdxR].msg[dlc] + 6);

            larbRetryCount = LARB_RETRIES;
            canTransmitTimeout = 2;	// half second intervals

            TXB0CONbits.TXREQ = 1;    // Initiate transmission

            if (FifoTxIdxW == 0xFF)
                FifoTxIdxW = FifoTxIdxR; // clear full status

            if (++FifoTxIdxR == SW_FIFO )
                FifoTxIdxR = 0;

            PIE3bits.TXBnIE = 1;  // enable transmit buffer interrupt
        }
        else
        {
            TXB0CON = 0;
            PIE3bits.TXBnIE = 0;
        }
    }
    else
        PIE3bits.TXBnIE = 1;
 

}

// Called by high priority ISR at 0.5 sec intervals to check for timeout

void checkCANTimeout( void )
{
    if (canTransmitTimeout != 0)
        if (--canTransmitTimeout ==0)
        {    
            canTransmitFailed = TRUE;
            txTimeoutCount++;
            TXB0CONbits.TXREQ = 0;  // abort timed out packet
        }    
}


//BOOL canSend(CANMsg *msg) {
//
//  BYTE i;
//  BYTE* ptr;
//  BYTE* tempPtr;
//  BYTE * pb[buffers];
//
//  pb[0] = (BYTE*) & TXB2CON;
//  pb[1] = (BYTE*) & TXB1CON;
//  pb[2] = (BYTE*) & TXB0CON;
//
//  // Transmit buffers do not work as a FIFO,
//  for (i = 0; i < buffers; i++) {
//    ptr = pb[i];
//    tempPtr = ptr;
//    /*
//     * Check to see if this buffer is empty
//     */
//    if (!(*ptr & 0x08)) {
//      msg->b[con] = 0;
//      memcpy(ptr, (void *) msg->b, msg->b[dlc] + 6);
//      if (!(*tempPtr & 0x04)) {
//        *tempPtr |= 0x08;
//      }
//      led1timer = 2;
//      LED1 = LED_ON;
//      return TRUE;
//    }
//  }
//  return FALSE;
//}


BOOL canbusSend(CANMsg *msg) {
 
  while (!canSend(msg)) {
    led3timer = 1;
    LED3 = LED_OFF;
    if (COMSTATbits.TXWARN) {
      led3timer = 20;
      LED3 = LED_OFF;
      return FALSE;
    }
  }
  return TRUE;
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

            if ( msgFound = ( !(ptr->b[dlc] & 0x40 ) && (ptr->b[dlc] & 0x0F) ) )   // Check not RTR and not zero payload
            {
                memcpy(msg->b, (void*) ptr, ptr->b[dlc] + 6);
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
  BYTE  hiIndex;


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
    
    if (!(ptr->b[dlc] & 0x40 ) && (ptr->b[dlc] & 0x0F))       // Check not RTR and not zero payload               
    {
        if (FifoIdxW >= SW_FIFO)  
          memcpy((void*) &Fifo1[FifoIdxW - SW_FIFO].msg, (void*) ptr, ptr->b[dlc] + 6);
        else
          memcpy((void*) &Fifo0[FifoIdxW].msg, (void*) ptr, ptr->b[dlc] + 6);
        
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

    hiIndex = ( FifoIdxW < FifoIdxR ? FifoIdxW + SW_FIFO : FifoIdxW);
    if ((hiIndex - FifoIdxR) > maxCANFifo )
        maxCANFifo = hiIndex - FifoIdxR;

  }  // While hardware FIFO not empty
}


// Process transmit error interrupt

void canTxError( void )
{
    if (TXB0CONbits.TXLARB) {  // lost arbitration
        if (larbRetryCount == 0) {	// already tried higher priority
            canTransmitFailed = TRUE;
            canTransmitTimeout = 0;

            TXB0CONbits.TXREQ = 0;
            larbCount++;
        }
        else if ( --larbRetryCount == 0) {	// Allow tries at lower level priority first
            TXB0CONbits.TXREQ = 0;
            TXB0SIDH &= 0b00111111; 		// change to high priority
            TXB0CONbits.TXREQ = 1;			// try again
        }
    }
    if (TXB0CONbits.TXERR) {	// bus error
      canTransmitFailed = TRUE;
      canTransmitTimeout = 0;
      TXB1CONbits.TXREQ = 0;
      txErrCount++;
    }
    
    if (canTransmitFailed)
        canTxFifo();
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
