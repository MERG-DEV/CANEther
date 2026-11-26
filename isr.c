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


#include <TCPIP Stack/TCPIP.h>
#include <GenericTypeDefs.h>
#include "project.h"
#include "isr.h"
#include "cangc1e.h"
#include "io.h"
#include "canbus.h"



unsigned short led500ms_timer;
unsigned short io_timer;
unsigned short led_timer;

volatile BOOL doEthTick;

//
// Interrupt Service Routine
//
// TMR1 generates a heartbeat every 50ms.
//

#pragma interrupt isr_high

void isr_high(void) {

  if (PIR1bits.TMR1IF) {
    PIR1bits.TMR1IF = 0;
    TMR1H = tmr1_reload / 256;
    TMR1L = tmr1_reload % 256;
  }

  //
  // LED timer  - 50ms
  //
  doLEDTimers();

  //
  // Timer 500ms
  //
  if (--led500ms_timer == 0) {
    led500ms_timer = 10;
    aliveCounter++;
    doLEDs();
    doEthTick = TRUE;
#ifdef WITHCAN
    checkCANTimeout();
#endif    
  }
}


//
// Low priority interrupt. Used for TCPIP TickUpdate, ECAN FIFO and CAN transmit interrupts
//
// Each source is handled only if its interrupt is enabled: the main loop
// disables an interrupt to keep its handler out while it works on the same
// data, and another source may still bring us here meanwhile. Each handler
// clears its own flag, so a flag raised while this runs is kept and brings us
// straight back.
//

#pragma interruptlow isr_low

void isr_low(void) {

  TickUpdate();

#ifdef WITHCAN
  // Check for receive high water mark interrupt
  if (PIE3bits.FIFOWMIE && PIR3bits.FIFOWMIF) {
    PIE3bits.FIFOWMIE = 0;
    PIR3bits.FIFOWMIF = 0;
    canbusFifo();
    PIE3bits.FIFOWMIE = 1;
  }

  // Check for transmit buffer interrupt
  if (PIE3bits.TXBnIE && PIR3bits.TXBnIF) {
    PIR3bits.TXBnIF = 0;
    canTxDone();
  }

  // Check for transmit error interrupt
  if (PIE3bits.ERRIE && PIR3bits.ERRIF)
  {
      PIR3bits.ERRIF = 0;
      canTxError();
  }

#endif
}
