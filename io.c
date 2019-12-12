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



#include <p18cxxx.h>
#include <stdio.h>

#include "project.h"
#include "canbus.h"
#include "utils.h"
#include "cbusdefs8h.h"
#include "cangc1e.h"
#include "io.h"

void setupIO(void) {
  int idx = 0;

  // Enable watchdog timer
  WDTCON = 1;  
  
  // Enable brown out reset
  RCONbits.SBOREN = 1; // Enable brown out reset
  
  // all digital I/O
  ADCON0 = 0x00;
  ADCON1 = 0x0F;

  // Set all I/O as outputs so any unused pins remain as outputs
  TRISA = 0;
  TRISB = 0;
  TRISC = 0;   
  
  // Set pin usage
  
  LED1_TRIS = 0;    /* LED1 */ 
  LED2_TRIS = 0;    /* LED2 */
  LED3_TRIS = 0;    /* LED3 */
  
  SW_TRIS = 1;       /* SW - FLiM switch */
  JP2_TRIS = 1;      /* JP2 - option jumper */
  
  SLIM_LEDG_TRIS = 0; /* Yellow FLiM LED */
  FLIM_LEDY_TRIS = 0; /* Green SLiM LED */

  // Enable internal pull-ups.
  INTCON2bits.RBPU = 0;

  LED1 = LED_OFF;
  LED2 = LED_ON;
  LED3 = LED_OFF;

  SLIM_LEDG = 1; // Start with both LEDs on for diagnostic
  FLIM_LEDY = 1;


}

void writeOutput(int idx, unsigned char val) {
}

unsigned char readInput(int idx) {
  unsigned char val = 0;
  return !val;
}

// Called every 3ms.

void doLEDTimers(void) {
  if (led1timer > 0) {
    led1timer--;
    if (led1timer == 0) {
      LED1 = LED_OFF;
    }
  }

  if (led2timer > 0) {
    led2timer--;
    if (led2timer == 0) {
      LED2 = LED_OFF;
    }
  }

  if (led3timer > 0) {
    led3timer--;
    if (led3timer == 0) {
      LED3 = LED_ON;
    }
  }

  if (Wait4NN) {
    return;
  }
}

void doIOTimers(void) {
  int i = 0;
}

void doTimedOff(int i) {
}

unsigned char checkFlimSwitch(void) {
  unsigned char val = SW;
  return !val;
}

unsigned char checkInput(unsigned char idx) {
  unsigned char ok = 1;
  return ok;
}

void saveOutputStates(void) {
  int idx = 0;
  byte o1 = 0;
  byte o2 = 0;

  //eeWrite(EE_PORTSTAT + 1, o2);


}



static unsigned char __LED3 = 0;

void doLEDs(void) {
  if (Wait4NN || isLearning) {
    LED3 = __LED3;
    FLIM_LEDY = __LED3;

    __LED3 ^= 1;
    led3timer = 20;
  }
}

void setOutput(ushort nn, ushort addr, byte on) {
}
