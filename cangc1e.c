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
 
 
 Version 2e	- 3/2/13
 
 Following changes (c) Pete Brownlow 2013:

 		- Use latest CBUS definitions
		- Define CBUS parameter block as per current spec
		- User vectors defined at 0x800 upwards
		- Add debounce counter for pushbutton
		- Add parse CBUS commands that arrive via CAN
		- Add parameter to calls to CBusEthBroadcast (see below)		
 
 */

#include <p18cxxx.h>
#include <stdio.h>
#include <TCPIP Stack/Tick.h>

#include "project.h"
#include "cbusdefs8r.h"
#include "rocrail.h"


#include "canbus.h"


#include "utils.h"
#include "commands.h"
#include "cangc1e.h"
#include "isr.h"
#include "io.h"
#include "eth.h"
#include "gcaeth.h"

#ifdef RES16M
    #pragma config OSC = HS
#else
    #pragma config OSC = HSPLL
#endif

#pragma config FCMEN=OFF, IESO=OFF
#pragma config PWRT=ON, BOREN=SBORENCTRL, BORV=2, WDT = OFF, WDTPS=1024 // Brown out enable under s/w control, power up timer on, Watchdog under software control, watchdog scaler 1024 gives 4 seconds
#pragma config MCLRE=ON, LPT1OSC=OFF, PBADEN=OFF, DEBUG=OFF
#pragma config XINST=OFF, BBSIZ=1024, LVP=OFF, STVREN=ON
#pragma config CP0=OFF, CP1=OFF, CPB=OFF, CPD=OFF
#pragma config WRT0=OFF, WRT1=OFF, WRTB=OFF, WRTC=OFF, WRTD=OFF
#pragma config EBTR0=OFF, EBTR1=OFF, EBTRB=OFF




unsigned short NN_temp;
unsigned char NV1;
unsigned char CANID;
unsigned char Wait4NN;
unsigned char led1timer;
unsigned char led2timer;
unsigned char led3timer;
unsigned char IdleTime;
unsigned char cmdticker;
unsigned char isLearning;
unsigned char maxcanq;
unsigned char maxethq;
unsigned char maxtxerr;
unsigned char maxrxerr;

BYTE    maxCANFifo;
WORD    aliveCounter;

volatile unsigned short tmr1_reload;


/*
  Para 1 Manufacturer number as allocated by the NMRA
  Para 2 Module version number or code
  Para 3 Module identifier
  Para 4 Number of events allowed
  Para 5 Number of event variables per event
  Para 6 Number of node variables
  Para 7 Not yet allocated.
 */

#pragma romdata PARAMETERS

#define PRM_CKSUM MANU_ID+MINOR_VER+MODULE_ID+EVT_NUM+EVperEVT+NV_NUM+MAJOR_VER+MODULE_FLAGS+CPU+BUS_TYPE+(LOAD_ADDRESS>>8)+(LOAD_ADDRESS&0xFF)+CPUM_MICROCHIP+BETA+NUM_PARAMS+(MNAME_ADDRESS>>8)+(MNAME_ADDRESS&0xFF)

const rom BYTE params[NUM_PARAMS] = {MANU_ID, MINOR_VER, MODULE_ID, EVT_NUM, EVperEVT, NV_NUM, MAJOR_VER,MODULE_FLAGS, CPU,BUS_TYPE,LOAD_ADDRESS&0xFF,LOAD_ADDRESS>>8,0,0,0,0,0,0,CPUM_MICROCHIP,BETA };
const rom BYTE SpareParams[SPARE_PARAMS];
const rom WORD FCUParams[FCU_PARAMS] = {NUM_PARAMS,MNAME_ADDRESS,0, (WORD) PRM_CKSUM};
const rom char module_type[] = MODULE_TYPE;

#pragma romdata

void initIO(void);

/*
 * Interrupt vectors
 */

#pragma code high_vector=0x808

void HIGH_INT_VECT(void) {
  _asm GOTO isr_high _endasm
}

#pragma code low_vector=0x818

void LOW_INT_VECT(void) {
  _asm GOTO isr_low _endasm
}

#pragma code APP

void main(void) {
  unsigned char swTrig = 0;
  BYTE  debounce_count = 0;
  BYTE  i;
  BOOL  FLiMPressed;    // Set true when powered on with FLiM button down
  BOOL  resetDefaults;  // Set true when powered on with button down and JP2 in
  BOOL  toggleDHCP;     // Set true when powered on with button down and JP2 out

  aliveCounter = 0;
  Wait4NN = FALSE;
  isLearning = FALSE;
  maxcanq = 0;
  maxethq = 0;
  cmdticker = 0;
  maxCANFifo = 0;
  IdleTime = 120;
  doEthTick = FALSE;

  lDelay();
  ClrWdt();
  
  NV1 = eeRead(EE_NV);
  if (NV1 == 0xFF) {
    eeWrite(EE_NV, 0);  // Default for NV1 is all flags off
    NV1 = 0;
    IdleTime = 120; // 120 * 500ms = 60 sec.
    eeWrite(EE_IDLETIME, IdleTime);
  }
  
  setupIO();
  
  if (FLiMPressed = checkFlimSwitch())
  {
      lDelay(); // debounce
      FLiMPressed = checkFlimSwitch();
      ClrWdt();
              
  }
  
  resetDefaults = FLiMPressed && !JP2;
  
          
  setMacAddress( resetDefaults );  

  initIO();

  initEth( resetDefaults, toggleDHCP );

  while (checkFlimSwitch())
      ClrWdt();    // Wait for button release before proceeding
  
  NN_temp = eeRead(EE_NN+1) * 256;
  NN_temp += eeRead(EE_NN);
  if (NN_temp == 0 || NN_temp == 0xFFFF || NN_temp == DEFAULT_NN) {
    NN_temp = DEFAULT_NN;
    eeWriteShort(EE_NN,NN_temp);
    SLIM_LEDG = 1;
    FLIM_LEDY = 0;
  }
  else
  {    
    FLIM_LEDY = 1;
    SLIM_LEDG = 0;
  }  


  CANID = eeRead(EE_CANID);
  if (CANID == 0 || CANID == 0xFF)
    CANID = NN_temp & 0x7F;

#ifdef WITHCAN
  cbusSetup();
#endif  

  LED3 = LED_ON; /* signal running system */



  // Loop forever (nothing lasts forever...)
  while (1) {
    CANMsg cmsg;
    BYTE rsend;

    rsend = FALSE;
    // Check for Rx
#ifdef WITHCAN
    while (canbusRecv(&cmsg)) 
    {
      ClrWdt();
      if (parseCmdEth(&cmsg, 0)) {
        if (CBusEthBroadcast(&cmsg, 0xFE)) {
          rsend = TRUE;
          break;
        }
      }
    }
#endif    

    if (doEthTick) {
      doEthTick = FALSE;
      CBusEthTick();
    }

    doEth();

    if (rsend) {
      if (!CBusEthBroadcast(&cmsg, 0xFF)) {
        LED3 = LED_ON;
      }
    }

    if (TXERRCNT > maxtxerr) {
      maxtxerr = TXERRCNT;
    }
    if (RXERRCNT > maxtxerr) {
      maxtxerr = TXERRCNT;
    }

    if (checkFlimSwitch()) {
        if (debounce_count < SW_DEBOUNCE_COUNT)
          debounce_count++;
    }
    else {
      if (debounce_count == SW_DEBOUNCE_COUNT) {
      
          if (Wait4NN) {
            Wait4NN = 0;
          } else {
            CANMsg canmsg;
            canmsg.b[sidh] = (CANID >> 3);
            canmsg.b[sidl] = (CANID << 5);

            if (JP2 == 0) // If JP2 is in, use correct CBUS spec, otherwise as per Rocrail - PNB 3/1/13
                canmsg.b[d0] = OPC_RQNN;
            else
                canmsg.b[d0] = OPC_NNACK;

            canmsg.b[d1] = NN_temp / 256;
            canmsg.b[d2] = NN_temp % 256;
            canmsg.b[dlc] = 3;
            CBusEthBroadcast(&canmsg, 0xFF);
            Wait4NN = 1;
          }
      }
      debounce_count = 0;
    }
  }
}

void initIO(void) {
  int idx = 0;


  INTCON = 0;
  EECON1 = 0;

  IPR3 = 0; // All IRQs low priority for now
  IPR2 = 0;
  IPR1 = 0;
  PIE3 = 0;
  PIE2 = 0;
  PIE1 = 0;
  INTCON3 = 0;
  INTCON2 = 0; // Port B pullups are enabled
  PIR3 = 0;
  PIR2 = 0;
  PIR1 = 0;
  RCONbits.IPEN = 1; // enable interrupt priority levels

  T1CON = 0b00110000;
  IPR1bits.TMR1IP = 1; // high_priority
  PIE1bits.TMR1IE = 1; // enable int

  tmr1_reload = TMR1_NORMAL;
  TMR1H = tmr1_reload / 256;
  TMR1L = tmr1_reload % 256;

  T1CONbits.TMR1ON = 1; // start timer

  // Start slot timeout timer
  led500ms_timer = 2; // 500ms

  // Set up global interrupts
  RCONbits.IPEN = 1; // Enable priority levels on interrupts
  INTCONbits.GIEL = 1; // Low priority interrupts allowed
  INTCONbits.GIEH = 1; // Interrupting enabled.
}
