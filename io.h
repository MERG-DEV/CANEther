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


#ifndef __IO_H
#define __IO_H


#define LED1    PORTCbits.RC2   // cbus activity
#define LED2    PORTCbits.RC0   // ethernet activity
#define LED3    PORTCbits.RC1   // running

#ifdef CANEtherT
    #define SW      PORTAbits.RA2   // FLiM button on PNB -T variant is on standard of RA2
#else
    #define SW      PORTAbits.RA0   // FLiM button
#endif


#define JP2     PORTCbits.RC6   // option jumper 2

#define SLIM_LEDG   PORTBbits.RB7   // Green SLiM LED (CANEther only)
#define FLIM_LEDY   PORTBbits.RB6   // Yellow FLiM LED (CANEther only)


void setupIO(void);
void writeOutput(int port, unsigned char val);
unsigned char readInput(int port);
void doIOTimers(void);
void doLEDTimers(void);
void doLEDs(void);
void setOutput(ushort nn, ushort addr, byte on);
unsigned char checkFlimSwitch(void);

#endif	// __IO_H
