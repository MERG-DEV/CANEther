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


#ifndef __PROJECT_H
#define __PROJECT_H


#include <p18cxxx.h>
#include <p18f2585.h>
#include <GenericTypeDefs.h>

#define CPU   P18F2585
    
#define FIXED_CAN_ID 0x7F // use only for enummeration

#define TRUE 1
#define FALSE 0
#define PORT_ON 1
#define PORT_OFF 0

#ifdef CANEtherT    // CANEtherT drives LEDs the other way up
    #define LED_ON 1
    #define LED_OFF 0
#else
    #define LED_ON 0
    #define LED_OFF 1
#endif

typedef unsigned short ushort;
typedef unsigned char byte;

extern const rom BYTE params[];
extern const rom char module_type[];

#define TMR1_NORMAL	15535

#endif	// __PROJECT_H
