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


#ifndef __ISR_H

#define LONG_PRE	22	// Increased from 20 - PNB 16/4/11 to meet NMRA spec 
#define NORM_PRE	14

// ISR prototype 
extern void isr_high(void);
extern void isr_low(void);

// extern unsigned short an0;
// extern unsigned short retry_delay;
// extern unsigned short sum;
// extern unsigned short ave;

extern unsigned short led500ms_timer;
extern unsigned short io_timer;
extern unsigned short led_timer;

extern volatile BOOL doEthTick;


#define __ISR_H
#endif
