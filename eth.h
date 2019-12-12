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


#ifndef __ETH_H
#define __ETH_H

#define THIS_IS_STACK_APPLICATION
/*
 * These headers must be included for required defs.
 */
#include <TCPIP Stack/TCPIP.h>


void setMacAddress( void );
BOOL readUnioMac(void);
BOOL readSpiMac(void);

void InitAppConfig(void);
void initEth(void);
void doEth(void);


#endif
