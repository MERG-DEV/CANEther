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

#ifndef GCAETH_H
#define GCAETH_H

#define DEFAULT_CBUSETH_PORT            (5550)
#define MAX_CBUSETH_CMD_LEN     (32)

#define RC_UNKNOWN_ASCII_FRAME 1

#include "canbus.h"

void CBusEthTick(void);
void CBusEthInit(void);
void openBridge( void );
BOOL isBridgeConnected( void );
void CBusEthServer(void);
unsigned char CBusEthBroadcast(CANMsg* msg, BYTE excludeConn);

extern BYTE nrClients;

#endif
