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


#ifndef __CANBUS_H
#define __CANBUS_H


typedef struct {
  unsigned char b[14];
  unsigned char status;
} CANMsg;

void cbusSetup(void);

typedef union _Byte_Val
{
    struct
    {
        unsigned int b0:1;
        unsigned int b1:1;
        unsigned int b2:1;
        unsigned int b3:1;
        unsigned int b4:1;
        unsigned int b5:1;
        unsigned int b6:1;
        unsigned int b7:1;
    } bits;
    BYTE val;
} Byte_Val;

enum bufbytes {
        con=0,
        sidh,
        sidl,
        eidh,
        eidl,
        dlc,
        d0,
        d1,
        d2,
        d3,
        d4,
        d5,
        d6,
        d7
};

#define ECAN_MSG_STD    0
#define ECAN_MSG_XTD    1

extern BYTE  maxCanTxFifo;

BOOL canbusRecv(CANMsg *msg);
BOOL canbusSend( CANMsg *msg );
void canbusFifo(void);
void canTxFifo( void );
void checkCANTimeout( void );
void canTxError( void );

#endif

