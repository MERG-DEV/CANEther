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


#include <p18f2585.h>
#include "utils.h"

#pragma code APP

/*
 * ee_read - read from data EEPROM
 */
unsigned char eeRead(unsigned char addr) {
  EEADR = addr; /* Data Memory Address to read */
  EECON1bits.EEPGD = 0; /* Point to DATA memory */
  EECON1bits.CFGS = 0; /* Access program FLASH or Data EEPROM memory */
  EECON1bits.RD = 1; /* EEPROM Read */
  return EEDATA;
}

/*
 * ee_write - write to data EEPROM
 */
void eeWrite(unsigned char addr, unsigned char data) {

  
  INTCONbits.GIE = 0; // Disable interupts

  EECON1bits.EEPGD = 0; // Select the EEPROM memory
  EECON1bits.CFGS = 0; // Access the EEPROM memory
  EECON1bits.WREN = 1; // Enable writing
  EEADR = addr; // Set the address
  EEDATA = data; // Set the data
  EECON2 = 0x55; // Write initiate sequence
  EECON2 = 0xaa; // Write initiate sequence
  WDTCON =0;            // Disable watchdog reset during EEPROM write
  RCONbits.SBOREN = 0;  // Disable brown out reset during EEPROM write
  EECON1bits.WR = 1; // Start writing
  
  while (!PIR2bits.EEIF)
    ; // Wait for write to finish
  PIR2bits.EEIF = 0; // Clear EEIF bit
  
  EECON1bits.WREN = 0;  // Disable writing
  
  RCONbits.SBOREN = 1;   // Re-enable brown out reset
  WDTCON = 1;          // Re-enable watchdog
  INTCONbits.GIE = 1; // Enable interupts
}

/*
 * ee_read_short() - read a short (16 bit) word from EEPROM
 *
 * Data is stored in little endian format
 */
unsigned short eeReadShort(unsigned char addr) {
  unsigned char byte_addr = addr;
  unsigned short ret = eeRead(byte_addr++);
  ret = ret | ((unsigned short) eeRead(byte_addr) << 8);
  return ret;
}

/*
 * ee_write_short() - write a short (16 bit) data to EEPROM
 *
 * Data is stored in little endian format
 */
void eeWriteShort(unsigned char addr, unsigned short data) {
  unsigned char byte_addr = addr;
  eeWrite(byte_addr++, (unsigned char) data);
  eeWrite(byte_addr, (unsigned char) (data >> 8));
}

/*
 * A delay routine
 */
void delay(void) {
  unsigned char i, j;
  for (i = 0; i < 10; i++) { // dely     movlw    .10
    //          movwf    Count1
    for (j = 0; j <= 245; j++) // dely2    clrf    Count
      ; // dely1    decfsz    Count,F
    //          goto    dely1
    //          decfsz    Count1
  } //          bra        dely2
} //          return

/*
 * Long delay
 */
void lDelay(void) {
  unsigned char i;
  for (i = 0; i < 100; i++) { //ldely	movlw	.100
    //		movwf	Count2
    delay(); //ldely1	call	dely
    //		decfsz	Count2
  } //		bra		ldely1
  //
} //		return

unsigned char getDataLen(unsigned char OPC, unsigned char eth) {
  if (!eth) {
    if (OPC < 0x20) return 0;
    if (OPC < 0x40) return 1;
    if (OPC < 0x60) return 2;
    if (OPC < 0x80) return 3;
    if (OPC < 0xA0) return 4;
    if (OPC < 0xC0) return 5;
    if (OPC < 0xE0) return 6;
    return 7;
  } else {
    if (OPC == 1) return 4;
    if (OPC == 2) return 3;
    if (OPC == 3) return 6;
  }
  return 0;
}
