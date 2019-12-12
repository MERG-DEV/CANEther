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


#ifndef __HARDWARE_PROFILE_H
#define __HARDWARE_PROFILE_H

#include <p18f2585.h>

#define GetSystemClock()               (32000000ul)
#define GetInstructionClock()          (GetSystemClock()/4)
#define GetPeripheralClock()           (GetSystemClock()/4)    // Should be GetSystemClock()/4 for PIC18

// I/O pins for Ethernet chip - either ENC28J60 or ENC424J600

#ifdef ENC28J60

    // ENC28J60 I/O pins
    #define ENC_RST_TRIS            (TRISAbits.RA4)
    #define ENC_RST_IO              (LATAbits.LATA4)
    #define ENC_CS_TRIS             (TRISAbits.RA5)
    #define ENC_CS_IO               (LATAbits.LATA5)

    #define ENC_SPI_IF              (PIR1bits.SSPIF)
    #define ENC_SCK_TRIS            (TRISCbits.RC3)
    #define ENC_SDI_TRIS            (TRISCbits.RC4)
    #define ENC_SDO_TRIS            (TRISCbits.RC5)

    #define ENC_SSPBUF              (SSPBUF)
    #define ENC_SPISTAT             (SSPSTAT)
    #define ENC_SPISTATbits         (SSPSTATbits)
    #define ENC_SPICON1             (SSPCON1)
    #define ENC_SPICON1bits         (SSPCON1bits)
    #define ENC_SPICON2             (SSPCON2)

#elif defined ENC424J600
    // ENC424J600 definitions
    #define ENC100_INTERFACE_MODE   0   // SPI mode
    #define ENC100_CS_TRIS          (TRISAbits.RA5)
    #define ENC100_CS_IO            (LATAbits.LATA5)

    #define ENC100_SPI_IF              (PIR1bits.SSPIF)
    #define ENC100_SCK_AL_TRIS         (TRISCbits.RC3)
    #define ENC100_SO_WR_B0SEL_EN_TRIS (TRISCbits.RC4)
    #define ENC100_SI_RD_RW_TRIS       (TRISCbits.RC5)

    #define ENC100_SSPBUF              (SSPBUF)
    #define ENC100_SPISTAT             (SSPSTAT)
    #define ENC100_SPISTATbits         (SSPSTATbits)
    #define ENC100_SPICON1             (SSPCON1)
    #define ENC100_SPICON1bits         (SSPCON1bits)
    #define ENC100_SPICON2             (SSPCON2)
#else
    Error - need to define either ENC28J60 or ENC424J600 for Ethernet chip
#endif





// MAC Chip additional I/O pins

#define MAC_CS_TRIS             (TRISCbits.RC7)
#define MAC_CS_IO               (LATCbits.LATC7)

// MAC Chip insructions

#define MAC_READ                0x03        // Send read command followed by address then clock the data into the PIC
#define MAC_WRITE               0x02        // send write command follwed by address followed by data (WREN must be done first) then CS high to write
#define MAC_WRDI                0x04        // Reset write enable latch (disable write operations)
#define MAC_WREN                0x06        // Set write enable latch (enable write operations)
#define MAC_RDSR                0x05        // Read status register
#define MSC_WRSR                0x01        // Read status register

// MAC address in MAC chip

#define MAC_EUI48_ADDR          0xFA



// Macros for SPI operation - used for MAC chip when ENC28J60 fitted
// When ENC424J600 fitted, SPI only used for that, and macros defined in ENCX24J600.c

#ifdef ENC28J60

#if defined (__18CXX)
    #define ClearSPIDoneFlag()  {ENC_SPI_IF = 0;}
    #define WaitForDataByte()   {while(!ENC_SPI_IF); ENC_SPI_IF = 0;}
    #define SPI_ON_BIT          (ENC_SPICON1bits.SSPEN)
#elif defined(__C30__)
    #define ClearSPIDoneFlag()
    static inline __attribute__((__always_inline__)) void WaitForDataByte( void )
    {
        while ((ENC_SPISTATbits.SPITBF == 1) || (ENC_SPISTATbits.SPIRBF == 0));
    }

    #define SPI_ON_BIT          (ENC_SPISTATbits.SPIEN)
#elif defined( __PIC32MX__ )
    #define ClearSPIDoneFlag()
    static inline __attribute__((__always_inline__)) void WaitForDataByte( void )
    {
        while (!ENC_SPISTATbits.SPITBE || !ENC_SPISTATbits.SPIRBF);
    }

    #define SPI_ON_BIT          (ENC_SPICON1bits.ON)
#else
    #error Determine SPI flag mechanism
#endif

#endif

#endif	// __HARDWARE_PROFILE_H
