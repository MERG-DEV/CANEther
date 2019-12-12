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

// Make DHCP setting toggle by powering up with pushbutton held down
// and make the setting stick by saving NV1 to EEPROM - PNB 1/1/13


#include <TCPIP Stack/TCPIP.h>
#include "cangc1e.h"
#include "eth.h"
#include "project.h"
#include "io.h"
#include "gcaeth.h"
#include "utils.h"


// Read in MAC address from MAC chip if fitted

void setMacAddress( BOOL resetDefaults )

// If unitialised, or powered on with JP2 in and button pressed, then reset MAC and IP to defaults

{   if ( resetDefaults || (eeRead(EE_MACADDR) == 0xFF))  // Flag for virgin state, or powered up with button held in and JP2 in
    {                                                        // (First byte of valid MAC address cannot be 0xFF)
    
        // Call routines to try and read MAC address from one of two types of chip, if fitted.
        // Note that the ReadUnio routine uses a timer and must not be interrupted, so this routine is called
        // before timers are set up or Interrupts enabled.

        #ifdef ENC28J60
        if (!readUnioMac())             // Read MAC address from UNI/O chip
            if (!readSpiMac())          // Read MAC address from SPI chip
        #endif
            {                           // No MAC chip, so just set default
                eeWrite(EE_MACADDR + 0, MY_DEFAULT_MAC_BYTE1);
                eeWrite(EE_MACADDR + 1, MY_DEFAULT_MAC_BYTE2);
                eeWrite(EE_MACADDR + 2, MY_DEFAULT_MAC_BYTE3);
                eeWrite(EE_MACADDR + 3, MY_DEFAULT_MAC_BYTE4);
                eeWrite(EE_MACADDR + 4, MY_DEFAULT_MAC_BYTE5);
                eeWrite(EE_MACADDR + 5, MY_DEFAULT_MAC_BYTE6);
            }
    }        
}

#ifdef ENC28J60
// Read in MAC address from UNI/O MAC address chip.
// If fails (chip not fitted), return false

BOOL readUnioMac(void)

{
    return( FALSE );
}

// Read in MAC address from SPI MAC address chip.
// If fails (chip not fitted), return false


BOOL readSpiMac(void)

{
    BOOL    gotMAC;
    BYTE    readVal;
    BYTE    i;
    BYTE    macAddress[6];

    // Set up the SPI module on the PIC for communications with the MAC chip

    ENC_CS_IO = 1;          // Make sure ENC chip is not selected whilst we read MAC chip
    ENC_CS_TRIS = 0;        // Make the Chip Select pin an output
    MAC_CS_IO = 1;          // MAC chip deselected to start with
    MAC_CS_TRIS = 0;        // Make the Chip Select pin an output

    ENC_SCK_TRIS = 0;       // SPI Clock is an output
    ENC_SDO_TRIS = 0;       // SPI Data out is an output
    ENC_SDI_TRIS = 1;       // SPI Data in is an input

    // Set up SPI
    ENC_SPI_IF = 0;         // Clear SPI done flag

    ENC_SPICON1 = 0x21;     // SSPEN bit is set, SPI in master mode, FOSC/16
                            //   IDLE state is low level
    ENC_SPISTATbits.CKE = 1;// Transmit data on rising edge of clock
    ENC_SPISTATbits.SMP = 0;// Input sampled at middle of data output time

    // Read status register to see if MAC chip fitted

    MAC_CS_IO = 0;
 
    ENC_SSPBUF = MAC_RDSR;          // Send the opcode
    WaitForDataByte();              // Wait until opcode is transmitted 
    readVal = ENC_SSPBUF;           // Dummy read
    ENC_SSPBUF = 0;
    WaitForDataByte();              // Wait until data received back
    readVal = ENC_SSPBUF;

    MAC_CS_IO = 1;

    // If expected value in status register, go ahead and read MAC address

    if (gotMAC = (readVal == 0x04 )) // Check expected status register
    {
        MAC_CS_IO = 0;
        ENC_SSPBUF = MAC_READ;          // Transmit the opcode
        WaitForDataByte();              // Wait until opcode is transmitted 
        readVal = ENC_SSPBUF;           // Dummy read
        ENC_SSPBUF = MAC_EUI48_ADDR;    // Transmit the address of the MAC in the chip
        WaitForDataByte();              // Wait until opcode is transmitted and data received back
        readVal = ENC_SSPBUF;           // Dummy read

        
        for (i=0; i<6; i++)
        {
            ENC_SSPBUF = 0;             // Get next data byte
            WaitForDataByte();     
            macAddress[i] = ENC_SSPBUF; // Note - cannot write to EEPROM as we go, as write is 4mS and we'd miss subsequent SPI bytes arriving
        }    

        for (i=0; i<6; i++)
        {
            eeWrite(EE_MACADDR + i, macAddress[i]);
        }

        MAC_CS_IO = 1;          // Leave MAC chip deselected so does not affect ENC operation
    }


    return (gotMAC);
}

#endif

/*
 * This is used by other stack elements.
 * Main application must define this and initialize it with
 * proper values.
 */
APP_CONFIG AppConfig;

void initEth(BOOL resetDefaults, BOOL toggleDHCP) {
  /*
   * Initialize all stack related components.
   * Following steps must be performed for all applications using
   * PICmicro TCP/IP Stack.
   */
  TickInit();

  /*
   * Following steps must be performed for all applications using
   * PICmicro TCP/IP Stack.
   */
  /*
   * Initialize Stack and application related NV variables.
   */
  InitAppConfig(resetDefaults);
  
  setDHCPMode( toggleDHCP );

  StackInit();

  CBusEthInit();
}

void InitAppConfig(BOOL resetDefaults)
{

  BYTE slen;
  BYTE i;
  unsigned char defaultHostName[] = MY_DEFAULT_HOST_NAME;


  if (resetDefaults || (eeRead(EE_CLEAN) == 0xFF))
  {
    eeWrite(EE_CLEAN, 0);
    eeWrite(EE_IPADDR + 0, MY_DEFAULT_IP_ADDR_BYTE1);
    eeWrite(EE_IPADDR + 1, MY_DEFAULT_IP_ADDR_BYTE2);
    eeWrite(EE_IPADDR + 2, MY_DEFAULT_IP_ADDR_BYTE3);
    eeWrite(EE_IPADDR + 3, MY_DEFAULT_IP_ADDR_BYTE4);

    eeWrite(EE_NETMASK + 0, MY_DEFAULT_MASK_BYTE1);
    eeWrite(EE_NETMASK + 1, MY_DEFAULT_MASK_BYTE2);
    eeWrite(EE_NETMASK + 2, MY_DEFAULT_MASK_BYTE3);
    eeWrite(EE_NETMASK + 3, MY_DEFAULT_MASK_BYTE4);
  }

  if (resetDefaults || (eeReadShort(EE_PORTNUM) == 0xFFFF))   // Check this separately in case upgrading from firmware that does not have these NVs, when they will be set to FF
  {
    eeWriteShort(EE_PORTNUM, DEFAULT_CBUSETH_PORT );
    eeWriteShort(EE_REMOTEPORT, DEFAULT_CBUSETH_PORT);
    
    //  Set all remote bridge parameters to zero

    for (i=0; i<8; i++)
    {
        eeWrite(EE_REMOTEIP + i, 0);
    }

  }


  /*
   * Load default configuration into RAM.
   */
  AppConfig.MyIPAddr.v[0] = eeRead(EE_IPADDR + 0);
  AppConfig.MyIPAddr.v[1] = eeRead(EE_IPADDR + 1);
  AppConfig.MyIPAddr.v[2] = eeRead(EE_IPADDR + 2);
  AppConfig.MyIPAddr.v[3] = eeRead(EE_IPADDR + 3);

  AppConfig.MyMask.v[0] = eeRead(EE_NETMASK + 0);
  AppConfig.MyMask.v[1] = eeRead(EE_NETMASK + 1);
  AppConfig.MyMask.v[2] = eeRead(EE_NETMASK + 2);
  AppConfig.MyMask.v[3] = eeRead(EE_NETMASK + 3);

  if (eeRead(EE_GATEWAY) != 0)
  {
    AppConfig.MyGateway.v[0] = eeRead(EE_GATEWAY + 0);
    AppConfig.MyGateway.v[1] = eeRead(EE_GATEWAY + 1);
    AppConfig.MyGateway.v[2] = eeRead(EE_GATEWAY + 2);
    AppConfig.MyGateway.v[3] = eeRead(EE_GATEWAY + 3);
  }
  else
  {
    AppConfig.MyGateway.v[0] = AppConfig.MyMask.v[0];
    AppConfig.MyGateway.v[1] = AppConfig.MyMask.v[1];
    AppConfig.MyGateway.v[2] = AppConfig.MyMask.v[2];
    AppConfig.MyGateway.v[3] = AppConfig.MyMask.v[3];
  }

  AppConfig.MyMACAddr.v[0] = eeRead(EE_MACADDR + 0);
  AppConfig.MyMACAddr.v[1] = eeRead(EE_MACADDR + 1);
  AppConfig.MyMACAddr.v[2] = eeRead(EE_MACADDR + 2);
  AppConfig.MyMACAddr.v[3] = eeRead(EE_MACADDR + 3);
  AppConfig.MyMACAddr.v[4] = eeRead(EE_MACADDR + 4);
  AppConfig.MyMACAddr.v[5] = eeRead(EE_MACADDR + 5);

  strcpy( AppConfig.NetBIOSName , defaultHostName );

}

void setDHCPMode( BOOL toggleDHCP )
{
#if defined(STACK_USE_DHCP_CLIENT) || defined(STACK_USE_IP_GLEANING)
  // Make DHCP setting toggle by powering up with pushbutton held down
  // and make the setting stick by saving NV1 to EEPROM - PNB 1/1/13
  if (toggleDHCP) 
  {
    NV1 ^= CFG_DHCP_CLIENT; // Toggle DHCP flag;  
    eeWrite(EE_NV, NV1);
  }
  AppConfig.Flags.bIsDHCPEnabled = ((NV1 && CFG_DHCP_CLIENT) != 0);
#else
  AppConfig.Flags.bIsDHCPEnabled = FALSE;
#endif

}

void doEth(void) {
  /*
   * This task performs normal stack task including checking
   * for incoming packet, type of packet and calling
   * appropriate stack entity to process it.
   */
  StackTask();
  StackApplications();    // Calls announce and NBNS
  CBusEthServer();
}