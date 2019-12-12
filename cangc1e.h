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
 
 For version 2e, 3/2/13, following changes PNB:
 
 		- Add #if directives so can be built for CANGC1e or CANEther.
		- Define manufacturer, module type, name and default NN for CANEther
		- Define additional parameters used in latest CBUS parameter block
		- Define pushbutton software debounce count
 
 
*/



#ifndef __CANGC1E_H
#define __CANGC1E_H

#if CANGC1e
  #define MANU_ID       MANU_ROCRAIL
  #define MODULE_ID 	MTYP_CANGC1e
  #define MODULE_TYPE   "GC1e"
  #define DEFAULT_NN   11
#elif CANEther
  #define MANU_ID       MANU_MERG
  #define MODULE_ID 	MTYP_CANEther
  #define MODULE_TYPE   "Ether"
  #define DEFAULT_NN    0xFFFD
#else
    No hardware defined - set command line switch -D<hardwaretype> in project build options
#endif



#define MAJOR_VER 2     // major version number
#define MINOR_VER 'e'	// Minor version character
#define MODULE_FLAGS    0b00001100  // Neither Producer or consumer, but can boot and is in FLiM mode
#define BUS_TYPE        PB_CAN      // Primarily a CAN device
#define BETA            13            

#define EVT_NUM 0
#define EVperEVT 0
#define NV_NUM 37

#define NUM_PARAMS      20          // Number of bytes in main parameter block
#define SPARE_PARAMS    10          // Number of spare unused parameter bytes
#define FCU_PARAMS      4           // Number of words (16 bit) in FCU parameter block

#define LOAD_ADDRESS    0x0800
#define MNAME_ADDRESS   LOAD_ADDRESS + 0x20 + NUM_PARAMS + SPARE_PARAMS + (FCU_PARAMS*2)   // Put module type string above params so checksum can be calculated at compile time


// EEPROM addresses
#define EE_MAGIC 0
#define EE_NV 1                         // 1 byte configuration flags
#define EE_NN 2                         // 2 bytes node number of this unit
#define EE_CANID EE_NN + 2              // 1 byte CANID
#define EE_IPADDR EE_CANID + 1          // 4 bytes IP address
#define EE_NETMASK EE_IPADDR + 4        // 4 bytes subnet mask
#define EE_MACADDR EE_NETMASK + 4       // 6 bytes MAC address
#define EE_CLEAN EE_MACADDR + 6         // 1 byte
#define EE_IDLETIME EE_CLEAN + 1        // 1 byte idle timeout
#define EE_PORTNUM EE_IDLETIME + 1      // 2 bytes port number
#define EE_REMOTEPORT EE_PORTNUM + 2    // 2 bytes remote bridge port number
#define EE_REMOTEIP EE_REMOTEPORT + 2   // 4 bytes remote bridge IP address
#define EE_GATEWAY EE_REMOTEIP + 4      // 4 bytes remote bridge gateway








// values
#define MAGIC 93

// node var 1
#define CFG_ALL  0xFF
#define CFG_IDLE_TIMEOUT    0x01
#define CFG_POWEROFF_ATIDLE 0x02
#define CFG_COMMAND_ACK     0x04
#define CFG_DHCP_CLIENT     0x08
#define CFG_REINIT_TCPIP       0x80   // Set msbit to reinitialise the TCP/IP paramters

#define SW_DEBOUNCE_COUNT   0XA0



extern unsigned short NN_temp;
extern unsigned char NV1;
extern unsigned char CANID;
extern unsigned char IdleTime;
extern unsigned char led1timer;
extern unsigned char led2timer;
extern unsigned char led3timer;
extern unsigned char Wait4NN;
extern unsigned char isLearning;
extern unsigned char maxcanq;
extern unsigned char maxethq;
extern unsigned char maxtxerr;
extern unsigned char maxrxerr;
extern unsigned char cmdticker;
extern BYTE          maxCANFifo;
extern WORD          aliveCounter;

extern volatile unsigned short tmr1_reload;

#endif
