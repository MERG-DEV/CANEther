/*
 *  Copyright (C) Pete Brownlow, 2013
 * 
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
 
  Recoded version of FLiM command parser implementing correct CBUS parameter block and additional config commands
 In the original Rocrail code, the main command  parsing routine only parsed and acted upon commands arriving via Ethernet.  It is now also called for all packets arriving whether by Ethernet or CBUS.
 
 */

#include <string.h>
#include <TCPIP Stack/TCPIP.h>

#include "project.h"
#include "cbusdefs8j.h"
#include "canbus.h"
#include "rocrail.h"
#include "utils.h"
#include "commands.h"
#include "cangc1e.h"
#include "io.h"
#include "gcaeth.h"

#pragma romdata BOOTFLAG
rom unsigned char bootflag = 0;

#pragma code APP
//
// parse_cmd()
//
// Decode the OPC and call the function to handle it.
//

/*
 * returns TRUE if the OPC can be broadcasted.
 */
unsigned char parseCmdEth(CANMsg* p_canmsg, unsigned char frametype) {
  BYTE broadcast = TRUE;
  CANMsg canmsg;
  BYTE i;
  BOOL eos = FALSE;


  if (frametype == 0) { // Message came from CAN, work out frame type
     frametype = ( (p_canmsg->b[sidl] & 0x08) ? EXT_FRAME : CAN_FRAME );
  }

  if (frametype == EXT_FRAME) {
    return TRUE;
  }


  if (frametype == ETH_FRAME) {
    switch (p_canmsg->b[d0]) {
      case 0:
        break;
      case 3:
      {
        canmsg.b[sidh] = (CANID >> 3);
        canmsg.b[sidl] = (CANID << 5);
        canmsg.b[d0] = 3; // CAN Status
        if (maxtxerr > 64 || maxrxerr > 64) {
          canmsg.b[d1] = 0;
        } else {
          canmsg.b[d1] = 0;
        }
        canmsg.b[d2] = maxethq;
        canmsg.b[d3] = maxcanq;
        canmsg.b[d4] = maxtxerr;
        canmsg.b[d5] = maxrxerr;
        canmsg.b[dlc] = 0x80 + 6;
        CBusEthBroadcast(&canmsg, 0xFF);
        maxethq = 0;
        maxcanq = 0;
        maxtxerr = 0;
        maxrxerr = 0;
        break;
      }
    }
    return FALSE;
  }

  cmdticker++;


  if (NV1 & CFG_COMMAND_ACK) {
    canmsg.b[sidh] = (CANID >> 3);
    canmsg.b[sidl] = (CANID << 5);
    canmsg.b[d0] = 2; // ack
    canmsg.b[d1] = 0; // rc
    canmsg.b[d2] = p_canmsg->b[d0]; // opc
    canmsg.b[d3] = cmdticker;
    canmsg.b[dlc] = 0x80 + 4;
    CBusEthBroadcast(&canmsg,0xFF);
  }


  switch (p_canmsg->b[d0]) {

    case OPC_QNN:
      canmsg.b[sidh] = (CANID >> 3);
      canmsg.b[sidl] = (CANID << 5);
      canmsg.b[d0] = OPC_PNN;
      canmsg.b[d1] = (NN_temp / 256) & 0xFF;
      canmsg.b[d2] = (NN_temp % 256) & 0xFF;
      canmsg.b[d3] = params[0];
      canmsg.b[d4] = params[2];
      canmsg.b[d5] = NV1;
      canmsg.b[dlc] = 6;
      CBusEthBroadcast(&canmsg,0xFF);

      break;

    case OPC_RQNPN:
      // Request to read a parameter
      if (thisNN(p_canmsg) == 1) {
        doRqnpn((unsigned int) p_canmsg->b[d3]);
        broadcast = FALSE;
      }
      break;

    case OPC_SNN:
    {
      if (Wait4NN) {
        unsigned char nnH = p_canmsg->b[d1];
        unsigned char nnL = p_canmsg->b[d2];
        NN_temp = nnH * 256 + nnL;
        eeWrite(EE_NN+1, nnH);
        eeWrite(EE_NN, nnL);
        Wait4NN = 0;
        SLIM_LEDG = 0;
        FLIM_LEDY = 1;
        
        // Respond to SNN with NNACK (only if JP2 is in, as might confuse Rocrail)  - PNB 3/1/13

        if (JP2 == 0) {
            canmsg.b[sidh] = (CANID >> 3);
            canmsg.b[sidl] = (CANID << 5);
            canmsg.b[d0] = OPC_NNACK;
            canmsg.b[d1] = nnH;
            canmsg.b[d2] = nnL;
            canmsg.b[dlc] = 3;
            CBusEthBroadcast(&canmsg, 0xFF);
        }
        broadcast = FALSE;
      }
      break;
    }


    case OPC_RQNP:
      if (Wait4NN) {
        canmsg.b[sidh] = (CANID >> 3);
        canmsg.b[sidl] = (CANID << 5);
        canmsg.b[d0] = OPC_PARAMS;
        canmsg.b[d1] = params[0];
        canmsg.b[d2] = params[1];
        canmsg.b[d3] = params[2];
        canmsg.b[d4] = params[3];
        canmsg.b[d5] = params[4];
        canmsg.b[d6] = params[5];
        canmsg.b[d7] = params[6];
        canmsg.b[dlc] = 8;
        CBusEthBroadcast(&canmsg,0xFF);
        broadcast = FALSE;
      }
      break;

    case OPC_RQMN:
      if (Wait4NN) {
        canmsg.b[sidh] = (CANID >> 3);
        canmsg.b[sidl] = (CANID << 5);
        canmsg.b[d0] = OPC_NAME;
        
        for (i=0; i<7; i++) {
            eos |= (module_type[i] == '\0');
            canmsg.b[d1+i] = (eos ? ' ' : module_type[i] );  // Blank pad reply with spaces
        }
      
        canmsg.b[dlc] = 8;
        CBusEthBroadcast(&canmsg,0xFF);
        broadcast = FALSE;
      }
      break;

      /* bootloader is CAN only so although the opcode will put us into BOOT mode wherever
       * it came from, the actual bootload packets will only be seen arriving on CAN */
    case OPC_BOOT:
        // Enter bootloader mode if NN matches
        if (thisNN(p_canmsg) == 1) {
            eeWrite((unsigned char) (&bootflag), 0xFF);
            Reset();
        }
        break;

    case OPC_NNRST:
        // Reset module if NN matches
        if (thisNN(p_canmsg) == 1) {
             Reset();
        }
        break;



    case OPC_NVRD:
      if (thisNN(p_canmsg)) {
        BYTE nvnr = p_canmsg->b[d3];
        canmsg.b[sidh] = ((CANID & 0x78) >> 3);
        canmsg.b[sidl] = ((CANID & 0x07) << 5);
        canmsg.b[d0] = OPC_NVANS;
        canmsg.b[d1] = (NN_temp / 256) & 0xFF;
        canmsg.b[d2] = (NN_temp % 256) & 0xFF;
        canmsg.b[d3] = nvnr;
        canmsg.b[dlc] = 5;

        if (nvnr == 1) {
          canmsg.b[d4] = NV1;
          CBusEthBroadcast(&canmsg,0xFF);
          broadcast = FALSE;
        } else if (nvnr == 2) {
          canmsg.b[d4] = CANID;
          CBusEthBroadcast(&canmsg,0xFF);
          broadcast = FALSE;
        } else if (nvnr > 2 && nvnr < 7) {
          canmsg.b[d4] = eeRead(EE_IPADDR + nvnr - 3);
          CBusEthBroadcast(&canmsg,0xFF);
          broadcast = FALSE;
        } else if (nvnr > 6 && nvnr < 11) {
          canmsg.b[d4] = eeRead(EE_NETMASK + nvnr - 7);
          CBusEthBroadcast(&canmsg,0xFF);
          broadcast = FALSE;
        } else if (nvnr > 10 && nvnr < 17) {
          canmsg.b[d4] = eeRead(EE_MACADDR + nvnr - 11);
          CBusEthBroadcast(&canmsg,0xFF);
          broadcast = FALSE;
        } else if (nvnr == 17) {
          canmsg.b[d4] = eeRead(EE_IDLETIME);
          CBusEthBroadcast(&canmsg,0xFF);
          broadcast = FALSE;
        } else if (nvnr > 17 && nvnr < 30) {
          canmsg.b[d4] = eeRead(EE_PORTNUM + nvnr - 18);
          CBusEthBroadcast(&canmsg,0xFF);
          broadcast = FALSE;
        } else if (nvnr > 29 && nvnr < 32) {    // Spare NVs below read onlys
          canmsg.b[d4] = 0;
          CBusEthBroadcast(&canmsg,0xFF);
          broadcast = FALSE;
        } else if (nvnr > 31 && nvnr < 36) {  // Read only NVs for IP address currently in use
          canmsg.b[d4] = AppConfig.MyIPAddr.v[nvnr - 32];
          CBusEthBroadcast(&canmsg,0xFF);
          broadcast = FALSE;
        } else if (nvnr > 35 && nvnr < 37) {  // Read only NV for connections status
          canmsg.b[d4] = nrClients;
          if (isBridgeConnected())
              canmsg.b[d4] |= 0x80;          // Number of active connections plus set msbit if bridge connected
          CBusEthBroadcast(&canmsg,0xFF);
          broadcast = FALSE;
        } else if (nvnr > 36 && nvnr < 38) {  // Read only NV for max CAN RX software fifo entries used
            canmsg.b[d4] = maxCANFifo;
            CBusEthBroadcast(&canmsg,0xFF);
          broadcast = FALSE;
        } else if (nvnr > 37 && nvnr < 39) {  // Read only NV for max CAN TX software fifo entries used
#ifdef WITHCAN            
            canmsg.b[d4] = maxCanTxFifo;
#else
            canmsg.b[d4] = 0;
#endif            
            CBusEthBroadcast(&canmsg,0xFF);
          broadcast = FALSE;
        }
      }
      break;

    case OPC_NVSET:
      if (thisNN(p_canmsg)) {
        byte nvnr = p_canmsg->b[d3];
        if (nvnr == 1) {
          NV1 = p_canmsg->b[d4];

          if (NV1 & CFG_REINIT_TCPIP)   // MS bit set to reinitialise TCP/IP parameters
          {
             InitAppConfig();   // Reinitialise TCP/IP parameters
          }
          else
          {
              eeWrite(EE_NV, NV1);

              if (NV1 & CFG_DHCP_CLIENT) {
                AppConfig.Flags.bIsDHCPEnabled = TRUE;
                DHCPInit(0);
              } else {
                AppConfig.Flags.bIsDHCPEnabled = FALSE;
                DHCPDisable(0);
              }
          }
        } else if (nvnr == 2) {
          CANID = p_canmsg->b[d4];
          eeWrite(EE_CANID, CANID);
        } else if (nvnr > 2 && nvnr < 7) {
          eeWrite(EE_IPADDR + nvnr - 3, p_canmsg->b[d4]);
        } else if (nvnr > 6 && nvnr < 11) {
          eeWrite(EE_NETMASK + nvnr - 7, p_canmsg->b[d4]);
        } else if (nvnr > 10 && nvnr < 17) {
          eeWrite(EE_MACADDR + nvnr - 11, p_canmsg->b[d4]);
        } else if (nvnr == 17) {
          IdleTime = p_canmsg->b[d4];
          eeWrite(EE_IDLETIME, IdleTime);
        } else if (nvnr > 17 && nvnr < 30) {
          eeWrite(EE_PORTNUM + nvnr - 18, p_canmsg->b[d4]);
        } 

        // Acknowledge NV write with WRACK, sender should not attempt another
        // operation until WRACK received, to allow for non volatile write times - PNB 3/1/13

        canmsg.b[sidh] = (CANID >> 3);
        canmsg.b[sidl] = (CANID << 5);
        canmsg.b[d0] = OPC_WRACK;
        canmsg.b[d1] = NN_temp / 256;
        canmsg.b[d2] = NN_temp % 256;
        canmsg.b[dlc] = 3;
        CBusEthBroadcast(&canmsg, 0xFF);

        broadcast = FALSE;
      }
      break;

    default:
      break;
  }


  return broadcast;
}

void doRqnpn(unsigned int idx) {
  CANMsg canmsg;

  if (idx <= NUM_PARAMS) {
    canmsg.b[sidh] = ((CANID & 0x78) >> 3);
    canmsg.b[sidl] = ((CANID & 0x07) << 5);
    canmsg.b[d0] = OPC_PARAN;
    canmsg.b[d1] = (NN_temp / 256) & 0xFF;
    canmsg.b[d2] = (NN_temp % 256) & 0xFF;
    canmsg.b[d3] = idx;
    
    if (idx == 0)  // Index zero to read number of parameters supported - PNB 3/1/13  
        canmsg.b[d4] = NUM_PARAMS;
    else
        canmsg.b[d4] = params[idx - 1];
    
    canmsg.b[dlc] = 5;
    CBusEthBroadcast(&canmsg,0xFF);
  }
}

byte thisNN(CANMsg* p_canmsg) {
  if ((((unsigned short) (p_canmsg->b[d1]) << 8) + p_canmsg->b[d2]) == NN_temp) {
    return 1;
  } else
    return 0;

}

// Send status sends a CBUS data event with the CANEther node number and 5 bytes of status as follows:
//  Byte 1 - Number of active connections (msbit set for bridge client active)
//  Byte 2 - Max CAN rx s/w FIFO buffers used
//  Byte 3 - Max tx s/w FIFO buffers used
//  Byte 4 - Transmit error count
//  Byte 5 - lsnibble is lost arbitration count
//         - msnibble is transmit timeout count

// The error counts max out at 0xFF (or 0xF for a nibble)


void sendStatus(void)

{
    CANMsg canmsg;

    canmsg.b[sidh] = (CANID >> 3);
    canmsg.b[sidl] = (CANID << 5);
    canmsg.b[d0] = OPC_ACDAT;
    canmsg.b[d1] = (NN_temp / 256) & 0xFF;
    canmsg.b[d2] = (NN_temp % 256) & 0xFF;
    canmsg.b[d3] = nrClients;
    if (isBridgeConnected())
        canmsg.b[d3] |= 0x80;          // Number of active connetions plus set msbit if bridge connected
    canmsg.b[d4] = maxCANFifo;
    canmsg.b[dlc] = 5;
}

