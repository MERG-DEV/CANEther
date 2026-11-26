/* Host test build: the parts of Microchip's TCP/IP stack API that the firmware calls,
 * implemented by the test model in sim.c. */
#ifndef HOST_TCPIP_H
#define HOST_TCPIP_H
#include <GenericTypeDefs.h>
#include <p18f2585.h>
#include <string.h>

typedef BYTE TCP_SOCKET;
#define INVALID_SOCKET      (0xFE)
#define TCP_OPEN_SERVER     0
#define TCP_OPEN_IP_ADDRESS 3
#define TCP_PURPOSE_GENERIC_TCP_CLIENT 0
#define TCP_PURPOSE_GENERIC_TCP_SERVER 1

typedef union { DWORD Val; BYTE v[4]; } IP_ADDR;
typedef struct { BYTE v[6]; } MAC_ADDR;
typedef struct {
    IP_ADDR MyIPAddr, MyMask, MyGateway, PrimaryDNSServer, SecondaryDNSServer;
    MAC_ADDR MyMACAddr;
    BYTE NetBIOSName[16];
    struct { unsigned char bIsDHCPEnabled:1; unsigned char bInConfigMode:1; } Flags;
} APP_CONFIG;
extern APP_CONFIG AppConfig;

#define STACK_USE_DHCP_CLIENT
#define MY_DEFAULT_HOST_NAME   "CBUS           "
#define MY_DEFAULT_MAC_BYTE1 (0x00)
#define MY_DEFAULT_MAC_BYTE2 (0x04)
#define MY_DEFAULT_MAC_BYTE3 (0xA3)
#define MY_DEFAULT_MAC_BYTE4 (0x02)
#define MY_DEFAULT_MAC_BYTE5 (0x65)
#define MY_DEFAULT_MAC_BYTE6 (0x0D)
#define MY_DEFAULT_IP_ADDR_BYTE1 (192ul)
#define MY_DEFAULT_IP_ADDR_BYTE2 (168ul)
#define MY_DEFAULT_IP_ADDR_BYTE3 (0ul)
#define MY_DEFAULT_IP_ADDR_BYTE4 (200ul)
#define MY_DEFAULT_MASK_BYTE1 (255ul)
#define MY_DEFAULT_MASK_BYTE2 (255ul)
#define MY_DEFAULT_MASK_BYTE3 (255ul)
#define MY_DEFAULT_MASK_BYTE4 (0ul)
#define MY_DEFAULT_GATE_BYTE1 (192ul)
#define MY_DEFAULT_GATE_BYTE2 (168ul)
#define MY_DEFAULT_GATE_BYTE3 (0ul)
#define MY_DEFAULT_GATE_BYTE4 (1ul)

TCP_SOCKET TCPOpen(DWORD dwRemoteHost, BYTE vRemoteHostType, WORD wPort, BYTE vSocketPurpose);
BOOL TCPIsConnected(TCP_SOCKET s);
WORD TCPIsGetReady(TCP_SOCKET s);
BOOL TCPGet(TCP_SOCKET s, BYTE *b);
WORD TCPGetArray(TCP_SOCKET s, BYTE *buf, WORD len);
void TCPDiscard(TCP_SOCKET s);
WORD TCPIsPutReady(TCP_SOCKET s);
WORD TCPPutArray(TCP_SOCKET s, BYTE *data, WORD len);
void TCPFlush(TCP_SOCKET s);
void TCPDisconnect(TCP_SOCKET s);
void TCPClose(TCP_SOCKET s);
void TickInit(void);
void TickUpdate(void);
DWORD TickGet(void);
void StackInit(void);
void StackTask(void);
void StackApplications(void);
void AnnounceIP(void);
void DHCPInit(BYTE i);
void DHCPDisable(BYTE i);
#endif
