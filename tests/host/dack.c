/*
 * Host test environment: the firmware's TCP delayed-acknowledgement time.
 * TCPIPConfig.h can set TCP_DELAYED_ACK_TIMEOUT; TCP.c otherwise uses 100 ms.
 * The stack keeps the timer in units of 256 ticks of TICK_SECOND (31250 Hz on
 * the CANEther, 8 MHz peripheral clock / 256), so it rounds down to 8.192 ms.
 */
#include <GenericTypeDefs.h>
#define TICK_SECOND 31250ull
#include "TCPIPConfig.h"
#ifndef TCP_DELAYED_ACK_TIMEOUT
#define TCP_DELAYED_ACK_TIMEOUT ((DWORD)TICK_SECOND/10)     /* TCP.c */
#endif
const unsigned long long fw_delayed_ack_ns = ((unsigned long long)((TCP_DELAYED_ACK_TIMEOUT) >> 8)) * 256ull * 1000000000ull / TICK_SECOND;
