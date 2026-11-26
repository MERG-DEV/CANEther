/* Host test build: C18's <delays.h>. Each delay lets simulated time pass (8 MIPS: 125 ns a cycle). */
#ifndef HOST_DELAYS_H
#define HOST_DELAYS_H
void Delay10TCYx(unsigned char n);
void Delay100TCYx(unsigned char n);
void Delay1KTCYx(unsigned char n);
void Delay10KTCYx(unsigned char n);
#endif
