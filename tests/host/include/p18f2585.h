/*
 * Host test build: a stand-in for C18's <p18f2585.h>.
 *
 * Every special function register the CANEther firmware uses is declared here.
 * Registers that the CAN controller, the interrupt logic or the data EEPROM
 * change by themselves are reached through hw_*() accessors. Each access runs
 * the hardware model in sim.c: simulated time moves on a little, the CAN bus
 * and timers progress, and an enabled, pending interrupt is taken there and
 * then, as the PIC would take it between two instructions. That is how the
 * tests reach the windows inside the firmware's critical sections.
 *
 * Bit positions follow the PIC18F2585 data sheet (ECAN Mode 2 names).
 */
#ifndef HOST_P18F2585_H
#define HOST_P18F2585_H

typedef unsigned char hw_byte;

/* C18 keywords and intrinsics */
#define rom
#define far
#define near
#define _asm
#define _endasm ;
#define GOTO
#define Nop()       ((void)0)
#define ClrWdt()    hw_clrwdt()
#define Reset()     hw_reset()

typedef union { hw_byte v; struct { hw_byte TMR0IF:1, INT0IF:1, RBIF:1, TMR0IE:1, INT0IE:1, RBIE:1, GIEL:1, GIEH:1; } b; } hw_intcon_t;
#define GIE  GIEH
#define PEIE GIEL
typedef union { hw_byte v; struct { hw_byte FIFOWMIF:1, RXBnIF:1, TXB0IF:1, TXB1IF:1, TXBnIF:1, ERRIF:1, WAKIF:1, IRXIF:1; } b; } hw_pir3_t;
typedef union { hw_byte v; struct { hw_byte FIFOWMIE:1, RXBnIE:1, TXB0IE:1, TXB1IE:1, TXBnIE:1, ERRIE:1, WAKIE:1, IRXIE:1; } b; } hw_pie3_t;
typedef union { hw_byte v; struct { hw_byte EWARN:1, RXWARN:1, TXWARN:1, RXBP:1, TXBP:1, TXBO:1, RXBnOVFL:1, FIFOEMPTY:1; } b; } hw_comstat_t;
typedef union { hw_byte v; struct { hw_byte EICODE:5, OPMODE0:1, OPMODE1:1, OPMODE2:1; } b; } hw_canstat_t;
typedef struct { hw_byte TXPRI0:1, TXPRI1:1, unused:1, TXREQ:1, TXERR:1, TXLARB:1, TXABT:1, TXBIF:1; } hw_txbcon_bits_t;
typedef union { hw_byte r[14]; hw_txbcon_bits_t b; } hw_txbuf_t;     /* CON SIDH SIDL EIDH EIDL DLC D0..D7 */
typedef union { hw_byte v; struct { hw_byte TMR1IF:1, TMR2IF:1, CCP1IF:1, SSPIF:1, TXIF:1, RCIF:1, ADIF:1, PSPIF:1; } b; } hw_pir1_t;
typedef union { hw_byte v; struct { hw_byte CCP2IF:1, TMR3IF:1, HLVDIF:1, BCLIF:1, EEIF:1, unused:1, CMIF:1, OSCFIF:1; } b; } hw_pir2_t;
typedef union { hw_byte v; struct { hw_byte RD:1, WR:1, WREN:1, WRERR:1, FREE:1, unused:1, CFGS:1, EEPGD:1; } b; } hw_eecon1_t;

typedef struct {
    hw_intcon_t  intcon;
    hw_pir3_t    pir3;
    hw_pie3_t    pie3;
    hw_comstat_t comstat;
    hw_canstat_t canstat;
    hw_byte      cancon;
    hw_txbuf_t   txb[3];
    hw_byte      rxb[8][14];        /* RXB0, RXB1, B0..B5: the 8-deep Mode 2 FIFO */
    hw_byte      txerrcnt, rxerrcnt;
    hw_pir1_t    pir1;
    hw_pir2_t    pir2;
    hw_eecon1_t  eecon1;
    hw_byte      eeadr, eedata, eecon2;
    hw_byte      wdtcon;
    struct { hw_byte BOR:1, POR:1, TO:1, PD:1, RI:1, unused:1, SBOREN:1, IPEN:1; } rcon;
} hw_regs_t;
extern hw_regs_t hw;

void hw_access(void);       /* one register access: the model runs here */
void hw_clrwdt(void);
void hw_reset(void);
hw_byte *hw_eedata(void);
hw_pir2_t *hw_pir2(void);
hw_byte *hw_eecon2(void);
hw_eecon1_t *hw_eecon1(void);

#define HWREG(field)        (*(hw_access(), &hw.field))

#define INTCON          HWREG(intcon.v)
#define INTCONbits      HWREG(intcon.b)
#define PIR3            HWREG(pir3.v)
#define PIR3bits        HWREG(pir3.b)
#define PIE3            HWREG(pie3.v)
#define PIE3bits        HWREG(pie3.b)
#define COMSTATbits     HWREG(comstat.b)
#define CANSTATbits     HWREG(canstat.b)
#define CANCON          HWREG(cancon)
#define TXB0CON         HWREG(txb[0].r[0])
#define TXB0CONbits     HWREG(txb[0].b)
#define TXB0SIDH        HWREG(txb[0].r[1])
#define TXB1CON         HWREG(txb[1].r[0])
#define TXB1CONbits     HWREG(txb[1].b)
#define TXB2CON         HWREG(txb[2].r[0])
#define RXB0CON         HWREG(rxb[0][0])
#define RXB1CON         HWREG(rxb[1][0])
#define B0CON           HWREG(rxb[2][0])
#define B1CON           HWREG(rxb[3][0])
#define B2CON           HWREG(rxb[4][0])
#define B3CON           HWREG(rxb[5][0])
#define B4CON           HWREG(rxb[6][0])
#define B5CON           HWREG(rxb[7][0])
#define TXERRCNT        HWREG(txerrcnt)
#define RXERRCNT        HWREG(rxerrcnt)
#define PIR1            HWREG(pir1.v)
#define PIR1bits        HWREG(pir1.b)
#define PIR2            (hw_pir2()->v)
#define PIR2bits        (hw_pir2()->b)
#define EECON1          (hw_eecon1()->v)
#define EECON1bits      (hw_eecon1()->b)
#define EECON2          (*hw_eecon2())
#define WDTCON          HWREG(wdtcon)
#define RCONbits        HWREG(rcon)
#define EEADR           HWREG(eeadr)
#define EEDATA          (*hw_eedata())

/* Registers with no behaviour in the model: plain storage. */
typedef struct { hw_byte RA0:1, RA1:1, RA2:1, RA3:1, RA4:1, RA5:1, RA6:1, RA7:1; } hw_porta_t;
typedef struct { hw_byte RB0:1, RB1:1, RB2:1, RB3:1, RB4:1, RB5:1, RB6:1, RB7:1; } hw_portb_t;
typedef struct { hw_byte RC0:1, RC1:1, RC2:1, RC3:1, RC4:1, RC5:1, RC6:1, RC7:1; } hw_portc_t;
typedef struct { hw_byte TRISA0:1, TRISA1:1, TRISA2:1, TRISA3:1, TRISA4:1, TRISA5:1, TRISA6:1, TRISA7:1; } hw_trisa_t;
typedef struct { hw_byte TRISB0:1, TRISB1:1, TRISB2:1, TRISB3:1, TRISB4:1, TRISB5:1, TRISB6:1, TRISB7:1; } hw_trisb_t;
typedef struct { hw_byte TRISC0:1, TRISC1:1, TRISC2:1, TRISC3:1, TRISC4:1, TRISC5:1, TRISC6:1, TRISC7:1; } hw_trisc_t;
typedef struct { hw_byte TXB0IE:1, TXB1IE:1, TXB2IE:1, rest:5; } hw_txbie_t;
typedef struct { hw_byte TMR1ON:1, rest:7; } hw_t1con_t;
typedef struct { hw_byte TMR1IP:1, rest:7; } hw_ipr1_t;
typedef struct { hw_byte TMR1IE:1, rest:7; } hw_pie1_t;
typedef struct { hw_byte RBIP:1, rest:6, RBPU:1; } hw_intcon2_t;

extern hw_porta_t PORTAbits;
extern hw_portb_t PORTBbits;
extern hw_portc_t PORTCbits;
extern hw_trisa_t TRISAbits;
extern hw_trisb_t TRISBbits;
extern hw_trisc_t TRISCbits;
extern hw_txbie_t TXBIEbits;
extern hw_t1con_t T1CONbits;
extern hw_ipr1_t  IPR1bits;
extern hw_pie1_t  PIE1bits;
extern hw_intcon2_t INTCON2bits;
extern hw_byte ADCON0, ADCON1, T1CON, TMR1H, TMR1L, IPR1, IPR2, IPR3, PIE1, PIE2,
               INTCON2, INTCON3, BRGCON1, BRGCON2, BRGCON3, CIOCON, ECANCON, BSEL0, BIE0,
               RXM0SIDL, RXM0SIDH, RXM1SIDL, RXM1SIDH, RXM0EIDL, RXM0EIDH, RXM1EIDL, RXM1EIDH,
               TRISA, TRISB, TRISC;

#endif
