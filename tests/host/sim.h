/*
 * Host test environment for the CANEther firmware: shared declarations.
 *
 * sim.c models the parts of the CANEther the firmware talks to: the PIC's
 * interrupt logic and timers, the ECAN controller on a CAN bus with other
 * nodes, the data EEPROM, and the TCP connections to PCs (in place of
 * Microchip's TCP/IP stack and the ENC28J60). scenarios.c drives it.
 */
#ifndef SIM_H
#define SIM_H

#include <stdint.h>

#define SIM_MAX_FRAMES   200000
#define SIM_SOCKETS      5
#define US               1000ull          /* nanoseconds per microsecond */
#define MS               1000000ull       /* nanoseconds per millisecond */

/* A CAN frame as the model sees it. */
typedef struct {
    uint8_t  sidh, sidl, eidh, eidl, dlc;   /* dlc: as in the DLC register, bit 6 = RTR */
    uint8_t  d[8];
} sim_frame_t;

/* One frame observed at a point in the system. */
typedef struct {
    uint64_t    t;          /* when, in ns of simulated time (for frames on the bus: end of frame) */
    uint64_t    start;      /* for frames on the bus: start of frame */
    sim_frame_t f;
    int         socket;     /* for frames to a PC: which connection */
} sim_event_t;

/* A frame another CAN node wants to send. */
typedef struct {
    uint64_t    ready;      /* when the node queues it */
    sim_frame_t f;
} sim_ext_frame_t;

/* A chunk of bytes a PC writes to its connection (one TCP segment). */
typedef struct {
    uint64_t t;
    int      socket;
    int      len;
    char     data[64];
    int      frame_seq;     /* sequence number of the frame that ends in this chunk, or -1 */
} sim_pc_write_t;

typedef struct {
    /* configuration, set by the scenario */
    uint64_t end_ns;
    unsigned seed;
    int      connected[SIM_SOCKETS];
    int      pc_acks[SIM_SOCKETS];          /* 0: this PC stops acknowledging (a stalled client) */
    int      pc_nagle[SIM_SOCKETS];         /* 1: this PC holds back data until earlier data is
                                               acknowledged (JMRI "Send frames: Grouped") */
    uint64_t pc_ack_delay_ns;               /* PC delayed-ACK time for a lone segment */
    uint64_t bus_dead_until_ns;             /* no other node acknowledges our frames until then */
    int      ee_writes;                     /* EEPROM writes made */
    unsigned fuzz_per_mille;                /* timing fuzz: chance per register access, in 1/1000, that
                                               the frame on the bus ends there, or that the stack's tick
                                               timer interrupts there */
    sim_ext_frame_t *ext;  int n_ext;       /* other nodes' traffic, sorted by ready time */
    sim_pc_write_t  *pcw;  int n_pcw;       /* PC writes, sorted by time */

    /* results */
    sim_event_t can_out[SIM_MAX_FRAMES];  int n_can_out;    /* frames this node put on the bus */
    sim_event_t can_in[SIM_MAX_FRAMES];   int n_can_in;     /* other nodes' frames, at end of frame */
    sim_event_t to_pc[SIM_MAX_FRAMES];    int n_to_pc;      /* frames delivered to PCs */
    int  hw_rx_overflows;
    int  tx_aborts;
    int  tx_rearm_errors;                   /* TXREQ set again while our frame was still on the bus */
    int  resets;
    int  watchdog_resets;
    int  eeprom_write_failures;
    uint64_t now;
    void   (*on_access)(int in_low_isr, int in_high_isr);  /* directed tests: called at every register access */
} sim_t;

extern sim_t sim;

void     sim_run(void);                     /* runs the firmware's main() until end_ns */
void     sim_reset(void);                   /* model to power-on state, for the directed tests */
void     sim_inject_rx(const sim_frame_t *f);
void     sim_force_tx_end(void);
void     sim_idle(uint64_t ns);
int      sim_in_low_isr(void);
void     sim_tick_interrupt(void);
uint8_t  sim_eeprom(uint8_t addr);
uint64_t sim_now(void);
int      sim_parse_gc(const char *s, int len, sim_frame_t *f);
int      sim_frame_bits(const sim_frame_t *f);

#endif
