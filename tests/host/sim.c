/*
 * Host test environment for the CANEther firmware: the hardware and network model.
 *
 * The firmware's own source files are compiled unchanged against
 * include/p18f2585.h, whose register accessors call hw_access() here. Each call
 * stands for a few instructions: simulated time moves on, the CAN bus, timers,
 * EEPROM and TCP connections progress, and any enabled, pending interrupt is
 * taken at once, as the PIC takes it between instructions. Interrupts can so
 * land anywhere the firmware touches a register, including inside its critical
 * sections; the random cost of each access (seeded) varies where they land.
 *
 * Model limits, stated so results are read correctly:
 *  - CAN: 125 kbit/s, frame length from DLC plus a fixed allowance for bit
 *    stuffing; arbitration by identifier; 3-bit interframe space; when the bus
 *    is "dead" no node acknowledges our frames, which then fail and retry, with
 *    the transmit error counter rising as in the CAN specification.
 *  - Clearing TXREQ while our frame is on the bus: the frame is still sent
 *    in full (data sheet 23.6.2), then counts as sent or, if it fails, as
 *    aborted (TXABT). The data sheet does not say what TXREQ reads meanwhile;
 *    the model reads 0, the harder case for the firmware, and counts it as an
 *    error if the firmware sets TXREQ again before that frame has ended.
 *  - ECAN Mode 2 receive FIFO of 8 buffers; the FIFO interrupt (FIFOWM = 0) is
 *    raised when four buffers are full; overflow sets RXBnOVFL and ERRIF.
 *  - ERRIF is raised when the transmit error counter crosses 96 or 128, and on
 *    receive overflow (data sheet 23.15.6: overflow or error state change).
 *    It is not raised by lost arbitration.
 *  - TCP: per connection, a 256-byte receive window and a 512-byte send buffer,
 *    as TCPIPConfig.h sets them. Unflushed data goes when the buffer is half
 *    full or 40 ms after the first unflushed byte, at the next StackTask(), as
 *    TCP.c does; TCPFlush() sends at once. The PC acknowledges every second
 *    segment at once and a lone segment after pc_ack_delay_ns. Wire time 150 us.
 *  - Timing fuzz (optional, per scenario): at a small chance per register
 *    access, the frame on the bus ends there and then (it is cut short), or
 *    the tick timer interrupts there. Frame ends and
 *    interrupts so land at any point in the code, which finds rare
 *    interleavings quickly. Frames are shorter than real ones, so the bus is
 *    faster than 125 kbit/s; the firmware runs at its usual speed.
 *  - A PC with Nagle's algorithm on (JMRI "Grouped") sends what it has only
 *    when its earlier data is acknowledged. We acknowledge a second segment
 *    at once and a lone one after the firmware's TCP_DELAYED_ACK_TIMEOUT
 *    (read from its TCPIPConfig.h, in the stack's 8.192 ms units), at the
 *    next StackTask(); any data we send carries the acknowledgement.
 *  - Watchdog: once software enables it (WDTCON), the module resets if
 *    ClrWdt() is not called for 4.1 s (postscaler 1024, as 2eBETA14 sets).
 *  - Data EEPROM: a write begins only if WR is set straight after the two
 *    EECON2 writes (data sheet 7.4); otherwise EEIF never comes.
 *  - StackTask() costs 150 to 400 us of simulated time per call, and 0.5 to
 *    1.5 ms more while a connection has data moving (an estimate: the ENC28J60
 *    is read over SPI and the TCP state is kept in its RAM).
 */
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sim.h"
#include "p18f2585.h"
#include "TCPIP Stack/TCPIP.h"

sim_t sim;
hw_regs_t hw;

hw_porta_t PORTAbits; hw_portb_t PORTBbits; hw_portc_t PORTCbits;
hw_trisa_t TRISAbits; hw_trisb_t TRISBbits; hw_trisc_t TRISCbits;
hw_txbie_t TXBIEbits; hw_t1con_t T1CONbits; hw_ipr1_t IPR1bits; hw_pie1_t PIE1bits;
hw_intcon2_t INTCON2bits;
hw_byte ADCON0, ADCON1, T1CON, TMR1H, TMR1L, IPR1, IPR2, IPR3, PIE1, PIE2,
        INTCON2, INTCON3, BRGCON1, BRGCON2, BRGCON3, CIOCON, ECANCON, BSEL0, BIE0,
        RXM0SIDL, RXM0SIDH, RXM1SIDL, RXM1SIDH, RXM0EIDL, RXM0EIDH, RXM1EIDL, RXM1EIDH,
        TRISA, TRISB, TRISC;

extern void isr_high(void);
extern void isr_low(void);
extern void firmware_main(void);

static jmp_buf run_end;
static int in_low, in_high, in_model;
static uint32_t rng;
static uint8_t eeprom[256];
static unsigned long access_seq, eecon2_seq[2], wr_unlocked;
static uint64_t last_clrwdt;
static void wdt_check(void);

static uint32_t rnd(void) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }
static uint64_t rnd_range(uint64_t lo, uint64_t hi) { return lo + rnd() % (hi - lo + 1); }
uint64_t sim_now(void) { return sim.now; }

/* ------------------------------------------------------------------ CAN bus */

#define BIT_NS      8000ull                 /* 125 kbit/s */
#define IFS_BITS    3
#define ERRFRAME_BITS 20

int sim_frame_bits(const sim_frame_t *f)
{
    int n = f->dlc & 0x0F;
    int ext = (f->sidl & 0x08) != 0;
    if (n > 8) n = 8;
    if (f->dlc & 0x40) n = 0;
    /* SOF..EOF: 44 bits standard or 64 extended, plus data, plus about 10% stuffing */
    return (ext ? 64 : 44) + 8 * n + (ext ? 64 : 44) / 10 + n;
}

static unsigned arb_id(const sim_frame_t *f)
{
    unsigned id = ((unsigned)f->sidh << 3) | (f->sidl >> 5);
    if (f->sidl & 0x08)
        id = (id << 18) | ((unsigned)(f->sidl & 0x03) << 16) | ((unsigned)f->eidh << 8) | f->eidl;
    else
        id <<= 18;
    return id;
}

static struct {
    int busy, who;                  /* who: -1 this node, else index into sim.ext */
    uint64_t start, end, free_at;
    sim_frame_t f;
    int ext_next;                   /* next sim.ext entry not yet queued */
    int pend[4096], n_pend;         /* other nodes' queued frames (indices into sim.ext) */
    int tx_contending, tx_rearm_seen, abort_req;
    uint64_t tx_ready;
    sim_frame_t tx_frame;
    int rx_rd, rx_wr, rx_count;
    int tec;
} can;

static void tec_change(int newtec)
{
    int old = can.tec;
    if (newtec > 255) newtec = 255;
    if (newtec < 0) newtec = 0;
    can.tec = newtec;
    if ((old < 96) != (newtec < 96) || (old < 128) != (newtec < 128))
        hw.pir3.b.ERRIF = 1;
}

static void can_sync_registers(void)
{
    /* Our transmit buffer: TXREQ set by the firmware starts a request; cleared while waiting is an
       abort; cleared while our frame is on the bus requests an abort that takes effect at its end. */
    int ours_on_wire = can.busy && can.who == -1;
    if (ours_on_wire) {
        if (!hw.txb[0].b.TXREQ)
            can.abort_req = 1;
        else if (can.abort_req && !can.tx_rearm_seen) {
            /* The firmware has set TXREQ again before the aborted frame has left the bus. The
               data sheet does not say what the controller does then; firmware must not do it. */
            can.tx_rearm_seen = 1;
            sim.tx_rearm_errors++;
        }
    } else if (hw.txb[0].b.TXREQ && !can.tx_contending) {
        memcpy(&can.tx_frame, &hw.txb[0].r[1], 5);
        memcpy(can.tx_frame.d, &hw.txb[0].r[6], 8);
        hw.txb[0].b.TXABT = 0; hw.txb[0].b.TXLARB = 0; hw.txb[0].b.TXERR = 0;
        can.tx_contending = 1;
        can.tx_ready = sim.now;
    } else if (!hw.txb[0].b.TXREQ && can.tx_contending) {
        can.tx_contending = 0;
        hw.txb[0].b.TXABT = 1;
        sim.tx_aborts++;
    }
    /* Receive FIFO: the read pointer moves on when the firmware clears RXFUL. */
    while (can.rx_count > 0 && !(hw.rxb[can.rx_rd][0] & 0x80)) {
        can.rx_rd = (can.rx_rd + 1) & 7;
        can.rx_count--;
    }
    hw.cancon = (hw.cancon & 0xE0) | can.rx_rd;
    hw.comstat.b.FIFOEMPTY = can.rx_count > 0;          /* Mode 2: 1 = FIFO not empty */
    hw.comstat.b.TXWARN = can.tec > 95;
    hw.comstat.b.TXBP = can.tec > 127;
    hw.txerrcnt = (hw_byte)can.tec;
    hw.canstat.b.OPMODE2 = (hw.cancon >> 7) & 1;
}

static void can_complete(void)
{
    if (can.who == -1) {
        if (can.end < sim.bus_dead_until_ns) {
            /* No acknowledgement: error frame, counter up, the controller retries. */
            hw.txb[0].b.TXERR = 1;
            if (can.tec < 128)              /* an error-passive sender's ACK errors don't count (CAN 2.0B 7.2) */
                tec_change(can.tec + 8);
            can.free_at = can.end + ERRFRAME_BITS * BIT_NS;
            if (can.abort_req) {                /* abort requested while on the wire */
                can.tx_contending = 0;
                can.abort_req = can.tx_rearm_seen = 0;
                hw.txb[0].b.TXABT = 1;
                sim.tx_aborts++;
            }
            return;
        }
        can.abort_req = can.tx_rearm_seen = 0;
        tec_change(can.tec - 1);
        hw.txb[0].b.TXREQ = 0;
        hw.txb[0].b.TXBIF = 1;
        if (TXBIEbits.TXB0IE)
            hw.pir3.b.TXBnIF = 1;
        can.tx_contending = 0;
        if (sim.n_can_out < SIM_MAX_FRAMES) {
            sim.can_out[sim.n_can_out].t = can.end;
            sim.can_out[sim.n_can_out].start = can.start;
            sim.can_out[sim.n_can_out].f = can.f;
            sim.n_can_out++;
        }
    } else {
        sim_frame_t *f = &sim.ext[can.who].f;
        if (sim.n_can_in < SIM_MAX_FRAMES) {
            sim.can_in[sim.n_can_in].t = can.end;
            sim.can_in[sim.n_can_in].start = can.start;
            sim.can_in[sim.n_can_in].f = *f;
            sim.n_can_in++;
        }
        sim.can_in[sim.n_can_in - 1].socket = -1;
        if (can.rx_count == 8) {
            sim.can_in[sim.n_can_in - 1].socket = -2;     /* lost in hardware: FIFO full */
            hw.comstat.b.RXBnOVFL = 1;
            hw.pir3.b.ERRIF = 1;
            sim.hw_rx_overflows++;
        } else {
            hw_byte *b = hw.rxb[can.rx_wr];
            b[0] = 0x80;                                    /* RXFUL */
            b[1] = f->sidh; b[2] = f->sidl; b[3] = f->eidh; b[4] = f->eidl;
            b[5] = f->dlc & 0x4F;                           /* RB1:RB0 read as 0 */
            memcpy(&b[6], f->d, 8);
            can.rx_wr = (can.rx_wr + 1) & 7;
            can.rx_count++;
            hw.pir3.b.RXBnIF = 1;
            if (can.rx_count == 4)
                hw.pir3.b.FIFOWMIF = 1;                     /* FIFOWM = 0: four buffers remain */
        }
    }
    can.free_at = can.end + IFS_BITS * BIT_NS;
}

static void can_until(uint64_t target)
{
    for (;;) {
        uint64_t start, earliest;
        int i, winner, have;

        if (can.busy) {
            if (can.end > target) break;
            sim.now = can.end;
            can.busy = 0;
            can_complete();
            can_sync_registers();
            continue;
        }
        while (can.ext_next < sim.n_ext && sim.ext[can.ext_next].ready <= target && can.n_pend < 4096)
            can.pend[can.n_pend++] = can.ext_next++;

        have = 0; earliest = 0;
        if (can.tx_contending) { earliest = can.tx_ready; have = 1; }
        for (i = 0; i < can.n_pend; i++) {
            uint64_t r = sim.ext[can.pend[i]].ready;
            if (!have || r < earliest) { earliest = r; have = 1; }
        }
        if (!have) break;
        start = earliest > can.free_at ? earliest : can.free_at;
        if (start > target) break;

        /* Arbitration among everyone ready at start. */
        winner = -2;
        {
            unsigned best = 0xFFFFFFFFu;
            if (can.tx_contending && can.tx_ready <= start) { best = arb_id(&can.tx_frame); winner = -1; }
            for (i = 0; i < can.n_pend; i++) {
                sim_ext_frame_t *e = &sim.ext[can.pend[i]];
                if (e->ready <= start && arb_id(&e->f) < best) { best = arb_id(&e->f); winner = i; }
            }
        }
        if (winner == -1) {
            can.who = -1;
            can.f = can.tx_frame;
        } else {
            can.who = can.pend[winner];
            can.f = sim.ext[can.who].f;
            memmove(&can.pend[winner], &can.pend[winner + 1], (can.n_pend - winner - 1) * sizeof(int));
            can.n_pend--;
            if (can.tx_contending && can.tx_ready <= start)
                hw.txb[0].b.TXLARB = 1;
        }
        can.busy = 1;
        can.start = start;
        can.end = start + (uint64_t)sim_frame_bits(&can.f) * BIT_NS;
    }
}

/* ------------------------------------------------------------------- timers */

static uint64_t next_tmr1, next_tmr0;
#define TMR1_PERIOD (50 * MS)
#define TMR0_PERIOD 2097152000ull

static void timers_until(uint64_t target)
{
    if (T1CONbits.TMR1ON) {
        if (!next_tmr1) next_tmr1 = sim.now + TMR1_PERIOD;
        while (next_tmr1 <= target) { hw.pir1.b.TMR1IF = 1; next_tmr1 += TMR1_PERIOD; }
    }
    if (!next_tmr0) next_tmr0 = sim.now + TMR0_PERIOD;
    while (next_tmr0 <= target) { hw.intcon.b.TMR0IF = 1; next_tmr0 += TMR0_PERIOD; }
}

/* ---------------------------------------------------------------------- TCP */

#define RXWIN       256
#define TXBUF       512
#define WIRE_NS     (150 * US)

typedef struct { int len; uint64_t ack_at; } seg_t;

static struct {
    uint8_t  rx[RXWIN]; int rx_head, rx_len;
    int      pc_win;                        /* free window the PC knows of */
    uint64_t win_update_at;
    uint8_t  pend[1 << 16]; int pend_head, pend_len;
    int      unsent;
    seg_t    seg[64]; int n_seg;
    uint64_t auto_tx_at;
    int      asap;
    int      segs_unacked;
    char     pcbuf[256]; int pclen;
    int      pc_inflight;                   /* Nagle: bytes the PC has sent and we have not acknowledged */
    uint64_t pc_ack_arrives;                /* when our acknowledgement reaches the PC */
    int      dev_unacked_segs;              /* segments received and not yet acknowledged */
    uint64_t dev_ack_at;                    /* delayed acknowledgement due */
} sock[SIM_SOCKETS];
extern const unsigned long long fw_delayed_ack_ns;
static int pcw_next;
static TCP_SOCKET next_handle;

static int inflight(int s) { int i, n = 0; for (i = 0; i < sock[s].n_seg; i++) n += sock[s].seg[i].len; return n; }

static void pc_receive(int s, const uint8_t *data, int len, uint64_t t)
{
    int i;
    for (i = 0; i < len; i++) {
        char c = (char)data[i];
        if (c == ':') sock[s].pclen = 0;
        if (sock[s].pclen < (int)sizeof sock[s].pcbuf) sock[s].pcbuf[sock[s].pclen++] = c;
        if (c == ';') {
            sim_frame_t f;
            if (sim_parse_gc(sock[s].pcbuf, sock[s].pclen, &f) && sim.n_to_pc < SIM_MAX_FRAMES) {
                sim.to_pc[sim.n_to_pc].t = t;
                sim.to_pc[sim.n_to_pc].f = f;
                sim.to_pc[sim.n_to_pc].socket = s;
                sim.n_to_pc++;
            }
            sock[s].pclen = 0;
        }
    }
}

static uint8_t txdata[SIM_SOCKETS][TXBUF];   /* unsent bytes, in order */

static void device_ack(int s)
{
    if (!sock[s].dev_unacked_segs) return;
    sock[s].dev_unacked_segs = 0;
    sock[s].dev_ack_at = 0;
    sock[s].pc_ack_arrives = sim.now + WIRE_NS;
}

static void send_segment(int s)
{
    seg_t *g;
    int i;
    if (!sock[s].unsent || sock[s].n_seg == 64) return;
    pc_receive(s, txdata[s], sock[s].unsent, sim.now + WIRE_NS);
    g = &sock[s].seg[sock[s].n_seg++];
    g->len = sock[s].unsent;
    sock[s].unsent = 0;
    sock[s].auto_tx_at = 0;
    sock[s].asap = 0;
    device_ack(s);                              /* our data carries the acknowledgement */
    if (!sim.pc_acks[s]) { g->ack_at = UINT64_MAX; return; }
    if (++sock[s].segs_unacked >= 2) {
        uint64_t a = sim.now + 2 * WIRE_NS;
        for (i = 0; i < sock[s].n_seg; i++) if (sock[s].seg[i].ack_at > a) sock[s].seg[i].ack_at = a;
        sock[s].segs_unacked = 0;
    } else
        g->ack_at = sim.now + WIRE_NS + sim.pc_ack_delay_ns;
    /* Every segment carries our window, so the PC learns of free space. */
    sock[s].pc_win = RXWIN - sock[s].rx_len;
}

static void tcp_until(uint64_t target)
{
    int s, i;
    while (pcw_next < sim.n_pcw && sim.pcw[pcw_next].t <= target) {
        sim_pc_write_t *w = &sim.pcw[pcw_next++];
        s = w->socket;
        if (sim.connected[s])
            for (i = 0; i < w->len; i++)
                sock[s].pend[(sock[s].pend_head + sock[s].pend_len++) & 0xFFFF] = (uint8_t)w->data[i];
    }
    for (s = 0; s < SIM_SOCKETS; s++) {
        int j = 0;
        for (i = 0; i < sock[s].n_seg; i++)
            if (sock[s].seg[i].ack_at > target) sock[s].seg[j++] = sock[s].seg[i];
        sock[s].n_seg = j;
        if (sock[s].win_update_at && sock[s].win_update_at <= target) {
            sock[s].pc_win = RXWIN - sock[s].rx_len;
            sock[s].win_update_at = 0;
        }
        if (sim.pc_nagle[s]) {
            int n = 0;
            if (sock[s].pc_ack_arrives && sock[s].pc_ack_arrives <= target) { sock[s].pc_inflight = 0; sock[s].pc_ack_arrives = 0; }
            if (sock[s].pc_inflight) continue;          /* Nagle: wait for the acknowledgement */
            while (sock[s].pend_len && sock[s].pc_win > 0 && sock[s].rx_len < RXWIN) {
                sock[s].rx[(sock[s].rx_head + sock[s].rx_len++) % RXWIN] = sock[s].pend[sock[s].pend_head];
                sock[s].pend_head = (sock[s].pend_head + 1) & 0xFFFF;
                sock[s].pend_len--;
                sock[s].pc_win--;
                n++;
            }
            if (n) { sock[s].pc_inflight = n; sock[s].dev_unacked_segs++; }
            continue;
        }
        while (sock[s].pend_len && sock[s].pc_win > 0 && sock[s].rx_len < RXWIN) {
            sock[s].rx[(sock[s].rx_head + sock[s].rx_len++) % RXWIN] = sock[s].pend[sock[s].pend_head];
            sock[s].pend_head = (sock[s].pend_head + 1) & 0xFFFF;
            sock[s].pend_len--;
            sock[s].pc_win--;
        }
    }
}

/* ------------------------------------------------------------ the model step */

static void advance(uint64_t ns)
{
    uint64_t target = sim.now + ns;
    in_model = 1;
    can_sync_registers();
    timers_until(target);
    can_until(target);
    tcp_until(target);
    sim.now = target;
    can_sync_registers();
    in_model = 0;
}

static int low_pending(void)
{
    return (hw.intcon.b.TMR0IE && hw.intcon.b.TMR0IF) || (hw.pie3.v & hw.pir3.v);
}

static void take_interrupts(void)
{
    if (!in_high && hw.intcon.b.GIEH && PIE1bits.TMR1IE && hw.pir1.b.TMR1IF) {
        in_high = 1; isr_high(); in_high = 0;
    }
    if (!in_low && !in_high && hw.intcon.b.GIEH && hw.intcon.b.GIEL && low_pending()) {
        in_low = 1; isr_low(); in_low = 0;
    }
}

static void spend(uint64_t ns)
{
    advance(ns);
    wdt_check();
    if (sim.now >= sim.end_ns) longjmp(run_end, 1);
    take_interrupts();
}

void hw_access(void)
{
    if (in_model) return;
    access_seq++;
    if (sim.on_access) sim.on_access(in_low, in_high);
    if (sim.fuzz_per_mille && rnd() % 1000 < sim.fuzz_per_mille) {
        /* Timing fuzz: the frame on the bus ends right here, whatever code is running,
           or the tick timer's interrupt arrives here. */
        if (rnd() & 1) { if (can.busy) can.end = sim.now; }
        else hw.intcon.b.TMR0IF = 1;
    }
    spend(rnd_range(250, 1500));
}

void hw_clrwdt(void) { last_clrwdt = sim.now; }

/* Watchdog: about 4.1 s (4 ms nominal period, postscaler 1024) once software enables it. */
static void wdt_check(void)
{
    if (!(hw.wdtcon & 1)) { last_clrwdt = sim.now; return; }
    if (sim.now - last_clrwdt > 4096 * MS) { sim.watchdog_resets++; longjmp(run_end, 3); }
}

/* Data EEPROM write: EECON2 = 0x55, EECON2 = 0xAA, then WR, with nothing between (data sheet 7.4). */
hw_byte *hw_eecon2(void)
{
    hw_access();
    eecon2_seq[0] = eecon2_seq[1];
    eecon2_seq[1] = access_seq;
    return &hw.eecon2;
}
hw_eecon1_t *hw_eecon1(void)
{
    hw_access();
    wr_unlocked = eecon2_seq[1] == access_seq - 1 && eecon2_seq[0] == access_seq - 2;
    return &hw.eecon1;
}
void hw_reset(void) { sim.resets++; longjmp(run_end, 2); }

hw_byte *hw_eedata(void)
{
    hw_access();
    if (hw.eecon1.b.RD) { hw.eedata = eeprom[hw.eeadr]; hw.eecon1.b.RD = 0; }
    return &hw.eedata;
}

static void delay_cycles(unsigned long n) { spend((uint64_t)n * 125); }
void Delay10TCYx(unsigned char n) { delay_cycles(10ul * (n ? n : 256)); }
void Delay100TCYx(unsigned char n) { delay_cycles(100ul * (n ? n : 256)); }
void Delay1KTCYx(unsigned char n) { delay_cycles(1000ul * (n ? n : 256)); }
void Delay10KTCYx(unsigned char n) { delay_cycles(10000ul * (n ? n : 256)); }

hw_pir2_t *hw_pir2(void)
{
    hw_access();
    if (hw.eecon1.b.WR) {
        if (!wr_unlocked) {                     /* the write does not begin: EEIF never comes */
            hw.eecon1.b.WR = 0;
            sim.eeprom_write_failures++;
        } else {
            if (hw.eecon1.b.WREN) {
                eeprom[hw.eeadr] = hw.eedata;
                sim.ee_writes++;
            }
            hw.eecon1.b.WR = 0;
            hw.pir2.b.EEIF = 1;
            advance(4 * MS);
        }
    }
    return &hw.pir2;
}

/* ------------------------------------------------------- TCP/IP stack stubs */

#define API_COST() spend(rnd_range(5 * US, 20 * US))

TCP_SOCKET TCPOpen(DWORD h, BYTE t, WORD port, BYTE purpose)
{
    (void)h; (void)t; (void)port; (void)purpose;
    return next_handle < SIM_SOCKETS ? next_handle++ : INVALID_SOCKET;
}
BOOL TCPIsConnected(TCP_SOCKET s) { API_COST(); return s < SIM_SOCKETS && sim.connected[s]; }
WORD TCPIsGetReady(TCP_SOCKET s) { API_COST(); return s < SIM_SOCKETS ? (WORD)sock[s].rx_len : 0; }

static void after_read(int s)
{
    if (sock[s].rx_len == 0) {
        if (!sock[s].win_update_at || sock[s].win_update_at > sim.now + 2 * WIRE_NS)
            sock[s].win_update_at = sim.now + 2 * WIRE_NS;
    } else if (!sock[s].win_update_at)
        sock[s].win_update_at = sim.now + 200 * MS;           /* TCP_WINDOW_UPDATE_TIMEOUT_VAL */
}

BOOL TCPGet(TCP_SOCKET s, BYTE *b)
{
    API_COST();
    if (s >= SIM_SOCKETS || !sock[s].rx_len) return 0;
    *b = sock[s].rx[sock[s].rx_head];
    sock[s].rx_head = (sock[s].rx_head + 1) % RXWIN;
    sock[s].rx_len--;
    after_read(s);
    return 1;
}
WORD TCPGetArray(TCP_SOCKET s, BYTE *buf, WORD len)
{
    WORD n = 0;
    while (n < len && TCPGet(s, buf ? &buf[n] : &(BYTE){0})) n++;
    return n;
}
void TCPDiscard(TCP_SOCKET s)
{
    API_COST();
    if (s >= SIM_SOCKETS) return;
    sock[s].rx_len = 0;
    after_read(s);
}
WORD TCPIsPutReady(TCP_SOCKET s)
{
    API_COST();
    if (s >= SIM_SOCKETS || !sim.connected[s]) return 0;
    return (WORD)(TXBUF - sock[s].unsent - inflight(s));
}
WORD TCPPutArray(TCP_SOCKET s, BYTE *data, WORD len)
{
    int room;
    API_COST();
    if (s >= SIM_SOCKETS) return 0;
    room = TXBUF - sock[s].unsent - inflight(s);
    if (len > room) len = (WORD)room;
    memcpy(&txdata[s][sock[s].unsent], data, len);
    sock[s].unsent += len;
    if (!sock[s].auto_tx_at) sock[s].auto_tx_at = sim.now + 40 * MS;   /* TCP_AUTO_TRANSMIT_TIMEOUT_VAL */
    if (TXBUF - sock[s].unsent - inflight(s) <= TXBUF / 2) sock[s].asap = 1;
    return len;
}
void TCPFlush(TCP_SOCKET s) { API_COST(); if (s < SIM_SOCKETS) send_segment(s); }
void TCPDisconnect(TCP_SOCKET s) { API_COST(); if (s < SIM_SOCKETS) sim.connected[s] = 0; }
void TCPClose(TCP_SOCKET s) { (void)s; }
void TickInit(void) { hw.intcon.b.TMR0IE = 1; }        /* as Tick.c: timer 0 interrupt, low priority */
void TickUpdate(void) { hw.intcon.b.TMR0IF = 0; }
DWORD TickGet(void) { return (DWORD)(sim.now / 32000); }
void StackInit(void) { }
void StackApplications(void) { API_COST(); }
void AnnounceIP(void) { }
void DHCPInit(BYTE i) { (void)i; }
void DHCPDisable(BYTE i) { (void)i; }

void StackTask(void)
{
    int s, k, busy = 0, steps = (int)rnd_range(3, 8);
    for (s = 0; s < SIM_SOCKETS; s++)
        if (sock[s].unsent || sock[s].n_seg || sock[s].pend_len) busy = 1;
    if (busy) steps += (int)rnd_range(10, 30);      /* a packet to handle: 0.5 to 1.5 ms more */
    for (k = 0; k < steps; k++)
        spend(rnd_range(40 * US, 60 * US));          /* 150 to 400 us when idle, interruptible */
    for (s = 0; s < SIM_SOCKETS; s++) {
        if (sock[s].unsent && (sock[s].asap || (sock[s].auto_tx_at && sock[s].auto_tx_at <= sim.now)))
            send_segment(s);
        if (sock[s].dev_unacked_segs >= 2)
            device_ack(s);                              /* a second segment: acknowledge at once */
        else if (sock[s].dev_unacked_segs) {
            if (!sock[s].dev_ack_at) sock[s].dev_ack_at = sim.now + fw_delayed_ack_ns;
            if (sock[s].dev_ack_at <= sim.now) device_ack(s);
        }
    }
}

/* -------------------------------------------------------------------- utils */

static int hexv(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

/* Parses ":S|X|Y hdr N|R data ;" as the PC would. Returns the type letter, or 0. */
int sim_parse_gc(const char *s, int len, sim_frame_t *f)
{
    int hdr, i, n, p;
    memset(f, 0, sizeof *f);
    if (len < 4 || s[0] != ':' || s[len - 1] != ';') return 0;
    hdr = s[1] == 'X' ? 8 : 4;
    if (s[1] != 'S' && s[1] != 'X' && s[1] != 'Y') return 0;
    if (len < 2 + hdr + 2) return 0;
    {
        uint8_t h[4] = {0};
        for (i = 0; i < hdr / 2; i++) {
            int a = hexv(s[2 + 2 * i]), b = hexv(s[3 + 2 * i]);
            if (a < 0 || b < 0) return 0;
            h[i] = (uint8_t)(a << 4 | b);
        }
        f->sidh = h[0]; f->sidl = h[1]; f->eidh = h[2]; f->eidl = h[3];
    }
    p = 2 + hdr;
    if (s[p] == 'R') f->dlc = 0x40; else if (s[p] != 'N') return 0;
    p++;
    n = (len - 1 - p) / 2;
    if (n > 8 || (len - 1 - p) % 2) return 0;
    for (i = 0; i < n; i++) {
        int a = hexv(s[p + 2 * i]), b = hexv(s[p + 2 * i + 1]);
        if (a < 0 || b < 0) return 0;
        f->d[i] = (uint8_t)(a << 4 | b);
    }
    f->dlc |= (uint8_t)n;
    return s[1];
}

/* ---------------------------------------------------------------------- run */

/* ------------------------------------------------- for the directed tests */

int sim_in_low_isr(void) { return in_low; }

/* The stack's tick timer (timer 0) interrupt arrives now. */
void sim_tick_interrupt(void) { hw.intcon.b.TMR0IF = 1; }

/* Puts a frame from another node straight into the receive FIFO, as at the end of its frame. */
void sim_inject_rx(const sim_frame_t *f)
{
    sim_ext_frame_t *e = &sim.ext[sim.n_ext];
    e->ready = sim.now; e->f = *f;
    in_model = 1;
    can_sync_registers();
    can.who = sim.n_ext++;
    can.ext_next = sim.n_ext;           /* delivered here, so not sent again by the bus model */
    can.start = can.end = sim.now;
    can_complete();
    can_sync_registers();
    in_model = 0;
}

/* Ends our frame now: the one on the bus, or the one waiting for it. */
void sim_force_tx_end(void)
{
    in_model = 1;
    can_sync_registers();
    if (can.busy && can.who == -1) can.end = sim.now;
    else if (can.tx_contending && !can.busy) {
        can.busy = 1; can.who = -1; can.f = can.tx_frame; can.start = can.end = sim.now;
    }
    in_model = 0;
}

/* Lets simulated time pass with the firmware idle (interrupts are taken). */
void sim_idle(uint64_t ns)
{
    uint64_t end = sim.now + ns;
    while (sim.now < end) hw_access();
}

void sim_reset(void)
{
    int i;
    rng = sim.seed * 2654435761u + 12345u;
    if (!rng) rng = 1;
    memset(&hw, 0, sizeof hw);
    memset(&can, 0, sizeof can);
    memset(sock, 0, sizeof sock);
    for (i = 0; i < SIM_SOCKETS; i++) sock[i].pc_win = RXWIN;
    memset(eeprom, 0xFF, sizeof eeprom);
    PORTAbits.RA0 = 1; PORTAbits.RA2 = 1;   /* push button not pressed */
    PORTCbits.RC6 = 0;                      /* JP2 fitted */
    TXBIEbits.TXB0IE = 0;
    pcw_next = 0;
    next_handle = 1;                        /* so HCB[i].socket == i for the server sockets */
    next_tmr1 = next_tmr0 = 0;
    in_low = in_high = in_model = 0;
    access_seq = eecon2_seq[0] = eecon2_seq[1] = wr_unlocked = 0;
    last_clrwdt = 0;
}

void sim_run(void)
{
    sim_reset();
    if (setjmp(run_end) == 0)
        firmware_main();
}

/* For the directed tests: a byte of the model's EEPROM. */
uint8_t sim_eeprom(uint8_t addr) { return eeprom[addr]; }
