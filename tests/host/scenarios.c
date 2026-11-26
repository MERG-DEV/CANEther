/*
 * Host test environment for the CANEther firmware: the test scenarios.
 *
 * Each scenario sets up traffic from other CAN nodes and from PCs, runs the
 * firmware's own main() on the model in sim.c, then checks every frame:
 *
 *   CAN to PC   each frame another node sends reaches each connected PC once,
 *               in bus order; latency from end of frame on the bus to arrival
 *               at the PC.
 *   PC to CAN   each frame a PC writes reaches the bus once, in the order
 *               written; latency from the PC's write to end of frame on the bus.
 *   echo        no frame that came from the bus is sent back onto it.
 *
 * Test frames are ACON (0x90) with the source in the node number bytes and a
 * sequence number in the event number bytes: node number 0x01nn for CAN node
 * nn, 0x02ss for the PC on connection ss.
 *
 * Every run is a separate process (the firmware keeps its state in globals),
 * so a crash, or an address sanitizer report, fails that run only.
 *
 * Usage: scenarios [scenario|all] [first_seed] [seeds]
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>
#include "sim.h"

#define SEC 1000000000ull

static sim_ext_frame_t ext[SIM_MAX_FRAMES];
static sim_pc_write_t  pcw[SIM_MAX_FRAMES];
static int ext_seq, pc_seq[SIM_SOCKETS];
static uint32_t rs;

static uint32_t r32(void) { rs ^= rs << 13; rs ^= rs >> 17; rs ^= rs << 5; return rs; }
static uint64_t rr(uint64_t lo, uint64_t hi) { return lo + r32() % (hi - lo + 1); }
static uint64_t rexp(uint64_t mean) { double u = (r32() % 1000000 + 1) / 1000001.0; double x = -(double)mean * __builtin_log(u); return (uint64_t)x; }

static void add_ext(uint64_t t, int node, int dlc)
{
    sim_ext_frame_t *e = &ext[sim.n_ext++];
    memset(e, 0, sizeof *e);
    e->ready = t;
    e->f.sidh = (uint8_t)(0xB0 | (node >> 3));
    e->f.sidl = (uint8_t)(node << 5);
    e->f.dlc = (uint8_t)dlc;
    e->f.d[0] = 0x90; e->f.d[1] = 0x01; e->f.d[2] = (uint8_t)node;
    e->f.d[3] = (uint8_t)(ext_seq >> 8); e->f.d[4] = (uint8_t)ext_seq;
    e->f.d[5] = 0xA5; e->f.d[6] = 0x5A; e->f.d[7] = 0xC3;
    ext_seq++;
}

static int cmp_ext(const void *a, const void *b)
{
    const sim_ext_frame_t *x = a, *y = b;
    return x->ready < y->ready ? -1 : x->ready > y->ready;
}
static int cmp_pcw(const void *a, const void *b)
{
    const sim_pc_write_t *x = a, *y = b;
    return x->t < y->t ? -1 : x->t > y->t;
}

/* The PC on connection s writes one ACON frame, optionally split in two segments. */
static void add_pc_text(uint64_t t, int s, const char *txt, int seq, int split, uint64_t gap)
{
    int len = (int)strlen(txt);
    if (split > 0 && split < len) {
        sim_pc_write_t *a = &pcw[sim.n_pcw++], *b = &pcw[sim.n_pcw++];
        a->t = t; a->socket = s; a->len = split; memcpy(a->data, txt, split); a->frame_seq = -1;
        b->t = t + gap; b->socket = s; b->len = len - split; memcpy(b->data, txt + split, len - split); b->frame_seq = seq;
    } else {
        sim_pc_write_t *a = &pcw[sim.n_pcw++];
        a->t = t; a->socket = s; a->len = len; memcpy(a->data, txt, len); a->frame_seq = seq;
    }
}
static void add_pc(uint64_t t, int s, int split, uint64_t gap, int lower)
{
    char txt[64];
    int seq = pc_seq[s]++;
    sprintf(txt, lower ? ":Sbfa0N9002%02x%04x;" : ":SBFA0N9002%02X%04X;", s, seq);
    add_pc_text(t, s, txt, seq, split, gap);
}

/* ------------------------------------------------------------------ checks */

typedef struct {
    int expected, delivered, missing, dup, order, garbage;
    double lat_max, lat_mean;
    double idle_max;        /* PC to CAN: longest the bus sat idle while a frame waited */
} dir_t;

/* Time the bus was idle between a and b, counting only gaps longer than 100 us
   (so the 3-bit interframe space and the turnaround between frames don't count). */
typedef struct { uint64_t s, e; } span_t;
static int cmp_span(const void *x, const void *y)
{
    const span_t *p = x, *q = y;
    return p->s < q->s ? -1 : p->s > q->s;
}
static uint64_t bus_idle(uint64_t a, uint64_t b)
{
    static span_t *sp; static int n;
    uint64_t idle = 0, cur;
    int i, lo, hi;
    if (b <= a) return 0;
    if (!sp) {
        sp = malloc(sizeof *sp * (sim.n_can_in + sim.n_can_out + 1));
        for (i = 0; i < sim.n_can_in; i++) { sp[n].s = sim.can_in[i].start; sp[n++].e = sim.can_in[i].t; }
        for (i = 0; i < sim.n_can_out; i++) { sp[n].s = sim.can_out[i].start; sp[n++].e = sim.can_out[i].t; }
        qsort(sp, n, sizeof *sp, cmp_span);
    }
    lo = 0; hi = n;                         /* first span starting at or after a, less one */
    while (lo < hi) { int m = (lo + hi) / 2; if (sp[m].s < a) lo = m + 1; else hi = m; }
    i = lo > 0 ? lo - 1 : 0;
    cur = a;
    for (; i < n && sp[i].s < b; i++) {
        if (sp[i].e <= cur) continue;
        if (sp[i].s > cur && sp[i].s - cur > 100 * US) idle += sp[i].s - cur;
        if (sp[i].e > cur) cur = sp[i].e;
    }
    if (b > cur && b - cur > 100 * US) idle += b - cur;
    return idle;
}

static int is_acon(const sim_frame_t *f, int src) { return (f->dlc & 0x0F) >= 5 && f->d[0] == 0x90 && f->d[1] == src; }
static int seq_of(const sim_frame_t *f) { return f->d[3] << 8 | f->d[4]; }
static int dlen(const sim_frame_t *f) { int n = f->dlc & 0x0F; return n > 8 ? 8 : n; }
/* Same header and data, as the PC sees it: a DLC of 9 to 15 means 8 bytes. */
static int same_frame(const sim_frame_t *pc, const sim_frame_t *bus)
{
    return pc->sidh == bus->sidh && (pc->sidl & 0xEB) == (bus->sidl & 0xEB) && dlen(pc) == dlen(bus)
        && !memcmp(pc->d, bus->d, dlen(bus));
}

static dir_t check_can_to_pc(int s, uint64_t cutoff)
{
    static int idx_of_seq[65536];
    static char seen[65536];
    dir_t r; int i, last = -1, nlat = 0; double sum = 0;
    memset(&r, 0, sizeof r);
    memset(seen, 0, sizeof seen);
    for (i = 0; i < 65536; i++) idx_of_seq[i] = -1;
    for (i = 0; i < sim.n_can_in; i++)
        if (is_acon(&sim.can_in[i].f, 1)) idx_of_seq[seq_of(&sim.can_in[i].f)] = i;
    for (i = 0; i < sim.n_to_pc; i++) {
        sim_event_t *e = &sim.to_pc[i];
        int q, k; double lat;
        if (e->socket != s || !is_acon(&e->f, 1)) continue;
        q = seq_of(&e->f); k = idx_of_seq[q];
        if (k < 0 || !same_frame(&e->f, &sim.can_in[k].f)) { r.garbage++; continue; }
        if (seen[q]) { r.dup++; continue; }
        seen[q] = 1;
        r.delivered++;
        if (k < last) r.order++;
        last = k;
        lat = (double)(e->t - sim.can_in[k].t) / 1e6;
        if (lat > r.lat_max) r.lat_max = lat;
        sum += lat; nlat++;
    }
    for (i = 0; i < sim.n_can_in; i++) {
        sim_event_t *e = &sim.can_in[i];
        if (!is_acon(&e->f, 1) || e->t > cutoff || e->socket == -2) continue;
        r.expected++;
        if (!seen[seq_of(&e->f)]) r.missing++;
    }
    r.delivered = r.expected - r.missing;
    r.lat_mean = nlat ? sum / nlat : 0;
    return r;
}

static dir_t check_pc_to_can(int s, uint64_t cutoff, uint64_t from)
{
    static uint64_t wtime[65536];
    static char seen[65536], written[65536];
    dir_t r; int i, last = -1, nlat = 0; double sum = 0;
    memset(&r, 0, sizeof r);
    memset(seen, 0, sizeof seen); memset(written, 0, sizeof written);
    for (i = 0; i < sim.n_pcw; i++)
        if (pcw[i].socket == s && pcw[i].frame_seq >= 0) { wtime[pcw[i].frame_seq] = pcw[i].t; written[pcw[i].frame_seq] = 1; }
    for (i = 0; i < sim.n_can_out; i++) {
        sim_event_t *e = &sim.can_out[i];
        int q; double lat;
        if (!is_acon(&e->f, 2) || e->f.d[2] != s) continue;
        q = seq_of(&e->f);
        if (!written[q] || e->f.d[5] || e->f.d[6] || e->f.d[7] || (e->f.dlc & 0x0F) != 5) { r.garbage++; continue; }
        if (seen[q]) { r.dup++; continue; }
        seen[q] = 1;
        r.delivered++;
        if (q < last) r.order++;
        last = q;
        if (wtime[q] < from) continue;
        lat = (double)(e->t - wtime[q]) / 1e6;
        if (lat > r.lat_max) r.lat_max = lat;
        sum += lat; nlat++;
        {   /* allow 3 ms for the firmware to read and queue it */
            double idle = (double)bus_idle(wtime[q] + 3 * MS, e->start) / 1e6;
            if (idle > r.idle_max) r.idle_max = idle;
            if (getenv("SIM_DEBUG_IDLE") && idle > 5.0)
                printf("    debug: seq %d written %.3f ms, on bus %.3f-%.3f ms, idle %.3f ms\n", q,
                       wtime[q] / 1e6, e->start / 1e6, e->t / 1e6, idle);
        }
    }
    for (i = 0; i < 65536; i++)
        if (written[i] && wtime[i] <= cutoff && wtime[i] >= from) { r.expected++; if (!seen[i]) r.missing++; }
    r.delivered = r.expected - r.missing;
    r.lat_mean = nlat ? sum / nlat : 0;
    return r;
}

static int echoes(void)
{
    int i, n = 0;
    for (i = 0; i < sim.n_can_out; i++) if (is_acon(&sim.can_out[i].f, 1)) n++;
    return n;
}

/* The firmware's own counters, where the version under test has them. */
extern unsigned char txTimeoutCount __attribute__((weak)), txDropCount __attribute__((weak)),
       ethDropCount __attribute__((weak)), gcRejectCount __attribute__((weak)), maxCanTxFifo __attribute__((weak));
static void show_counters(void)
{
    printf("    firmware counters:");
    if (&txTimeoutCount) printf(" txTimeoutCount %u", txTimeoutCount);
    if (&txDropCount)    printf(" txDropCount %u", txDropCount);
    if (&ethDropCount)   printf(" ethDropCount %u", ethDropCount);
    if (&gcRejectCount)  printf(" gcRejectCount %u", gcRejectCount);
    if (&maxCanTxFifo)   printf(" maxCanTxFifo %u", maxCanTxFifo);
    printf("\n");
}

static int fails;
static char why[512];
static void need(int ok, const char *fmt, ...)
{
    if (!ok) {
        va_list ap; char b[160];
        va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
        if (strlen(why) + strlen(b) + 3 < sizeof why) { if (*why) strcat(why, "; "); strcat(why, b); }
        fails++;
    }
}
static void need_clean(const char *name, dir_t d)
{
    need(d.missing == 0, "%s: %d of %d missing", name, d.missing, d.expected);
    need(d.dup == 0, "%s: %d duplicated", name, d.dup);
    need(d.order == 0, "%s: %d out of order", name, d.order);
    need(d.garbage == 0, "%s: %d corrupt", name, d.garbage);
}
static void show(const char *name, dir_t d)
{
    printf("    %-14s %5d/%-5d delivered  missing %d  dup %d  order %d  corrupt %d  latency mean %.2f ms  max %.2f ms",
           name, d.delivered, d.expected, d.missing, d.dup, d.order, d.garbage, d.lat_mean, d.lat_max);
    if (d.idle_max > 0) printf("  bus idle while waiting %.2f ms", d.idle_max);
    printf("\n");
}
/* A frame from the PC must not wait while the bus sits idle (beyond 3 ms to read it, and 5 ms slack). */
static void need_no_hold(const char *name, dir_t d)
{
    need(d.idle_max < 5.0, "%s: a frame waited %.1f ms with the bus idle", name, d.idle_max);
}

/* --------------------------------------------------------------- scenarios */

static void base(uint64_t end)
{
    memset(&sim, 0, sizeof sim);
    sim.ext = ext; sim.pcw = pcw;
    sim.end_ns = end;
    sim.pc_ack_delay_ns = 40 * MS;
    sim.connected[1] = 1; sim.pc_acks[1] = 1;
    ext_seq = 0; memset(pc_seq, 0, sizeof pc_seq);
}
static void finish_setup(void)
{
    qsort(ext, sim.n_ext, sizeof ext[0], cmp_ext);
    qsort(pcw, sim.n_pcw, sizeof pcw[0], cmp_pcw);
}

/* L1: sensor reports on a quiet bus reach the PC at once. */
static void setup_quiet(void)
{
    uint64_t t;
    base(10 * SEC);
    for (t = SEC / 2; t < sim.end_ns; t += rr(50 * MS, 150 * MS)) add_ext(t, 5, 5);
    finish_setup();
}
static void check_quiet(void)
{
    dir_t a = check_can_to_pc(1, sim.end_ns - 200 * MS);
    show("CAN to PC", a); need_clean("CAN to PC", a);
    need(a.lat_max < 5.0, "CAN to PC max latency %.1f ms (limit 5)", a.lat_max);
}

/* L3, L5, R1: both directions busy; the interrupt races get their chance. */
static void setup_busy(void)
{
    uint64_t t; int node;
    base(20 * SEC);
    for (node = 1; node <= 4; node++)
        for (t = SEC / 2 + rr(0, 5 * MS); t < sim.end_ns; t += 2 * MS + rexp(4 * MS))
            add_ext(t, node, 5);
    for (t = SEC; t < sim.end_ns - SEC; t += rr(20 * MS, 200 * MS)) {
        int k, n = (int)rr(1, 12);
        for (k = 0; k < n; k++) add_pc(t + k * 200 * US, 1, 0, 0, 0);
    }
    finish_setup();
}
static void check_busy(void)
{
    dir_t a = check_can_to_pc(1, sim.end_ns - 300 * MS);
    dir_t b = check_pc_to_can(1, sim.end_ns - SEC, 0);
    show("CAN to PC", a); show("PC to CAN", b);
    printf("    hardware RX overflows %d, transmit aborts %d\n", sim.hw_rx_overflows, sim.tx_aborts);
    need_clean("CAN to PC", a); need_clean("PC to CAN", b); need_no_hold("PC to CAN", b);
    need(a.lat_max < 20.0, "CAN to PC max latency %.1f ms (limit 20)", a.lat_max);
    need(b.lat_max < 100.0, "PC to CAN max latency %.1f ms (limit 100)", b.lat_max);
    need(sim.tx_aborts == 0, "%d frames aborted on a working bus", sim.tx_aborts);
}

/* L3, L5, R1 again, harder: bursts both ways, with timing fuzz to reach the rare interleavings. */
static void setup_race(void)
{
    uint64_t t; int node, k;
    base(30 * SEC);
    sim.fuzz_per_mille = 20;
    for (node = 1; node <= 4; node++)
        for (t = SEC / 2 + rr(0, 5 * MS); t < sim.end_ns; t += rr(10 * MS, 30 * MS)) {
            int n = (int)rr(1, 6);
            for (k = 0; k < n; k++) add_ext(t, node, 5);        /* a burst, all ready at once */
        }
    for (t = SEC; t < sim.end_ns - SEC; t += rr(20 * MS, 40 * MS)) {
        int n = (int)rr(1, 10);
        for (k = 0; k < n; k++) add_pc(t + k * 100 * US, 1, 0, 0, 0);
    }
    finish_setup();
}
static void check_race(void)
{
    dir_t a = check_can_to_pc(1, sim.end_ns - 300 * MS);
    dir_t b = check_pc_to_can(1, sim.end_ns - 2 * SEC, 0);
    show("CAN to PC", a); show("PC to CAN", b);
    printf("    hardware RX overflows %d, transmit aborts %d\n", sim.hw_rx_overflows, sim.tx_aborts);
    need_clean("CAN to PC", a); need_clean("PC to CAN", b); need_no_hold("PC to CAN", b);
    need(sim.tx_aborts == 0, "%d frames aborted on a working bus", sim.tx_aborts);
}

/* L5: the receive path under a heavy, steady bus load, with timing fuzz. */
static void setup_rxrace(void)
{
    uint64_t t; int node;
    base(20 * SEC);
    sim.fuzz_per_mille = 50;
    for (node = 1; node <= 3; node++)
        for (t = SEC / 2 + node * 100 * US; t < sim.end_ns; t += rr(2 * MS, 4 * MS))
            add_ext(t, node, 5);
    finish_setup();
}
static void check_rxrace(void)
{
    dir_t a = check_can_to_pc(1, sim.end_ns - 300 * MS);
    show("CAN to PC", a);
    printf("    hardware RX overflows %d\n", sim.hw_rx_overflows);
    need_clean("CAN to PC", a);
}

/* L4: a long burst from the PC to a busy bus must not stall sensor reports. */
static void setup_burst(void)
{
    uint64_t t; int node, k, b;
    base(12 * SEC);
    for (node = 1; node <= 2; node++)
        for (t = SEC / 2; t < sim.end_ns; t += rr(1500 * US, 2600 * US))
            add_ext(t, node, 5);
    for (t = SEC / 2; t < sim.end_ns; t += 30 * MS) add_ext(t, 9, 5);
    for (b = 0; b < 4; b++)
        for (k = 0; k < 150; k++) add_pc(SEC + b * 2 * SEC + k * 50 * US, 1, 0, 0, 0);
    finish_setup();
}
static void check_burst(void)
{
    dir_t a = check_can_to_pc(1, sim.end_ns - 300 * MS);
    dir_t b = check_pc_to_can(1, sim.end_ns - 2 * SEC, 0);
    show("CAN to PC", a); show("PC to CAN", b);
    printf("    hardware RX overflows %d\n", sim.hw_rx_overflows);
    need_clean("CAN to PC", a); need_clean("PC to CAN", b); need_no_hold("PC to CAN", b);
    need(a.lat_max < 20.0, "CAN to PC max latency %.1f ms during bursts (limit 20)", a.lat_max);
    need(sim.hw_rx_overflows == 0, "%d frames lost to receive FIFO overflow", sim.hw_rx_overflows);
}

/* L2: JMRI "Send frames: Grouped": the PC waits for our acknowledgement between frames of a burst. */
static void setup_grouped(void)
{
    uint64_t t; int k;
    base(10 * SEC);
    sim.pc_nagle[1] = 1;
    for (t = SEC; t < sim.end_ns - SEC; t += rr(200 * MS, 400 * MS))
        for (k = 0; k < 4; k++) add_pc(t + k * 100 * US, 1, 0, 0, 0);
    finish_setup();
}
static void check_grouped(void)
{
    extern const unsigned long long fw_delayed_ack_ns;
    dir_t b = check_pc_to_can(1, sim.end_ns - SEC, 0);
    show("PC to CAN", b);
    printf("    firmware's delayed acknowledgement: %.1f ms\n", fw_delayed_ack_ns / 1e6);
    need_clean("PC to CAN", b);
    need(b.lat_max < 20.0, "PC to CAN max latency %.1f ms with Grouped (limit 20)", b.lat_max);
}

/* R1, R8: no acknowledgement on the bus, then the bus recovers. */
static void setup_deadbus(void)
{
    uint64_t t;
    base(6 * SEC);
    sim.bus_dead_until_ns = 3 * SEC;
    add_pc(1000 * MS, 1, 0, 0, 0);
    add_pc(1001 * MS, 1, 0, 0, 0);
    add_pc(1002 * MS, 1, 0, 0, 0);
    add_pc(3500 * MS, 1, 0, 0, 0);
    add_pc(4500 * MS, 1, 0, 0, 0);
    for (t = 3200 * MS; t < sim.end_ns; t += 100 * MS) add_ext(t, 5, 5);
    finish_setup();
}
static void check_deadbus(void)
{
    dir_t a = check_can_to_pc(1, sim.end_ns - 300 * MS);
    dir_t b = check_pc_to_can(1, sim.end_ns - 500 * MS, 3 * SEC);
    dir_t early = check_pc_to_can(1, 3 * SEC, 0);
    dir_t all = check_pc_to_can(1, sim.end_ns, 0);
    show("CAN to PC", a); show("PC to CAN", b);
    printf("    written while the bus was dead: %d of 3 sent after it recovered, transmit aborts %d\n",
           early.delivered, sim.tx_aborts);
    need_clean("CAN to PC", a); need_clean("PC to CAN after recovery", b);
    need(all.dup == 0 && all.order == 0, "frames duplicated or reordered around the dead bus");
    need(b.lat_max < 50.0, "after recovery, PC to CAN max latency %.1f ms (limit 50)", b.lat_max);
    {
        int i;
        for (i = 0; i < sim.n_can_out; i++)
            if (is_acon(&sim.can_out[i].f, 2) && seq_of(&sim.can_out[i].f) < 3 && sim.can_out[i].t > 3 * SEC + 50 * MS)
                need(0, "frame %d, written while the bus was dead, held until %.0f ms, after the bus recovered at 3000 ms",
                     seq_of(&sim.can_out[i].f), (double)sim.can_out[i].t / 1e6);
    }
    need(a.lat_max < 5.0, "CAN to PC max latency %.1f ms (limit 5)", a.lat_max);
}

/* R2: frames split across two TCP segments. */
static void setup_split(void)
{
    int k;
    base(4 * SEC);
    for (k = 0; k < 60; k++) add_pc(SEC / 2 + k * 40 * MS, 1, 1 + k % 21, rr(1 * MS, 3 * MS), 0);
    finish_setup();
}
static void check_split(void)
{
    dir_t b = check_pc_to_can(1, sim.end_ns - 300 * MS, 0);
    show("PC to CAN", b); need_clean("PC to CAN", b); need_no_hold("PC to CAN", b);
}

/* R3 and hex parsing: a malformed frame, then good frames in the same segment; lower-case hex. */
static void setup_malformed(void)
{
    int k;
    base(4 * SEC);
    for (k = 0; k < 20; k++) {
        char txt[64], one[32];
        int seq = pc_seq[1]++, j;
        uint64_t t = SEC / 2 + k * 50 * MS;
        add_pc_text(t, 1, k % 2 ? ":SBFA0N90ZZ0102030405060708090A0B0C0D0E0F"  /* overlong, no ';' */
                                : ":SBFA0N90ZZ01", -1, 0, 0);                    /* short, no ';' */
        sprintf(txt, ":SBFA0N9002%02X%04X;", 1, seq);
        for (j = 0; j < 2; j++) { sprintf(one, ":SBFA0N9002%02X%04X;", 1, pc_seq[1]++); strcat(txt, one); }
        add_pc_text(t + 1, 1, txt, seq + 2, 0, 0);
        for (j = 0; j < 2; j++) {           /* empty entries that record the write time of the first two */
            sim_pc_write_t *w = &pcw[sim.n_pcw++];
            memset(w, 0, sizeof *w); w->t = t + 1; w->socket = 1; w->frame_seq = seq + j;
        }
    }
    for (k = 0; k < 10; k++) add_pc(SEC + SEC / 2 + k * 20 * MS, 1, 0, 0, 1);
    finish_setup();
}
static void check_malformed(void)
{
    dir_t b = check_pc_to_can(1, sim.end_ns - 300 * MS, 0);
    show("PC to CAN", b); need_clean("PC to CAN", b); need_no_hold("PC to CAN", b);
}

/* R7: frames whose DLC field is 9 to 15, which CAN allows (it means 8 bytes). */
static void setup_longdlc(void)
{
    int k;
    base(3 * SEC);
    for (k = 0; k < 40; k++) add_ext(SEC / 2 + k * 20 * MS, 5, k % 2 ? 15 : 9);
    {   /* an extended frame with DLC 15 */
        sim_ext_frame_t *e = &ext[sim.n_ext++];
        memset(e, 0, sizeof *e);
        e->ready = SEC / 2 + 900 * MS;
        e->f.sidh = 0x12; e->f.sidl = 0x08 | 0x03; e->f.eidh = 0x45; e->f.eidl = 0x67; e->f.dlc = 15;
        memset(e->f.d, 0x77, 8);
    }
    for (k = 0; k < 20; k++) add_ext(SEC + SEC / 2 + k * 20 * MS, 6, 5);
    finish_setup();
}
static void check_longdlc(void)
{
    dir_t a = check_can_to_pc(1, sim.end_ns - 300 * MS);
    int i, ext_ok = 0;
    for (i = 0; i < sim.n_to_pc; i++)
        if ((sim.to_pc[i].f.sidl & 0x08) && sim.to_pc[i].f.eidl == 0x67 && (sim.to_pc[i].f.dlc & 0x0F) == 8) ext_ok++;
    show("CAN to PC", a);
    need_clean("CAN to PC", a);
    need(ext_ok == 1, "extended frame with DLC 15 delivered %d times with 8 bytes (want 1)", ext_ok);
}

/* R6, R9: one PC stops acknowledging; the other PC and the bus must not suffer. */
static void setup_slowclient(void)
{
    uint64_t t;
    base(6 * SEC);
    sim.connected[2] = 1; sim.pc_acks[2] = 0;
    for (t = SEC / 2; t < sim.end_ns; t += rr(3 * MS, 7 * MS)) add_ext(t, 5, 5);
    finish_setup();
}
static void check_slowclient(void)
{
    dir_t a = check_can_to_pc(1, sim.end_ns - 300 * MS);
    dir_t c = check_can_to_pc(2, sim.end_ns - 300 * MS);
    int e = echoes();
    show("CAN to PC 1", a);
    printf("    CAN to PC 2 (stalled) %d delivered, dup %d, order %d\n", c.delivered, c.dup, c.order);
    printf("    frames from the bus sent back onto the bus: %d\n", e);
    need_clean("CAN to PC 1", a);
    need(c.dup == 0 && c.order == 0, "stalled PC got duplicates or reordering");
    need(e == 0, "%d frames from the bus echoed back onto it", e);
    need(a.lat_max < 20.0, "CAN to PC 1 max latency %.1f ms (limit 20)", a.lat_max);
}

typedef struct { const char *name; void (*setup)(void); void (*check)(void); int seeds; const char *what; } scen_t;
static const scen_t scen[] = {
    { "quiet",      setup_quiet,      check_quiet,      3,  "L1  sensor reports on a quiet bus" },
    { "busy",       setup_busy,       check_busy,       20, "L3 L5 R1  both directions busy, interrupt races" },
    { "race",       setup_race,       check_race,       20, "L3 L5 R1  bursts both ways, timing fuzz" },
    { "rxrace",     setup_rxrace,     check_rxrace,     10, "L5  receive path, heavy steady load, timing fuzz" },
    { "burst",      setup_burst,      check_burst,      5,  "L4  150-frame bursts from the PC to a busy bus" },
    { "grouped",    setup_grouped,    check_grouped,    3,  "L2  PC sends with Nagle on (JMRI Grouped), quiet bus" },
    { "deadbus",    setup_deadbus,    check_deadbus,    5,  "R1 R8  no acknowledgement, then recovery" },
    { "split",      setup_split,      check_split,      3,  "R2  frames split across TCP segments" },
    { "malformed",  setup_malformed,  check_malformed,  3,  "R3  malformed frame before good ones; lower-case hex" },
    { "longdlc",    setup_longdlc,    check_longdlc,    3,  "R7  CAN frames with DLC 9 to 15" },
    { "slowclient", setup_slowclient, check_slowclient, 3,  "R9  one PC stalls; echo onto the bus" },
};

static int run_one(const scen_t *sc, unsigned seed)
{
    int fd[2], status; pid_t pid; static char buf[1 << 20]; ssize_t n; size_t got = 0;
    if (pipe(fd)) { perror("pipe"); exit(2); }
    fflush(stdout);
    pid = fork();
    if (pid == 0) {
        dup2(fd[1], 1); dup2(fd[1], 2); close(fd[0]);
        rs = seed * 7919u + 17u; if (!rs) rs = 1;
        sc->setup();
        sim.seed = seed;
        sim_run();
        fails = 0; why[0] = 0;
        sc->check();
        show_counters();
        if (sim.resets) need(0, "firmware reset %d times", sim.resets);
        if (sim.watchdog_resets) need(0, "watchdog reset the module");
        if (sim.eeprom_write_failures) need(0, "%d EEPROM writes never began", sim.eeprom_write_failures);
        if (sim.tx_rearm_errors) need(0, "TXREQ set %d times while the previous frame was on the bus", sim.tx_rearm_errors);
        printf("RESULT %s %s\n", fails ? "FAIL" : "PASS", why);
        fflush(stdout);
        _exit(0);
    }
    close(fd[1]);
    for (;;) {                                  /* read it all; keep the first 1 MB */
        char sink[4096];
        n = got < sizeof buf - 1 ? read(fd[0], buf + got, sizeof buf - 1 - got) : read(fd[0], sink, sizeof sink);
        if (n <= 0) break;
        if (got < sizeof buf - 1) got += n;
    }
    buf[got] = 0;
    close(fd[0]);
    waitpid(pid, &status, 0);
    {
        char *res = strstr(buf, "RESULT ");
        int pass = res && !strncmp(res + 7, "PASS", 4) && WIFEXITED(status) && !WEXITSTATUS(status);
        printf("  %-10s seed %-3u %s", sc->name, seed, pass ? "PASS" : "FAIL");
        if (!res) {
            char *nl;
            printf("  crashed (%s)\n", WIFSIGNALED(status) ? strsignal(WTERMSIG(status)) : "exit");
            nl = strstr(buf, "ERROR: AddressSanitizer");
            if (!nl) nl = strstr(buf, "runtime error");
            if (nl) { char *e = strchr(nl, '\n'); if (e) *e = 0; printf("    %s\n", nl); }
        } else {
            char *e = strchr(res, '\n'); if (e) *e = 0;
            printf("%s%s\n", fails ? "" : "", res[11] ? res + 11 : "");
            *res = 0;
            fputs(buf, stdout);
        }
        return pass;
    }
}

int main(int argc, char **argv)
{
    const char *which = argc > 1 ? argv[1] : "all";
    unsigned first = argc > 2 ? (unsigned)atoi(argv[2]) : 1;
    int seeds = argc > 3 ? atoi(argv[3]) : 0, i, total = 0, passed = 0;
    for (i = 0; i < (int)(sizeof scen / sizeof scen[0]); i++) {
        unsigned s; int n;
        if (strcmp(which, "all") && strcmp(which, scen[i].name)) continue;
        n = seeds ? seeds : scen[i].seeds;
        printf("%s: %s\n", scen[i].name, scen[i].what);
        for (s = first; s < first + (unsigned)n; s++) { total++; passed += run_one(&scen[i], s); }
    }
    printf("\n%d of %d runs passed\n", passed, total);
    return passed == total ? 0 : 1;
}
