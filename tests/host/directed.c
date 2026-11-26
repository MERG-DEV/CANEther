/*
 * Host test environment for the CANEther firmware: directed interrupt tests.
 *
 * The scenarios find races by chance. These tests place an interrupt at each
 * register access in turn inside the code that the race needs, so every
 * interleaving of that kind is tried once:
 *
 *   rx   L5  While the main loop reads a frame straight from the receive
 *            FIFO (canbusRecv, its FIFO interrupt disabled), a fourth frame
 *            arrives (the FIFO interrupt flag is raised) and the stack's tick
 *            timer interrupts. Each frame must be received once, in order.
 *
 *   tx   L3  While the low priority interrupt runs (entered for the receive
 *            FIFO), our frame on the bus finishes, raising the transmit
 *            interrupt flag there. The two frames queued behind it must go
 *            out within 10 ms, in order.
 *
 * Each case runs in its own process, as the firmware keeps state in globals.
 * Usage: directed [rx|tx|all]
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>
#include "sim.h"

typedef struct { unsigned char b[14]; unsigned char status; } CANMsg;   /* as canbus.h */
unsigned char canbusSend(CANMsg *msg);
unsigned char canbusRecv(CANMsg *msg);
void cbusSetup(void);
void initIO(void);
void TickInit(void);
extern void canbusService(void) __attribute__((weak));     /* in the fixed firmware only */

static sim_ext_frame_t ext[64];
static long arm_at, accesses;
static int armed_rx, armed_tx;

static sim_frame_t frame(int seq)
{
    sim_frame_t f;
    memset(&f, 0, sizeof f);
    f.sidh = 0xB0; f.sidl = 0x20; f.dlc = 5;
    f.d[0] = 0x90; f.d[1] = 0x01; f.d[2] = 0x01; f.d[3] = 0; f.d[4] = (unsigned char)seq;
    return f;
}

static void setup(void)
{
    memset(&sim, 0, sizeof sim);
    sim.end_ns = 1000ull * 1000 * 1000 * 1000;
    sim.ext = ext;
    sim_reset();
    initIO();                           /* as main(): interrupts, then the stack, then CAN */
    TickInit();                         /* the stack's tick timer: a low priority interrupt */
    cbusSetup();
}

/* ------------------------------------------------------------------- rx */

static void on_access_rx(int in_low, int in_high)
{
    if (!armed_rx || in_low || in_high) return;
    if (++accesses == arm_at) {
        sim_frame_t f = frame(3);
        armed_rx = 0;
        sim_inject_rx(&f);              /* fourth frame: FIFO interrupt flag set */
        sim_tick_interrupt();           /* and another low priority interrupt arrives */
    }
}

static int test_rx(long k)
{
    CANMsg m;
    int got[16], n = 0, i, idle_rounds = 0;
    setup();
    for (i = 0; i < 3; i++) { sim_frame_t f = frame(i); sim_inject_rx(&f); }
    sim.on_access = on_access_rx;
    arm_at = k; accesses = 0; armed_rx = 1;
    while (idle_rounds < 3 && n < 16) {
        if (canbusRecv(&m)) { got[n++] = m.b[10]; idle_rounds = 0; }
        else {
            if (armed_rx) return -1;    /* k beyond the accesses canbusRecv made: all placements tried */
            sim_idle(1000000); idle_rounds++;
        }
    }
    if (n != 4) { printf("    k=%ld: %d frames received, want 4:", k, n); for (i = 0; i < n; i++) printf(" %d", got[i]); printf("\n"); return 0; }
    for (i = 0; i < 4; i++)
        if (got[i] != i) { printf("    k=%ld: order", k); for (i = 0; i < n; i++) printf(" %d", got[i]); printf("\n"); return 0; }
    return 1;
}

/* ------------------------------------------------------------------- tx */

static void on_access_tx(int in_low, int in_high)
{
    if (!armed_tx || !in_low || in_high) return;
    if (++accesses == arm_at) {
        armed_tx = 0;
        sim_force_tx_end();             /* our frame finishes now, inside the interrupt */
    }
}

static int test_tx(long k)
{
    CANMsg m;
    int i;
    uint64_t t0;
    setup();
    for (i = 0; i < 3; i++) {
        sim_frame_t f = frame(10 + i);
        memset(&m, 0, sizeof m);
        m.b[1] = f.sidh; m.b[2] = f.sidl; m.b[5] = f.dlc; memcpy(&m.b[6], f.d, 8);
        canbusSend(&m);
    }
    for (i = 0; i < 3; i++) { sim_frame_t f = frame(i); sim_inject_rx(&f); }
    sim.on_access = on_access_tx;
    arm_at = k; accesses = 0; armed_tx = 1;
    { sim_frame_t f = frame(3); sim_inject_rx(&f); }   /* fourth frame: the receive interrupt */
    t0 = sim_now();
    while (sim_now() < t0 + 10000000ull) {              /* 10 ms of main loop passes */
        if (canbusService) canbusService();
        while (canbusRecv(&m)) ;
        sim_idle(200000);
    }
    if (armed_tx) return -1;
    {
        int seen = 0;
        for (i = 0; i < sim.n_can_out; i++)
            if (sim.can_out[i].f.d[4] == 10 + seen) seen++;
        if (seen != 3) {
            printf("    k=%ld: %d of 3 queued frames sent within 10 ms\n", k, seen);
            return 0;
        }
    }
    return 1;
}

static int run_case(int (*fn)(long), long k)
{
    int fd[2], status, r = -2; pid_t pid; char buf[4096]; ssize_t n;
    if (pipe(fd)) exit(2);
    fflush(stdout);
    pid = fork();
    if (pid == 0) {
        int v;
        close(fd[0]); dup2(fd[1], 1);
        v = fn(k);
        printf("R%d\n", v);
        fflush(stdout);
        _exit(0);
    }
    close(fd[1]);
    n = read(fd[0], buf, sizeof buf - 1);
    buf[n > 0 ? n : 0] = 0;
    close(fd[0]);
    waitpid(pid, &status, 0);
    {
        char *p = strstr(buf, "R");
        while (p && p[1] != '-' && (p[1] < '0' || p[1] > '9')) p = strstr(p + 1, "R");
        if (p) { r = atoi(p + 1); *p = 0; }
        fputs(buf, stdout);
    }
    if (!WIFEXITED(status) || WEXITSTATUS(status)) { printf("    k=%ld: crashed\n", k); return 0; }
    return r;
}

static int sweep(const char *name, int (*fn)(long))
{
    long k; int pass = 0, fail = 0;
    for (k = 1; k < 400; k++) {
        int r = run_case(fn, k);
        if (r == -1) break;             /* past the last access: all placements tried */
        if (r == 1) pass++; else fail++;
    }
    printf("%s: %d cases: %d passed, %d failed\n", name, pass + fail, pass, fail);
    return fail == 0;
}

int main(int argc, char **argv)
{
    const char *which = argc > 1 ? argv[1] : "all";
    int ok = 1;
    if (!strcmp(which, "all") || !strcmp(which, "rx")) ok &= sweep("rx (L5)", test_rx);
    if (!strcmp(which, "all") || !strcmp(which, "tx")) ok &= sweep("tx (L3)", test_tx);
    return ok ? 0 : 1;
}
