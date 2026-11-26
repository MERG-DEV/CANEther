# Host tests for the CANEther firmware

These tests run the firmware's own C files on a PC, against a model of the
hardware around it, so changes can be checked before a build is put on a
CANEther. They do not replace a test on the module: the model is described
below, with its limits.

## What is compiled

`canbus.c cangc1e.c commands.c eth.c gcaeth.c io.c isr.c utils.c`, unchanged,
with gcc. `include/` stands in for the C18 and stack headers:

- `p18f2585.h` declares the registers. Those that hardware changes by itself
  (interrupt flags, ECAN, EEPROM) are reached through accessors that run the
  model in `sim.c` at every access, and take any enabled, pending interrupt
  there, as the PIC does between instructions.
- `TCPIP Stack/TCPIP.h` declares the stack calls the firmware makes; `sim.c`
  implements them as a model of the TCP connections to PCs.
- `cbusdefs8j.h` (and copies named `8h` and `8r`, the names other files
  include) gives the CBUS values used, as the real files are not in the
  repository.

## The model (`sim.c`)

- CAN at 125 kbit/s with other nodes: arbitration by identifier, interframe
  space, frame length from the DLC plus a stuffing allowance. A "dead" bus
  gives no acknowledgement, so our frames fail and retry, and the transmit
  error counter rises as the CAN specification says.
- ECAN Mode 2: 8-deep receive FIFO, FIFO interrupt when four are full,
  overflow raising RXBnOVFL and ERRIF; ERRIF also on error state changes,
  not on lost arbitration (data sheet 23.15.6). Clearing TXREQ while our
  frame is on the bus lets it finish (23.6.2); TXREQ then reads 0, the harder
  case, and setting it again before that frame ends is reported as an error.
- Timer 1 every 50 ms (high priority), timer 0 every 2.1 s (low priority).
- Watchdog: once software enables it (WDTCON, as 2eBETA14 does), the run
  fails if `ClrWdt()` is not called for 4.1 s.
- Data EEPROM: a write begins only if WR is set straight after the two
  EECON2 writes (data sheet 7.4); otherwise EEIF never comes.
- TCP per connection: 256-byte receive window, 512-byte send buffer, data
  sent on TCPFlush, when half full, or 40 ms after the first unflushed byte
  (as TCP.c); window updates as TCP.c sends them. The PC acknowledges every
  second segment at once and a lone one after 40 ms. A PC can use Nagle's
  algorithm (JMRI "Grouped"); we then acknowledge after the firmware's own
  `TCP_DELAYED_ACK_TIMEOUT`, read from its `TCPIPConfig.h` (`dack.c`).
- Time: each register access costs 0.25 to 1.5 us, a pass of `StackTask`
  0.15 to 0.4 ms, and 0.5 to 1.5 ms more while data is moving. These are
  estimates; the firmware's speed on the PIC has not been measured.
- Timing fuzz (some scenarios): at a small chance per access, the frame on
  the bus ends there (cut short) or the tick timer interrupts there, so
  events land anywhere in the code.

## Scenarios (`scenarios.c`)

Every frame is numbered and checked end to end: each CAN frame reaches each
PC once and in order, each PC frame reaches the bus once and in order, no
frame from the bus goes back onto it, latency limits, and no frame from a PC
waits while the bus is idle.

| Scenario   | Covers   | What                                                    |
|------------|----------|---------------------------------------------------------|
| quiet      | L1       | sensor reports on a quiet bus reach the PC at once      |
| busy       | L3 L5 R1 | both directions busy                                    |
| race       | L3 L5 R1 | bursts both ways, timing fuzz                           |
| rxrace     | L5       | steady heavy receive load, timing fuzz                  |
| burst      | L4       | 150-frame bursts from the PC to a busy bus              |
| grouped    | L2       | PC with Nagle on (JMRI Grouped)                         |
| deadbus    | R1 R8    | no acknowledgement for 3 s, then recovery               |
| split      | R2       | frames split across TCP segments                        |
| malformed  | R3       | malformed frames before good ones; lower-case hex       |
| longdlc    | R7       | CAN frames with DLC 9 to 15                             |
| slowclient | R9       | one PC stops acknowledging                              |

## Directed tests (`directed.c`)

The scenarios find races by chance. These place an interrupt at each register
access in turn inside the code a race needs, so each placement is tried once:
`rx` for the receive FIFO race (L5), `tx` for a transmit-complete raised
inside the low priority interrupt (L3).

## Checking the tests (`mutation_check.py`)

Puts each fault back into a copy of the fixed source, one at a time, and
checks that its test fails.

## Running

    make                                  # build and run everything on this tree
    make FW=/path/to/other/source OUT=build-other   # run on another version
    ./build/scenarios race 1 50           # one scenario, seeds 1 to 50
    ./build/directed tx
    python3 mutation_check.py

Needs gcc (or clang) with the address and undefined-behaviour sanitizers,
GNU make and Python 3. Each run is a separate process, so a crash or a
sanitizer report fails that run only.
