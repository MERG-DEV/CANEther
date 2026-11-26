#!/usr/bin/env python3
"""
Checks that the host tests can tell a fault from a fix.

Each mutation puts one of the faults found in the 2eBETA13 firmware back into
a copy of the fixed source. The tests are then built against that copy and the
scenario that covers the fault is run; it must fail. A mutation the tests do
not catch is reported, so a passing test run means something.

Usage: python3 mutation_check.py [firmware_dir]     (default: ../..)
"""
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, '..', '..'))
FILES = ['canbus.c', 'canbus.h', 'cangc1e.c', 'cangc1e.h', 'commands.c', 'commands.h', 'eth.c', 'eth.h',
         'gcaeth.c', 'gcaeth.h', 'io.c', 'io.h', 'isr.c', 'isr.h', 'project.h', 'rocrail.h', 'utils.c', 'utils.h',
         'TCPIPConfig.h']

# (name, finding, scenario or directed:test, seeds, [(file, old, new), ...])
MUTATIONS = [
    ('no flush after CAN receive', 'L1', 'quiet', 2, [
        ('cangc1e.c', '    CBusEthFlush();     // send CAN frames to Ethernet now', '    // removed'),
        ('cangc1e.c', '    CBusEthFlush();     // frames relayed between connections', '    // removed'),
    ]),
    ('PIR3 = 0 in the interrupt, and no restart from the main loop', 'L3', 'directed:tx', 0, [
        ('isr.c', '#endif\n}\n', '#endif\n  PIR3 = 0;\n}\n'),
        ('canbus.c', '      txTimeoutCount++;\n    }\n  }\n  txStartNext();\n',
                     '      txTimeoutCount++;\n    }\n  }\n'),
    ]),
    ('interrupt handles the FIFO flag even while its interrupt is disabled', 'L5', 'directed:rx', 0, [
        ('isr.c', 'if (PIE3bits.FIFOWMIE && PIR3bits.FIFOWMIF)', 'if (PIR3bits.FIFOWMIF)'),
    ]),
    ('no restart of the transmit FIFO after a time-out', 'R1', 'deadbus', 2, [
        ('canbus.c', '      txTimeoutCount++;\n    }\n  }\n  txStartNext();\n',
                     '      txTimeoutCount++;\n    }\n  }\n'),
    ]),
    ('transmit buffer reused before an abort is confirmed', 'R1', 'deadbus', 2, [
        ('canbus.c', '    if (!TXB0CONbits.TXABT)\n      return;\n', ''),
    ]),
    ('no back-pressure: frames from Ethernet taken with the FIFO full', 'L4', 'burst', 2, [
        ('gcaeth.c', '  return canbusTxFree() >= CAN_TX_RESERVE || canbusTxErrorWarning();', '  return TRUE;'),
    ]),
    ('partial frame forgotten between reads', 'R2', 'split', 2, [
        ('gcaeth.c', '    if (TCPIsGetReady(ph->socket)) {\n      ph->idle = 0;\n',
                     '    if (TCPIsGetReady(ph->socket)) {\n      ph->idle = 0;\n      ph->rxLen = 0;\n'),
    ]),
    ('no resynchronisation on ":" inside an overlong frame', 'R3', 'malformed', 2, [
        ('gcaeth.c', "  if (c == ':') {\n    ph->rxBuf[0] = c;", "  if (c == ':' && ph->rxLen == 0) {\n    ph->rxBuf[0] = c;"),
    ]),
    ('DLC of 9 to 15 not limited to 8', 'R7', 'longdlc', 1, [
        ('canbus.c', '    return (len > 8 ? 8 : len);', '    return len;'),
    ]),
    ('retry of a frame from CAN sent to every connection and to CAN', 'R9', 'slowclient', 1, [
        ('cangc1e.c', '    if (CBusEthSendTo(&canRxPending, canRxPendingConns))',
                      '    if (CBusEthBroadcast(&canRxPending, 0xFF))'),
    ]),
]


def run(cmd, cwd):
    return subprocess.run(cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


def main():
    results = []
    with tempfile.TemporaryDirectory() as tmp:
        for name, finding, scenario, seeds, edits in MUTATIONS:
            src = os.path.join(tmp, 'fw')
            shutil.rmtree(src, ignore_errors=True)
            os.makedirs(src)
            for f in FILES:
                shutil.copy(os.path.join(FW, f), src)
            for f, old, new in edits:
                p = os.path.join(src, f)
                text = open(p).read()
                if text.count(old) != 1:
                    sys.exit('mutation "%s": text to change found %d times in %s' % (name, text.count(old), f))
                open(p, 'w').write(text.replace(old, new))
            out = os.path.join(tmp, 'build')
            shutil.rmtree(out, ignore_errors=True)
            b = run(['make', 'FW=' + src, 'OUT=' + out, out + '/scenarios', out + '/directed'], HERE)
            if b.returncode:
                sys.exit('build failed for mutation "%s":\n%s' % (name, b.stdout[-2000:]))
            if scenario.startswith('directed:'):
                r = run([out + '/directed', scenario.split(':')[1]], HERE)
            else:
                r = run([out + '/scenarios', scenario, '1', str(seeds)], HERE)
            caught = r.returncode != 0
            first_fail = next((l.strip() for l in r.stdout.splitlines() if 'FAIL' in l or 'k=' in l), '')
            results.append(caught)
            print('%-4s %-70s %-10s %s' % (finding, name, scenario, 'caught' if caught else 'NOT CAUGHT'))
            if first_fail:
                print('       ' + first_fail[:150])
    print('\n%d of %d mutations caught' % (sum(results), len(results)))
    return 0 if all(results) else 1


if __name__ == '__main__':
    sys.exit(main())
