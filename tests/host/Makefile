# Host test environment for the CANEther firmware.
#
#   make                 builds and runs the tests on the firmware in this tree
#   make FW=/path/to/src builds and runs them on another copy of the firmware
#                        (for example a checkout of an earlier version)
#   ./build/scenarios busy 1 50   runs one scenario for 50 seeds
#   ./build/directed rx           runs one directed interrupt test
#
# Needs gcc (or clang) with the address and undefined-behaviour sanitizers.

FW      ?= ../..
OUT     ?= build
FWSRC   := canbus.c cangc1e.c commands.c eth.c gcaeth.c io.c isr.c utils.c
SAN     := -fsanitize=address,undefined -fno-omit-frame-pointer
DEFS    := -DCANEther=1 -DWITHCAN -Dmain=firmware_main
CC      ?= gcc

all: $(OUT)/scenarios $(OUT)/directed
	./$(OUT)/directed all
	./$(OUT)/scenarios all

$(OUT)/scenarios: sim.c sim.h scenarios.c dack.c include/*.h include/TCPIP\ Stack/*.h $(addprefix $(FW)/,$(FWSRC)) $(FW)/TCPIPConfig.h
	mkdir -p $(OUT)/fw
	for f in $(FWSRC); do \
	  $(CC) -std=gnu99 -O1 -g $(SAN) $(DEFS) -w -Iinclude -I$(FW) -c $(FW)/$$f -o $(OUT)/fw/$${f%.c}.o || exit 1; \
	done
	$(CC) -std=gnu99 -O1 -g $(SAN) -w -Iinclude -I$(FW) -c dack.c -o $(OUT)/fw/dack.o
	$(CC) -std=gnu99 -O1 -g $(SAN) -Wall -Wextra -Wno-unused-parameter -Iinclude -c sim.c -o $(OUT)/sim.o
	$(CC) -std=gnu99 -O1 -g $(SAN) -Wall -Wextra -c scenarios.c -o $(OUT)/scenarios.o
	$(CC) $(SAN) $(OUT)/sim.o $(OUT)/scenarios.o $(OUT)/fw/*.o -lm -o $@

$(OUT)/directed: $(OUT)/scenarios directed.c
	$(CC) -std=gnu99 -O1 -g $(SAN) -Wall -Wextra -c directed.c -o $(OUT)/directed.o
	$(CC) $(SAN) $(OUT)/sim.o $(OUT)/directed.o $(OUT)/fw/*.o -lm -o $@

clean:
	rm -rf $(OUT)

.PHONY: all clean
