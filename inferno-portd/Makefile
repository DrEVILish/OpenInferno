# inferno-portd - the AMU control plane of the OpenProSeries FPGA Inferno ports (fpga/docs/inferno.md section 9).
# Native C11, no dependencies beyond libc / libm. Licence: GPL-3.0-or-later.
#
#   make              the x86 host build (build/host/inferno-portd)
#   make ppc          the AMU build: PowerPC e300c3 (MPC8315), big-endian, static (build/ppc/inferno-portd)
#   make check        unit tests + the in-process tests (virtual time), then the network-namespace end-to-end test
#   make check-unit   the unit / in-process tests only
#   make check-ppc    the unit / in-process tests built for PowerPC, run under qemu-ppc-static
#   make check-e2e    two daemons in network namespaces on a veth pair (needs root; skipped otherwise)
# Project rule: run heavy commands as  nice -n 10 taskset -c <two cores> make -j2
CC      ?= gcc
PPCCC   ?= powerpc-linux-gnu-gcc
CFLAGS  ?= -O2 -g
WARN    := -std=gnu11 -Wall -Wextra -Werror -Wno-format-truncation
QEMU    ?= qemu-ppc-static
SRCS    := util.c hw_devmem.c fpga_model.c pktq.c net.c mdns.c port.c arc.c flows_tx.c subscriber.c info.c settings.c \
           ptp.c clock.c ctl.c daemon.c
OBJS_H  := $(SRCS:%.c=build/host/%.o)
OBJS_P  := $(SRCS:%.c=build/ppc/%.o)
TESTS   := test_unit test_inproc test_clock test_net test_edges test_hw

.PHONY: all ppc check check-unit check-ppc check-e2e check-cli clean
all: build/host/inferno-portd

build/host build/ppc:
	mkdir -p $@

build/host/%.o: src/%.c src/*.h | build/host
	$(CC) $(WARN) $(CFLAGS) -c -o $@ $<
build/ppc/%.o: src/%.c src/*.h | build/ppc
	$(PPCCC) $(WARN) $(CFLAGS) -mcpu=e300c3 -c -o $@ $<

build/host/inferno-portd: build/host/main.o $(OBJS_H)
	$(CC) -o $@ $^ -lm
build/ppc/inferno-portd: build/ppc/main.o $(OBJS_P)
	$(PPCCC) -static -o $@ $^ -lm
ppc: build/ppc/inferno-portd

build/host/test_%.o: tests/test_%.c tests/*.h src/*.h | build/host
	$(CC) $(WARN) $(CFLAGS) -Isrc -c -o $@ $<
build/ppc/test_%.o: tests/test_%.c tests/*.h src/*.h | build/ppc
	$(PPCCC) $(WARN) $(CFLAGS) -mcpu=e300c3 -Isrc -c -o $@ $<
# The hardware test includes a backend with local mocks; libc is never interposed.
build/host/test_hw.o build/ppc/test_hw.o: src/hw_devmem.c
build/host/test_hw: build/host/test_hw.o build/host/util.o
	$(CC) -o $@ $^ -lm
build/ppc/test_hw: build/ppc/test_hw.o build/ppc/util.o
	$(PPCCC) -static -o $@ $^ -lm

build/host/test_%: build/host/test_%.o $(OBJS_H)
	$(CC) -o $@ $^ -lm
build/ppc/test_%: build/ppc/test_%.o $(OBJS_P)
	$(PPCCC) -static -o $@ $^ -lm

check-unit: $(TESTS:%=build/host/%)
	@failed=0; for t in $(TESTS); do build/host/$$t || failed=1; done; exit $$failed

check-ppc: $(TESTS:%=build/ppc/%)
	@for t in $(TESTS); do $(QEMU) build/ppc/$$t || exit 1; done

check-e2e: build/host/inferno-portd
	sh tests/e2e.sh build/host/inferno-portd

check-cli: build/host/inferno-portd
	python3 -I tests/test_cli.py $(abspath build/host/inferno-portd)

check: check-unit check-cli check-e2e

clean:
	rm -rf build
