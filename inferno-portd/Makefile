# inferno-portd - the AMU control plane of the OpenProSeries FPGA Inferno ports (fpga/docs/inferno.md section 9).
# Native C11, no dependencies beyond libc / libm. Licence: GPL-3.0-or-later.
#
#   make              the x86 host build (build/host/inferno-portd)
#   make ppc          generic PowerPC e300c3 glibc build, static, for modern kernels; not a Pro1 binary (build/ppc/inferno-portd)
#   make amu          the Pro1 AMU build (MPC8315, Linux 2.6.29): musl, static; set MUSL_CC (build/amu/inferno-portd)
#   make check        unit tests + the in-process tests (virtual time), then the network-namespace end-to-end test
#   make check-unit   the unit / in-process tests only
#   make check-ppc    the unit / in-process tests built for PowerPC, run under qemu-ppc-static
#   make check-amu    the amu build's tests + a daemon smoke run under $(AMU_QEMU); AMU_CHECK = a static-binary checker
#   make check-e2e    two daemons in network namespaces on a veth pair (needs root; skipped otherwise)
# Project rule: run heavy commands as  nice -n 10 taskset -c <two cores> make -j2
CC      ?= gcc
PPCCC   ?= powerpc-linux-gnu-gcc
CFLAGS  ?= -O2 -g
WARN    := -std=gnu11 -Wall -Wextra -Werror -Wno-format-truncation
QEMU    ?= qemu-ppc-static
PPCB    ?= build/ppc
MUSL_CC ?=
AMU_QEMU ?= qemu-ppc -r 2.6.29 -cpu e300c3
AMU_CHECK ?=
SRCS    := util.c hw_devmem.c fpga_model.c pktq.c net.c mdns.c port.c arc.c flows_tx.c subscriber.c info.c settings.c \
           ptp.c clock.c ctl.c daemon.c
OBJS_H  := $(SRCS:%.c=build/host/%.o)
OBJS_P  := $(SRCS:%.c=$(PPCB)/%.o)
TESTS   := test_unit test_inproc test_clock test_net test_edges test_hw

.PHONY: all ppc amu check check-unit check-ppc check-amu check-e2e check-cli clean
all: build/host/inferno-portd

build/host $(PPCB):
	mkdir -p $@

build/host/%.o: src/%.c src/*.h | build/host
	$(CC) $(WARN) $(CFLAGS) -c -o $@ $<
$(PPCB)/%.o: src/%.c src/*.h | $(PPCB)
	$(PPCCC) $(WARN) $(CFLAGS) -mcpu=e300c3 -c -o $@ $<

build/host/inferno-portd: build/host/main.o $(OBJS_H)
	$(CC) -o $@ $^ -lm
$(PPCB)/inferno-portd: $(PPCB)/main.o $(OBJS_P)
	$(PPCCC) -static -o $@ $^ -lm
ppc: $(PPCB)/inferno-portd

build/host/test_%.o: tests/test_%.c tests/*.h src/*.h | build/host
	$(CC) $(WARN) $(CFLAGS) -Isrc -c -o $@ $<
$(PPCB)/test_%.o: tests/test_%.c tests/*.h src/*.h | $(PPCB)
	$(PPCCC) $(WARN) $(CFLAGS) -mcpu=e300c3 -Isrc -c -o $@ $<
# The hardware test includes a backend with local mocks; libc is never interposed.
build/host/test_hw.o $(PPCB)/test_hw.o: src/hw_devmem.c
build/host/test_hw: build/host/test_hw.o build/host/util.o
	$(CC) -o $@ $^ -lm
$(PPCB)/test_hw: $(PPCB)/test_hw.o $(PPCB)/util.o
	$(PPCCC) -static -o $@ $^ -lm

build/host/test_%: build/host/test_%.o $(OBJS_H)
	$(CC) -o $@ $^ -lm
$(PPCB)/test_%: $(PPCB)/test_%.o $(OBJS_P)
	$(PPCCC) -static -o $@ $^ -lm

check-unit: $(TESTS:%=build/host/%)
	@! grep -rn '#include <linux/' src tests  # the musl sysroot has no linux/ headers
	@failed=0; for t in $(TESTS); do build/host/$$t || failed=1; done; exit $$failed

check-ppc: $(TESTS:%=$(PPCB)/%)
	@for t in $(TESTS); do $(QEMU) $(PPCB)/$$t || exit 1; done

# Pro1: Linux 2.6.29, too old for Debian's glibc; qemu's -r only fakes uname, so this is no real-AMU test.
AMU_MAKE = $(MAKE) PPCB=build/amu PPCCC="$(MUSL_CC) -mlong-double-64 -no-pie"
amu:
	@test -x "$(MUSL_CC)" || { echo "amu: set MUSL_CC to a PowerPC musl-gcc (got '$(MUSL_CC)')"; exit 2; }
	$(AMU_MAKE) ppc

check-amu: amu
	$(AMU_MAKE) QEMU="$(AMU_QEMU)" check-ppc
	@$(AMU_QEMU) build/amu/inferno-portd --port 0; test $$? -eq 2 || { echo "amu smoke: --port 0 must exit 2"; exit 1; }
	@if [ -n "$(AMU_CHECK)" ]; then python3 -I $(AMU_CHECK) build/amu/inferno-portd; else echo "SKIP static check: set AMU_CHECK"; fi

check-e2e: build/host/inferno-portd
	sh tests/e2e.sh build/host/inferno-portd

check-cli: build/host/inferno-portd
	python3 -I tests/test_cli.py $(abspath build/host/inferno-portd)

check: check-unit check-cli check-e2e

clean:
	rm -rf build
