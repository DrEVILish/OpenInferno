# Standalone Inferno media path and control-plane checks.
.DEFAULT_GOAL := check
.DELETE_ON_ERROR:
PY ?= python3 -I
TASKSET ?= nice -n 10
B := build
RTL := rtl/rmii_tx.v rtl/rmii_rx.v rtl/inferno_tx.v rtl/inferno_rx.v rtl/cpu_pktq.v
# CH_CPF_LAT[_96k|_48k[_MAXCH[_NF[_JIT_US]]]] (MAXCH default 64, any even 2..64; NF 256; JIT_US: a switch adds 0..JIT_US
# us per packet); MAX_CH 2 has no column bits in its 96 kHz address; the _48 runs (the 48 / 24-column port) reach every
# buffer slot, past the end of an unpadded 2^CW stride. NF 64 is the MAXLAT 1 build (build 28, b32): 96 kHz, 1 ms,
# fpp 16 (CPF 16) and fpp 32 (CPF 8, a Dante sender's 32-frame packets)
CAP := 48_8_0 48_8_1 48_8_2 48_8_3 64_16_0 64_16_1 64_16_2 64_16_3 64_32_0 32_16_3_96k \
       2_2_0_48k_2 1_1_0_96k_2 4_4_0_48k_4 2_2_0_96k_4 \
       48_16_3_48k_48 24_8_3_96k_48 48_8_0_48k_48 48_32_1_48k_48 \
       32_16_1_96k_64_64_400 32_8_1_96k_64_64_250
# expected to fail (XFAIL; an XPASS fails the check): 32-frame packets at 96 kHz / 1 ms leave about 340 us for
# transit, so 400 us of jitter loses frames whatever the buffer size (clock design note 2026-11-14 section 8, PR 10)
CAP_XFAIL := 32_8_1_96k_64_64_400
.PHONY: check sim xcheck control cli amu clean
check: xcheck control cli amu sim
cli:
	$(MAKE) -C inferno-portd check-cli
control:
	$(MAKE) -C inferno-portd check-unit
amu:
	@$(if $(MUSL_CC),$(MAKE) -C inferno-portd check-amu,echo "SKIP amu: set MUSL_CC")
$(B):
	mkdir -p $@
$(B)/tb_inferno.vvp: tb/tb_inferno.v $(wildcard rtl/*) | $(B)
	iverilog -g2005 -Irtl -Itb -s tb_inferno -o $@ $< $(RTL)
capw = $(word $2,$(subst _, ,$1))
$(B)/tb_infcap_%.vvp: tb/tb_infcap.v $(wildcard rtl/*) | $(B)
	iverilog -g2005 -Irtl -Itb -s tb_infcap -DCH=$(call capw,$*,1) -DCPF=$(call capw,$*,2) -DLAT=$(call capw,$*,3) $(if $(filter 96k,$(call capw,$*,4)),-DFS96) $(if $(call capw,$*,5),-DMAXCH=$(call capw,$*,5)) $(if $(call capw,$*,6),-DNF=$(call capw,$*,6)) $(if $(call capw,$*,7),-DJIT_US=$(call capw,$*,7)) $(if $(filter 64_16_0,$*),-DGOLDEN=\"golden/inferno_64ch.hex\") -o $@ $< $(RTL) rtl/inferno_regs.v
$(B)/%.log: $(B)/%.vvp
	vvp -n $< > $@
	@cat $@
	@grep -q '^PASS' $@ && ! grep -q '^FAIL' $@
$(foreach c,$(CAP_XFAIL),$(B)/tb_infcap_$(c).log): $(B)/%.log: $(B)/%.vvp
	vvp -n $< > $@
	@cat $@
	@if grep -q '^PASS' $@; then echo "XPASS $*: expected to fail; move it to CAP"; exit 1; fi
	@grep -q 'samples lost [1-9]' $@ || { echo "FAIL $*: failed, but not by losing samples"; exit 1; }
	@echo "XFAIL $*: $$(grep -o 'dropped late .*' $@)"
EXTRA_BENCHES := tb_inferno_staging tb_inferno_short_after_long tb_inferno_pace tb_inferno_lengths tb_rmii tb_ptp tb_ptp_servo tb_fs_phase
$(addprefix $(B)/,$(addsuffix .vvp,$(EXTRA_BENCHES))): $(B)/%.vvp: tb/%.v $(wildcard rtl/*) | $(B)
	iverilog -g2005 -Irtl -Itb -s $* -o $@ $< $(RTL) rtl/ptp_clock.v rtl/fs_phase.v
sim: $(addprefix $(B)/,$(addsuffix .log,$(EXTRA_BENCHES))) $(B)/tb_inferno.log $(foreach c,$(CAP) $(CAP_XFAIL),$(B)/tb_infcap_$(c).log)
clean:
	rm -rf build
	$(MAKE) -C inferno-portd clean

INFERNO_UP     ?= ../inferno
INFERNO_COMMIT := 9767558e0905c9e9c960f5a60e050fbdc38300b7
xcheck: tools/golden.py tools/inferno_xcheck.c tools/inferno_pin.sh | $(B)
	$(TASKSET) cc -std=c11 -O1 -Wall -Wextra -o $(B)/inferno_xcheck tools/inferno_xcheck.c
	$(B)/inferno_xcheck tx tb 48 > $(B)/xcheck_model_tb.txt
	$(PY) tools/golden.py inferno --packets 48 --out $(B)/xcheck_tb.hex --frames-txt $(B)/xcheck_golden_tb.txt
	cmp $(B)/xcheck_model_tb.txt $(B)/xcheck_golden_tb.txt
	$(B)/inferno_xcheck tx cap64 64 > $(B)/xcheck_model_cap64.txt
	$(PY) tools/golden.py inferno --cfg cap64 --packets 64 --out $(B)/xcheck_cap64.hex --frames-txt $(B)/xcheck_golden_cap64.txt
	cmp $(B)/xcheck_model_cap64.txt $(B)/xcheck_golden_cap64.txt
	$(B)/inferno_xcheck rx tb < $(B)/xcheck_golden_tb.txt
	$(B)/inferno_xcheck rx cap64 < $(B)/xcheck_golden_cap64.txt
	sh tools/inferno_pin.sh $(INFERNO_UP) $(INFERNO_COMMIT)
	@echo "xcheck: 48 + 64 Inferno packets (3 + 4 flows) identical to the upstream-Inferno model, decoded back sample-exact"
