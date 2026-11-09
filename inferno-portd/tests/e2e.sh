#!/bin/sh
# e2e.sh - inferno-portd end to end, in real time: two daemons, each on its own FPGA model, in two network namespaces
# joined by a veth pair (no LAN traffic: the namespaces see only each other). A = FOH-1 (PTP leader, desk leader),
# B = MON-2 (+50 ppm oscillator, 3 s off, PTP follower steering its desk clock). Checks: PTP lock, 64 channels
# B <- A through mDNS + flow requests with the audio pattern checked by B's model, 8 channels A <- B, a multicast
# bundle, the state file. Needs root (ip netns); prints SKIP otherwise.
# Part of OpenProSeries inferno-portd. Licence: GPL-3.0-or-later.
set -u
BIN=$(readlink -f "${1:-build/host/inferno-portd}")
TS="nice -n 10 taskset -c ${CORES:-3,5}"
D=$(mktemp -d /tmp/portd-e2e.XXXXXX)
NA=portdA$$ NB=portdB$$
fails=0
pass() { echo "  PASS $*"; }
fail() { echo "  FAIL $*"; fails=$((fails + 1)); }
cleanup() {
    [ -n "${PA:-}" ] && kill "$PA" 2>/dev/null
    [ -n "${PB:-}" ] && kill "$PB" 2>/dev/null
    sleep 0.3
    ip netns del $NA 2>/dev/null; ip netns del $NB 2>/dev/null
    rm -rf "$D"
}
trap cleanup EXIT
if ! ip netns add $NA 2>/dev/null; then echo "SKIP e2e: no network namespaces (needs root)"; exit 0; fi
ip netns add $NB
ip link add vA$$ netns $NA type veth peer name vB$$ netns $NB || { echo "SKIP e2e: no veth"; exit 0; }
for x in "$NA vA$$" "$NB vB$$"; do
    set -- $x
    ip netns exec $1 sysctl -qw net.ipv6.conf.all.disable_ipv6=1 2>/dev/null
    ip netns exec $1 ip link set lo up
    ip netns exec $1 ip link set $2 up
done
TOD=1760000000000000000
ip netns exec $NA $TS $BIN --port 1 --desk FOH --backend model --phy vA$$ --model-id 1 --model-tod $TOD --owner --ptp leader \
    --steer --latency 5 --ctl $D/a.sock --state $D/a.state -q 2> $D/a.log &
PA=$!
ip netns exec $NB $TS $BIN --port 2 --desk MON --backend model --phy vB$$ --model-id 2 --model-ppm 50 \
    --model-tod $((TOD + 3123456789)) --owner --ptp follower --steer --latency 5 --ctl $D/b.sock --state $D/b.state -q 2> $D/b.log &
PB=$!
ctl() { s=$1; shift; $BIN ctl $D/$s.sock "$@" 2>/dev/null; }
field() { ctl $1 status | sed -n "s/^$2=//p"; }
waitfor() {   # waitfor <seconds> <shell test>
    t=0
    while [ $t -lt $(($1 * 4)) ]; do eval "$2" && return 0; sleep 0.25; t=$((t + 1)); done
    return 1
}
sleep 1
echo "e2e: PTP (B follows A over the veth, software stamps of the model)"
if waitfor 40 '[ "$(field b ptp_state)" = SLAVE ] && [ "$(field b clock_locked)" = 1 ]'; then
    pass "B SLAVE and locked, offset $(field b ptp_offset_ns) ns, delay $(field b ptp_delay_ns) ns, steps $(field b clock_steps)"
else fail "B not locked: $(field b ptp_state) locked=$(field b clock_locked) offset $(field b ptp_offset_ns)"; fi
[ "$(field a ptp_state)" = MASTER ] && pass "A MASTER, $(field a ptp_syncs_tx) Syncs sent" || fail "A is $(field a ptp_state)"
sleep 3
echo "e2e: B RX 1..64 <- FOH-1 TX 01..64"
for k in $(seq 1 64); do ctl b sub $k $(printf %02d $k)@FOH-1 > /dev/null; done
if waitfor 30 '[ "$(field b subs_receiving)" = 64 ]'; then pass "64 of 64 receiving, $(field b rxflows) RX flows"
else fail "$(field b subs_receiving) of 64 receiving"; ctl b status | grep rxflow; fi
sleep 2
ok=$(field b model_pattern_ok); bad=$(field b model_pattern_bad)
[ "${ok:-0}" -gt 100000 ] && [ "${bad:-1}" = 0 ] && pass "audio pattern: $ok samples right, $bad wrong" || fail "pattern ok=$ok bad=$bad"
m=0
for k in 1 17 33 64; do [ "$(ctl b colsrc $k | sed -n 's/^colsrc=//p')" = "1:$k" ] && m=$((m + 1)); done
[ $m = 4 ] && pass "columns 1 / 17 / 33 / 64 carry FOH-1's channels 1 / 17 / 33 / 64" || fail "column map $m of 4"
late=$(field b hw_rx_late); early=$(field b hw_rx_early)
echo "  (B model: late $late, early $early packets since start: the ones before the lock and the media reloads)"
echo "e2e: A RX 1..8 <- MON-2 TX 41..48"
for k in $(seq 1 8); do ctl a sub $k $((40 + k))@MON-2 > /dev/null; done
if waitfor 20 '[ "$(field a subs_receiving)" = 8 ]'; then pass "8 of 8 receiving"; else fail "$(field a subs_receiving) of 8"; fi
[ "$(ctl a colsrc 3 | sed -n 's/^colsrc=//p')" = "2:43" ] && pass "A column 3 carries MON-2 channel 43" || fail "A column 3: $(ctl a colsrc 3)"
echo "e2e: multicast bundle FOH-1 flow 16 (channels 9..16), B RX 9..16 move onto it"
ctl a mcast 16 9,10,11,12,13,14,15,16 > /dev/null
sleep 3
for k in $(seq 9 16); do ctl b unsub $k > /dev/null; ctl b sub $k $(printf %02d $k)@FOH-1 > /dev/null; done
if waitfor 20 '[ "$(ctl b status | grep -c "^rxflow.*mcast")" -ge 1 ] && [ "$(field b subs_receiving)" = 64 ]'; then
    pass "B receives the bundle ($(ctl b status | grep '^rxflow.*mcast'))"
else fail "no multicast RX flow: $(ctl b status | grep rxflow)"; fi
echo "e2e: state file"
ctl a quit > /dev/null; ctl b quit > /dev/null
sleep 1
grep -q "^sub 3 43@MON-2" $D/a.state && grep -q "^mcast 16 9,10" $D/a.state && pass "A's state keeps its subscriptions and the bundle" || fail "state: $(cat $D/a.state)"
PA= PB=
if [ $fails = 0 ]; then echo "PASS e2e"; else echo "FAIL e2e: $fails"; tail -n 20 $D/a.log $D/b.log; fi
[ $fails = 0 ]
