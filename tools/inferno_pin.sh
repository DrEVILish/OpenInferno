#!/bin/sh
# inferno_pin.sh - check that the upstream Inferno lines that tools/inferno_xcheck.c transliterates are still in the
# pinned commit of a local clone (github.com/teodly/inferno). Written for OpenProSeries (W16). Licence: GPL-3.0.
#   tools/inferno_pin.sh <clone dir> <commit>
# Exit 0: all present at that commit; 1: a line is missing or the clone is at another commit; 0 with a note when the
# clone is absent (the model in inferno_xcheck.c is self-contained).
UP="$1"; PIN="$2"
if [ ! -d "$UP/inferno_aoip" ]; then
    echo "pin: no upstream Inferno clone at $UP (git clone https://github.com/teodly/inferno); model check only"
    exit 0
fi
HEAD=$(git -C "$UP" rev-parse HEAD 2>/dev/null)
if [ "$HEAD" != "$PIN" ]; then echo "pin: $UP is at $HEAD, not $PIN"; exit 1; fi
D="$UP/inferno_aoip/src/device_server"
fail=0
chk() {
    if grep -qF -- "$2" "$D/$1"; then :; else echo "pin: missing in $1: $2"; fail=1; fi
}
chk flows_tx.rs 'pbuff[0] = 2u8; // ???'
chk flows_tx.rs 'let seconds = packet_ts / (sample_rate as LongClock);'
chk flows_tx.rs 'let subsec_samples = packet_ts % (sample_rate as LongClock);'
chk flows_tx.rs 'pbuff[1..5].copy_from_slice(&(seconds as u32).to_be_bytes());'
chk flows_tx.rs 'pbuff[5..9].copy_from_slice(&(subsec_samples as u32).to_be_bytes());'
chk flows_tx.rs 'let start = 9 + index_in_flow * flow.bytes_per_sample;'
chk flows_tx.rs 'let stride = channels_in_flow * flow.bytes_per_sample;'
chk flows_tx.rs 'let to_send = 9 + stride * flow.fpp;'
chk flows_tx.rs 'self.next_ts = now.wrapping_add(self.fpp as LongClock - remainder);'
chk flows_tx.rs 'const CLOCK_OFFSET_NS: ClockDiff = -500_000;'
chk flows_tx.rs 'pub const FPP_MAX_ADVERTISED: u16 = 32;'
chk samples_utils.rs 'dst[pos..pos + $bytes].copy_from_slice(&out_sample.to_be_bytes()[0..$bytes]);'
chk flows_rx.rs '.wrapping_mul(sample_rate as usize)'
chk flows_rx.rs '.wrapping_add(u32::from_be_bytes([buf[5], buf[6], buf[7], buf[8]]) as usize)'
chk flows_rx.rs 'let samples_count = audio_bytes.len() / stride;'
chk flows_rx.rs 'const KEEPALIVE_CONTENT: [u8; 2] = [0x13, 0x37];'
chk tx_multicasts.rs 'pub const MEDIA_PORT: u16 = 4321;'
if [ $fail -eq 0 ]; then echo "pin: upstream Inferno $PIN has every line the model follows"; fi
exit $fail
