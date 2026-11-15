# PR3 scoped RX timestamp correction

The AF_PACKET model converts SO_TIMESTAMPNS realtime stamps using a fresh
CLOCK_MONOTONIC / CLOCK_REALTIME / CLOCK_MONOTONIC bracket. Cached poll time
is no longer an operand. Brackets wider than 1 us, missing/duplicate/malformed
cmsgs, truncation, invalid nanoseconds, future stamps and ages over 100 ms are
rejected. Socket timestamp option failure prevents opening this backend.

For real packet sockets, rate changes and ToD steps establish an interpolation
boundary: older arrivals are dropped rather than evaluated with the new model
rate/epoch. In-memory behavior is unchanged. Successive clock pair offsets detect
realtime discontinuities outside a 1 us allowance plus 500 ppm elapsed-time
allowance; queued events predating that detection are rejected. This is bounded
step detection, not proof of detecting arbitrarily small realtime steps. RX
processing remains bounded at 256 records per service call.

References: https://docs.kernel.org/networking/timestamping.html and local libc
socket/time headers. No linux/ headers or new syscalls are introduced.

Tests: test_rx_poll replays the actual phy_poll receive seam with the cached poll
1 ms before the contemporaneous bracket. It fails the timestamp assertion against
c8631da and passes 3/3 against this change. test_stamp passes 27/27: conversion
bounds, malformed/missing/truncated timestamps, reordered arrivals, realtime
steps and model rate/ToD boundaries. These tests are included in host and AMU runs.
AMU test_clock remains 185/185; static hard-float/e300c3/Linux2.6.29 checks pass.

This is only the RX portion of CLK02/CLK03/CLK04 production work. TX still uses
post-send model time. Outstanding: SOFTWARE SO_TIMESTAMPING errorqueue integration,
bounded ID/tag/frame correlation with asynchronous/reordered/lost/send-failure
handling, safe unsupported-kernel handling, daemon actual TX timestamp error-budget
measurement and RX accuracy measurement under load. Keep errorqueue work bounded
so media progresses. Historical TX stamps must obey epoch boundaries too.

No lock thresholds, readiness thresholds, slew, mute bypasses, timeouts or e2e
assertions were changed. Strict raw 1 us/eight consecutive/minimum-span lock,
KEEP20us and whole-output 5 ppm/s requirements remain. No hardware claim.

The unchanged namespace e2e still fails strict lock and dependent media assertions.
No PR is warranted. Original nine full PTP/noise/outlier cases, jitter/missing/
delayed Sync cases, dynamic quantized MEDIA_INC averages and fault playback/mute/
resume coverage remain owed. The earlier observation-relative 500 ms pre-mute
limitation (possible 750 ms event-relative RX-out) remains.

VM evidence: /root/q59-logs/pr3-rx-final-repeat-{check,e2e}.log,
/root/q59-logs/pr3-rx-final2-amu.log. An earlier full-check attempt hit concurrent
recursive compilation of fpga_model.o; its failed linker run is preserved as
pr3-rx-final-check.log. Final-repeat uses the already built object.

Subsequent scoped TX integration and newer validation are recorded in
[pr3-timestamp-tx.md](pr3-timestamp-tx.md); the failures and missing TX work above
are the historical RX-only result, not the latest combined timestamp result.
