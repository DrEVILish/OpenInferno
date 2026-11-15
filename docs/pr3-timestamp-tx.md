# PR3 kernel TX integration (CLK-02, CLK-03, CLK-04)

AF_PACKET CQ timestamps now come exclusively from validated SOFTWARE TX
SO_TIMESTAMPING errorqueue records. In-memory and model-wire behavior is unchanged.
The socket requires TX_SOFTWARE | SOFTWARE | OPT_ID, with an explicit open error
on unsupported options. This does not claim support on actual AMU packet devices.
AMU/devmem still compile without kernel headers or additional syscalls.

Kernel ABI sources: https://docs.kernel.org/networking/timestamping.html,
https://github.com/torvalds/linux/blob/v6.12/net/packet/af_packet.c,
and VM linux/net_tstamp.h and linux/errqueue.h. The mirrored error record uses
fixed-width fields; native libc SCM_TIMESTAMPING timespec ABI matches the selected
SO_TIMESTAMPING option on host and musl PPC.

All successful packet sends, including media and untagged CQ packets, consume the
same ID sequence. A 64-entry table retains only requested CQ ID/tag/exact frame
associations for at most 100 ms. Full tables omit timestamps. EAGAIN does not
advance IDs: packet_snd allocates the timestamp ID after socket allocation.
Other send failures may occur after ID allocation, so disable TX stamp publication
and log a reopen-required error rather than guess the next ID. There is no retry
or post-send timestamp fallback. The real-kernel test validates IDs through 48,000
competing media sends and 3,000 CQ sends; EAGAIN injection is a seam test, not a
real congestion experiment.

Each service drains at most 256 TX error records, then runs RX and media.
Checks cover truncation, duplicate/missing control records, errno ENOMSG,
TIMESTAMPING origin, zero type/code, SND info, ID and exact returned frame.
Unknown/expired IDs cannot publish. Valid completion removes its association;
consumed duplicates cannot republish. A busy single CQ result register drops a
completion rather than overwrite an unread tag. There is no completion FIFO.
The existing RX bracket helper converts kernel realtime into monotonic time,
rejecting broad brackets and old model/realtime epochs. A TX emitted before a
rate/ToD write but consumed afterwards is rejected; no historical cache is altered.

The root check targets now run serially across recursive make invocations, fixing
the shared-object compile race; each child retains its -j4 budget. Clean builds
were used, rather than relying on existing objects.

## Validation on root@192.168.10.59, /root/oi-acq

Logs are under /root/q59-logs/ (VM-local paths).

- pr3-tx-clean-check.log: clean full host check passed, expected RTL XFAIL retained.
- pr3-tx-clean-e2e.log: unchanged strict-lock namespace e2e passed, including both
  media directions, multicast and state persistence. No assertion/timeout changes.
- pr3-tx-final-check.log and pr3-tx-final-unit.log: full host repeat and 18 TX seam
  checks passed. RX 27 conversion + 3 poll checks remain passing.
- pr3-tx-final-amu.log: final AMU suite, including clock 185 checks, plus static
  hard-float/e300c3/Linux2.6.29 validation. Earlier pr3-tx-amu.log failed an endian
  assumption in the new test fixture; the fixture was corrected and rerun.
- pr3-tx-before-final.log: real CQ/kernel test against parent source fails with
  unexpected synchronous CQ stamp before errorqueue consumption (exit 5).
- pr3-tx-kernel-final.log: tests/test_tx_kernel.c runs real CQ send + phy_poll on
  an isolated veth, 16 media sends per CQ send. 3000/3000 TX, 3000 RX batches;
  maximum TX/RX conversion differences 48/8 ns against an independent initial
  81 ns clock bracket. This measures software conversion consistency, including
  pairing uncertainty; it is not hardware or physical network accuracy proof.
- pr3-tx-daemon-e2e.log: a separately compiled, instrumented daemon passed the
  unchanged scenario. Instrumentation and preserved a.log/b.log are in
  pr3-tx-daemon-artifacts and /tmp/q59-tx-audit.c; metrics are in
  pr3-tx-daemon-metrics.json. A/B published 197/30 validated actual TX records.
  A/B valid RX+TX conversions: 8889/143163; largest brackets 301/979 ns;
  largest kernel queue ages 828123/3476551 ns. Recomputed host conversion residual
  was zero. This audits the daemon path and bounded pairing uncertainty; it does
  not independently measure clock truth or attribute e2e offset to individual legs.

Reproduce the privileged kernel check by compiling tests/test_tx_kernel.c with
src/util.c, -Isrc and -lm, then passing two up interfaces of a fresh isolated veth
pair. It deliberately remains outside the AMU/unprivileged default unit suite.

Readiness: scoped TX implementation is local reviewable WIP, no PR/merge/deploy.
Strict raw 1 us, eight samples/minimum span, KEEP20us, SW-ready 50us lock gating,
slew/mute behavior and source-return clock cache assertions are unchanged.
Original nine full PTP/noise/outlier cases, delayed/missing/jitter qualification,
dynamic quantized MEDIA_INC tracking and fault packet playback/mute/resume remain
for the broader PR3 step. Real congestion-induced EAGAIN and actual old-kernel
AF_PACKET runtime are not proven by host/qemu tests.
