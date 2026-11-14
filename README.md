# OpenInferno

FPGA audio-over-IP media engines and a native C control daemon for Inferno-compatible flows. Extracted from [OpenProSeries](https://github.com/DrEVILish/openproseries), source commit `9b0115b6566c95f0f63a8ba5255b461c3718b471`.

The media path supports up to 64 channels per port, 16 TX and 16 RX flows, unicast/multicast, and selectable 0.5 / 1 / 2 / 5 ms playout latency. The RTL includes RMII Ethernet MACs, packet queues, flow registers and shared PTP/media-clock building blocks. The daemon provides discovery, subscriptions, flow control and clock control.

## Build and test

Install GNU Make, a C compiler, Python 3 and Icarus Verilog (Debian/Ubuntu: `apt install build-essential python3 iverilog`).

```sh
make check                         # packet cross-check, C tests, RTL simulations
make xcheck                        # independent C encoder/decoder vs Python packets
make control                       # daemon unit and in-process tests
make sim                           # media loopback, 14 capacity/latency cases, MAC/PTP/fs_phase/pace benches
make -C inferno-portd               # host daemon
make -C inferno-portd ppc           # generic PowerPC/glibc build for modern kernels; not a Pro1 binary
make -C inferno-portd amu MUSL_CC=/path/to/musl-ppc/bin/musl-gcc  # Pro1 AMU (Linux 2.6.29) build
```

The Pro1 AMU runs Linux 2.6.29; Debian's static PowerPC glibc requires Linux 3.2 or newer (its ABI tag says 3.2.0 and it calls newer syscalls such as `rseq`), so `ppc` is not a Pro1 binary. Build the Pro1 daemon with `make amu`, which needs `MUSL_CC`: a PowerPC musl-gcc wrapper (musl 1.2.6, static, over Debian's `gcc-powerpc-linux-gnu`, hard float). `amu` adds `-mlong-double-64 -no-pie` to the existing `-O2 -static -mcpu=e300c3` and builds into `build/amu`. `make check MUSL_CC=...` also runs `check-amu`: the unit tests and a daemon smoke run under `qemu-ppc -r 2.6.29 -cpu e300c3`, plus `AMU_CHECK`, a script that checks the static binary (e.g. its syscalls against 2.6.29), when set. Without `MUSL_CC` it prints `SKIP amu`. QEMU's `-r 2.6.29` only fakes the reported kernel version; it is not a test on a real AMU.

`make xcheck` runs without an upstream checkout. To also verify the upstream source pin, clone https://github.com/teodly/inferno at `9767558e0905c9e9c960f5a60e050fbdc38300b7` and set `INFERNO_UP=/path/to/inferno`.

## Integration and status

This is a reusable core extraction, not a board-ready bitstream. Wire the media engines, register blocks, packet queue and clock to your platform; the C daemon's hardware backend expects the OpenProSeries register layout. There are no board pin constraints or synthesis project in this repository.

- **Size:** `inferno_tx` / `inferno_rx` take `MAX_CH` = any even 2..64 columns (half of them at 96 kHz; an odd value fails elaboration). The OpenProSeries target is 48 per port at 48 kHz, 24 at 96 kHz. Buffer addresses are `{slot, column}`, so the memories are padded to a 2^⌈log2 MAX_CH⌉ stride (48 columns use 64-column RAM); a map byte at or past the columns in use is muted on TX and ignored on RX. The daemon reads `INF_CAPS` (columns, channels per flow, TX and RX flows) at start and uses those as its limits, so receive-only (0 TX flows) and small builds run; it refuses caps above its compile-time tables.
- **Latency:** a subscription whose transmitter latency exceeds the jitter buffer (`JB_GEOM` frames) is refused rather than clamped: one warning, ARC RX status 0x1A, a retry every 10 s.
- **Clock:** on a follower's first lock the audio loop measures the frequency error and, above 10 ppm, glides there before its phase loop starts. `--audio-acquire-ppm-s R` sets the glide (0 < R ≤ 1000, default 5 ppm/s, the loop's own limit). After a media load (e.g. a 48/96 kHz change) the loop skips the stale `fs_phase` snapshot.
- **MAC pace:** the engines assume the RMII MACs' 4 clocks an octet. `inferno_rx` needs at least 3 clocks an octet evenly spaced (2 only up to flow 11); `inferno_tx` and `cpu_pktq` TX give a correct octet at most every 2 clocks. A faster MAC needs a pacer. A short packet right after a long one waits for the previous commit instead of being dropped. Exact limits: docs/inferno.md §3.3–3.4, checked by `tb_inferno_pace` and `tb_inferno_short_after_long`.

See [the original architecture and register reference](docs/inferno.md). That document retains historical OpenProSeries paths, board resource measurements and integration references; files outside this extraction remain in the parent project. These measurements are not standalone synthesis results.

The implementation has simulation and host-model tests. Hardware interoperability with Dante devices has not been established. This is an independent project with no Audinate affiliation or certification.

## License and provenance

GPL version 3; see [LICENSE](LICENSE) and individual source notices (the control daemon is GPL-3.0-or-later). Original OpenProSeries attribution is preserved. The Inferno media format and protocol references follow [teodly/inferno](https://github.com/teodly/inferno) at the pinned commit above. No vendor firmware, bitstreams, firmware dumps or upstream source checkout are included.
