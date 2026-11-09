# OpenInferno

FPGA audio-over-IP media engines and a native C control daemon for Inferno-compatible flows. Extracted from [OpenProSeries](https://github.com/DrEVILish/openproseries), source commit `9b0115b6566c95f0f63a8ba5255b461c3718b471`.

The media path supports up to 64 channels per port, 16 TX and 16 RX flows, unicast/multicast, and selectable 0.5 / 1 / 2 / 5 ms playout latency. The RTL includes RMII Ethernet MACs, packet queues, flow registers and shared PTP/media-clock building blocks. The daemon provides discovery, subscriptions, flow control and clock control.

## Build and test

Install GNU Make, a C compiler, Python 3 and Icarus Verilog (Debian/Ubuntu: `apt install build-essential python3 iverilog`).

```sh
make check                         # packet cross-check, C tests, RTL simulations
make xcheck                        # independent C encoder/decoder vs Python packets
make control                       # daemon unit and in-process tests
make sim                           # media loopback and ten capacity/latency cases
make -C inferno-portd               # host daemon
make -C inferno-portd ppc           # MPC8315 build; needs powerpc-linux-gnu-gcc
```

`make xcheck` runs without an upstream checkout. To also verify the upstream source pin, clone https://github.com/teodly/inferno at `9767558e0905c9e9c960f5a60e050fbdc38300b7` and set `INFERNO_UP=/path/to/inferno`.

## Integration and status

This is a reusable core extraction, not a board-ready bitstream. Wire the media engines, register blocks, packet queue and clock to your platform; the C daemon's hardware backend expects the OpenProSeries register layout. There are no board pin constraints or synthesis project in this repository.

See [the original architecture and register reference](docs/inferno.md). That document retains historical OpenProSeries paths, board resource measurements and integration references; files outside this extraction remain in the parent project. These measurements are not standalone synthesis results.

The implementation has simulation and host-model tests. Hardware interoperability with Dante devices has not been established. This is an independent project with no Audinate affiliation or certification.

## License and provenance

GPL version 3; see [LICENSE](LICENSE) and individual source notices (the control daemon is GPL-3.0-or-later). Original OpenProSeries attribution is preserved. The Inferno media format and protocol references follow [teodly/inferno](https://github.com/teodly/inferno) at the pinned commit above. No vendor firmware, bitstreams, firmware dumps or upstream source checkout are included.
