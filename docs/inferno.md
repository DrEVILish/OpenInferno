# Inferno (Dante-compatible) flows on the AES50 ports: FPGA / AMU split

Status: 2026-10-14, second revision, simulation and ECP5 synthesis; 2026-10-19: the AMU control plane `inferno-portd`
(§9.4) and the desk-clock loops (§9.3, `fs_phase.v`), host-tested. **AES67 is dropped; Inferno is the only AoIP
format** (user decision 2026-10-14). The port engines are `rtl/inferno_tx.v`, `rtl/inferno_rx.v`, `rtl/inferno_regs.v`,
`rtl/inferno_lat.vh`, `rtl/cpu_pktq.v` (with `tx_arb`) and `rtl/udp_ipv4.vh`, wired into `rtl/aoip_port.v` as port
mode 1. Tests: `tb/tb_inferno.v`, `tb/tb_infcap.v` (10 runs), `tb/tb_top_inferno.v` (2 ports, 128 channels),
`tb/tb_top.v`, and `make xcheck`. No hardware has been involved.

Clean room: written from the public standards (IEEE 802.3, IEEE 1588, RFC 768 / 791 / 1071 / 1112), the project
documents listed in §1, and **upstream Inferno** (github.com/teodly/inferno, GPL-3.0), cloned to
`work/scratch-inferno/upstream` at commit **`9767558e0905c9e9c960f5a60e050fbdc38300b7`** (2026-09-08). The files read
are `inferno_aoip/src/device_server/flows_tx.rs`, `flows_rx.rs`, `samples_utils.rs`, `tx_multicasts.rs`,
`mdns_server.rs`, `media_clock.rs`, `device_info.rs` and `protocol/*.rs` (port numbers). No vendor firmware,
bitstream or recon material was used.

The first revision of this design (earlier on 2026-10-14) took its packet notes from a separate ESP32-P4 bridge
project. That dependency is removed: the RTL comments, `tools/golden.py` and `make xcheck` now cite and check only
upstream Inferno, and the control-plane plan (§9) is based on upstream's modules.

## 1. Decisions this design follows

| Source | Decision | Where it shows here |
|---|---|---|
| user, 2026-10-14 | **drop AES67, keep Inferno**; at least 48 channels each way per port, **128 channels over two ports** acceptable; latency **0.5 / 1 / 2 / 5 ms** | port modes AES50 / Inferno / ACE; 64 columns per port (§5); `LATENCY` register (§4) |
| docs/68 §2 | Inferno is a **mode of an AES50 port** (`aoip`); several ports may use it at once; each Inferno port is its own NetAudio device **`<DESK>-<n>`** (port A = 1) | mode 1 of `aoip_port.v`; one engine per port; one control-plane instance per port (§9) |
| docs/68 §3 | **one desk clock** in the FPGA; the desk is leader (PTP leader on every Inferno port) or follower of one source | `ptp_clock.v` is the single time base; per-port SFD stamps go through the CPU queue; servo and roles are AMU software |
| docs/68 §4 | the port mode lives in the show, the desk name in the console prefs | flow tables and the latency mode are run-time state the AMU daemon programs |
| upstream `flows_tx.rs` / `flows_rx.rs` | media flows are separate from the control servers; packet = 9-byte header + interleaved big-endian samples; receivers stamp T = sec × fs + sub | the FPGA is the media path, the AMU runs the control servers (§2) |
| project rule | server daemons native C/C++ (no Node.js, no Qt) | the AMU control plane is a C daemon written against upstream's protocol code (§9), not the Rust binary |

## 2. The split

```
               PRO1 IO FPGA (per AES50 port in mode 1)                                 AMU (MPC8315, Linux)
 SMIB slots ─┐                                                                       ┌──────────────────────────────┐
 (64 ch)     ▼                                                                       │ inferno-portd  (one per port) │
        ┌─────────────┐   ┌────────┐                                                 │  ARC 4440  CMC 8800           │
        │ inferno_tx  │──►│ tx_arb │──► rmii_tx ──► PHY ══ RJ45 (Dante network)       │  flow control 4455            │
        │ ring 128 fr │   │ audio  │      │ SFD ─┐                                    │  info 8700 / 8702 / 8708      │
        │ 16 flows    │   │ first  │      ▼      │                                    │  mDNS 5353, subscriptions     │
        │ <=32 ch/flow│   └────────┘   tod ─► stamps                                  │  keepalives 13 37             │
        └─────────────┘     ▲ CPU TX frames   │                                       │  ─ programs the flow tables ─ │
              ▲ flow table  │                 ▼                                       ├──────────────────────────────┤
        ┌─────────────┐   ┌──────────────────────┐                                    │ ptpd: PTPv1 follower servo /  │
        │inferno_regs │◄──┤ cpu_pktq             │◄═══ local bus (16-bit GPCM) ══════►│ leader; writes PTP_INCR,      │
        │ LATENCY     │   │ TX ring 2 KB, RX 8 KB │                                    │ MEDIA_INC, steps              │
        └─────────────┘   │ SFD time stamps       │                                    └──────────────────────────────┘
              │ flow cfg  └──────────────────────┘
              ▼                    ▲ frames not claimed by audio
        ┌─────────────┐            │
 SMIB ◄─┤ inferno_rx  │◄──── rmii_rx ◄── PHY
 slots  │ jitter 256  │  claim ┘
        │ fr x 64 col │
        └─────────────┘
 desk clock: ptp_clock (time of day, media NCO; media_ts and (media_sec, media_sub)), shared by all ports
```

**FPGA (data plane, per port):**
- **`inferno_tx`**: NTXF = 16 TX flows. Each flow has up to MAXFCH = 32 channels picked from the port's 64 (or a
  silent channel), 1…32 frames per packet (or *automatic*: the latency mode's), 24 or 32 bits, its own destination
  (unicast or multicast), DSCP and TTL, and the timestamp advance of the port.
- **`inferno_rx`**: NRXF = 16 RX flows. Each takes up to 32 channels of 16/24/32 bits onto the port's 64 columns, with
  a per-flow SHIFT. One timestamp-indexed jitter buffer, played at the latency mode's offset.
- **SFD time stamps** for every CPU frame received and for CPU frames sent with TS_REQ (PTP event messages).
- **The media clock** is `ptp_clock`'s NCO: `media_ts` = T mod 2³² and (`media_sec`, `media_sub`) = (T / fs,
  T mod fs). T counts samples since the PTP epoch at fs (upstream `media_clock.rs`: time × fs).
- **`cpu_pktq`**: the per-port mailbox for every frame the audio depacketiser does not claim.

**AMU (control plane):** everything with state machines, timers, names and persistence (§9).

## 3. The audio flow packet (exact)

### 3.1 Frame layout

| Bytes | Field | Value |
|---|---|---|
| 0–5 | Ethernet destination | the receiver's MAC (unicast, from ARP), or `01:00:5E` + the group's low 23 bits (RFC 1112) |
| 6–11 | Ethernet source | the port's MAC |
| 12–13 | EtherType | `08 00` |
| 14 | IPv4 version / IHL | `45` |
| 15 | TOS | per flow; the daemon's default DSCP 46 (EF) → `B8` |
| 16–17 | total length | 20 + 8 + 9 + payload |
| 18–19 | identification | +1 per packet, per port |
| 20–21 | flags / offset | `40 00` (DF) |
| 22 | TTL | per flow (default 64 unicast, 32 multicast) |
| 23 | protocol | 17 (UDP) |
| 24–25 | header checksum | RFC 1071 |
| 26–29, 30–33 | source / destination IP | the port's IP; the receiver's IP or the group (upstream multicast bundles: 239.255.x.y) |
| 34–35 | UDP source port | the TX flow's port (the daemon's choice; upstream binds an ephemeral port per flow) |
| 36–37 | UDP destination port | the port in the receiver's flow request (unicast); **4321** for multicast (upstream `MEDIA_PORT`) |
| 38–39 | UDP length | 8 + 9 + payload |
| 40–41 | UDP checksum | `00 00` (none, legal over IPv4) |
| 42 | media byte 0 | **`02`** (upstream: `pbuff[0] = 2u8; // ???`) |
| 43–46 | seconds | u32 big-endian, **Ts / fs** |
| 47–50 | sub-second | u32 big-endian, **Ts mod fs** (samples) |
| 51… | samples | fpp frames × nch samples, frame-interleaved, channels in the flow's order, big-endian |

- **Ts = T − TX_TS_ADVANCE**, T being the media time of the packet's **first frame**. Upstream stamps
  `next_ts + clock_offset_samples` with `CLOCK_OFFSET_NS = −500 000`, i.e. 0.5 ms early (24 frames at 48 kHz). Our
  default advance is 0; the daemon sets 0.5 ms × fs if Dante gear proves to expect upstream's convention (§11).
- **Receiving:** T = seconds × fs + sub (upstream `flows_rx.rs`, wrapping), then T + SHIFT is the frame's place.
- **Sample formats:** bps 3 = the 24-bit sample big-endian; bps 4 = the 24-bit sample left-justified in 32 bits, low
  byte 0 (upstream writes the top bps bytes of the left-justified i32, `samples_utils.rs`); bps 2 (RX only) keeps 16
  bits.
- **Channels per flow:** up to MAXFCH = 32 each way. Upstream's own TX limit is `MAX_CHANNELS_IN_FLOW = 8`; Dante
  devices request up to their own limit. 64 channels at 0.5 ms need ≥ 11 channels per flow (§5).
- **Frames per packet:** upstream accepts 2…256 in a request and advertises `fpp=32,2`. `inferno_tx` builds 1…32 and
  never more than fits one 1500-byte IPv4 packet: payload = fpp × nch × bps ≤ 1472 − 9 = **1463 bytes**.
- **No sequence number:** loss is a timestamp gap (`RX_TS_GAPS`).
- **Keepalive:** the receiver sends the 2-byte datagram **`13 37`** every 250 ms (upstream `KEEPALIVE_INTERVAL`) from
  its flow port to the packets' source; a transmitter drops a unicast flow after 4 s without one
  (`KEEPALIVE_TIMEOUT_SECONDS`). Both directions are control-plane work through `cpu_pktq`.

### 3.2 What `make xcheck` and the golden files prove
- `tools/golden.py` builds the packets independently in Python (`inferno_media`, `inferno_packet`,
  `inferno_schedule`); flow sets `tb` (the 3 flows of `tb_inferno`) and `cap64` (4 × 16 channels, fpp 8).
- `tools/inferno_xcheck.c` is an **independent C model of upstream Inferno**, a line-by-line transliteration of
  `transmit()` (flows_tx.rs), `write_s24/s32_samples` (samples_utils.rs) and `receive()` (flows_rx.rs). `make xcheck`
  compares 48 `tb` packets and 64 `cap64` packets with golden.py byte for byte, decodes the golden packets back as
  upstream's receiver does (every timestamp and all 9 764 samples checked), and runs `tools/inferno_pin.sh`, which
  checks that the 17 upstream lines the model and this document rely on are present in the pinned commit of the local clone. No Rust
  toolchain is installed here, so upstream itself is not executed; the model plus the pin check stand in for it.
- `tb_inferno` compares the first 24 RTL packets with `golden/inferno_tx.hex`; `tb_infcap 64_16_0` the first 16 of
  the 64-channel configuration with `golden/inferno_64ch.hex`.

### 3.3 Transmit (`inferno_tx.v`)
- **Sample ring:** the port's 64 channels go into a ring of NR = 128 frames, at slot = media time mod 128 (address
  {slot, channel}: no multiplier). A frame is written before its frame strobe.
- **Effective fpp** per flow = min(requested or automatic, MAX_FPP = 32, the largest power of two that fits the MTU
  for nch × bps). A flow whose effective fpp changes (a `LATENCY` or w0 write) restarts on its next boundary.
- **Due packets:** a flow's packet is due on the strobe of its last frame; due flows are sent one at a time, the
  lowest flow number first; one becoming due while its previous packet still waits counts an overrun.
- **Alignment:** a power-of-two fpp starts on a multiple of fpp of the media time (upstream `bootstrap_next_ts`); other
  fpp start on the next frame. 48 000 is a multiple of every power of two up to 128, so aligned packets never
  straddle a second.
- **Header vs payload:** 13 table words load in 14 clocks (block RAM, registered reads); the payload reads the ring at
  (T + frame) mod 128 and the column of the flow's channel map, one byte per 4 clocks (the MAC's pace).

### 3.4 Receive (`inferno_rx.v`)
- **Matching:** Ethernet II, IPv4 IHL 5 not fragmented, UDP; {dst IP, dst port} against the enabled flows' keys, one
  a clock from byte 37; the flow's configuration (13 words) loads before the payload starts.
- **Payload length** from the UDP length (Ethernet padding ignored); a payload that is not a whole number of frames
  is bad (upstream floors it).
- **The packet time** T = seconds × fs + sub mod 2³² by a 17-step shift-and-add during the header.
- **Late / early:** a packet wholly before the playout point is late; one reaching past the buffer is early (NF = 256
  frames ahead at 48 kHz, 512 at 96 kHz). Both are dropped and counted.
- **Commit:** samples are staged per packet (two banks of 512) and copied into the jitter buffer only after a good
  FCS, one a clock; in parallel the tag engine marks each frame time {mask of the flows that wrote it, T[15:0]}
  (2 clocks a frame).
- **Playout:** one clock after the frame strobe the frame `media_ts − offset` is played as n_out words; a column plays
  only if its last writer's bit is set in that frame's tag. The tag is then **swept** (cleared), so a frame is never
  played twice and a stopped flow plays silence, not old audio. Several transmitters can fill one port frame.
- **96 kHz:** the same 16 K-sample buffer is 512 frames × 32 columns (address {slot[8:0], column[4:0]}), so the buffer
  covers the same 5.3 ms at both rates.
- **Per-flow status:** source IP and port (keepalive destination), packet count, peak latency.

## 4. Port modes and latency modes

| MODE (aoip_regs 0x04) | Engine | Notes |
|---|---|---|
| 0 | AES50 (the OpenAES50 engine on `AES50_MASK` ports, aes50_engine.v; else a quiet stub) | default; drives the AES50 clock channels |
| 1 | **Inferno** | was "Inferno/AES67": AES67 is retired, Inferno is the only format |
| 2 | A&H ACE | docs/ace.md |
| 3 | off | quiet |

`inferno_regs` `LATENCY` (0x2C) bits 1:0 select the latency mode at run time:

| mode | latency | playout offset 48 kHz / 96 kHz | automatic fpp 48 / 96 kHz | effective fpp at 48 kHz for 8 / 16 / 32-ch flows (MTU) | budget at 48 kHz, 64 ch |
|---|---|---|---|---|---|
| 0 | **0.5 ms** | 24 / 48 frames | 8 / 16 | 8 / 8 / 8 | fpp 8 + send queue ≤ 7 (4 × 459 B) + 1 hop ≤ 2 + 2 = 19 < 24 (measured arrival ≤ 14 frames) |
| 1 | **1 ms** | 48 / 96 | 16 / 32 | 16 / 16 / 8 | measured arrival ≤ 27 frames < 48 |
| 2 | **2 ms** | 96 / 192 | 32 / 32 | 32 / 16 / 8 | 32 + ≤ 17 + 2 + 2 < 96 |
| 3 | **5 ms** | 240 / 480 | 32 / 32 | 32 / 16 / 8 | ample; the 5 ms mode buys jitter tolerance (long chains, busy switches), not channels |

- **Playout offset** = latency × fs, exactly; `LAT_STATUS` (0x30) reads back the offset and fpp in use. `LATENCY`
  bit 8 (MANUAL) plays at `aoip_regs` `LINK_OFFSET` (0x3C, frames) instead, for tests or odd latencies.
- **Automatic fpp** is used by TX flows whose w0 fpp field is 0 (the desk's own multicast flows, and unicast flows
  whose request leaves it to us). A Dante receiver's flow request names an fpp; the daemon writes it as is (bounded
  to the advertised 2…32).
- **Why fpp 32, not 64, at 5 ms:** 64 frames × 8 channels × 3 bytes = 1536 bytes, over the 1463-byte payload limit;
  only flows of ≤ 7 channels could use it. Upstream also advertises fpp ≤ 32. Larger packets would not add channels
  (§5): the payload limit caps the overhead saving.
- **Jitter-buffer depth** is fixed by the build (NF = 256 frames × 64 columns at 48 kHz, 512 × 32 at 96 kHz), which
  holds the 5 ms mode plus a packet; a flow may run up to NF − fpp frames ahead (its own latency above ours).
  `JB_GEOM` (0x34) reports it. A packet that arrives as soon as its last frame exists reaches offset + SHIFT (the
  larger of the two latencies) past the playout point, so `inferno-portd` subscribes only to transmitters whose
  advertised latency is at most JB_GEOM frames; above that it requests no flow and reports the channel as Dante RX
  status 26 (0x1A, channel latency), retrying every 10 s.

## 5. Channels and capacity

**Per port: 64 columns each way at 48 kHz (MAX_CH), 32 at 96 kHz. Two Inferno ports give 128 channels each way**
(`tb_top_inferno`). The binding limit is the 100 Mb/s link.

Model (`python3 -I tools/inferno_capacity.py [--fs 96000]` prints it): 24-bit samples, 48 kHz, wire cost per packet = 75 B
(preamble + SFD 8, Ethernet 14, IPv4 20, UDP 8, Inferno header 9, FCS 4, gap 12) + fpp × nch × 3; N channels in
⌊N / nch⌋ full flows plus one partial flow; budget 95 % of 100 Mb/s. "max ch" is the largest N that fits.

| latency | fpp (nominal) | ch / flow | payload | max ch (nominal fpp) | reference | **max ch, this design** (MTU-cut fpp) | load at 64 ch | 64 ch reached |
|---|---|---|---|---|---|---|---|---|
| 0.5 ms | 8 | 8 | 192 B | 57 | 57 ✓ | **57** | 102.5 Mb/s | no (48 ch: 76.9 Mb/s, tested) |
| 0.5 ms | 8 | 16 | 384 B | 66 | 66 ✓ | **66** | 88.1 Mb/s (measured 88.0) | **yes, tested** |
| 0.5 ms | 8 | 32 | 768 B | 73 | 73 ✓ | **73** | 80.9 Mb/s (measured) | **yes, tested** |
| 1 ms | 16 | 8 | 384 B | 68 | 68 ✓ | **68** | 88.1 Mb/s | yes |
| 1 ms | 16 | 16 | 768 B | 74 | 74 ✓ | **74** | 80.9 Mb/s (measured) | **yes, tested** |
| 1 ms | 16 | 32 | 1536 B > MTU | 77 | 77 (needs jumbo) | **73** (fpp 8) | 80.9 Mb/s | yes |
| 2 ms | 32 | 8 | 768 B | 74 | 74 ✓ | **74** | 80.9 Mb/s | yes (48 ch tested: 60.6 Mb/s) |
| 2 ms | 32 | 16 | 1536 B > MTU | 78 | 78 (needs jumbo) | **74** (fpp 16) | 80.9 Mb/s (measured) | **yes, tested** |
| 5 ms | 64 | 8 | 1536 B > MTU | 78 | 78 (needs jumbo) | **74** (fpp 32) | 80.9 Mb/s | yes (48 ch tested) |
| 5 ms | 64 | 16 | 3072 B > MTU | 80 | 80 (needs jumbo) | **74** (fpp 16) | 80.9 Mb/s (measured) | **yes, tested** |

- The reference numbers (from the task brief) are reproduced exactly by this model. Four of them assume packets
  larger than a 1500-byte Ethernet MTU allows; with the payload cut to 1463 bytes the absolute ceiling is
  **78 channels** (fpp × nch × 3 = 1440–1456 bytes per packet), and with the power-of-two fpp this design uses,
  **73–74** above 0.5 ms.
- **Achieved per port: 64 channels each way in every latency mode** (the 64-column limit, not the link), with
  ≥ 16-channel flows at 0.5 ms and any of 8 / 16 / 32-channel flows above it, **two ports = 128 channels each way**.
  48 channels fit every mode with 8-channel flows (upstream's own TX flow width).
- **96 kHz:** 32 columns per port; the link carries 33–37 channels (fpp 16 / 32 automatic), so 32 in every mode.
- **Admission control (daemon):** refuse a flow request (0x0302) that would push the port's TX over ~95 Mb/s by this
  model; keep RX flow requests at ≥ 16 channels per flow when the port carries more than 57 channels at 0.5 ms.
- RX is the other direction of the full-duplex link and has its own 100 Mb/s.

## 6. Register map

Each port's 4 KB window holds three blocks (the port decodes `reg_addr[11:8]`):

| Offset | Block |
|---|---|
| 0x000 | `aoip_regs.v` (`OAP2`): mode, addresses, desk clock, ACE; the AES67 registers are retired (reserved) |
| 0x100 | `inferno_regs.v` (`OIN2`) |
| 0x200 | `cpu_pktq.v` (`OCQ1`, unchanged) |

### `aoip_regs.v` (0x000), what changed
`0x04` MODE: 1 = Inferno (was Inferno/AES67). `0x14/0x18` SRC_MAC and `0x1C` SRC_IP (was TX_SRC_IP) are the port's
addresses for Inferno and ACE. `0x3C` LINK_OFFSET is the manual playout offset (frames). **Reserved** (read 0, writes
ignored; were AES67): `0x08–0x10`, `0x20–0x38`, `0x80–0x9C`, `0xA4`. `0xA0` MAC_RX_CRC_ERR, the clock registers and
ACE keep their addresses.

### `inferno_regs.v` (0x100)

| Off | Name | Access | Meaning |
|---|---|---|---|
| 0x00 | ID | RO | `0x4F494E32` ("OIN2": 16-word table rows, latency modes) |
| 0x04 | CTRL | RW | bit 0 TX enable, bit 1 RX enable, bits 14:8 columns played per frame (default 64); bit 31 RO 1 (was FORMAT; the AES67 value 0 is retired, writes ignored) |
| 0x08 | TBL_ADDR | RW | table word address, 10 bits (below); each TBL_DATA access adds one |
| 0x0C | TBL_DATA | RW | the table word at TBL_ADDR |
| 0x10 / 0x14 | TX_PENDING / TX_RUNNING | RO | flows with a packet waiting / started |
| 0x18 | MEDIA_FS | RW | media rate in Hz, 48000 (default) or 96000 (port 0's drives the desk clock) |
| 0x1C / 0x20 | SET_MSEC / SET_MSUB | RW | loaded with SET_MEDIA by `aoip_regs` CLOCK_CMD bit 1 |
| 0x24 / 0x28 | MEDIA_SEC / MEDIA_SUB | RO | the media clock now |
| **0x2C** | **LATENCY** | RW | bits 1:0 mode (0 = 0.5 ms default, 1 = 1 ms, 2 = 2 ms, 3 = 5 ms; a mode above the build's MAXLAT is refused, the field keeps its value); bit 8 MANUAL (use LINK_OFFSET); bits 23:16 TX_TS_ADVANCE (frames) |
| 0x30 | LAT_STATUS | RO | bits 10:0 playout offset in use (frames), bits 21:16 automatic fpp |
| 0x34 | JB_GEOM | RO | bits 15:0 jitter-buffer frames, bits 23:16 columns, at the current rate (256 × 64 / 512 × 32) |
| 0x38 | CAPS | RO | bits 7:0 MAX_CH (64), 15:8 channels per flow (32), 23:16 TX flows (16), 31:24 RX flows (16) |
| 0x3C | CAPS2 | RO | bits 7:0 MAX_FPP (32), bits 9:8 MAXLAT (the largest latency mode built, 3 = 5 ms), bits 31:16 TX ring frames (128) |
| 0x40…0x5C | counters | RO | TX_PACKETS, TX_OVERRUNS, RX_OK, RX_BAD, RX_LATE, RX_EARLY, RX_UNDERRUNS, RX_TS_GAPS |

Table words (TBL_ADDR; block RAM, the word at TBL_ADDR is ready one clock after the address changes):

| Address | Contents |
|---|---|
| 0x000 + 16f + w | **TX flow f**: w0 CTRL {TTL 31:24, TOS 23:16, channels − 1 12:8, fpp 7:2 (0 = automatic), 32-bit 1, enable 0} (**write w0 last**: it (re)starts the flow); w1 dst MAC[47:32]; w2 dst MAC[31:0]; w3 dst IP; w4 {src port, dst port}; w5…w12 channel map, one byte per flow channel (byte c of word 5 + c / 4: column 5:0, bit 7 = silence); w13…w15 spare |
| 0x100 + 16f + w | **RX flow f**: w0 {bps 18:16, channels 13:8, enable 0}; w1 dst IP (match); w2 dst port (match); w3 SHIFT (signed frames); w4…w11 channel map (32 bytes: column 5:0, bit 7 = ignore); w12…w15 spare |
| 0x200 + 16f + w | **RX flow f status** (RO): w0 source IP, w1 source port, w2 packets, w3 latency peak in frames (write: clear) |

The flow configuration registers of `inferno_tx` follow the table and are not cleared by the engine's reset, so
flows written while the port is still switching mode are kept (found by `tb_top_inferno`).

### `cpu_pktq.v` (0x200)
Unchanged: ID `OCQ1`; CTRL (TX, RX, all-multicast, drop 239/8 except 239.255.255.255, promiscuous, FLUSH); TX_FREE,
TX_DATA, TX_SEND {tag, TS_REQ, length}, TX_STATUS, TX stamp; RX_STATUS, RX_LEN, RX stamp, RX_DATA, RX_POP; counters.

### `fs_phase.v` (desk-global, 0x0100 of the window)
The desk audio clock (the PRO1's CS2000 → DCM frame clock; on the ECP5 board `clk_audio` / 1024) against the media NCO,
for the desk-clock loops of §9.3. At each desk frame edge (synchronised, 2–3 clocks late, constant) it latches
`media_ts` and the clocks since the last `frame_strobe`; the AMU writes SNAP and reads one coherent set.

| Off | Name | Access | Meaning |
|---|---|---|---|
| 0x00 | ID | RO | `0x4F465031` ("OFP1") |
| 0x04 | CTRL | RW | bit 0 enable (count desk edges); write bit 1 = 1: clear the count |
| 0x08 | SNAP | WO | copy the latches into the snapshot |
| 0x0C | DESK_FRAMES | RO | desk edges counted |
| 0x10 / 0x14 | MEDIA_TS / TICKS | RO | `media_ts` at that edge, clocks since its strobe |
| 0x18 / 0x1C | PERIOD / DESK_PERIOD | RO | clocks between the last two media strobes / desk edges |

Phase = DESK_FRAMES − (MEDIA_TS + TICKS / PERIOD) (mod 2³²), resolution one clock (1/1042 frame at 48 kHz).
`tb/tb_fs_phase.v`: a desk clock 100 ppm slow reads −0.0960 frames over 960 media frames (want −0.0960). In
`top_ecp5.v` it sits at 0x0100 and measures `clk_audio` / 1024; `make sim` runs `tb_fs_phase` and `tb_ptp_servo`.

## 7. Latency in hardware terms

| Term | FPGA |
|---|---|
| send lag after the packet is due | ≤ 20 clocks to start (the pick → stamp pipeline); it then waits behind the other flows due at the same frame (64 ch at 0.5 ms: 4 × 459 B = 147 µs ≈ 7 frames) and at most one CPU frame (≤ 123 µs) |
| RX decode → playable | the frame's end + one clock a sample of commit (≤ 10 µs) |
| playout | at exactly `media_ts − offset`, ± 0 frames, one clock after the frame strobe |

Measured in `tb_infcap` (TX → MAC → wire → MAC → RX, no switch): arrival latency (media time at arrival − T) ≤ 13–14
frames at 0.5 ms, ≤ 25–27 at 1 ms, ≤ 50 at 2 / 5 ms with fpp 32 (27 with fpp 16): all inside the offsets with margin.

- One way, analogue in on this desk → a Dante receiver: L + 1 frame (the SMIB slot write). Dante → this desk: L.
- These are model numbers until the W19 board test.

## 8. Resources

### Spartan-3 RX resource optimization (2026-11-09)

Out-of-context `inferno_rx`, default parameters, Yosys 0.52 (`fee39a3284c90249e1d9684cf6944ffbbcbb8f90`),
`xc3s` mapping; baseline RTL is commit `ac2cf9b`. These are synthesis estimates, not an ISE MAP/PAR result or a
complete Pro1 image. The historical ECP5 and ISE totals below predate this change.

| RX resource | Baseline | Optimized |
|---|---:|---:|
| RAMB16 blocks | 30 | 29 |
| LUT1..LUT4 cells | 1,553 | 1,477 |
| Estimated logic cells (Yosys) | 1,126 | 945 |
| Flip-flops | 1,088 | 1,088 |
| MUXF5..MUXF8 cells | 359 | 383 |
| RAM16X1D cells | 240 | 240 |

One staging bank replaces two: the preceding packet's commit consumes one sample per clock, while the next
packet first traverses its header and then writes at most one sample per two clocks. The writer therefore
cannot overwrite an unread staged sample. The channel maps remain double-buffered because the commit reuses
them across frames. A dedicated back-to-back-packet test checks actual read/write overlap and all 960 output
samples across two differently mapped flows. No channel count, sample precision, buffer latency mode, packet
capacity or external interface was reduced.

The serial seconds-times-sample-rate calculation now shifts its existing seconds register each cycle instead
of using a variable barrel shifter. It retains the same cycle count and modulo-32-bit result; timestamp tests
include all-one seconds and 32 pseudorandom seconds values.

Reproduce the RX mapping from the repository root (run identically against the baseline and optimized RTL):

```sh
yosys -Q -T -p 'read_verilog -Irtl rtl/inferno_rx.v; synth_xilinx -family xc3s -top inferno_rx -noiopad -noclkbuf; stat'
make sim
```

Final Spartan-3 ISE placement, routing and 50 MHz timing still need validation. The increase in dedicated mux
cells is included above rather than treating every logic resource as improved.

### RAM per port (block RAM, ECP5 DP16KD = 18 kbit; buffers sized for the worst case, the 5 ms mode)

| Memory | Size | DP16KD |
|---|---|---|
| TX sample ring (NR 128 frames × 64 ch × 24 bit) | 192 kbit | 12 |
| TX flow table (256 × 32, two read ports) | 2 × 8 kbit | 2 |
| RX jitter buffer (256 frames × 64 ch × 24 bit; 512 × 32 at 96 kHz) | 384 kbit | 24 |
| RX frame tags (512 × 32, two read ports) | 2 × 16 kbit | 2 |
| RX staging (2 × 512 × 24) | 24 kbit | 2 |
| RX flow configuration (256 × 32, two read ports) | 2 × 8 kbit | 2 |
| **Inferno engine** | | **44** |
| CPU packet queue (TX 2 KB, RX 8 KB, descriptors) | ≈ 82 kbit | 5 |
| ACE engine (same port) | | 3 |
| **Port total** | | **52** |

What each latency mode needs if a build were sized for it alone (same 64 columns):

| mode | RX buffer frames needed (offset + fpp, rounded to 2ⁿ) | RX buffer | TX ring frames (≥ 2 fpp + 8) | TX ring | Inferno DP16KD (with tables, tags, staging) |
|---|---|---|---|---|---|
| 0.5 ms | 24 + 8 → 32 | 48 kbit, 3 | 24 → 32 (fpp ≤ 8) | 48 kbit, 3 | ≈ 14 |
| 1 ms | 48 + 16 → 64 | 96 kbit, 6 | 40 → 64 (fpp ≤ 16) | 96 kbit, 6 | ≈ 20 |
| 2 ms | 96 + 32 → 128 | 192 kbit, 12 | 72 → 128 (fpp ≤ 32) | 192 kbit, 12 | ≈ 32 |
| 5 ms (built) | 240 + 16 → 256 | 384 kbit, 24 | 128 | 192 kbit, 12 | **44** |

The default build is for the worst case (5 ms). **`top_ecp5` parameter `MAXLAT`** (0…3 = 0.5 / 1 / 2 / 5 ms) sets NF /
NR / MAX_FPP to the column of this table; `LATENCY` then refuses the modes above it and `CAPS2` bits 9:8 report it.
On the XC3S5000 (RAMB16, with single-RAMB16 tables and tags) the Inferno engine takes 41 / 29 / 17 / ≤ 11 blocks for
5 / 2 / 1 / 0.5 ms (docs/xc3s5000.md §2).

### Synthesis (Yosys 0.52 `synth_ecp5`, nextpnr-ecp5 0.7, speed 6, seed 1)

| Build | LUT4 | FF | DP16KD | Other | Fit / Fmax |
|---|---|---|---|---|---|
| **1 port** (Inferno + ACE + CPU queue) + HOST, LFE5U-25F (`syn/report-1port-25k.txt`) | 7 961 LUT4 + 926 PFUMX + 106 L6MUX; nextpnr 12 387 COMB (51.0 %) | 5 778 (23.8 %) | **56 / 56 (100 %)** | 136 DPR16X4, 1 MULT18 | fits; **Fmax 55.8 MHz** vs 50 MHz, met |
| **6 ports, 2 with Inferno** (A, B: 128 ch each way) + 4 AES50 / ACE + HOST, LFE5U-85F (`syn/report-6port-85k.txt`) | 21 412 LUT4 + 2 468 PFUMX | 15 518 | **120 / 208** | 464 DPR16X4, 2 MULT18 | fits: nextpnr 39.8 % COMB, 57.7 % DP16KD; **Fmax 52.7 MHz**, met |
| 6 ports, all 6 with Inferno (Yosys only) | 43 749 LUT4 + 4 973 PFUMX | 30 475 | **316** | 816 DPR16X4 | does **not** fit any ECP5 (the 85F has 208 DP16KD); LUTs would fit the 85F |

Per module (Yosys, not flattened, one port): `inferno_rx` 2 473 LUT4 / 2 134 FF / 30 DP16KD; `inferno_tx` 2 081 /
1 102 / 14; `inferno_regs` 920 / 96 / 0; `cpu_pktq` 1 262 / 677 / 5. Before this revision (48 columns, 8-word table
rows, AES67 + Inferno + ACE) the 1-port design was 9 587 LUT4, 6 267 FF, 36 DP16KD at 52.4 MHz: removing AES67 and
moving the flow tables into block RAM saved about 1.8 k LUT4 while the buffers for 64 channels × 5 ms added 20 DP16KD.

- **The 1-port design fills all 56 block RAMs of the 25F.** Anything more on that part (a second Inferno port, a
  bigger CPU queue) needs a smaller build (above) or the 45F / 85F.
- **LFE5U-45F (108 DP16KD):** one Inferno port + five ACE / AES50 ports fits (≈ 52 + 5 × 3 + 4 = 71); two full
  Inferno ports (120) do not, unless the second is built for ≤ 1 ms (Inferno ≈ 20 + CPU queue 5 + ACE 3:
  52 + 28 + 4 × 3 + 4 ≈ 96).
- The TX flow pick is registered (the first 6-port run closed at 47.1 MHz on pending → pick → timestamp borrow);
  the 1-port critical path is now the RX judge (packet time → early / late compare → commit start).
- **Spartan-3 XC3S5000** (104 RAMB16), ISE 14.7 runs in docs/xc3s5000.md: 1 port 53 RAMB16, **two Inferno ports at
  5 ms 102**, six ports with Inferno on A / B 114 at 5 ms (does not fit) and **90 at `MAXLAT=2`**; all three
  configurations meet 50 MHz at speed grade -4.

## 9. The control plane on the AMU (based on upstream Inferno)

### 9.1 Shape
- **One native C daemon, `inferno-portd`, one instance per port in mode 1** (project rule: native C/C++, no Node.js,
  no Qt; upstream's Rust binary is not deployed on the AMU). Its protocol code is written from upstream Inferno's
  modules at the pinned commit (GPL-3.0, same licence as ours):

| Upstream module (inferno_aoip/src) | Role | On the AMU (C) | FPGA side |
|---|---|---|---|
| `protocol/proto_arc.rs`, `device_server/arc_server.rs` | ARC (UDP 4440): device / channel names, subscriptions, flow listing | `arc.c` | — |
| `protocol/proto_cmc.rs`, `cmc_server.rs` | CMC (UDP 8800) | `cmc.c` | — |
| `protocol/flows_control.rs`, `flows_control_server.rs` | flow requests (UDP 4455): create / update / delete TX flows (fpp, bps, channels, destination) | `flowctl.c` | writes the TX flow table (w1…w12, then w0); fpp from the request or 0 = automatic |
| `flows_tx.rs` | TX flow bookkeeping, keepalive expiry (4 s), ARP refresh | `flows_tx.c` | the packetiser itself is `inferno_tx` |
| `channels_subscriber.rs`, `flows_rx.rs` | subscribing to other devices' channels: flow requests to them, RX sockets, keepalives `13 37` every 250 ms | `subscriber.c`, `flows_rx.c` | writes the RX flow table; keepalives go to the source in the RX status words |
| `tx_multicasts.rs` | multicast bundles (239.255.x.y : 4321) | `mcast_tx.c` | a TX flow with a multicast destination, automatic fpp |
| `info_mcast_server.rs`, `protocol/mcast.rs` | info / heartbeat multicasts (8700 / 8702 / 8708) | `info.c` | — |
| `mdns_server.rs` | `_netaudio-arc/-cmc/-chan/-bund` records, TXT incl. `latency_ns`, `fpp=32,2` | `mdns.c`: a small responder **per port** (each port is host `<DESK>-<n>`; avahi serves one host name) | `latency_ns` = the LATENCY mode |
| `saved_settings.rs`, `state_storage.rs` | persistent names, subscriptions, multicasts | a file per port next to `omc-console.prefs` | — |
| `media_clock.rs` + upstream's statime fork (PTPv1 follower, PTPv2) | the clock | `ptpd`: PTPv1 follower servo and leader role, SFD stamps from `cpu_pktq` | `PTP_INCR` / `MEDIA_INC` / steps / `SET_MEDIA` + `SET_MSEC` / `SET_MSUB` |

- **Identity per port:** name `<DESK>-<n>`, device id = EUI-64 of the port's MAC, channel names `01`…`64`
  (`01`…`32` at 96 kHz).
- **Latency:** the console's per-port latency setting (0.5 / 1 / 2 / 5 ms) writes `LATENCY`; mDNS advertises it as
  `latency_ns`; subscriptions request flows at it.
- **Mapping to the PRO1's channels:** controller channel k ↔ port column k − 1; routing (`ProControl`, SMIB slots)
  addresses the columns. docs/68 §2 has the desk side.

### 9.2 Networking through `cpu_pktq`: a netdev
- **Each port is a Linux network interface** (`aoip0`…). A small kernel driver moves frames between `cpu_pktq` and
  sk_buffs and reports the SFD stamps as hardware time stamps (`SO_TIMESTAMPING`); the PHC (`/dev/ptpN`) of every
  port is the one desk clock.
- Linux then supplies ARP, IGMP, DHCP / link-local and UDP sockets; the daemon binds to its port
  (`SO_BINDTODEVICE`). The audio never touches the stack. TX flow source ports: the daemon reserves them with a bound
  socket (upstream binds an ephemeral port per flow).
- Control traffic is kB/s; PIO over the local bus is enough. `cpu_pktq` drops 239/8 media of other devices.
- Fallback without a driver: UIO / `/dev/mem` with a minimal ARP / IGMP / UDP layer in the daemon.
- The FPGA → AMU interrupt wiring is unknown, so the queue is polled (1 ms poll: at most 1 ms on control messages;
  PTP is stamped in hardware).

### 9.3 Clock roles (docs/68 §3)
**Constraints (2026-10-14):**
- **Upstream has no PTPv1 leader.** It cannot be a PTPv1 (Dante) clock leader: the README lists the
  leader as PTPv2 only, via Statime, and "PTPv1 leader" is on its to-do list. Desk-as-leader on a
  Dante network is therefore **our own work** (milestone I7): PTPv1 leader messages and BMCA fields.
  Until it exists, the desk must be a **follower** whenever a Dante device is on the port's network.
- **The NCO is not the desk clock.** On the PRO1 the desk sample clock is the hardware audio clock
  (CS2000 → DCM), not the `ptp_clock` NCO running on the 50 MHz crystal.
  - **Leader:** the servo must lock the NCO to the desk frame clock.
  - **Follower of an Inferno port:** the servo's frequency must steer the CS2000 (its reference from
    the FPGA, or its ratio over the AMU). Steering only the NCO lets the Inferno port slip against
    the rest of the desk.
- **One leader per PTP domain.** Ports on the same network share a domain. Only one desk port may
  lead it; the others stay passive. A desk that follows port X must never lead the network X is on.

- **Follower of port X:** Sync RX stamp t2 = `RX_TS`, Delay_Req with TS_REQ (t3 = `TX_TS`), the servo trims
  `PTP_INCR` / `MEDIA_INC`; every other Inferno port leads on the desk time.
- **Leader:** every Inferno port is a PTPv1 leader (Sync two-step, Follow_Up with the TX stamp, Delay_Resp).
- **Loading the media clock:** `SET_MEDIA` = T mod 2³², `SET_MSEC` / `SET_MSUB` = T / fs, T mod fs, then
  `CLOCK_CMD` bit 1. A step re-anchors every TX flow (w0 rewrite).

**The control law (inferno-portd `clock.c`, 2026-10-19).** One daemon per desk owns the clock (`clock owner`): the
follower port's when the desk follows an Inferno port, else the first Inferno port's.
1. **PTP loop (follower):** offset o = local − leader from t1…t4; a PI servo, kp = 0.7 ppb/ns, ki = 0.3 ppb/(ns·s)
   (natural frequency ≈ 0.09 Hz, damping ≈ 0.64), a frequency estimate from the first two samples, a step above 1 ms
   (then 10 ms). The frequency f (ppb) goes to PTP_INCR = 0x14000000 × (1 + f) and MEDIA_INC = PTP_INCR × fs × 2⁸ / 10⁹,
   both sigma-delta dithered every 1 ms poll (their averages are exact). The media clock is loaded from the time of day
   after every step and when it is 2 frames off (checked each second). `tb/tb_ptp_servo.v` runs the same law (faster
   gains) on the RTL: one step, ≤ 0.8 ns, PTP_INCR at −50 ppm, media within 0.22 frames of tod × fs.
2. **Audio loop:** phase φ = desk frames − media frames from `fs_phase.v`, every 125 ms; a type-2 PLL, ωn = 2π·0.02 Hz,
   ζ = 0.7: u = kp·(φ − φ₀) + ∫ki·(φ − φ₀), kp = 2ζωn/K, ki = ωn²/K, K = fs × 10⁻⁶ frames/s per ppm (48 kHz: 3.7 ppm per
   frame, 0.33 ppm per frame-second), u limited to ±250 ppm and 5 ppm/s (no audible pitch steps). φ₀ = the phase when the
   loop starts (it holds the phase, not a sample index).
   - **Follower (PRO1):** the CS2000 ratio R = R₀ × (1 − u·10⁻⁶), 12.20 fixed point, sigma-delta dithered (one LSB =
     2⁻²⁰ = 0.48 ppm at R₀ = 2.0); written as Freeze (0x05 ← 0x09), Ratio 0x06…0x09 MSB first, unfreeze (0x05 ← 0x01)
     through the IO command engine (`+0xC0` = reg << 8 | data, `+0xC8` strobe, `+0xC4` bit 31 busy, echo checked within
     2 ms) by `cs2k_steer`: one write per 125 ms toward the target, moving the commanded ratio ≤ 0.625 ppm per write
     (5 ppm/s, `DCLK_SLEW_PPM_S`; ≤ 3 LSB per write at R₀ = 2.0), so the ratio **never steps** (PLAN D15, 2026-10-20).
     A media load gives the loop a new phase reference (it keeps its correction) instead of a reset, taken from the
     first `fs_phase` snapshot whose edge count has moved past the one read right after the load (until a desk edge
     follows the load, the latch still holds the old count). R₀ is a daemon option (`--cs2k-r0`, default 0x00200000): **the PRO1's CS2000 reference
     and ratio are not known (hardware)**.
   - **Leader:** the CS2000 stays at R₀ and the loop drives the time of day instead: f = +u × 1000 ppb, so the PTP time
     the leader sends follows the desk's own sample clock.
   - **Source lost** (PTP leaves the follower state, or no Sync for 4 s): **HOLDOVER** keeps the ratio and the time-of-day
     rate in force; **GO LEADER** glides the CS2000 to R₀ at ≤ 5 ppm/s, the time of day following the desk.
   - **Source back (automatic, user 2026-10-20):** the ratio glides back to the last tracked value; the audio loop
     restarts without a bump once PTP has held its lock for 2 s and that glide is done. After a long GO LEADER the PTP
     servo may still slew or step the network time once (the desk clock does not move).
   - `clockfail holdover|leader` applies at run time (at once while the source is lost); `master` and `internal` are
     read as leader, other words are refused.
3. Leader fields (ours, open question 6): subdomain `_DFLT`, identifier `DFLT`, stratum 4, variance 0xF060,
   `preferred` = 1 for `ptp leader` (it then wins the comparison as statime maps preferred to priority1 64),
   Sync every 2⁻² s (`--log-sync`), two-step, TTL 1, DSCP CS7.

### 9.4 `inferno-portd` (written 2026-10-19: host-tested, no hardware)
`inferno-portd/` (C11, libc only; `make` x86, `make ppc` static PowerPC e300c3 glibc for modern kernels, not a Pro1 binary;
`make amu MUSL_CC=<PowerPC musl-gcc>` the Pro1 AMU build: MPC8315, Linux 2.6.29, see `check-amu` below). One process per
Inferno port. Its pieces and where they come from:

| File | Does | From |
|---|---|---|
| `hw_devmem.c` / `fpga_model.c` | the register backend: the IO FPGA window through `/dev/mem`, or a register-exact software model of the port (aoip_regs, inferno_regs + TX packetiser / RX depacketiser with a test pattern per column, cpu_pktq, ptp_clock, fs_phase, a CS2000 engine) | our RTL's documented behaviour |
| `pktq.c`, `net.c` | the CPU frame pump and a minimal stack on it: Ethernet, ARP (probe, announce, defend), IPv4 link-local (RFC 3927) or static, UDP, IGMPv2, ICMP echo; TX paced 8 frames per 1 ms poll (the far end's CPU queue holds 16) | the §9.2 fallback ("a minimal ARP / IGMP / UDP layer in the daemon"); no TAP: this container has no `/dev/net/tun` |
| `mdns.c` | a per-port responder (`_netaudio-arc/-cmc/-chan/-bund`, the `in-addr` reservations, A records) and the resolver | upstream `mdns_server.rs` / `mdns_client.rs` / searchfire |
| `arc.c` | ARC 4440 (counts, names, RX / TX channels with pagination, renames, TX / RX flow listings, multicast flow create / delete, subscribe / unsubscribe, 0x1100 / 0x1102 / 0x2320 / 0x3300) and CMC 8800 | upstream `arc_server.rs`, `proto_arc.rs`, `cmc_server.rs` |
| `flows_tx.c` | flow requests 0x0100 / stop 0x0101 / update 0x0102 → the TX flow table (w1…w12, then w0), keepalive expiry 4 s, admission control (95 Mb/s, §5), multicast bundles 239.255.x.y:4321 after an mDNS address check | `flows_control_server.rs`, `flows_tx.rs`, `tx_multicasts.rs` |
| `subscriber.c` | our RX channels: mDNS resolution, multicast bundles first, reuse / growth of flows by update, new flows of up to min(nchan, 32) channels, the RX flow table, keepalives 13 37 every 250 ms to the source in the status words, the 2 s / 6 s orphan rule, SHIFT = the transmitter's latency above ours (refused, status 0x1A, beyond `JB_GEOM`) | `channels_subscriber.rs`, `flows_rx.rs`, `flows_control.rs` |
| `info.c` | board / product / network info, the 1 s heartbeat (frequency offset, RX latency), channel-change notifications | `info_mcast_server.rs`, `mcast.rs` |
| `ptp.c` | PTPv1: follower, **our own leader** (two-step Sync + Follow_Up, Delay_Resp), the BMCA leader choice (1588 best master clock algorithm) | statime fork `messages_v1`, 1588-2002 octet positions |
| `clock.c` | the PI servo → PTP_INCR / MEDIA_INC (sigma-delta: the nominal 48 kHz MEDIA_INC is 4 123 168.6), steps, media loads; the audio loop → the CS2000 ratio | our own (§9.3 below) |
| `ctl.c`, `settings.c` | the control socket (OMC's per-port settings; `inferno-portd ctl <socket> <command>`), the state file | |

**Identity:** device `<DESK>-<n>`, factory name `OPS-<MAC>`, device id = EUI-64 of the MAC, channels `01`…`64`,
advertised `nchan=16` (option `--nchan`), `fpp=32,2`, `latency_ns` = the port's mode. **Control lines** (OMC
`PortdConfig`, aoip-ports.h): `deskname`, `rate`, `channels`, `latency 0.5|1|2|5`, `ptp leader|follower [steer]|auto|off`,
`clock owner|none`, `clockfail`, `sub <k> <ch>@<dev>`, `unsub`, `txname`, `rxname`, `mcast`, `mcastdel`, `status`.

**Tests** (`make check`, `make check-ppc`):
- `test_unit` (85 checks): ARC / CMC / flow-control packets byte by byte against upstream's layouts; PTPv1 Sync /
  Follow_Up / Delay_Resp octet by octet; the BMCA comparison; the servo (+50 ppm, 3 ms off: one step, ≤ 200 ns,
  −50 000 ppb ± 100); the audio loop (+30 ppm through the dithered CS2000 ratio: ≤ 0.05 frames); the desk clock on the
  model (fs_phase + CS2000 engine, desk +40 ppm: desk rate = media rate within 1 ppm, phase ≤ 0.1 frames, media clock
  = time of day × fs within 1.5 frames over 600 s).
- `test_inproc` (29 checks, virtual time, two models on an in-memory wire, 5 µs each way): link-local addresses; PTP
  lock (offset < 100 ns, delay 5 µs ± 0.3); 64 channels in 4 flows of 16, every sample right, every column its channel;
  the reverse way; a TX rename; a multicast bundle; a latency change; a stopped flow; the election of two would-be
  leaders.
- `e2e.sh` (real time, two daemons in two network namespaces on a veth pair; no LAN traffic): PTP lock (software
  stamps, ~10 µs), 64 channels B ← A with ~7 M samples right and 0 wrong, 8 channels A ← B, the bundle, the state file.
- `test_clock` (107 checks, 2026-10-20): source loss and return on two models with fs_phase and the CS2000 engine:
  holdover gaps from 4 to 12 s (no stuck holdover), GO LEADER for 20 s and 60 s and the return, `clockfail` at run
  time; no CS2000 write moves the ratio by more than the slew step.
- `check-ppc`: the same unit / in-process / clock tests built for the e300c3 and run big-endian under qemu-ppc-static.
- `check-amu` (in `make check` when `MUSL_CC` is set, else `SKIP amu`): the Pro1 build. Debian's static PowerPC glibc
  needs Linux >= 3.2 and the AMU runs 2.6.29, so `amu` links musl instead (musl 1.2.6, static, over
  `gcc-powerpc-linux-gnu`, hard float; `-O2 -static -mcpu=e300c3 -mlong-double-64 -no-pie`, objects in `build/amu`).
  It runs the `check-ppc` tests and `inferno-portd --port 0` (must exit 2) under `qemu-ppc -r 2.6.29 -cpu e300c3`
  (`AMU_QEMU`), then `AMU_CHECK` (a static-binary checker script) when set. `-r` only fakes the kernel version (qemu
  still runs the glibc `ppc` build, so it cannot catch this): this is not a real-AMU run. The source must not include
  `<linux/...>` headers (the musl sysroot has none; `check-unit` greps for them).
- `status` keys include `clock_fail=`, `cs2k_ppm=` and `ptp_leader=` (was `ptp_master=`).

## 10. Milestones

| # | Milestone | Status |
|---|---|---|
| I1 | Packet format, golden model, cross-check against upstream Inferno (`make xcheck`: C model of upstream + source pin) | **done** |
| I2 | `inferno_tx` / `inferno_rx` / `cpu_pktq` / `inferno_regs` in `aoip_port` mode 1; `tb_inferno`, `tb_top` | **done** (simulation) |
| I2b | AES67 removed; 64 columns, 32 channels per flow, latency modes 0.5 / 1 / 2 / 5 ms, MTU-cut fpp, 96 kHz geometry; `tb_infcap` (48 / 64 ch, every mode), `tb_top_inferno` (2 ports, 128 ch, run-time mode changes) | **done** (simulation, ECP5 synthesis) |
| I3 | Resource: register the TX pick; the XC3S5000 build; 6-port ECP5 with all ports Inferno is out of reach (316 DP16KD) | **done** (docs/xc3s5000.md: 2 ports at 5 ms, 6 ports at 2 ms, 50 MHz met) |
| I4 | AMU: `cpu_pktq` netdev driver + PHC | open |
| I5 | AMU: `inferno-portd` in C from upstream's modules (§9), per-port mDNS, prefs file; `ptpd` follower | **done on the host** (§9.4: the model, two namespaces, PowerPC under qemu); the AMU run is I4 / I6 |
| I6 | ECP5 board + RMII PHY (W19): Dante Controller sees `<DESK>-1`; RX from / TX to Inferno and a Dante device; PTP lock | open (hardware) |
| I7 | PTPv1 leader role, holdover, CS2000 reference from the media NCO | **leader, election, holdover, the CS2000 law: done in software** (§9.3, `fs_phase.v`); the CS2000's bus and reference need the board |
| I8 | 16-bit TX with TPDF dither; 96 kHz flows against a Dante device | open |
| I9 | XC3S5000 port (XST / ISE), the PRO1 pin map | open (hardware) |

## 11. Open questions (protocol points upstream does not settle)

1. **Byte 0 = 2.** Upstream marks it "???". We send 2 and ignore it on receive.
2. **Timestamp reference.** Upstream stamps `next_ts + clock_offset_samples` (−0.5 ms) with a commented
   `wrapping_sub(fpp)`; whether Dante hardware stamps the first sample, the packet end or an offset is not confirmed
   by a capture. `TX_TS_ADVANCE` and the RX SHIFT cover any of them; measurable with one Dante device.
3. **Epoch.** Media time = PTP time × fs (upstream `media_clock.rs`); whether Dante devices add an offset is
   unverified.
4. **fpp alignment** for non-power-of-two fpp (ours: the next frame); receivers use the timestamp.
5. **Channels per flow from Dante transmitters:** we accept ≤ 32; Dante's maximum is not in upstream.
6. **PTPv1 leader fields** (stratum, identifier, preferred flag, interval) for the desk-leader role: from upstream's
   statime fork or captures.
7. **96 kHz:** ARC / CMC fields for 96 kHz flows and Dante's fpp at 96 kHz are untested.
8. **16-bit flows:** refused until TX dither exists (upstream dithers 16- and 24-bit output).
9. **IP identification / DSCP:** ours count per port; EF (46) for audio and CS7 (56) for PTP are the daemon's
   defaults, not captured.
10. **Flow update (0x0102):** our table rewrite restarts the flow (as upstream re-bootstraps).
11. **nchan advertised:** 16 (64 channels at 0.5 ms need ≥ 11 per flow); what Dante receivers do with more than 8 is not
    known. Our receiver requests min(nchan, 32) per flow (upstream 8).
12. **Media load precision:** the media clock is loaded to the frame (±1 frame against tod × fs; the NCO's fraction is
    not settable). A load at a programmed time-of-day instant would make it exact (an RTL change).
13. **PTPv1 Sync interval and the stamps of a Dante leader** (0.25 s here) and Dante's Delay_Req expectations: captures.
14. **DHCP:** not implemented (link-local or a static address); Dante devices use both.
