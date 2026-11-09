#!/usr/bin/env python3
"""Golden packet files for the W16 testbenches, built independently of the RTL from the public standards.

Provenance: written for OpenProSeries. Ethernet II framing, preamble / SFD and the FCS (IEEE 802.3: CRC-32, the
same polynomial zlib.crc32 computes, sent least-significant byte first), IPv4 + header checksum (RFC 791 /
RFC 1071), UDP without checksum (RFC 768). A&H ACE frames (raw 802.3 frame, length 0x00DD, 65 nibble-swapped
3-byte slots, slot 0 = 0x40 + 4n, 26 control bytes) from the public notes of P. Lindahl
(github.com/PatrLind/ah_ace_protocol), see docs/ace.md. Inferno (Dante-compatible) flow packets as upstream Inferno
builds them (github.com/teodly/inferno, commit 9767558e, flows_tx.rs transmit(); fpga/docs/inferno.md section 3).
AES67 was dropped on 2026-10-14 (user decision); its generator is gone.
Licence: GPL-3.0.

Output: one wire byte per line (hex), every frame from its first preamble byte to the last FCS byte, frames
back to back; the testbench concatenates what it sees on TXD while TX_EN is high and compares.

    golden.py frames --seed 1 --out golden/mac_frames.hex     (the MAC test's frames)
    golden.py ace --ch 48 --packets 18 --out golden/ace_48ch.hex   (ACE frames 0..17: the counter wraps once)
    golden.py inferno --packets 12 --out golden/inferno_tx.hex   (the tb_inferno flows, in send order)
    golden.py inferno --cfg cap64 --packets 16 --out golden/inferno_64ch.hex   (64 ch: 4 flows x 16 ch, fpp 8)
"""
import argparse
import random
import struct
import sys
import zlib

SRC_MAC = bytes.fromhex("020000000101")
SRC_IP = bytes([192, 0, 2, 10])


def sample(frame, ch):
    """the test pattern both sides use: 24 bits, distinct per frame and channel"""
    return ((frame * 0x010203) ^ (ch << 16) ^ 0x5A5A5A) & 0xFFFFFF


def ip_checksum(hdr):
    s = 0
    for i in range(0, len(hdr), 2):
        s += (hdr[i] << 8) | hdr[i + 1]
    while s >> 16:
        s = (s & 0xFFFF) + (s >> 16)
    return (~s) & 0xFFFF


def wire(frame_wo_fcs):
    """preamble + SFD + frame padded to 60 bytes + FCS"""
    f = frame_wo_fcs + bytes(max(0, 60 - len(frame_wo_fcs)))
    fcs = zlib.crc32(f) & 0xFFFFFFFF
    return bytes([0x55] * 7 + [0xD5]) + f + struct.pack("<I", fcs)


def nswap(b):
    """ACE payload bytes go on the wire with their nibbles swapped"""
    return ((b & 0x0F) << 4) | (b >> 4)


def ace_frame(k, ch):
    """ACE frame k (frame counter k mod 16): 64 audio slots, channels >= ch zero, no tunnelled data"""
    slots = [0x40 + 4 * (k % 16)]                                  # slot 0: the sync counter
    slots += [sample(k, c) if c < ch else 0 for c in range(64)]    # slots 1..64: channels 0..63
    payload = bytearray()
    for v in slots:
        payload += bytes(nswap(x) for x in v.to_bytes(3, "big"))
    payload += bytes(26)                                           # control byte 0x00 + 25 tunnel bytes
    assert len(payload) == 221
    return b"\xff" * 6 + SRC_MAC + struct.pack("!H", len(payload)) + bytes(payload)


# ---- Inferno (Dante-compatible) audio flows: fpga/docs/inferno.md section 3, as upstream Inferno builds them
# (flows_tx.rs transmit(): pbuff[0] = 2, pbuff[1..5] = seconds BE, pbuff[5..9] = subsec samples BE, then the samples
# interleaved with stride nch x bps, each the top bps bytes of the left-justified 32-bit sample, samples_utils.rs):
# bps 3: the 24-bit sample; bps 4: the 24-bit sample << 8. IPv4 with DSCP / TTL per flow, DF, IP id counting
# packets, UDP checksum 0 (the operating system's part upstream; RFC 791 / 768 here).
INF_FS = 48000
INF_S0 = 1760000000              # seconds of the first frame (2025-10)
INF_U0 = 47990                   # its sub-second sample: the second rolls over 10 frames in
INF_T0 = INF_S0 * INF_FS + INF_U0
# the tb_inferno flows: (flow index, fpp, bps, dst MAC, dst IP, src port, dst port, TTL, TOS, channel map)
INF_FLOWS = [
    (0, 16, 3, bytes.fromhex("020000000202"), bytes([192, 0, 2, 20]), 14400, 14336, 64, 0xB8, [0, 1, 2, 3, 4, 5, 6, 7]),
    (1, 4, 4, bytes.fromhex("01005e7f0102"), bytes([239, 255, 1, 2]), 14401, 4321, 32, 0xB8, [40, 41, None, 47, 3, 10]),
    (2, 6, 3, SRC_MAC, SRC_IP, 14402, 14338, 64, 0xB8, [44, 45]),     # to ourselves: the RX must claim it
]


def inferno_media(t, bps, chmap, fpp, sample_of):
    """the Inferno media packet (UDP payload) whose first frame has media time t (64-bit samples)"""
    p = bytearray([2]) + struct.pack("!II", (t // INF_FS) & 0xFFFFFFFF, t % INF_FS)
    for f in range(fpp):
        for ch in chmap:
            s = 0 if ch is None else sample_of(t + f, ch)
            p += (s << 8).to_bytes(4, "big") if bps == 4 else s.to_bytes(3, "big")
    return bytes(p)


def inferno_packet(flow, t, ip_id, sample_of):
    _, fpp, bps, dmac, dip, sport, dport, ttl, tos, chmap = flow
    payload = inferno_media(t, bps, chmap, fpp, sample_of)
    udp = struct.pack("!HHHH", sport, dport, 8 + len(payload), 0)
    ip = bytearray(struct.pack("!BBHHHBBH4s4s", 0x45, tos, 20 + 8 + len(payload), ip_id & 0xFFFF, 0x4000, ttl, 17, 0,
                               SRC_IP, dip))
    struct.pack_into("!H", ip, 10, ip_checksum(ip))
    return dmac + SRC_MAC + b"\x08\x00" + bytes(ip) + udp + payload


def inferno_schedule(n, flows=INF_FLOWS, t0=INF_T0):
    """the first n packets in send order: a flow with a power-of-two fpp starts at the first multiple of its fpp after
    t0 (inferno_tx.v, Inferno's bootstrap_next_ts), any other fpp at t0 + 1; a packet is due once its last frame is
    in; packets due at the same frame go out lower flow first"""
    ev = []
    for fl in flows:
        fpp = fl[1]
        x = (t0 // fpp + 1) * fpp if fpp & (fpp - 1) == 0 else t0 + 1
        for _ in range(n):
            ev.append((x + fpp - 1, fl[0], x, fl))
            x += fpp
    ev.sort(key=lambda e: (e[0], e[1]))
    return [(e[3], e[2]) for e in ev[:n]]


# tb_infcap 64-channel configuration: 4 unicast flows x 16 channels, 24 bit, fpp 8 (the 0.5 ms latency mode's
# automatic fpp), columns 16 f .. 16 f + 15, from 14400 + f to 192.0.2.20:14336 + f
INF_CAP64 = [(f, 8, 3, bytes.fromhex("020000000202"), bytes([192, 0, 2, 20]), 14400 + f, 14336 + f, 64, 0xB8,
              list(range(16 * f, 16 * f + 16))) for f in range(4)]
INF_CFGS = {"tb": INF_FLOWS, "cap64": INF_CAP64}


def inferno_frames(n, cfg="tb"):
    sample_of = lambda t, ch: sample(t - INF_T0, ch)
    return [inferno_packet(fl, t, k, sample_of) for k, (fl, t) in enumerate(inferno_schedule(n, INF_CFGS[cfg]))]


def mac_frames(seed):
    """the MAC test's frames: lengths across the padding boundary and up to 1514"""
    rnd = random.Random(seed)
    out = []
    for n in (14, 46, 59, 60, 61, 64, 200, 1514):
        out.append(bytes(rnd.randrange(256) for _ in range(n)))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["frames", "ace", "inferno"])
    ap.add_argument("--frames-txt", help="inferno: also each frame (no preamble / FCS) as one hex line (make xcheck)")
    ap.add_argument("--cfg", default="tb", choices=sorted(INF_CFGS), help="inferno: the flow set")
    ap.add_argument("--ch", type=int, default=48)
    ap.add_argument("--packets", type=int, default=3)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", required=True)
    ap.add_argument("--payload-out", help="frames: also the bytes the receiver must deliver (padded, FCS stripped)")
    ap.add_argument("--in-out", help="frames: also the frames as the transmitter is given them (no padding)")
    a = ap.parse_args()
    if a.what == "ace":
        frames = [ace_frame(k, a.ch) for k in range(a.packets)]
        assert all(len(fr) == 235 for fr in frames)
    elif a.what == "inferno":
        frames = inferno_frames(a.packets, a.cfg)
        if a.frames_txt:
            with open(a.frames_txt, "w") as f:
                for fr in frames:
                    f.write(fr.hex() + "\n")
    else:
        frames = mac_frames(a.seed)
    with open(a.out, "w") as f:
        for fr in frames:
            for b in wire(fr):
                f.write("%02x\n" % b)
    if a.payload_out:
        with open(a.payload_out, "w") as f:
            for fr in frames:
                p = fr + bytes(max(0, 60 - len(fr)))
                for b in p:
                    f.write("%02x\n" % b)
    if a.in_out:
        with open(a.in_out, "w") as f:
            for fr in frames:
                for b in fr:
                    f.write("%02x\n" % b)
    total = sum(len(wire(fr)) for fr in frames)
    print("%s: %d frames, %d wire bytes" % (a.out, len(frames), total), file=sys.stderr)


if __name__ == "__main__":
    main()
