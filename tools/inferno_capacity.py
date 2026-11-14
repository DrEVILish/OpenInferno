#!/usr/bin/env python3
"""Inferno channel capacity of one 100 Mb/s port per latency mode and flow width (fpga/docs/inferno.md section 5).

Written for OpenProSeries (W16). Licence: GPL-3.0.

Model: 24-bit samples; wire cost per packet = 75 bytes (preamble + SFD 8, Ethernet 14, IPv4 20, UDP 8, Inferno media
header 9, FCS 4, inter-packet gap 12) + fpp x nch x 3; N channels in floor(N / nch) full flows plus one partial flow;
budget 95 % of 100 Mb/s. "nominal" uses the latency mode's nominal fpp (latency / 3 rounded to a power of two, 8 / 16 /
32 / 64), "design" the fpp inferno_tx really sends: at most 32 and cut to the largest power of two whose payload fits
one 1500-byte IPv4 packet (fpp x nch x 3 <= 1463), as rtl/inferno_lat.vh.

    python3 -I tools/inferno_capacity.py [--fs 48000] [--ch 64]   (--ch: the port's columns for the load column)
"""
import argparse

OVERHEAD = 75
LINK = 0.95 * 100e6
PAYLOAD_MAX = 1472 - 9


def mtu_fpp(nchb):
    return 32 if nchb <= 45 else 16 if nchb <= 91 else 8


def load(n, fpp, cpf, fs, bps=3):
    full, rem = divmod(n, cpf)
    bits = full * (OVERHEAD + fpp * cpf * bps) * 8 * fs / fpp
    if rem:
        bits += (OVERHEAD + fpp * rem * bps) * 8 * fs / fpp
    return bits


def capacity(fpp, cpf, fs):
    n = 0
    while load(n + 1, fpp, cpf, fs) <= LINK:
        n += 1
    return n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fs", type=int, default=48000)
    ap.add_argument("--ch", type=int, default=64)
    a = ap.parse_args()
    hi = a.fs > 50000
    print("| latency | nominal fpp | ch / flow | payload | max ch (nominal) | design fpp | max ch (design) | Mb/s at %d ch |" % a.ch)
    print("|---|---|---|---|---|---|---|---|")
    for name, nominal, auto in (("0.5 ms", 8, 8), ("1 ms", 16, 16), ("2 ms", 32, 32), ("5 ms", 64, 32)):
        if hi:
            nominal, auto = nominal * 2, min(32, auto * 2)
        for cpf in (8, 16, 32):
            pay = nominal * cpf * 3
            fe = min(auto, mtu_fpp(cpf * 3))
            print("| %s | %d | %d | %d B%s | %d | %d | %d | %.1f |" % (
                name, nominal, cpf, pay, " > MTU" if pay > PAYLOAD_MAX else "", capacity(nominal, cpf, a.fs), fe,
                capacity(fe, cpf, a.fs), load(a.ch, fe, cpf, a.fs) / 1e6))


if __name__ == "__main__":
    main()
