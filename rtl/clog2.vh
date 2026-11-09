// clog2.vh - ceil(log2(x)) as a constant macro for tools without $clog2 (ISE XST's Spartan-3 Verilog parser).
// Valid for 1 <= x <= 2^24. Provenance: written for OpenProSeries (W16). Licence: GPL-3.0.
`ifndef CLOG2_VH
`define CLOG2_VH
`define CLOG2(x) ((x) <= 1 ? 0 : (x) <= 2 ? 1 : (x) <= 4 ? 2 : (x) <= 8 ? 3 : (x) <= 16 ? 4 : (x) <= 32 ? 5 : \
    (x) <= 64 ? 6 : (x) <= 128 ? 7 : (x) <= 256 ? 8 : (x) <= 512 ? 9 : (x) <= 1024 ? 10 : (x) <= 2048 ? 11 : \
    (x) <= 4096 ? 12 : (x) <= 8192 ? 13 : (x) <= 16384 ? 14 : (x) <= 32768 ? 15 : (x) <= 65536 ? 16 : \
    (x) <= 131072 ? 17 : (x) <= 262144 ? 18 : (x) <= 524288 ? 19 : (x) <= 1048576 ? 20 : (x) <= 2097152 ? 21 : \
    (x) <= 4194304 ? 22 : (x) <= 8388608 ? 23 : 24)
`endif
