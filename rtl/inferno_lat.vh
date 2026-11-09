// inferno_lat.vh - the Inferno port's latency modes and the Ethernet MTU limit on frames per packet, shared by
// inferno_regs.v, inferno_tx.v and the testbenches.
//
// Provenance: written for OpenProSeries (W16): our own table (fpga/docs/inferno.md section 7). The MTU arithmetic
// is IEEE 802.3 / RFC 791 / RFC 768 (1500-byte IPv4 packet = 20 IPv4 + 8 UDP + 1472 UDP payload) and the 9-byte
// Inferno media header (upstream Inferno, github.com/teodly/inferno, flows_tx.rs). Licence: GPL-3.0.
//
// LATENCY mode (inferno_regs 0x2C bits 1:0):  0: 0.5 ms   1: 1 ms   2: 2 ms   3: 5 ms
//   playout offset (frames) = latency x fs: 24 / 48 / 96 / 240 at 48 kHz, 48 / 96 / 192 / 480 at 96 kHz
//   automatic frames per packet (TX flows with fpp field 0): 8 / 16 / 32 / 32 at 48 kHz, 16 / 32 / 32 / 32 at
//   96 kHz (a packet time of 1/3 of the latency, at most 32 frames, Inferno's advertised maximum), then cut down to
//   what fits the MTU (inferno_mtu_fpp).
// The functions are pure combinational logic; include them inside a module.

// playout offset in frames; hi = 96 kHz (or any rate above 50 kHz)
function [9:0] inferno_lat_offset;
    input [1:0] mode;
    input       hi;
    reg   [9:0] f;
    begin
        case (mode)
            2'd0: f = 10'd24;
            2'd1: f = 10'd48;
            2'd2: f = 10'd96;
            default: f = 10'd240;
        endcase
        inferno_lat_offset = hi ? {f[8:0], 1'b0} : f;
    end
endfunction

// the automatic frames per packet of a latency mode
function [5:0] inferno_lat_fpp;
    input [1:0] mode;
    input       hi;
    begin
        case (mode)
            2'd0: inferno_lat_fpp = hi ? 6'd16 : 6'd8;
            2'd1: inferno_lat_fpp = hi ? 6'd32 : 6'd16;
            default: inferno_lat_fpp = 6'd32;
        endcase
    end
endfunction

// the largest power-of-two frames per packet (at most 32) whose payload fits one 1500-byte IPv4 packet:
// fpp x nchb <= 1463 (1472 UDP payload - 9 media header); nchb = channels x bytes per sample (3..128)
function [5:0] inferno_mtu_fpp;
    input [7:0] nchb;
    begin
        if (nchb <= 8'd45)      inferno_mtu_fpp = 6'd32;     // 32 x 45 = 1440
        else if (nchb <= 8'd91) inferno_mtu_fpp = 6'd16;     // 16 x 91 = 1456
        else                    inferno_mtu_fpp = 6'd8;      //  8 x 128 = 1024
    end
endfunction
