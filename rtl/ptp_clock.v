// ptp_clock.v - the desk clock as a PTP time of day plus a media clock (Inferno flow timestamps), and frame-time stamps.
//
// Provenance: written for OpenProSeries (W16) from IEEE 1588-2008 (a time of day in seconds and nanoseconds,
// event-message timestamps taken at the SFD) and RFC 7273 (a media clock as samples counted from the PTP epoch at
// the stream's rate; upstream Inferno's flow timestamps use the same count, as seconds and samples). The servo (offset and rate from Sync / Delay_Req)
// runs in software on the AMU or a softcore (OpenProSeries docs/68 section 8 "split"): this block only counts,
// steps and trims. No vendor material. Licence: GPL-3.0.
//
// tod: seconds (48 bits) and nanoseconds (30 bits); each clock adds incr (ns in 8.24 fixed point, e.g. 20 ns =
// 0x14000000 at 50 MHz). set_tod steps it. The media clock is an NCO on the same clock: media_inc =
// fs / f_clk x 2^32 (48 kHz at 50 MHz = 0x003EEA21 (96 kHz: 0x007DD441)), trimmed by the servo with the same ratio as incr; each
// carry is one sample period: frame_strobe pulses and media_ts counts. set_media loads media_ts (aligning it to
// the PTP epoch is the servo's job: media_ts = floor(tod x fs) at a known instant).
// ts_tx / ts_rx latch tod at the SFD strobes of the MAC (one-step / two-step PTP timestamps).
// The same media time is also kept as (media_sec, media_sub) = (T / fs, T mod fs), counting with media_ts and loaded
// with it by set_media from (set_media_sec, set_media_sub): the timestamp form of Inferno flow packets (inferno_tx.v;
// fs = media_fs, 48000 or 96000).
`timescale 1ns / 1ps
`default_nettype none
module ptp_clock (
    input  wire        clk,
    input  wire        rst,
    input  wire [31:0] incr,          // ns per clock, 8.24
    input  wire        set_tod,
    input  wire [47:0] set_sec,
    input  wire [29:0] set_ns,
    output reg  [47:0] sec,
    output reg  [29:0] ns,
    input  wire [31:0] media_inc,
    input  wire        set_media,
    input  wire [31:0] set_media_ts,
    output reg  [31:0] media_ts,
    output reg         frame_strobe,
    input  wire [16:0] media_fs,
    input  wire [31:0] set_media_sec,
    input  wire [16:0] set_media_sub,
    output reg  [31:0] media_sec,
    output reg  [16:0] media_sub,
    input  wire        sfd_tx,
    input  wire        sfd_rx,
    output reg  [77:0] ts_tx,         // {sec, ns}
    output reg  [77:0] ts_rx
);
    reg [23:0] frac;
    reg [31:0] nco;
    wire [24:0] f_sum  = {1'b0, frac} + {1'b0, incr[23:0]};
    wire [30:0] ns_sum = {1'b0, ns} + {23'd0, incr[31:24]} + {30'd0, f_sum[24]};
    wire [32:0] n_sum  = {1'b0, nco} + {1'b0, media_inc};

    always @(posedge clk) begin
        frame_strobe <= 1'b0;
        if (rst) begin
            sec <= 48'd0; ns <= 30'd0; frac <= 24'd0; nco <= 32'd0; media_ts <= 32'd0; media_sec <= 32'd0; media_sub <= 17'd0;
        end else begin
            if (set_tod) begin
                sec <= set_sec; ns <= set_ns; frac <= 24'd0;
            end else begin
                frac <= f_sum[23:0];
                if (ns_sum >= 31'd1000000000) begin ns <= ns_sum - 31'd1000000000; sec <= sec + 48'd1; end
                else ns <= ns_sum[29:0];
            end
            nco <= n_sum[31:0];
            if (set_media) begin media_ts <= set_media_ts; media_sec <= set_media_sec; media_sub <= set_media_sub; end
            else if (n_sum[32]) begin
                media_ts <= media_ts + 32'd1; frame_strobe <= 1'b1;
                if (media_sub + 17'd1 >= media_fs) begin media_sub <= 17'd0; media_sec <= media_sec + 32'd1; end
                else media_sub <= media_sub + 17'd1;
            end
        end
        if (sfd_tx) ts_tx <= {sec, ns};
        if (sfd_rx) ts_rx <= {sec, ns};
    end
endmodule
`default_nettype wire
