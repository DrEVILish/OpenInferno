// inferno_regs.v - the AMU-visible configuration and status of a port's Inferno (Dante-compatible) flows, the
// latency mode, and the media clock's seconds / sub-second form (port 0's registers drive the desk clock, as
// aoip_regs.v).
//
// Provenance: written for OpenProSeries (W16); our own register map (fpga/docs/inferno.md section 6).
// Licence: GPL-3.0.
//
// 32-bit registers, byte offsets inside the block (aoip_port.v puts the block at 0x100 of the port):
//   0x00 ID          RO 0x4F494E32 ("OIN2": 16-word flow table rows, latency modes)
//   0x04 CTRL        RW bit0 TX enable, bit1 RX enable, bits 14:8 columns played per frame (default MAX_CH);
//                       bit31 RO 1: mode 1 carries Inferno flows (it was FORMAT; its old value 0 = AES67 is retired:
//                       writes are ignored)
//   0x08 TBL_ADDR    RW table word address (below); every TBL_DATA access moves it on by one
//   0x0C TBL_DATA    RW the table word at TBL_ADDR
//   0x10 TX_PENDING  RO flows with a packet waiting      0x14 TX_RUNNING RO flows started (aligned and sending)
//   0x18 MEDIA_FS    RW the media clock's rate in Hz (48000 / 96000; default 48000)
//   0x1C SET_MSEC    RW   0x20 SET_MSUB RW: loaded into the media clock's seconds / sub-second counter together
//                       with SET_MEDIA by CLOCK_CMD bit1 (aoip_regs.v); the servo writes SET_MEDIA = SET_MSEC x fs +
//                       SET_MSUB mod 2^32 so that both forms name the same instant
//   0x24 MEDIA_SEC   RO   0x28 MEDIA_SUB RO: the media clock now (seconds since the PTP epoch, samples in it)
//   0x2C LATENCY     RW bits 1:0 latency mode (0: 0.5 ms (default), 1: 1 ms, 2: 2 ms, 3: 5 ms; a mode above the build's
//                       MAXLAT (CAPS2 bits 9:8) is refused: the mode field keeps its value); bit 8 MANUAL: play at
//                       aoip_regs LINK_OFFSET (0x3C, frames) instead of the mode's offset; bits 23:16 TX_TS_ADVANCE:
//                       frames the TX timestamps are stamped before the first frame (upstream Inferno: 0.5 ms)
//   0x30 LAT_STATUS  RO bits 10:0 playout offset in use (frames), bits 21:16 automatic TX frames per packet
//   0x34 JB_GEOM     RO bits 15:0 jitter-buffer frames at the current rate, bits 23:16 columns at that rate
//   0x38 CAPS        RO bits 7:0 MAX_CH, 15:8 channels per flow (MAXFCH), 23:16 TX flows, 31:24 RX flows
//   0x3C CAPS2       RO bits 7:0 MAX_FPP, bits 9:8 MAXLAT (the largest latency mode built), bits 31:16 TX ring frames
//   0x40 TX_PACKETS  0x44 TX_OVERRUNS  0x48 RX_OK  0x4C RX_BAD  0x50 RX_LATE  0x54 RX_EARLY  0x58 RX_UNDERRUNS
//   0x5C RX_TS_GAPS  (all RO)
// Table words (TBL_ADDR, 10 bits): 0x000 + flow x 16 + w: TX flow (inferno_tx.v); 0x100 + flow x 16 + w: RX flow
//   configuration, 0x200 + flow x 16 + w: RX flow status (inferno_rx.v). The table RAMs have registered reads; the
//   word at TBL_ADDR is ready one clock after TBL_ADDR changes (long before the next bus access).
`timescale 1ns / 1ps
`default_nettype none
module inferno_regs #(
    parameter NTXF    = 16,
    parameter NRXF    = 16,
    parameter MAX_CH  = 64,
    parameter MAXFCH  = 32,
    parameter MAX_FPP = 32,
    parameter NR      = 128,
    parameter NF      = 256,
    parameter MAXLAT  = 3              // the largest latency mode the buffers of this build hold (0..3)
) (
    input  wire        clk,
    input  wire        rst,
    input  wire [7:0]  addr,
    input  wire [7:0]  raddr,          // the read address (lbus_target's live bus address)
    input  wire        we,
    input  wire [31:0] wdata,
    input  wire        re,
    output reg  [31:0] rdata,
    // configuration out
    output reg         tx_en,
    output reg         rx_en,
    output reg  [6:0]  n_out,
    output reg  [16:0] media_fs,
    output reg  [31:0] set_msec,
    output reg  [16:0] set_msub,
    output reg  [1:0]  lat_mode,
    output reg         lat_manual,
    output reg  [7:0]  ts_adv,
    input  wire [16:0] fs_now,         // the rate the desk clock runs at (port 0's MEDIA_FS)
    input  wire [31:0] link_offset,    // aoip_regs LINK_OFFSET (frames)
    output reg  [31:0] play_offset,    // the playout offset in use (frames; registered)
    output reg  [5:0]  fpp_auto,       // (registered)
    // tables
    output wire        tx_tbl_we,
    output wire        rx_tbl_we,
    output wire [9:0]  tbl_addr,
    output wire [31:0] tbl_wdata,
    input  wire [31:0] tx_tbl_rdata,
    input  wire [31:0] rx_tbl_rdata,
    // status in
    input  wire [NTXF-1:0] tx_pending,
    input  wire [NTXF-1:0] tx_running,
    input  wire [31:0] media_sec,
    input  wire [16:0] media_sub,
    input  wire [31:0] tx_packets, tx_overruns, rx_ok, rx_bad, rx_late, rx_early, rx_underruns, rx_ts_gaps
);
`include "inferno_lat.vh"
    reg  [9:0] ta;
    wire       tbl_acc_w = we && addr == 8'h0C;
    wire       tbl_acc_r = re && addr == 8'h0C;
    assign tbl_addr  = ta;
    assign tbl_wdata = wdata;
    assign tx_tbl_we = tbl_acc_w && ta[9:8] == 2'd0;
    assign rx_tbl_we = tbl_acc_w && ta[9:8] != 2'd0;

    localparam [3:0] LAT_MASK = (MAXLAT == 0) ? 4'b0001 : (MAXLAT == 1) ? 4'b0011 : (MAXLAT == 2) ? 4'b0111 : 4'b1111;
    wire [3:0]  lat_mask = LAT_MASK;   // latency modes this build accepts
    localparam [1:0] MAXLAT2 = MAXLAT;
    wire        hi = fs_now > 17'd50000;
    wire [9:0]  mode_off = inferno_lat_offset(lat_mode, hi);
    // registered: they follow LATENCY / MEDIA_FS / LINK_OFFSET one clock later (off the per-flow fpp paths)
    always @(posedge clk) begin
        play_offset <= lat_manual ? link_offset : {22'd0, mode_off};
        fpp_auto    <= inferno_lat_fpp(lat_mode, hi);
    end

    always @(posedge clk) begin
        if (rst) begin
            tx_en <= 1'b0; rx_en <= 1'b0; n_out <= MAX_CH; ta <= 10'd0;
            media_fs <= 17'd48000; set_msec <= 32'd0; set_msub <= 17'd0;
            lat_mode <= 2'd0; lat_manual <= 1'b0; ts_adv <= 8'd0;
        end else begin
            if (we) case (addr)
                8'h04: begin tx_en <= wdata[0]; rx_en <= wdata[1]; n_out <= wdata[14:8]; end
                8'h08: ta <= wdata[9:0];
                8'h18: media_fs <= wdata[16:0];
                8'h1C: set_msec <= wdata;
                8'h20: set_msub <= wdata[16:0];
                8'h2C: begin
                    if (lat_mask[wdata[1:0]]) lat_mode <= wdata[1:0];   // a mode above MAXLAT is refused
                    lat_manual <= wdata[8]; ts_adv <= wdata[23:16];
                end
                default: ;
            endcase
            if (tbl_acc_w || tbl_acc_r) ta <= ta + 10'd1;
        end
    end
    wire [15:0] jb_frames = hi ? NF * 2 : NF;
    wire [7:0]  jb_cols   = hi ? MAX_CH / 2 : MAX_CH;
    always @(*) begin
        case (raddr)
            8'h00: rdata = 32'h4F494E32;
            8'h04: rdata = {1'b1, 16'd0, n_out, 6'd0, rx_en, tx_en};
            8'h08: rdata = {22'd0, ta};
            8'h0C: rdata = (ta[9:8] == 2'd0) ? tx_tbl_rdata : rx_tbl_rdata;
            8'h10: rdata = {{(32 - NTXF){1'b0}}, tx_pending};
            8'h14: rdata = {{(32 - NTXF){1'b0}}, tx_running};
            8'h18: rdata = {15'd0, media_fs};
            8'h1C: rdata = set_msec;
            8'h20: rdata = {15'd0, set_msub};
            8'h24: rdata = media_sec;
            8'h28: rdata = {15'd0, media_sub};
            8'h2C: rdata = {8'd0, ts_adv, 7'd0, lat_manual, 6'd0, lat_mode};
            8'h30: rdata = {10'd0, fpp_auto, 5'd0, play_offset[10:0]};
            8'h34: rdata = {8'd0, jb_cols, jb_frames};
            8'h38: rdata = {NRXF[7:0], NTXF[7:0], MAXFCH[7:0], MAX_CH[7:0]};
            8'h3C: rdata = {NR[15:0], 6'd0, MAXLAT2, MAX_FPP[7:0]};
            8'h40: rdata = tx_packets;
            8'h44: rdata = tx_overruns;
            8'h48: rdata = rx_ok;
            8'h4C: rdata = rx_bad;
            8'h50: rdata = rx_late;
            8'h54: rdata = rx_early;
            8'h58: rdata = rx_underruns;
            8'h5C: rdata = rx_ts_gaps;
            default: rdata = 32'd0;
        endcase
    end
    wire unused = &{1'b0, re};
endmodule
`default_nettype wire
