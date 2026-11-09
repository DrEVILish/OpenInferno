// aoip_regs.v - the AMU-visible configuration and status of one AES50-class port: the port mode, the port's
// addresses, the playout offset, the desk clock, the MAC counter and the A&H ACE engine.
//
// Provenance: written for OpenProSeries (W16); our own register map (docs/68 leaves the AoIP control words
// open: "Mode and clock control words: none are sent yet"). Licence: GPL-3.0.
//
// AES67 was dropped on 2026-10-14 (user decision): the AES67 stream registers and counters are retired. Their
// offsets are reserved (read 0, writes ignored) so that every other register keeps its address.
//
// 32-bit registers, byte offsets inside the port's block (the top level decodes the block):
//   0x00 ID            RO 0x4F415032 ("OAP2")
//   0x04 MODE          RW bits 1:0 requested port mode (0 AES50, 1 Inferno, 2 ACE, 3 off); RO bits 5:4 active
//                         mode, bit 8 switching
//   0x08..0x10         reserved (were the AES67 TX_CTRL / TX_DST_MAC)
//   0x14 SRC_MAC_HI    RW bits 15:0 = MAC[47:32]      0x18 SRC_MAC_LO RW MAC[31:0]   (Inferno and ACE)
//   0x1C SRC_IP        RW the port's IPv4 address (Inferno flows)
//   0x20..0x38         reserved (were the AES67 TX / RX stream words)
//   0x3C LINK_OFFSET   RW playout offset in frames behind the media clock, used when inferno_regs LATENCY bit 8
//                         (MANUAL) is set (default 48)
//   0x40 PTP_INCR      RW ns per clock, 8.24             0x44 MEDIA_INC RW (fs / f_clk x 2^32)
//   0x48 SET_SEC_HI    RW   0x4C SET_SEC_LO RW   0x50 SET_NS RW   0x54 CLOCK_CMD WO bit0 step the time of day,
//                         bit1 load the media clock from SET_MEDIA
//   0x58 SET_MEDIA     RW   0x5C MEDIA_TS RO
//   0x60 TOD_SEC_LO    RO (reading it latches TOD_NS)   0x64 TOD_NS RO (latched)
//   0x68 TS_TX_NS RO   0x6C TS_RX_NS RO (the last SFD stamps, ns field)
//   0x80..0x9C, 0xA4   reserved (were the AES67 counters)
//   0xA0 MAC_RX_CRC_ERR RO
// A&H ACE (mode 2, ace_tx.v / ace_rx.v, docs/ace.md; the source MAC is SRC_MAC):
//   0x70 ACE_CTRL      RW bit0 TX enable, bit1 RX enable, bits 14:8 channels (0..64, default 48), bits 19:16 RX
//                         playout offset in frames (1..15, default 2)
//   0x74 ACE_STATUS    RO bit0 RX locked, bits 7:4 last RX frame counter, bits 11:8 next TX frame counter,
//                         bits 31:16 RX phase (clocks from the frame strobe to the end of the last good frame)
//   0xB0 ACE_TX_FRAMES 0xB4 ACE_TX_OVERRUNS 0xB8 ACE_RX_OK 0xBC ACE_RX_BAD 0xC0 ACE_RX_LOST 0xC4 ACE_RX_SYNC_ERR
//   0xC8 ACE_RX_UNDERRUNS 0xCC ACE_RX_OVERRUNS 0xD0 ACE_RX_CTRL_FRAMES (all RO)
`timescale 1ns / 1ps
`default_nettype none
module aoip_regs (
    input  wire        clk,
    input  wire        rst,
    input  wire [7:0]  addr,
    input  wire [7:0]  raddr,          // the read address (lbus_target's live bus address)
    input  wire        we,
    input  wire [31:0] wdata,
    input  wire        re,
    input  wire        rd_hold,        // a read is on the bus: the TOD view stays as the reader sees it
    output reg  [31:0] rdata,
    // configuration out
    output reg  [1:0]  mode_req,
    output reg  [47:0] src_mac,
    output reg  [31:0] src_ip,
    output reg  [31:0] link_offset,
    output reg  [31:0] ptp_incr,
    output reg  [31:0] media_inc,
    output reg  [47:0] set_sec,
    output reg  [29:0] set_ns,
    output reg         set_tod,
    output reg  [31:0] set_media_ts,
    output reg         set_media,
    output reg         ace_tx_en,
    output reg         ace_rx_en,
    output reg  [6:0]  ace_ch,
    output reg  [3:0]  ace_offset,
    // status in
    input  wire [1:0]  mode_active,
    input  wire        switching,
    input  wire [31:0] media_ts,
    input  wire [47:0] tod_sec,
    input  wire [29:0] tod_ns,
    input  wire [29:0] ts_tx_ns,
    input  wire [29:0] ts_rx_ns,
    input  wire [31:0] mac_crc_err,
    input  wire [31:0] ace_status,
    input  wire [31:0] ace_tx_frames, ace_tx_overruns, ace_rx_ok, ace_rx_bad, ace_rx_lost, ace_rx_sync_err,
    input  wire [31:0] ace_rx_underruns, ace_rx_overruns, ace_rx_ctrl
);
    reg [29:0] ns_latch, tod_ns_v;  // TOD: a reading of TOD_SEC latches the nanoseconds of the same clock as the
    reg [31:0] tod_sec_v;           // seconds on the bus (the pair is frozen while the read is on the bus)
    always @(posedge clk) if (!rd_hold) begin tod_sec_v <= tod_sec[31:0]; tod_ns_v <= tod_ns; end
    always @(posedge clk) begin
        set_tod <= 1'b0; set_media <= 1'b0;
        if (rst) begin
            mode_req <= 2'd0; ptp_incr <= 32'h14000000; media_inc <= 32'h003EEA21; link_offset <= 32'd48;
            src_mac <= 48'd0; src_ip <= 32'd0;
            ace_tx_en <= 1'b0; ace_rx_en <= 1'b0; ace_ch <= 7'd48; ace_offset <= 4'd2;
        end else begin
            if (re && addr == 8'h60) ns_latch <= tod_ns_v;
            if (we) case (addr)
                8'h04: mode_req <= wdata[1:0];
                8'h14: src_mac[47:32] <= wdata[15:0];
                8'h18: src_mac[31:0]  <= wdata;
                8'h1C: src_ip <= wdata;
                8'h3C: link_offset <= wdata;
                8'h40: ptp_incr <= wdata;
                8'h44: media_inc <= wdata;
                8'h48: set_sec[47:32] <= wdata[15:0];
                8'h4C: set_sec[31:0] <= wdata;
                8'h50: set_ns <= wdata[29:0];
                8'h54: begin set_tod <= wdata[0]; set_media <= wdata[1]; end
                8'h58: set_media_ts <= wdata;
                8'h70: begin ace_tx_en <= wdata[0]; ace_rx_en <= wdata[1]; ace_ch <= wdata[14:8]; ace_offset <= wdata[19:16]; end
                default: ;
            endcase
        end
    end
    always @(*) begin
        case (raddr)
            8'h00: rdata = 32'h4F415032;
            8'h04: rdata = {23'd0, switching, 2'd0, mode_active, 2'd0, mode_req};
            8'h14: rdata = {16'd0, src_mac[47:32]};
            8'h18: rdata = src_mac[31:0];
            8'h1C: rdata = src_ip;
            8'h3C: rdata = link_offset;
            8'h40: rdata = ptp_incr;
            8'h44: rdata = media_inc;
            8'h48: rdata = {16'd0, set_sec[47:32]};
            8'h4C: rdata = set_sec[31:0];
            8'h50: rdata = {2'd0, set_ns};
            8'h58: rdata = set_media_ts;
            8'h5C: rdata = media_ts;
            8'h60: rdata = tod_sec_v;
            8'h64: rdata = {2'd0, ns_latch};
            8'h68: rdata = {2'd0, ts_tx_ns};
            8'h6C: rdata = {2'd0, ts_rx_ns};
            8'hA0: rdata = mac_crc_err;
            8'h70: rdata = {12'd0, ace_offset, 1'b0, ace_ch, 6'd0, ace_rx_en, ace_tx_en};
            8'h74: rdata = ace_status;
            8'hB0: rdata = ace_tx_frames;
            8'hB4: rdata = ace_tx_overruns;
            8'hB8: rdata = ace_rx_ok;
            8'hBC: rdata = ace_rx_bad;
            8'hC0: rdata = ace_rx_lost;
            8'hC4: rdata = ace_rx_sync_err;
            8'hC8: rdata = ace_rx_underruns;
            8'hCC: rdata = ace_rx_overruns;
            8'hD0: rdata = ace_rx_ctrl;
            default: rdata = 32'd0;
        endcase
    end
endmodule
`default_nettype wire
