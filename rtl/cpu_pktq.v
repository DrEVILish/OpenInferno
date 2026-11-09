// cpu_pktq.v - a port's CPU packet path: the AMU sends and receives whole Ethernet frames through the port's PHY
// (PTP, ARP, IGMP, mDNS, the Inferno / Dante control protocols ARC, CMC, flow control, keepalives), with SFD time
// stamps for PTP. Plus tx_arb, which shares the port's MAC transmitter between the audio packetiser and this queue.
//
// Provenance: written for OpenProSeries (W16): our own design for the "control-frame mailbox next to the media
// path" that fpga/README.md lists as open; time stamps per IEEE 1588 (event messages stamped at the start of frame
// delimiter). The split (audio in the FPGA, control in software) mirrors upstream Inferno (github.com/teodly/
// inferno): media flows (flows_tx.rs / flows_rx.rs) apart from the UDP control servers. No vendor material.
// Licence: GPL-3.0.
//
// Registers (32-bit, byte offsets inside the block; the port decodes the block, aoip_port.v puts it at 0x200):
//   0x00 ID          RO 0x4F435131 ("OCQ1")
//   0x04 CTRL        RW bit0 TX enable, bit1 RX enable, bit2 all multicast, bit3 drop IPv4 239/8 except
//                       239.255.255.255 (other devices' AoIP media; default on), bit4 promiscuous,
//                       bit8 FLUSH (write 1: empty both queues; self-clearing)
//   0x08 TX_FREE     RO free words in the TX buffer
//   0x0C TX_DATA     WO one word of the frame being written, big-endian (bits 31:24 = first byte)
//   0x10 TX_SEND     WO queue the words written since the last TX_SEND as one frame: bits 10:0 length in bytes
//                       (14..1514, without FCS; ceil(length / 4) words must have been written), bit 15 time-stamp
//                       request, bits 23:16 tag. A wrong length or a full queue drops the frame (TX_REJECTS).
//   0x14 TX_STATUS   RO bits 7:0 frames queued (not yet on the wire)
//   0x18 TX_TS_STATUS RO bit31 a stamp is waiting, bits 7:0 its frame's tag
//   0x1C TX_TS_SEC   RO the stamp's seconds (low 32 bits)   0x20 TX_TS_NS RO its nanoseconds (reading clears bit31)
//   0x30 RX_STATUS   RO bits 7:0 frames waiting
//   0x34 RX_LEN      RO the head frame: bit31 valid, bits 10:0 length in bytes (FCS stripped)
//   0x38 RX_TS_SEC   RO   0x3C RX_TS_NS RO   the head frame's SFD time stamp
//   0x40 RX_DATA     RO the next word of the head frame (one word per 32-bit read)
//   0x44 RX_POP      WO drop the head frame (whatever is left of it) and go to the next
//   0x50 TX_FRAMES   0x54 TX_REJECTS   0x58 RX_FRAMES   0x5C RX_OVERFLOWS (no room: frame lost)   0x60 RX_FILTERED
// RX takes a frame that the audio depacketiser did not claim, with a good FCS, for our MAC, broadcast, or
// multicast with "all multicast" (or anything when promiscuous).
`include "clog2.vh"
`timescale 1ns / 1ps
`default_nettype none
module cpu_pktq #(
    parameter TXW = 512,               // TX buffer words (a power of two)
    parameter RXW = 2048,              // RX buffer words (a power of two)
    parameter ND  = 16                 // frames per queue (a power of two)
) (
    input  wire        clk,
    input  wire        rst,
    input  wire [7:0]  reg_addr,
    input  wire [7:0]  reg_raddr,      // the read address (lbus_target's live bus address)
    input  wire        reg_we,
    input  wire [31:0] reg_wdata,
    input  wire        reg_re,
    input  wire        rd_hold,        // a read is on the bus: RX_DATA's valid view stays as the reader sees it
    output reg  [31:0] reg_rdata,
    input  wire [47:0] our_mac,
    input  wire [31:0] tod_sec,
    input  wire [29:0] tod_ns,
    // TX
    output wire [7:0]  m_tdata,
    output wire        m_tvalid,
    output wire        m_tlast,
    input  wire        m_tready,
    input  wire        tx_mine,        // the arbiter has given the MAC to this queue
    input  wire        sfd_tx,
    // RX
    input  wire [7:0]  s_tdata,
    input  wire        s_tvalid,
    input  wire        s_tlast,
    input  wire        s_tuser,
    input  wire        sfd_rx,
    input  wire        claim           // the audio engine took the frame (valid at s_tlast)
);
    localparam TA = `CLOG2(TXW), RA = `CLOG2(RXW), DA = `CLOG2(ND);

    reg        tx_en, rx_en, allmulti, drop239, promisc, flush;
    reg [31:0] tx_frames, tx_rejects, rx_frames, rx_over, rx_filt;

    // ================= TX
    reg  [31:0] txm [0:TXW-1];
    reg  [TA:0] twp, tcm, trd;         // CPU write, committed end, sender read
    reg  [TA:0] td_start [0:ND-1];
    reg  [10:0] td_len   [0:ND-1];
    reg  [8:0]  td_flags [0:ND-1];     // {ts request, tag}
    reg  [DA:0] tdw, tdr;
    wire [TA:0] tx_used = twp - trd;
    wire [TA:0] tx_free = TXW - tx_used;
    wire [DA:0] td_used = tdw - tdr;
    wire        td_full = td_used == ND;
    wire        twr = reg_we && reg_addr == 8'h0C;
    wire        tsend = reg_we && reg_addr == 8'h10;
    wire [10:0] s_len = reg_wdata[10:0];
    wire [TA:0] s_words = (s_len + 11'd3) >> 2;
    always @(posedge clk) if (twr && tx_free != 0) txm[twp[TA-1:0]] <= reg_wdata;

    reg         tbusy, trdy, tsreq;
    reg  [7:0]  ttag;
    reg  [10:0] tlen, tbi;
    reg  [31:0] tq;
    always @(posedge clk) tq <= txm[trd[TA-1:0]];
    reg         ts_valid;
    reg  [7:0]  ts_tag;
    reg  [31:0] ts_sec;
    reg  [29:0] ts_ns;
    reg         stamp_pend;            // this frame wants its SFD stamp

    always @(posedge clk) begin
        if (rst || flush) begin
            twp <= 0; tcm <= 0; trd <= 0; tdw <= 0; tdr <= 0; tbusy <= 1'b0; trdy <= 1'b0; stamp_pend <= 1'b0;
            if (rst) begin ts_valid <= 1'b0; tx_frames <= 32'd0; tx_rejects <= 32'd0; end
        end else begin
            if (twr && tx_free != 0) twp <= twp + 1'b1;
            if (tsend) begin
                if (s_len < 11'd14 || s_len > 11'd1514 || s_words != twp - tcm || td_full) begin
                    tx_rejects <= tx_rejects + 32'd1; twp <= tcm;
                end else begin
                    td_start[tdw[DA-1:0]] <= tcm; td_len[tdw[DA-1:0]] <= s_len;
                    td_flags[tdw[DA-1:0]] <= {reg_wdata[15], reg_wdata[23:16]};
                    tdw <= tdw + 1'b1; tcm <= twp;
                end
            end
            // sender
            trdy <= tbusy;
            if (!tbusy && tdw != tdr && tx_en) begin
                tbusy <= 1'b1; trdy <= 1'b0;
                trd <= td_start[tdr[DA-1:0]]; tlen <= td_len[tdr[DA-1:0]];
                {tsreq, ttag} <= td_flags[tdr[DA-1:0]]; tbi <= 11'd0;
                tdr <= tdr + 1'b1;
            end
            if (tbusy && trdy && m_tready) begin
                tbi <= tbi + 11'd1;
                if (tbi[1:0] == 2'd3 || m_tlast) trd <= trd + 1'b1;
                if (m_tlast) begin tbusy <= 1'b0; trdy <= 1'b0; tx_frames <= tx_frames + 32'd1; end
            end
            // the SFD of a frame we are sending
            if (tbusy && trdy && tx_mine && tbi == 11'd0 && tsreq) stamp_pend <= 1'b1;
            if (sfd_tx && stamp_pend) begin
                stamp_pend <= 1'b0; ts_valid <= 1'b1; ts_tag <= ttag; ts_sec <= tod_sec; ts_ns <= tod_ns;
            end
            if (reg_re && reg_addr == 8'h20) ts_valid <= 1'b0;
        end
    end
    assign m_tvalid = tbusy && trdy;
    // byte select as a mux (XST misreads a part-select whose base is an expression)
    assign m_tdata  = (tbi[1:0] == 2'd0) ? tq[31:24] : (tbi[1:0] == 2'd1) ? tq[23:16] : (tbi[1:0] == 2'd2) ? tq[15:8] : tq[7:0];
    assign m_tlast  = tbusy && trdy && (tbi == tlen - 11'd1);

    // ================= RX
    reg  [31:0] rxm [0:RXW-1];
    reg  [RA:0] rwp, rws, rrp;         // committed end, speculative write, CPU read
    reg  [RA:0] rd_start [0:ND-1];
    reg  [10:0] rd_len   [0:ND-1];
    reg  [31:0] rd_sec   [0:ND-1];
    reg  [29:0] rd_ns    [0:ND-1];
    reg  [DA:0] rdw, rdr;
    wire [DA:0] rd_used = rdw - rdr;
    wire [RA:0] rx_used = rws - rrp;
    wire        rd_full = rd_used == ND;
    wire        rd_have = rdw != rdr;
    wire [RA:0] h_start = rd_start[rdr[DA-1:0]];
    wire [10:0] h_len   = rd_len[rdr[DA-1:0]];
    wire [RA:0] h_end   = h_start + ((h_len + 11'd3) >> 2);
    reg  [10:0] rbi;                   // byte index in the frame
    reg  [23:0] racc;
    reg         rover;                 // no room for this frame
    reg         f_uc, f_bc, f_mc, f_239, f_bcast239, f_ip;
    reg  [31:0] r_sec;
    reg  [29:0] r_ns;
    reg         rwe;
    reg  [31:0] rwd;
    reg  [RA-1:0] rwa;
    always @(posedge clk) if (rwe) rxm[rwa] <= rwd;
    reg  [31:0] rq;
    reg  [RA:0] rrp_d;
    always @(posedge clk) begin rq <= rxm[rrp[RA-1:0]]; rrp_d <= rrp; end
    wire        rq_ok = rrp_d == rrp && rd_have && rrp != h_end;
    reg         rq_ok_r;                // RX_DATA: the word and the pop use this view (frozen during a read)
    always @(posedge clk) if (!rd_hold) rq_ok_r <= rq_ok;
    wire [7:0]  b = s_tdata;
    wire [23:0] racc_n = {racc[15:0], b};
    wire [47:0] mac_sh = our_mac << (8 * rbi[2:0]);
    wire [7:0]  macb = mac_sh[47:40];  // our MAC's byte rbi

    always @(posedge clk) begin
        rwe <= 1'b0;
        if (rst || flush) begin
            rwp <= 0; rws <= 0; rrp <= 0; rdw <= 0; rdr <= 0; rbi <= 11'd0; rover <= 1'b0;
            if (rst) begin rx_frames <= 32'd0; rx_over <= 32'd0; rx_filt <= 32'd0; end
        end else begin
            if (sfd_rx) begin r_sec <= tod_sec; r_ns <= tod_ns; end
            if (s_tvalid) begin
                rbi <= s_tlast ? 11'd0 : rbi + 11'd1;
                if (rbi == 11'd0) begin
                    f_uc <= b == our_mac[47:40]; f_bc <= b == 8'hFF; f_mc <= b[0]; f_ip <= 1'b1; f_239 <= 1'b0;
                    f_bcast239 <= 1'b1; rover <= 1'b0;
                end else if (rbi < 11'd6) begin
                    if (b != macb) f_uc <= 1'b0;
                    if (b != 8'hFF) f_bc <= 1'b0;
                end
                if (rbi == 11'd12 && b != 8'h08) f_ip <= 1'b0;
                if (rbi == 11'd13 && b != 8'h00) f_ip <= 1'b0;
                if (rbi == 11'd30) f_239 <= b == 8'd239;
                if (rbi > 11'd30 && rbi < 11'd34 && b != 8'hFF) f_bcast239 <= 1'b0;
                // words into the buffer (speculative)
                racc <= racc_n;
                if (rbi[1:0] == 2'd3 || s_tlast) begin
                    if (rx_used >= RXW || rover) rover <= 1'b1;
                    else begin
                        rwe <= 1'b1; rwa <= rws[RA-1:0];
                        case (rbi[1:0])
                            2'd0: rwd <= {b, 24'd0};
                            2'd1: rwd <= {racc[7:0], b, 16'd0};
                            2'd2: rwd <= {racc[15:0], b, 8'd0};
                            default: rwd <= {racc, b};
                        endcase
                        rws <= rws + 1'b1;
                    end
                end
                if (s_tlast) begin
                    if (!rx_en || claim || s_tuser) rws <= rwp;
                    else if (!(promisc || f_uc || f_bc || (f_mc && allmulti)) || (drop239 && f_ip && f_239 && !f_bcast239)) begin
                        rws <= rwp; rx_filt <= rx_filt + 32'd1;
                    end else if (rover || (rx_used >= RXW) || rd_full) begin
                        rws <= rwp; rx_over <= rx_over + 32'd1;
                    end else begin
                        rd_start[rdw[DA-1:0]] <= rwp; rd_len[rdw[DA-1:0]] <= rbi + 11'd1;
                        rd_sec[rdw[DA-1:0]] <= r_sec; rd_ns[rdw[DA-1:0]] <= r_ns;
                        rdw <= rdw + 1'b1; rwp <= rws + 1'b1; rx_frames <= rx_frames + 32'd1;
                    end
                end
            end
            // the CPU side
            if (reg_re && reg_addr == 8'h40 && rq_ok_r) rrp <= rrp + 1'b1;
            if (reg_we && reg_addr == 8'h44 && rd_have) begin rrp <= h_end; rdr <= rdr + 1'b1; end
        end
    end

    // ================= registers
    always @(posedge clk) begin
        flush <= 1'b0;
        if (rst) begin
            tx_en <= 1'b0; rx_en <= 1'b0; allmulti <= 1'b0; drop239 <= 1'b1; promisc <= 1'b0;
        end else if (reg_we && reg_addr == 8'h04) begin
            tx_en <= reg_wdata[0]; rx_en <= reg_wdata[1]; allmulti <= reg_wdata[2]; drop239 <= reg_wdata[3];
            promisc <= reg_wdata[4]; flush <= reg_wdata[8];
        end
    end
    // the head frame's stamp, registered (bus reads only; off the bus read path)
    reg  [31:0] rd_sec_q;
    reg  [29:0] rd_ns_q;
    always @(posedge clk) begin rd_sec_q <= rd_sec[rdr[DA-1:0]]; rd_ns_q <= rd_ns[rdr[DA-1:0]]; end
    always @(*) begin
        case (reg_raddr)
            8'h00: reg_rdata = 32'h4F435131;
            8'h04: reg_rdata = {27'd0, promisc, drop239, allmulti, rx_en, tx_en};
            8'h08: reg_rdata = {{(31 - TA){1'b0}}, tx_free};
            8'h14: reg_rdata = {{(31 - DA){1'b0}}, tdw - tdr + (tbusy ? 1'b1 : 1'b0)};
            8'h18: reg_rdata = {ts_valid, 23'd0, ts_tag};
            8'h1C: reg_rdata = ts_sec;
            8'h20: reg_rdata = {2'd0, ts_ns};
            8'h30: reg_rdata = {{(31 - DA){1'b0}}, rdw - rdr};
            8'h34: reg_rdata = {rd_have, 20'd0, rd_have ? h_len : 11'd0};
            8'h38: reg_rdata = rd_sec_q;
            8'h3C: reg_rdata = {2'd0, rd_ns_q};
            8'h40: reg_rdata = rq_ok_r ? rq : 32'd0;
            8'h50: reg_rdata = tx_frames;
            8'h54: reg_rdata = tx_rejects;
            8'h58: reg_rdata = rx_frames;
            8'h5C: reg_rdata = rx_over;
            8'h60: reg_rdata = rx_filt;
            default: reg_rdata = 32'd0;
        endcase
    end
endmodule

// tx_arb - the port's MAC transmitter shared by two frame sources; a frame is never interrupted. Source A (audio)
// wins when both wait. mine_* says who owns the MAC (for the CPU queue's TX time stamp).
module tx_arb (
    input  wire       clk,
    input  wire       rst,
    input  wire [7:0] a_tdata,
    input  wire       a_tvalid,
    input  wire       a_tlast,
    output wire       a_tready,
    input  wire [7:0] b_tdata,
    input  wire       b_tvalid,
    input  wire       b_tlast,
    output wire       b_tready,
    output wire [7:0] m_tdata,
    output wire       m_tvalid,
    output wire       m_tlast,
    input  wire       m_tready,
    output wire       mine_b
);
    reg [1:0] own;                     // 0 none, 1 A, 2 B
    always @(posedge clk) begin
        if (rst) own <= 2'd0;
        else if (own == 2'd0) own <= a_tvalid ? 2'd1 : b_tvalid ? 2'd2 : 2'd0;
        else if (m_tready && m_tlast) own <= 2'd0;
    end
    assign m_tdata  = (own == 2'd2) ? b_tdata : a_tdata;
    assign m_tvalid = (own == 2'd1) ? a_tvalid : (own == 2'd2) ? b_tvalid : 1'b0;
    assign m_tlast  = (own == 2'd1) ? a_tlast : (own == 2'd2) ? b_tlast : 1'b0;
    assign a_tready = (own == 2'd1) && m_tready;
    assign b_tready = (own == 2'd2) && m_tready;
    assign mine_b   = own == 2'd2;
endmodule
`default_nettype wire
