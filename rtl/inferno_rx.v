// inferno_rx.v - Inferno (Dante-compatible) audio flow depacketiser: up to NFLOW received flows per port, each up to
// MAXFCH channels mapped onto the port's MAX_CH columns, into one timestamp-indexed jitter buffer.
//
// Provenance: written for OpenProSeries (W16). The media packet and the receiver's timestamp follow upstream Inferno
// (github.com/teodly/inferno, GPL-3.0, commit 9767558e, inferno_aoip/src/device_server/flows_rx.rs `receive()`:
// timestamp = seconds x sample_rate + subsec, wrapping; samples interleaved with stride channels x bytes per sample);
// the lower layers are the public standards (IEEE 802.3, RFC 791, RFC 768). No vendor material. Licence: GPL-3.0.
//
// A frame is taken when it is IPv4 (IHL 5, not fragmented) / UDP and {dst IP, dst UDP port} equals an enabled
// flow's key (unicast: our IP and the flow's local port; multicast: the group and 4321). Its UDP payload is the
// Inferno media packet (inferno_tx.v): byte 0 (not checked, as upstream), u32 seconds, u32 sub-second samples, then
// whole frames of nch samples of bps bytes (2: 16 bit, 3: 24 bit, 4: 32 bit; kept as 24 bits). A payload that is not
// a whole number of frames is bad (upstream floors it). The media time T = seconds x fs + sub (mod 2^32, a serial
// shift-and-add multiply during the header) plus the flow's SHIFT (frames; the flow's latency above the port's
// playout offset) gives the frames' jitter-buffer slots.
//
// Jitter buffer: NF x MAX_CH samples (block RAM). At 48 kHz (hi = 0) it is NF frames x MAX_CH columns; at 96 kHz
// (hi = 1) the same memory is 2 NF frames x MAX_CH / 2 columns (columns >= MAX_CH / 2 are not played), so the
// buffer covers the same time at both rates. Per frame time a tag {mask, T[15:0]} (block RAM, 2 NF entries,
// index T mod 2 NF): mask bit f = flow f wrote frame T. A column remembers the flow that last wrote it (owner).
// One clock after frame_strobe the frame of media time play_ts (the media clock minus the playout offset) is played
// as n_out (out_ch, out_sample) words, one a clock; a column plays its sample only if the frame's tag is play_ts and
// the column owner's mask bit is set, else silence. After the read the tag is cleared (swept), so a frame is never
// played twice and a flow that stops leaves silence, not old audio. Several flows (from several transmitters) can
// fill one port frame. Packets wholly before play_ts count as late, packets reaching past the buffer (NF frames
// ahead at 48 kHz, 2 NF at 96 kHz) as early; both are dropped.
//
// Flow configuration (NFLOW x 16 words, through inferno_regs.v; address = flow * 16 + w, block RAM, reads registered):
//   w0 CTRL  bit0 enable, bits 13:8 channels in the flow (1..MAXFCH), bits 18:16 bytes per sample (2, 3, 4)
//   w1 DST_IP (match)   w2 DST_PORT (bits 15:0, match)   w3 SHIFT (signed frames added to the packet time)
//   w4..w11 MAP: one byte per flow channel c (word 4 + c / 4, byte c mod 4 at bits 8 x (c mod 4) + 7 .. ),
//            bits 5:0 port column, bit 7 = ignore      w12..w15 spare
// Flow status (read only, address bit FW+4 set: 16 x flow + w): w0 source IP, w1 source UDP port, w2 packets taken,
//   w3 largest latency seen (media_ts - T at arrival, frames; writing w3 clears it)
// The status gives the control plane what it needs for keepalives (13 37 to the source address every 250 ms,
// upstream KEEPALIVE_INTERVAL) and for the flow timeout.
//
// Input pace (clocks; tb_inferno_pace). rmii_rx (4 clocks an octet) meets all of it for packets up to 256 frames:
//   octet 50 >= 15 + f after octet 37 (f = the matched flow's number: key search, then the 13-word loader), else the
//   packet is dropped uncounted; the last octet >= 17 after octet 46 (the multiply), else bad. Evenly spaced: 3 a
//   octet for every flow (2 only up to flow 11). A packet's commit ends max(samples, 2 x frames) + 3 after its last
//   octet (a few more per frame strobe); a good packet that ends sooner waits for it, but is counted bad if the frame
//   after it reaches octet 28 first.
`include "clog2.vh"
`timescale 1ns / 1ps
`default_nettype none
module inferno_rx #(
    parameter MAX_CH  = 64,            // port columns (a power of two, <= 64)
    parameter NFLOW   = 16,
    parameter MAXFCH  = 32,            // channels per flow (<= 32)
    parameter MAX_SMP = 512,           // samples staged per packet (1463-byte payload / 3 = 487)
    parameter NF      = 256            // jitter buffer frames at 48 kHz (a power of two; 2 NF at 96 kHz)
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        enable,
    input  wire [6:0]  n_out,          // columns played per frame (1..MAX_CH; at most MAX_CH / 2 at 96 kHz)
    input  wire [16:0] media_fs,
    input  wire [31:0] media_ts,
    // tables: address bit FW+4 selects the status words
    input  wire        tbl_we,
    input  wire [`CLOG2(NFLOW)+4:0] tbl_waddr,
    input  wire [31:0] tbl_wdata,
    input  wire [`CLOG2(NFLOW)+4:0] tbl_raddr,
    output wire [31:0] tbl_rdata,
    // frames in (rmii_rx)
    input  wire [7:0]  s_tdata,
    input  wire        s_tvalid,
    input  wire        s_tlast,
    input  wire        s_tuser,
    output wire        claim,          // the frame in progress belongs to a flow (valid from byte ~42 to its end)
    // playout
    input  wire        frame_strobe,
    input  wire [31:0] play_ts,        // media_ts - playout offset (registered here)
    output reg         out_valid,
    output reg  [5:0]  out_ch,
    output reg  [23:0] out_sample,
    output reg         out_frame_ok,   // with the frame's first word: some flow had data for this frame
    // status
    output reg  [31:0] packets_ok,
    output reg  [31:0] packets_bad,    // FCS errors or not a whole number of frames, for a matching flow
    output reg  [31:0] late,
    output reg  [31:0] early,
    output reg  [31:0] underruns,      // frames played with no flow's data while enabled
    output reg  [31:0] ts_gaps         // a flow's packet did not start where its previous one ended
);
    localparam FW  = `CLOG2(NFLOW);
    localparam NFW = `CLOG2(NF);       // slot bits at 48 kHz (NFW + 1 at 96 kHz)
    localparam CW  = `CLOG2(MAX_CH);
    localparam SW  = `CLOG2(MAX_SMP);
    localparam JAW = NFW + CW;
    localparam TGW = NFW + 1;          // tag index bits (2 NF entries)
    localparam HDR = 51;
    wire hi = media_fs > 17'd50000;

    // ---- configuration (block RAM: host write port; reads: host, header loader) and the match keys (search port)
    // Memory coding (XST / Spartan-3 inference): every array has one write port; the host shares port A of the
    // configuration RAM (write address when writing, read address otherwise: the two are the same bus address in
    // aoip_port), the header loader has port B.
    reg  [31:0] cfg [0:NFLOW*16-1];
    integer ii;
    initial for (ii = 0; ii < NFLOW * 16; ii = ii + 1) cfg[ii] = 32'd0;
    wire cfg_we = tbl_we && !tbl_waddr[FW+4];
    wire [FW+3:0] cfg_a = tbl_we ? tbl_waddr[FW+3:0] : tbl_raddr[FW+3:0];
    reg  [31:0] cfg_host_q;
    always @(posedge clk) begin
        if (cfg_we) cfg[cfg_a] <= tbl_wdata;
        cfg_host_q <= cfg[cfg_a];
    end
    // the match keys {enable, port, ip} follow w0 / w1 / w2 writes: an enable vector and two LUT RAMs, one write port
    // each (the search reads them asynchronously, one flow a clock)
    reg  [NFLOW-1:0] key_en = {NFLOW{1'b0}};
    reg  [31:0] key_ip   [0:NFLOW-1];
    reg  [15:0] key_port [0:NFLOW-1];
    wire [FW-1:0] key_wf = tbl_waddr[FW+3:4];
    always @(posedge clk) if (cfg_we && tbl_waddr[3:0] == 4'd0) key_en[key_wf] <= tbl_wdata[0];
    always @(posedge clk) if (cfg_we && tbl_waddr[3:0] == 4'd1) key_ip[key_wf] <= tbl_wdata;
    always @(posedge clk) if (cfg_we && tbl_waddr[3:0] == 4'd2) key_port[key_wf] <= tbl_wdata[15:0];

    // ---- status (written once per packet; host read port)
    reg  [31:0] st_ip   [0:NFLOW-1];
    reg  [15:0] st_port [0:NFLOW-1];
    reg  [31:0] st_pkts [0:NFLOW-1];
    reg  [31:0] st_lat  [0:NFLOW-1];   // latency peak; one write port (the judge): the host's clear is lat_clr
    reg  [NFLOW-1:0] lat_clr = {NFLOW{1'b1}};  // peak cleared (reads 0) until the judge writes it again
    reg  [31:0] nexp_ts  [0:NFLOW-1];   // where the flow's next packet should start
    reg  [NFLOW-1:0] have;
    initial for (ii = 0; ii < NFLOW; ii = ii + 1) begin
        st_ip[ii] = 32'd0; st_port[ii] = 16'd0; st_pkts[ii] = 32'd0; st_lat[ii] = 32'd0; nexp_ts[ii] = 32'd0;
    end
    wire [FW-1:0] rfl = tbl_raddr[FW+3:4];
    reg  [31:0] st_q;
    // array reads as continuous assigns (XST: no memory reads inside @(*))
    wire [31:0] st_ip_q = st_ip[rfl], st_pkts_q = st_pkts[rfl];
    wire [31:0] st_lat_r = st_lat[rfl];
    wire [31:0] st_lat_q = lat_clr[rfl] ? 32'd0 : st_lat_r;
    wire [15:0] st_port_q = st_port[rfl];
    always @(*) case (tbl_raddr[1:0])
        2'd0: st_q = st_ip_q;
        2'd1: st_q = {16'd0, st_port_q};
        2'd2: st_q = st_pkts_q;
        default: st_q = st_lat_q;
    endcase
    // the status read is registered like the configuration RAM's (the word follows the address one clock later)
    reg  [31:0] st_q_r;
    reg         st_sel, st_zero;
    always @(posedge clk) begin st_q_r <= st_q; st_sel <= tbl_raddr[FW+4]; st_zero <= tbl_raddr[3:2] != 2'd0; end
    assign tbl_rdata = st_sel ? (st_zero ? 32'd0 : st_q_r) : cfg_host_q;

    // ---- parser
    reg  [10:0] idx;
    reg  [15:0] w16;
    reg  [31:0] w32;
    reg         drop;                  // not IPv4 / UDP / not for a flow
    reg  [31:0] p_sip, p_dip, p_sec;
    reg  [15:0] p_sport, p_dport, p_ulen, p_iplen;
    reg  [16:0] p_sub;
    // flow search: one key a clock from byte 37 on
    reg         srch, matched;
    reg  [FW:0] sidx;
    reg  [FW-1:0] fidx;
    wire [31:0] key_ip_q   = key_ip[sidx[FW-1:0]];
    wire [15:0] key_port_q = key_port[sidx[FW-1:0]];
    wire        key_en_q   = key_en[sidx[FW-1:0]];
    // the matched flow's configuration, loaded during the header: ld = the word addressed (data one clock later)
    reg  [4:0]  ld;                    // 0..12, 16 = done
    reg  [31:0] cfg_q;
    always @(posedge clk) cfg_q <= cfg[{fidx, ld[3:0]}];
    wire        ld_done = ld[4];
    reg  [31:0] f_shift;
    reg  [5:0]  f_nch;
    reg  [2:0]  f_bps;
    // serial multiply sec x fs
    reg         mul_on;
    reg  [4:0]  mul_i;
    reg  [31:0] mul_acc;
    // payload staging
    reg  [1:0]  sb;
    reg  [23:0] acc;                   // the sample's first bytes
    reg  [SW:0] nsmp;
    reg  [5:0]  sc;                    // channel of the sample being assembled
    reg  [9:0]  nfr;                   // whole frames staged
    reg         sbank;
    // Commit reads one sample per clock. A new packet starts staging only after its
    // header, then writes at most one sample per two clocks, so it cannot overtake
    // the previous commit. Maps remain double-buffered because they are reused.
    reg  [23:0] stage [0:MAX_SMP-1];
    reg         fin;                   // the frame ended last clock: judge it
    reg         fin_bad;
    wire [16:0] pay_end = 17'd34 + {1'b0, p_ulen};  // byte index after the UDP payload (excludes Ethernet padding)
    assign claim = matched && !drop;

    // ---- commit engines (staging -> jitter buffer: samples, one a clock; tags, one frame per two clocks)
    reg         cbusy, cbank, tbusy, tph;
    reg  [31:0] cts;                   // media time of the packet's first frame (with the shift)
    reg  [SW:0] ck, cn;
    reg  [5:0]  cc;
    reg  [9:0]  cf, tf, tn;
    reg  [FW-1:0] cfl;
    reg  [5:0]  cnch;
    reg         chi;
    reg         cwe;
    reg  [JAW-1:0] cwa;
    reg  [23:0] stage_q;
    reg  [23:0] jbuf [0:NF*MAX_CH-1];
    reg  [NFLOW+15:0] tags [0:2*NF-1];  // {mask, T[15:0]}
    reg  [FW:0] owner [0:MAX_CH-1];     // {valid, flow}
    initial begin
        for (ii = 0; ii < 2 * NF; ii = ii + 1) tags[ii] = {(NFLOW+16){1'b0}};
        for (ii = 0; ii < MAX_CH; ii = ii + 1) owner[ii] = {(FW+1){1'b0}};
    end
    always @(posedge clk) stage_q <= stage[ck[SW-1:0]];
    always @(posedge clk) if (cwe) jbuf[cwa] <= stage_q;

    // packet media time, its slot time and its latency: registered (stable long before the frame ends)
    reg  [31:0] t_frame, w_frame, lat_now, play_r;
    always @(posedge clk) begin
        t_frame <= mul_acc + {15'd0, p_sub};
        w_frame <= t_frame + f_shift;
        lat_now <= media_ts - t_frame;
        play_r  <= play_ts;
    end
    // the judge takes two clocks: at fin the checks are registered (j_*), the clock after acts on them (j_go). ahead_r
    // is the packet's distance past the playout point, one clock old (the point moves once a frame)
    wire [10:0] nf_eff = hi ? (NF * 2) : NF;
    reg  [31:0] ahead_r;
    always @(posedge clk) ahead_r <= w_frame - play_r;
    wire [31:0] ahead_end  = ahead_r + {22'd0, nfr};               // past the playout point at the packet's end
    wire [31:0] behind_end = 32'd0 - ahead_end;                    // play_r - (w_frame + nfr)
    reg         j_go, j_bad, j_early, j_late;
    wire        j_new  = fin && matched && !drop;
    wire        j_wait = j_go && (cbusy || tbusy) && !j_bad && !j_early && !j_late;
    // the flow's channel map (w4..w11, 32 bytes) in a LUT RAM of two banks x 8 words: the loader writes the packet's
    // staging bank (sbank), the commit reads the bank it copies (cbank), so the next header can load while a commit
    // runs (it was a 256-bit register copied at the commit)
    reg  [31:0] mapm [0:15];
    wire        map_we = !rst && !ld_done && ld[3:0] >= 4'd5 && ld[3:0] <= 4'd12;
    wire [2:0]  map_wa = ld[2:0] - 3'd5;
    always @(posedge clk) if (map_we) mapm[{sbank, map_wa}] <= cfg_q;
    wire [31:0] mapw = mapm[{cbank, cc[4:2]}];
    wire [7:0]  cmapb = (cc[1:0] == 2'd0) ? mapw[7:0] : (cc[1:0] == 2'd1) ? mapw[15:8] : (cc[1:0] == 2'd2) ? mapw[23:16] : mapw[31:24];
    wire        c_col_ok = !cmapb[7] && cmapb[5:0] < (chi ? MAX_CH / 2 : MAX_CH) && cc < MAXFCH;
    wire [NFW:0] cslot = cts[NFW:0] + cf[NFW:0];                  // frame slot (NFW + 1 bits used at 96 kHz)
    wire [NFLOW-1:0] fbit = {{(NFLOW-1){1'b0}}, 1'b1} << cfl;
    // The tag RAM is one block RAM with two ports. Port A (read / write): the playout sweep (write) first, else the
    // tag engine (read at tph 0, merge and write at tph 1). Port B (read): the playout. The tag engine's read result
    // is void when the sweep had port A in that clock, and its write waits while the sweep or the playout's tag read
    // (the clock after the strobe, strobe_d) uses the RAM, so no clock writes the word the other port reads.
    reg         sweep, sweep_d, strobe_d;
    reg  [TGW-1:0] sweep_ix;
    wire [31:0] tt = cts + {22'd0, tf};                          // the tag engine's frame time
    reg  [NFLOW+15:0] ttag_q;
    wire        t_hold = sweep || sweep_d || strobe_d;           // tph 1: read again
    wire        t_wr = tbusy && tph && !t_hold;
    wire [NFLOW+15:0] t_new = (ttag_q[15:0] == tt[15:0]) ? {ttag_q[NFLOW+15:16] | fbit, tt[15:0]} : {fbit, tt[15:0]};
    wire [TGW-1:0] tag_a  = sweep ? sweep_ix : tt[TGW-1:0];
    wire        tag_we = sweep || t_wr;
    wire [NFLOW+15:0] tag_d = sweep ? {(NFLOW+16){1'b0}} : t_new;
    always @(posedge clk) begin
        if (tag_we) tags[tag_a] <= tag_d;
        ttag_q <= tags[tag_a];
    end
    always @(posedge clk) sweep_d <= sweep;

    always @(posedge clk) begin
        cwe <= 1'b0; fin <= 1'b0;
        if (rst) begin
            idx <= 11'd0; drop <= 1'b1; matched <= 1'b0; srch <= 1'b0; mul_on <= 1'b0; ld <= 5'd16; j_go <= 1'b0;
            sbank <= 1'b0; cbusy <= 1'b0; tbusy <= 1'b0; have <= {NFLOW{1'b0}};
            packets_ok <= 32'd0; packets_bad <= 32'd0; late <= 32'd0; early <= 32'd0; ts_gaps <= 32'd0;
        end else begin
            // ---- header and payload
            if (s_tvalid) begin
                w16 <= {w16[7:0], s_tdata};
                w32 <= {w32[23:0], s_tdata};
                idx <= s_tlast ? 11'd0 : idx + 11'd1;
                if (idx == 11'd0) begin
                    drop <= !enable; matched <= 1'b0; srch <= 1'b0; ld <= 5'd16;
                    fin_bad <= 1'b0;
                end
                case (idx)
                    11'd13: if ({w16[7:0], s_tdata} != 16'h0800) drop <= 1'b1;
                    11'd14: if (s_tdata != 8'h45) drop <= 1'b1;
                    11'd17: p_iplen <= {w16[7:0], s_tdata};
                    11'd20: if (s_tdata[5:0] != 6'd0) drop <= 1'b1;     // MF or a fragment offset
                    11'd21: if (s_tdata != 8'd0) drop <= 1'b1;
                    11'd23: if (s_tdata != 8'd17) drop <= 1'b1;
                    // a waiting verdict (j_wait) still needs the counts: cleared here, where it gives up
                    11'd28: begin nsmp <= {(SW+1){1'b0}}; sb <= 2'd0; sc <= 6'd0; nfr <= 10'd0; end
                    11'd29: p_sip <= {w32[23:0], s_tdata};
                    11'd33: p_dip <= {w32[23:0], s_tdata};
                    11'd35: p_sport <= {w16[7:0], s_tdata};
                    11'd37: begin p_dport <= {w16[7:0], s_tdata}; srch <= !drop; sidx <= {(FW+1){1'b0}}; end
                    11'd39: p_ulen <= {w16[7:0], s_tdata};
                    11'd46: begin p_sec <= {w32[23:0], s_tdata}; mul_on <= 1'b1; mul_i <= 5'd0; mul_acc <= 32'd0; end
                    11'd50: p_sub <= {w16[8], w16[7:0], s_tdata};
                    default: ;
                endcase
                // payload: samples of bps bytes; the first three are kept (16 bit: low byte 0, 32 bit: byte 3 dropped)
                if (idx >= HDR && idx < pay_end && !drop) begin
                    if (sb < 2'd3) acc <= {acc[15:0], s_tdata};
                    if ({1'b0, sb} + 3'd1 == f_bps) begin
                        sb <= 2'd0;
                        if (nsmp < MAX_SMP)
                            stage[nsmp[SW-1:0]] <= (f_bps == 3'd2) ? {acc[7:0], s_tdata, 8'h00}
                                                          : (f_bps == 3'd3) ? {acc[15:0], s_tdata} : acc;
                        nsmp <= nsmp + 1'b1;
                        if (sc + 6'd1 >= f_nch) begin sc <= 6'd0; nfr <= nfr + 10'd1; end else sc <= sc + 6'd1;
                    end else sb <= sb + 2'd1;
                end
                if (s_tlast) begin
                    fin <= 1'b1;
                    fin_bad <= s_tuser || p_ulen < 16'd17 || p_iplen > 16'd1500
                               || {1'b0, p_iplen} != 17'd20 + {1'b0, p_ulen}
                               || {6'd0, idx} + 17'd1 < pay_end;
                end
            end
            // ---- flow search (keys are flip-flop / LUT RAM: one a clock)
            if (srch) begin
                if (key_en_q && key_port_q == p_dport && key_ip_q == p_dip) begin
                    srch <= 1'b0; matched <= 1'b1; fidx <= sidx[FW-1:0]; ld <= 5'd0;
                end else if (sidx == NFLOW - 1) begin
                    srch <= 1'b0; drop <= 1'b1;
                end
                sidx <= sidx + 1'b1;
            end
            // ---- configuration loader: word ld - 1 is on cfg_q
            if (!ld_done) begin
                case (ld[3:0])
                    4'd1: begin f_nch <= cfg_q[13:8]; f_bps <= cfg_q[18:16]; end
                    4'd4: f_shift <= cfg_q;
                    default: ;
                endcase
                ld <= (ld == 5'd12) ? 5'd16 : ld + 5'd1;
            end
            if (mul_on) begin
                if (media_fs[mul_i]) mul_acc <= mul_acc + p_sec;
                p_sec <= {p_sec[30:0], 1'b0};
                mul_i <= mul_i + 5'd1;
                if (mul_i == 5'd16) mul_on <= 1'b0;
            end
            // a header that ended before the search, the loader or the multiply finished is not a flow packet
            if (s_tvalid && idx == HDR - 1 && (srch || !ld_done)) drop <= 1'b1;
            // ---- judge the frame: the checks (fin), then the verdict (j_go, the clock after)
            // a good packet judged while the previous commit runs waits for it (a short packet after a long one); its
            // state lasts until the next frame's byte 28 (p_sip, the counts), so it is dropped as bad only there
            j_go <= j_new || (j_wait && !(s_tvalid && idx == 11'd28));
            if (j_wait && s_tvalid && idx == 11'd28) packets_bad <= packets_bad + 32'd1;
            if (j_new) begin
                j_bad   <= fin_bad || f_bps < 3'd2 || f_bps > 3'd4 || f_nch == 6'd0 || f_nch > MAXFCH || sb != 2'd0
                           || sc != 6'd0 || nfr == 10'd0 || nsmp > MAX_SMP || mul_on;
                j_early <= !ahead_r[31] && ahead_end > {21'd0, nf_eff};
                j_late  <= !behind_end[31];
            end
            if (j_go && !j_wait) begin
                if (j_bad)
                    packets_bad <= packets_bad + 32'd1;
                else if (j_early)
                    early <= early + 32'd1;
                else if (j_late)
                    late <= late + 32'd1;
                else begin
                    packets_ok <= packets_ok + 32'd1;
                    if (have[fidx] && nexp_ts[fidx] != t_frame) ts_gaps <= ts_gaps + 32'd1;
                    have[fidx] <= 1'b1;
                    nexp_ts[fidx] <= t_frame + {22'd0, nfr};
                    st_ip[fidx] <= p_sip; st_port[fidx] <= p_sport; st_pkts[fidx] <= st_pkts[fidx] + 32'd1;
                    if (!lat_now[31] && lat_now > (lat_clr[fidx] ? 32'd0 : st_lat[fidx])) begin
                        st_lat[fidx] <= lat_now; lat_clr[fidx] <= 1'b0;
                    end
                    cbusy <= 1'b1; cbank <= sbank; cts <= w_frame; ck <= {(SW+1){1'b0}}; cn <= nsmp;
                    cc <= 6'd0; cf <= 10'd0; cfl <= fidx; cnch <= f_nch; chi <= hi;
                    tbusy <= 1'b1; tph <= 1'b0; tf <= 10'd0; tn <= nfr;
                    sbank <= ~sbank;
                end
            end
            // host clears the latency peak (w3 of the status)
            if (tbl_we && tbl_waddr[FW+4] && tbl_waddr[3:0] == 4'd3) lat_clr[tbl_waddr[FW+3:4]] <= 1'b1;
            if (tbl_we && !tbl_waddr[FW+4] && tbl_waddr[3:0] == 4'd0 && !tbl_wdata[0]) have[tbl_waddr[FW+3:4]] <= 1'b0;
            // ---- commit samples: one a clock (stage_q is the word of ck from the clock before)
            if (cbusy) begin
                cwe <= c_col_ok;
                cwa <= chi ? {cslot[NFW:0], cmapb[CW-2:0]} : {cslot[NFW-1:0], cmapb[CW-1:0]};
                if (c_col_ok) owner[cmapb[CW-1:0]] <= {1'b1, cfl};
                ck <= ck + 1'b1;
                if (cc + 6'd1 >= cnch) begin cc <= 6'd0; cf <= cf + 10'd1; end else cc <= cc + 6'd1;
                if (ck + 1'b1 >= cn) cbusy <= 1'b0;
            end
            // ---- commit tags: read (tph 0), merge and write (tph 1; held while the sweep has the write port)
            if (tbusy) begin
                if (!tph) tph <= 1'b1;
                else if (t_hold) tph <= 1'b0;                  // read again (the sweep may have cleared this tag)
                else begin
                    tph <= 1'b0;
                    tf <= tf + 10'd1;
                    if (tf + 10'd1 >= tn) tbusy <= 1'b0;
                end
            end
        end
    end

    // ---- playout (one clock after the frame strobe, from the registered play_ts)
    reg         pbusy, prd, tchk, phi;
    reg  [6:0]  pc, n_eff;
    reg  [5:0]  prd_ch;
    reg  [NFW:0] pslot;
    reg  [31:0] pts;
    reg  [NFLOW+15:0] ptag, ptag_q;
    reg  [23:0] jq;
    reg  [FW:0] own_q;
    // port B of the tag RAM and the jitter buffer's read port are enabled only in the clocks whose data are used (the
    // tag in the clock after the strobe, the samples while the burst runs): no idle reads collide with writes
    always @(posedge clk) if (strobe_d) ptag_q <= tags[play_r[TGW-1:0]];
    wire [JAW-1:0] pa = phi ? {pslot[NFW:0], pc[CW-2:0]} : {pslot[NFW-1:0], pc[CW-1:0]};
    always @(posedge clk) if (pbusy) jq <= jbuf[pa];
    always @(posedge clk) own_q <= owner[pc[CW-1:0]];
    wire        tag_hit = ptag[15:0] == pts[15:0];
    wire        col_ok  = tag_hit && own_q[FW] && ptag[16 + own_q[FW-1:0]];
    wire [6:0]  n_cap = hi ? ((n_out > MAX_CH / 2) ? MAX_CH / 2 : n_out) : ((n_out > MAX_CH) ? MAX_CH : n_out);

    always @(posedge clk) begin
        out_valid <= 1'b0; out_frame_ok <= 1'b0; sweep <= 1'b0;
        if (rst) begin
            pbusy <= 1'b0; prd <= 1'b0; tchk <= 1'b0; strobe_d <= 1'b0; underruns <= 32'd0;
        end else begin
            strobe_d <= frame_strobe && enable;
            tchk <= 1'b0;
            if (strobe_d) begin
                tchk <= 1'b1; pts <= play_r; pslot <= play_r[NFW:0]; phi <= hi; pc <= 7'd0; n_eff <= n_cap;
                sweep_ix <= play_r[TGW-1:0];
            end
            if (tchk) begin
                ptag <= ptag_q; sweep <= 1'b1;
                pbusy <= n_eff != 7'd0;
                if (!(ptag_q[15:0] == pts[15:0] && |ptag_q[NFLOW+15:16])) underruns <= underruns + 32'd1;
            end
            prd <= 1'b0;
            if (pbusy) begin
                prd <= 1'b1; prd_ch <= pc[5:0];
                if (pc + 7'd1 >= n_eff) pbusy <= 1'b0;
                pc <= pc + 7'd1;
            end
            if (prd) begin
                out_valid    <= 1'b1;
                out_ch       <= prd_ch;
                out_sample   <= col_ok ? jq : 24'd0;
                out_frame_ok <= prd_ch == 6'd0 && tag_hit && |ptag[NFLOW+15:16];
            end
        end
    end
endmodule
`default_nettype wire
