// inferno_tx.v - Inferno (Dante-compatible) audio flow packetiser: up to NFLOW flows per port, each up to MAXFCH
// channels picked from the port's MAX_CH, sent as Ethernet II / IPv4 / UDP datagrams in the Inferno media format.
//
// Provenance: written for OpenProSeries (W16). The media packet follows upstream Inferno (github.com/teodly/inferno,
// GPL-3.0, commit 9767558e, inferno_aoip/src/device_server/flows_tx.rs `transmit()` and samples_utils.rs
// `write_s24_samples` / `write_s32_samples`); the lower layers are the public standards (IEEE 802.3, RFC 791 / 1071 /
// 768; udp_ipv4.vh). No vendor material. Licence: GPL-3.0.
//
// The UDP payload of one packet ("flow packet", 9 + fpp x nch x bps bytes):
//   byte 0      0x02                                   (upstream: "pbuff[0] = 2u8; // ???")
//   bytes 1..4  seconds,     u32 big-endian            = Ts / fs
//   bytes 5..8  sub-second,  u32 big-endian, samples   = Ts mod fs
//   then fpp frames, each nch samples big-endian, channel order of the flow's map (bps 3: 24 bit; bps 4: the
//   24-bit sample left-justified in 32 bits, low byte 0)
// Ts = T - ts_adv, where T is the media time (samples since the PTP epoch at fs) of the packet's first frame. ts_adv
// (inferno_regs TX_TS_ADVANCE) is 0 by default; upstream Inferno stamps 0.5 ms early (CLOCK_OFFSET_NS = -500000, i.e.
// 24 frames at 48 kHz). The desk's media clock keeps T mod 2^32 in media_ts and (T / fs, T mod fs) in (media_sec,
// media_sub) (ptp_clock.v).
// IPv4: DSCP / TOS and TTL per flow (DSCP 46 = 0xB8, TTL 64 unicast / 32 multicast are the control plane's
// defaults), DF, IP id counting packets, UDP checksum 0.
//
// Audio side: write a frame's channels (in_we, in_ch, in_sample), then frame_strobe with media_ts = the media time of
// that frame. Samples go into a ring of NR frames x MAX_CH (slot = media time mod NR), so a flow reads its channels for
// any recent frame. A flow's packet is due when its fpp-th frame has been strobed; due flows are sent one after
// another, the lowest-numbered first. With a power-of-two fpp the packet boundaries are aligned to multiples of fpp of
// the media time, as upstream's bootstrap_next_ts; other fpp start at the next frame.
//
// Frames per packet: the flow's fpp field, or (field 0) the latency mode's automatic fpp (fpp_auto, inferno_lat.vh),
// then cut to MAX_FPP and to the largest power of two whose payload fits a 1500-byte IPv4 packet (inferno_mtu_fpp).
// A flow whose effective fpp changes (a LATENCY write, a w0 write) restarts on its next aligned boundary.
//
// Flow table (NFLOW x 16 words, written / read by the host through inferno_regs.v; word address = flow * 16 + w):
//   w0 CTRL     bit0 enable, bit1 32-bit samples (else 24), bits 7:2 fpp (0 = automatic, else 1..MAX_FPP),
//               bits 12:8 channels - 1 (1..MAXFCH), bits 23:16 TOS, bits 31:24 TTL. Writing w0 (re)starts the flow;
//               write it last.
//   w1 DST_MAC_HI (15:0)   w2 DST_MAC_LO   w3 DST_IP   w4 PORTS (src << 16 | dst)
//   w5..w12 MAP: one byte per flow channel c (word 5 + c / 4, bits 8 x (c mod 4) + 7 ..): bits 5:0 port channel,
//               bit 7 = silence
//   w13..w15 spare (read back)
// The table is block RAM with registered reads: the host's read data follow the address one clock later.
`include "clog2.vh"
`timescale 1ns / 1ps
`default_nettype none
module inferno_tx #(
    parameter MAX_CH  = 64,            // port columns (a power of two, 2..64)
    parameter NFLOW   = 16,
    parameter MAXFCH  = 32,            // channels per flow (<= 32)
    parameter MAX_FPP = 32,            // frames per packet (<= 32)
    parameter NR      = 128            // ring frames (a power of two, >= 2 x MAX_FPP + 8)
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        enable,
    input  wire [47:0] src_mac,
    input  wire [31:0] src_ip,
    input  wire [5:0]  fpp_auto,       // the latency mode's frames per packet (inferno_lat_fpp)
    input  wire [7:0]  ts_adv,         // frames the timestamp is stamped early (0: the first frame's time)
    // flow table
    input  wire        tbl_we,
    input  wire [`CLOG2(NFLOW)+3:0] tbl_waddr,
    input  wire [31:0] tbl_wdata,
    input  wire [`CLOG2(NFLOW)+3:0] tbl_raddr,
    output reg  [31:0] tbl_rdata,
    // audio in and the media clock
    input  wire        in_we,
    input  wire [5:0]  in_ch,
    input  wire [23:0] in_sample,
    input  wire        frame_strobe,
    input  wire [31:0] media_ts,
    input  wire [31:0] media_sec,
    input  wire [16:0] media_sub,
    input  wire [16:0] media_fs,
    // frames out (to rmii_tx through the port's arbiter)
    output wire [7:0]  m_tdata,
    output wire        m_tvalid,
    output wire        m_tlast,
    input  wire        m_tready,
    // status
    output reg  [NFLOW-1:0] pending,
    output wire [NFLOW-1:0] running,
    output reg  [31:0] packets_sent,
    output reg  [31:0] overruns         // a flow became due while its previous packet still waited
);
`include "udp_ipv4.vh"
`include "inferno_lat.vh"
    localparam FW  = `CLOG2(NFLOW);
    localparam TW  = FW + 4;
    localparam RW  = `CLOG2(NR);
    localparam CW  = `CLOG2(MAX_CH);
    localparam HDR = 51;               // 42 Ethernet / IPv4 / UDP + 9 media header

    // ---- flow table (block RAM, reads registered): port A is the host's (the write address while writing, else the
    // read address; aoip_port gives both the same bus address), port B the sender's
    reg  [31:0] tbl [0:NFLOW*16-1];
    integer ti;
    initial for (ti = 0; ti < NFLOW * 16; ti = ti + 1) tbl[ti] = 32'd0;
    wire [TW-1:0] tbl_a = tbl_we ? tbl_waddr : tbl_raddr;
    always @(posedge clk) begin
        if (tbl_we) tbl[tbl_a] <= tbl_wdata;
        tbl_rdata <= tbl[tbl_a];
    end

    // ---- per-flow scheduler state (flip-flops)
    wire [NFLOW-1:0] due_v, run_v;
    wire [6*NFLOW-1:0] cnt_v, fpp_v;
    wire [5:0] fpp_cap = MAX_FPP;
    wire [4:0] w_nchm1 = tbl_wdata[12:8];
    wire [7:0] w_nchb  = tbl_wdata[1] ? {1'b0, w_nchm1, 2'b00} + 8'd4 : {3'd0, w_nchm1} + {2'd0, w_nchm1, 1'b0} + 8'd3;
    genvar g;
    generate
        for (g = 0; g < NFLOW; g = g + 1) begin : gf
            // the flow's configuration follows w0 (like the table RAM, it is not cleared by the engine's reset, so
            // a flow written while the port changes mode is kept); run, cnt and fpp_r are the running state
            reg       en_r = 1'b0, auto_r = 1'b1;
            reg [5:0] fpp_req = 6'd0, maxf = 6'd32;
            reg       run;
            reg [5:0] fpp_r, cnt;
            wire      w0   = tbl_we && tbl_waddr == g * 16;
            wire [5:0] req  = auto_r ? fpp_auto : fpp_req;
            wire [5:0] req1 = (req == 6'd0) ? 6'd1 : req;
            wire [5:0] lim  = (maxf < fpp_cap) ? maxf : fpp_cap;
            wire [5:0] want = (req1 > lim) ? lim : req1;
            wire      pow2 = (fpp_r & (fpp_r - 6'd1)) == 6'd0;
            wire      due  = frame_strobe && enable && run && (cnt + 6'd1 >= fpp_r);
            always @(posedge clk) if (w0) begin
                en_r    <= tbl_wdata[0];
                auto_r  <= tbl_wdata[7:2] == 6'd0;
                fpp_req <= tbl_wdata[7:2];
                maxf    <= inferno_mtu_fpp(w_nchb);
            end
            always @(posedge clk) begin
                if (rst) begin
                    run <= 1'b0; cnt <= 6'd0; fpp_r <= 6'd1;
                end else if (w0) begin
                    run <= 1'b0; cnt <= 6'd0;
                end else if (want != fpp_r) begin
                    fpp_r <= want; run <= 1'b0; cnt <= 6'd0;
                end else if (!enable || !en_r) begin
                    run <= 1'b0; cnt <= 6'd0;
                end else if (frame_strobe) begin
                    if (run) cnt <= due ? 6'd0 : cnt + 6'd1;
                    // start so that the first packet begins at media time media_ts + 1
                    else if (!pow2 || ((media_ts[5:0] + 6'd1) & (fpp_r - 6'd1)) == 6'd0) begin run <= 1'b1; cnt <= 6'd0; end
                end
            end
            assign due_v[g] = due;
            assign run_v[g] = run;
            assign cnt_v[6*g +: 6] = cnt;
            assign fpp_v[6*g +: 6] = fpp_r;
        end
    endgenerate
    assign running = run_v;

    // ---- the sample ring: NR frames x MAX_CH; a frame written before its strobe belongs to media time media_ts + 1
    reg [23:0] ring [0:NR*MAX_CH-1];
    wire [RW-1:0] wslot = media_ts[RW-1:0] + 1'b1;
    always @(posedge clk) if (in_we && in_ch < MAX_CH) ring[{wslot, in_ch[CW-1:0]}] <= in_sample;

    // ---- sender
    localparam S_IDLE = 2'd0, S_LOAD = 2'd1, S_CSUM = 2'd2, S_SEND = 2'd3;
    reg  [1:0]    state;
    reg  [FW-1:0] cur;
    reg  [FW-1:0] pick;                // the lowest pending flow
    integer pi;
    always @(*) begin
        pick = {FW{1'b0}};
        for (pi = NFLOW - 1; pi >= 0; pi = pi - 1) if (pending[pi]) pick = pi;
    end
    reg  [3:0]    lw;                  // table word being addressed (its data arrive one clock later)
    reg  [31:0]   c_w0, c_dip, c_ports;
    reg  [47:0]   c_dmac;
    reg  [5:0]    c_fpp;
    reg  [31:0]   p_sec;               // the packet's stamp: seconds, sub-second samples; its first frame's ring slot
    reg  [16:0]   p_sub;
    reg  [RW-1:0] p_slot;
    reg  [15:0]   ip_id;
    reg  [15:0]   csum;
    reg  [10:0]   idx, plen;
    // the header lengths and the IPv4 checksum, computed during S_LOAD one step a clock (the header is per packet;
    // the multiply -> lengths -> checksum chain was one 27 ns path on a Spartan-3)
    reg  [7:0]    nchb_r;
    reg  [13:0]   plen_m;
    reg  [15:0]   udp_len, ip_len;
    reg  [10:0]   plast;               // idx of the packet's last byte
    reg  [17:0]   cs_a, cs_b, cs_c;
    reg  [19:0]   cs_s;
    reg  [16:0]   cs_f;
    reg  [15:0]   csum_r;
    reg  [5:0]    rf;                  // payload walk: frame, channel, byte of the sample
    reg  [4:0]    rc;
    reg  [1:0]    rb;
    wire [4:0]    c_nchm1 = c_w0[12:8];
    wire          c_b32 = c_w0[1];
    wire [7:0]    c_nchb = c_b32 ? {1'b0, c_nchm1, 2'b00} + 8'd4 : {3'd0, c_nchm1} + {2'd0, c_nchm1, 1'b0} + 8'd3;
    // The pick and the packet's stamp are a three-stage pipeline (the pick -> counters -> stamp chain was the critical
    // path): stage 1 the lowest pending flow, stage 2 its fpp and the frame counts back to its first frame and to the
    // stamp, stage 3 the first frame's ring slot and the stamp as (seconds, samples). Pending flows, the counters and
    // the media clock change only with a frame strobe, so the result is used only when no strobe came in the last
    // three clocks nor in this one (sq >= 3, !frame_strobe): then every stage saw the same state.
    reg  [FW-1:0] pick_r, pick_rr, pick_rrr;
    reg  [5:0]    pk_fpp, pk_fpp_r;
    reg  [6:0]    back;               // media_ts - back = the completed packet's first frame
    reg  [8:0]    sback;              // media_ts - sback = the stamp
    reg           any_r, any_rr, any_rrr;
    reg  [2:0]    sq;                 // clocks since the last frame strobe (saturates at 4)
    reg  [RW-1:0] slot_c;
    reg  [31:0]   sec_c;
    reg  [16:0]   sub_c;
    always @(posedge clk) begin
        pick_r   <= pick;
        any_r    <= |pending;
        pick_rr  <= pick_r;
        any_rr   <= any_r;
        pk_fpp   <= fpp_v[6*pick_r +: 6];
        back     <= cnt_v[6*pick_r +: 6] + fpp_v[6*pick_r +: 6] - 7'd1;
        sback    <= cnt_v[6*pick_r +: 6] + fpp_v[6*pick_r +: 6] - 9'd1 + {1'b0, ts_adv};
        pick_rrr <= pick_rr;
        any_rrr  <= any_rr;
        pk_fpp_r <= pk_fpp;
        slot_c   <= media_ts[RW-1:0] - back;
        if (media_sub >= {8'd0, sback}) begin sub_c <= media_sub - sback; sec_c <= media_sec; end
        else begin sub_c <= media_sub + media_fs - sback; sec_c <= media_sec - 32'd1; end
        sq       <= rst ? 3'd4 : frame_strobe ? 3'd0 : (sq == 3'd4) ? 3'd4 : sq + 3'd1;
    end
    reg  [31:0]   tbl_q;
    always @(posedge clk) tbl_q <= tbl[{cur, lw}];
    // the flow's channel map (w5..w12, one byte per flow channel) in an 8-word LUT RAM, loaded in S_LOAD, read
    // asynchronously by the payload walk (it was a 256-bit register)
    reg  [31:0]   cmapm [0:7];
    wire          cmap_we = !rst && state == S_LOAD && lw >= 4'd6 && lw <= 4'd13;
    wire [2:0]    cmap_wa = lw[2:0] - 3'd6;
    always @(posedge clk) if (cmap_we) cmapm[cmap_wa] <= tbl_q;

    always @(posedge clk) begin
        if (rst) begin
            state <= S_IDLE; pending <= {NFLOW{1'b0}}; ip_id <= 16'd0;
            packets_sent <= 32'd0; overruns <= 32'd0;
        end else begin
            pending <= (pending | due_v) & run_v;
            if (|(due_v & pending)) overruns <= overruns + 32'd1;
            case (state)
                S_IDLE: begin
                    // the lowest-numbered due flow first (deterministic: flows due at the same frame go out in
                    // flow order); never latch in a strobe clock (media_ts and the counters change together there)
                    if (any_rrr && pending[pick_rrr] && !frame_strobe && sq >= 3'd3) begin
                        cur <= pick_rrr; lw <= 4'd0; state <= S_LOAD;
                        pending[pick_rrr] <= 1'b0;
                        c_fpp  <= pk_fpp_r;
                        p_slot <= slot_c;
                        p_sub  <= sub_c; p_sec <= sec_c;
                    end
                end
                S_LOAD: begin
                    // word lw - 1 is on tbl_q
                    case (lw)
                        4'd1: c_w0 <= tbl_q;
                        4'd2: c_dmac[47:32] <= tbl_q[15:0];
                        4'd3: c_dmac[31:0] <= tbl_q;
                        4'd4: c_dip <= tbl_q;
                        4'd5: c_ports <= tbl_q;
                        default: ;
                    endcase
                    // lengths and checksum (c_w0 is valid from lw 2, c_dip from lw 5; ip_id only changes in S_SEND)
                    case (lw)
                        4'd2: nchb_r <= c_nchb;
                        4'd3: plen_m <= c_fpp * nchb_r;
                        4'd4: plen <= plen_m[10:0];
                        4'd5: begin
                            udp_len <= 16'd17 + {5'd0, plen}; ip_len <= 16'd37 + {5'd0, plen};
                            plast   <= HDR + plen - 11'd1;
                        end
                        4'd6: begin
                            cs_a <= {2'd0, 8'h45, c_w0[23:16]} + {2'd0, ip_len} + {2'd0, ip_id} + 18'h4000;
                            cs_b <= {2'd0, c_w0[31:24], 8'd17} + {2'd0, src_ip[31:16]} + {2'd0, src_ip[15:0]};
                            cs_c <= {2'd0, c_dip[31:16]} + {2'd0, c_dip[15:0]};
                        end
                        4'd7: cs_s <= {2'd0, cs_a} + {2'd0, cs_b} + {2'd0, cs_c};
                        4'd8: cs_f <= {1'b0, cs_s[15:0]} + {13'd0, cs_s[19:16]};
                        4'd9: csum_r <= ~(cs_f[15:0] + {15'd0, cs_f[16]});
                        default: ;
                    endcase
                    lw <= lw + 4'd1;
                    if (lw == 4'd13) state <= S_CSUM;
                end
                S_CSUM: begin
                    csum <= csum_r;
                    idx <= 11'd0; rf <= 6'd0; rc <= 5'd0; rb <= 2'd0;
                    state <= S_SEND;
                end
                default: begin
                    if (m_tready) begin
                        idx <= idx + 11'd1;
                        if (idx >= HDR) begin
                            if (rb == (c_b32 ? 2'd3 : 2'd2)) begin
                                rb <= 2'd0;
                                if (rc == c_nchm1) begin rc <= 5'd0; rf <= rf + 6'd1; end
                                else rc <= rc + 5'd1;
                            end else rb <= rb + 2'd1;
                        end
                        if (m_tlast) begin
                            state <= S_IDLE; ip_id <= ip_id + 16'd1; packets_sent <= packets_sent + 32'd1;
                        end
                    end
                end
            endcase
            if (!enable) begin pending <= {NFLOW{1'b0}}; if (state != S_SEND) state <= S_IDLE; end
        end
    end

    // the sample under the payload pointer (one clock after the pointer moves; the MAC takes a byte at most
    // every 4 clocks)
    wire [31:0]   mapw = cmapm[rc[4:2]];
    wire [7:0]    mapb = (rc[1:0] == 2'd0) ? mapw[7:0] : (rc[1:0] == 2'd1) ? mapw[15:8] : (rc[1:0] == 2'd2) ? mapw[23:16] : mapw[31:24];
    wire          mute = mapb[7] || mapb[5:0] >= MAX_CH || rc >= MAXFCH;
    wire [RW-1:0] rslot = p_slot + rf;
    reg  [23:0]   smp;
    reg           smute;
    always @(posedge clk) begin
        smp   <= ring[{rslot, mapb[CW-1:0]}];
        smute <= mute;
    end

    reg [7:0] hb;
    always @(*) begin
        case (idx)
            11'd42: hb = 8'h02;
            11'd43: hb = p_sec[31:24]; 11'd44: hb = p_sec[23:16]; 11'd45: hb = p_sec[15:8]; 11'd46: hb = p_sec[7:0];
            11'd47: hb = 8'h00;        11'd48: hb = {7'd0, p_sub[16]}; 11'd49: hb = p_sub[15:8]; 11'd50: hb = p_sub[7:0];
            default: hb = eth_ipv4_udp_byte(idx[5:0], c_dmac, src_mac, c_w0[23:16], ip_len, ip_id, c_w0[31:24], csum,
                                            src_ip, c_dip, c_ports[31:16], c_ports[15:0], udp_len);
        endcase
    end
    wire [7:0] pb = smute ? 8'h00 : (rb == 2'd0) ? smp[23:16] : (rb == 2'd1) ? smp[15:8] : (rb == 2'd2) ? smp[7:0] : 8'h00;
    assign m_tdata  = (idx < HDR) ? hb : pb;
    assign m_tvalid = (state == S_SEND);
    assign m_tlast  = (state == S_SEND) && (idx == plast);
    wire unused = &{1'b0, plen_m[13:11], tbl_wdata[15:13]};
endmodule
`default_nettype wire
