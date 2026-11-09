// tb_inferno.v - Inferno flows and the CPU packet queue: three TX flows (8 ch x 16 frames 24 bit unicast, 6 ch x 4
// frames 32 bit multicast with a silent channel, 2 ch x 6 frames to ourselves) against golden/inferno_tx.hex
// (tools/golden.py, cross-checked against the upstream-Inferno model by `make xcheck`); the wire looped back into
// inferno_rx with three RX flows mapping them onto port columns, every played frame checked, one flow-0 packet
// corrupted (its frames silent on flow 0's columns only, the other flows play on); CPU frames sent and received
// through cpu_pktq between the audio packets (filtering, claim, SFD time stamps, a rejected send).
// Provenance: written for OpenProSeries (W16). Licence: GPL-3.0.
`timescale 1ns/1ps
`default_nettype none
module tb_inferno;
    localparam PERIOD = 1042, FRAMES = 170, OFFSET = 24, GOLDEN_PACKETS = 24, BAD_F0 = 2;
    localparam [31:0] T0 = 32'h8864BB76;               // (1760000000 x 48000 + 47990) mod 2^32
    localparam [31:0] S0 = 32'd1760000000;
    localparam [16:0] U0 = 17'd47990;
    reg clk = 0, rst = 1;
    always #10 clk = ~clk;

    function [23:0] pattern(input integer frame, input integer ch);
        pattern = ((frame * 32'h010203) ^ (ch << 16) ^ 32'h5A5A5A) & 32'hFFFFFF;
    endfunction

    reg [7:0] golden [0:16383];
    initial $readmemh("golden/inferno_tx.hex", golden);

    // ---- the media clock and the time of day
    reg        frame_strobe = 0;
    reg [31:0] media_ts = T0 - 1, media_sec = S0;
    reg [16:0] media_sub = U0 - 1;
    reg [31:0] tod_sec = 32'd1000;
    reg [29:0] tod_ns = 30'd0;
    always @(posedge clk) begin
        if (tod_ns >= 30'd999999980) begin tod_ns <= 30'd0; tod_sec <= tod_sec + 1; end else tod_ns <= tod_ns + 30'd20;
    end

    // ---- TX side
    reg        in_we = 0;
    reg [5:0]  in_ch = 0;
    reg [23:0] in_sample = 0;
    reg        ttbl_we = 0;
    reg [7:0]  ttbl_a = 0;
    reg [31:0] ttbl_d = 0;
    wire [31:0] ttbl_q;
    wire [7:0] a_data; wire a_valid, a_last, a_ready;
    wire [15:0] tx_pend, tx_run;
    wire [31:0] tx_packets, tx_overruns;
    inferno_tx #(.MAX_CH(64), .NFLOW(16), .MAXFCH(32), .MAX_FPP(32), .NR(128)) dut_tx (
        .clk(clk), .rst(rst), .enable(1'b1), .src_mac(48'h020000000101), .src_ip(32'hC000020A), .fpp_auto(6'd8), .ts_adv(8'd0),
        .tbl_we(ttbl_we), .tbl_waddr(ttbl_a), .tbl_wdata(ttbl_d), .tbl_raddr(ttbl_a), .tbl_rdata(ttbl_q),
        .in_we(in_we), .in_ch(in_ch), .in_sample(in_sample), .frame_strobe(frame_strobe), .media_ts(media_ts),
        .media_sec(media_sec), .media_sub(media_sub), .media_fs(17'd48000),
        .m_tdata(a_data), .m_tvalid(a_valid), .m_tlast(a_last), .m_tready(a_ready),
        .pending(tx_pend), .running(tx_run), .packets_sent(tx_packets), .overruns(tx_overruns));

    // CPU queue registers
    reg  [7:0]  q_addr = 0;
    reg         q_we = 0, q_re = 0;
    reg  [31:0] q_wdata = 0;
    wire [31:0] q_rdata;
    wire [7:0]  b_data; wire b_valid, b_last, b_ready, mine_b;
    wire [7:0]  t_data; wire t_valid, t_last, t_ready;
    tx_arb dut_arb (.clk(clk), .rst(rst), .a_tdata(a_data), .a_tvalid(a_valid), .a_tlast(a_last), .a_tready(a_ready),
        .b_tdata(b_data), .b_tvalid(b_valid), .b_tlast(b_last), .b_tready(b_ready),
        .m_tdata(t_data), .m_tvalid(t_valid), .m_tlast(t_last), .m_tready(t_ready), .mine_b(mine_b));
    wire [1:0] txd; wire tx_en, sfd_tx, und, done;
    rmii_tx dut_mtx (.clk(clk), .rst(rst), .s_tdata(t_data), .s_tvalid(t_valid), .s_tlast(t_last), .s_tready(t_ready),
        .txd(txd), .tx_en(tx_en), .sfd_strobe(sfd_tx), .tx_underrun(und), .tx_done(done));

    // ---- the wire: one bit flipped in flow 0's packet BAD_F0 (its payload)
    integer f0_on_wire = 0, dib_in_pkt = 0;
    reg prev_en = 0;
    reg [7:0] wbuf [0:2047];
    integer wlen = 0;
    reg is_f0;
    always @(posedge clk) begin
        prev_en <= tx_en;
        dib_in_pkt <= tx_en ? dib_in_pkt + 1 : 0;
    end
    wire flip = is_f0 && f0_on_wire == BAD_F0 && dib_in_pkt == 1000;
    wire [1:0] rxd = flip ? txd ^ 2'b01 : txd;

    // ---- RX side
    wire [7:0] r_data; wire r_valid, r_last, r_user, sfd_rx, rok, rcrc, rrunt;
    rmii_rx dut_mrx (.clk(clk), .rst(rst), .crs_dv(tx_en), .rxd(rxd), .m_tdata(r_data), .m_tvalid(r_valid), .m_tlast(r_last),
        .m_tuser(r_user), .sfd_strobe(sfd_rx), .rx_ok(rok), .rx_crc_err(rcrc), .rx_runt(rrunt));
    reg        rtbl_we = 0;
    reg [8:0]  rtbl_a = 0;
    reg [31:0] rtbl_d = 0;
    wire [31:0] rtbl_q;
    wire        claim, out_valid, out_frame_ok;
    wire [5:0]  out_ch;
    wire [23:0] out_sample;
    wire [31:0] p_ok, p_bad, late, early, underruns, ts_gaps;
    inferno_rx #(.MAX_CH(64), .NFLOW(16), .MAXFCH(32), .MAX_SMP(512), .NF(256)) dut_rx (
        .clk(clk), .rst(rst), .enable(1'b1), .n_out(7'd48), .media_fs(17'd48000), .media_ts(media_ts),
        .tbl_we(rtbl_we), .tbl_waddr(rtbl_a), .tbl_wdata(rtbl_d), .tbl_raddr(rtbl_a), .tbl_rdata(rtbl_q),
        .s_tdata(r_data), .s_tvalid(r_valid), .s_tlast(r_last), .s_tuser(r_user), .claim(claim),
        .frame_strobe(frame_strobe), .play_ts(media_ts - OFFSET), .out_valid(out_valid), .out_ch(out_ch),
        .out_sample(out_sample), .out_frame_ok(out_frame_ok), .packets_ok(p_ok), .packets_bad(p_bad), .late(late),
        .early(early), .underruns(underruns), .ts_gaps(ts_gaps));
    cpu_pktq #(.TXW(512), .RXW(2048), .ND(16)) dut_q (
        .clk(clk), .rst(rst), .reg_addr(q_addr), .reg_raddr(q_addr), .reg_we(q_we), .reg_wdata(q_wdata), .reg_re(q_re),
        .rd_hold(1'b0), .reg_rdata(q_rdata),
        .our_mac(48'h020000000101), .tod_sec(tod_sec), .tod_ns(tod_ns),
        .m_tdata(b_data), .m_tvalid(b_valid), .m_tlast(b_last), .m_tready(b_ready), .tx_mine(mine_b), .sfd_tx(sfd_tx),
        .s_tdata(r_data), .s_tvalid(r_valid), .s_tlast(r_last), .s_tuser(r_user), .sfd_rx(sfd_rx), .claim(claim));

    integer errors = 0;
    // ---- golden compare: every Inferno packet on the wire (UDP source port 14400..14402) in order
    integer gi = 0, gpk = 0, n_cpu_wire = 0, k;
    reg [7:0] wb; reg [1:0] wdib = 0;
    always @(posedge clk) begin
        if (tx_en) begin
            wb = {txd, wb[7:2]};
            if (wdib == 2'd3) begin
                wbuf[wlen] = wb; wlen = wlen + 1;
                if (wlen == 8 + 36) is_f0 = (wbuf[8 + 34] == 8'h38 && wbuf[8 + 35] == 8'h40);   // src port 14400
            end
            wdib = wdib + 2'd1;
        end else begin
            wdib = 2'd0;
            if (prev_en) begin
                // a frame ended: Inferno if IPv4 / UDP from 14400..14402
                if (wbuf[8 + 12] == 8'h08 && wbuf[8 + 13] == 8'h00 && wbuf[8 + 23] == 8'd17 && wbuf[8 + 34] == 8'h38
                    && wbuf[8 + 35] >= 8'h40 && wbuf[8 + 35] <= 8'h42) begin
                    if (gpk < GOLDEN_PACKETS) begin
                        for (k = 0; k < wlen; k = k + 1)
                            if (golden[gi + k] !== wbuf[k]) begin
                                if (errors < 10) $display("WIRE packet %0d byte %0d: got %02x want %02x", gpk, k, wbuf[k], golden[gi + k]);
                                errors = errors + 1;
                            end
                        gi = gi + wlen; gpk = gpk + 1;
                    end
                    if (is_f0) f0_on_wire = f0_on_wire + 1;
                end else n_cpu_wire = n_cpu_wire + 1;
                wlen = 0; is_f0 = 0;
            end
        end
    end

    // ---- playout check: every column of every played frame
    integer play_n = -1, p_rel, c_ok = 0, c_silent = 0;
    reg [23:0] want;
    always @(posedge clk) begin
        if (out_valid) begin
            if (out_ch == 0) play_n = play_n + 1;
            p_rel = play_n - OFFSET;                     // the frame's media time - T0
            want = 24'd0;
            if (out_ch < 8 && p_rel >= 10 && !(p_rel >= 10 + 16 * BAD_F0 && p_rel < 10 + 16 * (BAD_F0 + 1)))
                want = pattern(p_rel, out_ch);
            if (p_rel >= 2) case (out_ch)
                20: want = pattern(p_rel, 40); 21: want = pattern(p_rel, 41); 23: want = pattern(p_rel, 47);
                24: want = pattern(p_rel, 3);  25: want = pattern(p_rel, 10);
                default: ;
            endcase
            if (p_rel >= 1 && (out_ch == 30 || out_ch == 31)) want = pattern(p_rel, out_ch == 30 ? 44 : 45);
            if (out_sample !== want) begin
                if (errors < 10) $display("PLAY frame %0d col %0d: got %06x want %06x", p_rel, out_ch, out_sample, want);
                errors = errors + 1;
            end
            if (want != 0) c_ok = c_ok + 1;
        end
    end

    // ---- register helpers
    task twr(input [7:0] a, input [31:0] d); begin @(posedge clk); ttbl_we <= 1; ttbl_a <= a; ttbl_d <= d; @(posedge clk); ttbl_we <= 0; end endtask
    task rwr(input [8:0] a, input [31:0] d); begin @(posedge clk); rtbl_we <= 1; rtbl_a <= a; rtbl_d <= d; @(posedge clk); rtbl_we <= 0; end endtask
    task qw(input [7:0] a, input [31:0] d); begin @(posedge clk); q_we <= 1; q_addr <= a; q_wdata <= d; @(posedge clk); q_we <= 0; repeat (2) @(posedge clk); end endtask
    task qr(input [7:0] a, output [31:0] d);
        begin @(posedge clk); q_re <= 1; q_addr <= a; #1 d = q_rdata; @(posedge clk); q_re <= 0; repeat (3) @(posedge clk); end
    endtask

    // TX flow table (16 words a flow): w0 = {ttl, tos, 3'd0, nch - 1, fpp, b32, en}
    task tx_flow(input integer f, input [5:0] fpp, input b32, input [3:0] nch, input [47:0] dmac, input [31:0] dip,
                 input [15:0] sp, input [15:0] dp, input [7:0] ttl, input [31:0] map0, input [31:0] map1);
        begin
            twr(f * 16 + 1, {16'd0, dmac[47:32]}); twr(f * 16 + 2, dmac[31:0]); twr(f * 16 + 3, dip);
            twr(f * 16 + 4, {sp, dp}); twr(f * 16 + 5, map0); twr(f * 16 + 6, map1);
            twr(f * 16 + 0, {ttl, 8'hB8, 3'd0, {1'b0, nch} - 5'd1, fpp, b32, 1'b1});
        end
    endtask
    // RX flow configuration (16 words a flow): w0 = {bps 18:16, nch 13:8, en}
    task rx_flow(input integer f, input [4:0] nch, input [2:0] bps, input [31:0] dip, input [15:0] dp, input [31:0] shift,
                 input [31:0] map0, input [31:0] map1);
        integer i;
        begin
            rwr(f * 16 + 1, dip); rwr(f * 16 + 2, {16'd0, dp}); rwr(f * 16 + 3, shift);
            rwr(f * 16 + 4, map0); rwr(f * 16 + 5, map1);
            for (i = 6; i < 12; i = i + 1) rwr(f * 16 + i, 32'h80808080);
            rwr(f * 16 + 0, {13'd0, bps, 2'd0, 1'b0, nch, 7'd0, 1'b1});
        end
    endtask

    // ---- CPU frames
    reg [7:0] fr [0:127];
    integer fl;
    task cpu_send(input integer len, input [7:0] tag, input tsreq);
        integer i;
        begin
            for (i = 0; i < len; i = i + 4) qw(8'h0C, {fr[i], fr[i + 1], fr[i + 2], fr[i + 3]});
            qw(8'h10, {8'd0, tag, tsreq, 4'd0, len[10:0]});
        end
    endtask
    task mk_frame(input [47:0] dst, input [15:0] etype, input integer len, input integer seed);
        integer i;
        begin
            for (i = 0; i < 128; i = i + 1) fr[i] = (i * 7 + seed) & 8'hFF;
            {fr[0], fr[1], fr[2], fr[3], fr[4], fr[5]} = dst;
            {fr[6], fr[7], fr[8], fr[9], fr[10], fr[11]} = 48'h020000000909;
            {fr[12], fr[13]} = etype;
            for (i = len; i < 128; i = i + 1) fr[i] = 8'h00;
        end
    endtask
    task mk_ipv4(input [31:0] dip, input [47:0] dmac, input [15:0] port);
        begin
            mk_frame(dmac, 16'h0800, 64, 3);
            fr[14] = 8'h45; fr[23] = 8'd17; {fr[30], fr[31], fr[32], fr[33]} = dip; {fr[34], fr[35]} = port; {fr[36], fr[37]} = port;
        end
    endtask
    // check the head of the RX queue against fr[0 .. len-1], then pop it
    reg [31:0] v, ts_tx_s, ts_tx_n, ts_rx_s, ts_rx_n, stamp_d;
    task cpu_recv(input integer len, input [8*12-1:0] what);
        integer i, bad;
        begin
            bad = 0;
            qr(8'h34, v);
            if (v !== {1'b1, 20'd0, len[10:0]}) begin $display("RX %0s: RX_LEN %08x want length %0d", what, v, len); errors = errors + 1; end
            qr(8'h38, ts_rx_s); qr(8'h3C, ts_rx_n);
            for (i = 0; i < len; i = i + 4) begin
                qr(8'h40, v);
                if (v !== {fr[i], fr[i + 1], fr[i + 2], fr[i + 3]}) bad = bad + 1;
            end
            if (bad) begin $display("RX %0s: %0d words differ", what, bad); errors = errors + 1; end
            qw(8'h44, 32'd1);
        end
    endtask

    // ---- the audio source: 48 channels, then the frame strobe
    integer f, c, rx_frames_cpu;
    reg cpu_go = 0, cpu_done = 0;
    initial begin
        if ($test$plusargs("vcd")) begin $dumpfile("build/tb_inferno.vcd"); $dumpvars(0, tb_inferno); end
        repeat (5) @(posedge clk); rst <= 0;
        repeat (5) @(posedge clk);
        // flows (as golden.py INF_FLOWS): flow 0 8 ch x 16, 24 bit; flow 1 6 ch x 4, 32 bit, ch 2 silent; flow 2 2 ch x 6
        tx_flow(0, 6'd16, 1'b0, 4'd8, 48'h020000000202, 32'hC0000214, 16'd14400, 16'd14336, 8'd64, 32'h03020100, 32'h07060504);
        tx_flow(1, 6'd4,  1'b1, 4'd6, 48'h01005E7F0102, 32'hEFFF0102, 16'd14401, 16'd4321,  8'd32, 32'h2F802928, 32'h80800A03);
        tx_flow(2, 6'd6,  1'b0, 4'd2, 48'h020000000101, 32'hC000020A, 16'd14402, 16'd14338, 8'd64, 32'h80802D2C, 32'h80808080);
        // RX: flow 3 <- TX flow 0 on columns 0..7; flow 5 <- TX flow 1 on 20..25 (its silent channel ignored);
        // flow 9 <- TX flow 2 on 30, 31
        rx_flow(3, 5'd8, 3'd3, 32'hC0000214, 16'd14336, 32'd0, 32'h03020100, 32'h07060504);
        rx_flow(5, 5'd6, 3'd4, 32'hEFFF0102, 16'd4321,  32'd0, 32'h17801514, 32'h80801918);
        rx_flow(9, 5'd2, 3'd3, 32'hC000020A, 16'd14338, 32'd0, 32'h80801F1E, 32'h80808080);
        // the table reads back
        ttbl_a <= 8'd22; @(posedge clk); #1 if (ttbl_q !== 32'h80800A03) begin errors = errors + 1; $display("TX table readback %08x", ttbl_q); end
        rtbl_a <= 9'd80 + 9'd4; @(posedge clk); #1 if (rtbl_q !== 32'h17801514) begin errors = errors + 1; $display("RX table readback %08x", rtbl_q); end
        for (f = 0; f < FRAMES; f = f + 1) begin
            for (c = 0; c < 48; c = c + 1) begin
                @(posedge clk); in_we <= 1; in_ch <= c; in_sample <= pattern(f, c);
            end
            @(posedge clk); in_we <= 0; frame_strobe <= 1; media_ts <= T0 + f;
            if (media_sub == 17'd47999) begin media_sub <= 17'd0; media_sec <= media_sec + 1; end else media_sub <= media_sub + 1;
            @(posedge clk); frame_strobe <= 0;
            if (f == 60) cpu_go = 1;
            repeat (PERIOD - 48 - 2) @(posedge clk);
        end
        wait (cpu_done);
        repeat (200) @(posedge clk);
        // ---- counters
        if (gpk != GOLDEN_PACKETS) begin $display("only %0d golden packets compared", gpk); errors = errors + 1; end
        if (p_bad != 1)  begin $display("RX bad %0d, want 1 (the corrupted packet)", p_bad); errors = errors + 1; end
        if (ts_gaps != 1) begin $display("RX timestamp gaps %0d, want 1", ts_gaps); errors = errors + 1; end
        if (late != 0 || early != 0 || tx_overruns != 0) begin $display("late %0d early %0d overruns %0d", late, early, tx_overruns); errors = errors + 1; end
        // RX flow status: flow 9's source is ourselves, port 14402
        rtbl_a <= 9'd256 + 9'd144; @(posedge clk); #1 if (rtbl_q !== 32'hC000020A) begin errors = errors + 1; $display("status src ip %08x", rtbl_q); end
        rtbl_a <= 9'd256 + 9'd145; @(posedge clk); #1 if (rtbl_q !== 32'd14402) begin errors = errors + 1; $display("status src port %0d", rtbl_q); end
        rtbl_a <= 9'd256 + 9'd146; @(posedge clk); #1 if (rtbl_q < 20) begin errors = errors + 1; $display("status packets %0d", rtbl_q); end
        rtbl_a <= 9'd256 + 9'd147; @(posedge clk); #1 $display("flow 9 latency peak %0d frames", rtbl_q);
        if (errors == 0)
            $display("PASS tb_inferno: %0d golden packets byte-exact (3 flows: 24/32 bit, fpp 16/4/6, second roll-over); %0d packets looped, %0d played samples checked (OFFSET %0d) with the corrupted flow-0 packet silent on its columns only; CPU queue: %0d frames round trip, 2 filtered, 1 reject, TX->RX SFD stamps %0d ns apart in loopback",
                     GOLDEN_PACKETS, p_ok, c_ok, OFFSET, rx_frames_cpu, stamp_d);
        else $display("FAIL tb_inferno: %0d errors (golden %0d, ok %0d bad %0d gaps %0d late %0d early %0d)", errors, gpk, p_ok, p_bad, ts_gaps, late, early);
        $finish;
    end

    // ---- the CPU path, while the audio runs
    initial begin : cpu
        wait (cpu_go);
        qr(8'h00, v); if (v !== 32'h4F435131) begin errors = errors + 1; $display("pktq ID %08x", v); end
        qw(8'h04, 32'h0000_000F);              // TX, RX, all multicast, drop 239/8
        // A: to us, 60 bytes, time-stamped, tag 5A
        mk_frame(48'h020000000101, 16'h88B5, 60, 1); cpu_send(60, 8'h5A, 1'b1);
        // B: broadcast, 42 bytes (arrives padded to 60)
        mk_frame(48'hFFFFFFFFFFFF, 16'h0806, 42, 2); cpu_send(44, 8'h00, 1'b0);   // (44: a whole word count; 2 bytes 0)
        // C: someone else's unicast: filtered
        mk_frame(48'h020000000707, 16'h88B5, 60, 3); cpu_send(60, 8'h00, 1'b0);
        // D: IPv4 to 239.1.2.3 (another device's AoIP media): filtered; E: PTP event to 224.0.1.129:319: taken
        mk_ipv4(32'hEF010203, 48'h01005E010203, 16'd5004); cpu_send(64, 8'h00, 1'b0);
        mk_ipv4(32'hE0000181, 48'h01005E000181, 16'd319); cpu_send(64, 8'h00, 1'b0);
        // a send whose length does not match the words written: rejected
        qw(8'h0C, 32'h01020304); qw(8'h10, {16'd0, 16'd60});
        qr(8'h54, v); if (v !== 1) begin errors = errors + 1; $display("TX rejects %0d", v); end
        repeat (40000) @(posedge clk);         // all sent and looped back
        qr(8'h50, v); if (v !== 5) begin errors = errors + 1; $display("TX frames %0d, want 5", v); end
        qr(8'h18, v); if (v !== 32'h8000005A) begin errors = errors + 1; $display("TX_TS_STATUS %08x", v); end
        qr(8'h1C, ts_tx_s); qr(8'h20, ts_tx_n);
        qr(8'h18, v); if (v[31]) begin errors = errors + 1; $display("TX_TS_STATUS still valid"); end
        qr(8'h30, v); rx_frames_cpu = v; if (v !== 3) begin errors = errors + 1; $display("RX frames waiting %0d, want 3 (A, B, E)", v); end
        mk_frame(48'h020000000101, 16'h88B5, 60, 1); cpu_recv(60, "A");
        stamp_d = ts_rx_n - ts_tx_n;
        if (ts_rx_s !== ts_tx_s || ts_rx_n < ts_tx_n || ts_rx_n - ts_tx_n > 200) begin
            errors = errors + 1; $display("stamps: TX %0d.%09d RX %0d.%09d", ts_tx_s, ts_tx_n, ts_rx_s, ts_rx_n);
        end
        mk_frame(48'hFFFFFFFFFFFF, 16'h0806, 42, 2); cpu_recv(60, "B");
        mk_ipv4(32'hE0000181, 48'h01005E000181, 16'd319); cpu_recv(64, "E");
        qr(8'h30, v); if (v !== 0) begin errors = errors + 1; $display("RX queue not empty: %0d", v); end
        qr(8'h60, v); if (v !== 2) begin errors = errors + 1; $display("RX filtered %0d, want 2 (C, D)", v); end
        qr(8'h5C, v); if (v !== 0) begin errors = errors + 1; $display("RX overflows %0d", v); end
        if (n_cpu_wire != 5) begin errors = errors + 1; $display("%0d CPU frames seen on the wire", n_cpu_wire); end
        cpu_done = 1;
    end
    initial begin #20000000; $display("FAIL tb_inferno: timeout"); $finish; end
endmodule
`default_nettype wire
