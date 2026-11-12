// The engines' octet pace (the limits stated in the inferno_rx / inferno_tx / cpu_pktq headers): each RX limit is
// swept to its exact minimum (the minimum passes, one clock less fails) and compared with the documented value;
// inferno_tx and cpu_pktq run at 4 (reference), 2 and 1 clocks an octet and must match the reference at 2 and differ at
// 1. Licence: GPL-3.0.
`timescale 1ns/1ps
`default_nettype none
module tb_inferno_pace;
    // documented limits (clocks)
    localparam EXP_UNIFORM = 3, EXP_UNIFORM_F11 = 2, EXP_H37 = 15, EXP_T46 = 17, EXP_COMMIT = 3;
    reg clk = 0, rst = 1;
    always #10 clk = ~clk;
    integer errors = 0, cyc = 0, t28, tend;
    always @(posedge clk) cyc <= cyc + 1;
    // ================= RX
    reg we = 0, valid = 0, last = 0;
    reg [8:0] addr = 0;
    reg [31:0] wd = 0;
    reg [7:0] data = 0;
    wire [31:0] ok, bad;
    inferno_rx rx (.clk(clk), .rst(rst), .enable(1'b1), .n_out(7'd17),
        .media_fs(17'd48000), .media_ts(32'd130), .tbl_we(we), .tbl_waddr(addr), .tbl_wdata(wd),
        .tbl_raddr(addr), .tbl_rdata(), .s_tdata(data), .s_tvalid(valid), .s_tlast(last), .s_tuser(1'b0),
        .claim(), .frame_strobe(1'b0), .play_ts(32'd100), .out_valid(), .out_ch(), .out_sample(),
        .out_frame_ok(), .packets_ok(ok), .packets_bad(bad), .late(), .early(), .underruns(), .ts_gaps());
    task cfg(input [8:0] a, input [31:0] d);
        begin @(negedge clk); we = 1; addr = a; wd = d; @(negedge clk); we = 0; end
    endtask
    reg [7:0] pkt [0:1499];
    integer plen;
    task build(input integer flow, input integer nch, input integer fpp);
        integer i;
        begin
            plen = 51 + nch * fpp * 3;
            for (i = 0; i < 1500; i = i + 1) pkt[i] = i;
            for (i = 0; i < 51; i = i + 1) pkt[i] = 0;
            pkt[12] = 8'h08; pkt[14] = 8'h45;
            pkt[16] = (plen - 14) >> 8; pkt[17] = plen - 14;
            pkt[22] = 64; pkt[23] = 17;
            pkt[26] = 192; pkt[28] = 2; pkt[29] = 11;
            pkt[30] = 192; pkt[32] = 2; pkt[33] = 10;
            pkt[34] = 8'hC3; pkt[35] = 8'h60;
            pkt[36] = (50000 + flow) >> 8; pkt[37] = 50000 + flow;
            pkt[38] = (plen - 34) >> 8; pkt[39] = plen - 34;
            pkt[42] = 2; pkt[50] = 100;
        end
    endtask
    // send pkt: u clocks an octet; octet 50 h37 clocks after octet 37, octet 38 g38 after 37, the last octet t46 after
    // octet 46 (0: u)
    task send(input integer u, input integer h37, input integer g38, input integer t46);
        integer i, g;
        begin
            for (i = 0; i < plen; i = i + 1) begin
                g = u;
                if (h37 && i == 50) g = h37 - 12 * u;
                if (g38 && i == 38) g = g38;
                if (t46 && i == plen - 1) g = t46 - (plen - 48) * u;
                if (g < 1) g = 1;
                if (i && g > 1) begin @(negedge clk); valid = 0; last = 0; repeat (g - 2) @(negedge clk); end
                @(negedge clk); valid = 1; data = pkt[i]; last = i == plen - 1;
                if (i == 28) t28 = cyc;
                if (i == plen - 1) tend = cyc;
            end
            @(negedge clk); valid = 0; last = 0;
        end
    endtask
    integer ok0;
    // one 1 x 3 packet to the flow with the given timing; 1 when taken
    task try(input integer flow, input integer u, input integer h37, input integer g38, input integer t46,
             output integer took);
        begin
            build(flow, 1, 3); ok0 = ok; send(u, h37, g38, t46); repeat (700) @(negedge clk); took = ok != ok0;
        end
    endtask
    // a long packet (flow 0: nch x fpp), a 1 x 3 packet (flow 1) straight after it, then after gap clocks one to flow 2:
    // 1 when all three are taken; dist = the long packet's last octet to flow 2's octet 28
    integer t_long, dist;
    task try_commit(input integer nch, input integer fpp, input integer gap, output integer took);
        begin
            ok0 = ok; build(0, nch, fpp); send(3, 0, 0, 0); t_long = tend;
            build(1, 1, 3); send(3, 0, 0, 0); repeat (gap) @(negedge clk);
            build(2, 1, 3); send(3, 0, 0, 0); dist = t28 - t_long;
            repeat (1500) @(negedge clk); took = ok - ok0 == 3;
        end
    endtask
    integer v, took, m;
    task check(input [8*40-1:0] what, input integer got, input integer want);
        begin
            $display("%0s: %0d clocks", what, got);
            if (got != want) begin $display("FAIL %0s: measured %0d, documented %0d", what, got, want); errors = errors + 1; end
        end
    endtask
    initial begin
        repeat (4) @(negedge clk); rst = 0;
        for (v = 0; v < 16; v = v + 1) begin
            cfg(v * 16 + 1, 32'hC000020A); cfg(v * 16 + 2, 50000 + v);
            cfg(v * 16 + 4, v); cfg(v * 16 + 0, 32'h00030101);
        end
        // uniform pace: the smallest that takes flow 15
        m = 0; took = 0; while (!took && m < 200) begin m = m + 1; try(15, m, 0, 0, 0, took); end
        check("uniform, flow 15", m, EXP_UNIFORM);
        m = 0; took = 0; while (!took && m < 200) begin m = m + 1; try(11, m, 0, 0, 0, took); end
        check("uniform, flow 11", m, EXP_UNIFORM_F11);
        // 1 clock an octet except octet 37 -> 50 (search + loader) and octet 46 -> last (multiply)
        m = 11; took = 0; while (!took && m < 200) begin m = m + 1; try(0, 1, m, 0, 60, took); end
        check("octet 37 -> 50, flow 0", m, EXP_H37);
        m = 11; took = 0; while (!took && m < 200) begin m = m + 1; try(15, 1, m, 0, 60, took); end
        check("octet 37 -> 50, flow 15", m, EXP_H37 + 15);
        m = 9; took = 0; while (!took && m < 200) begin m = m + 1; try(15, 1, 0, 60, m, took); end
        check("octet 46 -> last", m, EXP_T46);
        // the same two limits as issue 009 put them: octet 38 after 37, then 1 clock an octet (flow 15)
        m = 0; took = 0; while (!took && m < 200) begin m = m + 1; try(15, 1, 0, m, 60, took); end
        $display("(octet 37 -> 38 alone, flow 15: %0d clocks)", m);
        m = 0; took = 0; while (!took && m < 200) begin m = m + 1; try(12, m, 0, 0, 0, took); end
        check("uniform, flow 12", m, EXP_UNIFORM);
        // the commit: samples one a clock, tags two clocks a frame
        cfg(0, 32'h00031001);
        m = 190; try_commit(16, 30, m, took); while (!took && m < 400) begin m = m + 1; try_commit(16, 30, m, took); end
        check("commit 16 x 30 (480 samples)", dist, 480 + EXP_COMMIT);
        if (m == 190) begin $display("FAIL commit sweep: its first gap passed"); errors = errors + 1; end
        cfg(0, 32'h00030101);
        m = 30; try_commit(1, 160, m, took); while (!took && m < 200) begin m = m + 1; try_commit(1, 160, m, took); end
        check("commit 1 x 160 (2 x 160 tag clocks)", dist, 320 + EXP_COMMIT);
        if (m == 30) begin $display("FAIL commit sweep: its first gap passed"); errors = errors + 1; end
        $display("errors %0d", errors);
        tx_go = 1; wait (tx_done);
        if (errors) $display("FAIL tb_inferno_pace: %0d errors", errors);
        else $display("PASS tb_inferno_pace: RX limits as documented; inferno_tx and cpu_pktq exact at 2 clocks an octet, wrong at 1");
        $finish;
    end
    // ================= TX: three copies of each engine, the MAC taking an octet every 4 (reference), 2 and 1 clocks
    reg        twe = 0, in_we = 0, fstb = 0, q_we = 0;
    reg [7:0]  ta = 0, q_a = 0;
    reg [31:0] td = 0, q_d = 0, mts = 0, msec = 32'd1000;
    reg [16:0] msub = 0;
    reg [5:0]  in_ch = 0;
    reg [23:0] in_sample = 0;
    genvar gk;
    generate for (gk = 0; gk < 3; gk = gk + 1) begin : e
        localparam K = gk == 0 ? 4 : gk == 1 ? 2 : 1;
        integer ph = 0, tn = 0, qn = 0;
        always @(posedge clk) ph <= (ph + 1) % K;
        wire rdy = ph == 0;
        reg  [7:0] tbuf [0:4095];
        reg  [7:0] qbuf [0:255];
        wire [7:0] t_d, q_dat;
        wire t_v, q_v;
        inferno_tx #(.MAX_CH(64), .NFLOW(16), .MAXFCH(32), .MAX_FPP(32), .NR(128)) tx (
            .clk(clk), .rst(rst), .enable(1'b1), .src_mac(48'h020000000101), .src_ip(32'hC000020A), .fpp_auto(6'd8),
            .ts_adv(8'd0), .tbl_we(twe), .tbl_waddr(ta), .tbl_wdata(td), .tbl_raddr(ta), .tbl_rdata(),
            .in_we(in_we), .in_ch(in_ch), .in_sample(in_sample), .frame_strobe(fstb), .media_ts(mts),
            .media_sec(msec), .media_sub(msub), .media_fs(17'd48000),
            .m_tdata(t_d), .m_tvalid(t_v), .m_tlast(), .m_tready(rdy),
            .pending(), .running(), .packets_sent(), .overruns());
        cpu_pktq #(.TXW(512), .RXW(2048), .ND(16)) q (
            .clk(clk), .rst(rst), .reg_addr(q_a), .reg_raddr(q_a), .reg_we(q_we), .reg_wdata(q_d), .reg_re(1'b0),
            .rd_hold(1'b0), .reg_rdata(), .our_mac(48'h020000000101), .tod_sec(32'd0), .tod_ns(30'd0),
            .m_tdata(q_dat), .m_tvalid(q_v), .m_tlast(), .m_tready(rdy), .tx_mine(1'b1), .sfd_tx(1'b0),
            .s_tdata(8'd0), .s_tvalid(1'b0), .s_tlast(1'b0), .s_tuser(1'b0), .sfd_rx(1'b0), .claim(1'b0));
        always @(posedge clk) begin
            if (t_v && rdy && tn < 4096) begin tbuf[tn] = t_d; tn = tn + 1; end
            if (q_v && rdy && qn < 256) begin qbuf[qn] = q_dat; qn = qn + 1; end
        end
    end endgenerate
    task twr(input [7:0] a, input [31:0] d); begin @(posedge clk); twe <= 1; ta <= a; td <= d; @(posedge clk); twe <= 0; end endtask
    task qw(input [7:0] a, input [31:0] d); begin @(posedge clk); q_we <= 1; q_a <= a; q_d <= d; @(posedge clk); q_we <= 0; end endtask
    function integer differ(input integer k, input integer pk);   // octets of copy k unlike the reference
        integer i, n;
        begin
            differ = 0;
            n = pk ? (e[0].qn < (k == 1 ? e[1].qn : e[2].qn) ? e[0].qn : (k == 1 ? e[1].qn : e[2].qn))
                   : (e[0].tn < (k == 1 ? e[1].tn : e[2].tn) ? e[0].tn : (k == 1 ? e[1].tn : e[2].tn));
            for (i = 0; i < n; i = i + 1)
                if (pk ? (e[0].qbuf[i] !== (k == 1 ? e[1].qbuf[i] : e[2].qbuf[i]))
                       : (e[0].tbuf[i] !== (k == 1 ? e[1].tbuf[i] : e[2].tbuf[i]))) differ = differ + 1;
        end
    endfunction
    reg tx_go = 0, tx_done = 0;
    integer f, c, d2, d1, q2, q1;
    initial begin
        wait (tx_go);
        // one flow, 8 channels x 16 frames, 24 bit; a 64-octet CPU frame
        twr(1, 32'h00000200); twr(2, 32'h00000202); twr(3, 32'hC0000214); twr(4, {16'd14400, 16'd14336});
        twr(5, 32'h03020100); twr(6, 32'h07060504); twr(0, {8'd64, 8'hB8, 3'd0, 5'd7, 6'd16, 1'b0, 1'b1});
        qw(8'h04, 32'h1);
        for (c = 0; c < 16; c = c + 1) qw(8'h0C, {c[7:0] * 8'd4, c[7:0] * 8'd4 + 8'd1, c[7:0] * 8'd4 + 8'd2, c[7:0] * 8'd4 + 8'd3} ^ 32'h5A3C0F96);
        qw(8'h10, 32'd64);
        for (f = 0; f < 50; f = f + 1) begin
            for (c = 0; c < 8; c = c + 1) begin
                @(posedge clk); in_we <= 1; in_ch <= c; in_sample <= (f * 32'h010203) ^ (c << 16) ^ 32'h5A5A5A;
            end
            @(posedge clk); in_we <= 0; fstb <= 1; mts <= f; msub <= f;
            @(posedge clk); fstb <= 0;
            repeat (1042 - 10) @(posedge clk);
        end
        d2 = differ(1, 0); d1 = differ(2, 0); q2 = differ(1, 1); q1 = differ(2, 1);
        $display("inferno_tx: %0d octets; unlike the reference: %0d at 2 clocks an octet, %0d at 1", e[0].tn, d2, d1);
        $display("cpu_pktq: %0d octets; unlike the reference: %0d at 2 clocks an octet, %0d at 1", e[0].qn, q2, q1);
        if (e[0].tn < 2 * 435 || e[1].tn != e[0].tn || e[2].tn != e[0].tn || e[0].qn != 64 || e[1].qn != 64 || e[2].qn != 64) begin
            $display("FAIL TX octet counts %0d %0d %0d / %0d %0d %0d", e[0].tn, e[1].tn, e[2].tn, e[0].qn, e[1].qn, e[2].qn);
            errors = errors + 1;
        end
        if (d2 || q2 || !d1 || !q1) begin $display("FAIL TX pace: 2 clocks an octet must match, 1 must not"); errors = errors + 1; end
        tx_done = 1;
    end
    initial begin #200000000; $display("FAIL tb_inferno_pace: timeout"); $finish; end
endmodule
`default_nettype wire
