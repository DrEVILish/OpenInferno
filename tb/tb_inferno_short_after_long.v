// A short flow packet right after a long one, through rmii_rx at the 100 Mbit pace: the short packet's verdict waits
// for the long packet's commit instead of being counted bad. Then, fed faster than the wait allows (3 clocks an octet,
// no gap), the waiting packet is dropped as bad when the next frame reaches byte 28. Licence: GPL-3.0.
`timescale 1ns/1ps
`default_nettype none
module tb_inferno_short_after_long;
`include "eth_crc32.vh"
    reg clk = 0, rst = 1;
    always #10 clk = ~clk;
    reg we = 0, strobe = 0, crs = 0, direct = 0, d_valid = 0, d_last = 0;
    reg [1:0] rxd = 0;
    reg [8:0] addr = 0;
    reg [31:0] wd = 0, play = 100;
    reg [7:0] d_data = 0;
    wire [7:0] m_data;
    wire m_valid, m_last, m_user;
    rmii_rx mac (.clk(clk), .rst(rst), .crs_dv(crs), .rxd(rxd), .m_tdata(m_data), .m_tvalid(m_valid), .m_tlast(m_last),
        .m_tuser(m_user), .sfd_strobe(), .rx_ok(), .rx_crc_err(), .rx_runt());
    wire [31:0] ok, bad, late, early, gaps;
    wire out_valid;
    wire [5:0] out_ch;
    wire [23:0] out_sample;
    inferno_rx dut (.clk(clk), .rst(rst), .enable(1'b1), .n_out(7'd17),
        .media_fs(17'd48000), .media_ts(32'd130), .tbl_we(we), .tbl_waddr(addr), .tbl_wdata(wd),
        .tbl_raddr(addr), .tbl_rdata(), .s_tdata(direct ? d_data : m_data), .s_tvalid(direct ? d_valid : m_valid),
        .s_tlast(direct ? d_last : m_last), .s_tuser(direct ? 1'b0 : m_user),
        .claim(), .frame_strobe(strobe), .play_ts(play), .out_valid(out_valid), .out_ch(out_ch), .out_sample(out_sample),
        .out_frame_ok(), .packets_ok(ok), .packets_bad(bad), .late(late), .early(early), .underruns(), .ts_gaps(gaps));
    integer errors = 0, samples = 0, play_frame = 0, waited = 0;
    function [23:0] pattern(input integer flow, input integer frame_no, input integer channel);
        pattern = ((flow + 1) << 20) | (frame_no << 8) | channel;
    endfunction
    always @(posedge clk) begin
        if (!rst && dut.j_wait) waited = waited + 1;
        if (out_valid) begin
            samples = samples + 1;
            if (out_sample !== pattern(out_ch / 16, play_frame, out_ch % 16)) begin
                if (errors < 5) $display("FAIL frame %0d column %0d sample %06x", play_frame, out_ch, out_sample);
                errors = errors + 1;
            end
        end
    end
    task cfg(input [8:0] a, input [31:0] d);
        begin @(negedge clk); we = 1; addr = a; wd = d; @(negedge clk); we = 0; end
    endtask
    reg [7:0] pkt [0:1499];
    integer plen;
    // flow 0: 16 channels x 30 frames (480 samples, 1491 octets); flow 1: 1 channel x 3 frames (60 octets)
    task build(input integer flow, input integer sub);
        integer i, f, ch, at, nch, fpp;
        reg [23:0] smp;
        begin
            nch = flow ? 1 : 16; fpp = flow ? 3 : 30;
            plen = 51 + nch * fpp * 3;
            for (i = 0; i < 1500; i = i + 1) pkt[i] = 0;
            pkt[12] = 8'h08; pkt[14] = 8'h45;
            pkt[16] = (plen - 14) >> 8; pkt[17] = plen - 14;
            pkt[22] = 64; pkt[23] = 17;
            pkt[26] = 192; pkt[28] = 2; pkt[29] = 11 + flow;
            pkt[30] = 192; pkt[32] = 2; pkt[33] = 10;
            pkt[34] = 8'hC3; pkt[35] = 8'h60;
            pkt[36] = 8'hC3; pkt[37] = 8'h50 + flow;
            pkt[38] = (plen - 34) >> 8; pkt[39] = plen - 34;
            pkt[42] = 2; pkt[49] = sub >> 8; pkt[50] = sub;
            at = 51;
            for (f = 0; f < fpp; f = f + 1) for (ch = 0; ch < nch; ch = ch + 1) begin
                smp = pattern(flow, sub - 100 + f, ch);
                pkt[at] = smp[23:16]; pkt[at + 1] = smp[15:8]; pkt[at + 2] = smp[7:0]; at = at + 3;
            end
        end
    endtask
    task dibits(input [7:0] b);
        integer k;
        begin for (k = 0; k < 4; k = k + 1) begin @(negedge clk); crs = 1; rxd = b >> (2 * k); end end
    endtask
    // one frame on the RMII wire: preamble, SFD, frame, FCS, then the 12-octet gap
    task send_rmii(input integer flow, input integer sub);
        integer i;
        reg [31:0] c;
        begin
            build(flow, sub);
            for (i = 0; i < 7; i = i + 1) dibits(8'h55);
            dibits(8'hD5);
            c = 32'hFFFFFFFF;
            for (i = 0; i < plen; i = i + 1) begin dibits(pkt[i]); c = crc32_byte(c, pkt[i]); end
            c = ~c;
            for (i = 0; i < 4; i = i + 1) dibits(c >> (8 * i));
            @(negedge clk); crs = 0; rxd = 0; repeat (47) @(negedge clk);
        end
    endtask
    // the byte stream straight into inferno_rx, one octet every 3 clocks, no gap
    task send_fast(input integer flow, input integer sub);
        integer i;
        begin
            build(flow, sub);
            for (i = 0; i < plen; i = i + 1) begin
                @(negedge clk); d_valid = 1; d_data = pkt[i]; d_last = i == plen - 1;
                @(negedge clk); d_valid = 0; d_last = 0; @(negedge clk);
            end
        end
    endtask
    integer flow, w, f;
    reg [31:0] map;
    initial begin
        repeat (4) @(negedge clk); rst = 0;
        for (flow = 0; flow < 2; flow = flow + 1) begin
            for (w = 0; w < 16; w = w + 1) cfg(flow * 16 + w, 0);
            cfg(flow * 16 + 1, 32'hC000020A); cfg(flow * 16 + 2, 50000 + flow);
        end
        for (w = 0; w < 4; w = w + 1) cfg(4 + w, (w * 4) | ((w * 4 + 1) << 8) | ((w * 4 + 2) << 16) | ((w * 4 + 3) << 24));
        cfg(16 + 4, 16);
        cfg(0, 32'h00031001); cfg(16, 32'h00030101);
        // 100 Mbit: long, short (judged while the long commit runs), short, long
        send_rmii(0, 100); send_rmii(1, 100); send_rmii(1, 103); send_rmii(0, 130);
        repeat (600) @(negedge clk);
        if (ok !== 4 || bad !== 0 || early !== 0 || late !== 0 || gaps !== 0 || waited == 0) begin
            $display("FAIL rmii counters ok=%0d bad=%0d early=%0d late=%0d gaps=%0d waited=%0d", ok, bad, early, late, gaps, waited);
            errors = errors + 1;
        end
        for (f = 0; f < 6; f = f + 1) begin
            play_frame = f; play = 100 + f; repeat (3) @(negedge clk);
            strobe = 1; @(negedge clk); strobe = 0; repeat (40) @(negedge clk);
        end
        if (samples != 6 * 17) begin $display("FAIL %0d samples played", samples); errors = errors + 1; end
        // too fast for the wait: long, short (waits, then lost at the next frame's byte 28), short (waits, taken)
        direct = 1; repeat (10) @(negedge clk);
        send_fast(0, 200); send_fast(1, 200); send_fast(1, 203);
        repeat (600) @(negedge clk);
        if (ok !== 6 || bad !== 1) begin $display("FAIL fast counters ok=%0d bad=%0d", ok, bad); errors = errors + 1; end
        if (errors) $display("FAIL tb_inferno_short_after_long: %0d errors", errors);
        else $display("PASS tb_inferno_short_after_long: %0d samples, %0d clocks waited", samples, waited);
        $finish;
    end
    initial begin #2000000; $display("FAIL tb_inferno_short_after_long: timeout"); $finish; end
endmodule
`default_nettype wire
