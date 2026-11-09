// Back-to-back packets overlap staging writes and the preceding sample commit. Licence: GPL-3.0.
`timescale 1ns/1ps
`default_nettype none
module tb_inferno_staging;
    reg clk = 0, rst = 1;
    always #10 clk = ~clk;
    reg we = 0, valid = 0, last = 0, strobe = 0;
    reg [8:0] addr = 0;
    reg [31:0] wd = 0, play = 100;
    reg [7:0] data = 0;
    wire [31:0] ok, bad, late, early;
    wire out_valid;
    wire [5:0] out_ch;
    wire [23:0] out_sample;
    inferno_rx dut (.clk(clk), .rst(rst), .enable(1'b1), .n_out(7'd32),
        .media_fs(17'd48000), .media_ts(32'd130), .tbl_we(we), .tbl_waddr(addr), .tbl_wdata(wd),
        .tbl_raddr(addr), .tbl_rdata(), .s_tdata(data), .s_tvalid(valid), .s_tlast(last), .s_tuser(1'b0),
        .claim(), .frame_strobe(strobe), .play_ts(play), .out_valid(out_valid), .out_ch(out_ch), .out_sample(out_sample),
        .out_frame_ok(), .packets_ok(ok), .packets_bad(bad), .late(late), .early(early), .underruns(), .ts_gaps());
    integer errors = 0, samples = 0, overlap = 0, play_frame = 0;
    function [23:0] pattern(input integer flow, input integer frame_no, input integer channel);
        pattern = ((flow + 1) << 20) | (frame_no << 8) | channel;
    endfunction
    always @(posedge clk) begin
        if (!rst && dut.cbusy && valid && dut.idx >= 51 && dut.idx < dut.pay_end) begin
            overlap = overlap + 1;
            if (dut.nsmp >= dut.ck) begin errors = errors + 1; $display("FAIL staging writer overtook commit reader"); end
        end
        if (out_valid) begin
            samples = samples + 1;
            if (out_sample !== pattern(out_ch / 16, play_frame, out_ch % 16)) begin
                if (errors < 5) $display("FAIL frame %0d column %0d sample %06x",play_frame,out_ch,out_sample);
                errors = errors + 1;
            end
        end
    end
    task cfg(input [8:0] a, input [31:0] d);
        begin @(negedge clk); we = 1; addr = a; wd = d; @(negedge clk); we = 0; end
    endtask
    reg [7:0] packet [0:1490];
    task send_packet(input integer flow);
        integer i, f, ch, at;
        reg [23:0] smp;
        reg [31:0] sum;
        begin
            for (i = 0; i < 1491; i = i + 1) packet[i] = 0;
            packet[12] = 8'h08; packet[14] = 8'h45;
            packet[16] = 8'h05; packet[17] = 8'hC5; // IP length 1477 = 20 + 8 + 9 + 30 * 16 * 3
            packet[22] = 64; packet[23] = 17;
            packet[26] = 192; packet[28] = 2; packet[29] = 11;
            packet[30] = 192; packet[32] = 2; packet[33] = 10;
            packet[34] = 8'hC3; packet[35] = 8'h60;
            packet[36] = 8'hC3; packet[37] = 8'h50 + flow;
            packet[38] = 8'h05; packet[39] = 8'hB1; // UDP length 1457
            packet[42] = 2; packet[50] = 100;
            at = 51;
            for (f = 0; f < 30; f = f + 1) for (ch = 0; ch < 16; ch = ch + 1) begin
                smp = pattern(flow,f,ch);
                packet[at] = smp[23:16]; packet[at + 1] = smp[15:8]; packet[at + 2] = smp[7:0]; at = at + 3;
            end
            sum = 0;
            for (i = 14; i < 34; i = i + 2) sum = sum + {packet[i],packet[i + 1]};
            sum = (sum & 65535) + (sum >> 16); sum = ~sum;
            packet[24] = sum >> 8; packet[25] = sum;
            for (i = 0; i < 1491; i = i + 1) begin
                @(negedge clk); data = packet[i]; valid = 1; last = i == 1490;
                @(negedge clk); valid = 0; last = 0; repeat (2) @(negedge clk);
            end
            repeat (48) @(negedge clk); // RMII minimum interpacket gap
        end
    endtask
    integer flow, w, f;
    reg [31:0] map;
    initial begin
        repeat (4) @(negedge clk); rst = 0;
        for (flow = 0; flow < 2; flow = flow + 1) begin
            for (w = 0; w < 16; w = w + 1) cfg(flow * 16 + w,0);
            cfg(flow * 16 + 1,32'hC000020A); cfg(flow * 16 + 2,50000 + flow);
            for (w = 0; w < 4; w = w + 1) begin
                map = (flow * 16 + w * 4) | ((flow * 16 + w * 4 + 1) << 8)
                    | ((flow * 16 + w * 4 + 2) << 16) | ((flow * 16 + w * 4 + 3) << 24);
                cfg(flow * 16 + 4 + w,map);
            end
            cfg(flow * 16,32'h00031001);
        end
        send_packet(0); send_packet(1); repeat (500) @(negedge clk);
        if (ok !== 2 || bad !== 0 || early !== 0 || late !== 0 || overlap == 0) begin
            $display("FAIL packet counters ok=%0d bad=%0d early=%0d late=%0d overlap=%0d",ok,bad,early,late,overlap); errors = errors + 1;
        end
        for (f = 0; f < 30; f = f + 1) begin
            play_frame = f; play = 100 + f; repeat (3) @(negedge clk);
            strobe = 1; @(negedge clk); strobe = 0; repeat (100) @(negedge clk);
        end
        if (samples != 960) errors = errors + 1;
        if (errors) $display("FAIL tb_inferno_staging: %0d errors",errors);
        else $display("PASS tb_inferno_staging: %0d samples, %0d overlapping byte cycles",samples,overlap);
        $finish;
    end
    initial begin #1000000; $display("FAIL tb_inferno_staging: timeout"); $finish; end
endmodule
`default_nettype wire
