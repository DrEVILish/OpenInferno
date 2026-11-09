// Receive length boundaries at the MAC byte-stream interface. Licence: GPL-3.0.
`timescale 1ns/1ps
`default_nettype none
module tb_inferno_lengths;
    reg clk = 0, rst = 1;
    always #10 clk = ~clk;
    reg we = 0, valid = 0, last = 0, bad = 0;
    reg [8:0] addr = 0;
    reg [31:0] wd = 0;
    reg [7:0] data = 0;
    reg [31:0] stamp_sec = 0;
    wire [31:0] media_now = stamp_sec * 32'd48000 + 32'd100;
    wire [31:0] ok, rejected;
    inferno_rx dut (.clk(clk), .rst(rst), .enable(1'b1), .n_out(7'd1),
        .media_fs(17'd48000), .media_ts(media_now), .tbl_we(we), .tbl_waddr(addr), .tbl_wdata(wd),
        .tbl_raddr(addr), .tbl_rdata(), .s_tdata(data), .s_tvalid(valid), .s_tlast(last), .s_tuser(bad),
        .claim(), .frame_strobe(1'b0), .play_ts(media_now - 32'd24), .out_valid(), .out_ch(), .out_sample(),
        .out_frame_ok(), .packets_ok(ok), .packets_bad(rejected), .late(), .early(), .underruns(), .ts_gaps());
    reg [7:0] packet [0:127];
    integer errors = 0, checks = 0;
    task cfg(input [8:0] a, input [31:0] d);
        begin @(negedge clk); we = 1; addr = a; wd = d; @(negedge clk); we = 0; end
    endtask
    task run_case(input integer id, input integer wire_len, input integer ip_len,
                  input integer udp_len, input integer crc_bad, input integer want);
        integer i;
        reg [31:0] checksum;
        begin
            rst = 1; repeat (4) @(negedge clk); rst = 0;
            for (i = 0; i < 16; i = i + 1) cfg(i, 0);
            cfg(1, 32'hC000020A); cfg(2, 32'd50000); cfg(0, 32'h00030101);
            for (i = 0; i < 128; i = i + 1) packet[i] = 0;
            packet[12] = 8'h08; packet[14] = 8'h45;
            packet[16] = ip_len >> 8; packet[17] = ip_len;
            packet[22] = 64; packet[23] = 17;
            packet[26] = 192; packet[28] = 2; packet[29] = 11;
            packet[30] = 192; packet[32] = 2; packet[33] = 10;
            packet[34] = 8'hC3; packet[35] = 8'h51; packet[36] = 8'hC3; packet[37] = 8'h50;
            packet[38] = udp_len >> 8; packet[39] = udp_len;
            packet[42] = 2; packet[50] = 100;
            {packet[43], packet[44], packet[45], packet[46]} = stamp_sec;
            for (i = 51; i < 75; i = i + 1) packet[i] = i;
            checksum = 0;
            for (i = 14; i < 34; i = i + 2) checksum = checksum + {packet[i], packet[i + 1]};
            checksum = (checksum & 65535) + (checksum >> 16);
            checksum = (checksum & 65535) + (checksum >> 16);
            checksum = ~checksum; packet[24] = checksum >> 8; packet[25] = checksum;
            for (i = 0; i < wire_len; i = i + 1) begin
                @(negedge clk); valid = 1; data = packet[i]; last = i == wire_len - 1; bad = last && crc_bad;
                @(negedge clk); valid = 0; last = 0; bad = 0;
                repeat (2) @(negedge clk);
            end
            repeat (200) @(negedge clk);
            checks = checks + 1;
            if (ok !== want) begin
                $display("FAIL length case %0d: accepted %0d, want %0d", id, ok, want); errors = errors + 1;
            end
        end
    endtask
    integer stamp_case;
    initial begin
        run_case(0, 75, 61, 41, 0, 1); // complete packet
        run_case(1, 80, 61, 41, 0, 1); // Ethernet padding is outside the IP packet
        run_case(2, 72, 61, 41, 0, 0); // whole sample missing, no Ethernet/FCS error
        run_case(3, 75, 60, 41, 0, 0); // UDP extends past declared IPv4 payload
        run_case(4, 75, 64, 44, 0, 0); // declared IP and UDP payload longer than arrival
        run_case(5, 74, 61, 41, 0, 0); // truncated inside a sample
        run_case(6, 75, 61, 41, 1, 0); // bad FCS
        stamp_sec = 32'hFFFFFFFF; run_case(7,75,61,41,0,1);
        for (stamp_case = 0; stamp_case < 32; stamp_case = stamp_case + 1) begin
            stamp_sec = $random; run_case(8 + stamp_case,75,61,41,0,1);
        end
        if (errors) $display("FAIL tb_inferno_lengths: %0d of %0d cases failed", errors, checks);
        else $display("PASS tb_inferno_lengths: %0d cases", checks);
        $finish;
    end
    initial begin #4000000; $display("FAIL tb_inferno_lengths: timeout"); $finish; end
endmodule
`default_nettype wire
