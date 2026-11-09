// tb_ptp.v - ptp_clock: nanoseconds and the media clock advance at the programmed rates, a step, the second
// roll-over, a +100 ppm trim, the SFD stamps, the media clock's seconds / sub-second form across a second.
// Provenance: written for OpenProSeries (W16). Licence: GPL-3.0.
`timescale 1ns/1ps
`default_nettype none
module tb_ptp;
    reg clk = 0, rst = 1;
    always #10 clk = ~clk;
    reg  [31:0] incr = 32'h14000000, media_inc = 32'h003EEA21;
    reg         set_tod = 0, set_media = 0, sfd_tx = 0, sfd_rx = 0;
    reg  [47:0] set_sec = 0; reg [29:0] set_ns = 0; reg [31:0] set_media_ts = 0;
    wire [47:0] sec; wire [29:0] ns; wire [31:0] media_ts; wire frame_strobe; wire [77:0] ts_tx, ts_rx;
    reg  [31:0] set_msec = 0; reg [16:0] set_msub = 0;
    wire [31:0] msec; wire [16:0] msub;
    ptp_clock dut (.clk(clk), .rst(rst), .incr(incr), .set_tod(set_tod), .set_sec(set_sec), .set_ns(set_ns), .sec(sec), .ns(ns),
        .media_inc(media_inc), .set_media(set_media), .set_media_ts(set_media_ts), .media_ts(media_ts), .frame_strobe(frame_strobe),
        .sfd_tx(sfd_tx), .sfd_rx(sfd_rx), .ts_tx(ts_tx), .ts_rx(ts_rx), .media_fs(17'd48000), .set_media_sec(set_msec),
        .set_media_sub(set_msub), .media_sec(msec), .media_sub(msub));
    integer errors = 0, strobes = 0;
    reg [29:0] ns0; reg [31:0] m0;
    always @(posedge clk) if (frame_strobe) strobes = strobes + 1;
    initial begin
        repeat (3) @(posedge clk); rst <= 0;
        @(posedge clk); ns0 = ns; m0 = media_ts; strobes = 0;
        repeat (50000) @(posedge clk);                         // 1 ms
        if (ns - ns0 != 1000000) begin errors = errors + 1; $display("1 ms = %0d ns", ns - ns0); end
        if (media_ts - m0 != 48 || strobes != 48) begin errors = errors + 1; $display("1 ms = %0d samples, %0d strobes", media_ts - m0, strobes); end
        // step to just before a second boundary: it rolls over
        @(posedge clk); set_tod <= 1; set_sec <= 48'd1000; set_ns <= 30'd999999900;
        @(posedge clk); set_tod <= 0;
        repeat (10) @(posedge clk);
        if (sec != 48'd1001 || ns > 30'd200) begin errors = errors + 1; $display("roll-over: %0d s %0d ns", sec, ns); end
        // +100 ppm: 20.002 ns a clock
        incr <= 32'h14000000 + 32'h00008312;                   // 0.002 ns in 8.24: 0.002 x 2^24 = 0x8312
        @(posedge clk); ns0 = ns;
        repeat (50000) @(posedge clk);
        if (ns - ns0 < 1000090 || ns - ns0 > 1000110) begin errors = errors + 1; $display("+100 ppm: 1 ms = %0d ns", ns - ns0); end
        // SFD stamps
        @(posedge clk); sfd_tx <= 1; @(posedge clk); sfd_tx <= 0;
        if (ts_tx[77:30] != sec && ts_tx[77:30] != sec - 1) begin errors = errors + 1; $display("ts_tx"); end
        set_media_ts <= 32'hDEAD0000; set_media <= 1; @(posedge clk); set_media <= 0; @(posedge clk);
        if (media_ts != 32'hDEAD0000 && media_ts != 32'hDEAD0001) begin errors = errors + 1; $display("media load %08x", media_ts); end
        // seconds / sub-second: load 10 samples before a second, run 1 ms (48 samples): 38 into the next second
        @(posedge frame_strobe); @(posedge clk);
        set_media_ts <= 32'h8864BB76; set_msec <= 32'd1759999999; set_msub <= 17'd47990; set_media <= 1;
        @(posedge clk); set_media <= 0; m0 = 32'h8864BB76;
        repeat (50000) @(posedge clk);
        if (msec != 32'd1760000000 || msub != media_ts - m0 - 32'd10) begin
            errors = errors + 1; $display("media sec/sub %0d / %0d after %0d samples", msec, msub, media_ts - m0);
        end
        if (errors == 0) $display("PASS tb_ptp: 1 ms = 1 000 000 ns and 48 samples, step and roll-over, +100 ppm trim, stamps, media load, media seconds / samples across a second");
        else $display("FAIL tb_ptp: %0d errors", errors);
        $finish;
    end
endmodule
`default_nettype wire
