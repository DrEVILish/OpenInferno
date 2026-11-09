// tb_fs_phase.v - fs_phase.v against ptp_clock.v: a desk frame clock 100 ppm slow of the media clock (48 kHz, its
// own asynchronous clock); the snapshot's phase (DESK_FRAMES - MEDIA_TS - TICKS / PERIOD) must fall by 100 ppm x the
// media frames, the edge count and the desk period must match, CTRL clear and ID.
// Provenance: written for OpenProSeries (W16). Licence: GPL-3.0.
`timescale 1ns/1ps
`default_nettype none
module tb_fs_phase;
    reg clk = 0, rst = 1;
    always #10 clk = ~clk;
    // the desk frame clock: 1 / 48 kHz x (1 + 100e-6), 50 % duty
    real half = 1.0e9 / 48000.0 / 2.0 * (1.0 + 100.0e-6);
    reg desk = 0;
    initial begin #3.7; forever #(half) desk = ~desk; end
    wire [47:0] sec; wire [29:0] ns; wire [31:0] media_ts; wire frame_strobe; wire [77:0] ts_tx, ts_rx;
    wire [31:0] msec; wire [16:0] msub;
    ptp_clock u_clk (.clk(clk), .rst(rst), .incr(32'h14000000), .set_tod(1'b0), .set_sec(48'd0), .set_ns(30'd0), .sec(sec), .ns(ns),
        .media_inc(32'h003EEA21), .set_media(1'b0), .set_media_ts(32'd0), .media_ts(media_ts), .frame_strobe(frame_strobe),
        .media_fs(17'd48000), .set_media_sec(32'd0), .set_media_sub(17'd0), .media_sec(msec), .media_sub(msub),
        .sfd_tx(1'b0), .sfd_rx(1'b0), .ts_tx(ts_tx), .ts_rx(ts_rx));
    reg  [7:0]  a = 0;
    reg         we = 0;
    reg  [31:0] wd = 0;
    wire [31:0] rd;
    fs_phase dut (.clk(clk), .rst(rst), .frame_strobe(frame_strobe), .media_ts(media_ts), .desk_fs(desk), .reg_addr(a), .bus_addr(a),
        .reg_we(we), .reg_wdata(wd), .reg_re(1'b0), .reg_rdata(rd));
    integer errors = 0;
    task wr(input [7:0] ad, input [31:0] d); begin @(posedge clk); a <= ad; wd <= d; we <= 1; @(posedge clk); we <= 0; end endtask
    task rdr(input [7:0] ad, output [31:0] d); begin @(posedge clk); a <= ad; @(posedge clk); #1 d = rd; end endtask
    reg [31:0] f0, t0, k0, p0, f1, t1, k1, p1, dp, id;
    real ph0, ph1, dmed, want;
    initial begin
        repeat (4) @(posedge clk); rst <= 0;
        rdr(8'h00, id);
        if (id != 32'h4F465031) begin errors = errors + 1; $display("ID %08x", id); end
        wr(8'h04, 32'h3);                                  // enable, clear
        #2_000_000;                                        // 2 ms
        wr(8'h08, 0);
        rdr(8'h0C, f0); rdr(8'h10, t0); rdr(8'h14, k0); rdr(8'h18, p0);
        #20_000_000;                                       // 20 ms more
        wr(8'h08, 0);
        rdr(8'h0C, f1); rdr(8'h10, t1); rdr(8'h14, k1); rdr(8'h18, p1); rdr(8'h1C, dp);
        ph0 = $itor($signed(f0 - t0)) - $itor(k0) / $itor(p0);
        ph1 = $itor($signed(f1 - t1)) - $itor(k1) / $itor(p1);
        dmed = $itor(t1 - t0) + $itor(k1) / $itor(p1) - $itor(k0) / $itor(p0);   // media frames between the snapshots
        want = -dmed * 100.0e-6 / (1.0 + 100.0e-6);
        if ((ph1 - ph0) - want > 0.003 || (ph1 - ph0) - want < -0.003) begin
            errors = errors + 1; $display("phase change %f, want %f", ph1 - ph0, want);
        end
        if (f1 - f0 < 958 || f1 - f0 > 961) begin errors = errors + 1; $display("desk edges %0d in 20 ms", f1 - f0); end
        if (dp < 1041 || dp > 1043) begin errors = errors + 1; $display("desk period %0d clocks", dp); end
        if (p1 < 1041 || p1 > 1042) begin errors = errors + 1; $display("media period %0d clocks", p1); end
        wr(8'h04, 32'h3); wr(8'h08, 0); rdr(8'h0C, f0);
        if (f0 > 1) begin errors = errors + 1; $display("clear: %0d", f0); end
        if (errors == 0) $display("PASS tb_fs_phase: a -100 ppm desk clock reads %0.4f frames over %0.1f media frames (want %0.4f), 1 042-clock periods, clear",
                                  ph1 - ph0, dmed, want);
        else $display("FAIL tb_fs_phase: %0d errors", errors);
        $finish;
    end
endmodule
`default_nettype wire
