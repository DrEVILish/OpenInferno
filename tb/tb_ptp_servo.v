// tb_ptp_servo.v - ptp_clock.v disciplined through its register interface (step, PTP_INCR / MEDIA_INC trims), as the
// AMU servo does it (inferno-portd clock.c): the oscillator runs 50 ppm fast and the time of day starts 3 ms off; a
// PI loop on (time of day - the reference time) every 100 us (the test's time constant is 1 ms, the daemon's ~2 s)
// must step once, hold the offset within 2 clocks and settle PTP_INCR at -50 ppm; MEDIA_INC follows with the same
// ratio, so the media clock keeps time of day x 48 kHz to within a frame.
// Provenance: written for OpenProSeries (W16). Licence: GPL-3.0.
`timescale 1ns/1fs
`default_nettype none
module tb_ptp_servo;
    reg clk = 0, rst = 1;
    real half = 10.0 / (1.0 + 50.0e-6);              // a 50 MHz oscillator 50 ppm fast
    always #(half) clk = ~clk;
    reg  [31:0] incr = 32'h14000000, minc = 32'h003EEA21;
    reg         set_tod = 0, set_media = 0;
    reg  [47:0] set_sec = 0; reg [29:0] set_ns = 0; reg [31:0] set_mts = 0, set_msec = 0; reg [16:0] set_msub = 0;
    wire [47:0] sec; wire [29:0] ns; wire [31:0] media_ts; wire frame_strobe; wire [77:0] ts_tx, ts_rx;
    wire [31:0] msec; wire [16:0] msub;
    ptp_clock dut (.clk(clk), .rst(rst), .incr(incr), .set_tod(set_tod), .set_sec(set_sec), .set_ns(set_ns), .sec(sec), .ns(ns),
        .media_inc(minc), .set_media(set_media), .set_media_ts(set_mts), .media_ts(media_ts), .frame_strobe(frame_strobe),
        .media_fs(17'd48000), .set_media_sec(set_msec), .set_media_sub(set_msub), .media_sec(msec), .media_sub(msub),
        .sfd_tx(1'b0), .sfd_rx(1'b0), .ts_tx(ts_tx), .ts_rx(ts_rx));
    // the reference (the leader's time): 1 000 s + the simulation time
    function real ref_ns; input dummy; begin ref_ns = $realtime; end endfunction
    real off, integ, ppb, worst, tod, r, dt, mfs;
    integer k, steps, errors;
    initial begin
        errors = 0; steps = 0; integ = 0; worst = 0; dt = 100.0e-6;
        repeat (3) @(posedge clk); rst <= 0;
        @(posedge clk); set_tod <= 1; set_sec <= 48'd0; set_ns <= 30'd3000000; @(posedge clk); set_tod <= 0;   // 3 ms off
        for (k = 0; k < 300; k = k + 1) begin
            #100000;
            @(posedge clk);
            tod = $itor(sec) * 1.0e9 + $itor(ns);
            off = tod - ref_ns(0);
            if (off > 1.0e6 || off < -1.0e6) begin
                // step: SET_SEC / SET_NS + CLOCK_CMD bit 0, then the media clock from the time of day
                r = ref_ns(0) + 20.0;
                set_sec <= $rtoi(r / 1.0e9); set_ns <= $rtoi(r - $floor(r / 1.0e9) * 1.0e9); set_tod <= 1;
                @(posedge clk); set_tod <= 0;
                set_mts <= $rtoi(r * 48.0e-6); set_msec <= 0; set_msub <= $rtoi(r * 48.0e-6); set_media <= 1;
                @(posedge clk); set_media <= 0;
                steps = steps + 1;
            end else begin
                integ = integ - 5.1e5 * off * dt;
                ppb = integ - 1000.0 * off;
                incr <= $rtoi(335544320.0 * (1.0 + ppb * 1.0e-9) + 0.5);
                minc <= $rtoi(335544320.0 * (1.0 + ppb * 1.0e-9) * 48000.0 * 256.0 / 1.0e9 + 0.5);
                if (k > 200 && (off > worst || -off > worst)) worst = (off > 0) ? off : -off;
            end
        end
        mfs = ($itor(media_ts) - ($itor(sec) * 1.0e9 + $itor(ns)) * 48.0e-6);
        if (steps != 1) begin errors = errors + 1; $display("%0d steps", steps); end
        if (worst > 40.0) begin errors = errors + 1; $display("worst offset %f ns", worst); end
        if (ppb > -49000.0 || ppb < -51000.0) begin errors = errors + 1; $display("ppb %f", ppb); end
        if (mfs > 1.5 || mfs < -1.5) begin errors = errors + 1; $display("media - tod x fs = %f frames", mfs); end
        if (errors == 0) $display("PASS tb_ptp_servo: one step, |offset| <= %0.1f ns over the last 10 ms, PTP_INCR %08x (%0.1f ppb), media %0.2f frames off tod x fs",
                                  worst, incr, ppb, mfs);
        else $display("FAIL tb_ptp_servo: %0d errors", errors);
        $finish;
    end
endmodule
`default_nettype wire
