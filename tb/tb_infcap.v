// tb_infcap.v - Inferno capacity and latency modes: CH channels (48 or 64) in flows of CPF channels, the latency
// mode LAT (0: 0.5 ms, 1: 1 ms, 2: 2 ms, 3: 5 ms) choosing the frames per packet (automatic, cut to the MTU) and the
// playout offset; inferno_tx -> RMII MAC -> wire -> RMII MAC -> inferno_rx, every column of every played frame
// checked bit-exact at exactly the mode's offset; packet sizes, link load and arrival latency measured; with GOLDEN
// the first packets compared byte for byte with golden.py's (cross-checked against the upstream-Inferno model,
// `make xcheck`).
// MAXCH sets both ends' MAX_CH (default 64; CH <= MAXCH, <= MAXCH / 2 at 96 kHz); inferno_regs' CAPS, JB_GEOM and
// CTRL reset columns are checked against it.
//   iverilog -DCH=64 -DCPF=16 -DLAT=0 [-DFS96] [-DMAXCH=64] [-DGOLDEN=\"golden/inferno_64ch.hex\"] tb/tb_infcap.v ...
// Provenance: written for OpenProSeries (W16). Licence: GPL-3.0.
`timescale 1ns/1ps
`default_nettype none
`ifndef CH
`define CH 64
`endif
`ifndef CPF
`define CPF 16
`endif
`ifndef LAT
`define LAT 0
`endif
`ifndef MAXCH
`define MAXCH 64
`endif
module tb_infcap;
`include "inferno_lat.vh"
    localparam CH = `CH, CPF = `CPF, LAT = `LAT, MAXCH = `MAXCH;
`ifdef FS96
    localparam HI = 1, FS = 96000, PERIOD = 521;
`else
    localparam HI = 0, FS = 48000, PERIOD = 1042;
`endif
    localparam NFL = (CH + CPF - 1) / CPF;
    localparam NCHECK = 48;                              // frames of audio checked after the buffer filled
    localparam [31:0] S0 = 32'd1760000000;
    localparam [16:0] U0 = FS - 10;                      // the second rolls over 10 frames in
    localparam [63:0] T064 = 64'd1760000000 * FS + FS - 10;
    localparam [31:0] T0 = T064[31:0];
    localparam [9:0]  OFFSET = inferno_lat_offset(LAT[1:0], HI[0]);
    localparam [5:0]  FPP_AUTO = inferno_lat_fpp(LAT[1:0], HI[0]);
    localparam [5:0]  FPP = (FPP_AUTO < inferno_mtu_fpp(CPF * 3)) ? FPP_AUTO : inferno_mtu_fpp(CPF * 3);
    localparam integer START = FPP - (T064 % FPP);      // the first packet's first frame, relative to T0
    localparam integer OFF = OFFSET;
    localparam integer FRAMES = OFFSET + START + NCHECK + FPP + 4;

    reg clk = 0, rst = 1;
    always #10 clk = ~clk;

    function [23:0] pattern(input integer frame, input integer ch);
        pattern = ((frame * 32'h010203) ^ (ch << 16) ^ 32'h5A5A5A) & 32'hFFFFFF;
    endfunction

`ifdef GOLDEN
    reg [7:0] golden [0:65535];
    initial $readmemh(`GOLDEN, golden);
`endif

    // ---- the media clock
    reg        frame_strobe = 0;
    reg [31:0] media_ts = T0 - 1, media_sec = S0;
    reg [16:0] media_sub = U0 - 1;

    // ---- TX
    reg        in_we = 0;
    reg [5:0]  in_ch = 0;
    reg [23:0] in_sample = 0;
    reg        ttbl_we = 0;
    reg [7:0]  ttbl_a = 0;
    reg [31:0] ttbl_d = 0;
    wire [31:0] ttbl_q;
    wire [7:0] t_data; wire t_valid, t_last, t_ready;
    wire [15:0] tx_pend, tx_run;
    wire [31:0] tx_packets, tx_overruns;
    inferno_tx #(.MAX_CH(MAXCH), .NFLOW(16), .MAXFCH(32), .MAX_FPP(32), .NR(128)) dut_tx (
        .clk(clk), .rst(rst), .enable(1'b1), .src_mac(48'h020000000101), .src_ip(32'hC000020A), .fpp_auto(FPP_AUTO),
        .ts_adv(8'd0), .tbl_we(ttbl_we), .tbl_waddr(ttbl_a), .tbl_wdata(ttbl_d), .tbl_raddr(ttbl_a), .tbl_rdata(ttbl_q),
        .in_we(in_we), .in_ch(in_ch), .in_sample(in_sample), .frame_strobe(frame_strobe), .media_ts(media_ts),
        .media_sec(media_sec), .media_sub(media_sub), .media_fs(FS[16:0]),
        .m_tdata(t_data), .m_tvalid(t_valid), .m_tlast(t_last), .m_tready(t_ready),
        .pending(tx_pend), .running(tx_run), .packets_sent(tx_packets), .overruns(tx_overruns));
    wire [1:0] txd; wire tx_en, sfd_tx, und, done;
    rmii_tx dut_mtx (.clk(clk), .rst(rst), .s_tdata(t_data), .s_tvalid(t_valid), .s_tlast(t_last), .s_tready(t_ready),
        .txd(txd), .tx_en(tx_en), .sfd_strobe(sfd_tx), .tx_underrun(und), .tx_done(done));

    // ---- RX
    wire [7:0] r_data; wire r_valid, r_last, r_user, sfd_rx, rok, rcrc, rrunt;
    rmii_rx dut_mrx (.clk(clk), .rst(rst), .crs_dv(tx_en), .rxd(txd), .m_tdata(r_data), .m_tvalid(r_valid), .m_tlast(r_last),
        .m_tuser(r_user), .sfd_strobe(sfd_rx), .rx_ok(rok), .rx_crc_err(rcrc), .rx_runt(rrunt));
    reg        rtbl_we = 0;
    reg [8:0]  rtbl_a = 0;
    reg [31:0] rtbl_d = 0;
    wire [31:0] rtbl_q;
    wire        claim, out_valid, out_frame_ok;
    wire [5:0]  out_ch;
    wire [23:0] out_sample;
    wire [31:0] p_ok, p_bad, late, early, underruns, ts_gaps;
    inferno_rx #(.MAX_CH(MAXCH), .NFLOW(16), .MAXFCH(32), .MAX_SMP(512), .NF(256)) dut_rx (
        .clk(clk), .rst(rst), .enable(1'b1), .n_out(CH[6:0]), .media_fs(FS[16:0]), .media_ts(media_ts),
        .tbl_we(rtbl_we), .tbl_waddr(rtbl_a), .tbl_wdata(rtbl_d), .tbl_raddr(rtbl_a), .tbl_rdata(rtbl_q),
        .s_tdata(r_data), .s_tvalid(r_valid), .s_tlast(r_last), .s_tuser(r_user), .claim(claim),
        .frame_strobe(frame_strobe), .play_ts(media_ts - {22'd0, OFFSET}), .out_valid(out_valid), .out_ch(out_ch),
        .out_sample(out_sample), .out_frame_ok(out_frame_ok), .packets_ok(p_ok), .packets_bad(p_bad), .late(late),
        .early(early), .underruns(underruns), .ts_gaps(ts_gaps));

    // ---- the host view of the port (CAPS, JB_GEOM, CTRL) at this MAX_CH
    reg  [7:0]  reg_a = 8'h38;
    wire [31:0] reg_q;
    inferno_regs #(.MAX_CH(MAXCH), .MAXFCH(32), .NF(256)) dut_regs (
        .clk(clk), .rst(rst), .addr(8'd0), .raddr(reg_a), .we(1'b0), .wdata(32'd0), .re(1'b0), .rdata(reg_q),
        .tx_en(), .rx_en(), .n_out(), .media_fs(), .set_msec(), .set_msub(), .lat_mode(), .lat_manual(), .ts_adv(),
        .fs_now(FS[16:0]), .link_offset(32'd0), .play_offset(), .fpp_auto(), .tx_tbl_we(), .rx_tbl_we(), .tbl_addr(),
        .tbl_wdata(), .tx_tbl_rdata(32'd0), .rx_tbl_rdata(32'd0), .tx_pending(16'd0), .tx_running(16'd0),
        .media_sec(32'd0), .media_sub(17'd0), .tx_packets(32'd0), .tx_overruns(32'd0), .rx_ok(32'd0), .rx_bad(32'd0),
        .rx_late(32'd0), .rx_early(32'd0), .rx_underruns(32'd0), .rx_ts_gaps(32'd0));

    integer errors = 0;
    // ---- the wire: packet sizes, link load, golden compare
    reg [7:0] wbuf [0:2047];
    integer wlen = 0, gi = 0, gpk = 0, npk = 0, k, busy = 0, win = 0, win_pk = 0;
    reg [7:0] wb; reg [1:0] wdib = 0; reg prev_en = 0;
    reg measuring = 0;
    always @(posedge clk) begin
        prev_en <= tx_en;
        if (measuring) begin win = win + 1; if (tx_en) busy = busy + 1; end
        if (tx_en) begin
            wb = {txd, wb[7:2]};
            if (wdib == 2'd3) begin wbuf[wlen] = wb; wlen = wlen + 1; end
            wdib = wdib + 2'd1;
        end else begin
            wdib = 2'd0;
            if (prev_en) begin
                npk = npk + 1;
                if (measuring) win_pk = win_pk + 1;
                if ({wbuf[8 + 38], wbuf[8 + 39]} != 16'd17 + FPP * CPF * 3) begin
                    errors = errors + 1; $display("packet %0d: UDP length %0d, want %0d (fpp %0d)", npk, {wbuf[8 + 38], wbuf[8 + 39]}, 17 + FPP * CPF * 3, FPP);
                end
`ifdef GOLDEN
                if (gpk < 16) begin
                    for (k = 0; k < wlen; k = k + 1)
                        if (golden[gi + k] !== wbuf[k]) begin
                            if (errors < 10) $display("WIRE packet %0d byte %0d: got %02x want %02x", gpk, k, wbuf[k], golden[gi + k]);
                            errors = errors + 1;
                        end
                    gi = gi + wlen; gpk = gpk + 1;
                end
`endif
                wlen = 0;
            end
        end
    end

    // ---- playout check: every column of every played frame
    integer play_n = -1, p_rel, checked = 0, cols = 0;
    reg [23:0] want;
    always @(posedge clk) begin
        if (out_valid) begin
            if (out_ch == 0) begin
                play_n = play_n + 1;
                if (play_n - OFF >= START + 1 && cols != CH) begin errors = errors + 1; $display("frame %0d: %0d columns", play_n - 1, cols); end
                cols = 0;
            end
            cols = cols + 1;
            p_rel = play_n - OFF;                     // the frame's media time - T0
            want = (p_rel >= START) ? pattern(p_rel, out_ch) : 24'd0;
            if (out_sample !== want) begin
                if (errors < 10) $display("PLAY frame %0d col %0d: got %06x want %06x", p_rel, out_ch, out_sample, want);
                errors = errors + 1;
            end
            if (p_rel >= START && out_ch == CH - 1) checked = checked + 1;
        end
    end

    task twr(input [7:0] a, input [31:0] d); begin @(posedge clk); ttbl_we <= 1; ttbl_a <= a; ttbl_d <= d; @(posedge clk); ttbl_we <= 0; end endtask
    task rwr(input [8:0] a, input [31:0] d); begin @(posedge clk); rtbl_we <= 1; rtbl_a <= a; rtbl_d <= d; @(posedge clk); rtbl_we <= 0; end endtask

    integer f, c, fl, n, lat_max;
    reg [8*6-1:0] lat_s;
    initial lat_s = LAT == 0 ? "0.5 ms" : LAT == 1 ? "1 ms" : LAT == 2 ? "2 ms" : "5 ms";
    reg [31:0] mw;
    initial begin
        if ($test$plusargs("vcd")) begin $dumpfile("build/tb_infcap.vcd"); $dumpvars(0, tb_infcap); end
        repeat (5) @(posedge clk); rst <= 0;
        repeat (5) @(posedge clk);
        for (fl = 0; fl < NFL; fl = fl + 1) begin
            n = (CH - fl * CPF < CPF) ? CH - fl * CPF : CPF;
            // TX flow fl: 24 bit, automatic fpp, channels fl x CPF .. to 192.0.2.20:14336 + fl from 14400 + fl
            twr(fl * 16 + 1, 32'h0000_0200); twr(fl * 16 + 2, 32'h0000_0202); twr(fl * 16 + 3, 32'hC000_0214);
            twr(fl * 16 + 4, {16'd14400 + fl[15:0], 16'd14336 + fl[15:0]});
            for (c = 0; c < 32; c = c + 4) begin
                mw = 32'h80808080;
                for (k = 0; k < 4; k = k + 1) if (c + k < n) mw[8 * k +: 8] = fl * CPF + c + k;
                twr(fl * 16 + 5 + c / 4, mw);
                rwr(fl * 16 + 4 + c / 4, mw);
            end
            twr(fl * 16 + 0, {8'd64, 8'hB8, 3'd0, n[4:0] - 5'd1, 6'd0, 1'b0, 1'b1});
            // RX flow fl: the same channels onto the same columns
            rwr(fl * 16 + 1, 32'hC000_0214); rwr(fl * 16 + 2, 14336 + fl); rwr(fl * 16 + 3, 32'd0);
            rwr(fl * 16 + 0, {13'd0, 3'd3, 2'd0, n[5:0], 7'd0, 1'b1});
        end
        for (f = 0; f < FRAMES; f = f + 1) begin
            for (c = 0; c < CH; c = c + 1) begin
                @(posedge clk); in_we <= 1; in_ch <= c; in_sample <= pattern(f, c);
            end
            @(posedge clk); in_we <= 0; frame_strobe <= 1; media_ts <= T0 + f;
            if (media_sub == FS - 1) begin media_sub <= 17'd0; media_sec <= media_sec + 1; end else media_sub <= media_sub + 1;
            @(posedge clk); frame_strobe <= 0;
            if (f == START + 2 * FPP) measuring = 1;
            if (f == START + 2 * FPP + 64) measuring = 0;                  // 64 frames' worth of packets
            repeat (PERIOD - CH - 2) @(posedge clk);
        end
        repeat (200) @(posedge clk);
        // ---- counters and status
        lat_max = 0;
        for (fl = 0; fl < NFL; fl = fl + 1) begin
            rtbl_a <= 9'd256 + fl * 16 + 3; @(posedge clk); #1 if (rtbl_q > lat_max) lat_max = rtbl_q;
        end
        if (p_bad != 0 || late != 0 || early != 0 || ts_gaps != 0 || tx_overruns != 0) begin
            errors = errors + 1; $display("bad %0d late %0d early %0d gaps %0d overruns %0d", p_bad, late, early, ts_gaps, tx_overruns);
        end
        if (lat_max >= OFF) begin errors = errors + 1; $display("arrival latency %0d frames, offset %0d", lat_max, OFFSET); end
        reg_a <= 8'h38; #1 if (reg_q[7:0] != MAXCH) begin errors = errors + 1; $display("CAPS 0x%08x: MAX_CH %0d, want %0d", reg_q, reg_q[7:0], MAXCH); end
        reg_a <= 8'h34; #1 if (reg_q[23:16] != (HI ? MAXCH / 2 : MAXCH)) begin errors = errors + 1; $display("JB_GEOM 0x%08x: %0d columns", reg_q, reg_q[23:16]); end
        reg_a <= 8'h04; #1 if (reg_q[14:8] != MAXCH) begin errors = errors + 1; $display("CTRL 0x%08x: %0d columns played", reg_q, reg_q[14:8]); end
        if (checked < NCHECK) begin errors = errors + 1; $display("only %0d frames checked", checked); end
`ifdef GOLDEN
        if (gpk != 16) begin errors = errors + 1; $display("only %0d golden packets", gpk); end
`endif
        if (errors == 0)
            $display("PASS tb_infcap %0dch MAX_CH %0d %0d kHz %0s: %0d flows x %0d ch, fpp %0d, offset %0d frames; %0d frames x %0d columns bit-exact; %0d packets ok; link %0d.%0d Mb/s; arrival latency <= %0d frames%0s",
                     CH, MAXCH, FS / 1000, lat_s, NFL, CPF, FPP, OFFSET,
                     checked, CH, p_ok, (100 * busy + 100 * 48 * win_pk) / win, ((1000 * busy + 1000 * 48 * win_pk) / win) % 10, lat_max,
`ifdef GOLDEN
                     "; 16 golden packets byte-exact");
`else
                     "");
`endif
        else $display("FAIL tb_infcap %0dch lat %0d: %0d errors (ok %0d bad %0d late %0d early %0d checked %0d)", CH, LAT, errors, p_ok, p_bad, late, early, checked);
        $finish;
    end
    initial begin #400000000; $display("FAIL tb_infcap: timeout"); $finish; end
endmodule
`default_nettype wire
