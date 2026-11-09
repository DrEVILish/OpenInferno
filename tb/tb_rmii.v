// tb_rmii.v - the RMII MAC: transmit against golden wire bytes, loop back into the receiver (with the RMII
// CRS_DV toggling tail), check the delivered bytes, then a corrupted frame and a runt.
// Provenance: written for OpenProSeries (W16). Licence: GPL-3.0.
`timescale 1ns/1ps
`default_nettype none
module tb_rmii;
    reg clk = 0, rst = 1;
    always #10 clk = ~clk;                 // 50 MHz

    // golden files (tools/golden.py frames)
    reg [7:0] tx_in  [0:4095];
    reg [7:0] wire_g [0:8191];
    reg [7:0] rx_g   [0:8191];
    integer   lens [0:7];
    initial begin
        $readmemh("golden/mac_tx_in.hex", tx_in, 0, 2017);
        $readmemh("golden/mac_wire.hex", wire_g, 0, 2174);
        $readmemh("golden/mac_rx.hex", rx_g, 0, 2078);
        lens[0] = 14; lens[1] = 46; lens[2] = 59; lens[3] = 60; lens[4] = 61; lens[5] = 64; lens[6] = 200; lens[7] = 1514;
    end

    // ---- transmitter fed from tx_in
    reg  [7:0] s_tdata;
    reg        s_tvalid = 0, s_tlast = 0;
    wire       s_tready;
    wire [1:0] txd;
    wire       tx_en, sfd_tx, tx_underrun, tx_done;
    rmii_tx dut_tx (.clk(clk), .rst(rst), .s_tdata(s_tdata), .s_tvalid(s_tvalid), .s_tlast(s_tlast), .s_tready(s_tready),
        .txd(txd), .tx_en(tx_en), .sfd_strobe(sfd_tx), .tx_underrun(tx_underrun), .tx_done(tx_done));

    // ---- the wire: a delay line, and CRS_DV that toggles in the frame's last 4 di-bits (RMII 1.2 end of frame)
    localparam DL = 8;
    reg [1:0] dl_d [0:DL-1];
    reg       dl_v [0:DL-1];
    integer   i;
    reg       corrupt = 0;                 // flip one bit of the next frame on the wire
    integer   corrupt_at = -1, dibit_no = 0;
    always @(posedge clk) begin
        for (i = DL - 1; i > 0; i = i - 1) begin dl_d[i] <= dl_d[i-1]; dl_v[i] <= dl_v[i-1]; end
        dl_d[0] <= (corrupt && dibit_no == corrupt_at) ? txd ^ 2'b01 : txd;
        dl_v[0] <= tx_en;
        dibit_no <= tx_en ? dibit_no + 1 : 0;
    end
    wire       data_v = dl_v[DL-1];
    reg  [1:0] rxd;
    reg        crs_dv;
    reg        toggle_ph;
    always @(*) begin
        rxd = data_v ? dl_d[DL-1] : 2'b00;
        // the last 4 di-bits: CRS_DV low on the first di-bit of each nibble, high on the second
        if (data_v && !(dl_v[DL-5])) crs_dv = toggle_ph;
        else crs_dv = data_v;
    end
    always @(posedge clk) toggle_ph <= (data_v && !dl_v[DL-5]) ? ~toggle_ph : 1'b0;

    wire [7:0] m_tdata;
    wire       m_tvalid, m_tlast, m_tuser, sfd_rx, rx_ok, rx_crc_err, rx_runt;
    reg        rx_crs_dv_force = 0, rx_force_en = 0;
    reg  [1:0] rx_rxd_force = 0;
    rmii_rx dut_rx (.clk(clk), .rst(rst), .crs_dv(rx_force_en ? rx_crs_dv_force : crs_dv), .rxd(rx_force_en ? rx_rxd_force : rxd),
        .m_tdata(m_tdata), .m_tvalid(m_tvalid), .m_tlast(m_tlast), .m_tuser(m_tuser), .sfd_strobe(sfd_rx),
        .rx_ok(rx_ok), .rx_crc_err(rx_crc_err), .rx_runt(rx_runt));

    // ---- checkers
    integer errors = 0, wire_n = 0, rx_n = 0, frames_ok = 0, frames_bad = 0, runts = 0, sfds_tx = 0, sfds_rx = 0;
    reg [7:0] wb; reg [1:0] wdib; reg wact;
    // reassemble the wire bytes from TXD
    always @(posedge clk) begin
        if (tx_en) begin
            wb = {txd, wb[7:2]};
            if (wdib == 2'd3) begin
                if (!corrupt && wire_g[wire_n] !== wb) begin
                    if (errors < 10) $display("WIRE byte %0d: got %02x want %02x", wire_n, wb, wire_g[wire_n]);
                    errors = errors + 1;
                end
                if (!corrupt) wire_n = wire_n + 1;
            end
            wdib = wdib + 2'd1;
        end else wdib = 2'd0;
        if (sfd_tx) sfds_tx = sfds_tx + 1;
        if (sfd_rx) sfds_rx = sfds_rx + 1;
        if (m_tvalid && !corrupt) begin
            if (rx_g[rx_n] !== m_tdata) begin
                if (errors < 10) $display("RX byte %0d: got %02x want %02x", rx_n, m_tdata, rx_g[rx_n]);
                errors = errors + 1;
            end
            rx_n = rx_n + 1;
        end
        if (m_tvalid && m_tlast) begin
            if (m_tuser) frames_bad = frames_bad + 1; else frames_ok = frames_ok + 1;
        end
        if (rx_runt) runts = runts + 1;
    end

    // ---- stimulus
    integer f, b, base;
    task send_frame(input integer start, input integer n);
        integer k;
        begin
            k = 0;
            @(posedge clk);
            s_tdata <= tx_in[start]; s_tlast <= (n == 1); s_tvalid <= 1;
            while (k < n) begin
                @(posedge clk);
                if (s_tready) begin
                    k = k + 1;
                    if (k < n) begin s_tdata <= tx_in[start + k]; s_tlast <= (k == n - 1); end
                    else begin s_tvalid <= 0; s_tlast <= 0; end
                end
            end
            @(posedge tx_done);
        end
    endtask

    initial begin
        if ($test$plusargs("vcd")) begin $dumpfile("build/tb_rmii.vcd"); $dumpvars(0, tb_rmii); end
        wdib = 0; wb = 0;
        repeat (5) @(posedge clk); rst <= 0;
        repeat (5) @(posedge clk);
        base = 0;
        for (f = 0; f < 8; f = f + 1) begin
            send_frame(base, lens[f]);
            base = base + lens[f];
        end
        repeat (100) @(posedge clk);
        if (wire_n != 2175) begin $display("WIRE: %0d bytes compared, want 2175", wire_n); errors = errors + 1; end
        if (rx_n != 2079) begin $display("RX: %0d bytes delivered, want 2079", rx_n); errors = errors + 1; end
        if (frames_ok != 8 || frames_bad != 0) begin $display("RX frames ok %0d bad %0d, want 8 / 0", frames_ok, frames_bad); errors = errors + 1; end
        if (sfds_tx != 8 || sfds_rx != 8) begin $display("SFD strobes tx %0d rx %0d", sfds_tx, sfds_rx); errors = errors + 1; end
        // a frame with one bit flipped on the wire: delivered, flagged bad
        corrupt = 1; corrupt_at = 200;
        send_frame(0, 64);
        repeat (100) @(posedge clk);
        corrupt = 0;
        if (frames_bad != 1) begin $display("corrupted frame: bad count %0d, want 1", frames_bad); errors = errors + 1; end
        // a runt: preamble, SFD and two bytes
        rx_force_en = 1;
        repeat (15) begin @(posedge clk); rx_crs_dv_force <= 1; rx_rxd_force <= 2'b01; end
        @(posedge clk); rx_rxd_force <= 2'b11;
        repeat (8) begin @(posedge clk); rx_rxd_force <= 2'b10; end
        @(posedge clk); rx_crs_dv_force <= 0; rx_rxd_force <= 2'b00;
        repeat (10) @(posedge clk);
        rx_force_en = 0;
        if (runts != 1) begin $display("runt count %0d, want 1", runts); errors = errors + 1; end
        if (errors == 0) $display("PASS tb_rmii: 8 frames (14..1514 B) byte-exact on the wire and back, FCS error and runt detected");
        else $display("FAIL tb_rmii: %0d errors", errors);
        $finish;
    end
    initial begin #20000000; $display("FAIL tb_rmii: timeout"); $finish; end
endmodule
`default_nettype wire
