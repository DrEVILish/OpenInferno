// rmii_rx.v - 100BASE-TX MAC receiver on an RMII PHY.
//
// Provenance: written for OpenProSeries (W16) from IEEE 802.3 clause 3 (preamble, SFD, FCS residue 0xDEBB20E3)
// and the RMII 1.2 specification (RXD[1:0] least-significant di-bit first; CRS_DV, which at the end of a frame
// may toggle at 25 MHz while the PHY's elastic buffer drains). No vendor material. Licence: GPL-3.0.
//
// Output (m_*): the frame from the destination MAC to the end of the payload, FCS stripped; m_tuser is set
// with m_tlast when the frame is bad (FCS, a frame that ends inside a byte). Frames shorter than 5 bytes after
// the SFD are dropped (rx_runt). A di-bit is data while CRS_DV is high in its clock or the next one, so the
// toggling tail is taken whole; two low clocks end the frame.
// sfd_strobe pulses when the SFD has been seen (PTP receive timestamp).
`timescale 1ns / 1ps
`default_nettype none
module rmii_rx (
    input  wire       clk,
    input  wire       rst,
    input  wire       crs_dv,
    input  wire [1:0] rxd,
    output reg  [7:0] m_tdata,
    output reg        m_tvalid,
    output reg        m_tlast,
    output reg        m_tuser,
    output reg        sfd_strobe,
    output reg        rx_ok,        // one pulse per good frame
    output reg        rx_crc_err,   // one pulse per bad frame
    output reg        rx_runt
);
`include "eth_crc32.vh"

    localparam S_IDLE = 2'd0, S_PRE = 2'd1, S_DATA = 2'd2;

    reg [1:0]  state;
    reg [1:0]  rxd_d;
    reg        dv_d;
    reg [1:0]  dib;
    reg [7:0]  acc;
    reg [39:0] sr;          // the last five bytes (newest in [7:0])
    reg [2:0]  have;        // bytes in sr (saturates at 5)
    reg [31:0] crc;

    wire dibit_valid = dv_d | crs_dv;      // the di-bit of the previous clock is data
    wire frame_end   = ~dv_d & ~crs_dv;

    always @(posedge clk) begin
        m_tvalid   <= 1'b0;
        m_tlast    <= 1'b0;
        m_tuser    <= 1'b0;
        sfd_strobe <= 1'b0;
        rx_ok      <= 1'b0;
        rx_crc_err <= 1'b0;
        rx_runt    <= 1'b0;
        rxd_d <= rxd;
        dv_d  <= crs_dv;
        if (rst) begin
            state <= S_IDLE; dv_d <= 1'b0;
        end else begin
            case (state)
                S_IDLE: if (dv_d && rxd_d == 2'b01) state <= S_PRE;
                S_PRE: begin
                    if (!dibit_valid) state <= S_IDLE;
                    else if (rxd_d == 2'b11) begin
                        state <= S_DATA; dib <= 2'd0; have <= 3'd0; crc <= 32'hFFFFFFFF; sfd_strobe <= 1'b1;
                    end else if (rxd_d != 2'b01) state <= S_IDLE;   // not a preamble
                end
                S_DATA: begin
                    if (frame_end) begin
                        state <= S_IDLE;
                        if (have == 3'd5) begin
                            // sr[39:32] is the last payload byte; sr[31:0] the FCS
                            m_tdata  <= sr[39:32];
                            m_tvalid <= 1'b1;
                            m_tlast  <= 1'b1;
                            m_tuser  <= (crc != 32'hDEBB20E3) || (dib != 2'd0);
                            rx_ok      <= (crc == 32'hDEBB20E3) && (dib == 2'd0);
                            rx_crc_err <= (crc != 32'hDEBB20E3) || (dib != 2'd0);
                        end else rx_runt <= 1'b1;
                    end else if (dibit_valid) begin
                        acc <= {rxd_d, acc[7:2]};
                        dib <= dib + 2'd1;
                        if (dib == 2'd3) begin
                            // a whole byte: the oldest of five leaves (it is payload, not FCS)
                            if (have == 3'd5) begin
                                m_tdata <= sr[39:32]; m_tvalid <= 1'b1;
                            end else have <= have + 3'd1;
                            sr  <= {sr[31:0], rxd_d, acc[7:2]};
                            crc <= crc32_byte(crc, {rxd_d, acc[7:2]});
                        end
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
`default_nettype wire
