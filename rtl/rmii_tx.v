// rmii_tx.v - 100BASE-TX MAC transmitter on an RMII PHY (one di-bit per 50 MHz REF_CLK).
//
// Provenance: written for OpenProSeries (W16) from IEEE 802.3 clause 3 / 4 (frame format, preamble, SFD,
// minimum frame 64 bytes incl. FCS, inter-packet gap 96 bit times) and the RMII 1.2 specification (TXD[1:0]
// least-significant di-bit first, TX_EN, 50 MHz reference). No vendor material. Licence: GPL-3.0.
//
// Byte stream in (s_*): the frame from the destination MAC up to the end of the payload; the MAC adds the
// preamble, the SFD, padding to 60 bytes and the FCS, and keeps 12 byte times of gap after it. s_tready is a
// one-clock pulse when a byte is taken (at most every 4 clocks). The source must not run dry inside a frame:
// if s_tvalid is low when a byte is due, the frame is ended with a deliberately wrong FCS (the receiver drops
// it) and tx_underrun pulses.
// sfd_strobe pulses in the clock the SFD's last di-bit is on the wire (PTP transmit timestamp, IEEE 1588).
`timescale 1ns / 1ps
`default_nettype none
module rmii_tx (
    input  wire       clk,          // RMII REF_CLK, 50 MHz
    input  wire       rst,
    input  wire [7:0] s_tdata,
    input  wire       s_tvalid,
    input  wire       s_tlast,
    output reg        s_tready,
    output reg  [1:0] txd,
    output reg        tx_en,
    output reg        sfd_strobe,
    output reg        tx_underrun,
    output reg        tx_done       // one pulse per frame sent
);
`include "eth_crc32.vh"

    localparam S_IDLE = 3'd0, S_PRE = 3'd1, S_DATA = 3'd2, S_PAD = 3'd3, S_FCS = 3'd4, S_IFG = 3'd5;

    reg [2:0]  state;
    reg [1:0]  dib;        // di-bit of the current byte (0 = bits 1:0)
    reg [7:0]  sh;         // the byte on the wire
    reg [3:0]  cnt;        // preamble / FCS / gap byte counter
    reg [10:0] len;        // data + pad bytes sent
    reg [31:0] crc;
    reg        last_in;    // the byte on the wire was the source's last
    reg        bad;        // underrun: corrupt the FCS
    reg        crc_upd;    // the byte loaded last clock goes into the CRC now (from a register: no RAM -> CRC path)
    wire [31:0] fcs = bad ? crc : ~crc;   // (an un-inverted register is a wrong FCS)

    always @(posedge clk) begin
        s_tready    <= 1'b0;
        sfd_strobe  <= 1'b0;
        tx_underrun <= 1'b0;
        tx_done     <= 1'b0;
        if (rst) begin
            state <= S_IDLE; tx_en <= 1'b0; txd <= 2'b00; dib <= 2'd0; cnt <= 4'd0; crc_upd <= 1'b0;
        end else begin
            // the wire: one di-bit per clock of the byte in sh
            case (state)
                S_IDLE: begin
                    tx_en <= 1'b0; txd <= 2'b00;
                    if (s_tvalid) begin
                        state <= S_PRE; cnt <= 4'd0; dib <= 2'd0; sh <= 8'h55;
                    end
                end
                default: begin
                    if (crc_upd) begin crc <= crc32_byte(crc, sh); crc_upd <= 1'b0; end
                    if (state == S_IFG) begin tx_en <= 1'b0; txd <= 2'b00; end
                    else begin tx_en <= 1'b1; txd <= sh[1:0]; end
                    sh  <= {2'b00, sh[7:2]};
                    dib <= dib + 2'd1;
                    if (state == S_PRE && cnt == 4'd7 && dib == 2'd3) sfd_strobe <= 1'b1;
                    // the next byte is loaded on the last di-bit
                    if (dib == 2'd3) begin
                        case (state)
                            S_PRE: begin
                                if (cnt == 4'd6) begin sh <= 8'hD5; cnt <= 4'd7; end
                                else if (cnt == 4'd7) begin
                                    // first data byte
                                    crc <= 32'hFFFFFFFF; crc_upd <= 1'b1; len <= 11'd1; bad <= 1'b0; state <= S_DATA;
                                    if (s_tvalid) begin
                                        sh <= s_tdata; s_tready <= 1'b1; last_in <= s_tlast;
                                    end else begin
                                        bad <= 1'b1; tx_underrun <= 1'b1; sh <= 8'h00; last_in <= 1'b1;
                                    end
                                end else begin sh <= 8'h55; cnt <= cnt + 4'd1; end
                            end
                            S_DATA: begin
                                if (!last_in) begin
                                    crc_upd <= 1'b1; len <= len + 11'd1;
                                    if (s_tvalid) begin
                                        sh <= s_tdata; s_tready <= 1'b1; last_in <= s_tlast;
                                    end else begin
                                        // underrun: end the frame here with a bad FCS
                                        bad <= 1'b1; tx_underrun <= 1'b1; last_in <= 1'b1; sh <= 8'h00;
                                    end
                                end else if (len < 11'd60) begin
                                    state <= S_PAD; sh <= 8'h00; crc_upd <= 1'b1; len <= len + 11'd1;
                                end else begin
                                    state <= S_FCS; cnt <= 4'd0; sh <= fcs[7:0];
                                end
                            end
                            S_PAD: begin
                                if (len < 11'd60) begin sh <= 8'h00; crc_upd <= 1'b1; len <= len + 11'd1; end
                                else begin state <= S_FCS; cnt <= 4'd0; sh <= fcs[7:0]; end
                            end
                            S_FCS: begin
                                cnt <= cnt + 4'd1;
                                case (cnt)
                                    4'd0: sh <= fcs[15:8];
                                    4'd1: sh <= fcs[23:16];
                                    4'd2: sh <= fcs[31:24];
                                    default: begin state <= S_IFG; cnt <= 4'd0; sh <= 8'h00; tx_done <= 1'b1; end
                                endcase
                            end
                            S_IFG: begin
                                // 12 byte times of gap (96 bit times)
                                if (cnt == 4'd11) begin state <= S_IDLE; cnt <= 4'd0; end
                                else cnt <= cnt + 4'd1;
                            end
                            default: state <= S_IDLE;
                        endcase
                    end
                end
            endcase
        end
    end
endmodule
`default_nettype wire
