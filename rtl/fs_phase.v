// fs_phase.v - the desk audio clock measured against the media clock: the phase detector of the desk-clock loops
// (fpga/docs/inferno.md section 9.3). On the PRO1 the desk's sample clock is the hardware audio clock (CS2000 ->
// DCM), not ptp_clock's NCO; a PTP follower must steer the CS2000 so that the desk's frames stay on the media
// clock, and a leader must steer the NCO (PTP_INCR / MEDIA_INC) to the desk's frames. Both loops run on the AMU
// (inferno-portd clock.c); this block gives them the phase.
//
// Provenance: written for OpenProSeries (W16), our own design. No vendor material. Licence: GPL-3.0.
//
// At every rising edge of desk_fs (the desk frame clock, any clock domain: two-stage synchroniser, edge detect) the
// block latches the media clock's position: media_ts (the frame of the last frame_strobe) and the clocks since that
// strobe; the media frame position at the desk edge is MEDIA_TS + TICKS / PERIOD frames. The desk edges are counted.
// The AMU writes SNAP and then reads one coherent set: phase = DESK_FRAMES - (MEDIA_TS + TICKS / PERIOD) (mod 2^32;
// the loop holds its change at 0). The synchroniser adds a constant 2..3 clocks, which the loop does not see.
// Resolution: one clock, 1 / 1042 frame at 48 kHz on 50 MHz.
//
// Registers (32-bit, byte offsets; the top level decodes the block):
//   0x00 ID           RO 0x4F465031 ("OFP1")
//   0x04 CTRL         RW bit0 enable (count desk edges); write bit1 = 1: clear the edge count
//   0x08 SNAP         WO any value: copy the latches below into the snapshot
//   0x0C DESK_FRAMES  RO snapshot: desk edges counted
//   0x10 MEDIA_TS     RO snapshot: media_ts at that edge
//   0x14 TICKS        RO snapshot: clocks from the last frame_strobe to that edge
//   0x18 PERIOD       RO snapshot: clocks between the last two frame_strobes
//   0x1C DESK_PERIOD  RO snapshot: clocks between the last two desk edges
`timescale 1ns / 1ps
`default_nettype none
module fs_phase (
    input  wire        clk,
    input  wire        rst,
    input  wire        frame_strobe,   // ptp_clock: one clock per media frame
    input  wire [31:0] media_ts,       // ptp_clock: the media frame (already counted in the strobe's clock)
    input  wire        desk_fs,        // the desk frame clock, asynchronous
    input  wire [7:0]  reg_addr,
    input  wire [7:0]  bus_addr,       // the read address (lbus_target: the live bus address)
    input  wire        reg_we,
    input  wire [31:0] reg_wdata,
    input  wire        reg_re,
    output reg  [31:0] reg_rdata
);
    reg  [2:0]  sy;
    always @(posedge clk) sy <= {sy[1:0], desk_fs};
    wire        edge_d = sy[1] && !sy[2];

    reg         en;
    reg  [31:0] since_strobe, period, since_desk;
    reg  [31:0] frames, l_ts, l_ticks, l_period, l_dperiod;
    reg  [31:0] s_frames, s_ts, s_ticks, s_period, s_dperiod;
    always @(posedge clk) begin
        if (rst) begin
            en <= 1'b0; since_strobe <= 32'd0; period <= 32'd0; since_desk <= 32'd0; frames <= 32'd0;
            l_ts <= 32'd0; l_ticks <= 32'd0; l_period <= 32'd0; l_dperiod <= 32'd0;
            s_frames <= 32'd0; s_ts <= 32'd0; s_ticks <= 32'd0; s_period <= 32'd0; s_dperiod <= 32'd0;
        end else begin
            if (frame_strobe) begin since_strobe <= 32'd1; period <= since_strobe; end
            else since_strobe <= since_strobe + 32'd1;
            since_desk <= edge_d ? 32'd1 : since_desk + 32'd1;
            if (edge_d && en) begin
                frames <= frames + 32'd1;
                // a strobe in the same clock: the edge belongs to the new frame, 0 clocks into it
                l_ts <= media_ts;
                l_ticks <= frame_strobe ? 32'd0 : since_strobe;
                l_period <= frame_strobe ? since_strobe : period;
                l_dperiod <= since_desk;
            end
            if (reg_we && reg_addr == 8'h04) begin en <= reg_wdata[0]; if (reg_wdata[1]) frames <= 32'd0; end
            if (reg_we && reg_addr == 8'h08) begin
                s_frames <= frames; s_ts <= l_ts; s_ticks <= l_ticks; s_period <= l_period; s_dperiod <= l_dperiod;
            end
        end
    end
    always @(*) begin
        case (bus_addr)
            8'h00: reg_rdata = 32'h4F465031;
            8'h04: reg_rdata = {31'd0, en};
            8'h0C: reg_rdata = s_frames;
            8'h10: reg_rdata = s_ts;
            8'h14: reg_rdata = s_ticks;
            8'h18: reg_rdata = s_period;
            8'h1C: reg_rdata = s_dperiod;
            default: reg_rdata = 32'd0;
        endcase
    end
    wire unused = &{1'b0, reg_re};
endmodule
`default_nettype wire
