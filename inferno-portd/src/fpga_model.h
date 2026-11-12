/* fpga_model.h - a software model of one FPGA Inferno port (aoip_regs + inferno_regs / inferno_tx / inferno_rx +
 * cpu_pktq + ptp_clock), the desk audio clock (fs_phase.v) and the CS2000 command engine, for host tests.
 *
 * The model is register-exact for what inferno-portd uses (fpga/docs/inferno.md section 6), sends and receives real
 * Ethernet frames on a "PHY" (an AF_PACKET socket on a veth, or a datagram socket pair), packetises its TX flows from
 * the flow table at the media clock (a test pattern per column) and claims received flows from the RX table, checking
 * the pattern. Time comes from g_now_ns (real or virtual). Part of OpenProSeries inferno-portd.
 * Licence: GPL-3.0-or-later.
 */
#ifndef PORTD_FPGA_MODEL_H
#define PORTD_FPGA_MODEL_H

#include "hw.h"

/* an in-memory wire between two models of one process (tests): frames with their send time, both ways */
typedef struct model_wire model_wire_t;
model_wire_t *model_wire_new(void);
void model_wire_cut(model_wire_t *w, int cut);   /* 1: frames sent are lost, both ways (a pulled cable); 0: joined */

typedef struct {
    int      id;              /* 0..15: goes into the test pattern */
    double   xtal_ppm;        /* the model's 50 MHz oscillator error */
    double   desk_ppm;        /* the desk audio clock's (CS2000 path) error at the nominal ratio */
    int      maxlat;          /* MAXLAT of the build (0..3) */
    uint32_t port_base, clock_base, dclk_base, cs2k_base;
    int      phy_fd;          /* PHY descriptor; -1: none (frames sent are dropped) */
    int      phy_packet;      /* 1: AF_PACKET (skip outgoing copies), 0: datagram socket */
    uint32_t wire_delay_ns;   /* datagram / in-memory PHY: frames arrive this long after the send time */
    model_wire_t *wire;       /* in-memory PHY (phy_fd -1), side 0 or 1 */
    int      wire_side;
    uint32_t cs2k_r0;         /* the CS2000 ratio (12.20) at which the desk clock runs at its nominal rate */
    uint64_t tod_start_ns;    /* the time of day at power-up (the RTL starts at 0) */
    int      jb_frames;       /* NF: jitter-buffer frames at 48 kHz (2 NF at 96 kHz); 0: 256, the full build */
} model_cfg_t;

hw_t *model_open(const model_cfg_t *cfg);
/* an AF_PACKET socket on an interface, promiscuous (the e2e test's veth) */
int   model_packet_socket(const char *ifname);

typedef struct {
    uint64_t tx_frames_cpu, rx_frames_cpu, tx_media, rx_media, rx_late, rx_early, rx_bad;
    uint64_t pat_ok, pat_bad;
    double   tod_s;           /* the model's time of day (seconds) */
    double   media_pos;       /* media_ts + NCO fraction (samples, not wrapped) */
    double   desk_pos;        /* desk audio frames (samples) */
    double   desk_rate_rel;   /* the desk audio clock's rate / nominal */
    double   tick_rate_rel;   /* the oscillator rate / nominal */
    uint32_t cs2k_ratio;      /* the CS2000 ratio in force (12.20) */
    uint32_t cs2k_max_step;   /* the largest ratio change one write made (LSB; R0 before the first) */
    uint64_t cs2k_applied;    /* ratio writes applied (on unfreeze) */
} model_stats_t;
void model_get_stats(hw_t *h, model_stats_t *s);
/* the received column map: col -> {valid, transmitter id, transmitter column} */
int  model_colsrc(hw_t *h, int col, int *tx_id, int *tx_col);

#endif
