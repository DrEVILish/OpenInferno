/* daemon.h - port start-up options (daemon.c). Part of OpenProSeries inferno-portd. Licence: GPL-3.0-or-later. */
#ifndef PORTD_DAEMON_H
#define PORTD_DAEMON_H

#include "portd.h"

typedef struct {
    int      clock_owner;      /* this daemon writes the desk clock (port 0's registers) */
    int      audio_loop, cs2k, fail_leader;
    uint32_t cs2k_r0;
    int      ptp_role, ptp_steer;
    int8_t   log_sync;
    uint8_t  stratum;
    double   acq_ppm_s;        /* --audio-acquire-ppm-s (0: DCLK_SLEW_PPM_S) */
} portd_opts_t;

int  portd_start(port_t *p, hw_t *hw, const portd_opts_t *o);
void portd_stop(port_t *p);

#endif
