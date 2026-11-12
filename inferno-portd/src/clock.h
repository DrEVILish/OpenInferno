/* clock.h - the desk clock (fpga/docs/inferno.md section 9.3, docs/68 section 3): the servo that disciplines
 * fpga/rtl/ptp_clock.v (PTP_INCR, MEDIA_INC, steps, media loads), and the audio loop that ties the desk's sample
 * clock to it through fs_phase.v: on the PRO1 a follower steers the CS2000 (its ratio, through the IO FPGA's command
 * engine at +0xC0 / +0xC4 / +0xC8, docs/68 section 10.3), a leader steers PTP_INCR to the desk's own sample clock.
 * Part of OpenProSeries inferno-portd. Licence: GPL-3.0-or-later.
 */
#ifndef PORTD_CLOCK_H
#define PORTD_CLOCK_H

#include "hw.h"
#include <stdint.h>

/* ---- the PI servo of the PTP follower (offset in ns -> frequency in ppb) */
enum { SERVO_UNLOCKED = 0, SERVO_JUMP = 1, SERVO_LOCKED = 2 };
typedef struct {
    double   kp, ki;               /* per sample (offset ns -> ppb) */
    double   step_ns;              /* |offset| above it: step (before lock; 10 x after) */
    double   max_ppb;
    int      state, count, lock_good;
    double   integ;                /* ppb */
    double   off0;
    uint64_t t0;
    double   last_ppb;
} servo_t;
void   servo_init(servo_t *s);
/* one measurement: offset = local - leader (ns) at local time t (ns). Returns the state; *ppb the frequency to set
 * (relative to the oscillator), *step the time step to apply (ns, local += step) when SERVO_JUMP */
int    servo_sample(servo_t *s, double offset_ns, uint64_t t_ns, double *ppb, double *step_ns);

/* ---- the audio loop: phase (desk frames - media frames) in samples -> a frequency correction in ppm */
typedef struct {
    double kp, ki;                 /* ppm per sample, ppm per sample-second */
    double rate_ppm_s, range_ppm;
    double integ, out_ppm;
    double ref;                    /* the phase at the start (the loop holds it) */
    int    have_ref;
    uint64_t last_t;
    double K;                      /* samples per second per ppm */
    /* frequency acquisition (a follower's first start, issue 007): 1 = measuring the phase's slope for DCLK_ACQ_NS
     * with the output held; 2 = gliding the output to the frequency measured at acq_ppm_s, then the phase found is
     * the reference and the phase loop runs from there (no phase to pull back, no bump); 3 = gliding, then measuring
     * again. acq_n: the rounds so far */
    int    acq, acq_n;
    double acq_ppm_s;
    uint64_t t0;                   /* when ref was taken */
} aloop_t;
void   aloop_init(aloop_t *a, int fs);
/* start a frequency acquisition at ppm_s (the CS2000 then moves ppm_s x 0.125 a write while it glides): a follower's
 * first start (loop_restart) */
void   aloop_acquire(aloop_t *a, double ppm_s);
double aloop_sample(aloop_t *a, double phase, uint64_t t_ns);   /* returns the correction (ppm) */

/* The desk clock never steps its rate (user 2026-10-20: lose the source and regain it with a glide): the audio loop's
 * output and the CS2000's commanded ratio move at most DCLK_SLEW_PPM_S, the ratio is written at most once per
 * DCLK_PERIOD_NS (the audio loop's period), so one write moves the commanded ratio by at most CS2K_STEP_PPM (0.625
 * ppm). 5 ppm/s is the audio loop's rate limit of 2026-10-19 ("no audible pitch steps"); GO LEADER from 40 ppm off
 * takes 8 s. */
#define DCLK_SLEW_PPM_S 5.0
#define DCLK_PERIOD_NS  125000000ull
#define CS2K_STEP_PPM   (DCLK_SLEW_PPM_S * 0.125)
#define DCLK_SETTLE_NS  2000000000ull   /* a follower's audio loop starts after the PTP has held its lock this long */
#define DCLK_ACQ_NS     250000000ull    /* the acquisition's measurement: 1 / 1042 sample in 0.25 s = 0.08 ppm at 48 kHz */
/* a first measurement up to DCLK_ACQ_MIN_PPM is left to the phase loop: 2 s into its lock the PTP may still move the
 * media clock a few ppm (4 ppm in test_clock), which the acquisition would chase. Above it: a glide, then rounds until
 * one measures at most DCLK_ACQ_DONE_PPM (the PTP has settled by then), DCLK_ACQ_ROUNDS at most */
#define DCLK_ACQ_MIN_PPM 10.0
#define DCLK_ACQ_DONE_PPM 1.0
#define DCLK_ACQ_ROUNDS 4
/* the most one write moves the ratio, in LSB: the commanded step plus the dither's carry, ceil(r0 x 0.625e-6) + 1
 * (3 LSB = 1.43 ppm at R0 = 2.0) */
uint32_t cs2k_max_step(uint32_t r0);

/* CS2000-CP ratio (12.20 fixed point) for a relative frequency, around r0 */
uint32_t cs2k_ratio(uint32_t r0, double ppm);
/* the same with sigma-delta dither (acc keeps the fraction): the average ratio is exact, finer than one LSB
 * (2^-20, 0.48 ppm at r0 = 2.0) */
uint32_t cs2k_dither(double *acc, uint32_t r0, double ppm);

/* ---- the desk clock */
enum { DCLK_LEADER = 0, DCLK_FOLLOWER = 1, DCLK_HOLDOVER = 2 };
typedef struct dclock dclock_t;
typedef struct {
    int      owner;                /* this daemon writes the desk clock (one daemon per desk) */
    int      fs;
    int      audio_loop;           /* fs_phase.v present and wanted */
    int      cs2k;                 /* steer the CS2000 (PRO1) */
    uint32_t cs2k_r0;
    int      fail_leader;          /* on source loss: 1 = GO LEADER, 0 = HOLDOVER (dclock_set_fail at run time) */
    double   acq_ppm_s;            /* a follower's acquisition glide (ppm/s); 0 = DCLK_SLEW_PPM_S */
} dclock_cfg_t;

dclock_t *dclock_new(hw_t *h, const dclock_cfg_t *c);
void   dclock_free(dclock_t *d);
/* FOLLOWER (the PTP follows the source), LEADER, HOLDOVER (the source lost: GO LEADER when fail_leader). The ratio and
 * the time of day's rate glide across every change, never step */
void   dclock_set_mode(dclock_t *d, int mode);
int    dclock_mode(const dclock_t *d);
/* clockfail at run time: what the next source loss does, and at once when the source is lost now */
void   dclock_set_fail(dclock_t *d, int fail_leader);
void   dclock_set_rate(dclock_t *d, int fs);
/* the PTP side */
void   dclock_set_freq(dclock_t *d, double ppb);     /* the time of day's rate against the oscillator */
double dclock_freq(const dclock_t *d);
void   dclock_step(dclock_t *d, int64_t delta_ns);
void   dclock_media_load(dclock_t *d);               /* media time = time of day x fs, now */
void   dclock_now(dclock_t *d, uint32_t *sec, uint32_t *ns);
int    dclock_freq_ppb(const dclock_t *d, int32_t *ppb);   /* for the heartbeat; 0 when not locked */
void   dclock_set_locked(dclock_t *d, int locked);   /* a follower's audio loop waits DCLK_SETTLE_NS into a lock */
void   dclock_tick(dclock_t *d, uint64_t now);
/* called after every media load (flows re-anchor) */
void   dclock_on_load(dclock_t *d, void (*fn)(void *), void *ctx);

typedef struct {
    int      mode, locked;
    double   ppb;
    double   audio_phase, audio_ppm;
    uint32_t cs2k_ratio;
    double   cs2k_ppm;             /* the commanded ratio, ppm of R0 */
    int      fail_leader;
    uint64_t cs2k_writes, cs2k_errors, media_loads, steps;
    double   media_err;            /* samples, media clock against time of day x fs */
} dclock_stats_t;
void   dclock_get_stats(const dclock_t *d, dclock_stats_t *s);

#endif
