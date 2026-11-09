/* clock.c - see clock.h. Our own design (OpenProSeries); the PI servo follows the usual PTP practice (proportional +
 * integral on the offset, a frequency estimate from the first two samples, a step when far off). The CS2000 register
 * use (Ratio 0x06..0x09 in 12.20 fixed point, Global Configuration 0x05: Freeze bit 3, EnDevCfg2 bit 0) is from the
 * public Cirrus Logic CS2000-CP data sheet; the command engine (+0xC0 = reg << 8 | data, +0xC8 strobe, +0xC4 busy /
 * echo) is docs/68 section 10.3. Part of OpenProSeries inferno-portd. Licence: GPL-3.0-or-later.
 */
#include "clock.h"
#include "regs.h"
#include "util.h"

#include <math.h>
#include <stdlib.h>

#define INCR_NOMINAL 335544320.0   /* 20 ns in 8.24 (50 MHz) */

/* ---- servo */
void servo_init(servo_t *s)
{
    double keep = s->last_ppb;
    memset(s, 0, sizeof(*s));
    s->kp = 0.7;          /* ppb per ns */
    s->ki = 0.3;          /* ppb per ns-second */
    s->step_ns = 1000000.0;
    s->max_ppb = 500000.0;
    s->last_ppb = keep;
    s->integ = keep;
}

static double clampd(double v, double m) { return v > m ? m : v < -m ? -m : v; }

int servo_sample(servo_t *s, double off, uint64_t t, double *ppb, double *step)
{
    *step = 0;
    if (s->count == 0) {
        s->off0 = off; s->t0 = t; s->count = 1;
        *ppb = s->last_ppb;
        return SERVO_UNLOCKED;
    }
    if (s->count == 1) {
        double dt = (double)(t - s->t0) * 1e-9;
        if (dt < 0.05) { *ppb = s->last_ppb; return SERVO_UNLOCKED; }
        /* the offset grows at (our rate - the leader's): the frequency that stops it */
        s->integ = clampd(s->last_ppb - (off - s->off0) / dt, s->max_ppb);
        s->count = 2;
        s->t0 = t;
        if (fabs(off) > s->step_ns) {
            *step = -off;
            *ppb = s->last_ppb = s->integ;
            s->state = SERVO_JUMP;
            return SERVO_JUMP;
        }
    }
    double dt = (double)(t - s->t0) * 1e-9;
    s->t0 = t;
    if (dt > 4.0) dt = 4.0;
    if (fabs(off) > s->step_ns * 10.0) {   /* far off again: step, keep the frequency */
        *step = -off;
        *ppb = s->last_ppb;
        return SERVO_JUMP;
    }
    s->integ = clampd(s->integ - s->ki * off * dt, s->max_ppb);
    double f = clampd(s->integ - s->kp * off, s->max_ppb);
    s->last_ppb = f;
    *ppb = f;
    s->lock_good = fabs(off) < 5000.0 ? s->lock_good + 1 : 0;
    s->state = SERVO_LOCKED;
    return SERVO_LOCKED;
}

/* ---- audio loop: a type-2 PLL, natural frequency 0.02 Hz, damping 0.7, on the phase in samples; the plant gain is
 * fs x 1e-6 samples per second per ppm */
void aloop_init(aloop_t *a, int fs)
{
    memset(a, 0, sizeof(*a));
    double K = (double)fs * 1e-6, wn = 2.0 * M_PI * 0.02, z = 0.7;
    a->kp = 2.0 * z * wn / K;      /* 48 kHz: 3.7 ppm per sample */
    a->ki = wn * wn / K;           /* 48 kHz: 0.33 ppm per sample-second */
    a->rate_ppm_s = DCLK_SLEW_PPM_S;   /* no audible pitch steps */
    a->range_ppm = 250.0;
}

double aloop_sample(aloop_t *a, double phase, uint64_t t)
{
    if (!a->have_ref) { a->ref = phase; a->have_ref = 1; a->last_t = t; return a->out_ppm; }
    double e = phase - a->ref;
    double dt = (double)(t - a->last_t) * 1e-9;
    a->last_t = t;
    if (dt <= 0 || dt > 2.0) dt = 0.125;
    a->integ = clampd(a->integ + a->ki * e * dt, a->range_ppm);
    double u = clampd(a->kp * e + a->integ, a->range_ppm);
    double lim = a->rate_ppm_s * dt;
    if (u > a->out_ppm + lim) u = a->out_ppm + lim;
    if (u < a->out_ppm - lim) u = a->out_ppm - lim;
    a->out_ppm = u;
    return u;
}

uint32_t cs2k_ratio(uint32_t r0, double ppm) { return (uint32_t)llround((double)r0 * (1.0 + ppm * 1e-6)); }
uint32_t cs2k_max_step(uint32_t r0) { return (uint32_t)ceil((double)r0 * CS2K_STEP_PPM * 1e-6) + 1; }

uint32_t cs2k_dither(double *acc, uint32_t r0, double ppm)
{
    double x = (double)r0 * (1.0 + ppm * 1e-6), b = floor(x);
    *acc += x - b;
    if (*acc >= 1.0) { *acc -= 1.0; return (uint32_t)b + 1; }
    return (uint32_t)b;
}

/* ---- the desk clock */
struct dclock {
    hw_t    *hw;
    dclock_cfg_t c;
    int      mode, locked;
    int      failed;           /* the source is lost: HOLDOVER or GO LEADER, as clockfail says (dclock_set_fail) */
    int      wait_lock;        /* a follower's audio loop waits for the PTP's lock and for its ratio to be back at the
                                * one it tracked last */
    uint64_t lock_t;           /* locked since */
    double   ppb;              /* PTP servo's frequency */
    double   leader_ppb;       /* leader: the audio loop's */
    double   incr_acc, minc_acc;
    uint32_t incr_w, minc_w;
    aloop_t  al;
    int      al_have;
    uint32_t al_last_diff;
    double   al_unwrap;
    double   audio_phase, audio_ppm;
    uint32_t cs_ratio;         /* the ratio written */
    double   cs_acc;
    double   cs_ppm;           /* the commanded ratio (ppm of R0; the dither's average): it moves CS2K_STEP_PPM a write */
    double   cs_track;         /* the commanded ratio while the follower's loop last ran */
    double   cs_base;          /* the commanded ratio when the leader started (the time of day follows its change) */
    uint64_t next_audio, next_check, last_cs_write;
    int      loaded;
    void   (*onload)(void *);
    void    *onload_ctx;
    dclock_stats_t st;
};

static int has_dclk(const dclock_t *d) { return d->hw->dclk_base != 0xFFFFFFFFu; }

static int cs2k_cmd(dclock_t *d, uint8_t reg, uint8_t val)
{
    hw_t *h = d->hw;
    if (h->cs2k_base == 0xFFFFFFFFu) return -1;
    uint32_t cmd = ((uint32_t)reg << 8) | val;
    hw_wr(h, h->cs2k_base + CS2K_CMD, cmd);
    hw_wr(h, h->cs2k_base + CS2K_STROBE, 1);
    uint64_t t0 = mono_ns();
    for (;;) {
        uint32_t e = hw_rd(h, h->cs2k_base + CS2K_ECHO);
        if (!(e & 0x80000000u)) {
            if ((e & 0xFFFF) == cmd) return 0;
            d->st.cs2k_errors++;
            return -1;
        }
        if (mono_ns() - t0 > 2000000ull) { d->st.cs2k_errors++; return -1; }   /* the vendor allows about 1 ms */
    }
}

static void cs2k_write_ratio(dclock_t *d, uint32_t r)
{
    /* Freeze, the four ratio bytes, unfreeze: the new ratio applies at once, never half-written */
    int e = cs2k_cmd(d, 0x05, 0x09);
    for (int i = 0; i < 4; i++) e |= cs2k_cmd(d, (uint8_t)(0x06 + i), (uint8_t)(r >> (24 - 8 * i)));
    e |= cs2k_cmd(d, 0x05, 0x01);
    if (!e) { d->cs_ratio = r; d->st.cs2k_writes++; }
}

dclock_t *dclock_new(hw_t *h, const dclock_cfg_t *c)
{
    dclock_t *d = calloc(1, sizeof(*d));
    d->hw = h;
    d->c = *c;
    if (!d->c.fs) d->c.fs = 48000;
    if (!d->c.cs2k_r0) d->c.cs2k_r0 = 0x00200000u;
    d->mode = DCLK_LEADER;
    d->cs_ratio = d->c.cs2k_r0;
    aloop_init(&d->al, d->c.fs);
    if (d->c.owner && has_dclk(d)) {
        if (hw_rd(h, h->dclk_base + FP_ID) == FP_ID_VALUE) hw_wr(h, h->dclk_base + FP_CTRL, 3);
        else { LOGW("fs_phase not found: no audio loop"); d->c.audio_loop = 0; }
    }
    return d;
}

void dclock_free(dclock_t *d) { free(d); }
int  dclock_mode(const dclock_t *d) { return d->mode; }
void dclock_on_load(dclock_t *d, void (*fn)(void *), void *ctx) { d->onload = fn; d->onload_ctx = ctx; }

/* a leader: the audio loop's rate times the CS2000's change since the leader started, so the time of day follows a
 * glide of the ratio at once (feed-forward) and the loop sees no phase error from it */
static double leader_freq(const dclock_t *d)
{
    double k = (1.0 + d->cs_ppm * 1e-6) / (1.0 + d->cs_base * 1e-6);
    return ((1.0 + d->leader_ppb * 1e-9) * k - 1.0) * 1e9;
}
double dclock_freq(const dclock_t *d) { return d->mode == DCLK_LEADER ? leader_freq(d) : d->ppb; }

/* (re)start the audio loop without a bump: it takes the phase it finds as its reference (an old phase is never pulled
 * back) and starts from the correction in force, the CS2000's ratio (follower) or the time of day's rate (leader) */
static void loop_restart(dclock_t *d)
{
    aloop_init(&d->al, d->c.fs);
    d->al.integ = d->al.out_ppm = clampd(d->mode == DCLK_LEADER ? d->leader_ppb * 1e-3 : -d->cs_ppm, d->al.range_ppm);
    d->al_have = 0;
}

static void enter(dclock_t *d, int mode)
{
    if (mode == d->mode) return;
    LOGI("desk clock: mode %d -> %d", d->mode, mode);
    double f = dclock_freq(d);   /* the time of day keeps its rate across every change */
    d->mode = mode;
    d->ppb = d->leader_ppb = f;
    d->cs_base = d->cs_ppm;
    d->wait_lock = mode == DCLK_FOLLOWER;
    loop_restart(d);
}

/* HOLDOVER is the source lost: clockfail makes it GO LEADER. Every change glides (dclock_tick): a leader takes the
 * ratio to R0, holdover keeps the ratio in force, a follower (back at its source) takes it back to the one it tracked
 * last and starts its loop there */
void dclock_set_mode(dclock_t *d, int mode)
{
    d->failed = mode == DCLK_HOLDOVER;
    if (d->failed && d->c.fail_leader) mode = DCLK_LEADER;
    enter(d, mode);
}

void dclock_set_fail(dclock_t *d, int fail_leader)
{
    d->c.fail_leader = fail_leader;
    if (d->failed) enter(d, fail_leader ? DCLK_LEADER : DCLK_HOLDOVER);   /* the source is lost now: at once */
}

void dclock_set_rate(dclock_t *d, int fs)
{
    d->c.fs = fs;
    loop_restart(d);
    d->loaded = 0;   /* reload the media clock at the new rate */
}

void dclock_set_freq(dclock_t *d, double ppb) { d->ppb = ppb; }
void dclock_set_locked(dclock_t *d, int l)
{
    if (l && !d->locked) d->lock_t = g_now();
    d->locked = l;
}

int dclock_freq_ppb(const dclock_t *d, int32_t *ppb)
{
    if (!d->locked && d->mode != DCLK_LEADER) return 0;
    *ppb = (int32_t)llround(dclock_freq(d));
    return 1;
}

void dclock_now(dclock_t *d, uint32_t *sec, uint32_t *ns)
{
    *sec = clk_rd(d->hw, AOIP_TOD_SEC_LO);
    *ns = clk_rd(d->hw, AOIP_TOD_NS);
}

void dclock_media_load(dclock_t *d)
{
    if (!d->c.owner) return;
    hw_t *h = d->hw;
    uint32_t sec, ns;
    dclock_now(d, &sec, &ns);
    uint64_t fs = (uint64_t)d->c.fs;
    uint64_t sub = (uint64_t)ns * fs / 1000000000ull;
    uint64_t T = (uint64_t)sec * fs + sub;   /* the media time is the time of day x fs (upstream media_clock.rs) */
    clk_wr(h, AOIP_SET_MEDIA, (uint32_t)T);
    hw_wr(h, h->clock_base + INF_SET_MSEC, sec);
    hw_wr(h, h->clock_base + INF_SET_MSUB, (uint32_t)sub);
    clk_wr(h, AOIP_CLOCK_CMD, 2);
    d->st.media_loads++;
    d->loaded = 1;
    d->al_have = 0;
    if (d->onload) d->onload(d->onload_ctx);
}

void dclock_step(dclock_t *d, int64_t delta)
{
    if (!d->c.owner) return;
    uint32_t sec, ns;
    dclock_now(d, &sec, &ns);
    int64_t t = (int64_t)sec * 1000000000LL + ns + delta;
    if (t < 0) t = 0;
    uint64_t s = (uint64_t)t / 1000000000ull;
    clk_wr(d->hw, AOIP_SET_SEC_HI, (uint32_t)(s >> 32));
    clk_wr(d->hw, AOIP_SET_SEC_LO, (uint32_t)s);
    clk_wr(d->hw, AOIP_SET_NS, (uint32_t)((uint64_t)t % 1000000000ull));
    clk_wr(d->hw, AOIP_CLOCK_CMD, 1);
    d->st.steps++;
    dclock_media_load(d);
}

/* sigma-delta on the two 32-bit increments: their average is exact (MEDIA_INC alone is 0.24 ppm coarse, and the
 * nominal 48 kHz value 4123168.6 is not an integer) */
static void write_incs(dclock_t *d)
{
    double f = dclock_freq(d) * 1e-9;
    double incr = INCR_NOMINAL * (1.0 + f);
    double minc = incr * (double)d->c.fs * 256.0 / 1e9;
    double ib = floor(incr), mb = floor(minc);
    d->incr_acc += incr - ib;
    d->minc_acc += minc - mb;
    uint32_t iw = (uint32_t)ib, mw = (uint32_t)mb;
    if (d->incr_acc >= 1.0) { d->incr_acc -= 1.0; iw++; }
    if (d->minc_acc >= 1.0) { d->minc_acc -= 1.0; mw++; }
    if (iw != d->incr_w) { clk_wr(d->hw, AOIP_PTP_INCR, iw); d->incr_w = iw; }
    if (mw != d->minc_w) { clk_wr(d->hw, AOIP_MEDIA_INC, mw); d->minc_w = mw; }
}

/* *target: the follower's ratio (ppm) */
static void audio_loop(dclock_t *d, uint64_t now, double *target)
{
    hw_t *h = d->hw;
    hw_wr(h, h->dclk_base + FP_SNAP, 1);
    uint32_t frames = hw_rd(h, h->dclk_base + FP_DESK_FRAMES), ts = hw_rd(h, h->dclk_base + FP_MEDIA_TS);
    uint32_t ticks = hw_rd(h, h->dclk_base + FP_TICKS), period = hw_rd(h, h->dclk_base + FP_PERIOD);
    if (!period || !frames) return;
    uint32_t diff = frames - ts;
    /* a new count (start, media load): a new reference phase, the loop's correction stays (no step) */
    if (!d->al_have) { d->al_unwrap = 0; d->al_last_diff = diff; d->al_have = 1; d->al.have_ref = 0; }
    d->al_unwrap += (double)(int32_t)(diff - d->al_last_diff);
    d->al_last_diff = diff;
    /* desk frames - media frames at the same desk edge (samples) */
    double phase = d->al_unwrap - (double)ticks / (double)period;
    double u = aloop_sample(&d->al, phase, now);
    d->audio_phase = phase - d->al.ref;   /* the error the loop holds to 0 */
    d->audio_ppm = u;
    if (d->mode == DCLK_FOLLOWER) *target = -u;    /* the desk is ahead: slow the CS2000 */
    else d->leader_ppb = u * 1000.0;               /* the desk is ahead: the time of day speeds up */
}

/* one write at most per period: the commanded ratio moves CS2K_STEP_PPM at most toward the target, dithered (the
 * dither keeps the average exact in holdover too) */
static void cs2k_steer(dclock_t *d, double target, uint64_t now)
{
    d->cs_ppm += clampd(target - d->cs_ppm, CS2K_STEP_PPM);
    uint32_t r = cs2k_dither(&d->cs_acc, d->c.cs2k_r0, d->cs_ppm);
    if (r != d->cs_ratio) { cs2k_write_ratio(d, r); d->last_cs_write = now; }
}

void dclock_tick(dclock_t *d, uint64_t now)
{
    if (!d->c.owner) return;
    if (!d->loaded) dclock_media_load(d);
    write_incs(d);
    /* a follower's audio loop starts when the PTP has held its lock DCLK_SETTLE_NS (a pull-in swings through the lock
     * window and its servo is still settling there: the loop would drag the desk after it) and the ratio has glided
     * back to the one it tracked last (the loop takes its reference phase there: none of the glide to pull back) */
    if (d->wait_lock && d->locked && (int64_t)(now - d->lock_t) >= (int64_t)DCLK_SETTLE_NS && fabs(d->cs_ppm - d->cs_track) < 1e-6) {
        d->wait_lock = 0;
        loop_restart(d);
    }
    if (now >= d->next_audio) {
        /* the ratio each mode glides to: a follower's loop output (until the loop starts: the ratio it tracked last), R0
         * for a leader, the ratio in force in holdover */
        int follow = d->mode == DCLK_FOLLOWER && !d->wait_lock;
        double target = d->mode == DCLK_LEADER ? 0.0 : d->mode == DCLK_FOLLOWER && d->wait_lock ? d->cs_track : d->cs_ppm;
        int loop = d->c.audio_loop && has_dclk(d) && (follow || d->mode == DCLK_LEADER);
        if (loop) audio_loop(d, now, &target);
        if (d->c.cs2k) cs2k_steer(d, target, now);
        if (loop && follow) d->cs_track = d->cs_ppm;
        d->next_audio = now + DCLK_PERIOD_NS;
    }
    if (now >= d->next_check) {
        /* the media clock against the time of day x fs */
        hw_t *h = d->hw;
        uint32_t sec, ns;
        dclock_now(d, &sec, &ns);
        uint32_t msec = hw_rd(h, h->clock_base + INF_MEDIA_SEC), msub = hw_rd(h, h->clock_base + INF_MEDIA_SUB);
        double exp_sub = (double)ns * d->c.fs * 1e-9;
        double err = (double)(int32_t)(msec - sec) * d->c.fs + ((double)msub - exp_sub);
        d->st.media_err = err;
        if (fabs(err) > 2.0) { LOGI("media clock %.1f samples off the time of day: reloading", err); dclock_media_load(d); }
        d->next_check = now + 1000000000ull;
    }
}

void dclock_get_stats(const dclock_t *d, dclock_stats_t *s)
{
    *s = d->st;
    s->mode = d->mode;
    s->locked = d->locked;
    s->ppb = dclock_freq(d);
    s->audio_phase = d->audio_phase;
    s->audio_ppm = d->audio_ppm;
    s->cs2k_ratio = d->cs_ratio;
    s->cs2k_ppm = d->cs_ppm;
    s->fail_leader = d->c.fail_leader;
}
