/* test_clock.c - the desk clock across a source loss and its return, in virtual time: a PTPv1 leader A and a follower
 * B whose desk clock has fs_phase and the CS2000 engine (model: oscillator +50 ppm, CS2000 reference +40 ppm, so B
 * tracks A at about -40 ppm of R0), the wire between them cut and joined again. The user's rule (2026-10-20): keep the
 * automatic return to the source, but glide instead of stepping; the audio never sees a step in the CS2000 ratio.
 * Checks: no ratio write moves more than cs2k_max_step() (the model records every write), GO LEADER glides to R0, the
 * return glides to the tracked ratio, a 4..8 s gap leaves holdover (the loss seen by the best-master choice, and by
 * the 4 s timer while still following), clockfail applies at run time.
 * Part of OpenProSeries inferno-portd. Licence: GPL-3.0-or-later.
 */
#include "test.h"
#include "../src/clock.h"
#include "../src/ctl.h"
#include "../src/daemon.h"
#include "../src/fpga_model.h"
#include "../src/ptp.h"
#include "../src/util.h"

#include <math.h>
#include <string.h>

#define R0 0x00200000u

static uint64_t vt = 5000000000ull;
static uint64_t vnow(void) { return vt; }

static port_t A, B;
static model_wire_t *W;

/* B every 125 ms: the ratio in force (the model's), the desk clock's mode and lock, the audio phase, the model's desk
 * frames - media frames (samples, what an elastic buffer between the two sees) */
typedef struct { uint64_t t; double ppm; int mode, locked; double phase, abs; } smp_t;
#define NSMP 16384
static smp_t S[NSMP];
static int ns;

static double ppm_of(uint32_t r) { return ((double)r / (double)R0 - 1.0) * 1e6; }

static hw_t *mk(int side, int id, double xppm, double dppm, uint64_t tod0, int desk)
{
    model_cfg_t m;
    memset(&m, 0, sizeof(m));
    m.id = id; m.xtal_ppm = xppm; m.desk_ppm = dppm; m.maxlat = 3;
    m.port_base = 0x1000; m.clock_base = 0x1000;
    m.dclk_base = desk ? 0x0200 : 0xFFFFFFFFu; m.cs2k_base = desk ? 0x0300 : 0xFFFFFFFFu; m.cs2k_r0 = R0;
    m.phy_fd = -1; m.wire = W; m.wire_side = side; m.wire_delay_ns = 5000; m.tod_start_ns = tod0;
    return model_open(&m);
}

static void run(double seconds)
{
    uint64_t end = vt + (uint64_t)(seconds * 1e9);
    while (vt < end) {
        vt += 1000000ull;
        port_tick(&A, vt);
        port_tick(&B, vt);
        if (vt % 125000000ull == 0 && ns < NSMP) {
            model_stats_t m;
            dclock_stats_t c;
            model_get_stats(B.hw, &m);
            dclock_get_stats(B.clk, &c);
            S[ns++] = (smp_t){ vt, ppm_of(m.cs2k_ratio), c.mode, c.locked, c.audio_phase, m.desk_pos - m.media_pos };
        }
    }
}

/* log_sync: B's own; 1 makes its qualification window 8 s, so a loss is seen by the 4 s timer while B is SLAVE */
static void setup(int fail_leader, int8_t log_sync)
{
    W = model_wire_new();
    port_defaults(&A);
    port_defaults(&B);
    A.n = 0; snprintf(A.desk, sizeof(A.desk), "FOH"); A.mac[5] = 0x11;
    B.n = 1; snprintf(B.desk, sizeof(B.desk), "MON"); B.mac[5] = 0x22;
    portd_opts_t oa = { 1, 0, 0, 0, 0, PTP_ROLE_LEADER, 1, -2, 0 };
    portd_opts_t ob = { 1, 1, 1, fail_leader, R0, PTP_ROLE_FOLLOWER, 1, log_sync, 0 };
    CHECK(portd_start(&A, mk(0, 1, 0.0, 0.0, 1760000000ull * 1000000000ull, 0), &oa) == 0);
    CHECK(portd_start(&B, mk(1, 2, 50.0, 40.0, 1760000003ull * 1000000000ull + 123456789ull, 1), &ob) == 0);
    ns = 0;
}

static uint32_t max_step(void)
{
    model_stats_t m;
    model_get_stats(B.hw, &m);
    return m.cs2k_max_step;
}

static void teardown(void)
{
    hw_t *ha = A.hw, *hb = B.hw;
    portd_stop(&A);
    portd_stop(&B);
    ha->ops->close(ha);
    hb->ops->close(hb);
    free(W);
}

static double ratio_ppm(void) { return ns ? S[ns - 1].ppm : 0.0; }
static int mode(void) { return dclock_mode(B.clk); }

/* the samples from time t0 on: the most the ratio moved in any 1 s (8 samples), the largest |phase| while following */
static double worst_rate(uint64_t t0)
{
    double w = 0;
    for (int i = 8; i < ns; i++)
        if (S[i - 8].t >= t0 && fabs(S[i].ppm - S[i - 8].ppm) > w) w = fabs(S[i].ppm - S[i - 8].ppm);
    return w;
}
static double worst_phase(uint64_t t0, int want_mode)
{
    double w = 0;
    for (int i = 0; i < ns; i++)
        if (S[i].t >= t0 && S[i].mode == want_mode && fabs(S[i].phase) > w) w = fabs(S[i].phase);
    return w;
}
/* the first sample from t0 on whose ratio is within tol of ppm (-1: none) */
static int first_near(uint64_t t0, double ppm, double tol)
{
    for (int i = 0; i < ns; i++) if (S[i].t >= t0 && fabs(S[i].ppm - ppm) <= tol) return i;
    return -1;
}
static int first_off(uint64_t t0, double ppm, double tol)
{
    for (int i = 0; i < ns; i++) if (S[i].t >= t0 && fabs(S[i].ppm - ppm) > tol) return i;
    return -1;
}

/* frames an elastic buffer of +-1 sample, centred at the first sample after t0, repeats or drops from t0 on */
static int slips(uint64_t t0)
{
    int n = 0, have = 0;
    double c = 0;
    for (int i = 0; i < ns; i++) {
        if (S[i].t <= t0) continue;
        if (!have) { c = S[i].abs; have = 1; }
        while (S[i].abs - c > 1.0) { c += 1.0; n++; }
        while (c - S[i].abs > 1.0) { c -= 1.0; n++; }
    }
    return n;
}
static double worst_off(uint64_t t0, double ppm)
{
    double w = 0;
    for (int i = 0; i < ns; i++) if (S[i].t > t0 && fabs(S[i].ppm - ppm) > w) w = fabs(S[i].ppm - ppm);
    return w;
}

/* the dither moves the ratio by one LSB around the commanded value: 1 LSB = 0.477 ppm at R0 = 2.0 */
#define LSB_PPM (1e6 / (double)R0)

static double settle(void)
{
    run(200.0);
    dclock_stats_t c;
    dclock_get_stats(B.clk, &c);
    double r = ratio_ppm();
    CHECKF(c.mode == DCLK_FOLLOWER && c.locked, "mode %d locked %d", c.mode, c.locked);
    CHECKF(fabs(r + 40.0) < 1.0 && fabs(c.audio_phase) < 0.5, "ratio %.2f ppm, phase %.3f", r, c.audio_phase);
    return r;
}

static void test_holdover_gaps(int8_t log_sync)
{
    printf("- HOLDOVER (B's log_sync %d): gaps of 4.5 / 6 / 7.5 / 12 s: the ratio is held, then B follows the source again\n",
           log_sync);
    setup(0, log_sync);
    double track = settle();
    static const double gaps[] = { 4.5, 6.0, 7.5, 12.0 };
    for (int g = 0; g < 4; g++) {
        model_wire_cut(W, 1);
        uint64_t t0 = vt;
        run(gaps[g]);
        CHECKF(mode() == DCLK_HOLDOVER, "gap %.1f s: mode %d", gaps[g], mode());
        CHECKF(fabs(ratio_ppm() - track) < 1.5, "gap %.1f s: ratio %.2f (tracked %.2f)", gaps[g], ratio_ppm(), track);
        model_wire_cut(W, 0);
        run(40.0);
        dclock_stats_t c;
        dclock_get_stats(B.clk, &c);
        CHECKF(c.mode == DCLK_FOLLOWER && c.locked, "gap %.1f s: 40 s after the return mode %d locked %d", gaps[g], c.mode, c.locked);
        CHECKF(fabs(ratio_ppm() - track) < 1.0 && fabs(c.audio_phase) < 0.1, "gap %.1f s: ratio %.2f phase %.3f", gaps[g], ratio_ppm(),
               c.audio_phase);
        /* held through the gap and the return: only the dither's LSB moves */
        CHECKF(worst_rate(t0) <= 2.0 * LSB_PPM, "gap %.1f s: %.2f ppm in 1 s", gaps[g], worst_rate(t0));
        CHECKF(worst_phase(t0, DCLK_FOLLOWER) < 0.1, "gap %.1f s: phase %.3f samples", gaps[g], worst_phase(t0, DCLK_FOLLOWER));
    }
    uint32_t ms = max_step();
    CHECKF(ms <= cs2k_max_step(R0), "largest write %u LSB (limit %u)", ms, cs2k_max_step(R0));
    teardown();
}

static void test_go_leader(void)
{
    setup(1, -2);
    double track = settle();
    /* 20 s out: B's time of day (following its desk at +40 ppm) comes back 0.5 ms off, the PTP slews it; 60 s out:
     * 2 ms off, the PTP steps it (a media load) */
    for (int k = 0; k < 2; k++) {
        double outage = k ? 60.0 : 20.0;
        printf("- GO LEADER, %.0f s without the source: the ratio glides to R0 at <= 5 ppm/s, the time of day follows the "
               "desk\n", outage);
        model_wire_cut(W, 1);
        uint64_t t0 = vt;
        run(outage);
        CHECKF(mode() == DCLK_LEADER, "mode %d", mode());
        int a = first_off(t0, track, 2.0 * LSB_PPM), b = first_near(t0, 0.0, 0.0);
        CHECKF(a >= 0 && b > a, "left %d reached %d", a, b);
        if (a >= 0 && b > a) {
            double secs = (double)(S[b].t - S[a].t) * 1e-9;
            CHECKF(secs >= (fabs(track) - 2.0) / 5.0, "%.2f ppm in %.3f s", track, secs);
        }
        CHECKF(fabs(ratio_ppm()) < 1e-9, "ratio %.3f ppm", ratio_ppm());
        CHECKF(worst_rate(t0) <= 5.0 + 2.0 * LSB_PPM, "%.2f ppm in 1 s", worst_rate(t0));
        CHECKF(worst_phase(t0, DCLK_LEADER) < 0.1, "leader phase %.3f samples", worst_phase(t0, DCLK_LEADER));

        printf("- GO LEADER, the source back after %.0f s: B follows again, the ratio glides from R0 to the tracked one%s\n",
               outage, k ? " (time of day stepped)" : "");
        dclock_stats_t c;
        dclock_get_stats(B.clk, &c);
        uint64_t steps = c.steps;
        model_wire_cut(W, 0);
        uint64_t t1 = vt;
        run(150.0);
        dclock_get_stats(B.clk, &c);
        CHECKF(c.mode == DCLK_FOLLOWER && c.locked, "mode %d locked %d", c.mode, c.locked);
        CHECKF(k ? c.steps > steps : c.steps == steps, "time of day steps %llu -> %llu", (unsigned long long)steps,
               (unsigned long long)c.steps);
        CHECKF(fabs(ratio_ppm() - track) < 1.0 && fabs(c.audio_phase) < 0.1, "ratio %.2f (tracked %.2f) phase %.3f", ratio_ppm(), track,
               c.audio_phase);
        a = first_off(t1, 0.0, 2.0 * LSB_PPM);
        b = first_near(t1, track, 1.0);
        CHECKF(a >= 0 && b > a, "left %d reached %d", a, b);
        if (a >= 0 && b > a) {
            double secs = (double)(S[b].t - S[a].t) * 1e-9;
            CHECKF(secs >= (fabs(track) - 3.0) / 5.0, "%.2f ppm in %.3f s", track, secs);
        }
        CHECKF(worst_rate(t1) <= 5.0 + 2.0 * LSB_PPM, "%.2f ppm in 1 s", worst_rate(t1));
        /* the ratio glides back during the PTP's pull-in, the loop starts on a settled lock at the tracked ratio: no
         * phase to pull back */
        CHECKF(worst_phase(t1, DCLK_FOLLOWER) < 0.25, "phase %.3f samples while following", worst_phase(t1, DCLK_FOLLOWER));
    }
    uint32_t ms = max_step();
    CHECKF(ms <= cs2k_max_step(R0), "largest write %u LSB (limit %u)", ms, cs2k_max_step(R0));
    teardown();
}

static void test_clockfail_runtime(void)
{
    TEST("clockfail at run time: holdover -> leader glides to R0, leader -> holdover holds, the next loss obeys it");
    setup(0, -2);
    double track = settle();
    char out[16384];
    CHECK(ctl_exec(&B, "clockfail toaster", out, sizeof(out)) != 0 && strstr(out, "err"));
    model_wire_cut(W, 1);
    run(6.0);
    CHECKF(mode() == DCLK_HOLDOVER && fabs(ratio_ppm() - track) < 1.5, "mode %d ratio %.2f", mode(), ratio_ppm());
    CHECK(ctl_exec(&B, "clockfail leader", out, sizeof(out)) == 0 && strstr(out, "ok"));
    CHECKF(mode() == DCLK_LEADER, "mode %d", mode());
    uint64_t t0 = vt;
    run(12.0);
    CHECKF(fabs(ratio_ppm()) < 1e-9, "ratio %.3f ppm", ratio_ppm());
    CHECKF(worst_rate(t0) <= 5.0 + 2.0 * LSB_PPM, "%.2f ppm in 1 s", worst_rate(t0));
    CHECK(ctl_exec(&B, "status", out, sizeof(out)) == 0 && strstr(out, "clock_fail=leader"));
    CHECK(ctl_exec(&B, "clockfail holdover", out, sizeof(out)) == 0 && strstr(out, "ok"));
    run(2.0);
    CHECKF(mode() == DCLK_HOLDOVER && fabs(ratio_ppm()) < 1e-9, "mode %d ratio %.3f", mode(), ratio_ppm());
    model_wire_cut(W, 0);
    run(150.0);
    CHECKF(mode() == DCLK_FOLLOWER && fabs(ratio_ppm() - track) < 1.0, "mode %d ratio %.2f", mode(), ratio_ppm());
    CHECK(ctl_exec(&B, "clockfail master", out, sizeof(out)) == 0 && strstr(out, "ok"));   /* the old word */
    CHECKF(mode() == DCLK_FOLLOWER, "following: clockfail only sets what the next loss does (mode %d)", mode());
    model_wire_cut(W, 1);
    run(6.0);
    CHECKF(mode() == DCLK_LEADER, "the next loss: mode %d", mode());
    uint32_t ms = max_step();
    CHECKF(ms <= cs2k_max_step(R0), "largest write %u LSB (limit %u)", ms, cs2k_max_step(R0));
    teardown();
}

/* issue 008: a rate change in the tick in which the audio loop is due: the media load and the loop's snapshot come at
 * the same instant, so fs_phase's latch is still from before the load (MEDIA_TS on the old count). The loop must not
 * take it as its new reference */
static void test_rate_change_stale(void)
{
    TEST("48 -> 96 kHz in the audio loop's tick (fs_phase's latch from before the media load): the loop re-locks, the ratio stays");
    setup(0, -2);
    double track = settle();
    /* the loop's tick: its phase changes there; the next one is 125 ms later */
    dclock_stats_t c;
    dclock_get_stats(B.clk, &c);
    double ph = c.audio_phase;
    for (int i = 0; i < 200 && c.audio_phase == ph; i++) { run(0.001); dclock_get_stats(B.clk, &c); }
    run(0.124);
    uint64_t loads = c.media_loads;
    CHECK(port_set_rate(&A, 96000) == 0 && port_set_rate(&B, 96000) == 0);
    uint64_t t0 = vt;
    run(40.0);
    dclock_get_stats(B.clk, &c);
    double off = worst_off(t0, track);
    int sl = slips(t0);
    printf("  after the change: ratio up to %.3f ppm off the tracked %.2f, %d slips, phase %.4f samples, %llu media loads\n",
           off, track, sl, c.audio_phase, (unsigned long long)(c.media_loads - loads));
    CHECKF(c.media_loads > loads, "media loads %llu", (unsigned long long)c.media_loads);
    CHECKF(c.mode == DCLK_FOLLOWER && c.locked, "mode %d locked %d", c.mode, c.locked);
    CHECKF(off < 2.0, "ratio %.3f ppm off the tracked one", off);
    CHECKF(sl == 0, "%d slips", sl);
    CHECKF(fabs(c.audio_phase) < 0.05, "phase %.4f samples 40 s after the change", c.audio_phase);
    teardown();
}

int main(void)
{
    g_log_level = getenv("V") ? LOG_DEBUG : LOG_WARN;
    g_now_ns = vnow;
    rnd_seed(11);
    test_holdover_gaps(-2);
    test_holdover_gaps(1);
    test_go_leader();
    test_clockfail_runtime();
    test_rate_change_stale();
    return t_done("test_clock");
}
