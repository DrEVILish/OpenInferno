/* test_inproc.c - two inferno-portd ports in one process on two FPGA models joined by a datagram "wire" (5 us each
 * way), in virtual time: the PTPv1 election and the follower's lock (a +50 ppm, 3 s-off oscillator), mDNS discovery,
 * 64 channels subscribed through flow requests (4 flows of 16), the audio checked sample by sample by the models,
 * the reverse direction, renames, a multicast bundle, a latency change, a leader election between two would-be
 * leaders, the PI9696 48-channel pattern (6 flows of 8 each way). Part of OpenProSeries inferno-portd. Licence: GPL-3.0-or-later.
 */
#include "test.h"
#include "../src/clock.h"
#include "../src/ctl.h"
#include "../src/daemon.h"
#include "../src/fpga_model.h"
#include "../src/ptp.h"
#include "../src/regs.h"
#include "../src/util.h"

#include <math.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

static uint64_t vt = 5000000000ull;
static uint64_t vnow(void) { return vt; }

static port_t A, B;

static model_wire_t *W;
static uint32_t mk_caps;   /* INF_CAPS of the next model; 0 = the full build */
static hw_t *mk_jb(int side, int id, double ppm, uint64_t tod0, int jb)
{
    model_cfg_t m;
    memset(&m, 0, sizeof(m));
    m.id = id; m.xtal_ppm = ppm; m.maxlat = 3;
    m.port_base = 0x1000; m.clock_base = 0x1000; m.dclk_base = 0xFFFFFFFFu; m.cs2k_base = 0xFFFFFFFFu;
    m.phy_fd = -1; m.wire = W; m.wire_side = side; m.wire_delay_ns = 5000; m.tod_start_ns = tod0; m.jb_frames = jb;
    m.caps = mk_caps;
    return model_open(&m);
}
static hw_t *mk(int side, int id, double ppm, uint64_t tod0) { return mk_jb(side, id, ppm, tod0, 0); }

static void run(double seconds)
{
    uint64_t end = vt + (uint64_t)(seconds * 1e9);
    while (vt < end) {
        vt += 1000000ull;
        port_tick(&A, vt);
        port_tick(&B, vt);
    }
}

static int receiving(const port_t *p, int from, int to)
{
    int n = 0;
    for (int k = from; k < to; k++) n += p->sub[k].status == SUB_RX_UNICAST || p->sub[k].status == SUB_RX_MULTICAST;
    return n;
}

static int flows(const port_t *p, int tx)
{
    int n = 0;
    for (int f = 0; f < NRXF; f++) n += tx ? p->tx[f].used : p->rx[f].used;
    return n;
}

/* B plays from a 96-frame buffer at 0.5 ms (24 frames): A at 5 ms (240) cannot be played, 2 ms (96) just can */
static void latency_beyond_buffer(void)
{
    portd_stop(&A); A.hw->ops->close(A.hw);
    portd_stop(&B); B.hw->ops->close(B.hw);
    W = model_wire_new();
    port_defaults(&A);
    port_defaults(&B);
    A.n = 0; snprintf(A.desk, sizeof(A.desk), "FOH"); A.mac[5] = 0x31;
    B.n = 1; snprintf(B.desk, sizeof(B.desk), "MON"); B.mac[5] = 0x42;
    A.lat_mode = 3; B.lat_mode = 0;
    portd_opts_t oa = { 1, 0, 0, 0, 0, PTP_ROLE_LEADER, 1, -2, 0, 0, 0, 0 };
    portd_opts_t ob = { 1, 0, 0, 0, 0, PTP_ROLE_FOLLOWER, 1, -2, 0, 0, 0, 0 };
    CHECK(portd_start(&A, mk(0, 1, 0.0, 1760000000ull * 1000000000ull), &oa) == 0);
    CHECK(portd_start(&B, mk_jb(1, 2, 0.0, 1760000000ull * 1000000000ull, 96), &ob) == 0);
    run(2.0);

    TEST("a transmitter latency beyond the buffer: status 0x1A (channel latency), no flow requested, no renegotiation");
    sub_subscribe(&B, 0, "01", "FOH-1");
    int seen = 0;
    for (int i = 0; i < 30; i++) { run(0.5); seen += flows(&B, 0) + flows(&A, 1); }
    CHECKF(sub_status(&B, 0) == SUB_LATENCY, "status 0x%x", sub_status(&B, 0));
    CHECKF(seen == 0, "%d flow-half-seconds", seen);

    TEST("the boundary: A at 2 ms = the 96-frame buffer: subscribed with SHIFT 96 - 24 = 72");
    CHECK(port_set_latency(&A, 2) == 0);
    run(12.0);
    int f = B.sub[0].flow;
    CHECKF(f >= 0 && sub_status(&B, 0) != SUB_LATENCY, "flow %d status 0x%x", f, sub_status(&B, 0));
    if (f >= 0) CHECKF(B.rx[f].shift == 72 && tbl_read(B.hw, TBL_RX(f, 3)) == 72, "shift %d", B.rx[f].shift);

    TEST("a latency that fits: A at 1 ms: SHIFT 48 - 24 = 24, as before");
    sub_unsubscribe(&B, 0);
    run(2.0);
    CHECK(flows(&B, 0) == 0);
    CHECK(port_set_latency(&A, 1) == 0);
    sub_subscribe(&B, 0, "01", "FOH-1");
    run(4.0);
    f = B.sub[0].flow;
    CHECKF(f >= 0 && sub_status(&B, 0) != SUB_LATENCY, "flow %d status 0x%x", f, sub_status(&B, 0));
    if (f >= 0) CHECKF(B.rx[f].shift == 24 && tbl_read(B.hw, TBL_RX(f, 3)) == 24, "shift %d", B.rx[f].shift);
}

static int colsrc_ok(port_t *p, int k, int want_id, int want_col)
{
    int id, col;
    return model_colsrc(p->hw, k, &id, &col) && id == want_id && col == want_col;
}

/* fpga/docs/xc3s5000.md 8.11.21 (io build 35, XBAR = 1): RX SFD stamps 40 ns later, TX on the wire 20 ns after its
 * stamp; io builds 30 / 31 (LOG M423 / M438): RX +20 / +40 ns against build 29. A measure-only follower at 0 ppm,
 * X ns ahead of the leader, over a 5 us wire: offset X, delay 5000 ns when portd corrects the stamps */
#define STAMP_X 3123456780ll
static void stamp_case(const char *what, uint32_t rx_late, uint32_t tx_late, int xbar, int iobuild, uint32_t oiob,
                       double want_off, double want_delay, const char *want_corr, const char *want_warn)
{
    if (A.hw) { portd_stop(&A); A.hw->ops->close(A.hw); }
    if (B.hw) { portd_stop(&B); B.hw->ops->close(B.hw); }
    W = model_wire_new();
    port_defaults(&A);
    port_defaults(&B);
    A.n = 0; snprintf(A.desk, sizeof(A.desk), "FOH"); A.mac[5] = 0x11;
    B.n = 1; snprintf(B.desk, sizeof(B.desk), "MON"); B.mac[5] = 0x22;
    portd_opts_t oa = { 1, 0, 0, 0, 0, PTP_ROLE_LEADER, 1, -2, 0, 0, 0, 0 };
    portd_opts_t ob = { 0, 0, 0, 0, 0, PTP_ROLE_FOLLOWER, 0, -2, 0, 0, 0, 0 };
    CHECK(portd_start(&A, mk(0, 1, 0.0, 1760000000ull * 1000000000ull), &oa) == 0);
    model_cfg_t m;
    memset(&m, 0, sizeof(m));
    m.id = 2; m.maxlat = 3;
    m.port_base = 0x1000; m.clock_base = 0x1000; m.dclk_base = 0xFFFFFFFFu; m.cs2k_base = 0xFFFFFFFFu;
    m.phy_fd = -1; m.wire = W; m.wire_side = 1; m.wire_delay_ns = 5000;
    m.tod_start_ns = (uint64_t)(1760000000ll * 1000000000ll + STAMP_X);
    m.rx_stamp_late_ns = rx_late; m.tx_wire_late_ns = tx_late; m.xbar = xbar; m.iobuild = iobuild; m.oiob_info = oiob;
    /* B's start-up warnings, from stderr */
    char warn[4096] = "";
    FILE *cap = tmpfile();
    int save = dup(2);
    fflush(stderr);
    dup2(fileno(cap), 2);
    CHECK(portd_start(&B, model_open(&m), &ob) == 0);
    fflush(stderr);
    dup2(save, 2);
    close(save);
    rewind(cap);
    warn[fread(warn, 1, sizeof(warn) - 1, cap)] = 0;
    fclose(cap);
    fputs(warn, stderr);
    TEST(what);
    if (want_warn) CHECKF(strstr(warn, want_warn) != NULL, "no warning \"%s\"", want_warn);
    if (want_corr) {
        char out[8192];
        ctl_exec(&B, "status", out, sizeof(out));
        CHECKF(strstr(out, want_corr) != NULL, "no \"%s\" in status", want_corr);
    }
    run(12.0);
    ptp_stats_t s;
    ptp_get_stats(B.ptp, &s);
    double off = s.offset_ns - (double)STAMP_X;
    CHECKF(s.state == PTP_SLAVE && fabs(off - want_off) < 1.0 && fabs(s.delay_ns - want_delay) < 1.0,
           "offset X %+.1f ns (want %+.0f), delay %.1f ns (want %.0f)", off, want_off, s.delay_ns, want_delay);
}

static void xbar_stamps(void)
{
    stamp_case("SFD stamps: no offsets (io build 29): offset and delay exact", 0, 0, 0, 29, 0, 0.0, 5000.0, NULL, NULL);
    stamp_case("SFD stamps: XBAR + io build 32 pipeline, its IDs hidden: off by (80 - 20) / 2, delay + (80 + 20) / 2", 80, 20,
               0, 0, 0, 30.0, 5050.0, NULL, NULL);
    stamp_case("SFD stamps: io build 32 + XBAR (OXB1 at C0160, OFCP build 32 at C8900): RX -80, TX +20 ns: exact", 80, 20, 1,
               32, 0, 0.0, 5000.0, NULL, NULL);
    stamp_case("SFD stamps: io build 30: RX -20 ns: exact", 20, 0, 0, 30, 0, 0.0, 5000.0, NULL, NULL);
    /* OIOB (C01A0, INFO {1, IOBUILD, PORT_CH, flags}): the combined image without aes_fcap */
    stamp_case("SFD stamps: OIOB io build 36 + XBAR, no OFCP (INFO 0x0124300E): RX -80, TX +20 ns: exact", 80, 20, 1, 0,
               0x0124300Eu, 0.0, 5000.0, "stamp_corr=-80/+20\n", NULL);
    stamp_case("SFD stamps: OFCP only (io build 32, no OIOB, no XBAR): RX -40, TX 0: exact", 40, 0, 0, 32, 0, 0.0, 5000.0,
               "stamp_corr=-40/+0\n", NULL);
    stamp_case("SFD stamps: no OIOB, no OFCP (b32 pipeline): warns, stamp_corr unknown, no correction: off by 20 / 2, delay + 20",
               40, 0, 0, 0, 0, 20.0, 5020.0, "stamp_corr=unknown\n", "io build unknown");
    stamp_case("SFD stamps: OIOB says XBAR (0x0124300E), no OXB1: warns, corrects as OXB1: RX -40 ns: exact", 40, 0, 0, 0,
               0x0124300Eu, 0.0, 5000.0, "stamp_corr=-40/+0\n", "OXB1");
    stamp_case("SFD stamps: OIOB b30 overrides disagreeing OFCP b32", 20, 0, 0, 32,
               0x011E300Cu, 0.0, 5000.0, "stamp_corr=-20/+0\n", NULL);
    stamp_case("SFD stamps: unknown build still applies OXB1 correction", 40, 20, 1, 0,
               0, 0.0, 5000.0, "stamp_corr=unknown\n", "io build unknown");
    stamp_case("SFD stamps: OIOB no XBAR but OXB1 present", 80, 20, 1, 0,
               0x0124300Cu, 0.0, 5000.0, "stamp_corr=-80/+20\n", "OIOB XBAR flag 0 but OXB1");
}

/* desk b32 (2026-11-14): port 6 started as "--ptp follower" with no --owner / --steer followed a Dante leader 34 days
 * ahead (ptp_offset_ns -2938998389492930, ptp_servo 0) and was never stepped: a measure-only follower by design */
static const uint64_t TOD_L = 1760000000ull * 1000000000ull, DAYS34 = 2938998389492930ull;
static int pair(const portd_opts_t *ob)
{
    if (A.hw) { portd_stop(&A); A.hw->ops->close(A.hw); }
    if (B.hw) { portd_stop(&B); B.hw->ops->close(B.hw); }
    W = model_wire_new();
    port_defaults(&A);
    port_defaults(&B);
    A.n = 0; snprintf(A.desk, sizeof(A.desk), "DANTE"); A.mac[5] = 0x51;
    B.n = 5; snprintf(B.desk, sizeof(B.desk), "OpenProSeries"); B.mac[5] = 0x06;
    portd_opts_t oa = { 1, 0, 0, 0, 0, PTP_ROLE_LEADER, 1, -2, 0, 0, 0, 0 };
    CHECK(portd_start(&A, mk(0, 1, 0.0, TOD_L), &oa) == 0);
    hw_t *h = mk(1, 2, 2.8, TOD_L - DAYS34);
    if (portd_start(&B, h, ob)) { h->ops->close(h); B.hw = NULL; return -1; }
    return 0;
}

static void follower_days_off(void)
{
    ptp_stats_t sb;
    dclock_stats_t cb;
    TEST("desk b32 line (--ptp follower, no --owner / --steer), leader 34 days ahead: measured, never stepped");
    portd_opts_t desk = { 0, 0, 0, 0, 0, PTP_ROLE_FOLLOWER, 0, -2, 0, 0, 0, 0 };
    CHECK(pair(&desk) == 0);
    run(30.0);
    ptp_get_stats(B.ptp, &sb);
    dclock_get_stats(B.clk, &cb);
    CHECKF(sb.state == PTP_SLAVE && sb.offset_ns < -2.9e15 && sb.servo_state == 0 && cb.steps == 0,
           "%s offset %.0f servo %d steps %llu", ptp_state_name(sb.state), sb.offset_ns, sb.servo_state,
           (unsigned long long)cb.steps);

    TEST("--steer without --owner is refused (the servo's steps would be dropped: dclock_step needs the owner)");
    portd_opts_t noown = { 0, 0, 0, 0, 0, PTP_ROLE_FOLLOWER, 1, -2, 0, 0, 0, 0 };
    CHECK(pair(&noown) != 0);

    TEST("the fix on the desk (--clock-port 6 --owner --steer), leader 34 days ahead: steps and locks within 60 s");
    portd_opts_t own = { 1, 0, 0, 0, 0, PTP_ROLE_FOLLOWER, 1, -2, 0, 0, 0, 0 };
    CHECK(pair(&own) == 0);
    run(60.0);
    ptp_get_stats(B.ptp, &sb);
    dclock_get_stats(B.clk, &cb);
    CHECKF(sb.state == PTP_SLAVE && fabs(sb.offset_ns) < 1000.0 && cb.steps >= 1 && cb.locked,
           "offset %.0f ns steps %llu locked %d", sb.offset_ns, (unsigned long long)cb.steps, cb.locked);
    char out[256] = "";
    ctl_exec(&B, "ptp follower steer", out, sizeof(out));
    CHECKF(!strcmp(out, "ok\n"), "owner: ctl \"ptp follower steer\" answered \"%s\"", out);

    TEST("ctl \"ptp follower steer\" on a non-owner is refused, never a silent no-op");
    CHECK(pair(&desk) == 0);
    out[0] = 0;
    ctl_exec(&B, "ptp follower steer", out, sizeof(out));
    run(60.0);
    ptp_get_stats(B.ptp, &sb);
    dclock_get_stats(B.clk, &cb);
    CHECKF(!strncmp(out, "err", 3) && sb.servo_state == 0 && cb.steps == 0, "reply \"%s\" servo %d steps %llu", out,
           sb.servo_state, (unsigned long long)cb.steps);
}

/* clock design note 2026-11-14 section 4 (PR 4): INF_CTRL RX / TX enable stay off until the clock is ready (PTP locked,
 * |offset| < 5 us, held 1 s), and again on a re-acquire; a leader is ready at once */
static int enables(const port_t *p) { return (int)(port_rd(p->hw, INF_CTRL) & 3); }
static int ready_field(port_t *p)
{
    static char out[16384];
    out[0] = 0;
    ctl_exec(p, "status", out, sizeof(out));
    const char *f = strstr(out, "\nclock_ready=");
    return f ? atoi(f + 13) : -1;
}

/* run until B's enables come on (at most max_s): the first time of B's current good run (SLAVE, servo locked,
 * |offset| < 5 us), the unmute time, B's media frames played / packets sent while muted, B subscribed while muted */
static void until_unmuted(double max_s, double *t_good, double *t_on, uint64_t *leak, int *subs_muted)
{
    model_stats_t m0, m;
    model_get_stats(B.hw, &m0);
    *t_good = *t_on = -1;
    *leak = 0;
    *subs_muted = 0;
    for (uint64_t end = vt + (uint64_t)(max_s * 1e9); vt < end;) {
        run(0.001);
        ptp_stats_t s;
        ptp_get_stats(B.ptp, &s);
        int good = s.state == PTP_SLAVE && s.servo_state == SERVO_LOCKED && fabs(s.offset_ns) < 5000.0;
        if (!good) *t_good = -1;
        else if (*t_good < 0) *t_good = (double)vt * 1e-9;
        if (enables(&B)) { *t_on = (double)vt * 1e-9; return; }
        model_get_stats(B.hw, &m);
        *leak = (m.rx_media - m0.rx_media) + (m.tx_media - m0.tx_media);
        if (receiving(&B, 0, 8) == 8) *subs_muted = 1;
    }
}

static void mute_until_ready(void)
{
    portd_stop(&A); A.hw->ops->close(A.hw);
    portd_stop(&B); B.hw->ops->close(B.hw);
    W = model_wire_new();
    port_defaults(&A);
    port_defaults(&B);
    A.n = 0; snprintf(A.desk, sizeof(A.desk), "FOH"); A.mac[5] = 0x11;
    B.n = 1; snprintf(B.desk, sizeof(B.desk), "MON"); B.mac[5] = 0x22;
    A.lat_mode = 3; B.lat_mode = 3;
    portd_opts_t oa = { 1, 0, 0, 0, 0, PTP_ROLE_LEADER, 1, -2, 0, 0, 0, 0 };
    portd_opts_t ob = { 1, 0, 0, 0, 0, PTP_ROLE_FOLLOWER, 1, -2, 0, 0, 0, 0 };
    CHECK(portd_start(&A, mk(0, 1, 0.0, 1760000000ull * 1000000000ull), &oa) == 0);
    CHECK(portd_start(&B, mk(1, 2, 50.0, 1760000003ull * 1000000000ull + 123456789ull), &ob) == 0);

    TEST("mute gate: a leader (--ptp leader) is ready at once: INF_CTRL RX / TX on, clock_ready=1");
    run(0.002);
    CHECKF(enables(&A) == 3 && ready_field(&A) == 1, "A enables %d clock_ready %d", enables(&A), ready_field(&A));
    for (int k = 0; k < 8; k++) {
        char c[8];
        snprintf(c, sizeof(c), "%02d", k + 1);
        sub_subscribe(&B, k, c, "FOH-1");
        sub_subscribe(&A, k, c, "MON-2");
    }

    TEST("mute gate: the follower before ready: RX / TX off, clock_ready=0, no RX frame played, no TX packet sent");
    double tg, ton;
    uint64_t leak;
    int subs;
    CHECKF(enables(&B) == 0 && ready_field(&B) == 0, "B enables %d clock_ready %d", enables(&B), ready_field(&B));
    until_unmuted(40.0, &tg, &ton, &leak, &subs);
    CHECKF(ton > 0 && leak == 0, "unmuted at %.3f s, %llu media frames / packets while muted", ton, (unsigned long long)leak);

    TEST("mute gate: unmuted 1 s after the clock is ready (locked, |offset| < 5 us), then plays and sends");
    CHECKF(tg > 0 && ton - tg > 0.999 && ton - tg < 1.3, "good from %.3f s, unmuted %.3f s later", tg, ton - tg);
    CHECK(ready_field(&B) == 1);
    model_stats_t m0, m;
    model_get_stats(B.hw, &m0);
    run(3.0);
    model_get_stats(B.hw, &m);
    CHECKF(m.rx_media > m0.rx_media && m.tx_media > m0.tx_media && receiving(&B, 0, 8) == 8 && receiving(&A, 0, 8) == 8,
           "rx +%llu tx +%llu", (unsigned long long)(m.rx_media - m0.rx_media), (unsigned long long)(m.tx_media - m0.tx_media));

    TEST("mute gate: a forced re-acquire under traffic (ctl ptp follower steer) mutes at once: no RX played, no TX sent");
    char out[64] = "";
    ctl_exec(&B, "ptp follower steer", out, sizeof(out));
    run(0.002);
    CHECKF(!strcmp(out, "ok\n") && enables(&B) == 0 && ready_field(&B) == 0, "B enables %d clock_ready %d", enables(&B),
           ready_field(&B));
    until_unmuted(40.0, &tg, &ton, &leak, &subs);
    CHECKF(ton > 0 && leak == 0 && subs, "unmuted at %.3f s, %llu media frames / packets while muted, flows up while muted %d", ton,
           (unsigned long long)leak, subs);
    TEST("mute gate: after the re-acquire, unmuted 1 s after ready again");
    CHECKF(tg > 0 && ton - tg > 0.999 && ton - tg < 1.3, "unmuted %.3f s after good", ton - tg);
    TEST("D4 persistent leader phase fault: mute before reacquisition step, real packets resume");
    dclock_stats_t before, after;
    dclock_get_stats(B.clk, &before);
    uint64_t fault = vt;
    dclock_step(A.clk, 500000);
    for (int i=0; i<1000 && enables(&B); i++) run(.001);
    CHECK(enables(&B)==0);
    CHECK(vt-fault >= 500000000ull);
    uint64_t muted = vt;
    for (int i=0; i<1500; i++) {
        run(.001);
        dclock_get_stats(B.clk, &after);
        if(after.steps>before.steps) { CHECK(enables(&B)==0); break; }
    }
    CHECK(after.steps>before.steps);
    CHECK(after.ppb != 0);
    until_unmuted(60, &tg, &ton, &leak, &subs);
    CHECKF(ton>0 && leak==0, "return %.3f leak %llu",ton,(unsigned long long)leak);
    model_get_stats(B.hw, &m0); run(3); model_get_stats(B.hw, &m);
    CHECK(m.rx_media>m0.rx_media && m.tx_media>m0.tx_media);
    printf("fault mute %.3f s, resumed %.3f s after fault (actual model packets)\n",
           (muted-fault)*1e-9, ton-fault*1e-9);

    TEST("missing Sync cannot keep readiness on while the old 4 s source timer runs");
    model_wire_cut(W, 1); run(.6);
    CHECK(enables(&B)==0 && ready_field(&B)==0);
    model_wire_cut(W, 0); run(5);
    TEST("mute gate: the leader lost (no Sync for 4 s): muted");
    model_wire_cut(W, 1);
    run(5.0);
    CHECKF(enables(&B) == 0 && ready_field(&B) == 0, "B enables %d clock_ready %d", enables(&B), ready_field(&B));
}
/* The PI9696 pattern (Rust inferno, fork rev 5981fee, 48 channels; channels_subscriber.rs): it asks for flows of
 * min(nchan, 8) channels at fpp min(our fpp max, MTU), one socket per flow, and advertises MAX_CHANNELS_IN_FLOW = 8.
 * A = our 48-column port (0x10102030: 16 / 16 flows, 32 ch per flow); B stands in for PI9696 with 8 ch per flow
 * (0x10100830): B's subscriber asks A for 8-channel flows, and A must split its subscription to B into 8s. */
static int refused(const port_t *p)
{
    int n = 0;
    for (int k = 0; k < MAX_CH; k++) n += sub_status(p, k) == SUB_TOO_MANY || sub_status(p, k) == SUB_TX_FAIL;
    return n;
}

static void pi9696_48ch(void)
{
    portd_stop(&A); A.hw->ops->close(A.hw);
    portd_stop(&B); B.hw->ops->close(B.hw);
    W = model_wire_new();
    port_defaults(&A);
    port_defaults(&B);
    A.n = 0; snprintf(A.desk, sizeof(A.desk), "FOH"); A.mac[5] = 0x51;
    B.n = 1; snprintf(B.desk, sizeof(B.desk), "PI"); B.mac[5] = 0x62;
    A.lat_mode = 3; B.lat_mode = 3;
    portd_opts_t oa = { 1, 0, 0, 0, 0, PTP_ROLE_LEADER, 1, -2, 0, 0, 0, 0 };
    portd_opts_t ob = { 1, 0, 0, 0, 0, PTP_ROLE_FOLLOWER, 1, -2, 0, 0, 0, 0 };
    mk_caps = 0x10102030u;
    CHECK(portd_start(&A, mk(0, 1, 0.0, 1760000000ull * 1000000000ull), &oa) == 0);
    mk_caps = 0x10100830u;
    CHECK(portd_start(&B, mk(1, 2, 0.0, 1760000000ull * 1000000000ull), &ob) == 0);
    mk_caps = 0;
    CHECKF(A.channels == 48 && B.channels == 48 && A.caps_fch == 32 && B.caps_fch == 8, "A %d/%d B %d/%d", A.channels,
           A.caps_fch, B.channels, B.caps_fch);
    run(20.0);

    TEST("PI9696 48 ch: B RX 1..48 <- A TX 01..48 as 6 flows of 8 at 32 fpp; A RX 1..48 <- B TX 01..48 as 6 flows of 8");
    for (int k = 0; k < 48; k++) {
        char ch[8];
        snprintf(ch, sizeof(ch), "%02d", k + 1);
        sub_subscribe(&B, k, ch, "FOH-1");
        sub_subscribe(&A, k, ch, "PI-2");
    }
    int bad = 0;
    for (int i = 0; i < 100; i++) { run(0.1); bad += refused(&A) + refused(&B); }
    CHECKF(bad == 0, "%d refused channel-samples", bad);
    CHECKF(receiving(&B, 0, 48) == 48 && receiving(&A, 0, 48) == 48, "B %d A %d of 48", receiving(&B, 0, 48), receiving(&A, 0, 48));
    CHECKF(flows(&A, 1) == 6 && flows(&B, 0) == 6, "A TX %d, B RX %d flows", flows(&A, 1), flows(&B, 0));
    CHECKF(flows(&A, 0) == 6 && flows(&B, 1) == 6, "A RX %d, B TX %d flows", flows(&A, 0), flows(&B, 1));
    int txok = 0, cover = 0;
    for (int f = 0; f < NTXF; f++) {
        const txflow_t *t = &A.tx[f];
        if (!t->used) continue;
        int ok = t->programmed && t->nch == 8 && t->fpp == 32 && t->bps == 3 && t->ch[0] % 8 == 0;
        for (int c = 0; c < 8; c++) { ok &= t->ch[c] == t->ch[0] + c; if (t->ch[c] < 48) cover |= 1u << (t->ch[c] / 8); }
        txok += ok;
    }
    CHECKF(txok == 6 && cover == 0x3F, "%d A TX flows of 8 consecutive channels at fpp 32, blocks %02x", txok, cover);
    int rxok = 0;
    for (int f = 0; f < NRXF; f++) rxok += A.rx[f].used && A.rx[f].nch == 8 && A.rx[f].per == 8;
    CHECKF(rxok == 6, "%d A RX flows of 8", rxok);
    int ma = 0, mb = 0;
    for (int k = 0; k < 48; k++) { mb += colsrc_ok(&B, k, 1, k); ma += colsrc_ok(&A, k, 2, k); }
    CHECKF(mb == 48 && ma == 48, "columns carrying channel n: B %d, A %d of 48", mb, ma);
    /* Count a settled whole second: 6 flows * 1500 packets, 48 * 48000 samples each way. */
    model_stats_t before[2], after[2];
    model_get_stats(A.hw, &before[0]);
    model_get_stats(B.hw, &before[1]);
    run(1.0);
    model_get_stats(A.hw, &after[0]);
    model_get_stats(B.hw, &after[1]);
    for (int side = 0; side < 2; side++) {
        const model_stats_t *b = &before[side], *a = &after[side];
        CHECKF(a->tx_media - b->tx_media == 9000 && a->rx_media - b->rx_media == 9000,
               "side %d: TX %llu RX %llu packets (want 9000)", side,
               (unsigned long long)(a->tx_media - b->tx_media), (unsigned long long)(a->rx_media - b->rx_media));
        CHECKF(a->pat_ok - b->pat_ok == 48 * 48000 && a->pat_bad == b->pat_bad &&
               a->rx_bad == b->rx_bad && a->rx_late == b->rx_late && a->rx_early == b->rx_early,
               "side %d: %llu correct samples (want 2304000), bad/late/early unchanged", side,
               (unsigned long long)(a->pat_ok - b->pat_ok));
    }
    model_stats_t sa, sb;
    model_get_stats(A.hw, &sa);
    model_get_stats(B.hw, &sb);
    CHECKF(sb.pat_ok > 48 * 48000 && sb.pat_bad == 0 && sa.pat_ok > 48 * 48000 && sa.pat_bad == 0,
           "pattern B ok %llu bad %llu, A ok %llu bad %llu", (unsigned long long)sb.pat_ok, (unsigned long long)sb.pat_bad,
           (unsigned long long)sa.pat_ok, (unsigned long long)sa.pat_bad);
}

/* clock design note 2026-11-14 section 6 (PR 7): the live clock mode switch, "ctl clockmode leader|follow" (Desk leads | Sync to Dante), a saved
 * setting. B = the desk on the io (--owner, no audio loop / CS2000: the NCO is the desk's pitch), A = a Dante device
 * (auto: the desk's clock follower in Desk leads, the clock leader in Sync to Dante; --ready-us 100 as a device with pull tolerance,
 * crystal -8.5 ppm, the T1 figure). Every 125 ms B's NCO rate is logged; every 1 ms B's INF_CTRL enables */
#define CM_N 8192
static double cm_ppb[CM_N];
static int cm_n, cm_muted_ms;
static double cm_trim;   /* B's --trim-ppm */
static char cm_dir[] = "/tmp/openinferno-clockmode-XXXXXX";

static void cm_run(double seconds)
{
    for (uint64_t end = vt + (uint64_t)(seconds * 1e9); vt < end;) {
        run(0.001);
        cm_muted_ms += enables(&B) != 3;
        if (vt % 125000000ull == 0 && cm_n < CM_N) {
            dclock_stats_t c;
            dclock_get_stats(B.clk, &c);
            cm_ppb[cm_n++] = c.ppb;
        }
    }
}
/* the most B's NCO moved in any 1 s (8 samples) of samples i0..cm_n-1 (ppb) */
static double cm_slew(int i0)
{
    double w = 0;
    for (int i = i0 + 8; i < cm_n; i++) w = fmax(w, fabs(cm_ppb[i] - cm_ppb[i - 8]));
    return w;
}
static char cm_out[256];
static int cm_ctl(const char *cmd)
{
    cm_out[0] = 0;
    ctl_exec(&B, cmd, cm_out, sizeof(cm_out));
    return !strcmp(cm_out, "ok\n") ? 0 : !strncmp(cm_out, "err ", 4) ? -1 : -2;
}
/* B started with these launch flags on the state file cm_dir/state (A kept running when keep_a) */
static void cm_start_b(int owner, int role, int steer, int keep_a)
{
    if (!keep_a) {
        if (A.hw) { portd_stop(&A); A.hw->ops->close(A.hw); }
        W = model_wire_new();
        port_defaults(&A);
        A.n = 0; snprintf(A.desk, sizeof(A.desk), "DANTE"); A.mac[5] = 0x51; A.lat_mode = 3;
        portd_opts_t oa = { 1, 0, 0, 0, 0, PTP_ROLE_AUTO, 1, -2, 0, 0, 100, 0 };
        CHECK(portd_start(&A, mk(0, 1, -8.5, TOD_L + 3123456789ull), &oa) == 0);
    }
    if (B.hw) { portd_stop(&B); B.hw->ops->close(B.hw); }
    port_defaults(&B);
    B.n = 5; snprintf(B.desk, sizeof(B.desk), "OpenProSeries"); B.mac[5] = 0x06; B.lat_mode = 3;   /* 5 ms: the model delivers on 1 ms ticks */
    snprintf(B.state_path, sizeof(B.state_path), "%s/state", cm_dir);
    portd_opts_t ob = { owner, 0, 0, 0, 0, role, steer, -2, 0, 0, 0, cm_trim };
    CHECK(portd_start(&B, mk(1, 2, 0.0, TOD_L), &ob) == 0);
    for (int k = 0; k < 8; k++) {
        char c[8];
        snprintf(c, sizeof(c), "%02d", k + 1);
        sub_subscribe(&B, k, c, "DANTE-1");
        if (!keep_a) sub_subscribe(&A, k, c, "OpenProSeries-6");
    }
}
/* B's status line clockmode=WORD: 1 when it says want; with want NULL, 1 when there is no clockmode line */
static int cm_status(const char *want)
{
    static char out[16384];
    out[0] = 0;
    ctl_exec(&B, "status", out, sizeof(out));
    const char *f = strstr(out, "\nclockmode=");
    char w[16] = "";
    if (f) sscanf(f + 11, "%15s", w);
    return want ? !strcmp(w, want) : !f;
}
static int cm_state(const port_t *p) { ptp_stats_t s; ptp_get_stats(p->ptp, &s); return s.state; }
static int cm_saved(const char *want)
{
    char line[256];
    int got = 0;
    FILE *f = fopen(B.state_path, "r");
    while (f && fgets(line, sizeof(line), f)) got |= !strcmp(line, want);
    if (f) fclose(f);
    return got;
}

/* L -> F: muted at the command (no frame played or sent while muted), re-locked and unmuted under the #36 gate; then
 * 20 s: never muted, B's RX late / early / pattern errors +0, steps and media loads +0; the NCO <= 5 ppm/s throughout */
static int cm_to_follow(int cycle)
{
    int i0 = cm_n, ok = cm_ctl("clockmode follow") == 0;
    run(0.001);
    int muted_at_once = enables(&B) == 0;
    model_stats_t m0, m;
    model_get_stats(B.hw, &m0);
    uint64_t t0 = vt, leak = 0;
    while (enables(&B) != 3 && vt - t0 < 60000000000ull) {
        cm_run(0.001);
        model_get_stats(B.hw, &m);
        leak = (m.rx_media - m0.rx_media) + (m.tx_media - m0.tx_media);
    }
    double ready_s = (double)(vt - t0) * 1e-9;
    dclock_stats_t c0, c;
    dclock_get_stats(B.clk, &c0);
    model_get_stats(B.hw, &m0);
    cm_muted_ms = 0;
    cm_run(20.0);
    dclock_get_stats(B.clk, &c);
    model_get_stats(B.hw, &m);
    uint64_t slips = (m.rx_late - m0.rx_late) + (m.rx_early - m0.rx_early) + (m.pat_bad - m0.pat_bad);
    double slew = cm_slew(i0);
    int pass = ok && muted_at_once && leak == 0 && ready_s < 60.0 && cm_state(&B) == PTP_SLAVE && cm_muted_ms == 0 &&
               slips == 0 && m.rx_media > m0.rx_media && c.steps == c0.steps && c.media_loads == c0.media_loads &&
               slew <= 5000.000001;
    CHECKF(pass, "L->F %d: reply ok %d, muted at once %d, %llu frames while muted, ready %.1f s, B %s, muted %d ms after "
           "ready, slips %llu (rx +%llu), steps +%llu loads +%llu after ready, NCO slew %.1f ppb/s", cycle, ok, muted_at_once,
           (unsigned long long)leak, ready_s, ptp_state_name(cm_state(&B)), cm_muted_ms, (unsigned long long)slips,
           (unsigned long long)(m.rx_media - m0.rx_media), (unsigned long long)(c.steps - c0.steps),
           (unsigned long long)(c.media_loads - c0.media_loads), slew);
    printf("  L->F %d: muted %.1f s until ready, NCO slew <= %.0f ppb/s, 0 frames while muted: %d, slips after ready %llu\n", cycle,
           ready_s, slew, leak == 0, (unsigned long long)slips);
    return pass;
}

/* F -> L: never muted, no ToD step (steps and media loads +0), RX late / early / pattern errors +0, the NCO <= 1 ppm/s
 * and at the --trim-ppm value (nominal without one) within 30 s, B leads and A follows it */
static int cm_to_lead(int cycle)
{
    int i0 = cm_n;
    dclock_stats_t c0, c;
    model_stats_t m0, m;
    dclock_get_stats(B.clk, &c0);
    model_get_stats(B.hw, &m0);
    double from = c0.ppb;
    cm_muted_ms = 0;
    int ok = cm_ctl("clockmode leader") == 0;
    cm_run(30.0);
    dclock_get_stats(B.clk, &c);
    model_get_stats(B.hw, &m);
    uint64_t slips = (m.rx_late - m0.rx_late) + (m.rx_early - m0.rx_early) + (m.pat_bad - m0.pat_bad);
    double slew = cm_slew(i0 > 8 ? i0 - 8 : 0);
    int pass = ok && cm_muted_ms == 0 && c.steps == c0.steps && c.media_loads == c0.media_loads && slips == 0 &&
               m.rx_media > m0.rx_media && slew <= 1000.000001 && fabs(c.ppb - cm_trim * 1000.0) < 1.0 && cm_state(&B) == PTP_MASTER &&
               cm_state(&A) == PTP_SLAVE;
    CHECKF(pass, "F->L %d: reply ok %d, muted %d ms, steps +%llu loads +%llu, slips %llu (rx +%llu), NCO slew %.1f ppb/s, "
           "%.1f -> %.1f ppb, B %s A %s", cycle, ok, cm_muted_ms, (unsigned long long)(c.steps - c0.steps),
           (unsigned long long)(c.media_loads - c0.media_loads), (unsigned long long)slips,
           (unsigned long long)(m.rx_media - m0.rx_media), slew, from, c.ppb, ptp_state_name(cm_state(&B)),
           ptp_state_name(cm_state(&A)));
    printf("  F->L %d: NCO %.0f -> %.1f ppb, slew <= %.0f ppb/s, muted %d ms, steps +%llu, slips %llu\n", cycle, from, c.ppb, slew,
           cm_muted_ms, (unsigned long long)(c.steps - c0.steps), (unsigned long long)slips);
    return pass;
}

static void clockmode_switch(void)
{
    if (B.hw) { portd_stop(&B); B.hw->ops->close(B.hw); B.hw = NULL; }
    CHECK(mkdtemp(cm_dir) != NULL);
    cm_n = 0;
    cm_start_b(1, PTP_ROLE_LEADER, 0, 0);   /* Desk leads: --ptp leader --owner */
    cm_run(30.0);
    TEST("clockmode: Desk leads start: B is the clock leader, A (Dante, -8.5 ppm, 3 s off) is its clock follower");
    CHECKF(cm_state(&B) == PTP_MASTER && cm_state(&A) == PTP_SLAVE && ready_field(&A) == 1, "B %s A %s",
           ptp_state_name(cm_state(&B)), ptp_state_name(cm_state(&A)));

    TEST("clockmode: a repeated \"clockmode leader\" in Desk leads is a no-op (ok, no re-election, never muted)");
    cm_muted_ms = 0;
    CHECK(cm_ctl("clockmode leader") == 0);
    cm_run(0.5);
    CHECKF(cm_state(&B) == PTP_MASTER && cm_muted_ms == 0, "B %s, muted %d ms", ptp_state_name(cm_state(&B)), cm_muted_ms);

    TEST("clockmode: live Desk leads -> Sync to Dante (L -> F) and back (F -> L), 5 / 5 each way: L -> F muted only until ready, 0 slips after ready, NCO <= 5 "
         "ppm/s; F -> L never muted, no ToD step, 0 slips, NCO <= 1 ppm/s back to nominal");
    int lf = 0, fl = 0, i_all = cm_n, st_l = cm_status("leader"), st_f = 1;
    for (int k = 1; k <= 5; k++) {
        lf += cm_to_follow(k);
        st_f &= cm_status("follow");
        fl += cm_to_lead(k);
        st_l &= cm_status("leader");
    }
    CHECKF(lf == 5 && fl == 5, "L->F %d / 5, F->L %d / 5", lf, fl);
    CHECKF(cm_slew(i_all) <= 5000.000001, "NCO slew %.1f ppb/s over every switch", cm_slew(i_all));
    TEST("status: clockmode=leader in Desk leads (at start, after each clockmode leader), follow after each clockmode follow");
    CHECKF(st_l && st_f, "status clockmode: leader %d follow %d", st_l, st_f);

    TEST("clockmode: a repeated \"clockmode follow\" in Sync to Dante is a no-op (ok, still locked, never muted, no step)");
    cm_to_follow(6);
    dclock_stats_t c0, c;
    dclock_get_stats(B.clk, &c0);
    cm_muted_ms = 0;
    CHECK(cm_ctl("clockmode follow") == 0);
    cm_run(2.0);
    dclock_get_stats(B.clk, &c);
    CHECKF(cm_state(&B) == PTP_SLAVE && cm_muted_ms == 0 && c.locked && c.steps == c0.steps, "B %s muted %d ms locked %d steps +%llu",
           ptp_state_name(cm_state(&B)), cm_muted_ms, c.locked, (unsigned long long)(c.steps - c0.steps));

    TEST("clockmode: the setting is saved and restored at start (follow over the Desk leads flags, leader over the Sync to Dante flags)");
    cm_run(1.5);   /* past the 1 s save */
    CHECK(cm_saved("clockmode follow\n"));
    cm_start_b(1, PTP_ROLE_LEADER, 0, 1);   /* restart with the Desk leads flags */
    cm_run(40.0);
    CHECKF(cm_state(&B) == PTP_SLAVE && ready_field(&B) == 1, "after restart B %s ready %d", ptp_state_name(cm_state(&B)),
           ready_field(&B));
    CHECKF(cm_status("follow"), "status clockmode after the restart into the saved follow (Desk leads flags)");
    CHECK(cm_ctl("clockmode leader") == 0);
    cm_run(10.0);
    CHECK(cm_saved("clockmode leader\n"));
    cm_start_b(1, PTP_ROLE_FOLLOWER, 1, 1);   /* restart with the Sync to Dante flags */
    cm_run(10.0);
    CHECKF(cm_state(&B) == PTP_MASTER && ready_field(&B) == 1, "after restart B %s ready %d", ptp_state_name(cm_state(&B)),
           ready_field(&B));
    CHECKF(cm_status("leader"), "status clockmode after the restart into the saved leader (Sync to Dante flags)");

    TEST("clockmode: refused with an error reply without --owner (both ways) and for a bad word; the role is unchanged");
    char st[300];
    snprintf(st, sizeof(st), "%s/state", cm_dir);
    B.state_path[0] = 0;   /* this stop saves nothing: B starts from no saved mode */
    unlink(st);
    cm_start_b(0, PTP_ROLE_LEADER, 0, 1);
    cm_run(6.0);
    int f = cm_ctl("clockmode follow") == -1 && strstr(cm_out, "--owner");
    int l = cm_ctl("clockmode leader") == -1 && strstr(cm_out, "--owner");
    int x = cm_ctl("clockmode sideways") == -1 && strstr(cm_out, "leader or follow");
    cm_run(2.0);
    CHECKF(cm_status(NULL), "status: no clockmode line without --owner");
    CHECKF(f && l && x && cm_state(&B) == PTP_MASTER, "refused: follow %d leader %d bad word %d, B %s", f, l, x,
           ptp_state_name(cm_state(&B)));
    B.state_path[0] = 0;
    unlink(st);
    rmdir(cm_dir);
}

/* clock PR 8: B launched Desk leads with --trim-ppm -20 (A as in clockmode_switch: crystal -8.5 ppm, B's 0) */
static void clockmode_trim(void)
{
    strcpy(cm_dir, "/tmp/openinferno-clockmode-XXXXXX");
    CHECK(mkdtemp(cm_dir) != NULL);
    cm_n = 0;
    cm_trim = -20.0;
    cm_start_b(1, PTP_ROLE_LEADER, 0, 0);
    cm_run(40.0);
    dclock_stats_t b, a;
    dclock_get_stats(B.clk, &b);
    dclock_get_stats(A.clk, &a);
    TEST("--trim-ppm -20: the start glides the NCO from nominal to the trim (no jump, <= 5 ppm/s: D4's NCO limit)");
    CHECKF(fabs(cm_ppb[0]) <= 250.000001 && cm_slew(0) <= 5000.000001 && fabs(b.ppb + 20000.0) < 1.0,
           "first sample %.1f ppb, slew %.1f ppb/s, now %.1f ppb", cm_ppb[0], cm_slew(0), b.ppb);
    TEST("--trim-ppm -20: the clock leader's Syncs carry the trimmed rate (A, crystal -8.5 ppm, locks at (1 - 20e-6) / (1 - 8.5e-6))");
    /* A pulls in after B's glide (D4: lock < 60 s), then its frequency, 10 s averaged, is the trimmed rate seen from A */
    double want = ((1.0 - 20e-6) / (1.0 - 8.5e-6) - 1.0) * 1e9, avg = 0;
    uint64_t t_lock = vt;
    while (!a.locked && vt - t_lock < 90000000000ull) { cm_run(1.0); dclock_get_stats(A.clk, &a); }
    double lock_s = (double)(vt - t_lock) * 1e-9;
    cm_run(5.0);
    for (int k = 0; k < 10; k++) { cm_run(1.0); dclock_get_stats(A.clk, &a); avg += a.ppb * 0.1; }
    CHECKF(cm_state(&B) == PTP_MASTER && cm_state(&A) == PTP_SLAVE && a.locked && fabs(avg - want) < 200.0,
           "B %s A %s locked %d (%.0f s after the glide), A %.1f ppb (10 s average) want %.1f", ptp_state_name(cm_state(&B)),
           ptp_state_name(cm_state(&A)), a.locked, lock_s, avg, want);
    printf("  A locked %.0f s after B's start glide, at %.1f ppb (want %.1f)\n", lock_s, avg, want);
    TEST("--trim-ppm -20: Sync to Dante ignores the trim (B follows A); F -> L glides <= 1 ppm/s to the trim, not to nominal");
    int lf = cm_to_follow(1);
    dclock_get_stats(B.clk, &b);
    double followed = b.ppb;
    int fl = cm_to_lead(1);
    dclock_get_stats(B.clk, &b);
    CHECKF(lf && fl && fabs(followed + 20000.0) > 5000.0 && fabs(b.ppb + 20000.0) < 1.0,
           "L->F %d F->L %d, followed %.1f ppb, after F->L %.1f ppb", lf, fl, followed, b.ppb);
    char st[300];
    snprintf(st, sizeof(st), "%s/state", cm_dir);
    B.state_path[0] = 0;
    unlink(st);
    rmdir(cm_dir);
    cm_trim = 0;
}

int main(void)
{
    g_log_level = getenv("V") ? LOG_DEBUG : LOG_WARN;
    g_now_ns = vnow;
    rnd_seed(7);
    W = model_wire_new();
    port_defaults(&A);
    port_defaults(&B);
    A.n = 0; snprintf(A.desk, sizeof(A.desk), "FOH"); A.mac[5] = 0x11;
    B.n = 1; snprintf(B.desk, sizeof(B.desk), "MON"); B.mac[5] = 0x22;
    A.lat_mode = 3; B.lat_mode = 3;   /* 5 ms */
    portd_opts_t oa = { 1, 0, 0, 0, 0, PTP_ROLE_LEADER, 1, -2, 0, 0, 0, 0 };
    portd_opts_t ob = { 1, 0, 0, 0, 0, PTP_ROLE_FOLLOWER, 1, -2, 0, 0, 0, 0 };
    CHECK(portd_start(&A, mk(0, 1, 0.0, 1760000000ull * 1000000000ull), &oa) == 0);
    CHECK(portd_start(&B, mk(1, 2, 50.0, 1760000003ull * 1000000000ull + 123456789ull), &ob) == 0);

    TEST("link-local addresses (RFC 3927 probes), mDNS adverts");
    run(2.0);
    CHECK(net_ready(A.net) && net_ready(B.net) && net_ip(A.net) != net_ip(B.net));
    CHECK((net_ip(A.net) >> 16) == 0xA9FE);

    TEST("PTPv1: A leads (preferred), B follows, steps 3 s, locks: |offset| < 100 ns, rate -50 ppm");
    run(60.0);
    ptp_stats_t sa, sb;
    ptp_get_stats(A.ptp, &sa);
    ptp_get_stats(B.ptp, &sb);
    CHECKF(sa.state == PTP_MASTER && sb.state == PTP_SLAVE, "A %s B %s", ptp_state_name(sa.state), ptp_state_name(sb.state));
    CHECK(sa.syncs_tx > 100 && sa.fups_tx > 100 && sa.dresp_tx > 10 && sb.dresp_rx > 10);
    dclock_stats_t cb;
    dclock_get_stats(B.clk, &cb);
    CHECKF(fabs(sb.offset_ns) < 100.0 && cb.steps >= 1 && cb.locked, "offset %.1f ns, steps %llu, locked %d", sb.offset_ns,
           (unsigned long long)cb.steps, cb.locked);
    CHECKF(fabs(cb.ppb + 50000.0 / 1.00005) < 50.0, "ppb %.1f", cb.ppb);
    CHECKF(fabs(sb.delay_ns - 5000.0) < 300.0, "delay %.1f ns", sb.delay_ns);
    model_stats_t ma, mb;
    model_get_stats(A.hw, &ma);
    model_get_stats(B.hw, &mb);
    CHECKF(fabs(ma.tod_s - mb.tod_s) < 1e-6, "tod A - B = %.0f ns", (ma.tod_s - mb.tod_s) * 1e9);
    CHECKF(fabs(ma.media_pos - mb.media_pos) < 2.0, "media A %.1f B %.1f", ma.media_pos, mb.media_pos);

    TEST("64 channels: B RX 1..64 <- A TX 01..64 (mDNS, flow requests 0x0100 of 16 channels, the models' pattern)");
    for (int k = 0; k < 64; k++) {
        char ch[8];
        snprintf(ch, sizeof(ch), "%02d", k + 1);
        sub_subscribe(&B, k, ch, "FOH-1");
    }
    run(8.0);
    int nf = 0;
    for (int f = 0; f < NRXF; f++) nf += B.rx[f].used;
    CHECKF(receiving(&B, 0, 64) == 64, "%d of 64 receiving", receiving(&B, 0, 64));
    CHECKF(nf == 4, "%d RX flows", nf);
    int ntx = 0;
    for (int f = 0; f < NTXF; f++) ntx += A.tx[f].used && A.tx[f].programmed;
    CHECK(ntx == 4);
    model_get_stats(B.hw, &mb);
    CHECKF(mb.pat_ok > 64 * 48000 * 2 && mb.pat_bad == 0 && mb.rx_late == 0 && mb.rx_early == 0, "pattern ok %llu bad %llu late %llu early %llu",
           (unsigned long long)mb.pat_ok, (unsigned long long)mb.pat_bad, (unsigned long long)mb.rx_late, (unsigned long long)mb.rx_early);
    int map_ok = 0;
    for (int k = 0; k < 64; k++) map_ok += colsrc_ok(&B, k, 1, k);
    CHECKF(map_ok == 64, "%d columns carry their channel", map_ok);
    CHECKF(A.tx[0].fpp == 16, "fpp %d (5 ms: 32, cut to 16 by the MTU for 16 x 24 bit)", A.tx[0].fpp);

    TEST("the reverse way: A RX 1..8 <- B TX 33..40; keepalives keep B's flows alive past 4 s");
    for (int k = 0; k < 8; k++) {
        char ch[8];
        snprintf(ch, sizeof(ch), "%02d", 33 + k);
        sub_subscribe(&A, k, ch, "MON-2");
    }
    run(10.0);
    CHECKF(receiving(&A, 0, 8) == 8, "%d of 8", receiving(&A, 0, 8));
    int mok = 0;
    for (int k = 0; k < 8; k++) mok += colsrc_ok(&A, k, 2, 32 + k);
    CHECK(mok == 8);
    CHECK(receiving(&B, 0, 64) == 64);

    TEST("a TX rename on A: B resubscribes by the new name; an unsubscribe frees the slot");
    port_rename_tx(&A, 0, "Kick");
    sub_unsubscribe(&B, 0);
    sub_subscribe(&B, 0, "Kick", "FOH-1");
    run(4.0);
    CHECK(B.sub[0].status == SUB_RX_UNICAST && colsrc_ok(&B, 0, 1, 0));

    TEST("a multicast bundle: A TX flow 16 = channels 49..56 to 239.255.x.y:4321; B RX 57..64 re-subscribed take it");
    int16_t ch[8];
    for (int k = 0; k < 8; k++) ch[k] = (int16_t)(48 + k);
    CHECK(txflow_create_mcast(&A, 15, ch, 8) == 0);
    run(3.0);
    CHECK(A.tx[15].mc_state == 1 && (A.tx[15].dst_ip >> 16) == ((239u << 8) | 255u));
    for (int k = 56; k < 64; k++) {
        char c[8];
        snprintf(c, sizeof(c), "%02d", k - 7);
        sub_subscribe(&B, k, c, "FOH-1");
    }
    run(5.0);
    int mc = 0;
    for (int k = 56; k < 64; k++) mc += B.sub[k].status == SUB_RX_MULTICAST && colsrc_ok(&B, k, 1, k - 8);
    CHECKF(mc == 8, "%d of 8 on the bundle", mc);

    TEST("latency 1 ms on A: mDNS re-announced, automatic fpp 16");
    char out[16384];
    ctl_exec(&A, "latency 1", out, sizeof(out));
    CHECK(A.lat_mode == 1 && ((port_rd(A.hw, INF_LAT_STATUS) >> 16) & 63) == 16 && strstr(out, "ok"));
    run(2.0);
    CHECK(receiving(&B, 0, 64) == 64);
    ctl_exec(&B, "status", out, sizeof(out));
    CHECK(strstr(out, "ptp_state=SLAVE") && strstr(out, "subs_receiving=64"));

    TEST("an unsubscribed flow is stopped (0x0101) and A's TX flow goes away");
    for (int k = 0; k < 16; k++) sub_unsubscribe(&B, k);
    run(3.0);
    int left = 0;
    for (int f = 0; f < NTXF; f++) left += A.tx[f].used && !A.tx[f].multicast;
    CHECKF(left == 3, "%d unicast TX flows left", left);

    TEST("election: two would-be leaders (auto) -> the lower UUID leads, the other follows");
    ptp_set_role(A.ptp, PTP_ROLE_AUTO, 0);
    ptp_set_role(B.ptp, PTP_ROLE_AUTO, 1);
    run(15.0);
    ptp_get_stats(A.ptp, &sa);
    ptp_get_stats(B.ptp, &sb);
    CHECKF(sa.state == PTP_MASTER && sb.state == PTP_SLAVE, "A %s B %s", ptp_state_name(sa.state), ptp_state_name(sb.state));
    TEST("both peers renegotiate active subscriptions after 48 -> 96 -> 48 kHz");
    CHECK(port_set_rate(&A,96000) == 0); CHECK(port_set_rate(&B,96000) == 0);
    for (int k = 0; k < 8; k++) {
        char name[8]; snprintf(name,sizeof(name), "%02d",k + 1);
        CHECK(sub_subscribe(&A,k,name,"MON-2") == 0);
    }
    run(15.0);
    CHECKF(receiving(&B,16,32) == 16, "96 kHz: %d of 16",receiving(&B,16,32));
    CHECKF(receiving(&A,0,8) == 8, "96 kHz reverse: %d of 8",receiving(&A,0,8));
    CHECK(colsrc_ok(&B,16,1,16));
    CHECK(port_set_rate(&A,48000) == 0); CHECK(port_set_rate(&B,48000) == 0);
    run(15.0);
    CHECKF(receiving(&B,16,32) == 16, "48 kHz: %d of 16",receiving(&B,16,32));
    CHECKF(receiving(&A,0,8) == 8, "48 kHz reverse: %d of 8",receiving(&A,0,8));
    CHECK(colsrc_ok(&A,0,2,0));
    latency_beyond_buffer();
    mute_until_ready();
    xbar_stamps();
    follower_days_off();
    pi9696_48ch();
    clockmode_switch();
    clockmode_trim();
    return t_done("test_inproc");
}
