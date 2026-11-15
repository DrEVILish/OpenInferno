/* test_inproc.c - two inferno-portd ports in one process on two FPGA models joined by a datagram "wire" (5 us each
 * way), in virtual time: the PTPv1 election and the follower's lock (a +50 ppm, 3 s-off oscillator), mDNS discovery,
 * 64 channels subscribed through flow requests (4 flows of 16), the audio checked sample by sample by the models,
 * the reverse direction, renames, a multicast bundle, a latency change, a leader election between two would-be
 * leaders. Part of OpenProSeries inferno-portd. Licence: GPL-3.0-or-later.
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

static uint64_t vt = 5000000000ull;
static uint64_t vnow(void) { return vt; }

static port_t A, B;

static model_wire_t *W;
static hw_t *mk_jb(int side, int id, double ppm, uint64_t tod0, int jb)
{
    model_cfg_t m;
    memset(&m, 0, sizeof(m));
    m.id = id; m.xtal_ppm = ppm; m.maxlat = 3;
    m.port_base = 0x1000; m.clock_base = 0x1000; m.dclk_base = 0xFFFFFFFFu; m.cs2k_base = 0xFFFFFFFFu;
    m.phy_fd = -1; m.wire = W; m.wire_side = side; m.wire_delay_ns = 5000; m.tod_start_ns = tod0; m.jb_frames = jb;
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
    portd_opts_t oa = { 1, 0, 0, 0, 0, PTP_ROLE_LEADER, 1, -2, 0, 0, 0 };
    portd_opts_t ob = { 1, 0, 0, 0, 0, PTP_ROLE_FOLLOWER, 1, -2, 0, 0, 0 };
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
static void stamp_case(const char *what, uint32_t rx_late, uint32_t tx_late, int xbar, int iobuild, double want_off,
                       double want_delay)
{
    if (A.hw) { portd_stop(&A); A.hw->ops->close(A.hw); }
    if (B.hw) { portd_stop(&B); B.hw->ops->close(B.hw); }
    W = model_wire_new();
    port_defaults(&A);
    port_defaults(&B);
    A.n = 0; snprintf(A.desk, sizeof(A.desk), "FOH"); A.mac[5] = 0x11;
    B.n = 1; snprintf(B.desk, sizeof(B.desk), "MON"); B.mac[5] = 0x22;
    portd_opts_t oa = { 1, 0, 0, 0, 0, PTP_ROLE_LEADER, 1, -2, 0, 0, 0 };
    portd_opts_t ob = { 0, 0, 0, 0, 0, PTP_ROLE_FOLLOWER, 0, -2, 0, 0, 0 };
    CHECK(portd_start(&A, mk(0, 1, 0.0, 1760000000ull * 1000000000ull), &oa) == 0);
    model_cfg_t m;
    memset(&m, 0, sizeof(m));
    m.id = 2; m.maxlat = 3;
    m.port_base = 0x1000; m.clock_base = 0x1000; m.dclk_base = 0xFFFFFFFFu; m.cs2k_base = 0xFFFFFFFFu;
    m.phy_fd = -1; m.wire = W; m.wire_side = 1; m.wire_delay_ns = 5000;
    m.tod_start_ns = (uint64_t)(1760000000ll * 1000000000ll + STAMP_X);
    m.rx_stamp_late_ns = rx_late; m.tx_wire_late_ns = tx_late; m.xbar = xbar; m.iobuild = iobuild;
    CHECK(portd_start(&B, model_open(&m), &ob) == 0);
    TEST(what);
    run(12.0);
    ptp_stats_t s;
    ptp_get_stats(B.ptp, &s);
    double off = s.offset_ns - (double)STAMP_X;
    CHECKF(s.state == PTP_SLAVE && fabs(off - want_off) < 1.0 && fabs(s.delay_ns - want_delay) < 1.0,
           "offset X %+.1f ns (want %+.0f), delay %.1f ns (want %.0f)", off, want_off, s.delay_ns, want_delay);
}

static void xbar_stamps(void)
{
    stamp_case("SFD stamps: no offsets (io build 29): offset and delay exact", 0, 0, 0, 29, 0.0, 5000.0);
    stamp_case("SFD stamps: XBAR + io build 32 pipeline, its IDs hidden: off by (80 - 20) / 2, delay + (80 + 20) / 2", 80, 20,
               0, 0, 30.0, 5050.0);
    stamp_case("SFD stamps: io build 32 + XBAR (OXB1 at C0160, OFCP build 32 at C8900): RX -80, TX +20 ns: exact", 80, 20, 1,
               32, 0.0, 5000.0);
    stamp_case("SFD stamps: io build 32, no XBAR: RX -40 ns: exact", 40, 0, 0, 32, 0.0, 5000.0);
    stamp_case("SFD stamps: io build 30: RX -20 ns: exact", 20, 0, 0, 30, 0.0, 5000.0);
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
    portd_opts_t oa = { 1, 0, 0, 0, 0, PTP_ROLE_LEADER, 1, -2, 0, 0, 0 };
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
    portd_opts_t desk = { 0, 0, 0, 0, 0, PTP_ROLE_FOLLOWER, 0, -2, 0, 0, 0 };
    CHECK(pair(&desk) == 0);
    run(30.0);
    ptp_get_stats(B.ptp, &sb);
    dclock_get_stats(B.clk, &cb);
    CHECKF(sb.state == PTP_SLAVE && sb.offset_ns < -2.9e15 && sb.servo_state == 0 && cb.steps == 0,
           "%s offset %.0f servo %d steps %llu", ptp_state_name(sb.state), sb.offset_ns, sb.servo_state,
           (unsigned long long)cb.steps);

    TEST("--steer without --owner is refused (the servo's steps would be dropped: dclock_step needs the owner)");
    portd_opts_t noown = { 0, 0, 0, 0, 0, PTP_ROLE_FOLLOWER, 1, -2, 0, 0, 0 };
    CHECK(pair(&noown) != 0);

    TEST("the fix on the desk (--clock-port 6 --owner --steer), leader 34 days ahead: steps and locks within 60 s");
    portd_opts_t own = { 1, 0, 0, 0, 0, PTP_ROLE_FOLLOWER, 1, -2, 0, 0, 0 };
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
    portd_opts_t oa = { 1, 0, 0, 0, 0, PTP_ROLE_LEADER, 1, -2, 0, 0, 0 };
    portd_opts_t ob = { 1, 0, 0, 0, 0, PTP_ROLE_FOLLOWER, 1, -2, 0, 0, 0 };
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
    portd_opts_t oa = { 1, 0, 0, 0, 0, PTP_ROLE_LEADER, 1, -2, 0, 0, 0 };
    portd_opts_t ob = { 1, 0, 0, 0, 0, PTP_ROLE_FOLLOWER, 1, -2, 0, 0, 0 };
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
    return t_done("test_inproc");
}
