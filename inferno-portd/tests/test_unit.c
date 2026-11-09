/* test_unit.c - inferno-portd unit tests: ARC / CMC / flow-control packets against upstream Inferno's layouts, PTPv1
 * messages byte by byte against IEEE 1588-2002 octet positions, the best-master comparison, the PTP servo against a
 * drifting simulated clock, the audio loop and the CS2000 ratio against the model, DNS names, the model's registers.
 * Part of OpenProSeries inferno-portd. Licence: GPL-3.0-or-later.
 */
#include "test.h"
#include "../src/clock.h"
#include "../src/daemon.h"
#include "../src/fpga_model.h"
#include "../src/mdns.h"
#include "../src/ptp.h"
#include "../src/regs.h"
#include "../src/util.h"

#include <math.h>
#include <string.h>

static uint64_t vt = 1000000000ull;
static uint64_t vnow(void) { return vt; }

static hw_t *mkmodel(int dclk)
{
    model_cfg_t mc;
    memset(&mc, 0, sizeof(mc));
    mc.id = 1; mc.maxlat = 3; mc.port_base = 0x1000; mc.clock_base = 0x1000;
    mc.dclk_base = dclk ? 0x0200 : 0xFFFFFFFFu; mc.cs2k_base = dclk ? 0x0300 : 0xFFFFFFFFu;
    mc.phy_fd = -1; mc.cs2k_r0 = 0x00200000u;
    return model_open(&mc);
}

static int req(uint8_t *b, uint16_t op, const uint8_t *c, int clen)
{
    return rr_make(b, 0x27ff, 0x1234, op, 0, c, clen);
}

static void test_arc(port_t *p)
{
    TEST("ARC 0x1000 channels and flows count (proto_arc.rs channels_and_flows_count::Response)");
    uint8_t q[64], r[1500];
    int n = arc_handle(p, q, req(q, 0x1000, NULL, 0), r, sizeof(r));
    static const uint8_t want[34] = { 0x00, 0x30, 0x00, 0x40, 0x00, 0x40, 0x00, 0x04, 0x00, 0x20, 0x00, 0x08, 0x00, 0x10, 0x00, 0x10,
                                      0x00, 0x80, 0x00, 0x01, 0x00, 0x01 };
    CHECK(n == 44);
    CHECK(get16(r) == 0x27ff && get16(r + 2) == 44 && get16(r + 4) == 0x1234 && get16(r + 6) == 0x1000 && get16(r + 8) == 1);
    CHECK(memcmp(r + 10, want, 34) == 0);

    TEST("ARC 0x1002 / 0x1003 names (get_device_names::ResponseHeader, 38 bytes, packet-relative offsets)");
    n = arc_handle(p, q, req(q, 0x1002, NULL, 0), r, sizeof(r));
    CHECK(n == 10 + (int)strlen("MON-2") + 1 && !strcmp((char *)r + 10, "MON-2"));
    n = arc_handle(p, q, req(q, 0x1003, NULL, 0), r, sizeof(r));
    CHECK(get16(r + 10 + 12) == 48 && !strcmp((char *)r + 48, "MON-2"));
    CHECK(get16(r + 10 + 16) == 48 && get16(r + 10 + 30) == 0x2729 && get16(r + 10 + 34) == 0x1102);
    CHECK(!strcmp((char *)r + get16(r + 10 + 14), p->factory));
    CHECK(!strcmp((char *)r + get16(r + 10 + 6), "OpenProSeries") && !strcmp((char *)r + get16(r + 10 + 8), ":705"));

    TEST("ARC 0x3000 receive channels, 64 channels in two pages of 32 (serialize_items, 0x8112 = more)");
    static const uint8_t pg1[4] = { 0, 0, 0, 1 }, pg2[4] = { 0, 0, 0, 33 };
    n = arc_handle(p, q, req(q, 0x3000, pg1, 4), r, sizeof(r));
    CHECK(get16(r + 8) == 0x8112);
    CHECK(r[10] == 32 && r[11] == 32);
    const uint8_t *it = r + 12;
    CHECK(get16(it) == 1 && get16(it + 2) == 6 && get16(it + 4) == 652);           /* common descriptor after 32 items */
    CHECK(get32(r + 652) == 48000 && get16(r + 652 + 4 + 2) == 24 && get16(r + 652 + 14) == 0xe);
    CHECK(get16(it + 6) == 0 && get16(it + 8) == 0 && !strcmp((char *)r + get16(it + 10), "01"));
    CHECK(get16(r + 12 + 31 * 20) == 32);
    n = arc_handle(p, q, req(q, 0x3000, pg2, 4), r, sizeof(r));
    CHECK(get16(r + 8) == 1 && r[11] == 32 && get16(r + 12) == 33 && !strcmp((char *)r + get16(r + 12 + 31 * 20 + 10), "64"));

    TEST("ARC 0x2000 / 0x2010 transmit channels and friendly names; 0x2013 rename");
    n = arc_handle(p, q, req(q, 0x2000, pg1, 4), r, sizeof(r));
    CHECK(get16(r + 12) == 1 && get16(r + 14) == 7 && !strcmp((char *)r + get16(r + 12 + 6), "01"));
    uint8_t rn[32];
    memset(rn, 0, sizeof(rn));
    rn[0] = 1; rn[1] = 1;
    put16(rn + 2, 0); put16(rn + 4, 3); put16(rn + 6, 10 + 8);
    memcpy(rn + 8, "Kick", 5);
    n = arc_handle(p, q, req(q, 0x2013, rn, 13), r, sizeof(r));
    CHECK(get16(r + 8) == 1 && !strcmp(p->txname[2], "Kick"));
    n = arc_handle(p, q, req(q, 0x2010, pg1, 4), r, sizeof(r));
    CHECK(get16(r + 12 + 12) == 3 && get16(r + 12 + 14) == 3 && !strcmp((char *)r + get16(r + 12 + 16), "Kick"));
    CHECK(get32(r + 12 + 32 * 6) == 0);   /* upstream writes a u32 0 before the first name */

    TEST("ARC 0x3010 subscribe (offsets in the packet), 0x3014 unsubscribe");
    uint8_t sb[64];
    memset(sb, 0, sizeof(sb));
    sb[0] = 1; sb[1] = 1;
    put16(sb + 2, 5); put16(sb + 4, 10 + 8); put16(sb + 6, 10 + 13);
    memcpy(sb + 8, "Kick", 5);
    memcpy(sb + 13, "FOH-1", 6);
    n = arc_handle(p, q, req(q, 0x3010, sb, 19), r, sizeof(r));
    CHECK(get16(r + 8) == 1 && p->sub[4].active && !strcmp(p->sub[4].txch, "Kick") && !strcmp(p->sub[4].txhost, "FOH-1"));
    n = arc_handle(p, q, req(q, 0x3000, pg1, 4), r, sizeof(r));
    it = r + 12 + 4 * 20;
    CHECK(!strcmp((char *)r + get16(it + 6), "Kick") && !strcmp((char *)r + get16(it + 8), "FOH-1") && get32(it + 12) == SUB_UNRESOLVED);
    static const uint8_t un[6] = { 0, 1, 0, 0, 0, 5 };
    arc_handle(p, q, req(q, 0x3014, un, 6), r, sizeof(r));
    CHECK(!p->sub[4].active);

    TEST("ARC 0x3300 (clock domain), 0x2320, CMC 0x1001 device advertisement");
    n = arc_handle(p, q, req(q, 0x3300, NULL, 0), r, sizeof(r));
    CHECK(n == 18 && r[10] == 0x38 && r[17] == 0xff);
    n = arc_handle(p, q, req(q, 0x2320, NULL, 0), r, sizeof(r));
    CHECK(get16(r + 8) == 0x30);
    n = cmc_handle(p, q, req(q, 0x1001, NULL, 0), r, sizeof(r));
    CHECK(n == 32 && !memcmp(r + 12, p->devid, 8) && get16(r + 20) == 1 && get32(r + 24) == net_ip(p->net) && get16(r + 28) == 8700);
}

static void test_flowctl(port_t *p)
{
    TEST("flow request 0x0100 (flows_control.rs request_flow) parsed by the server; stop / update by handle");
    uint8_t q[600], r[64];
    uint16_t ch[16];
    for (int i = 0; i < 16; i++) ch[i] = (uint16_t)(i + 17);
    int n = fc_build_request(p, 7, 48000, 24, 16, ch, 16, 50000, "1_0", q, sizeof(q));
    CHECK(get16(q) == 0x1102 && get16(q + 6) == 0x0100);
    CHECK(get16(q + 10) == 0x26 + 32 + 10);                  /* the strings offset */
    CHECK(!strcmp((char *)q + get16(q + 10), p->name));
    int rd = get16(q + 10 + 14);
    CHECK(rd % 8 == 0 && get16(q + rd) == 0x0802 && get16(q + rd + 2) == 50000 && get32(q + rd + 4) == net_ip(p->net));
    CHECK(get16(q + 10 + 16 + 32 + 6) == 16);                /* fpp */
    int m = flowctl_handle(p, net_ip(p->net), q, n, r, sizeof(r));
    CHECK(m == 16 && get16(r + 8) == 1);
    uint32_t idx = get32(r + 10);
    CHECK(idx < NTXF && p->tx[idx].used && p->tx[idx].nch == 16 && p->tx[idx].ch[0] == 16 && p->tx[idx].fpp == 16 && p->tx[idx].bps == 3);
    /* the same destination again: the flow is reused */
    n = fc_build_request(p, 8, 48000, 24, 16, ch, 8, 50000, "1_0", q, sizeof(q));
    m = flowctl_handle(p, net_ip(p->net), q, n, r, sizeof(r));
    CHECK(get32(r + 10) == idx && p->tx[idx].nch == 8);
    /* rate mismatch */
    n = fc_build_request(p, 9, 96000, 24, 16, ch, 8, 50002, "2_0", q, sizeof(q));
    m = flowctl_handle(p, net_ip(p->net), q, n, r, sizeof(r));
    CHECK(get16(r + 8) == 0x0301);
    /* update: handle + 2 channels */
    uint8_t up[12];
    put32(up, idx); put16(up + 4, p->tx[idx].cookie); put16(up + 6, 2); put16(up + 8, 1); put16(up + 10, 0);
    n = rr_make(q, 0x1102, 10, 0x0102, 0, up, 12);
    m = flowctl_handle(p, 0, q, n, r, sizeof(r));
    CHECK(get16(r + 8) == 1 && p->tx[idx].nch == 2 && p->tx[idx].ch[0] == 0 && p->tx[idx].ch[1] == -1);
    put16(up + 4, (uint16_t)(p->tx[idx].cookie ^ 1));
    n = rr_make(q, 0x1102, 11, 0x0101, 0, up, 6);
    m = flowctl_handle(p, 0, q, n, r, sizeof(r));
    CHECK(get16(r + 8) == 0x0103 && p->tx[idx].used);   /* a wrong cookie */
    put16(up + 4, p->tx[idx].cookie);
    n = rr_make(q, 0x1102, 12, 0x0101, 0, up, 6);
    m = flowctl_handle(p, 0, q, n, r, sizeof(r));
    CHECK(get16(r + 8) == 1 && !p->tx[idx].used);
    TEST("admission control: the 95 Mb/s budget (fpga/docs/inferno.md section 5)");
    p->lat_mode = 0;
    double l64 = tx_load_bps(p, 16, 0, 3) * 4;   /* 64 ch at 0.5 ms in 16-ch flows: 88.1 Mb/s */
    CHECKF(fabs(l64 / 1e6 - 88.1) < 0.2, "%.2f Mb/s", l64 / 1e6);
    double l8 = tx_load_bps(p, 8, 0, 3) * 8;   /* 64 ch in 8-ch flows at 0.5 ms: 102.5 Mb/s, refused */
    CHECKF(fabs(l8 / 1e6 - 102.5) < 0.2, "%.2f Mb/s", l8 / 1e6);
}

static void test_ptp_msgs(void)
{
    TEST("PTPv1 Sync / Delay_Req (124 bytes) octet by octet (IEEE 1588-2002; statime messages_v1)");
    ptp1_hdr_t h;
    memset(&h, 0, sizeof(h));
    memcpy(h.subdomain, "_DFLT", 5);
    h.msgtype = 1; h.comm = 1;
    static const uint8_t U[6] = { 0x02, 0x4f, 0x50, 0x53, 0x00, 0x01 }, V[6] = { 0x00, 0x1d, 0xc1, 0x11, 0x22, 0x33 };
    memcpy(h.uuid, U, 6);
    h.port = 1; h.seq = 0x1234; h.control = PTP1_SYNC; h.flags = PTP1_FLAG_ASSIST;
    ptp1_sync_t s;
    memset(&s, 0, sizeof(s));
    s.sec = 0x01020304; s.ns = 0x05060708; s.utc_offset = 37;
    s.gm.comm = 1; memcpy(s.gm.uuid, V, 6); s.gm.port = 1; s.gm.seq = 0x1234; s.gm.stratum = 4; memcpy(s.gm.ident, "DFLT", 4);
    s.gm.variance = (int16_t)0xF060; s.gm.preferred = 1;
    s.sync_interval = -2; s.local_variance = (int16_t)0xF060; s.steps = 1; s.local_stratum = 4; memcpy(s.local_ident, "DFLT", 4);
    s.parent_comm = 1; memcpy(s.parent_uuid, V, 6); s.parent_port = 1; s.utc_reasonable = 1;
    uint8_t b[124];
    CHECK(ptp1_put_sync(b, &h, &s) == 124);
    CHECK(get16(b) == 1 && get16(b + 2) == 1 && !memcmp(b + 4, "_DFLT\0\0\0", 8) && b[20] == 1 && b[21] == 1);
    CHECK(!memcmp(b + 22, U, 6) && get16(b + 28) == 1 && get16(b + 30) == 0x1234 && b[32] == 0 && b[35] == 0x08);
    CHECK(get32(b + 40) == 0x01020304 && get32(b + 44) == 0x05060708 && get16(b + 50) == 37);
    CHECK(b[53] == 1 && !memcmp(b + 54, V, 6) && get16(b + 60) == 1 && get16(b + 62) == 0x1234);
    CHECK(b[67] == 4 && !memcmp(b + 68, "DFLT", 4) && get16(b + 74) == 0xF060 && b[77] == 1 && b[79] == 0);
    CHECK(b[83] == 0xFE && get16(b + 86) == 0xF060 && get16(b + 90) == 1 && b[95] == 4 && !memcmp(b + 96, "DFLT", 4));
    CHECK(b[101] == 1 && !memcmp(b + 102, V, 6) && get16(b + 110) == 1 && b[123] == 1);
    ptp1_hdr_t h2;
    ptp1_sync_t s2;
    CHECK(ptp1_get_hdr(b, 124, &h2) == 0 && ptp1_get_sync(b, 124, &s2) == 0);
    CHECK(h2.seq == 0x1234 && h2.flags == 8 && s2.gm.stratum == 4 && s2.gm.preferred && s2.sync_interval == -2 && s2.steps == 1);
    CHECK(!memcmp(&s2.gm, &s.gm, sizeof(s.gm)) || (s2.gm.variance == s.gm.variance && !memcmp(s2.gm.uuid, V, 6)));

    TEST("PTPv1 Follow_Up (52 bytes: associatedSequenceId at 42, preciseOriginTimestamp at 44) and Delay_Resp (60)");
    ptp1_fup_t f = { 0x1234, 1760000000u, 999999999u };
    h.msgtype = 2; h.control = PTP1_FOLLOW_UP; h.flags = 0;
    CHECK(ptp1_put_followup(b, &h, &f) == 52);
    CHECK(b[20] == 2 && b[32] == 2 && get16(b + 40) == 0 && get16(b + 42) == 0x1234 && get32(b + 44) == 1760000000u && get32(b + 48) == 999999999u);
    ptp1_fup_t f2;
    CHECK(ptp1_get_followup(b, 52, &f2) == 0 && f2.assoc_seq == 0x1234 && f2.sec == 1760000000u && f2.ns == 999999999u);
    ptp1_dresp_t dr = { 7, 8, 1, { 0 }, 1, 0x99 };
    memcpy(dr.req_uuid, V, 6);
    h.control = PTP1_DELAY_RESP;
    CHECK(ptp1_put_delay_resp(b, &h, &dr) == 60);
    CHECK(get32(b + 40) == 7 && get32(b + 44) == 8 && b[48] == 0 && b[49] == 1 && !memcmp(b + 50, V, 6) && get16(b + 56) == 1 && get16(b + 58) == 0x99);
    ptp1_dresp_t d2;
    CHECK(ptp1_get_delay_resp(b, 60, &d2) == 0 && d2.req_seq == 0x99 && !memcmp(d2.req_uuid, V, 6));

    TEST("best master (1588-2002 data set comparison, statime's preferred -> priority1)");
    ptp1_ds_t a = { 0, 4, "DFLT", (int16_t)0xF060, { 0x02, 0, 0, 0, 0, 2 }, 0 }, c = a;
    memcpy(c.uuid, "\x02\0\0\0\0\x01", 6);
    CHECK(ptp1_compare(&c, &a) < 0 && ptp1_compare(&a, &c) > 0 && ptp1_compare(&a, &a) == 0);
    a.preferred = 1;
    CHECK(ptp1_compare(&a, &c) < 0);
    a.preferred = 0; c.stratum = 5;
    CHECK(ptp1_compare(&a, &c) < 0);
    c.stratum = 4; memcpy(c.ident, "GPS\0", 4);
    CHECK(ptp1_compare(&c, &a) < 0);
    memcpy(c.ident, "DFLT", 4); c.variance = (int16_t)0xF000;
    CHECK(ptp1_compare(&c, &a) < 0);
}

static void test_servo(void)
{
    TEST("servo: +50 ppm oscillator, 3 ms off, 4 syncs/s with 50 ns noise: steps, then locks within 200 ns");
    servo_t s;
    memset(&s, 0, sizeof(s));
    servo_init(&s);
    double local = 3e6, drift = 50e-6, ppb = 0;   /* local - leader, ns */
    uint64_t t = 0;
    int steps = 0;
    double worst = 0;
    rnd_seed(42);
    for (int k = 0; k < 4 * 120; k++) {
        t += 250000000ull;
        local += 0.25e9 * (drift + ppb * 1e-9);
        double noise = ((double)(rnd32() % 101) - 50.0);
        double p, st;
        int r = servo_sample(&s, local + noise, t, &p, &st);
        if (r == SERVO_JUMP) { local += st; steps++; }
        ppb = p;
        if (k > 4 * 60 && fabs(local) > worst) worst = fabs(local);
    }
    CHECK(steps == 1);
    CHECKF(worst < 200.0, "worst %.0f ns", worst);
    CHECKF(fabs(ppb + 50000.0) < 100.0, "ppb %.1f", ppb);
}

static void test_aloop(void)
{
    TEST("audio loop: desk clock +30 ppm, steered through the CS2000 ratio (12.20, r0 2.0): phase held within 0.05 samples");
    aloop_t a;
    aloop_init(&a, 48000);
    double phase = 0, worst = 0, u = 0;
    uint64_t t = 0;
    double acc = 0;
    for (int k = 0; k < 8 * 600; k++) {
        t += 125000000ull;
        uint32_t r = cs2k_dither(&acc, 0x00200000u, -u);
        double rel = (1.0 + 30e-6) * (double)r / (double)0x00200000u;
        phase += 0.125 * 48000.0 * (rel - 1.0);
        u = aloop_sample(&a, phase, t);
        if (k > 8 * 300 && fabs(phase - a.ref) > worst) worst = fabs(phase - a.ref);   /* the loop holds its first phase */
    }
    CHECKF(worst < 0.05, "worst %.4f samples", worst);
    CHECKF(fabs(u - 30.0) < 1.0, "u %.2f ppm", u);
    CHECK(cs2k_ratio(0x00200000u, 0) == 0x00200000u && cs2k_ratio(0x00200000u, 1.0) == 0x00200002u);
}

static void test_dclock_model(void)
{
    TEST("desk clock on the model: fs_phase + CS2000 engine, follower, desk +40 ppm: ratio converges, phase bounded");
    g_now_ns = vnow;
    dclock_cfg_t c = { 1, 48000, 1, 1, 0x00200000u, 0 };
    model_cfg_t m;
    memset(&m, 0, sizeof(m));
    m.id = 1; m.maxlat = 3; m.port_base = 0x1000; m.clock_base = 0x1000; m.dclk_base = 0x200; m.cs2k_base = 0x300; m.phy_fd = -1;
    m.cs2k_r0 = 0x00200000u; m.desk_ppm = 40.0; m.xtal_ppm = -12.0;
    hw_t *h = model_open(&m);
    CHECK(hw_rd(h, 0x200 + FP_ID) == FP_ID_VALUE);
    dclock_t *d = dclock_new(h, &c);
    dclock_set_mode(d, DCLK_FOLLOWER);
    dclock_set_locked(d, 1);   /* the PTP's lock (no PTP here): a follower's audio loop starts DCLK_SETTLE_NS into it */
    double worst = 0;
    model_stats_t st;
    for (int k = 0; k < 600000; k++) {   /* 600 s in 1 ms ticks */
        vt += 1000000ull;
        dclock_tick(d, vt);
        if (k % 1000 == 0 && k > 400000) {
            dclock_stats_t s;
            dclock_get_stats(d, &s);
            if (fabs(s.audio_phase) > worst) worst = fabs(s.audio_phase);
        }
    }
    model_get_stats(h, &st);
    dclock_stats_t s;
    dclock_get_stats(d, &s);
    /* the desk's rate must equal the media clock's: the oscillator's -12 ppm (the time of day is not steered here) */
    double desk_vs_media = st.desk_rate_rel / st.tick_rate_rel;
    CHECKF(fabs(desk_vs_media - 1.0) < 1e-6, "desk / media %.3f ppm", (desk_vs_media - 1.0) * 1e6);
    CHECKF(worst < 0.1, "worst phase %.4f", worst);
    CHECKF(s.cs2k_writes > 3 && s.cs2k_errors == 0, "writes %llu errors %llu", (unsigned long long)s.cs2k_writes, (unsigned long long)s.cs2k_errors);
    CHECKF(fabs(s.media_err) < 1.5, "media err %.2f", s.media_err);
    TEST("desk clock: the media clock tracks the time of day x fs (sigma-delta MEDIA_INC) over 600 s");
    CHECKF(s.media_loads == 1, "media loads %llu (no slip reload)", (unsigned long long)s.media_loads);
    double tod_samples = st.tod_s * 48000.0;
    CHECKF(fabs(st.media_pos - tod_samples) < 1.5, "media %.2f tod x fs %.2f", st.media_pos, tod_samples);
    dclock_free(d);
    h->ops->close(h);
    g_now_ns = mono_ns;
}

static void test_dns(void)
{
    TEST("DNS names: labels, a compression pointer, an '@' instance label");
    uint8_t b[256];
    int n = dns_put_name(b, sizeof(b), "_netaudio-arc._udp.local");
    CHECK(n == 26 && b[0] == 13 && b[14] == 4 && b[19] == 5 && b[25] == 0);
    b[n] = 0xC0; b[n + 1] = 14;   /* a pointer to "_udp.local" */
    char out[128];
    CHECK(dns_get_name(b, n + 2, n, out, sizeof(out)) == n + 2 && !strcmp(out, "_udp.local"));
    mdns_svc_t s;
    memset(&s, 0, sizeof(s));
    strcpy(s.inst, "01@FOH-1"); strcpy(s.type, "_netaudio-chan"); s.port = 4455; s.ttl = 120; s.ntxt = 2;
    strcpy(s.txt[0], "id=1"); strcpy(s.txt[1], "fpp=32,2");
    uint8_t r[512];
    n = mdns_build_response(&s, IP4(169, 254, 1, 2), 0, -1, r, sizeof(r));
    CHECK(n > 0 && get16(r + 2) == 0x8400 && get16(r + 6) == 3 && get16(r + 10) == 1);
    int off = dns_get_name(r, n, 12, out, sizeof(out));
    CHECK(!strcmp(out, "_netaudio-chan._udp.local") && get16(r + off) == 12);
    off = dns_get_name(r, n, off + 10, out, sizeof(out));
    CHECK(!strcmp(out, "01@FOH-1._netaudio-chan._udp.local"));
}

static void test_model_regs(void)
{
    TEST("model registers: IDs, CAPS, LATENCY refusal above MAXLAT, table auto-increment, cpu_pktq frame loop");
    hw_t *h = mkmodel(0);
    CHECK(port_rd(h, AOIP_ID) == AOIP_ID_VALUE && port_rd(h, INF_ID) == INF_ID_VALUE && port_rd(h, CQ_ID) == CQ_ID_VALUE);
    CHECK(port_rd(h, INF_CAPS) == 0x10102040u && (port_rd(h, INF_CAPS2) & 0x3FF) == 0x320);
    port_wr(h, INF_LATENCY, 2);
    CHECK((port_rd(h, INF_LAT_STATUS) & 0x7FF) == 96 && ((port_rd(h, INF_LAT_STATUS) >> 16) & 63) == 32);
    uint32_t w[3] = { 1, 2, 3 };
    tbl_write(h, TBL_RX(3, 1), w, 3);
    CHECK(tbl_read(h, TBL_RX(3, 2)) == 2 && port_rd(h, INF_TBL_ADDR) == TBL_RX(3, 3));
    h->ops->close(h);
}

int main(void)
{
    g_log_level = LOG_WARN;
    test_ptp_msgs();
    test_servo();
    test_aloop();
    test_dns();
    test_model_regs();
    test_dclock_model();
    /* a port on the model, for the protocol handlers */
    g_now_ns = vnow;
    static port_t P;
    port_defaults(&P);
    P.n = 1;
    snprintf(P.desk, sizeof(P.desk), "MON");
    P.cfg_ip = IP4(169, 254, 10, 2);
    portd_opts_t o = { 1, 0, 0, 0, 0, 0, 0, -2, 0 };
    CHECK(portd_start(&P, mkmodel(0), &o) == 0);
    CHECK(!strcmp(P.name, "MON-2") && P.devid[3] == 0xFF && P.devid[4] == 0xFE);
    test_arc(&P);
    test_flowctl(&P);
    return t_done("test_unit");
}
