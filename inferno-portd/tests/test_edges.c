/* Failure-path regressions for control, flows and persistence. Licence: GPL-3.0-or-later. */
#include "test.h"
#include "../src/ctl.h"
#include "../src/daemon.h"
#include "../src/fpga_model.h"
#include "../src/regs.h"
#include "../src/util.h"
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

static uint64_t now = 5000000000ull;
static uint64_t vnow(void) { return now; }
static port_t p;
static void setup(void)
{
    port_defaults(&p); p.cfg_ip = IP4(192, 0, 2, 10);
    model_cfg_t m = {0}; m.phy_fd = -1; m.maxlat = 3;
    m.port_base = m.clock_base = 0x1000; m.dclk_base = m.cs2k_base = 0xffffffffu;
    portd_opts_t o = {0}; o.log_sync = -2;
    CHECK(portd_start(&p, model_open(&m), &o) == 0);
}
static void teardown(void)
{
    p.state_path[0] = 0;
    portd_stop(&p); p.hw->ops->close(p.hw);
}
static int request(int nch, int fpp, int port, int *code)
{
    uint8_t q[600], out[64]; uint16_t ch[MAXFCH];
    for (int i = 0; i < MAXFCH; i++) ch[i] = (uint16_t)(i + 1);
    int n = fc_build_request(&p, 1, 48000, 24, (uint16_t)fpp, ch, nch, (uint16_t)port, "edge", q, sizeof(q));
    int r = flowctl_handle(&p, p.cfg_ip, q, n, out, sizeof(out));
    CHECK(r >= RR_HDR);
    *code = r >= RR_HDR ? get16(out + 8) : 0;
    return *code == 1 && r >= RR_HDR + 6 ? (int)get32(out + 10) : -1;
}
static void test_reused_flow_budget(void)
{
    TEST("reused destination: over-budget fpp update is rejected without mutation"); setup();
    int code, a = request(32, 32, 50000, &code), b = request(32, 32, 50001, &code);
    CHECK(a >= 0 && b >= 0);
    if (a >= 0 && b >= 0) {
        request(32, 2, 50000, &code); CHECK(code == 1);
        request(32, 2, 50001, &code);
        CHECK(code == 0x0315); CHECK(p.tx[b].fpp == 32);
        CHECK(tx_load_bps(&p, 0, 0, 0) <= TX_BUDGET_BPS);
    }
    teardown();
}
static void test_handle_update_budget(void)
{
    TEST("handle update: increasing channel count cannot exceed the link budget"); setup();
    int code, first = -1;
    for (int i = 0; i < NTXF; i++) {
        int f = request(1, 8, 50000 + i, &code); CHECK(f >= 0);
        if (!i) first = f;
    }
    if (first >= 0) {
        uint8_t body[8 + 2 * MAXFCH], q[128], out[64];
        put32(body, (uint32_t)first); put16(body + 4, p.tx[first].cookie); put16(body + 6, MAXFCH);
        for (int i = 0; i < MAXFCH; i++) put16(body + 8 + i * 2, (uint16_t)(i + 1));
        int n = rr_make(q, DBCP1, 2, 0x0102, 0, body, sizeof(body));
        CHECK(flowctl_handle(&p, p.cfg_ip, q, n, out, sizeof(out)) >= RR_HDR);
        CHECK(get16(out + 8) == 0x0315); CHECK(p.tx[first].nch == 1);
        CHECK(tx_load_bps(&p, 0, 0, 0) <= TX_BUDGET_BPS);
    }
    teardown();
}
static void test_save_failure(void)
{
    TEST("buffered write failure preserves the previous state file"); setup();
    char dir[] = "/tmp/openinferno-save-XXXXXX", tmp[300];
    CHECK(mkdtemp(dir) != NULL);
    snprintf(p.state_path, sizeof(p.state_path), "%s/state", dir);
    FILE *f = fopen(p.state_path, "w"); CHECK(f != NULL);
    if (f) { fputs("old", f); fclose(f); }
    snprintf(tmp, sizeof(tmp), "%s.tmp", p.state_path);
    CHECK(symlink("/dev/full", tmp) == 0);
    CHECK(settings_save(&p) == -1);
    char old[4] = {0}; f = fopen(p.state_path, "r"); CHECK(f != NULL);
    if (f) { CHECK(fread(old, 1, 3, f) == 3); fclose(f); }
    CHECK(!strcmp(old, "old"));
    unlink(tmp); unlink(p.state_path); rmdir(dir); teardown();
}
static void test_save_retry(void)
{
    TEST("a failed save stays dirty and succeeds after the directory becomes available"); setup();
    char dir[] = "/tmp/openinferno-retry-XXXXXX", parent[256]; CHECK(mkdtemp(dir) != NULL);
    snprintf(parent, sizeof(parent), "%s/later", dir);
    snprintf(p.state_path, sizeof(p.state_path), "%s/state", parent);
    p.dirty = 1; p.next_save = 0; port_tick(&p, now);
    CHECK(p.dirty);
    CHECK(mkdir(parent, 0700) == 0);
    now += 2000000000ull; port_tick(&p, now);
    CHECK(access(p.state_path, F_OK) == 0); CHECK(!p.dirty);
    unlink(p.state_path); rmdir(parent); rmdir(dir); teardown();
}
static void arp(const uint8_t mac[6])
{
    uint8_t f[42] = {0}; stamp_t ts = {0};
    memcpy(f, p.mac, 6); memcpy(f + 6, mac, 6); put16(f + 12, 0x0806);
    put16(f + 14, 1); put16(f + 16, 0x0800); f[18] = 6; f[19] = 4; put16(f + 20, 2);
    memcpy(f + 22, mac, 6); put32(f + 28, p.cfg_ip); put32(f + 38, p.cfg_ip);
    /* The requester helper uses this port's IP as the receiver IP; use an actual peer instead. */
    put32(f + 28, IP4(192, 0, 2, 11));
    net_input(p.net, f, sizeof(f), &ts);
}
static void test_mac_refresh(void)
{
    TEST("ARP changes update the MAC programmed for an active flow"); setup();
    const uint8_t a[6] = {2, 0, 0, 0, 0, 1}, b[6] = {2, 0, 0, 0, 0, 2};
    arp(a);
    uint8_t q[600], out[64]; uint16_t ch = 1;
    int n = fc_build_request(&p, 1, 48000, 24, 8, &ch, 1, 50000, "edge", q, sizeof(q));
    put32(q + get16(q + RR_HDR + 14) + 4, IP4(192, 0, 2, 11));
    CHECK(flowctl_handle(&p, p.cfg_ip, q, n, out, sizeof(out)) == RR_HDR + 6);
    int f = (int)get32(out + RR_HDR); CHECK(f >= 0 && f < NTXF);
    if (f >= 0 && f < NTXF) {
        CHECK(p.tx[f].programmed); CHECK(tbl_read(p.hw, TBL_TX(f, 2)) == get32(a + 2));
        arp(b); txflows_tick(&p, now);
        uint8_t learned[6]; CHECK(net_mac_of(p.net, IP4(192, 0, 2, 11), learned));
        CHECK(!memcmp(learned, b, 6)); CHECK(tbl_read(p.hw, TBL_TX(f, 2)) == get32(b + 2));
    }
    teardown();
}
static void test_control_backpressure(void)
{
    TEST("slow control reader receives one complete reply per executed command"); setup();
    char dir[] = "/tmp/openinferno-ctl-XXXXXX", path[108]; CHECK(mkdtemp(dir) != NULL);
    snprintf(path, sizeof(path), "%s/control", dir);
    ctl_t c; CHECK(ctl_listen(&c, path) == 0);
    int fd[2]; CHECK(socketpair(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK, 0, fd) == 0);
    int small = 1024; CHECK(setsockopt(fd[0], SOL_SOCKET, SO_SNDBUF, &small, sizeof(small)) == 0);
    c.cl[0].fd = fd[0];
    char commands[1400]; for (int i = 0; i < 200; i++) memcpy(commands + 7 * i, "status\n", 7);
    CHECK(write(fd[1], commands, sizeof(commands)) == (ssize_t)sizeof(commands));
    ctl_poll(&c, &p);
    int replies = 0, state = 0;
    for (int i = 0; i < 500; i++) {
        char out[4096]; ssize_t len;
        while ((len = read(fd[1], out, sizeof(out))) > 0)
            for (ssize_t j = 0; j < len; j++) {
                if (out[j] == "ok\n"[state]) { if (++state == 3) { replies++; state = 0; } }
                else state = out[j] == 'o' ? 1 : 0;
            }
        ctl_poll(&c, &p);
    }
    CHECKF(replies == 200, "received %d of 200 replies", replies);
    close(fd[1]); ctl_close(&c); rmdir(dir); teardown();
}
static void test_control_eof(void)
{
    TEST("control peer half-close delivers complete commands and discards incomplete ones"); setup();
    char dir[] = "/tmp/openinferno-eof-XXXXXX", path[108]; CHECK(mkdtemp(dir) != NULL);
    snprintf(path, sizeof(path), "%s/control", dir);
    for (int complete = 0; complete < 2; complete++) {
        ctl_t c; CHECK(ctl_listen(&c, path) == 0);
        int fd[2]; CHECK(socketpair(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK, 0, fd) == 0);
        c.cl[0].fd = fd[0];
        const char *cmd = complete ? "status\n" : "sta";
        CHECK(write(fd[1], cmd, strlen(cmd)) == (ssize_t)strlen(cmd));
        CHECK(shutdown(fd[1], SHUT_WR) == 0);
        ctl_poll(&c, &p); ctl_poll(&c, &p);
        CHECK(c.cl[0].fd == -1);
        char out[16384] = ""; ssize_t n = read(fd[1], out, sizeof(out) - 1);
        if (complete) CHECK(n > 0 && strstr(out, "ok\n"));
        else CHECK(n == 0);
        close(fd[1]); ctl_close(&c);
    }
    rmdir(dir); teardown();
}

static void test_control_errors(void)
{
    TEST("invalid subscriptions and refused multicast commands report errors"); setup();
    char out[256];
    CHECK(ctl_exec(&p, "sub 0 01@peer", out, sizeof(out)) == -1);
    CHECK(!strncmp(out, "err ", 4));
    CHECK(ctl_exec(&p, "mcast 0 1", out, sizeof(out)) == -1);
    CHECK(!strncmp(out, "err ", 4));
    CHECK(ctl_exec(&p, "mcast 1 1", out, sizeof(out)) == 0);
    CHECK(ctl_exec(&p, "mcast 1 1", out, sizeof(out)) == -1);
    CHECK(!strncmp(out, "err ", 4));
    teardown();
}

static const struct hw_ops *real_ops;
static uint32_t caps_value;
static uint32_t test_caps_read(hw_t *h, uint32_t addr)
{
    return addr == h->port_base + INF_CAPS ? caps_value : real_ops->rd(h, addr);
}
static void test_capabilities(void)
{
    TEST("FPGA capacities beyond the daemon's tables, or with no flows, are rejected before configuration");
    /* INF_CAPS: 31:24 RX flows, 23:16 TX flows, 15:8 channels per flow, 7:0 columns */
    const uint32_t bad[] = {0x10102000u, 0x10102041u, 0x10102140u, 0x10100040u, 0x10112040u, 0x11102040u, 0x00002040u};
    for (size_t i = 0; i < ARRAY_LEN(bad); i++) {
        model_cfg_t m = {0}; m.phy_fd = -1; m.maxlat = 3; m.port_base = m.clock_base = 0x1000;
        m.dclk_base = m.cs2k_base = 0xffffffffu;
        hw_t *h = model_open(&m);
        real_ops = h->ops;
        struct hw_ops ops = *real_ops; ops.rd = test_caps_read; h->ops = &ops; caps_value = bad[i];
        port_defaults(&p); portd_opts_t o = {0}; o.log_sync = -2;
        CHECK(portd_start(&p, h, &o) == -1);
        CHECK(p.net == NULL); h->ops->close(h);
    }
    TEST("an explicit /0 mask is not replaced by the default /16"); setup();
    CHECK(net_mask(p.net) == IP4(255,255,0,0)); teardown();
    port_defaults(&p); p.cfg_ip = IP4(192,0,2,10); p.cfg_mask = 0;
    model_cfg_t m = {0}; m.phy_fd = -1; m.maxlat = 3; m.port_base = m.clock_base = 0x1000;
    m.dclk_base = m.cs2k_base = 0xffffffffu;
    hw_t *h = model_open(&m); portd_opts_t o = {0}; o.log_sync = -2;
    CHECK(portd_start(&p,h,&o) == 0); CHECK(net_mask(p.net) == 0); teardown();
}

static void test_discovery_flood(void)
{
    TEST("repeated discovery questions are coalesced and replies are paced"); setup();
    uint8_t frame[512] = {0};
    uint8_t *ip = frame + 14, *u = ip + 20, *d = u + 8;
    int pos = 12; put16(d + 4, 5);
    for (int i = 0; i < 5; i++) {
        pos += dns_put_name(d + pos, 400 - pos, "_netaudio-chan._udp.local");
        put16(d + pos, 12); put16(d + pos + 2, 1); pos += 4;
    }
    put16(frame + 12, 0x0800); ip[0] = 0x45; put16(ip + 2, (uint16_t)(28 + pos));
    ip[8] = 255; ip[9] = 17; put32(ip + 12, IP4(192,0,2,11)); put32(ip + 16, MDNS_GROUP);
    put16(ip + 10, inet_csum(ip,20,0)); put16(u,5353); put16(u + 2,5353); put16(u + 4,(uint16_t)(8 + pos));
    stamp_t ts = {0}; mdns_stats_t before, after; net_stats_t ns_before, ns_after;
    mdns_get_stats(p.mdns, &before); net_get_stats(p.net, &ns_before);
    for (int i = 0; i < 20; i++) net_input(p.net,frame,42 + pos,&ts);
    mdns_get_stats(p.mdns, &after); CHECK(after.tx_responses == before.tx_responses);
    /* Drain initial announcements before measuring query replies in isolation. */
    for (int i = 0; i < 2200; i++) { now += 1000000ull; port_tick(&p, now); }
    mdns_get_stats(p.mdns, &before); net_get_stats(p.net, &ns_before);
    for (int i = 0; i < 20; i++) net_input(p.net,frame,42 + pos,&ts);
    for (int i = 0; i < 100; i++) { now += 1000000ull; port_tick(&p,now); }
    mdns_get_stats(p.mdns, &after); net_get_stats(p.net, &ns_after);
    CHECK(after.tx_responses > before.tx_responses);
    CHECK(after.tx_responses - before.tx_responses <= 40);
    CHECK(ns_after.drops == ns_before.drops);
    for (int i = 0; i < 300; i++) { now += 1000000ull; port_tick(&p,now); }
    mdns_get_stats(p.mdns, &after);
    CHECK(after.tx_responses - before.tx_responses == 64);
    teardown();
}

static void update_reply(int f)
{
    rxflow_t *r = &p.rx[f];
    uint8_t frame[64] = {0}, reply[16];
    int n = rr_make(reply, DBCP1, r->req_seq, 0x0102, 1, NULL, 0);
    uint8_t *ip = frame + 14, *u = ip + 20;
    put16(frame + 12, 0x0800); ip[0] = 0x45; ip[9] = 17; put16(ip + 2, (uint16_t)(28 + n));
    put32(ip + 12, r->ctl_ip); put32(ip + 16, p.cfg_ip); put16(ip + 10, inet_csum(ip,20,0));
    put16(u,4455); put16(u + 2,(uint16_t)p.fc_sock); put16(u + 4,(uint16_t)(8 + n)); memcpy(u + 8,reply,(size_t)n);
    stamp_t ts = {0}; net_input(p.net,frame,42 + n,&ts);
}
static void test_cancel_pending(void)
{
    TEST("a delayed update cannot resurrect an unsubscribed or reassigned column"); setup();
    rxflow_t *r = &p.rx[0];
    r->used = 1; r->state = 2; r->ctl_ip = IP4(192,0,2,11); r->req_seq = 7;
    r->nch = r->pend_nch = 1; r->bps = 3; r->col[0] = -1; r->pend_col[0] = 0; r->pend_txch[0] = 1;
    p.sub[0].active = 1; p.sub[0].flow = p.sub[0].slot = 0;
    sub_unsubscribe(&p,0); CHECK(r->pend_col[0] == -1);
    update_reply(0); CHECK(r->col[0] == -1); CHECK(!p.sub[0].active);
    r->state = 2; r->req_seq = 8; r->pend_col[0] = 0;
    p.sub[0].active = 1; p.sub[0].flow = 1; p.sub[0].slot = 0;
    update_reply(0); CHECK(r->col[0] == -1);
    teardown();
}
static void test_latency_transitions(void)
{
    TEST("latency changes recompute receiver shifts in both directions"); setup();
    rxflow_t *r = &p.rx[0]; r->used = r->state = 1; r->nch = 1; r->bps = 3; r->col[0] = 0;
    r->peer_latency_ns = r->latency_ns = 5000000; r->shift = 216;
    CHECK(port_set_latency(&p,3) == 0); CHECK(r->shift == 0); CHECK(tbl_read(p.hw,TBL_RX(0,3)) == 0);
    CHECK(port_set_latency(&p,0) == 0); CHECK(r->shift == 216); CHECK(tbl_read(p.hw,TBL_RX(0,3)) == 216);
    r->peer_latency_ns = 500000; CHECK(port_set_latency(&p,3) == 0); CHECK(r->latency_ns == 5000000);
    CHECK(port_set_latency(&p,0) == 0); CHECK(r->latency_ns == 500000 && r->shift == 0);
    teardown();
    TEST("an over-budget latency change is refused without changing hardware"); setup();
    CHECK(port_set_latency(&p,3) == 0); int16_t ch[8]; for (int i = 0; i < 8; i++) ch[i] = (int16_t)i;
    for (int f = 0; f < 8; f++) CHECK(txflow_create_mcast(&p,f,ch,8) == 0);
    CHECK(port_set_latency(&p,0) == -1); CHECK(p.lat_mode == 3); CHECK((port_rd(p.hw,INF_LATENCY) & 3) == 3);
    char out[256]; p.maxlat = 0;
    CHECK(ctl_exec(&p,"mode 3",out,sizeof(out)) == -1); CHECK(!strncmp(out,"err ",4));
    CHECK(ctl_exec(&p,"mode junk",out,sizeof(out)) == -1);
    teardown();
}
static void test_latency_fit(void)
{
    TEST("the full build (256 frames): every latency mode pair fits; the jitter buffer bounds the peer frame-exactly");
    setup();
    static const uint32_t L[4] = { 500000, 1000000, 2000000, 5000000 };
    for (int ours = 0; ours < 4; ours++) {
        CHECK(port_set_latency(&p, ours) == 0);
        for (int their = 0; their < 4; their++) CHECK(sub_latency_fits(&p, L[their]));
    }
    CHECK(port_set_latency(&p, 1) == 0);
    CHECK(sub_latency_fits(&p, 5333334));    /* 256 frames at 48 kHz: SHIFT 208 + offset 48 = the buffer */
    CHECK(!sub_latency_fits(&p, 5354167));   /* 257 frames */
    CHECK(!sub_latency_fits(&p, 10000000));
    CHECK(port_set_rate(&p, 96000) == 0);    /* 512 frames: the same time */
    CHECK(sub_latency_fits(&p, 5333334) && !sub_latency_fits(&p, 5343750));
    teardown();

    TEST("a 64-frame build (MAXLAT 1): 10 ms does not fit, 1 ms and the 64-frame boundary do");
    port_defaults(&p); p.cfg_ip = IP4(192, 0, 2, 10);
    model_cfg_t m = {0}; m.phy_fd = -1; m.maxlat = 1; m.jb_frames = 64;
    m.port_base = m.clock_base = 0x1000; m.dclk_base = m.cs2k_base = 0xffffffffu;
    portd_opts_t o = {0}; o.log_sync = -2;
    CHECK(portd_start(&p, model_open(&m), &o) == 0);
    CHECK((port_rd(p.hw, INF_JB_GEOM) & 0xFFFF) == 64);
    CHECK(port_set_latency(&p, 1) == 0);
    CHECK(sub_latency_fits(&p, 1000000) && sub_latency_fits(&p, 1333334));
    CHECK(!sub_latency_fits(&p, 1354167) && !sub_latency_fits(&p, 10000000));
    teardown();
}

static void test_rate_transitions(void)
{
    TEST("rate changes stop old unicast flows and restart desired subscriptions"); setup();
    int code, f = request(1,8,50000,&code); CHECK(f >= 0);
    int16_t channel = 0; CHECK(txflow_create_mcast(&p,1,&channel,1) == 0);
    p.tx[1].programmed = p.tx[1].mc_state = 1;
    CHECK(sub_subscribe(&p,0,"01","peer") == 0);
    p.sub[0].flow = p.sub[0].slot = 0; p.sub[0].status = SUB_RX_UNICAST;
    p.rx[0].used = p.rx[0].state = 1; p.rx[0].nch = 1; p.rx[0].col[0] = 0; p.rx[0].bps = 3;
    CHECK(port_set_rate(&p,96000) == 0); CHECK(p.fs == 96000 && p.channels == 32);
    CHECK(!p.rx[0].used && p.sub[0].active && p.sub[0].flow == -1);
    CHECK(p.sub[0].status == SUB_UNRESOLVED && p.resolve_now);
    if (f >= 0) CHECK(!p.tx[f].used);
    CHECK(tbl_read(p.hw,TBL_RX(0,0)) == 0);
    CHECK(p.tx[1].used && p.tx[1].multicast && p.tx[1].programmed);
    CHECK(tbl_read(p.hw,TBL_TX(1,0)) & 1);
    teardown();
    TEST("a rate change that overfills retained multicast bundles is refused atomically"); setup();
    CHECK(port_set_latency(&p,3) == 0); int16_t ch[16]; for (int i = 0; i < 16; i++) ch[i] = (int16_t)i;
    for (int f = 0; f < 4; f++) CHECK(txflow_create_mcast(&p,f,ch,16) == 0);
    CHECK(port_set_rate(&p,96000) == -1); CHECK(p.fs == 48000 && port_rd(p.hw,INF_MEDIA_FS) == 48000);
    CHECK(p.tx[0].used && p.channels == 64);
    teardown();
}
static void legacy_query(net_t *n, int peer, int port, int id)
{
    uint8_t frame[256] = {0}; uint8_t *ip = frame + 14, *u = ip + 20, *d = u + 8;
    int pos = 12; put16(d,(uint16_t)id); put16(d + 4,1);
    pos += dns_put_name(d + pos,200 - pos,"_netaudio-chan._udp.local");
    put16(d + pos,12); put16(d + pos + 2,1); pos += 4;
    put16(frame + 12,0x0800); ip[0] = 0x45; ip[9] = 17; put16(ip + 2,(uint16_t)(28 + pos));
    put32(ip + 12,IP4(192,0,2,peer)); put32(ip + 16,MDNS_GROUP); put16(ip + 10,inet_csum(ip,20,0));
    put16(u,(uint16_t)port); put16(u + 2,5353); put16(u + 4,(uint16_t)(8 + pos));
    stamp_t ts = {0}; net_input(n,frame,42 + pos,&ts);
}
/* a port on a model whose INF_CAPS is caps, at rate fs */
static int setup_caps(uint32_t caps, int fs)
{
    port_defaults(&p); p.cfg_ip = IP4(192, 0, 2, 10); p.fs = fs;
    model_cfg_t m = {0}; m.phy_fd = -1; m.maxlat = 3; m.caps = caps;
    m.port_base = m.clock_base = 0x1000; m.dclk_base = m.cs2k_base = 0xffffffffu;
    portd_opts_t o = {0}; o.log_sync = -2;
    return portd_start(&p, model_open(&m), &o);
}
static int arc(uint16_t op, const uint8_t *c, int clen, uint8_t *r, int max)
{
    uint8_t q[64];
    return arc_handle(&p, q, rr_make(q, 0x27ff, 0x1234, op, 0, c, clen), r, max);
}
/* _netaudio-chan answers to one unicast PTR query (one per default-named TX channel) */
static uint64_t chan_answers(void)
{
    for (int i = 0; i < 2200; i++) { now += 1000000ull; port_tick(&p, now); }   /* the announcements */
    mdns_stats_t a, b; mdns_get_stats(p.mdns, &a);
    legacy_query(p.net, 11, 6000, 1);
    for (int i = 0; i < 100; i++) { now += 10000000ull; port_tick(&p, now); }
    mdns_get_stats(p.mdns, &b);
    return b.tx_responses - a.tx_responses;
}
static void test_small_builds(void)
{
    uint8_t r[1500]; int code;
    static const uint8_t pg1[4] = { 0, 0, 0, 1 };
    TEST("full build: 64 TX channels advertised (the reference for the smaller builds)");
    CHECK(setup_caps(0, 48000) == 0); CHECK(chan_answers() == 64); teardown();

    TEST("receive-only build (2 RX flows x 8 ch, 2 columns, 0 TX flows): starts with no TX channels or TX flows");
    CHECK(setup_caps(0x02000802u, 48000) == 0);
    CHECK(p.channels == 2 && port_tx_channels(&p) == 0);
    CHECK(arc(0x1000, NULL, 0, r, sizeof(r)) == 44);
    CHECK(get16(r + 12) == 0 && get16(r + 14) == 2 && get16(r + 18) == 2);
    CHECK(get16(r + 22) == 0 && get16(r + 24) == 2 && get16(r + 26) == 2);
    arc(0x2000, pg1, 4, r, sizeof(r)); CHECK(r[11] == 0);   /* no TX channels listed */
    arc(0x2200, pg1, 4, r, sizeof(r)); CHECK(r[11] == 0);
    CHECK(request(1, 8, 50000, &code) < 0 && code != 1);
    CHECK(txflow_create_mcast(&p, 0, (const int16_t[]){0}, 1) == -1);
    int used = 0; for (int f = 0; f < NTXF; f++) used += p.tx[f].used; CHECK(used == 0);
    CHECK(chan_answers() == 0);
    CHECK(sub_subscribe(&p, 1, "01", "peer") == 0 && sub_subscribe(&p, 2, "01", "peer") == -1);
    teardown();

    TEST("small build (1 TX / 2 RX flows, 8 ch per flow, 2 columns): ARC reports it, larger requests are refused");
    CHECK(setup_caps(0x02010802u, 48000) == 0);
    CHECK(arc(0x1000, NULL, 0, r, sizeof(r)) == 44);
    CHECK(get16(r + 12) == 2 && get16(r + 14) == 2 && get16(r + 18) == 2);
    CHECK(get16(r + 22) == 1 && get16(r + 24) == 2 && get16(r + 26) == 4);
    CHECK(chan_answers() == 2);
    char out[128];
    CHECK(ctl_exec(&p, "channels 3", out, sizeof(out)) == -1);
    request(9, 8, 50000, &code); CHECK(code == 0x0302);   /* more than 8 channels per flow */
    request(3, 8, 50000, &code); CHECK(code == 0x0302);   /* TX channel 3 of 2 */
    CHECK(request(2, 8, 50000, &code) >= 0);
    CHECK(request(1, 8, 50001, &code) < 0 && code == 0x0315);   /* the one TX flow is taken */
    CHECK(txflow_create_mcast(&p, 1, (const int16_t[]){0}, 1) == -1);
    teardown();

    TEST("96 kHz plays half the build's columns, not half of MAX_CH");
    CHECK(setup_caps(0x10102028u, 96000) == 0); CHECK(p.channels == 20);   /* 40 columns */
    CHECK(((port_rd(p.hw, INF_CTRL) >> 8) & 0x7F) == 20);
    char o2[64]; CHECK(ctl_exec(&p, "channels 21", o2, sizeof(o2)) == -1);
    teardown();
    CHECK(setup_caps(0x02010802u, 48000) == 0);
    CHECK(port_set_rate(&p, 96000) == 0); CHECK(p.channels == 1);
    teardown();

    TEST("ARC 0x1000 offsets 2/4/16 (TX / RX / total channels) follow the build: full, TX-only, RX-only, small");
    static const struct { uint32_t caps; int tx, rx; } bc[] = {
        { 0, 64, 64 }, { 0x00020804u, 4, 0 }, { 0x02000802u, 0, 2 }, { 0x02010802u, 2, 2 } };
    for (size_t i = 0; i < sizeof(bc) / sizeof(bc[0]); i++) {
        CHECK(setup_caps(bc[i].caps, 48000) == 0);
        CHECK(arc(0x1000, NULL, 0, r, sizeof(r)) == 44);
        CHECKF(get16(r + 12) == bc[i].tx && get16(r + 14) == bc[i].rx && get16(r + 26) == bc[i].tx + bc[i].rx,
               "caps %08x: TX %d RX %d total %d, want %d/%d", bc[i].caps, get16(r + 12), get16(r + 14), get16(r + 26),
               bc[i].tx, bc[i].rx);
        teardown();
    }
}
static void test_distinct_discovery_clients(void)
{
    TEST("pending unicast replies retain distinct clients, ports and transaction IDs");
    model_cfg_t cfg = {0}; cfg.phy_fd = -1;
    hw_t *h = model_open(&cfg); uint8_t mac[6] = {2,0,0,0,0,1}; net_t *n = net_new(h,mac);
    net_set_ip(n,IP4(192,0,2,10),IP4(255,255,255,0),0); mdns_t *m = mdns_new(n);
    mdns_svc_t svc = {0}; strcpy(svc.inst,"01@desk"); strcpy(svc.type,"_netaudio-chan"); svc.port = 4455;
    CHECK(mdns_add(m,&svc) >= 0); mdns_tick(m,now); now += 1000000000ull; mdns_tick(m,now);
    mdns_stats_t before, after; mdns_get_stats(m,&before);
    legacy_query(n,11,6000,1); legacy_query(n,12,6000,1); legacy_query(n,11,6001,1); legacy_query(n,11,6000,2);
    legacy_query(n,11,6000,1); // exact duplicate must not consume a second slot
    for (int i = 0; i < 100; i++) { now += 10000000ull; mdns_tick(m,now); }
    mdns_get_stats(m,&after); CHECK(after.tx_responses - before.tx_responses == 4);
    mdns_free(m); net_free(n); h->ops->close(h);
}

int main(void)
{
    g_log_level = LOG_ERR; g_now_ns = vnow;
    test_reused_flow_budget(); test_handle_update_budget(); test_save_failure(); test_save_retry();
    test_mac_refresh(); test_control_backpressure();
    test_control_eof(); test_control_errors(); test_capabilities(); test_discovery_flood();
    test_cancel_pending(); test_latency_transitions(); test_latency_fit(); test_rate_transitions(); test_distinct_discovery_clients();
    test_small_builds();
    return t_done("test_edges");
}
