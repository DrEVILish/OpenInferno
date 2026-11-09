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
int main(void)
{
    g_log_level = LOG_ERR; g_now_ns = vnow;
    test_reused_flow_budget(); test_handle_update_budget(); test_save_failure(); test_save_retry();
    test_mac_refresh(); test_control_backpressure();
    return t_done("test_edges");
}
