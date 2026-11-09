/* Incoming IPv4 bounds and UDP checksum regressions. Licence: GPL-3.0-or-later. */
#include "test.h"
#include "../src/fpga_model.h"
#include "../src/net.h"
#include "../src/util.h"

static int received;
static void on_udp(void *ctx, uint32_t sip, uint16_t sport, uint32_t dip, uint16_t dport,
                   const uint8_t *d, int len, const stamp_t *ts)
{
    (void)ctx; (void)sip; (void)sport; (void)dip; (void)dport; (void)d; (void)len; (void)ts;
    received++;
}

static void ipv4(uint8_t *f, int size, uint8_t proto)
{
    memset(f, 0, (size_t)size);
    put16(f + 12, 0x0800); f[14] = 0x45;
    put16(f + 16, (uint16_t)(size - 14));
    f[22] = 64; f[23] = proto;
    put32(f + 26, IP4(192, 168, 1, 3)); put32(f + 30, IP4(192, 168, 1, 2));
    put16(f + 24, inet_csum(f + 14, 20, 0));
}

int main(void)
{
    model_cfg_t cfg = {0}; cfg.phy_fd = -1;
    hw_t *h = model_open(&cfg);
    const uint8_t mac[6] = {2, 0, 0, 0, 0, 1};
    net_t *n = net_new(h, mac);
    net_set_ip(n, IP4(192, 168, 1, 2), IP4(255, 255, 255, 0), 0);
    CHECK(net_udp_bind(n, 5555, on_udp, NULL) == 5555);
    stamp_t ts = {0}; uint8_t f[1600]; net_stats_t stats;
    TEST("oversized ICMP is rejected before reading its payload");
    ipv4(f, sizeof(f), 1); f[34] = 8;
    net_input(n, f, sizeof(f), &ts);
    net_get_stats(n, &stats); CHECK(stats.icmp == 0);
    TEST("maximum-sized IPv4 ICMP remains supported");
    ipv4(f, 1514, 1); f[34] = 8;
    net_input(n, f, 1514, &ts);
    net_get_stats(n, &stats); CHECK(stats.icmp == 1);
    TEST("valid odd-length UDP, corrupt UDP, and optional zero checksum");
    ipv4(f, 45, 17);
    uint8_t *u = f + 34;
    put16(u, 5000); put16(u + 2, 5555); put16(u + 4, 11);
    memcpy(u + 8, "abc", 3);
    uint32_t sip = get32(f + 26), dip = get32(f + 30);
    uint32_t ps = (sip >> 16) + (sip & 65535) + (dip >> 16) + (dip & 65535) + 17 + 11;
    uint16_t sum = inet_csum(u, 11, ps); put16(u + 6, sum ? sum : 65535);
    net_input(n, f, 45, &ts); CHECK(received == 1);
    u[8] ^= 1;
    net_input(n, f, 45, &ts); CHECK(received == 1);
    put16(u + 6, 0);
    net_input(n, f, 45, &ts); CHECK(received == 2);
    net_free(n); h->ops->close(h);
    return t_done("test_net");
}
