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
    CHECK(len == 3 && !memcmp(d, "abc", 3));
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
    u[8] ^= 1;
    put16(u + 6, 0);
    net_input(n, f, 45, &ts); CHECK(received == 2);
    TEST("malformed IPv4/UDP lengths, header checksum and fragments never reach a callback");
    uint8_t good[45]; memcpy(good, f, sizeof(good));
    for (int i = 0; i < 9; i++) {
        memcpy(f, good, sizeof(good));
        int len = 45;
        switch (i) {
        case 0: len = 33; break;                         /* incomplete IP header */
        case 1: put16(f + 16, 19); break;               /* total below IHL */
        case 2: len = 44; break;                       /* IP payload missing */
        case 3: f[14] = 0x44; break;                   /* IHL below minimum */
        case 4: f[14] = 0x4f; break;                   /* options missing */
        case 5: f[22] ^= 1; break;                     /* bad IP checksum */
        case 6: put16(f + 20, 0x2000); break;          /* more fragments */
        case 7: put16(f + 38, 7); break;               /* UDP length below header */
        case 8: put16(f + 38, 12); break;              /* UDP length beyond IP payload */
        }
        if (i == 1 || i == 3 || i == 6) {
            put16(f + 24, 0); put16(f + 24, inet_csum(f + 14, 20, 0));
        }
        net_input(n, f, len, &ts); CHECKF(received == 2, "malformed case %d dispatched", i);
    }
    TEST("IPv4 options do not shift the UDP callback payload incorrectly");
    ipv4(f, 49, 17); f[14] = 0x46;
    memcpy(f + 38, good + 34, 11);
    put16(f + 24, 0); put16(f + 24, inet_csum(f + 14, 24, 0));
    net_input(n, f, 49, &ts); CHECK(received == 3);
    net_free(n); h->ops->close(h);
    return t_done("test_net");
}
