/* info.c - the info and heartbeat multicasts: board / product info and channel-change notifications to
 * 224.0.0.231:8702, a heartbeat each second to 224.0.0.233:8708 (the clock's frequency offset and the RX latency
 * part), answers to info requests on UDP 8700. Translated from upstream Inferno (github.com/teodly/inferno,
 * GPL-3.0-or-later, commit 9767558e): device_server/info_mcast_server.rs and protocol/mcast.rs.
 * Part of OpenProSeries inferno-portd. Licence: GPL-3.0-or-later.
 */
#include "portd.h"
#include "clock.h"
#include "regs.h"
#include "util.h"

#include <stdio.h>

#define DEVINFO_GROUP IP4(224, 0, 0, 231)
#define DEVINFO_PORT  8702
#define HB_GROUP      IP4(224, 0, 0, 233)
#define HB_PORT       8708
#define MCAST_HDR     32

static const char *VENDOR = "OpenProSeries";

int mcast_make(uint8_t *buf, uint16_t start, uint16_t seq, uint16_t process, const uint8_t devid[8], const char *vendor,
               const uint8_t opcode[8], const uint8_t *c, int clen)
{
    put16(buf, start);
    put16(buf + 2, (uint16_t)(MCAST_HDR + clen));
    put16(buf + 4, seq);
    put16(buf + 6, process);
    memcpy(buf + 8, devid, 8);
    memset(buf + 16, ' ', 8);   /* vendor: 8 bytes, upstream fills with spaces then writes the string */
    memcpy(buf + 16, vendor, MIN(strlen(vendor), 8));
    memcpy(buf + 24, opcode, 8);
    if (clen) memcpy(buf + MCAST_HDR, c, (size_t)clen);
    return MCAST_HDR + clen;
}

static void msend(port_t *p, uint32_t grp, uint16_t gport, uint16_t start, const uint8_t op[8], const uint8_t *c, int clen)
{
    uint8_t b[1500];
    if (clen + MCAST_HDR > (int)sizeof(b)) return;
    int n = mcast_make(b, start, p->info_seq, p->pid, p->devid, VENDOR, op, c, clen);
    p->info_seq++;
    net_udp_send(p->net, INFO_PORT, grp, gport, b, n, NULL);
}

static void wstr(uint8_t *c, int off, int max, const char *s) { memcpy(c + off, s, MIN((size_t)max, strlen(s))); }

static void board_info(port_t *p)
{
    uint8_t c[200];
    memset(c, 0, sizeof(c));
    static const uint8_t fw[4] = { 4, 1, 0, 6 }, hw[4] = { 4, 1, 0, 3 }, boot[4] = { 1, 0, 0, 0 };
    memcpy(c, fw, 4); c[0x23] = 2;
    memcpy(c + 4, hw, 4); c[0x27] = 1;
    memcpy(c + 0x28, boot, 4);
    c[0x16] = 0x10;
    c[0xbb] = 0x1f;   /* upstream: "if 0, device is flooded with info multicast requests" */
    wstr(c, 12, 8, "OpenProSeries");
    wstr(c, 0x38, 16, "OpenProSeries");
    static const uint8_t op[8] = { 0x07, 0x2a, 0x00, 0x60, 0, 0, 0, 0 };
    msend(p, DEVINFO_GROUP, DEVINFO_PORT, 0xffff, op, c, 200);
}

static void product_info(port_t *p)
{
    uint8_t c[336];
    memset(c, 0, sizeof(c));
    wstr(c, 0, 8, "OpenProSeries");
    wstr(c, 8, 8, "OpenProSeries");
    wstr(c, 0x2c, 16, "OpenProSeries");
    wstr(c, 0xac, 16, "PRO1 AoIP port");
    static const uint8_t ver[4] = { 0, 1, 0, 0 };
    memcpy(c + 0x1c, ver, 4);
    static const uint8_t op[8] = { 0x07, 0x2a, 0x00, 0xc0, 0, 0, 0, 0 };
    msend(p, DEVINFO_GROUP, DEVINFO_PORT, 0xffff, op, c, 336);
}

static void network_info(port_t *p)
{
    bbuf_t b;
    bb_init(&b);
    static const uint8_t h[6] = { 0, 1, 0, 0, 0, 0 };
    bb_bytes(&b, h, 6);
    bb_u16(&b, 100);   /* link speed, Mb/s */
    bb_u16(&b, 1);
    bb_bytes(&b, p->mac, 6);
    bb_u32(&b, net_ip(p->net));
    bb_u32(&b, net_mask(p->net));
    bb_u32(&b, net_gw(p->net));
    bb_u32(&b, net_gw(p->net));
    static const uint8_t t[4] = { 0x00, 0x18, 0x00, 0x30 };
    bb_bytes(&b, t, 4);
    bb_zero(&b, 28);
    static const uint8_t op[8] = { 0x07, 0x2a, 0x00, 0x11, 0, 0, 0, 0 };
    msend(p, DEVINFO_GROUP, DEVINFO_PORT, 0xffff, op, b.d, (int)b.len);
}

static void heartbeat(port_t *p)
{
    bbuf_t b;
    bb_init(&b);
    int32_t ppb;
    uint16_t ctr = p->info_seq;
    if (p->clk && dclock_freq_ppb(p->clk, &ppb)) {
        bb_u16(&b, 16); bb_u16(&b, 0x8001); bb_u16(&b, 4); bb_u16(&b, 4); bb_u16(&b, ctr); bb_u16(&b, 0);
        bb_u32(&b, (uint32_t)ppb);
        /* RX latency per flow (upstream's 0x8003 part; we report the peak the FPGA measured, frames) */
        bb_u16(&b, (uint16_t)(24 + NRXF * 4)); bb_u16(&b, 0x8003); bb_u16(&b, 4); bb_u16(&b, (uint16_t)(12 + NRXF * 4));
        bb_u16(&b, ctr); bb_u16(&b, 0); bb_u16(&b, NRXF); bb_u16(&b, 0); bb_u16(&b, 24); bb_u16(&b, 0);
        bb_u32(&b, (uint32_t)p->fs);
        for (int f = 0; f < NRXF; f++) {
            uint32_t l = 0;
            if (p->rx[f].used && p->rx[f].state) {
                l = tbl_read(p->hw, TBL_RXST(f, 3));
                uint32_t z = 0;
                tbl_write(p->hw, TBL_RXST(f, 3), &z, 1);
            }
            bb_u32(&b, (int32_t)l < 0 ? 0 : l);
        }
    }
    static const uint8_t op[8] = { 0, 8, 0, 1, 0x10, 0, 0, 0 };
    msend(p, HB_GROUP, HB_PORT, 0xfffe, op, b.d, (int)b.len);
}

void info_channel_change(port_t *p, const int *ch, int n)
{
    if (!net_ready(p->net) || n <= 0) return;
    uint8_t c[2 + MAX_CH / 8 + 1];
    memset(c, 0, sizeof(c));
    int bytes = 1;
    for (int i = 0; i < n; i++) {
        int by = ch[i] / 8;
        if (by + 1 > bytes) bytes = by + 1;
        c[2 + by] |= (uint8_t)(1u << (ch[i] % 8));
    }
    put16(c, (uint16_t)bytes);
    static const uint8_t op[8] = { 0x07, 0x2a, 1, 2, 0, 0, 0, 0 };
    msend(p, DEVINFO_GROUP, DEVINFO_PORT, 0xffff, op, c, 2 + bytes);
}

static void on_info(void *ctx, uint32_t sip, uint16_t sport, uint32_t dip, uint16_t dport, const uint8_t *d, int n,
                    const stamp_t *ts)
{
    (void)sip; (void)sport; (void)dip; (void)dport; (void)ts;
    port_t *p = ctx;
    if (n < 32) return;
    const uint8_t *op = d + 24;
    if (op[0] != 0x07 || op[2] != 0 || op[4] || op[5] || op[6]) return;
    switch (op[3]) {
    case 0x61: board_info(p); break;
    case 0xc1: product_info(p); break;
    case 0x13: network_info(p); break;
    case 0x77: {
        static const uint8_t r[8] = { 0, 0, 0, 3, 0, 0, 0, 0 }, o[8] = { 0x07, 0x2a, 0x00, 0x78, 0, 0, 0, 0 };
        msend(p, DEVINFO_GROUP, DEVINFO_PORT, 0xffff, o, r, 8);
        break;
    }
    default: LOGD("info request opcode %02x%02x%02x%02x", op[0], op[1], op[2], op[3]); break;
    }
}

void info_init(port_t *p)
{
    net_udp_unbind(p->net, INFO_PORT);
    net_udp_bind(p->net, INFO_PORT, on_info, p);
    board_info(p);
    product_info(p);
    p->next_heartbeat = g_now() + 1000000000ull;
}

void info_tick(port_t *p, uint64_t now)
{
    if (!net_ready(p->net) || now < p->next_heartbeat) return;
    heartbeat(p);
    p->next_heartbeat = now + 1000000000ull;
}
