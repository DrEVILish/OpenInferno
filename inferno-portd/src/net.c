/* net.c - see net.h. Written from the public standards: IEEE 802.3, RFC 826 (ARP), RFC 791 (IPv4), RFC 768 (UDP),
 * RFC 1071 (checksum), RFC 1112 (multicast MAC mapping), RFC 2236 (IGMPv2), RFC 2113 (router alert), RFC 3927
 * (link-local), RFC 5227 (ARP probes), RFC 792 (ICMP echo). Part of OpenProSeries inferno-portd.
 * Licence: GPL-3.0-or-later.
 */
#include "net.h"
#include "util.h"

#include <stdlib.h>

#define NARP   64
#define NSOCK  48
#define NGRP   32
#define NPEND  16
#define EPH_LO 49152
#define NTXQ   256
#define TX_BUDGET 8   /* frames per 1 ms poll: a burst must not overrun the far end's 16-frame CPU RX queue */

typedef struct { uint32_t ip; uint8_t mac[6]; uint8_t valid; uint64_t seen, asked; int tries; } arp_t;
typedef struct { uint16_t port; udp_cb cb; void *ctx; } sock_t;
typedef struct { uint32_t group; uint64_t report_at; int reports_left; } grp_t;
typedef struct { uint8_t f[1514]; int len; uint32_t nexthop; uint64_t since; txopt_t o; } pend_t;

struct net {
    hw_t    *h;
    uint8_t  mac[6];
    uint32_t ip, mask, gw;
    int      ready;
    /* link-local */
    uint32_t ll_cand;
    int      ll_probes, ll_tries;
    uint64_t ll_next;
    arp_t    arp[NARP];
    sock_t   s[NSOCK];
    grp_t    g[NGRP];
    pend_t   pend[NPEND];
    uint16_t ipid, eph;
    struct { uint8_t f[1514]; int len; txopt_t o; int has_o; } txq[NTXQ];
    int      txq_head, txq_n, budget;
    txstamp_cb tscb;
    void    *tsctx;
    net_stats_t st;
};

uint16_t inet_csum(const uint8_t *p, int n, uint32_t s)
{
    for (int i = 0; i + 1 < n; i += 2) s += get16(p + i);
    if (n & 1) s += (uint32_t)p[n - 1] << 8;
    while (s >> 16) s = (s & 0xFFFF) + (s >> 16);
    return (uint16_t)~s;
}

net_t *net_new(hw_t *h, const uint8_t mac[6])
{
    net_t *n = calloc(1, sizeof(*n));
    n->h = h;
    memcpy(n->mac, mac, 6);
    n->eph = EPH_LO + (uint16_t)(rnd32() % 8192);
    n->ipid = (uint16_t)rnd32();
    n->budget = TX_BUDGET;
    return n;
}
void net_free(net_t *n) { free(n); }

static uint32_t ll_pick(net_t *n)
{
    uint32_t hsh = 2166136261u;
    for (int i = 0; i < 6; i++) hsh = (hsh ^ n->mac[i]) * 16777619u;
    hsh = hsh * 2654435761u + (uint32_t)n->ll_tries * 40503u;
    uint32_t host = 0x100 + (hsh % (0xFEFF - 0x100 + 1));   /* 169.254.1.0 .. 169.254.254.255 */
    return IP4(169, 254, 0, 0) | host;
}

void net_set_ip(net_t *n, uint32_t ip, uint32_t mask, uint32_t gw)
{
    n->ip = ip; n->mask = mask; n->gw = gw;
    n->ready = ip != 0;
    if (!ip) {
        n->mask = IP4(255, 255, 0, 0); n->gw = 0;
        n->ll_cand = ll_pick(n);
        n->ll_probes = 0;
        n->ll_next = 0;
    }
}
int net_ready(const net_t *n) { return n->ready; }
uint32_t net_ip(const net_t *n) { return n->ready ? n->ip : 0; }
uint32_t net_mask(const net_t *n) { return n->mask; }
uint32_t net_gw(const net_t *n) { return n->gw; }
const uint8_t *net_mac(const net_t *n) { return n->mac; }
void net_set_txstamp_cb(net_t *n, txstamp_cb cb, void *ctx) { n->tscb = cb; n->tsctx = ctx; }
void net_get_stats(const net_t *n, net_stats_t *s) { *s = n->st; }

static int raw_send(net_t *n, const uint8_t *f, int len, const txopt_t *o)
{
    uint8_t pad[60];
    if (len < 60) { memcpy(pad, f, (size_t)len); memset(pad + len, 0, (size_t)(60 - len)); f = pad; len = 60; }
    if (n->txq_n == 0 && n->budget > 0 && pktq_send(n->h, f, len, o && o->ts_req, o ? o->tag : 0) == 0) {
        n->budget--;
        n->st.tx++;
        return 0;
    }
    if (n->txq_n >= NTXQ) { n->st.drops++; return -1; }
    int i = (n->txq_head + n->txq_n) % NTXQ;
    memcpy(n->txq[i].f, f, (size_t)len);
    n->txq[i].len = len;
    n->txq[i].has_o = o != NULL;
    if (o) n->txq[i].o = *o;
    n->txq_n++;
    return 0;
}

static void txq_drain(net_t *n)
{
    n->budget = TX_BUDGET;
    while (n->txq_n && n->budget > 0) {
        int i = n->txq_head;
        const txopt_t *o = n->txq[i].has_o ? &n->txq[i].o : NULL;
        if (pktq_send(n->h, n->txq[i].f, n->txq[i].len, o && o->ts_req, o ? o->tag : 0)) break;   /* the FPGA queue is full */
        n->st.tx++;
        n->budget--;
        n->txq_head = (n->txq_head + 1) % NTXQ;
        n->txq_n--;
    }
}

static void arp_send(net_t *n, int op, const uint8_t *tha, uint32_t tpa, uint32_t spa, const uint8_t *dst)
{
    uint8_t f[42];
    memcpy(f, dst, 6); memcpy(f + 6, n->mac, 6); f[12] = 0x08; f[13] = 0x06;
    put16(f + 14, 1); put16(f + 16, 0x0800); f[18] = 6; f[19] = 4; put16(f + 20, (uint16_t)op);
    memcpy(f + 22, n->mac, 6); put32(f + 28, spa);
    memcpy(f + 32, tha, 6); put32(f + 38, tpa);
    n->st.arp_tx++;
    raw_send(n, f, 42, NULL);
}

static arp_t *arp_find(net_t *n, uint32_t ip)
{
    for (int i = 0; i < NARP; i++) if (n->arp[i].ip == ip && (n->arp[i].valid || n->arp[i].asked)) return &n->arp[i];
    return NULL;
}

static arp_t *arp_slot(net_t *n, uint32_t ip)
{
    arp_t *a = arp_find(n, ip);
    if (a) return a;
    int old = 0;
    for (int i = 0; i < NARP; i++) {
        if (!n->arp[i].valid && !n->arp[i].asked) { old = i; break; }
        if (n->arp[i].seen < n->arp[old].seen) old = i;
    }
    memset(&n->arp[old], 0, sizeof(arp_t));
    n->arp[old].ip = ip;
    return &n->arp[old];
}

static void flush_pending(net_t *n, uint32_t ip, const uint8_t *mac)
{
    for (int i = 0; i < NPEND; i++) {
        pend_t *p = &n->pend[i];
        if (p->len && p->nexthop == ip) {
            memcpy(p->f, mac, 6);
            raw_send(n, p->f, p->len, &p->o);
            p->len = 0;
        }
    }
}

static void arp_learn(net_t *n, uint32_t ip, const uint8_t *mac)
{
    if (!ip) return;
    arp_t *a = arp_slot(n, ip);
    memcpy(a->mac, mac, 6);
    a->valid = 1;
    a->seen = g_now();
    a->asked = 0;
    flush_pending(n, ip, mac);
}

static void mcast_mac(uint32_t ip, uint8_t mac[6])
{
    mac[0] = 0x01; mac[1] = 0x00; mac[2] = 0x5E;
    mac[3] = (uint8_t)((ip >> 16) & 0x7F); mac[4] = (uint8_t)(ip >> 8); mac[5] = (uint8_t)ip;
}

static uint32_t nexthop(const net_t *n, uint32_t ip)
{
    if (!n->gw || ((ip ^ n->ip) & n->mask) == 0) return ip;
    if ((ip >> 16) == ((169u << 8) | 254u)) return ip;   /* link-local: always on the link */
    return n->gw;
}

int net_mac_of(net_t *n, uint32_t ip, uint8_t mac[6])
{
    if ((ip >> 28) == 0xE) { mcast_mac(ip, mac); return 1; }
    if (ip == 0xFFFFFFFFu || (n->mask && (ip | n->mask) == 0xFFFFFFFFu)) { memset(mac, 0xFF, 6); return 1; }
    uint32_t nh = nexthop(n, ip);
    arp_t *a = arp_find(n, nh);
    if (a && a->valid) { memcpy(mac, a->mac, 6); return 1; }
    if (!n->ready) return 0;
    uint64_t now = g_now();
    if (!a) { a = arp_slot(n, nh); a->tries = 0; }
    if (!a->asked || (int64_t)(now - a->asked) > 500000000LL) {
        a->asked = now ? now : 1;
        a->tries++;
        static const uint8_t z[6] = { 0 }, bc[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
        arp_send(n, 1, z, nh, n->ip, bc);
    }
    return 0;
}

static int ip_send(net_t *n, uint32_t dip, uint8_t proto, const uint8_t *opt, int optlen, const uint8_t *payload, int plen,
                   const txopt_t *o)
{
    if (!n->ready) return -1;
    int ihl = 20 + optlen;
    if (14 + ihl + plen > 1514) return -1;
    uint8_t f[1514];
    f[6] = 0; memcpy(f + 6, n->mac, 6); f[12] = 0x08; f[13] = 0x00;
    uint8_t *ip = f + 14;
    ip[0] = (uint8_t)(0x40 | (ihl / 4)); ip[1] = o ? o->tos : 0;
    put16(ip + 2, (uint16_t)(ihl + plen)); put16(ip + 4, n->ipid++); put16(ip + 6, 0x4000);
    ip[8] = o && o->ttl ? o->ttl : ((dip >> 28) == 0xE ? 32 : 64);
    ip[9] = proto; put16(ip + 10, 0); put32(ip + 12, n->ip); put32(ip + 16, dip);
    if (optlen) memcpy(ip + 20, opt, (size_t)optlen);
    put16(ip + 10, inet_csum(ip, ihl, 0));
    memcpy(ip + ihl, payload, (size_t)plen);
    int len = 14 + ihl + plen;
    uint8_t mac[6];
    if (net_mac_of(n, dip, mac)) {
        memcpy(f, mac, 6);
        return raw_send(n, f, len, o);
    }
    /* wait for ARP */
    uint32_t nh = nexthop(n, dip);
    for (int i = 0; i < NPEND; i++)
        if (!n->pend[i].len) {
            memcpy(n->pend[i].f, f, (size_t)len);
            n->pend[i].len = len; n->pend[i].nexthop = nh; n->pend[i].since = g_now();
            if (o) n->pend[i].o = *o; else memset(&n->pend[i].o, 0, sizeof(txopt_t));
            return 0;
        }
    n->st.drops++;
    return -1;
}

int net_udp_send(net_t *n, uint16_t sport, uint32_t dip, uint16_t dport, const void *d, int len, const txopt_t *o)
{
    uint8_t u[1480];
    if (len < 0 || len > 1472) return -1;
    put16(u, sport); put16(u + 2, dport); put16(u + 4, (uint16_t)(8 + len)); put16(u + 6, 0);
    memcpy(u + 8, d, (size_t)len);
    /* the pseudo-header sum */
    uint32_t ps = (n->ip >> 16) + (n->ip & 0xFFFF) + (dip >> 16) + (dip & 0xFFFF) + 17 + (uint32_t)(8 + len);
    uint16_t c = inet_csum(u, 8 + len, ps);
    put16(u + 6, c ? c : 0xFFFF);
    n->st.udp_tx++;
    return ip_send(n, dip, 17, NULL, 0, u, 8 + len, o);
}

int net_udp_bind(net_t *n, uint16_t port, udp_cb cb, void *ctx)
{
    if (port == 0) {
        for (int tries = 0; tries < 16384 && !port; tries++) {
            uint16_t p = n->eph++;
            if (n->eph < EPH_LO) n->eph = EPH_LO;
            int used = 0;
            for (int i = 0; i < NSOCK; i++) if (n->s[i].cb && n->s[i].port == p) used = 1;
            if (!used) port = p;
        }
    }
    for (int i = 0; i < NSOCK; i++) if (n->s[i].cb && n->s[i].port == port) return -1;
    for (int i = 0; i < NSOCK; i++)
        if (!n->s[i].cb) { n->s[i].port = port; n->s[i].cb = cb; n->s[i].ctx = ctx; return port; }
    return -1;
}

void net_udp_unbind(net_t *n, uint16_t port)
{
    for (int i = 0; i < NSOCK; i++) if (n->s[i].cb && n->s[i].port == port) memset(&n->s[i], 0, sizeof(sock_t));
}

static void igmp_send(net_t *n, uint8_t type, uint32_t group)
{
    static const uint8_t ra[4] = { 0x94, 0x04, 0x00, 0x00 };
    uint8_t m[8] = { type, 0, 0, 0 };
    put32(m + 4, group);
    put16(m + 2, inet_csum(m, 8, 0));
    txopt_t o = { 1, 0xC0, 0, 0 };
    n->st.igmp_tx++;
    ip_send(n, type == 0x17 ? IP4(224, 0, 0, 2) : group, 2, ra, 4, m, 8, &o);
}

void net_join(net_t *n, uint32_t group)
{
    for (int i = 0; i < NGRP; i++) if (n->g[i].group == group) return;
    for (int i = 0; i < NGRP; i++)
        if (!n->g[i].group) {
            n->g[i].group = group;
            n->g[i].reports_left = 2;   /* RFC 2236: repeat the unsolicited report */
            n->g[i].report_at = g_now();
            return;
        }
}

void net_leave(net_t *n, uint32_t group)
{
    for (int i = 0; i < NGRP; i++)
        if (n->g[i].group == group) {
            if ((group & 0xFFFFFF00u) != IP4(224, 0, 0, 0)) igmp_send(n, 0x17, group);
            n->g[i].group = 0;
        }
}

static void udp_input(net_t *n, uint32_t sip, uint32_t dip, const uint8_t *u, int len, const stamp_t *ts)
{
    if (len < 8) return;
    uint16_t sport = get16(u), dport = get16(u + 2), ulen = get16(u + 4);
    if (ulen < 8 || ulen > len) return;
    n->st.udp_rx++;
    for (int i = 0; i < NSOCK; i++)
        if (n->s[i].cb && n->s[i].port == dport) { n->s[i].cb(n->s[i].ctx, sip, sport, dip, dport, u + 8, ulen - 8, ts); return; }
}

static void arp_input(net_t *n, const uint8_t *f, int len)
{
    if (len < 42 || get16(f + 14) != 1 || get16(f + 16) != 0x0800 || f[18] != 6 || f[19] != 4) return;
    n->st.arp_rx++;
    uint16_t op = get16(f + 20);
    const uint8_t *sha = f + 22;
    uint32_t spa = get32(f + 28), tpa = get32(f + 38);
    if (!memcmp(sha, n->mac, 6)) return;
    if (!n->ready) {
        /* link-local probing: someone uses or probes our candidate */
        if (spa == n->ll_cand || (spa == 0 && tpa == n->ll_cand)) {
            n->ll_tries++;
            n->ll_cand = ll_pick(n);
            n->ll_probes = 0;
            n->ll_next = g_now() + 200000000ull;
        }
        return;
    }
    if (spa == n->ip && spa) {
        /* conflict: defend once (RFC 5227) */
        static const uint8_t bc[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
        arp_send(n, 2, bc, n->ip, n->ip, bc);
        return;
    }
    if (spa) {
        arp_t *a = arp_find(n, spa);
        if (a || tpa == n->ip) arp_learn(n, spa, sha);
    }
    if (op == 1 && tpa == n->ip) arp_send(n, 2, sha, spa, n->ip, sha);
}

void net_input(net_t *n, const uint8_t *f, int len, const stamp_t *ts)
{
    if (len < 14) return;
    n->st.rx++;
    uint16_t et = get16(f + 12);
    if (et == 0x0806) { arp_input(n, f, len); return; }
    if (et != 0x0800 || len < 34) return;
    const uint8_t *ip = f + 14;
    int ihl = (ip[0] & 15) * 4;
    if ((ip[0] >> 4) != 4 || ihl < 20 || 14 + ihl > len) return;
    int tot = get16(ip + 2);
    if (tot < ihl || 14 + tot > len) return;
    if (get16(ip + 6) & 0x3FFF) return;   /* fragments */
    if (inet_csum(ip, ihl, 0) != 0) return;
    uint32_t sip = get32(ip + 12), dip = get32(ip + 16);
    int mc = (dip >> 28) == 0xE;
    if (!mc && dip != 0xFFFFFFFFu && !(n->ready && (dip == n->ip || (n->mask && (dip | n->mask) == 0xFFFFFFFFu)))) return;
    const uint8_t *pl = ip + ihl;
    int plen = tot - ihl;
    switch (ip[9]) {
    case 17: udp_input(n, sip, dip, pl, plen, ts); break;
    case 1:
        if (plen >= 8 && pl[0] == 8 && !mc && n->ready) {
            uint8_t r[1480];
            memcpy(r, pl, (size_t)MIN(plen, 1480));
            r[0] = 0; put16(r + 2, 0); put16(r + 2, inet_csum(r, plen, 0));
            n->st.icmp++;
            ip_send(n, sip, 1, NULL, 0, r, plen, NULL);
        }
        break;
    case 2:
        if (plen >= 8 && pl[0] == 0x11) {   /* membership query: report each group within its response time */
            uint32_t qg = get32(pl + 4);
            uint64_t maxr = (uint64_t)(pl[1] ? pl[1] : 100) * 100000000ull;
            for (int i = 0; i < NGRP; i++)
                if (n->g[i].group && (!qg || qg == n->g[i].group)) {
                    n->g[i].reports_left = 1;
                    n->g[i].report_at = g_now() + (uint64_t)(rnd32() % 1000) * (maxr / 1000);
                }
        }
        break;
    default: break;
    }
}

void net_poll(net_t *n)
{
    uint8_t b[1600];
    stamp_t ts;
    uint8_t tag;
    /* TX stamps first: a reply (a Delay_Resp) can arrive in the same poll as its request's stamp */
    while (pktq_tx_stamp(n->h, &tag, &ts))
        if (n->tscb) n->tscb(n->tsctx, tag, &ts);
    for (int k = 0; k < 64; k++) {
        int len = pktq_recv(n->h, b, sizeof(b), &ts);
        if (len <= 0) break;
        net_input(n, b, len, &ts);
    }
    txq_drain(n);
    while (pktq_tx_stamp(n->h, &tag, &ts))
        if (n->tscb) n->tscb(n->tsctx, tag, &ts);
}

void net_tick(net_t *n, uint64_t now)
{
    if (!n->ready && n->ll_cand) {
        if (now >= n->ll_next) {
            static const uint8_t z[6] = { 0 }, bc[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
            if (n->ll_probes < 3) {
                arp_send(n, 1, z, n->ll_cand, 0, bc);   /* probe: sender IP 0 */
                n->ll_probes++;
                n->ll_next = now + 200000000ull;
            } else {
                n->ip = n->ll_cand;
                n->ready = 1;
                char b[16];
                LOGI("link-local address %s", ip_str(n->ip, b));
                arp_send(n, 1, z, n->ip, n->ip, bc);   /* announce */
                for (int i = 0; i < NGRP; i++) if (n->g[i].group) { n->g[i].reports_left = 2; n->g[i].report_at = now; }
            }
        }
    }
    if (!n->ready) return;
    for (int i = 0; i < NGRP; i++) {
        grp_t *g = &n->g[i];
        if (g->group && g->reports_left > 0 && now >= g->report_at) {
            if ((g->group & 0xFFFFFF00u) != IP4(224, 0, 0, 0)) igmp_send(n, 0x16, g->group);
            g->reports_left--;
            g->report_at = now + 1000000000ull;
        }
    }
    for (int i = 0; i < NPEND; i++)
        if (n->pend[i].len) {
            if ((int64_t)(now - n->pend[i].since) > 3000000000LL) { n->pend[i].len = 0; n->st.drops++; }
            else { uint8_t mac[6]; (void)net_mac_of(n, n->pend[i].nexthop, mac); }
        }
    /* refresh entries in use every 60 s, forget unanswered ones */
    for (int i = 0; i < NARP; i++) {
        arp_t *a = &n->arp[i];
        if (a->valid && (int64_t)(now - a->seen) > 60000000000LL) {
            a->valid = 0;
            a->asked = 0;
        } else if (!a->valid && a->asked && (int64_t)(now - a->asked) > 2000000000LL && a->tries >= 3) {
            memset(a, 0, sizeof(*a));
        }
    }
}
