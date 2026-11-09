/* flows_tx.c - the transmit side: flow requests from receivers (UDP 4455; upstream flows_control_server.rs), the TX
 * flow bookkeeping (handles, keepalive expiry; flows_tx.rs `add_flow`, `remove_flow`, `set_channels`, 4 s timeout),
 * multicast bundles (tx_multicasts.rs: 239.255.x.y : 4321, an mDNS address check first) - programmed into the FPGA's
 * TX flow table (fpga/rtl/inferno_tx.v; fpga/docs/inferno.md section 6). Translated from upstream Inferno
 * (github.com/teodly/inferno, GPL-3.0-or-later, commit 9767558e). Part of OpenProSeries inferno-portd.
 * Licence: GPL-3.0-or-later.
 */
#include "portd.h"
#include "regs.h"
#include "util.h"

#include <stdio.h>

void port_adv_channel(port_t *p, int k);
void port_adv_bundle(port_t *p, int f);

/* errors (flows_control.rs) */
#define FC_FLOW_NOT_FOUND 0x0103
#define FC_TOO_MANY       0x0315
#define FC_RATE_MISMATCH  0x0301
#define FC_BAD_REQUEST    0x0302

static int mtu_fpp(int nchb) { return nchb <= 45 ? 32 : nchb <= 91 ? 16 : 8; }   /* inferno_lat.vh inferno_mtu_fpp */

int tx_eff_fpp(const port_t *p, int fpp, int nch, int bps)
{
    int f = fpp ? fpp : port_auto_fpp(p);
    if (f > FPP_MAX_ADV) f = FPP_MAX_ADV;
    int lim = mtu_fpp(nch * bps);
    return f < lim ? f : lim;
}

/* wire bits per second of a flow (fpga/docs/inferno.md section 5: 75 bytes of overhead per packet) */
static double flow_bps(const port_t *p, int nch, int fpp, int bps)
{
    int f = tx_eff_fpp(p, fpp, nch, bps);
    return (75.0 + (double)f * nch * bps) * 8.0 * (double)p->fs / (double)f;
}

double tx_load_bps(const port_t *p, int nch, int fpp, int bps)
{
    double s = nch ? flow_bps(p, nch, fpp, bps) : 0;
    for (int f = 0; f < NTXF; f++) if (p->tx[f].used) s += flow_bps(p, p->tx[f].nch, p->tx[f].fpp, p->tx[f].bps);
    return s;
}

static void tx_program(port_t *p, int f)
{
    txflow_t *t = &p->tx[f];
    uint32_t w[13];
    memset(w, 0, sizeof(w));
    w[1] = ((uint32_t)t->mac[0] << 8) | t->mac[1];
    w[2] = get32(t->mac + 2);
    w[3] = t->dst_ip;
    w[4] = ((uint32_t)t->src_port << 16) | t->dst_port;
    for (int c = 0; c < MAXFCH; c++) {
        uint32_t b = c < t->nch && t->ch[c] >= 0 && t->ch[c] < p->channels ? (uint32_t)t->ch[c] : MAP_SILENT;
        w[5 + c / 4] |= b << (8 * (c % 4));
    }
    /* w1..w12, then w0 (it starts the flow) */
    tbl_write(p->hw, TBL_TX(f, 1), w + 1, 12);
    uint32_t w0 = TXW0(t->multicast ? 32 : 64, 0xB8, t->nch, t->fpp > FPP_MAX_ADV ? FPP_MAX_ADV : t->fpp, t->bps == 4, 1);
    tbl_write(p->hw, TBL_TX(f, 0), &w0, 1);
    t->programmed = 1;
}

static void tx_disable(port_t *p, int f)
{
    uint32_t z = 0;
    tbl_write(p->hw, TBL_TX(f, 0), &z, 1);
    p->tx[f].programmed = 0;
}

void txflows_reanchor(port_t *p)
{
    for (int f = 0; f < NTXF; f++) if (p->tx[f].programmed) tx_program(p, f);
}

static void on_keepalive(void *ctx, uint32_t sip, uint16_t sport, uint32_t dip, uint16_t dport, const uint8_t *d, int n,
                         const stamp_t *ts)
{
    (void)sip; (void)sport; (void)dip; (void)d; (void)n; (void)ts;
    port_t *p = ctx;
    for (int f = 0; f < NTXF; f++)
        if (p->tx[f].used && !p->tx[f].multicast && p->tx[f].src_port == dport) p->tx[f].expires = g_now() + KEEPALIVE_TIMEOUT_NS;
}

static void tx_free(port_t *p, int f)
{
    txflow_t *t = &p->tx[f];
    if (!t->used) return;
    if (t->programmed) tx_disable(p, f);
    net_udp_unbind(p->net, t->src_port);
    memset(t, 0, sizeof(*t));
}

/* a unicast flow for a receiver: the same destination reuses its flow (upstream: "setting channels instead") */
static int tx_add_unicast(port_t *p, const char *host, const char *flow, uint32_t ip, uint16_t port, const int16_t *ch, int nch,
                          int fpp, int bps, int *code)
{
    for (int f = 0; f < NTXF; f++)
        if (p->tx[f].used && !p->tx[f].multicast && p->tx[f].dst_ip == ip && p->tx[f].dst_port == port) {
            txflow_t *t = &p->tx[f];
            double load = tx_load_bps(p, nch, fpp, bps) - flow_bps(p, t->nch, t->fpp, t->bps);
            if (load > TX_BUDGET_BPS) { *code = FC_TOO_MANY; return -1; }
            t->nch = nch; t->fpp = fpp; t->bps = bps;
            for (int c = 0; c < MAXFCH; c++) t->ch[c] = c < nch ? ch[c] : -1;
            t->expires = g_now() + KEEPALIVE_TIMEOUT_NS;
            if (t->programmed) tx_program(p, f);
            return f;
        }
    if (tx_load_bps(p, nch, fpp, bps) > TX_BUDGET_BPS) { *code = FC_TOO_MANY; LOGW("flow request refused: over the 95 Mb/s budget"); return -1; }
    for (int f = 0; f < NTXF; f++)
        if (!p->tx[f].used) {
            txflow_t *t = &p->tx[f];
            memset(t, 0, sizeof(*t));
            int sp = net_udp_bind(p->net, 0, on_keepalive, p);
            if (sp < 0) { *code = FC_TOO_MANY; return -1; }
            t->used = 1;
            t->cookie = (uint16_t)(rnd32() | 1);
            snprintf(t->rx_host, sizeof(t->rx_host), "%s", host);
            snprintf(t->rx_flow, sizeof(t->rx_flow), "%s", flow);
            t->dst_ip = ip; t->dst_port = port; t->src_port = (uint16_t)sp;
            t->nch = nch; t->fpp = fpp; t->bps = bps;
            for (int c = 0; c < MAXFCH; c++) t->ch[c] = c < nch ? ch[c] : -1;
            t->expires = g_now() + KEEPALIVE_TIMEOUT_NS;
            if (net_mac_of(p->net, ip, t->mac)) tx_program(p, f);
            return f;
        }
    *code = FC_TOO_MANY;
    return -1;
}

static int handle_flow(const port_t *p, const uint8_t *h, int *f)
{
    uint32_t idx = get32(h);
    uint16_t ck = get16(h + 4);
    if (idx >= NTXF || !p->tx[idx].used || p->tx[idx].multicast || p->tx[idx].cookie != ck) return 0;
    *f = (int)idx;
    return 1;
}

int flowctl_handle(port_t *p, uint32_t sip, const uint8_t *req, int len, uint8_t *out, int max)
{
    (void)sip;
    if (len < RR_HDR || get16(req + 8) != 0) return 0;
    int tl = get16(req + 2);
    if (tl >= RR_HDR && tl < len) len = tl;
    const uint8_t *c = req + RR_HDR;
    int clen = len - RR_HDR;
    uint16_t op = get16(req + 6), code = 1;
    uint8_t body[8];
    int blen = 0;
    switch (op) {
    case 0x0100: {   /* request flow */
        if (clen <= 16) return 0;
        uint16_t host_off = get16(c);
        uint32_t rate = get32(c + 2), bits = get32(c + 6);
        int nch = get16(c + 12);
        uint16_t rem_off = get16(c + 14);
        int foff = 16 + nch * 2 + 6;
        if (nch < 1 || foff + 4 > clen) { code = FC_BAD_REQUEST; break; }
        int fpp = get16(c + foff);
        uint16_t flow_off = get16(c + foff + 2);
        char host[64] = "", flow[64] = "";
        str_at(req, (size_t)len, host_off, host, sizeof(host));
        str_at(req, (size_t)len, flow_off, flow, sizeof(flow));
        if ((int)rem_off + 8 > len) return 0;
        uint16_t rport = get16(req + rem_off + 2);
        uint32_t rip = get32(req + rem_off + 4);
        int16_t ch[MAXFCH];
        int bad = nch > MAXFCH;
        for (int i = 0; i < nch && i < MAXFCH; i++) {
            uint16_t id = get16(c + 16 + 2 * i);
            ch[i] = id ? (int16_t)(id - 1) : -1;
            if (id && id - 1 >= p->channels) bad = 1;
        }
        char ipb[16];
        LOGI("%s requests flow %s: %d ch, %u Hz, %u bit, fpp %d, to %s:%u", host, flow, nch, rate, bits, fpp, ip_str(rip, ipb), rport);
        if (bad) { code = FC_BAD_REQUEST; break; }
        if (rate != (uint32_t)p->fs) { code = FC_RATE_MISMATCH; break; }
        if (bits != 24 && bits != 32) { code = FC_BAD_REQUEST; break; }   /* 16 bit: no TX dither yet (inferno.md 11.8) */
        if (fpp > 256) { code = FC_BAD_REQUEST; break; }
        if (fpp && fpp < FPP_MIN) fpp = FPP_MIN;
        if (fpp > FPP_MAX_ADV) fpp = FPP_MAX_ADV;   /* bounded to the advertised 2..32 */
        int err = 0;
        int f = tx_add_unicast(p, host, flow, rip, rport, ch, nch, fpp, (int)bits / 8, &err);
        if (f < 0) { code = (uint16_t)err; break; }
        put32(body, (uint32_t)f);
        put16(body + 4, p->tx[f].cookie);
        blen = 6;
        break;
    }
    case 0x0101: {   /* stop flow */
        int f;
        if (clen < 6) return 0;
        if (!handle_flow(p, c, &f)) { code = FC_FLOW_NOT_FOUND; break; }
        LOGI("flow %d stopped by its receiver", f + 1);
        tx_free(p, f);
        break;
    }
    case 0x0102: {   /* update flow: handle, u16 count, channel ids */
        int f;
        if (clen < 8) return 0;
        if (!handle_flow(p, c, &f)) { code = FC_FLOW_NOT_FOUND; break; }
        int nch = get16(c + 6);
        if (nch < 1 || nch > MAXFCH || 8 + 2 * nch > clen) { code = FC_BAD_REQUEST; break; }
        txflow_t *t = &p->tx[f];
        double load = tx_load_bps(p, nch, t->fpp, t->bps) - flow_bps(p, t->nch, t->fpp, t->bps);
        if (load > TX_BUDGET_BPS) { code = FC_TOO_MANY; break; }
        t->nch = nch;
        for (int i = 0; i < MAXFCH; i++) {
            uint16_t id = i < nch ? get16(c + 8 + 2 * i) : 0;
            t->ch[i] = id && id - 1 < p->channels ? (int16_t)(id - 1) : -1;
        }
        if (t->programmed) tx_program(p, f);   /* a table rewrite restarts the flow (inferno.md 11.10) */
        break;
    }
    default:
        LOGD("flow control: unknown opcode 0x%04x", op);
        return 0;
    }
    return rr_make(out, get16(req), get16(req + 4), op, code, body, code == 1 ? blen : 0) * (max >= RR_HDR + blen);
}

static void on_flowctl(void *ctx, uint32_t sip, uint16_t sport, uint32_t dip, uint16_t dport, const uint8_t *d, int n,
                       const stamp_t *ts)
{
    (void)dip; (void)ts;
    port_t *p = ctx;
    uint8_t out[64];
    int r = flowctl_handle(p, sip, d, n, out, sizeof(out));
    if (r > 0) net_udp_send(p->net, dport, sip, sport, out, r, NULL);
}

void flowctl_init(port_t *p) { net_udp_bind(p->net, FLOWCTL_PORT, on_flowctl, p); }

/* ---- multicast bundles */
static void mc_pick(port_t *p, txflow_t *t)
{
    for (;;) {
        uint32_t ip = IP4(239, 255, rnd32() & 255, rnd32() & 255);
        int dup = 0;
        for (int f = 0; f < NTXF; f++) if (p->tx[f].used && &p->tx[f] != t && p->tx[f].dst_ip == ip) dup = 1;
        if (!dup && (ip & 255) != 255 && (ip & 255) != 0) { t->dst_ip = ip; break; }
    }
    t->dst_port = MEDIA_PORT;
    t->mc_state = 0;
    t->mc_probes = 0;
    t->mc_t = 0;
}

static void rev_name(uint32_t ip, char *out, size_t sz)
{
    snprintf(out, sz, "%u.%u.%u.%u.in-addr.local", ip & 255, (ip >> 8) & 255, (ip >> 16) & 255, ip >> 24);
}

int txflow_create_mcast(port_t *p, int f, const int16_t *ch, int nch)
{
    if (f < 0 || f >= NTXF || p->tx[f].used || nch < 1 || nch > MAXFCH) return -1;
    if (tx_load_bps(p, nch, 0, 3) > TX_BUDGET_BPS) { LOGW("multicast flow refused: over the budget"); return -1; }
    txflow_t *t = &p->tx[f];
    memset(t, 0, sizeof(*t));
    int sp = net_udp_bind(p->net, 0, on_keepalive, p);
    if (sp < 0) return -1;
    t->used = 1; t->multicast = 1; t->src_port = (uint16_t)sp;
    t->nch = nch; t->fpp = 0; t->bps = 3;
    for (int c = 0; c < MAXFCH; c++) t->ch[c] = c < nch ? ch[c] : -1;
    mc_pick(p, t);
    p->dirty = 1;
    LOGI("multicast flow %d: %d channels, checking 239.255.x.y", f + 1, nch);
    return 0;
}

int txflow_delete_mcast(port_t *p, int f)
{
    if (f < 0 || f >= NTXF || !p->tx[f].used || !p->tx[f].multicast) return -1;
    txflow_t *t = &p->tx[f];
    char inst[96], rev[64];
    snprintf(inst, sizeof(inst), "%d@%s", f + 1, p->name);
    mdns_remove(p->mdns, inst, "_netaudio-bund");
    snprintf(rev, sizeof(rev), "%u.%u.%u.%u", t->dst_ip & 255, (t->dst_ip >> 8) & 255, (t->dst_ip >> 16) & 255, t->dst_ip >> 24);
    mdns_remove(p->mdns, rev, "in-addr");
    int16_t ch[MAXFCH];
    int nch = t->nch, active = t->mc_state == 1;
    memcpy(ch, t->ch, sizeof(ch));
    tx_free(p, f);
    if (active) for (int c = 0; c < nch; c++) if (ch[c] >= 0) port_adv_channel(p, ch[c]);   /* drop the b.N keys */
    p->dirty = 1;
    return 0;
}

void txflows_tick(port_t *p, uint64_t now)
{
    for (int f = 0; f < NTXF; f++) {
        txflow_t *t = &p->tx[f];
        if (!t->used) continue;
        if (!t->multicast) {
            if (now > t->expires) {
                char b[16];
                LOGI("flow %d to %s:%u expired (no keepalives)", f + 1, ip_str(t->dst_ip, b), t->dst_port);
                tx_free(p, f);
                continue;
            }
            uint8_t mac[6];
            if (net_mac_of(p->net, t->dst_ip, mac) && (!t->programmed || memcmp(mac, t->mac, sizeof(mac)))) {
                memcpy(t->mac, mac, sizeof(mac));
                tx_program(p, f);
            }
            continue;
        }
        if (t->mc_state == 0 && net_ready(p->net)) {
            /* upstream is_multicast_ip_already_used: 3 A queries 400 ms apart, then reserve and look again */
            char rev[64];
            rev_name(t->dst_ip, rev, sizeof(rev));
            uint32_t other;
            if (mdns_a(p->mdns, rev, &other)) { LOGW("multicast address in use, picking another"); mdns_forget(p->mdns, rev); mc_pick(p, t); continue; }
            if (now >= t->mc_t) {
                if (t->mc_probes < 3) { mdns_forget(p->mdns, rev); mdns_query(p->mdns, rev, 1); t->mc_probes++; t->mc_t = now + 400000000ull; }
                else {
                    t->mc_state = 1;
                    net_mac_of(p->net, t->dst_ip, t->mac);
                    tx_program(p, f);
                    char b[16];
                    LOGI("multicast flow %d active on %s:%u", f + 1, ip_str(t->dst_ip, b), t->dst_port);
                    port_adv_bundle(p, f);
                    for (int c = 0; c < t->nch; c++) if (t->ch[c] >= 0) port_adv_channel(p, t->ch[c]);
                }
            }
        }
    }
}
