/* subscriber.c - the receive side: our RX channels subscribed to other devices' TX channels. Translated from upstream
 * Inferno (github.com/teodly/inferno, GPL-3.0-or-later, commit 9767558e): device_server/channels_subscriber.rs
 * (resolve_subscriptions: mDNS lookup of "<channel>@<device>", multicast bundles first, reuse of flows and of free
 * slots by "update flow", new flows per server; scan_flows: 2 s without media on a flow older than 6 s orphans its
 * channels), protocol/flows_control.rs (the client: request 0x0100 / stop 0x0101 / update 0x0102, start code 0x1102,
 * 3 s timeout), mdns_client.rs (TXT keys) and flows_rx.rs (keepalive 13 37 to the packets' source every 250 ms).
 * Differences: the media path is the FPGA's RX flow table (fpga/rtl/inferno_rx.v), one column per flow channel; flows
 * carry up to MAXFCH = 32 channels (upstream 8); the flow's SHIFT adds the transmitter's latency above ours.
 * Part of OpenProSeries inferno-portd. Licence: GPL-3.0-or-later.
 */
#include "portd.h"
#include "regs.h"
#include "util.h"

#include <stdio.h>
#include <stdlib.h>
#include <strings.h>

#define FC_TIMEOUT_NS 3000000000ull
#define GATHER_NS     300000000ull
#define GATHER_MAX_NS 3000000000ull

static int mtu_fpp(int nchb) { return nchb <= 45 ? 32 : nchb <= 91 ? 16 : 8; }

int sub_status(const port_t *p, int k) { return k >= 0 && k < MAX_CH && p->sub[k].active ? p->sub[k].status : 0; }

static void notify(port_t *p, int k) { p->changed_rx[k] = 1; p->any_changed = 1; }

static void set_status(port_t *p, int k, int st)
{
    if (p->sub[k].status != st) { p->sub[k].status = st; notify(p, k); }
}

int fc_build_request(const port_t *p, uint16_t seq, uint32_t rate, uint32_t bps, uint16_t fpp, const uint16_t *ch, int nch,
                     uint16_t rx_port, const char *flow_name, uint8_t *out, int max)
{
    bbuf_t b, s;
    bb_init(&b);
    bb_init(&s);
    size_t so = 0x26 + (size_t)nch * 2 + RR_HDR;   /* strings offset (packet-relative) */
    bb_u16(&b, (uint16_t)so);
    bb_str0(&s, p->name);
    uint16_t flow_off = (uint16_t)(s.pos + so);
    bb_str0(&s, flow_name);
    bb_u32(&b, rate);
    bb_u32(&b, bps);
    bb_u16(&b, 1);
    bb_u16(&b, (uint16_t)nch);
    while ((s.pos + so) % 8) bb_u8(&s, 0);
    bb_u16(&b, (uint16_t)(s.pos + so));
    bb_u16(&s, 0x0802);
    bb_u16(&s, rx_port);
    bb_u32(&s, net_ip(p->net));
    for (int i = 0; i < nch; i++) bb_u16(&b, ch[i]);
    bb_u16(&b, (uint16_t)(0x1c + 2 * nch));
    bb_u16(&b, 0x0a00);
    bb_u16(&b, 0x0002);
    bb_u16(&b, fpp);
    bb_u16(&b, flow_off);
    bb_zero(&b, 12);
    bb_bytes(&b, s.d, s.len);
    if (RR_HDR + (int)b.len > max) return 0;
    return rr_make(out, DBCP1, seq, 0x0100, 0, b.d, (int)b.len);
}

static void rx_program(port_t *p, int f)
{
    rxflow_t *r = &p->rx[f];
    uint32_t w[12];
    memset(w, 0, sizeof(w));
    w[1] = r->multicast ? r->group : net_ip(p->net);
    w[2] = r->multicast ? r->gport : r->rx_port;
    w[3] = (uint32_t)r->shift;
    for (int c = 0; c < MAXFCH; c++) {
        uint32_t b = c < r->nch && r->col[c] >= 0 ? (uint32_t)r->col[c] : MAP_SILENT;
        w[4 + c / 4] |= b << (8 * (c % 4));
    }
    tbl_write(p->hw, TBL_RX(f, 1), w + 1, 11);
    uint32_t w0 = RXW0(r->bps, r->nch, 1);
    tbl_write(p->hw, TBL_RX(f, 0), &w0, 1);
}

static void rx_free(port_t *p, int f, int send_stop);

static void fc_send(port_t *p, int f, uint16_t op, const uint8_t *body, int blen)
{
    rxflow_t *r = &p->rx[f];
    uint8_t pk[600];
    r->req_seq = ++p->fc_seq;
    int n = rr_make(pk, DBCP1, r->req_seq, op, 0, body, blen);
    net_udp_send(p->net, (uint16_t)p->fc_sock, r->ctl_ip, r->ctl_port, pk, n, NULL);
    r->req_at = g_now();
}

static void send_request(port_t *p, int f)
{
    rxflow_t *r = &p->rx[f];
    uint8_t pk[600];
    char fname[24];
    snprintf(fname, sizeof(fname), "%d_%u", f + 1, p->pid);
    r->req_seq = ++p->fc_seq;
    int n = fc_build_request(p, r->req_seq, (uint32_t)p->fs, (uint32_t)r->bps * 8, (uint16_t)r->fpp, r->txch, r->nch, r->rx_port,
                             fname, pk, sizeof(pk));
    net_udp_send(p->net, (uint16_t)p->fc_sock, r->ctl_ip, r->ctl_port, pk, n, NULL);
    r->req_at = g_now();
}

static void send_update(port_t *p, int f)
{
    rxflow_t *r = &p->rx[f];
    uint8_t b[8 + 2 * MAXFCH];
    memcpy(b, r->handle, 6);
    put16(b + 6, (uint16_t)r->pend_nch);
    for (int c = 0; c < r->pend_nch; c++) put16(b + 8 + 2 * c, r->pend_txch[c]);
    fc_send(p, f, 0x0102, b, 8 + 2 * r->pend_nch);
}

static void send_stop(port_t *p, int f)
{
    fc_send(p, f, 0x0101, p->rx[f].handle, 6);
}

static void on_fc_reply(void *ctx, uint32_t sip, uint16_t sport, uint32_t dip, uint16_t dport, const uint8_t *d, int n,
                        const stamp_t *ts)
{
    (void)sport; (void)dip; (void)dport; (void)ts;
    port_t *p = ctx;
    if (n < RR_HDR) return;
    uint16_t seq = get16(d + 4), op = get16(d + 6), code = get16(d + 8);
    for (int f = 0; f < NRXF; f++) {
        rxflow_t *r = &p->rx[f];
        if (!r->used || r->multicast || r->req_seq != seq || r->ctl_ip != sip) continue;
        r->req_seq = 0;
        if (op == 0x0100 && r->state == 0) {
            if (code == 1 && n >= RR_HDR + 6) {
                memcpy(r->handle, d + RR_HDR, 6);
                r->state = 1;
                r->created = g_now();
                rx_program(p, f);
                for (int k = 0; k < MAX_CH; k++) if (p->sub[k].active && p->sub[k].flow == f) set_status(p, k, SUB_IN_PROGRESS);
                LOGI("flow %d from %s: %d channels, fpp %d", f + 1, r->host, r->nch, r->fpp);
            } else {
                LOGW("flow request to %s refused: 0x%04x", r->host, code);
                for (int k = 0; k < MAX_CH; k++)
                    if (p->sub[k].active && p->sub[k].flow == f) {
                        set_status(p, k, code == 0x0315 ? SUB_TOO_MANY : SUB_TX_FAIL);
                        p->sub[k].flow = p->sub[k].slot = -1;
                        p->sub[k].next_try = g_now() + 10000000000ull;
                    }
                rx_free(p, f, 0);
            }
        } else if (op == 0x0102 && r->state == 2) {
            if (code == 1) {
                r->nch = r->pend_nch;
                for (int c = 0; c < r->nch; c++) {
                    int k = r->pend_col[c];
                    r->txch[c] = r->pend_txch[c];
                    r->col[c] = k >= 0 && k < p->channels && p->sub[k].active && p->sub[k].flow == f && p->sub[k].slot == c ? k : -1;
                }
                rx_program(p, f);
                for (int k = 0; k < MAX_CH; k++) if (p->sub[k].active && p->sub[k].flow == f && p->sub[k].status < SUB_IN_PROGRESS) set_status(p, k, SUB_IN_PROGRESS);
            } else {
                for (int k = 0; k < MAX_CH; k++)
                    if (p->sub[k].active && p->sub[k].flow == f && r->col[p->sub[k].slot] != k) {
                        p->sub[k].flow = p->sub[k].slot = -1;
                        set_status(p, k, SUB_TX_FAIL);
                        p->sub[k].next_try = g_now() + 10000000000ull;
                    }
            }
            r->state = 1;
        }
        return;
    }
}

static void on_media_port(void *ctx, uint32_t sip, uint16_t sport, uint32_t dip, uint16_t dport, const uint8_t *d, int n,
                          const stamp_t *ts)
{
    /* the FPGA claims the media; anything else that reaches the flow's port is dropped */
    (void)ctx; (void)sip; (void)sport; (void)dip; (void)dport; (void)d; (void)n; (void)ts;
}

void sub_init(port_t *p)
{
    p->fc_sock = net_udp_bind(p->net, 0, on_fc_reply, p);
    for (int k = 0; k < MAX_CH; k++) { p->sub[k].flow = p->sub[k].slot = -1; }
}

static void rx_free(port_t *p, int f, int stop)
{
    rxflow_t *r = &p->rx[f];
    if (!r->used) return;
    uint32_t z = 0;
    tbl_write(p->hw, TBL_RX(f, 0), &z, 1);
    if (r->multicast) net_leave(p->net, r->group);
    else {
        if (stop && r->state) send_stop(p, f);
        net_udp_unbind(p->net, r->rx_port);
    }
    for (int k = 0; k < MAX_CH; k++) if (p->sub[k].flow == f) { p->sub[k].flow = p->sub[k].slot = -1; }
    memset(r, 0, sizeof(*r));
}

int sub_subscribe(port_t *p, int k, const char *txch, const char *txhost)
{
    if (k < 0 || k >= p->channels || !txch[0] || !txhost[0]) return -1;
    sub_t *s = &p->sub[k];
    if (s->active && !strcmp(s->txch, txch) && !strcasecmp(s->txhost, txhost)) return 0;
    if (s->active) sub_unsubscribe(p, k);
    s->active = 1;
    snprintf(s->txch, sizeof(s->txch), "%s", txch);
    snprintf(s->txhost, sizeof(s->txhost), "%s", txhost);
    s->flow = s->slot = -1;
    s->next_try = 0;
    set_status(p, k, SUB_UNRESOLVED);
    p->resolve_now = 1;
    p->dirty = 1;
    LOGI("subscribe RX %d <- %s@%s", k + 1, txch, txhost);
    return 0;
}

void sub_unsubscribe(port_t *p, int k)
{
    if (k < 0 || k >= MAX_CH) return;
    sub_t *s = &p->sub[k];
    if (s->flow >= 0) {
        rxflow_t *r = &p->rx[s->flow];
        for (int c = 0; c < r->pend_nch; c++) if (r->pend_col[c] == k) r->pend_col[c] = -1;
        if (s->slot >= 0 && s->slot < MAXFCH && r->col[s->slot] == k) { r->col[s->slot] = -1; if (r->state) rx_program(p, s->flow); }
    }
    memset(s, 0, sizeof(*s));
    s->flow = s->slot = -1;
    notify(p, k);
    p->dirty = 1;
}

/* an advertised channel (mdns_client.rs AdvertisedChannel) */
typedef struct {
    uint32_t ip;
    uint16_t port, id, dbcp1;
    int nchan, bits, fpp_min, fpp_max, bundle, cib;
    uint32_t latency_ns;
} advch_t;

static int txt_int(port_t *p, const char *fq, const char *key, long *v)
{
    char b[64];
    if (!mdns_txt(p->mdns, fq, key, b, sizeof(b), NULL)) return 0;
    *v = strtol(b, NULL, 0);
    return 1;
}

static int resolve_chan(port_t *p, const sub_t *s, advch_t *a)
{
    char fq[200], target[160], b[64];
    snprintf(fq, sizeof(fq), "%s@%s._netaudio-chan._udp.local", s->txch, s->txhost);
    int any;
    int have_srv = mdns_srv(p->mdns, fq, &a->port, target, sizeof(target));
    mdns_txt(p->mdns, fq, "id", b, sizeof(b), &any);
    if (!have_srv) mdns_query(p->mdns, fq, 33);
    if (!any) mdns_query(p->mdns, fq, 16);
    if (!have_srv || !any) return 0;
    if (!mdns_a(p->mdns, target, &a->ip)) { mdns_query(p->mdns, target, 1); return 0; }
    long v;
    memset((uint8_t *)a + offsetof(advch_t, id), 0, sizeof(*a) - offsetof(advch_t, id));
    if (!txt_int(p, fq, "id", &v)) return 0;
    a->id = (uint16_t)v;
    a->nchan = txt_int(p, fq, "nchan", &v) ? (int)v : 8;
    a->bits = txt_int(p, fq, "enc", &v) || txt_int(p, fq, "en", &v) ? (int)v : 24;
    a->dbcp1 = txt_int(p, fq, "dbcp1", &v) ? (uint16_t)v : DBCP1;
    a->latency_ns = txt_int(p, fq, "latency_ns", &v) ? (uint32_t)v : 1000000;
    a->fpp_max = 32; a->fpp_min = 1;
    if (mdns_txt(p->mdns, fq, "fpp", b, sizeof(b), NULL)) {
        char *c = strchr(b, ',');
        a->fpp_max = atoi(b);
        a->fpp_min = c ? atoi(c + 1) : a->fpp_max;
    }
    a->bundle = 0;
    /* a "b.N=k" key: the channel is in multicast bundle N at position k */
    for (int nn = 1; nn <= 64 && !a->bundle; nn++) {
        char key[8];
        snprintf(key, sizeof(key), "b.%d", nn);
        if (mdns_txt(p->mdns, fq, key, b, sizeof(b), NULL) && atoi(b) > 0) { a->bundle = nn; a->cib = atoi(b) - 1; }
    }
    return 1;
}

static int free_rxflow(const port_t *p)
{
    for (int f = 0; f < NRXF; f++) if (!p->rx[f].used) return f;
    return -1;
}

static int shift_frames(const port_t *p, uint32_t their_ns)
{
    int their = (int)((uint64_t)their_ns * (uint64_t)p->fs / 1000000000ull), ours = port_offset_frames(p);
    return their > ours ? their - ours : 0;
}

static void attach_mcast(port_t *p, int k, const advch_t *a)
{
    sub_t *s = &p->sub[k];
    char fq[200], b[64];
    snprintf(fq, sizeof(fq), "%d@%s._netaudio-bund._udp.local", a->bundle, s->txhost);
    uint32_t grp;
    if (!mdns_txt(p->mdns, fq, "a.0", b, sizeof(b), NULL) || parse_ip(b, &grp)) { mdns_query(p->mdns, fq, 16); return; }
    long port = 0, nch = 0, enc = 24, lat = 1000000;
    txt_int(p, fq, "p.0", &port);
    txt_int(p, fq, "nchan", &nch);
    if (!txt_int(p, fq, "enc", &enc)) txt_int(p, fq, "en", &enc);
    txt_int(p, fq, "latency_ns", &lat);
    if (nch < 1 || nch > MAXFCH || a->cib >= nch) { LOGW("bundle %s: %ld channels unsupported", fq, nch); return; }
    int f = -1;
    for (int i = 0; i < NRXF; i++) if (p->rx[i].used && p->rx[i].multicast && p->rx[i].group == grp && p->rx[i].gport == port) f = i;
    if (f < 0) {
        f = free_rxflow(p);
        if (f < 0) { set_status(p, k, SUB_TOO_MANY); return; }
        rxflow_t *r = &p->rx[f];
        memset(r, 0, sizeof(*r));
        r->used = 1; r->multicast = 1; r->state = 1;
        scopy(r->host, sizeof(r->host), s->txhost);
        r->group = grp; r->gport = (uint16_t)port; r->bundle_id = a->bundle;
        r->nch = (int)nch; r->bps = (int)enc / 8;
        r->peer_latency_ns = (uint32_t)lat;
        r->latency_ns = MAX(r->peer_latency_ns, port_latency_ns(p));
        r->shift = shift_frames(p, (uint32_t)lat);
        for (int c = 0; c < MAXFCH; c++) { r->col[c] = -1; r->txch[c] = 0; }
        r->created = g_now();
        net_join(p->net, grp);
        LOGI("multicast flow %d: bundle %s on %s:%ld", f + 1, fq, b, port);
    }
    rxflow_t *r = &p->rx[f];
    if (r->col[a->cib] >= 0 && r->col[a->cib] != k) {
        /* one column per flow channel: a second local channel on the same bundle channel is not supported */
        LOGW("RX %d: bundle channel %d already plays on RX %d", k + 1, a->cib + 1, r->col[a->cib] + 1);
        set_status(p, k, SUB_TX_FAIL);
        s->next_try = g_now() + 10000000000ull;
        return;
    }
    r->col[a->cib] = (int16_t)k;
    r->txch[a->cib] = a->id;
    s->flow = f; s->slot = a->cib;
    rx_program(p, f);
    set_status(p, k, SUB_IN_PROGRESS);
}

static void resolve(port_t *p, uint64_t now)
{
    /* pending unicast channels of this pass, per server */
    int batch[MAX_CH], nb = 0;
    advch_t ads[MAX_CH];
    for (int k = 0; k < p->channels; k++) {
        sub_t *s = &p->sub[k];
        if (!s->active || s->flow >= 0 || now < s->next_try) continue;
        if (!strcasecmp(s->txhost, p->name)) { s->next_try = now + 5000000000ull; continue; }   /* ourselves */
        advch_t a;
        if (!resolve_chan(p, s, &a)) { s->next_try = now + 1000000000ull; continue; }
        if (a.bundle) { attach_mcast(p, k, &a); if (s->flow < 0) s->next_try = now + 1000000000ull; continue; }
        /* an existing flow from the same server: the channel already in a free slot, or a free slot to update */
        int done = 0, hold = 0;
        for (int f = 0; f < NRXF && !done; f++) {
            rxflow_t *r = &p->rx[f];
            if (!r->used || r->multicast || r->ctl_ip != a.ip || r->ctl_port != a.port) continue;
            if (r->state != 1) { if (r->nch < r->per) hold = 1; continue; }   /* busy: it may take this one next */
            for (int c = 0; c < r->nch && !done; c++)
                if (r->col[c] < 0 && r->txch[c] == a.id) {
                    r->col[c] = (int16_t)k; s->flow = f; s->slot = c;
                    rx_program(p, f);
                    set_status(p, k, r->pkts ? SUB_RX_UNICAST : SUB_IN_PROGRESS);
                    done = 1;
                }
            for (int c = 0; c < MIN(r->nch + 1, r->per) && !done; c++)   /* a free slot, or one more channel */
                if (c == r->nch || r->col[c] < 0) {
                    memcpy(r->pend_txch, r->txch, sizeof(r->txch));
                    memcpy(r->pend_col, r->col, sizeof(r->col));
                    r->pend_nch = c == r->nch ? r->nch + 1 : r->nch;
                    r->pend_txch[c] = a.id; r->pend_col[c] = (int16_t)k;
                    r->state = 2;
                    s->flow = f; s->slot = c;
                    send_update(p, f);
                    set_status(p, k, SUB_IN_PROGRESS);
                    done = 1;
                }
        }
        if (done) continue;
        if (hold) { s->next_try = now + 100000000ull; continue; }
        if (!s->ready_at) s->ready_at = now;
        ads[nb] = a;
        batch[nb++] = k;
    }
    /* gather: a server's channels resolve over a few mDNS answers; open its new flows once the first of them has
     * waited GATHER_NS, so the flows are full ones (upstream resolves a whole pass before it requests flows) */
    int keep[MAX_CH];
    for (int i = 0; i < nb; i++) {
        uint64_t oldest = now;
        for (int j = 0; j < nb; j++)
            if (ads[j].ip == ads[i].ip && ads[j].port == ads[i].port && p->sub[batch[j]].ready_at < oldest) oldest = p->sub[batch[j]].ready_at;
        /* hold while channels of the same device are still being resolved (at most GATHER_MAX_NS) */
        int waiting = 0;
        for (int u = 0; u < p->channels; u++) {
            const sub_t *x = &p->sub[u];
            if (x->active && x->flow < 0 && !x->ready_at && !strcasecmp(x->txhost, p->sub[batch[i]].txhost)) waiting = 1;
        }
        keep[i] = !(now - oldest < GATHER_NS || (waiting && now - oldest < GATHER_MAX_NS));
    }
    {
        int m = 0;
        for (int i = 0; i < nb; i++) if (keep[i]) { batch[m] = batch[i]; ads[m] = ads[i]; m++; }
        nb = m;
    }
    /* new flows: per server, chunks of up to min(advertised nchan, MAXFCH) channels (upstream: min(nchan, 8)) */
    int taken[MAX_CH] = { 0 };
    for (int i = 0; i < nb; i++) {
        if (taken[i]) continue;
        const advch_t *a0 = &ads[i];
        int per = MIN(MAX(a0->nchan, 1), MAXFCH);
        int f = free_rxflow(p);
        if (f < 0) { set_status(p, batch[i], SUB_TOO_MANY); p->sub[batch[i]].next_try = now + 10000000000ull; continue; }
        rxflow_t *r = &p->rx[f];
        memset(r, 0, sizeof(*r));
        int port = net_udp_bind(p->net, 0, on_media_port, p);
        if (port < 0) continue;
        r->used = 1; r->state = 0; r->rx_port = (uint16_t)port;
        scopy(r->host, sizeof(r->host), p->sub[batch[i]].txhost);
        r->ctl_ip = a0->ip; r->ctl_port = a0->port; r->dbcp1 = a0->dbcp1;
        r->bps = a0->bits / 8;
        if (r->bps < 2 || r->bps > 4) r->bps = 3;
        for (int c = 0; c < MAXFCH; c++) r->col[c] = -1;
        for (int j = i; j < nb && r->nch < per; j++) {
            if (taken[j] || ads[j].ip != a0->ip || ads[j].port != a0->port || ads[j].bits != a0->bits) continue;
            taken[j] = 1;
            r->txch[r->nch] = ads[j].id;
            r->col[r->nch] = (int16_t)batch[j];
            p->sub[batch[j]].flow = f; p->sub[batch[j]].slot = r->nch;
            p->sub[batch[j]].ready_at = 0;
            set_status(p, batch[j], SUB_IN_PROGRESS);
            r->nch++;
        }
        int fpp = MIN(a0->fpp_max, port_auto_fpp(p));
        fpp = MIN(fpp, mtu_fpp(r->nch * r->bps));
        if (fpp < a0->fpp_min) fpp = a0->fpp_min;
        r->fpp = fpp;
        r->per = per;
        r->peer_latency_ns = a0->latency_ns;
        r->latency_ns = MAX(r->peer_latency_ns, port_latency_ns(p));
        r->shift = shift_frames(p, a0->latency_ns);
        r->created = now;
        r->req_tries = 1;
        send_request(p, f);
    }
}

static void scan(port_t *p, uint64_t now)
{
    for (int f = 0; f < NRXF; f++) {
        rxflow_t *r = &p->rx[f];
        if (!r->used) continue;
        if (!r->multicast && r->state != 1 && r->req_at && (int64_t)(now - r->req_at) > (int64_t)FC_TIMEOUT_NS) {
            if (r->state == 0 && r->req_tries < 2) { r->req_tries++; send_request(p, f); continue; }
            if (r->state == 2) { send_update(p, f); continue; }
            if (r->state == 0) {
                LOGW("flow request to %s timed out", r->host);
                for (int k = 0; k < MAX_CH; k++)
                    if (p->sub[k].flow == f) { set_status(p, k, SUB_TX_FAIL); p->sub[k].next_try = now + 10000000000ull; }
                rx_free(p, f, 0);
                continue;
            }
        }
        if (r->state == 0) continue;
        int used = 0;
        for (int c = 0; c < r->nch; c++) used += r->col[c] >= 0;
        if (!used && r->state == 1) { LOGI("flow %d no longer needed", f + 1); rx_free(p, f, 1); continue; }
        uint32_t pk = tbl_read(p->hw, TBL_RXST(f, 2));
        if (pk != r->pkts) {
            r->pkts = pk;
            r->pkts_seen_at = now;
            r->src_ip = tbl_read(p->hw, TBL_RXST(f, 0));
            r->src_port = (uint16_t)tbl_read(p->hw, TBL_RXST(f, 1));
            for (int k = 0; k < MAX_CH; k++)
                if (p->sub[k].active && p->sub[k].flow == f) set_status(p, k, r->multicast ? SUB_RX_MULTICAST : SUB_RX_UNICAST);
        } else if ((int64_t)(now - (r->pkts_seen_at ? r->pkts_seen_at : r->created)) > 2000000000LL && (int64_t)(now - r->created) > 6000000000LL) {
            LOGW("flow %d from %s: no media, its channels are orphaned", f + 1, r->host);
            for (int k = 0; k < MAX_CH; k++)
                if (p->sub[k].active && p->sub[k].flow == f) {
                    char fq[200];
                    snprintf(fq, sizeof(fq), "%s@%s._netaudio-chan._udp.local", p->sub[k].txch, p->sub[k].txhost);
                    mdns_forget(p->mdns, fq);
                    set_status(p, k, SUB_UNRESOLVED);
                    p->sub[k].next_try = now + 1000000000ull;
                }
            rx_free(p, f, 1);
        }
    }
}

void sub_tick(port_t *p, uint64_t now)
{
    if (!net_ready(p->net)) return;
    if (p->resolve_now || now >= p->next_sub_scan) {
        p->resolve_now = 0;
        resolve(p, now);
        p->next_sub_scan = now + 500000000ull;
    }
    if (now >= p->next_flow_scan) { scan(p, now); p->next_flow_scan = now + 500000000ull; }
    if (now >= p->next_ka) {
        static const uint8_t ka[2] = { 0x13, 0x37 };
        for (int f = 0; f < NRXF; f++) {
            rxflow_t *r = &p->rx[f];
            if (r->used && !r->multicast && r->state >= 1 && r->src_ip) net_udp_send(p->net, r->rx_port, r->src_ip, r->src_port, ka, 2, NULL);
        }
        p->next_ka = now + KEEPALIVE_NS;
    }
    if (p->any_changed) {
        int ch[MAX_CH], n = 0;
        for (int k = 0; k < MAX_CH; k++) if (p->changed_rx[k]) { ch[n++] = k; p->changed_rx[k] = 0; }
        p->any_changed = 0;
        info_channel_change(p, ch, n);
    }
}

void sub_shutdown(port_t *p)
{
    for (int f = 0; f < NRXF; f++) rx_free(p, f, 1);
}

void sub_retime(port_t *p)
{
    for (int f = 0; f < NRXF; f++) {
        rxflow_t *r = &p->rx[f];
        if (!r->used) continue;
        r->shift = shift_frames(p, r->peer_latency_ns);
        r->latency_ns = MAX(r->peer_latency_ns, port_latency_ns(p));
        if (r->state) rx_program(p, f);
    }
}

void sub_reset_rate(port_t *p)
{
    sub_shutdown(p);
    for (int k = 0; k < MAX_CH; k++) if (p->sub[k].active) {
        p->sub[k].next_try = p->sub[k].ready_at = 0;
        set_status(p, k, SUB_UNRESOLVED);
    }
    p->resolve_now = 1;
}
