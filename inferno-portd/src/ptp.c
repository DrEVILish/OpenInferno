/* ptp.c - see ptp.h. Part of OpenProSeries inferno-portd. Licence: GPL-3.0-or-later.
 *
 * Follower: as Statime's PTPv1 slave (port/slave.rs handle_sync_v1 / handle_follow_up_v1 / handle_delay_resp_v1):
 * t1 from the Follow_Up (two-step) or the Sync, t2 = our RX SFD stamp, t3 = the Delay_Req TX stamp, t4 from the
 * Delay_Resp; offset = ((t2 - t1) - (t4 - t3)) / 2, delay = ((t2 - t1) + (t4 - t3)) / 2. The best master is picked
 * from the Syncs (PTPv1 has no Announce: statime register_sync_v1_message). Leader (ours): two-step Sync to
 * 224.0.1.129:319 every 2^log_sync s with the TX stamp in a Follow_Up, Delay_Resp to every Delay_Req.
 */
#include "ptp.h"
#include "clock.h"
#include "util.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/* ---- wire format */
static void put_hdr(uint8_t *b, const ptp1_hdr_t *h)
{
    memset(b, 0, 40);
    put16(b, 1);       /* versionPTP */
    put16(b + 2, 1);   /* versionNetwork */
    memcpy(b + 4, h->subdomain, 16);
    b[20] = h->msgtype;
    b[21] = h->comm;
    memcpy(b + 22, h->uuid, 6);
    put16(b + 28, h->port);
    put16(b + 30, h->seq);
    b[32] = h->control;
    put16(b + 34, h->flags);
}

static void put_gm(uint8_t *b, const ptp1_gm_t *g)
{
    memset(b, 0, 28);
    b[1] = g->comm;
    memcpy(b + 2, g->uuid, 6);
    put16(b + 8, g->port);
    put16(b + 10, g->seq);
    b[15] = g->stratum;
    memcpy(b + 16, g->ident, 4);
    put16(b + 22, (uint16_t)g->variance);
    b[25] = g->preferred;
    b[27] = g->boundary;
}

static void get_gm(const uint8_t *b, ptp1_gm_t *g)
{
    g->comm = b[1];
    memcpy(g->uuid, b + 2, 6);
    g->port = get16(b + 8);
    g->seq = get16(b + 10);
    g->stratum = b[15];
    memcpy(g->ident, b + 16, 4);
    g->variance = (int16_t)get16(b + 22);
    g->preferred = b[25] != 0;
    g->boundary = b[27] != 0;
}

int ptp1_put_sync(uint8_t *b, const ptp1_hdr_t *h, const ptp1_sync_t *s)
{
    put_hdr(b, h);
    uint8_t *c = b + 40;
    memset(c, 0, 84);
    put32(c, s->sec); put32(c + 4, s->ns);
    put16(c + 8, s->epoch);
    put16(c + 10, (uint16_t)s->utc_offset);
    put_gm(c + 12, &s->gm);
    c[43] = (uint8_t)s->sync_interval;
    put16(c + 46, (uint16_t)s->local_variance);
    put16(c + 50, s->steps);
    c[55] = s->local_stratum;
    memcpy(c + 56, s->local_ident, 4);
    c[61] = s->parent_comm;
    memcpy(c + 62, s->parent_uuid, 6);
    put16(c + 70, s->parent_port);
    put16(c + 74, (uint16_t)s->est_variance);
    put32(c + 76, (uint32_t)s->est_drift);
    c[83] = s->utc_reasonable;
    return 124;
}

int ptp1_put_followup(uint8_t *b, const ptp1_hdr_t *h, const ptp1_fup_t *f)
{
    put_hdr(b, h);
    uint8_t *c = b + 40;
    memset(c, 0, 12);
    put16(c + 2, f->assoc_seq);
    put32(c + 4, f->sec);
    put32(c + 8, f->ns);
    return 52;
}

int ptp1_put_delay_resp(uint8_t *b, const ptp1_hdr_t *h, const ptp1_dresp_t *r)
{
    put_hdr(b, h);
    uint8_t *c = b + 40;
    memset(c, 0, 20);
    put32(c, r->sec); put32(c + 4, r->ns);
    c[9] = r->req_comm;
    memcpy(c + 10, r->req_uuid, 6);
    put16(c + 16, r->req_port);
    put16(c + 18, r->req_seq);
    return 60;
}

int ptp1_get_hdr(const uint8_t *b, int n, ptp1_hdr_t *h)
{
    if (n < 40 || get16(b) != 1) return -1;   /* versionPTP 1 (statime is_compatible) */
    memcpy(h->subdomain, b + 4, 16);
    h->msgtype = b[20];
    h->comm = b[21];
    memcpy(h->uuid, b + 22, 6);
    h->port = get16(b + 28);
    h->seq = get16(b + 30);
    h->control = b[32];
    h->flags = get16(b + 34);
    return 0;
}

int ptp1_get_sync(const uint8_t *b, int n, ptp1_sync_t *s)
{
    if (n < 124) return -1;
    const uint8_t *c = b + 40;
    s->sec = get32(c); s->ns = get32(c + 4);
    s->epoch = get16(c + 8);
    s->utc_offset = (int16_t)get16(c + 10);
    get_gm(c + 12, &s->gm);
    s->sync_interval = (int8_t)c[43];
    s->local_variance = (int16_t)get16(c + 46);
    s->steps = get16(c + 50);
    s->local_stratum = c[55];
    memcpy(s->local_ident, c + 56, 4);
    s->parent_comm = c[61];
    memcpy(s->parent_uuid, c + 62, 6);
    s->parent_port = get16(c + 70);
    s->est_variance = (int16_t)get16(c + 74);
    s->est_drift = (int32_t)get32(c + 76);
    s->utc_reasonable = c[83] != 0;
    return 0;
}

int ptp1_get_followup(const uint8_t *b, int n, ptp1_fup_t *f)
{
    if (n < 52) return -1;
    f->assoc_seq = get16(b + 42);
    f->sec = get32(b + 44);
    f->ns = get32(b + 48);
    return 0;
}

int ptp1_get_delay_resp(const uint8_t *b, int n, ptp1_dresp_t *r)
{
    if (n < 60) return -1;
    const uint8_t *c = b + 40;
    r->sec = get32(c); r->ns = get32(c + 4);
    r->req_comm = c[9];
    memcpy(r->req_uuid, c + 10, 6);
    r->req_port = get16(c + 16);
    r->req_seq = get16(c + 18);
    return 0;
}

/* IEEE 1588-2002 clock identifiers, best first (statime: ATOM / GPS / NTP / HAND are "ptp timescale") */
static int ident_rank(const char *id)
{
    static const char *const R[] = { "ATOM", "GPS\0", "NTP\0", "HAND", "INIT", "DFLT" };
    for (int i = 0; i < 6; i++) if (!memcmp(id, R[i], 4)) return i;
    return 6;
}

int ptp1_compare(const ptp1_ds_t *a, const ptp1_ds_t *b)
{
    /* statime maps "preferred" to priority1 64 / 128 and compares the variance next; stratum and the identifier
     * (1588-2002 data set comparison) come between them here */
    if (a->preferred != b->preferred) return a->preferred ? -1 : 1;
    if (a->stratum != b->stratum) return a->stratum < b->stratum ? -1 : 1;
    int ra = ident_rank(a->ident), rb = ident_rank(b->ident);
    if (ra != rb) return ra < rb ? -1 : 1;
    if (a->variance != b->variance) return a->variance < b->variance ? -1 : 1;
    int c = memcmp(a->uuid, b->uuid, 6);
    if (c) return c < 0 ? -1 : 1;
    if (a->steps != b->steps) return a->steps < b->steps ? -1 : 1;
    return 0;
}

/* ---- the engine */
#define NFM 8
typedef struct {
    int      used;
    uint8_t  uuid[6];
    uint16_t port;
    ptp1_sync_t last;
    uint64_t rx[4];   /* the last Sync times (qualification: 2 within 4 intervals) */
    int      nrx;
} fm_t;

struct ptp {
    net_t   *n;
    dclock_t *clk;
    ptp_cfg_t c;
    uint8_t  uuid[6];
    int      state;
    uint64_t start, next_sync, next_dreq, next_bmca, last_leader_rx;
    int      lost;              /* following, no Sync from the leader for 4 s (the desk clock is in its source-fail mode) */
    uint16_t sync_seq, dreq_seq;
    fm_t     fm[NFM];
    int      parent;            /* fm index followed */
    /* follower measurement */
    uint16_t sync_wait_seq;
    int      t1_ok, t2_ok;
    int64_t  t1, t2, t3, t4;    /* ns (32-bit seconds widened) */
    int      t3_ok;
    double   delay_ns;
    int      have_delay;
    servo_t  servo;
    /* leader */
    uint16_t sync_pending_seq;
    int      sync_pending;
    ptp_stats_t st;
};

static int64_t ts_ns(uint32_t sec, uint32_t ns) { return (int64_t)sec * 1000000000LL + ns; }
/* a - b with 32-bit seconds (both near each other) */
static int64_t ts_diff(int64_t a, int64_t b)
{
    int64_t d = a - b, wrap = 4294967296LL * 1000000000LL;
    if (d > wrap / 2) d -= wrap;
    if (d < -wrap / 2) d += wrap;
    return d;
}

static void hdr_init(ptp_t *p, ptp1_hdr_t *h, int event, uint8_t control, uint16_t seq)
{
    memset(h, 0, sizeof(*h));
    memcpy(h->subdomain, "_DFLT", 5);   /* statime Header::new */
    h->msgtype = event ? 1 : 2;
    h->comm = 1;
    memcpy(h->uuid, p->uuid, 6);
    h->port = 1;
    h->seq = seq;
    h->control = control;
}

static void own_ds(const ptp_t *p, ptp1_ds_t *d)
{
    memset(d, 0, sizeof(*d));
    d->preferred = p->c.role == PTP_ROLE_LEADER;
    d->stratum = p->c.stratum;
    memcpy(d->ident, "DFLT", 4);
    d->variance = p->c.variance;
    memcpy(d->uuid, p->uuid, 6);
}

static void fm_ds(const fm_t *f, ptp1_ds_t *d)
{
    memset(d, 0, sizeof(*d));
    d->preferred = f->last.gm.preferred;
    d->stratum = f->last.gm.stratum;
    memcpy(d->ident, f->last.gm.ident, 4);
    d->variance = f->last.gm.variance;
    memcpy(d->uuid, f->last.gm.uuid, 6);
    d->steps = f->last.steps;
}

static int may_lead(const ptp_t *p) { return p->c.role == PTP_ROLE_AUTO || p->c.role == PTP_ROLE_LEADER; }
static int may_follow(const ptp_t *p) { return p->c.role == PTP_ROLE_AUTO || p->c.role == PTP_ROLE_FOLLOWER; }
static uint64_t sync_ns(const ptp_t *p) { return (uint64_t)(1e9 * pow(2.0, p->c.log_sync)); }

static void set_state(ptp_t *p, int s)
{
    if (p->state == s) return;
    LOGI("PTP: %s -> %s", ptp_state_name(p->state), ptp_state_name(s));
    int was = p->state;
    p->state = s;
    p->t1_ok = p->t2_ok = p->t3_ok = 0;
    p->have_delay = 0;
    servo_init(&p->servo);
    p->last_leader_rx = g_now();   /* the 4 s source-loss timer runs from here, never from a Sync before a gap */
    p->lost = 0;
    if (p->clk && p->c.steer) {
        dclock_set_locked(p->clk, 0);
        if (s == PTP_SLAVE) dclock_set_mode(p->clk, DCLK_FOLLOWER);              /* (back) at the source: it glides */
        else if (s == PTP_MASTER) dclock_set_mode(p->clk, DCLK_LEADER);
        else if (was == PTP_SLAVE) dclock_set_mode(p->clk, DCLK_HOLDOVER);       /* the leader is gone: clockfail */
    }
}

static void send_msg(ptp_t *p, int event, const uint8_t *b, int len, int ts_req, uint8_t tag)
{
    txopt_t o = { 1, 0xE0, (uint8_t)ts_req, tag };   /* TTL 1 (link-local PTP), DSCP CS7 (inferno.md 11.9) */
    net_udp_send(p->n, event ? PTP1_EVENT : PTP1_GENERAL, PTP1_GROUP, event ? PTP1_EVENT : PTP1_GENERAL, b, len, &o);
}

static void fill_sync(ptp_t *p, ptp1_sync_t *s, uint32_t sec, uint32_t ns)
{
    memset(s, 0, sizeof(*s));
    s->sec = sec; s->ns = ns;
    s->gm.comm = 1;
    memcpy(s->gm.uuid, p->uuid, 6);
    s->gm.port = 1;
    s->gm.seq = p->sync_seq;
    s->gm.stratum = p->c.stratum;
    memcpy(s->gm.ident, "DFLT", 4);
    s->gm.variance = p->c.variance;
    s->gm.preferred = p->c.role == PTP_ROLE_LEADER;
    s->sync_interval = p->c.log_sync;
    s->local_variance = p->c.variance;
    s->steps = 0;
    s->local_stratum = p->c.stratum;
    memcpy(s->local_ident, "DFLT", 4);
    s->parent_comm = 1;
    memcpy(s->parent_uuid, p->uuid, 6);
    s->parent_port = 1;
}

static void send_sync(ptp_t *p)
{
    uint32_t sec = 0, ns = 0;
    if (p->clk) dclock_now(p->clk, &sec, &ns);
    ptp1_hdr_t h;
    hdr_init(p, &h, 1, PTP1_SYNC, ++p->sync_seq);
    h.flags = PTP1_FLAG_ASSIST;
    ptp1_sync_t s;
    fill_sync(p, &s, sec, ns);
    uint8_t b[124];
    ptp1_put_sync(b, &h, &s);
    p->sync_pending = 1;
    p->sync_pending_seq = p->sync_seq;
    send_msg(p, 1, b, 124, 1, 1);
    p->st.syncs_tx++;
}

static void send_dreq(ptp_t *p)
{
    if (p->parent < 0) return;
    ptp1_hdr_t h;
    hdr_init(p, &h, 1, PTP1_DELAY_REQ, ++p->dreq_seq);
    ptp1_sync_t s;
    fill_sync(p, &s, 0, 0);
    /* as Statime's PTPv1 Delay_Req: the grandmaster from the parent, interval 0x7f, stratum 255 */
    const fm_t *f = &p->fm[p->parent];
    s.gm = f->last.gm;
    s.sync_interval = 0x7f;
    s.local_stratum = 255;
    s.steps = (uint16_t)(f->last.steps + 1);
    memcpy(s.parent_uuid, f->uuid, 6);
    s.parent_port = f->port;
    s.gm.preferred = f->last.gm.preferred;
    uint8_t b[124];
    ptp1_put_sync(b, &h, &s);
    p->t3_ok = 0;
    send_msg(p, 1, b, 124, 1, 2);
    p->st.dreq_tx++;
}

static void on_txstamp(void *ctx, uint8_t tag, const stamp_t *ts)
{
    ptp_t *p = ctx;
    if (tag == 1 && p->sync_pending && p->state == PTP_MASTER) {
        ptp1_hdr_t h;
        hdr_init(p, &h, 0, PTP1_FOLLOW_UP, p->sync_pending_seq);
        ptp1_fup_t f = { p->sync_pending_seq, ts->sec, ts->ns };
        uint8_t b[52];
        ptp1_put_followup(b, &h, &f);
        send_msg(p, 0, b, 52, 0, 0);
        p->sync_pending = 0;
        p->st.fups_tx++;
    } else if (tag == 2) {
        p->t3 = ts_ns(ts->sec, ts->ns);
        p->t3_ok = 1;
    }
}

static int fm_find(ptp_t *p, const uint8_t *uuid, uint16_t port, int create)
{
    int fr = -1;
    for (int i = 0; i < NFM; i++) {
        if (p->fm[i].used && !memcmp(p->fm[i].uuid, uuid, 6) && p->fm[i].port == port) return i;
        if (!p->fm[i].used && fr < 0) fr = i;
    }
    if (!create || fr < 0) return -1;
    memset(&p->fm[fr], 0, sizeof(fm_t));
    p->fm[fr].used = 1;
    memcpy(p->fm[fr].uuid, uuid, 6);
    p->fm[fr].port = port;
    return fr;
}

static void sample(ptp_t *p)
{
    if (!p->t1_ok || !p->t2_ok) return;
    int64_t ms = ts_diff(p->t2, p->t1);
    if (p->have_delay) {
        double off = (double)ms - p->delay_ns;
        p->st.offset_ns = off;
        p->st.samples++;
        if (p->c.steer && p->clk) {
            double ppb, step;
            int r = servo_sample(&p->servo, off, g_now(), &ppb, &step);
            p->st.servo_state = r;
            if (r == SERVO_JUMP) {
                LOGI("PTP: offset %.0f ns, stepping", off);
                dclock_step(p->clk, (int64_t)llround(step));
                p->have_delay = 0;
                p->t1 = p->t2 = 0;
            } else if (r != SERVO_UNLOCKED || ppb != 0.0) dclock_set_freq(p->clk, ppb);
            dclock_set_locked(p->clk, r == SERVO_LOCKED && fabs(off) < 20000.0);
        }
    }
    p->t1_ok = p->t2_ok = 0;
}

static void on_event(void *ctx, uint32_t sip, uint16_t sport, uint32_t dip, uint16_t dport, const uint8_t *d, int n,
                     const stamp_t *ts)
{
    (void)sip; (void)sport; (void)dip; (void)dport;
    ptp_t *p = ctx;
    ptp1_hdr_t h;
    if (p->c.role == PTP_ROLE_OFF || ptp1_get_hdr(d, n, &h) || !memcmp(h.uuid, p->uuid, 6)) return;
    if (h.control == PTP1_SYNC) {
        ptp1_sync_t s;
        if (ptp1_get_sync(d, n, &s)) return;
        p->st.syncs_rx++;
        int i = fm_find(p, h.uuid, h.port, 1);
        if (i < 0) return;
        fm_t *f = &p->fm[i];
        f->last = s;
        memmove(f->rx + 1, f->rx, sizeof(f->rx) - sizeof(f->rx[0]));
        f->rx[0] = g_now();
        if (f->nrx < 4) f->nrx++;
        if (p->state == PTP_SLAVE && i == p->parent) {
            p->last_leader_rx = g_now();
            p->t2 = ts_ns(ts->sec, ts->ns);
            p->t2_ok = 1;
            p->sync_wait_seq = h.seq;
            if (h.flags & PTP1_FLAG_ASSIST) p->t1_ok = 0;
            else { p->t1 = ts_ns(s.sec, s.ns); p->t1_ok = 1; sample(p); }
        }
    } else if (h.control == PTP1_DELAY_REQ && p->state == PTP_MASTER) {
        p->st.dreq_rx++;
        ptp1_hdr_t r;
        hdr_init(p, &r, 0, PTP1_DELAY_RESP, h.seq);
        ptp1_dresp_t dr = { ts->sec, ts->ns, h.comm, { 0 }, h.port, h.seq };
        memcpy(dr.req_uuid, h.uuid, 6);
        uint8_t b[60];
        ptp1_put_delay_resp(b, &r, &dr);
        send_msg(p, 0, b, 60, 0, 0);
        p->st.dresp_tx++;
    }
}

static void on_general(void *ctx, uint32_t sip, uint16_t sport, uint32_t dip, uint16_t dport, const uint8_t *d, int n,
                       const stamp_t *ts)
{
    (void)sip; (void)sport; (void)dip; (void)dport; (void)ts;
    ptp_t *p = ctx;
    ptp1_hdr_t h;
    if (p->c.role == PTP_ROLE_OFF || ptp1_get_hdr(d, n, &h) || p->state != PTP_SLAVE || p->parent < 0) return;
    const fm_t *f = &p->fm[p->parent];
    if (memcmp(h.uuid, f->uuid, 6) || h.port != f->port) return;
    if (h.control == PTP1_FOLLOW_UP) {
        ptp1_fup_t fu;
        if (ptp1_get_followup(d, n, &fu)) return;
        p->st.fups_rx++;
        if (p->t2_ok && fu.assoc_seq == p->sync_wait_seq) { p->t1 = ts_ns(fu.sec, fu.ns); p->t1_ok = 1; sample(p); }
    } else if (h.control == PTP1_DELAY_RESP) {
        ptp1_dresp_t r;
        if (ptp1_get_delay_resp(d, n, &r)) return;
        if (memcmp(r.req_uuid, p->uuid, 6) || r.req_port != 1 || r.req_seq != p->dreq_seq || !p->t3_ok) return;
        p->st.dresp_rx++;
        p->t4 = ts_ns(r.sec, r.ns);
        /* the delay from the last Sync pair and this request pair: ((t2 - t1) + (t4 - t3)) / 2 */
        if (p->t1 && p->t2) {
            double ms = (double)ts_diff(p->t2, p->t1), sm = (double)ts_diff(p->t4, p->t3);
            double dl = (ms + sm) / 2.0;
            if (dl > -1e6 && dl < 1e7) {
                p->delay_ns = p->have_delay ? p->delay_ns + (dl - p->delay_ns) / 8.0 : dl;
                p->have_delay = 1;
                p->st.delay_ns = p->delay_ns;
            }
        }
        p->t3_ok = 0;
    }
}

ptp_t *ptp_new(net_t *n, dclock_t *clk, const uint8_t mac[6], const ptp_cfg_t *cfg)
{
    ptp_t *p = calloc(1, sizeof(*p));
    p->n = n;
    p->clk = clk;
    p->c = *cfg;
    memcpy(p->uuid, mac, 6);
    p->parent = -1;
    p->state = PTP_LISTENING;
    p->start = g_now();
    servo_init(&p->servo);
    net_udp_bind(n, PTP1_EVENT, on_event, p);
    net_udp_bind(n, PTP1_GENERAL, on_general, p);
    net_join(n, PTP1_GROUP);
    net_set_txstamp_cb(n, on_txstamp, p);
    return p;
}

void ptp_free(ptp_t *p)
{
    net_udp_unbind(p->n, PTP1_EVENT);
    net_udp_unbind(p->n, PTP1_GENERAL);
    free(p);
}

void ptp_set_role(ptp_t *p, int role, int steer)
{
    p->c.role = role;
    p->c.steer = steer;
    p->parent = -1;
    set_state(p, role == PTP_ROLE_OFF ? PTP_DISABLED : PTP_LISTENING);
    p->start = g_now();
}

static int qualified(const ptp_t *p, const fm_t *f, uint64_t now)
{
    uint64_t win = 4 * MAX(sync_ns(p), 1000000000ull);   /* 4 intervals of the slowest plausible Sync rate */
    return f->nrx >= 2 && (int64_t)(now - f->rx[1]) < (int64_t)win;
}

static void bmca(ptp_t *p, uint64_t now)
{
    int best = -1;
    ptp1_ds_t bd, d;
    for (int i = 0; i < NFM; i++) {
        fm_t *f = &p->fm[i];
        if (!f->used) continue;
        if ((int64_t)(now - f->rx[0]) > 8000000000LL) { f->used = 0; continue; }   /* gone */
        if (!qualified(p, f, now)) continue;
        fm_ds(f, &d);
        if (best < 0 || ptp1_compare(&d, &bd) < 0) { best = i; bd = d; }
    }
    ptp1_ds_t own;
    own_ds(p, &own);
    int lead = may_lead(p) && (best < 0 || ptp1_compare(&own, &bd) < 0);
    if (lead) {
        /* listen for a while before leading (a better master may be there) */
        if (p->state != PTP_MASTER && (int64_t)(now - p->start) < (int64_t)(4 * MAX(sync_ns(p), 1000000000ull))) return;
        p->parent = -1;
        set_state(p, PTP_MASTER);
        return;
    }
    if (best >= 0 && may_follow(p)) {
        if (p->parent != best) { p->parent = best; p->t1_ok = p->t2_ok = 0; p->have_delay = 0; }
        memcpy(p->st.leader_uuid, p->fm[best].uuid, 6);
        set_state(p, PTP_SLAVE);
        return;
    }
    p->parent = -1;
    set_state(p, best >= 0 ? PTP_PASSIVE : PTP_LISTENING);
}

void ptp_tick(ptp_t *p, uint64_t now)
{
    if (p->c.role == PTP_ROLE_OFF || !net_ready(p->n)) return;
    if (now >= p->next_bmca) { bmca(p, now); p->next_bmca = now + 250000000ull; }
    if (p->state == PTP_MASTER && now >= p->next_sync) {
        send_sync(p);
        p->next_sync = now + sync_ns(p);
    }
    if (p->state == PTP_SLAVE && now >= p->next_dreq) {
        send_dreq(p);
        /* 1 s plus a random part (1588-2002 randomises the Delay_Req instant) */
        p->next_dreq = now + 500000000ull + (uint64_t)(rnd32() % 1000) * 1000000ull;
    }
    if (p->state == PTP_SLAVE && p->clk && p->c.steer) {
        /* no Sync from the leader for 4 s: the source is lost (HOLDOVER or GO LEADER); its Syncs again: back to it */
        int lost = (int64_t)(now - p->last_leader_rx) > 4000000000LL;
        if (lost != p->lost) {
            LOGI("PTP: %s", lost ? "no Sync from the leader for 4 s" : "the leader's Syncs are back");
            p->lost = lost;
            p->t1_ok = p->t2_ok = p->t3_ok = 0;
            p->have_delay = 0;
            servo_init(&p->servo);
            dclock_set_locked(p->clk, 0);
            dclock_set_mode(p->clk, lost ? DCLK_HOLDOVER : DCLK_FOLLOWER);
        }
    }
}

int ptp_state(const ptp_t *p) { return p->state; }

const char *ptp_state_name(int s)
{
    static const char *const N[] = { "LISTENING", "MASTER", "SLAVE", "PASSIVE", "DISABLED" };
    return s >= 0 && s <= 4 ? N[s] : "?";
}

void ptp_get_stats(const ptp_t *p, ptp_stats_t *s)
{
    *s = p->st;
    s->state = p->state;
}
