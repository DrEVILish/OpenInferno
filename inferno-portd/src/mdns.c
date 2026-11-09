/* mdns.c - see mdns.h. DNS message format: RFC 1035 (names with compression pointers on input), mDNS rules: RFC 6762
 * (port 5353, 224.0.0.251, the cache-flush bit, unicast replies for QU questions and legacy queriers, announcements
 * and TTL-0 goodbyes), DNS-SD record set: RFC 6763 and upstream Inferno's responder (mdns_server.rs).
 * Part of OpenProSeries inferno-portd. Licence: GPL-3.0-or-later.
 */
#include "mdns.h"
#include "util.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>

#define NSVC   200
#define NCACHE 320
#define NQ     96

enum { T_A = 1, T_PTR = 12, T_TXT = 16, T_SRV = 33, T_ANY = 255 };

typedef struct {
    int      used;
    char     name[160];
    uint16_t type;
    uint64_t expires;
    uint16_t port;
    char     target[160];
    int      ntxt;
    char     txt[MDNS_MAXTXT][100];
    uint32_t a;
} rr_t;

struct mdns {
    net_t     *n;
    mdns_svc_t svc[NSVC];
    int        used[NSVC];
    int        announce[NSVC];
    uint64_t   next_announce;
    rr_t       cache[NCACHE];
    struct { char name[160]; uint16_t type; uint64_t last; } q[NQ];
    mdns_stats_t st;
};

/* ---- wire helpers */
static int put_label(uint8_t *p, int max, const char *l, int n)
{
    if (n > 63) n = 63;
    if (n + 1 > max) return -1;
    p[0] = (uint8_t)n;
    memcpy(p + 1, l, (size_t)n);
    return n + 1;
}

int dns_put_name(uint8_t *p, int max, const char *fqdn)
{
    int o = 0;
    const char *s = fqdn;
    while (*s) {
        const char *d = strchr(s, '.');
        int n = d ? (int)(d - s) : (int)strlen(s);
        if (n) {
            int r = put_label(p + o, max - o, s, n);
            if (r < 0) return -1;
            o += r;
        }
        s += n;
        if (*s == '.') s++;
    }
    if (o + 1 > max) return -1;
    p[o++] = 0;
    return o;
}

/* a service's instance name: the instance as one label (it may hold '@'), then the type's labels */
static int put_inst_name(uint8_t *p, int max, const mdns_svc_t *s)
{
    char rest[64];
    int o;
    if (!strcmp(s->type, "in-addr")) {
        char f[160];
        snprintf(f, sizeof(f), "%s.in-addr.local", s->inst);
        return dns_put_name(p, max, f);
    }
    o = put_label(p, max, s->inst, (int)strlen(s->inst));
    if (o < 0) return -1;
    snprintf(rest, sizeof(rest), "%s._udp.local", s->type);
    int r = dns_put_name(p + o, max - o, rest);
    return r < 0 ? -1 : o + r;
}

int dns_get_name(const uint8_t *msg, int len, int off, char *out, int outsz)
{
    int o = 0, jumped = 0, end = -1, hops = 0;
    out[0] = 0;
    while (off < len) {
        uint8_t l = msg[off];
        if (l == 0) { if (!jumped) end = off + 1; break; }
        if ((l & 0xC0) == 0xC0) {
            if (off + 1 >= len || ++hops > 32) return -1;
            if (!jumped) end = off + 2;
            jumped = 1;
            off = ((l & 0x3F) << 8) | msg[off + 1];
            continue;
        }
        if (off + 1 + l > len) return -1;
        if (o && o + 1 < outsz) out[o++] = '.';
        for (int i = 0; i < l && o + 1 < outsz; i++) out[o++] = (char)msg[off + 1 + i];
        out[o] = 0;
        off += 1 + l;
    }
    if (off >= len && end < 0) return -1;
    return end;
}

void mdns_fqdn(const mdns_svc_t *s, char *out, size_t sz)
{
    if (!strcmp(s->type, "in-addr")) snprintf(out, sz, "%s.in-addr.local", s->inst);
    else snprintf(out, sz, "%s.%s._udp.local", s->inst, s->type);
}

static int rr_head(uint8_t *p, int max, int nlen, uint16_t type, int flush, uint32_t ttl)
{
    if (nlen + 10 > max) return -1;
    put16(p + nlen, type);
    put16(p + nlen + 2, (uint16_t)(1 | (flush ? 0x8000 : 0)));
    put32(p + nlen + 4, ttl);
    return nlen + 10;   /* rdlength at nlen + 8, filled by the caller */
}

int mdns_build_response(const mdns_svc_t *s, uint32_t ip, uint16_t id, long ttl_o, uint8_t *buf, int max)
{
    uint32_t ttl = ttl_o >= 0 ? (uint32_t)ttl_o : s->ttl;
    int inaddr = !strcmp(s->type, "in-addr");
    memset(buf, 0, 12);
    put16(buf, id);
    put16(buf + 2, 0x8400);   /* response, authoritative */
    int o = 12, r, h;
    char host[128];
    snprintf(host, sizeof(host), "%s.local", s->inst);
    if (inaddr) {
        put16(buf + 6, 1);
        put16(buf + 10, s->origin[0] ? 1 : 0);
        if ((r = put_inst_name(buf + o, max - o, s)) < 0) return -1;
        if ((h = rr_head(buf + o, max - o, r, T_A, 1, ttl)) < 0) return -1;
        put16(buf + o + r + 8, 4); o += h;
        if (o + 4 > max) return -1;
        put32(buf + o, ip); o += 4;
        if (s->origin[0]) {
            if ((r = dns_put_name(buf + o, max - o, "_inferno-response-origin.local")) < 0) return -1;
            if ((h = rr_head(buf + o, max - o, r, T_TXT, 1, ttl)) < 0) return -1;
            int sl = (int)strlen(s->origin);
            if (o + h + 1 + sl > max) return -1;
            put16(buf + o + r + 8, (uint16_t)(1 + sl)); o += h;
            buf[o++] = (uint8_t)sl; memcpy(buf + o, s->origin, (size_t)sl); o += sl;
        }
        return o;
    }
    put16(buf + 6, 3);    /* PTR, SRV, TXT */
    put16(buf + 10, 1);   /* A */
    char type[64];
    snprintf(type, sizeof(type), "%s._udp.local", s->type);
    /* PTR: type -> instance */
    if ((r = dns_put_name(buf + o, max - o, type)) < 0) return -1;
    if ((h = rr_head(buf + o, max - o, r, T_PTR, 0, ttl)) < 0) return -1;
    int rd = o + r + 8;
    o += h;
    if ((r = put_inst_name(buf + o, max - o, s)) < 0) return -1;
    put16(buf + rd, (uint16_t)r); o += r;
    /* SRV */
    if ((r = put_inst_name(buf + o, max - o, s)) < 0) return -1;
    if ((h = rr_head(buf + o, max - o, r, T_SRV, 0, ttl)) < 0) return -1;
    rd = o + r + 8; o += h;
    if (o + 6 > max) return -1;
    put16(buf + o, 0); put16(buf + o + 2, 0); put16(buf + o + 4, s->port); o += 6;
    if ((r = put_label(buf + o, max - o, s->inst, (int)strlen(s->inst))) < 0) return -1;
    o += r;
    if ((r = dns_put_name(buf + o, max - o, "local")) < 0) return -1;
    o += r;
    put16(buf + rd, (uint16_t)(o - rd - 2));
    /* TXT */
    if ((r = put_inst_name(buf + o, max - o, s)) < 0) return -1;
    if ((h = rr_head(buf + o, max - o, r, T_TXT, 1, ttl)) < 0) return -1;
    rd = o + r + 8; o += h;
    int t0 = o;
    for (int i = 0; i < s->ntxt; i++) {
        int sl = (int)strlen(s->txt[i]);
        if (o + 1 + sl > max) return -1;
        buf[o++] = (uint8_t)sl; memcpy(buf + o, s->txt[i], (size_t)sl); o += sl;
    }
    if (o == t0) { if (o + 1 > max) return -1; buf[o++] = 0; }
    put16(buf + rd, (uint16_t)(o - t0));
    /* A of the SRV target */
    if ((r = put_label(buf + o, max - o, s->inst, (int)strlen(s->inst))) < 0) return -1;
    int r2 = dns_put_name(buf + o + r, max - o - r, "local");
    if (r2 < 0) return -1;
    r += r2;
    if ((h = rr_head(buf + o, max - o, r, T_A, 1, ttl)) < 0) return -1;
    put16(buf + o + r + 8, 4); o += h;
    if (o + 4 > max) return -1;
    put32(buf + o, ip); o += 4;
    (void)host;
    return o;
}

/* ---- responder */
static void send_svc(mdns_t *m, int i, uint32_t dip, uint16_t dport, uint16_t id, long ttl)
{
    uint8_t b[1400];
    int len = mdns_build_response(&m->svc[i], net_ip(m->n), id, ttl, b, sizeof(b));
    if (len <= 0) { LOGW("mdns: %s too large", m->svc[i].inst); return; }
    txopt_t o = { 255, 0, 0, 0 };
    net_udp_send(m->n, MDNS_PORT, dip, dport, b, len, &o);
    m->st.tx_responses++;
}

int mdns_add(mdns_t *m, const mdns_svc_t *s)
{
    int free_i = -1;
    for (int i = 0; i < NSVC; i++) {
        if (m->used[i] && !strcasecmp(m->svc[i].inst, s->inst) && !strcmp(m->svc[i].type, s->type)) { free_i = i; break; }
        if (!m->used[i] && free_i < 0) free_i = i;
    }
    if (free_i < 0) { LOGE("mdns: no room for %s", s->inst); return -1; }
    m->svc[free_i] = *s;
    if (!m->svc[free_i].ttl) m->svc[free_i].ttl = 120;
    m->used[free_i] = 1;
    m->announce[free_i] = 2;
    if (m->next_announce == 0) m->next_announce = 1;
    return free_i;
}

void mdns_remove(mdns_t *m, const char *inst, const char *type)
{
    for (int i = 0; i < NSVC; i++)
        if (m->used[i] && !strcasecmp(m->svc[i].inst, inst) && !strcmp(m->svc[i].type, type)) {
            if (net_ready(m->n)) send_svc(m, i, MDNS_GROUP, MDNS_PORT, 0, 0);   /* goodbye */
            m->used[i] = 0;
        }
}

void mdns_remove_all(mdns_t *m)
{
    for (int i = 0; i < NSVC; i++)
        if (m->used[i]) { if (net_ready(m->n)) send_svc(m, i, MDNS_GROUP, MDNS_PORT, 0, 0); m->used[i] = 0; }
}

static int name_eq(const char *a, const char *b) { return strcasecmp(a, b) == 0; }

static void on_query(mdns_t *m, uint32_t sip, uint16_t sport, const uint8_t *d, int n)
{
    uint16_t id = get16(d), qd = get16(d + 4);
    int off = 12;
    m->st.rx_queries++;
    int legacy = sport != MDNS_PORT;
    for (int q = 0; q < qd && off < n; q++) {
        char qn[256];
        off = dns_get_name(d, n, off, qn, sizeof(qn));
        if (off < 0 || off + 4 > n) return;
        uint16_t qt = get16(d + off), qc = get16(d + off + 2);
        off += 4;
        int unicast = legacy || (qc & 0x8000);
        uint32_t dip = unicast ? sip : MDNS_GROUP;
        uint16_t dport = unicast ? sport : MDNS_PORT;
        for (int i = 0; i < NSVC; i++) {
            if (!m->used[i]) continue;
            mdns_svc_t *s = &m->svc[i];
            char id_f[200], type_f[64], host_f[128];
            mdns_fqdn(s, id_f, sizeof(id_f));
            snprintf(type_f, sizeof(type_f), "%s._udp.local", s->type);
            snprintf(host_f, sizeof(host_f), "%s.local", s->inst);
            int hit = 0;
            if (!strcmp(s->type, "in-addr")) hit = name_eq(qn, id_f) && (qt == T_A || qt == T_ANY);
            else hit = (name_eq(qn, type_f) && (qt == T_PTR || qt == T_ANY)) || (name_eq(qn, id_f) && qt != T_A) ||
                       (name_eq(qn, host_f) && (qt == T_A || qt == T_ANY));
            if (hit) send_svc(m, i, dip, dport, legacy ? id : 0, -1);
        }
    }
}

static rr_t *cache_slot(mdns_t *m, const char *name, uint16_t type)
{
    int fr = -1, old = 0;
    for (int i = 0; i < NCACHE; i++) {
        if (m->cache[i].used && m->cache[i].type == type && name_eq(m->cache[i].name, name)) return &m->cache[i];
        if (!m->cache[i].used && fr < 0) fr = i;
        if (m->cache[i].expires < m->cache[old].expires) old = i;
    }
    rr_t *r = &m->cache[fr >= 0 ? fr : old];
    memset(r, 0, sizeof(*r));
    r->used = 1;
    snprintf(r->name, sizeof(r->name), "%s", name);
    r->type = type;
    return r;
}

static void on_response(mdns_t *m, uint32_t sip, const uint8_t *d, int n)
{
    if (sip == net_ip(m->n)) return;   /* our own (upstream ignores answers from self) */
    m->st.rx_responses++;
    int qd = get16(d + 4), an = get16(d + 6), ns = get16(d + 8), ar = get16(d + 10);
    int off = 12;
    char name[256];
    for (int q = 0; q < qd; q++) {
        off = dns_get_name(d, n, off, name, sizeof(name));
        if (off < 0) return;
        off += 4;
    }
    uint64_t now = g_now();
    for (int k = 0; k < an + ns + ar && off < n; k++) {
        off = dns_get_name(d, n, off, name, sizeof(name));
        if (off < 0 || off + 10 > n) return;
        uint16_t type = get16(d + off);
        uint32_t ttl = get32(d + off + 4);
        int rdl = get16(d + off + 8);
        int rd = off + 10;
        off = rd + rdl;
        if (off > n) return;
        if (type != T_A && type != T_SRV && type != T_TXT) continue;
        if (ttl == 0) {
            for (int i = 0; i < NCACHE; i++)
                if (m->cache[i].used && m->cache[i].type == type && name_eq(m->cache[i].name, name)) m->cache[i].used = 0;
            continue;
        }
        rr_t *r = cache_slot(m, name, type);
        r->expires = now + (uint64_t)ttl * 1000000000ull;
        if (type == T_A && rdl == 4) r->a = get32(d + rd);
        else if (type == T_SRV && rdl > 6) {
            r->port = get16(d + rd + 4);
            if (dns_get_name(d, n, rd + 6, r->target, sizeof(r->target)) < 0) r->used = 0;
        } else if (type == T_TXT) {
            r->ntxt = 0;
            int p = rd;
            while (p < rd + rdl && r->ntxt < MDNS_MAXTXT) {
                int l = d[p];
                if (p + 1 + l > rd + rdl) break;
                int c = l < 99 ? l : 99;
                memcpy(r->txt[r->ntxt], d + p + 1, (size_t)c);
                r->txt[r->ntxt][c] = 0;
                r->ntxt++;
                p += 1 + l;
            }
        }
    }
}

static void on_udp(void *ctx, uint32_t sip, uint16_t sport, uint32_t dip, uint16_t dport, const uint8_t *d, int n,
                   const stamp_t *ts)
{
    (void)dip; (void)dport; (void)ts;
    mdns_t *m = ctx;
    if (n < 12) return;
    uint16_t fl = get16(d + 2);
    if ((fl & 0x7800) != 0) return;   /* opcode 0 only */
    if (fl & 0x8000) on_response(m, sip, d, n);
    else on_query(m, sip, sport, d, n);
}

mdns_t *mdns_new(net_t *n)
{
    mdns_t *m = calloc(1, sizeof(*m));
    m->n = n;
    if (net_udp_bind(n, MDNS_PORT, on_udp, m) < 0) LOGE("mdns: port 5353 busy");
    net_join(n, MDNS_GROUP);
    return m;
}
void mdns_free(mdns_t *m) { net_udp_unbind(m->n, MDNS_PORT); free(m); }

void mdns_tick(mdns_t *m, uint64_t now)
{
    if (!net_ready(m->n) || !m->next_announce || now < m->next_announce) return;
    int more = 0, sent = 0;
    for (int i = 0; i < NSVC && sent < 48; i++)
        if (m->used[i] && m->announce[i] > 0) {
            send_svc(m, i, MDNS_GROUP, MDNS_PORT, 0, -1);
            m->announce[i]--;
            sent++;
            if (m->announce[i]) more = 1;
        }
    for (int i = 0; i < NSVC; i++) if (m->used[i] && m->announce[i] > 0) more = 1;
    m->next_announce = more ? now + (sent >= 48 ? 20000000ull : 1000000000ull) : 0;
    for (int i = 0; i < NCACHE; i++) if (m->cache[i].used && m->cache[i].expires < now) m->cache[i].used = 0;
}

void mdns_query(mdns_t *m, const char *fqdn, uint16_t qtype)
{
    if (!net_ready(m->n)) return;
    uint64_t now = g_now();
    int slot = -1, old = 0;
    for (int i = 0; i < NQ; i++) {
        if (m->q[i].type == qtype && name_eq(m->q[i].name, fqdn)) { slot = i; break; }
        if (m->q[i].last < m->q[old].last) old = i;
    }
    if (slot >= 0 && now - m->q[slot].last < 1000000000ull) return;
    if (slot < 0) { slot = old; snprintf(m->q[slot].name, sizeof(m->q[slot].name), "%s", fqdn); m->q[slot].type = qtype; }
    m->q[slot].last = now ? now : 1;
    uint8_t b[512];
    memset(b, 0, 12);
    put16(b + 4, 1);
    int o = 12;
    /* the first label may hold '@' and other characters; dns_put_name splits on '.', which channel names lack */
    int r = dns_put_name(b + o, (int)sizeof(b) - o - 4, fqdn);
    if (r < 0) return;
    o += r;
    put16(b + o, qtype); put16(b + o + 2, 1); o += 4;
    net_udp_send(m->n, MDNS_PORT, MDNS_GROUP, MDNS_PORT, b, o, NULL);
    m->st.tx_queries++;
}

static rr_t *cache_get(mdns_t *m, const char *name, uint16_t type)
{
    uint64_t now = g_now();
    for (int i = 0; i < NCACHE; i++)
        if (m->cache[i].used && m->cache[i].type == type && m->cache[i].expires >= now && name_eq(m->cache[i].name, name))
            return &m->cache[i];
    return NULL;
}

int mdns_srv(mdns_t *m, const char *fqdn, uint16_t *port, char *target, size_t tsz)
{
    rr_t *r = cache_get(m, fqdn, T_SRV);
    if (!r) return 0;
    *port = r->port;
    snprintf(target, tsz, "%s", r->target);
    return 1;
}

int mdns_txt(mdns_t *m, const char *fqdn, const char *key, char *val, size_t vsz, int *any)
{
    rr_t *r = cache_get(m, fqdn, T_TXT);
    if (any) *any = r != NULL;
    if (!r) return 0;
    size_t kl = strlen(key);
    for (int i = 0; i < r->ntxt; i++)
        if (!strncmp(r->txt[i], key, kl) && r->txt[i][kl] == '=') { snprintf(val, vsz, "%s", r->txt[i] + kl + 1); return 1; }
        else if (!strncmp(r->txt[i], key, kl) && r->txt[i][kl] == 0) { if (vsz) val[0] = 0; return 1; }
    return 0;
}

int mdns_a(mdns_t *m, const char *fqdn, uint32_t *ip)
{
    rr_t *r = cache_get(m, fqdn, T_A);
    if (!r) return 0;
    *ip = r->a;
    return 1;
}

void mdns_forget(mdns_t *m, const char *fqdn)
{
    for (int i = 0; i < NCACHE; i++) if (m->cache[i].used && name_eq(m->cache[i].name, fqdn)) m->cache[i].used = 0;
}

void mdns_get_stats(const mdns_t *m, mdns_stats_t *s)
{
    *s = m->st;
    s->cache = 0;
    for (int i = 0; i < NCACHE; i++) s->cache += m->cache[i].used != 0;
}
