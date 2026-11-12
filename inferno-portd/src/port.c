/* port.c - one Inferno port's identity, adverts and hardware set-up (upstream Inferno settings.rs / device_info.rs /
 * mdns_server.rs, adapted: the identity comes from the desk name and the port, the audio is the FPGA's).
 * Part of OpenProSeries inferno-portd. Licence: GPL-3.0-or-later.
 */
#include "portd.h"
#include "clock.h"
#include "ptp.h"
#include "regs.h"
#include "util.h"

#include <stdio.h>
#include <stdlib.h>

int g_adv_nchan = 16;   /* nchan advertised per TX channel (main.c --nchan; fpga/docs/inferno.md section 11 item 5) */

static const char *const BOARD = "OpenProSeries";
static const char *const MANUFACTURER = "OpenProSeries";
static const char *const MODEL_NUMBER = "_000000000000000b";   /* upstream's value */

void sanitize_chname(const char *in, char *out, size_t sz)
{
    size_t o = 0;
    for (const char *s = in; *s && o + 1 < sz && o < 31; s++) {
        unsigned char c = (unsigned char)*s;
        if (c < 0x20 || c == '.' || c == '@' || c == '=' || c == 0x7F) continue;   /* DNS-SD label / TXT-safe */
        out[o++] = (char)c;
    }
    out[o] = 0;
}

void port_defaults(port_t *p)
{
    memset(p, 0, sizeof(*p));
    snprintf(p->desk, sizeof(p->desk), "OpenProSeries");
    p->fs = 48000;
    p->cfg_mask = IP4(255, 255, 0, 0);
    p->lat_mode = 0;
    p->channels = MAX_CH;
    p->maxlat = 3;
    p->caps_ch = MAX_CH; p->caps_fch = MAXFCH; p->caps_txf = NTXF; p->caps_rxf = NRXF;
    uint8_t mac[6] = { 0x02, 0x4F, 0x50, 0x53, 0x00, 0x01 };
    memcpy(p->mac, mac, 6);
    for (int k = 0; k < MAX_CH; k++) {
        snprintf(p->txname[k], sizeof(p->txname[k]), "%02d", k + 1);
        snprintf(p->rxname[k], sizeof(p->rxname[k]), "%02d", k + 1);
        p->sub[k].flow = p->sub[k].slot = -1;
    }
}

void port_set_identity(port_t *p)
{
    char d[32];
    sanitize_name(p->desk, d, 30);
    if (!d[0]) snprintf(d, sizeof(d), "OpenProSeries");
    snprintf(p->name, sizeof(p->name), "%s-%d", d, p->n + 1);
    snprintf(p->factory, sizeof(p->factory), "OPS-%02x%02x%02x%02x%02x%02x", p->mac[0], p->mac[1], p->mac[2], p->mac[3], p->mac[4],
             p->mac[5]);
    /* device id: the EUI-64 of the port's MAC (fpga/docs/inferno.md section 9.1) */
    uint8_t id[8] = { p->mac[0], p->mac[1], p->mac[2], 0xFF, 0xFE, p->mac[3], p->mac[4], p->mac[5] };
    memcpy(p->devid, id, 8);
}

uint32_t port_latency_ns(const port_t *p)
{
    static const uint32_t L[4] = { 500000, 1000000, 2000000, 5000000 };
    return L[p->lat_mode & 3];
}
int port_offset_frames(const port_t *p) { return (int)((uint64_t)port_latency_ns(p) * (uint64_t)p->fs / 1000000000ull); }
int port_auto_fpp(const port_t *p)
{
    static const int lo[4] = { 8, 16, 32, 32 }, hi[4] = { 16, 32, 32, 32 };
    return p->fs > 50000 ? hi[p->lat_mode & 3] : lo[p->lat_mode & 3];
}

static void txt(mdns_svc_t *s, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void txt(mdns_svc_t *s, const char *fmt, ...)
{
    if (s->ntxt >= MDNS_MAXTXT) return;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s->txt[s->ntxt++], sizeof(s->txt[0]), fmt, ap);
    va_end(ap);
}

/* the multicast bundle a TX channel is in: 1 and (bundle id, channel in bundle) */
static int chan_bundle(const port_t *p, int k, int *bid, int *cib)
{
    for (int f = 0; f < NTXF; f++) {
        const txflow_t *t = &p->tx[f];
        if (!t->used || !t->multicast || t->mc_state != 1) continue;
        for (int c = 0; c < t->nch; c++) if (t->ch[c] == k) { *bid = f + 1; *cib = c; return 1; }
    }
    return 0;
}

static void adv_chan(port_t *p, int k, const char *chname, int is_default)
{
    mdns_svc_t s;
    memset(&s, 0, sizeof(s));
    snprintf(s.inst, sizeof(s.inst), "%s@%s", chname, p->name);
    snprintf(s.type, sizeof(s.type), "_netaudio-chan");
    s.port = FLOWCTL_PORT;
    s.ttl = 120;
    txt(&s, "txtvers=2");
    txt(&s, "dbcp1=0x%04x", DBCP1);
    txt(&s, "dbcp=0x1004");
    txt(&s, "id=%d", k + 1);
    txt(&s, "rate=%d", p->fs);
    txt(&s, "pcm=3 e");
    txt(&s, "enc=24");
    txt(&s, "en=24");
    txt(&s, "latency_ns=%u", port_latency_ns(p));
    txt(&s, "fpp=%d,%d", FPP_MAX_ADV, FPP_MIN);
    txt(&s, "nchan=%d", MIN(g_adv_nchan, MIN(p->channels, p->caps_fch)));
    if (is_default) txt(&s, "default");
    int bid, cib;
    if (chan_bundle(p, k, &bid, &cib)) txt(&s, "b.%d=%d", bid, cib + 1);
    mdns_add(p->mdns, &s);
}

void port_adv_channel(port_t *p, int k);
void port_adv_channel(port_t *p, int k)
{
    char def[8];
    snprintf(def, sizeof(def), "%02d", k + 1);
    adv_chan(p, k, def, 1);
    if (strcmp(def, p->txname[k])) adv_chan(p, k, p->txname[k], 0);
}

void port_unadv_channel(port_t *p, int k, const char *oldname);
void port_unadv_channel(port_t *p, int k, const char *oldname)
{
    char inst[96];
    snprintf(inst, sizeof(inst), "%s@%s", oldname, p->name);
    mdns_remove(p->mdns, inst, "_netaudio-chan");
    (void)k;
}

void port_adv_bundle(port_t *p, int f);
void port_adv_bundle(port_t *p, int f)
{
    txflow_t *t = &p->tx[f];
    char b[16];
    mdns_svc_t s;
    memset(&s, 0, sizeof(s));
    snprintf(s.inst, sizeof(s.inst), "%d@%s", f + 1, p->name);
    snprintf(s.type, sizeof(s.type), "_netaudio-bund");
    s.port = FLOWCTL_PORT;
    s.ttl = 120;
    txt(&s, "txtvers=1");
    txt(&s, "id=%d", f + 1);
    txt(&s, "nchan=%d", t->nch);
    txt(&s, "latency_ns=%u", port_latency_ns(p));
    txt(&s, "fpp=%d", tx_eff_fpp(p, 0, t->nch, 3));
    txt(&s, "rate=%d", p->fs);
    txt(&s, "enc=24");
    txt(&s, "a.0=%s", ip_str(t->dst_ip, b));
    txt(&s, "p.0=%d", t->dst_port);
    mdns_add(p->mdns, &s);
    /* the address reservation (upstream reserve_multicast_ip) */
    mdns_svc_t r;
    memset(&r, 0, sizeof(r));
    snprintf(r.inst, sizeof(r.inst), "%u.%u.%u.%u", t->dst_ip & 255, (t->dst_ip >> 8) & 255, (t->dst_ip >> 16) & 255, t->dst_ip >> 24);
    snprintf(r.type, sizeof(r.type), "in-addr");
    r.ttl = 120;
    snprintf(r.origin, sizeof(r.origin), "%02x%02x%02x%02x%02x%02x%02x%02x,%u", p->devid[0], p->devid[1], p->devid[2], p->devid[3],
             p->devid[4], p->devid[5], p->devid[6], p->devid[7], p->pid);
    mdns_add(p->mdns, &r);
}

void port_advertise(port_t *p)
{
    if (!net_ready(p->net)) return;
    mdns_svc_t s;
    memset(&s, 0, sizeof(s));
    snprintf(s.inst, sizeof(s.inst), "%s", p->name);
    snprintf(s.type, sizeof(s.type), "_netaudio-arc");
    s.port = ARC_PORT;
    s.ttl = 4500;
    txt(&s, "arcp_vers=2.7.41");
    txt(&s, "arcp_min=0.2.4");
    txt(&s, "router_vers=4.0.2");
    txt(&s, "router_info=%s", BOARD);
    txt(&s, "mf=%s", MANUFACTURER);
    txt(&s, "model=%s", MODEL_NUMBER);
    mdns_add(p->mdns, &s);
    memset(&s, 0, sizeof(s));
    snprintf(s.inst, sizeof(s.inst), "%s", p->name);
    snprintf(s.type, sizeof(s.type), "_netaudio-cmc");
    s.port = CMC_PORT;
    s.ttl = 120;
    txt(&s, "id=%02x%02x%02x%02x%02x%02x%02x%02x", p->devid[0], p->devid[1], p->devid[2], p->devid[3], p->devid[4], p->devid[5],
        p->devid[6], p->devid[7]);
    txt(&s, "process=%u", p->pid);
    txt(&s, "cmcp_vers=1.2.0");
    txt(&s, "cmcp_min=1.0.0");
    txt(&s, "server_vers=4.0.2");
    txt(&s, "channels=0x6000004d");
    txt(&s, "mf=%s", MANUFACTURER);
    txt(&s, "model=%s", MODEL_NUMBER);
    txt(&s, "%s", "");
    txt(&s, "%s", "");
    mdns_add(p->mdns, &s);
    for (int k = 0; k < port_tx_channels(p); k++) port_adv_channel(p, k);
    for (int f = 0; f < NTXF; f++) if (p->tx[f].used && p->tx[f].multicast && p->tx[f].mc_state == 1) port_adv_bundle(p, f);
    p->advertised = 1;
}

void port_unadvertise(port_t *p)
{
    mdns_remove_all(p->mdns);
    p->advertised = 0;
}

void port_apply_hw(port_t *p)
{
    hw_t *h = p->hw;
    port_wr(h, AOIP_MODE, MODE_INFERNO);
    port_wr(h, AOIP_SRC_MAC_HI, ((uint32_t)p->mac[0] << 8) | p->mac[1]);
    port_wr(h, AOIP_SRC_MAC_LO, get32(p->mac + 2));
    port_wr(h, AOIP_SRC_IP, net_ip(p->net));
    port_wr(h, INF_MEDIA_FS, (uint32_t)p->fs);
    port_wr(h, INF_LATENCY, (uint32_t)(p->lat_mode & 3));
    uint32_t got = port_rd(h, INF_LATENCY) & 3;
    if ((int)got != p->lat_mode) { LOGW("latency mode %d refused by the build (MAXLAT %d)", p->lat_mode, p->maxlat); p->lat_mode = (int)got; }
    int cols = p->fs > 50000 ? MIN(p->channels, MAX_CH / 2) : p->channels;
    port_wr(h, INF_CTRL, 3u | ((uint32_t)cols << 8));
}

int port_set_latency(port_t *p, int mode)
{
    if (mode < 0 || mode > 3) return -1;
    if (mode > p->maxlat) { LOGW("latency mode %d above the build's MAXLAT %d", mode, p->maxlat); return -1; }
    int old_mode = p->lat_mode;
    p->lat_mode = mode;
    if (tx_load_bps(p, 0, 0, 0) > TX_BUDGET_BPS) { p->lat_mode = old_mode; return -1; }
    port_wr(p->hw, INF_LATENCY, (uint32_t)mode);
    p->dirty = 1;
    sub_retime(p);
    if (p->advertised) port_advertise(p);   /* latency_ns, the bundles' fpp */
    return 0;
}

int port_set_rate(port_t *p, int fs)
{
    if (fs != 48000 && fs != 96000) return -1;
    if (fs == p->fs) return 0;
    /* Unicast flows are renegotiated; retained multicast bundles must fit the new rate. */
    port_t candidate = *p;
    candidate.fs = fs;
    for (int f = 0; f < NTXF; f++) if (!candidate.tx[f].multicast) candidate.tx[f].used = 0;
    if (tx_load_bps(&candidate, 0, 0, 0) > TX_BUDGET_BPS) return -1;
    port_unadvertise(p);
    sub_reset_rate(p);
    port_wr(p->hw, INF_CTRL, 0);
    p->fs = fs;
    int lim = fs > 50000 ? p->caps_ch / 2 : p->caps_ch;   /* 96 kHz plays half the build's columns */
    if (p->channels > lim) p->channels = lim;
    txflows_rate_changed(p);
    if (p->clk) dclock_set_rate(p->clk, fs);
    port_apply_hw(p);
    p->dirty = 1;
    port_advertise(p);
    return 0;
}

void port_rename(port_t *p, const char *desk)
{
    char d[32];
    sanitize_name(desk, d, 30);
    if (!d[0] || !strcmp(d, p->desk)) return;
    port_unadvertise(p);
    snprintf(p->desk, sizeof(p->desk), "%s", d);
    port_set_identity(p);
    snprintf(g_log_tag, sizeof(g_log_tag), "%s", p->name);
    p->dirty = 1;
    port_advertise(p);
}

void port_rename_tx(port_t *p, int k, const char *name)
{
    if (k < 0 || k >= port_tx_channels(p)) return;
    char nm[32];
    sanitize_chname(name, nm, sizeof(nm));
    if (!nm[0]) snprintf(nm, sizeof(nm), "%02d", k + 1);
    if (!strcmp(nm, p->txname[k])) return;
    char def[8];
    snprintf(def, sizeof(def), "%02d", k + 1);
    if (strcmp(p->txname[k], def)) port_unadv_channel(p, k, p->txname[k]);
    snprintf(p->txname[k], sizeof(p->txname[k]), "%s", nm);
    p->dirty = 1;
    if (p->advertised) port_adv_channel(p, k);
}

void port_rename_rx(port_t *p, int k, const char *name)
{
    if (k < 0 || k >= p->channels) return;
    char nm[32];
    sanitize_chname(name, nm, sizeof(nm));
    if (!nm[0]) snprintf(nm, sizeof(nm), "%02d", k + 1);
    snprintf(p->rxname[k], sizeof(p->rxname[k]), "%s", nm);
    p->dirty = 1;
    p->changed_rx[k] = 1;
    p->any_changed = 1;
}

void port_tick(port_t *p, uint64_t now)
{
    p->hw->ops->service(p->hw);
    int was = net_ready(p->net);
    net_poll(p->net);
    net_tick(p->net, now);
    if (!was && net_ready(p->net)) {
        port_wr(p->hw, AOIP_SRC_IP, net_ip(p->net));
        info_init(p);
        port_advertise(p);
    }
    now = g_now();   /* the callbacks above took their own times: keep every difference below non-negative */
    mdns_tick(p->mdns, now);
    txflows_tick(p, now);
    sub_tick(p, now);
    info_tick(p, now);
    if (p->ptp) ptp_tick(p->ptp, now);
    if (p->clk) dclock_tick(p->clk, now);
    if (p->dirty && p->state_path[0] && now >= p->next_save) {
        if (settings_save(p) == 0) p->dirty = 0;
        p->next_save = now + 1000000000ull;
    }
}

int rr_make(uint8_t *buf, uint16_t start, uint16_t seq, uint16_t op1, uint16_t op2, const uint8_t *c, int clen)
{
    put16(buf, start);
    put16(buf + 2, (uint16_t)(RR_HDR + clen));
    put16(buf + 4, seq);
    put16(buf + 6, op1);
    put16(buf + 8, op2);
    if (clen > 0) memmove(buf + RR_HDR, c, (size_t)clen);
    return RR_HDR + clen;
}
