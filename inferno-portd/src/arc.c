/* arc.c - the ARC server (UDP 4440): device and channel names, channel and flow listings, renames, subscriptions,
 * multicast TX flows. Translated from upstream Inferno (github.com/teodly/inferno, GPL-3.0-or-later, commit 9767558e):
 * inferno_aoip/src/device_server/arc_server.rs `run_server` and protocol/proto_arc.rs (layouts, serialize_items,
 * paginate_*, deserialize_items). Unknown fields keep upstream's values and names. Part of OpenProSeries inferno-portd.
 * Licence: GPL-3.0-or-later.
 */
#include "portd.h"
#include "util.h"

#include <stdio.h>
#include <stdlib.h>

#define PACKET_SIZE_SOFT_LIMIT 800   /* proto_arc.rs */
#define RX_PAGE_LIMIT 1400           /* 0x3000 pages: one entry (at most 148 bytes) past it still fits a 1500-byte reply */

/* the item writer of serialize_items: writes strings / descriptors into b, the item (size isz) into item; 0 = skip */
typedef int (*item_fn)(port_t *p, int index, bbuf_t *b, uint8_t *item, void *ctx);

/* proto_arc.rs serialize_items: [space][count] + space x item slots, strings after; returns 1 if there are more */
static int serialize_items(port_t *p, int space, int start, int total, int isz, item_fn fn, void *ctx, bbuf_t *b)
{
    bb_init(b);
    bb_zero(b, RR_HDR);
    bb_u8(b, (uint8_t)space);
    bb_u8(b, 0);
    if (space == 0) return 0;
    bb_zero(b, (size_t)(space * isz));
    size_t item_pos = 2 + RR_HDR;
    int actual = 0, more = 0;
    for (int i = start; i < total; i++) {
        if (actual >= space) { more = 1; break; }
        uint8_t item[32];
        if (!fn(p, i, b, item, ctx)) continue;
        size_t prev = b->pos;
        bb_seek(b, item_pos);
        bb_bytes(b, item, (size_t)isz);
        bb_seek(b, prev);
        item_pos += (size_t)isz;
        if (prev >= PACKET_SIZE_SOFT_LIMIT) { more = 1; break; }   /* as upstream: this item is written, not counted */
        actual++;
    }
    b->d[RR_HDR + 1] = (uint8_t)actual;
    return more;
}

/* proto_arc.rs serialize_page (upstream dbd9570, U13), for the channel lists: the table holds only the entries sent, the
 * strings follow it, and both header bytes carry that count. With serialize_items a short page keeps zeroed slots
 * between table and strings and claims the full page size, which netaudio rejects as malformed. offs: bit w set =
 * the item's u16 word w is a string/descriptor offset (moved down with the strings); lim: the soft size limit.
 * Returns 1 if there are more. */
static int serialize_page(port_t *p, int space, int start, int total, int isz, unsigned offs, int lim, item_fn fn,
                          void *ctx, bbuf_t *b)
{
    int left = MAX(total - start, 0), n = MIN(space, left), more = left > n, k = 0;
    uint8_t items[32][32];
    bb_init(b);
    bb_zero(b, RR_HDR + 2 + (size_t)(n * isz));
    size_t table = RR_HDR + 2, heap = table + (size_t)(n * isz), heap_end = b->pos;
    for (int i = start; i < start + n; i++) {
        size_t entry_start = b->pos;
        if (!fn(p, i, b, items[k], ctx)) continue;
        /* as upstream: the entry that crosses the limit, and the data it wrote, go on the next page */
        if (b->pos >= (size_t)lim && k) { more = 1; heap_end = entry_start; break; }
        heap_end = b->pos;
        k++;
    }
    uint16_t gap = (uint16_t)((n - k) * isz);
    for (int j = 0; j < k; j++) {
        for (int w = 0; w < isz / 2; w++) {
            uint16_t v = get16(items[j] + 2 * w);
            if ((offs >> w & 1) && v) put16(items[j] + 2 * w, (uint16_t)(v - gap));
        }
        memcpy(b->d + table + (size_t)(j * isz), items[j], (size_t)isz);
    }
    memmove(b->d + heap - gap, b->d + heap, heap_end - heap);
    b->len = b->pos = heap_end - gap;
    b->d[RR_HDR] = b->d[RR_HDR + 1] = (uint8_t)k;
    return more;
}

/* extract_start_index: request content bytes 2..3, 1-based */
static int start_index(const uint8_t *c, int clen)
{
    if (clen < 4 || (c[2] | c[3]) == 0) return -1;
    return (int)get16(c + 2) - 1;
}

static int respond(const uint8_t *req, uint16_t code, const uint8_t *c, int clen, uint8_t *out, int max)
{
    if (RR_HDR + clen > max) return 0;
    return rr_make(out, get16(req), get16(req + 4), get16(req + 6), code, c, clen);
}

/* paginate_make_response + respond: b holds header room + content */
static int respond_page(const uint8_t *req, int more, bbuf_t *b, uint8_t *out, int max)
{
    return respond(req, more ? 0x8112 : 1, b->d + RR_HDR, (int)b->len - RR_HDR, out, max);
}

/* CommonChannelsDescriptor (16 bytes) */
static uint16_t write_common(port_t *p, bbuf_t *b)
{
    uint16_t at = (uint16_t)b->pos;
    bb_u32(b, (uint32_t)p->fs);
    bb_u8(b, 1); bb_u8(b, 1);
    bb_u16(b, 24); bb_u16(b, 0x400); bb_u16(b, 24); bb_u16(b, 24);
    bb_u16(b, 0xe);   /* pcm_type */
    return at;
}

typedef struct { uint16_t common; int wrote; } chctx_t;

static int rx_item(port_t *p, int k, bbuf_t *b, uint8_t *it, void *vc)
{
    chctx_t *c = vc;
    if (!c->common) c->common = write_common(p, b);
    const sub_t *s = &p->sub[k];
    uint16_t txo = 0, hosto = 0;
    if (s->active) { txo = bb_str0(b, s->txch); hosto = bb_str0(b, s->txhost); }
    uint16_t fo = bb_str0(b, p->rxname[k]);
    put16(it, (uint16_t)(k + 1)); put16(it + 2, 6); put16(it + 4, c->common); put16(it + 6, txo); put16(it + 8, hosto);
    put16(it + 10, fo);
    put32(it + 12, s->active ? (uint32_t)s->status : 0);
    put32(it + 16, 0);
    return 1;
}

static int tx_item(port_t *p, int k, bbuf_t *b, uint8_t *it, void *vc)
{
    chctx_t *c = vc;
    if (!c->common) c->common = write_common(p, b);
    char def[8];
    snprintf(def, sizeof(def), "%02d", k + 1);   /* the factory name */
    uint16_t no = bb_str0(b, def);
    put16(it, (uint16_t)(k + 1)); put16(it + 2, 7); put16(it + 4, c->common); put16(it + 6, no);
    return 1;
}

static int txf_item(port_t *p, int k, bbuf_t *b, uint8_t *it, void *vc)
{
    chctx_t *c = vc;
    if (!c->wrote) { bb_u32(b, 0); c->wrote = 1; }
    put16(it, (uint16_t)(k + 1)); put16(it + 2, (uint16_t)(k + 1)); put16(it + 4, bb_str0(b, p->txname[k]));
    return 1;
}

static int qtx_item(port_t *p, int f, bbuf_t *b, uint8_t *it, void *vc)
{
    (void)vc;
    const txflow_t *t = &p->tx[f];
    if (!t->used || (t->multicast && t->mc_state != 1)) return 0;
    char nm[24];
    snprintf(nm, sizeof(nm), "%d_%u", f + 1, p->pid);
    uint16_t local = bb_str0(b, nm);
    uint16_t rh = t->multicast ? 0 : bb_str0(b, t->rx_host);
    uint16_t rf = t->multicast ? 0 : bb_str0(b, t->rx_flow);
    int mc = rh == 0 && rf == 0;
    bb_align(b, 4);
    uint16_t sock = (uint16_t)b->pos;
    bb_u16(b, 0x8002); bb_u16(b, t->dst_port); bb_u32(b, t->dst_ip);
    uint16_t names = (uint16_t)b->pos;
    bb_u16(b, 0xa00); bb_u16(b, 1); bb_u16(b, rh); bb_u16(b, rf); bb_u16(b, 0x10); bb_u16(b, local); bb_zero(b, 8);
    uint16_t main_d = (uint16_t)b->pos;
    bb_u16(b, (uint16_t)(f + 1)); bb_u16(b, mc ? 2 : 0x11); bb_u32(b, (uint32_t)p->fs); bb_u16(b, 0);
    bb_u16(b, (uint16_t)(t->bps * 8)); bb_u16(b, 1); bb_u16(b, (uint16_t)t->nch); bb_u16(b, sock);
    for (int c = 0; c < t->nch; c++) bb_u16(b, t->ch[c] >= 0 ? (uint16_t)(t->ch[c] + 1) : 0);
    bb_u16(b, names);
    put16(it, main_d);
    return 1;
}

static int qrx_item(port_t *p, int f, bbuf_t *b, uint8_t *it, void *vc)
{
    (void)vc;
    const rxflow_t *r = &p->rx[f];
    if (!r->used || r->state == 0) return 0;
    bb_align(b, 4);
    uint16_t sock = (uint16_t)b->pos;
    bb_u16(b, 0x8002);
    bb_u16(b, r->multicast ? r->gport : r->rx_port);
    bb_u32(b, r->multicast ? r->group : net_ip(p->net));
    uint16_t d2 = (uint16_t)b->pos;
    bb_u16(b, 9); bb_u16(b, 1); bb_u16(b, 0x800); bb_u16(b, 0);
    bb_u32(b, r->latency_ns ? r->latency_ns : port_latency_ns(p));
    bb_u32(b, 0);
    /* one bit mask of local channels per flow channel (we map one column per flow channel) */
    int wpb = (p->channels + 15) / 16;
    if (wpb < 1) wpb = 1;
    uint16_t masks[MAXFCH];
    for (int c = 0; c < r->nch; c++) {
        masks[c] = (uint16_t)b->pos;
        for (int w = 0; w < wpb; w++) {
            uint16_t word = 0;
            if (r->col[c] >= 0 && r->col[c] / 16 == w) word = (uint16_t)(1u << (r->col[c] % 16));
            bb_u16(b, word);
        }
    }
    bb_align(b, 4);
    uint16_t main_d = (uint16_t)b->pos;
    bb_u16(b, (uint16_t)(f + 1)); bb_u16(b, 1); bb_u32(b, (uint32_t)p->fs); bb_u16(b, 0); bb_u16(b, (uint16_t)(r->bps * 8));
    bb_u16(b, 1); bb_u16(b, (uint16_t)r->nch); bb_u16(b, (uint16_t)wpb); bb_u16(b, sock);
    for (int c = 0; c < r->nch; c++) bb_u16(b, masks[c]);
    bb_u16(b, d2);
    put16(it, main_d);
    return 1;
}

static const char *cstr(const uint8_t *c, int clen, uint16_t off, char *buf, size_t sz)
{
    if (off < RR_HDR) return NULL;
    return str_at(c, (size_t)clen, (size_t)(off - RR_HDR), buf, sz);
}

int arc_handle(port_t *p, const uint8_t *req, int len, uint8_t *out, int max)
{
    if (len < RR_HDR) return 0;
    int tl = get16(req + 2);
    if (tl >= RR_HDR && tl < len) len = tl;
    const uint8_t *c = req + RR_HDR;
    int clen = len - RR_HDR;
    uint16_t op = get16(req + 6);
    if (get16(req + 8) != 0) { LOGD("ARC: opcode2 0x%04x", get16(req + 8)); return 0; }
    bbuf_t *b = malloc(sizeof(bbuf_t));
    int r = 0;
    chctx_t cx = { 0, 0 };
    switch (op) {
    case 0x1000: {   /* channels_and_flows_count */
        uint8_t x[34];
        memset(x, 0, sizeof(x));
        x[0] = 0;
        x[1] = 0x30;   /* Flags2: supports_tx_channel_rename (bit 4), supports_tx_multicast (bit 5) */
        put16(x + 2, (uint16_t)port_tx_channels(p)); put16(x + 4, (uint16_t)port_rx_channels(p));
        put16(x + 6, 4);
        put16(x + 8, (uint16_t)MIN(p->caps_fch, port_tx_channels(p)));   /* maximum_transmit_flow_channel_slots */
        put16(x + 10, (uint16_t)(p->caps_rxf ? MIN(p->caps_fch, p->channels) : 0));   /* maximum_receive_flow_channel_slots */
        put16(x + 12, (uint16_t)p->caps_txf); put16(x + 14, (uint16_t)p->caps_rxf);
        put16(x + 16, (uint16_t)(port_tx_channels(p) + port_rx_channels(p)));
        put16(x + 18, 1); put16(x + 20, 1);
        r = respond(req, 1, x, 34, out, max);
        break;
    }
    case 0x1002: {   /* device name */
        bb_init(b);
        bb_str0(b, p->name);
        r = respond(req, 1, b->d, (int)b->len, out, max);
        break;
    }
    case 0x1003: {   /* get_device_names */
        bb_init(b);
        bb_zero(b, 38);
        uint16_t fr = (uint16_t)(b->pos + RR_HDR); bb_str0(b, p->name);
        uint16_t fa = (uint16_t)(b->pos + RR_HDR); bb_str0(b, p->factory);
        uint16_t bo = (uint16_t)(b->pos + RR_HDR); bb_str0(b, "OpenProSeries");
        uint16_t rv = (uint16_t)(b->pos + RR_HDR); bb_bytes(b, ":705", 5);
        uint8_t *h = b->d;
        memset(h, 0, 38);
        put16(h + 6, bo); put16(h + 8, rv); put16(h + 12, fr); put16(h + 14, fa); put16(h + 16, fr);
        put16(h + 30, 0x2729); put16(h + 34, 0x1102);
        r = respond(req, 1, b->d, (int)b->len, out, max);
        break;
    }
    case 0x3000: {   /* receive channels */
        int st = start_index(c, clen);
        if (st < 0) { r = respond(req, 0xFFFF, NULL, 0, out, max); break; }
        /* netaudio takes at most 16 entries per RX page, and rejects a cut one (fewer than 16 and "more"): 16 entries
         * with 31-character names already pass 800 bytes, so RX pages may grow to the MTU.
         * shortcut: 16 entries with long names and many long subscriptions still cut; dedupe strings if that bites */
        int more = serialize_page(p, 16, st, port_rx_channels(p), 20, 0x3C, RX_PAGE_LIMIT, rx_item, &cx, b);
        r = respond_page(req, more, b, out, max);
        break;
    }
    case 0x2000: {
        int st = start_index(c, clen);
        if (st < 0) { r = respond(req, 0xFFFF, NULL, 0, out, max); break; }
        int more = serialize_page(p, 32, st, port_tx_channels(p), 8, 0x0C, PACKET_SIZE_SOFT_LIMIT, tx_item, &cx, b);
        r = respond_page(req, more, b, out, max);
        break;
    }
    case 0x2010: {
        int st = start_index(c, clen);
        if (st < 0) { r = respond(req, 0xFFFF, NULL, 0, out, max); break; }
        int more = serialize_page(p, 32, st, port_tx_channels(p), 6, 0x04, PACKET_SIZE_SOFT_LIMIT, txf_item, &cx, b);
        r = respond_page(req, more, b, out, max);
        break;
    }
    case 0x2013: {   /* rename TX channels: items {u16 0, u16 id, u16 name offset} */
        int n = clen >= 2 ? c[1] : 0, any = 0;
        for (int i = 0; i < n && 2 + 6 * i + 6 <= clen; i++) {
            const uint8_t *it = c + 2 + 6 * i;
            uint16_t id = get16(it + 2), no = get16(it + 4);
            char nm[64];
            if (!id || !no || !cstr(c, clen, no, nm, sizeof(nm))) continue;
            if (id - 1 < port_tx_channels(p)) { port_rename_tx(p, id - 1, nm); any = 1; }
        }
        static const uint8_t ok[2] = { 0, 0 };
        r = any ? respond(req, 1, ok, 2, out, max) : respond(req, 0xFFFF, NULL, 0, out, max);
        break;
    }
    case 0x3001: {   /* rename RX channels: {u16 id, u16 name offset} */
        int n = clen >= 2 ? c[1] : 0, any = 0;
        int ch[MAX_CH] = { 0 }, nc = 0;
        for (int i = 0; i < n && 2 + 4 * i + 4 <= clen; i++) {
            const uint8_t *it = c + 2 + 4 * i;
            uint16_t id = get16(it), no = get16(it + 2);
            char nm[64];
            if (!id || !no || !cstr(c, clen, no, nm, sizeof(nm))) continue;
            if (id - 1 < p->channels) { port_rename_rx(p, id - 1, nm); any = 1; if (nc < MAX_CH) ch[nc++] = id - 1; }
        }
        p->any_changed = 0;
        for (int k = 0; k < MAX_CH; k++) p->changed_rx[k] = 0;
        info_channel_change(p, ch, nc);
        r = respond(req, any ? 1 : 0xFFFF, NULL, 0, out, max);
        break;
    }
    case 0x2200: {   /* query TX flows */
        int st = start_index(c, clen);
        if (st < 0) { r = respond(req, 0xFFFF, NULL, 0, out, max); break; }
        int more = serialize_items(p, MIN(p->caps_txf, 16), st, p->caps_txf, 2, qtx_item, &cx, b);
        r = respond_page(req, more, b, out, max);
        break;
    }
    case 0x2201: {   /* create multicast TX flows: items = u16 offsets of descriptors */
        int n = clen >= 2 ? c[1] : 0;
        int ids[NTXF], ni = 0;
        for (int i = 0; i < n && 2 + 2 * i + 2 <= clen; i++) {
            int off = (int)get16(c + 2 + 2 * i) - RR_HDR;
            if (off < 0 || off + 16 > clen) continue;
            const uint8_t *d = c + off;
            uint16_t id = get16(d), type = get16(d + 2), nch = get16(d + 14);
            if (type != 2 || id == 0 || id > p->caps_txf) { LOGW("ARC: bad multicast flow id %u type %u", id, type); continue; }
            if (p->tx[id - 1].used) { LOGW("ARC: tx flow id %u busy", id); continue; }
            if (off + 16 + 2 * nch + 2 > clen || nch == 0 || nch > p->caps_fch) continue;
            int16_t chs[MAXFCH];
            for (int k = 0; k < nch; k++) { uint16_t v = get16(d + 16 + 2 * k); chs[k] = v ? (int16_t)(v - 1) : -1; }
            if (txflow_create_mcast(p, id - 1, chs, nch) == 0) ids[ni++] = id;
        }
        if (ni) {
            bb_init(b);
            bb_u16(b, (uint16_t)ni); bb_u16(b, 0);
            for (int i = 0; i < ni; i++) bb_u16(b, (uint16_t)ids[i]);
            r = respond(req, 1, b->d, (int)b->len, out, max);
        } else r = respond(req, 0xFFFF, NULL, 0, out, max);
        break;
    }
    case 0x2202: {   /* delete multicast TX flows: u16 count, u16 0, ids */
        int any = 0;
        if (clen >= 4) {
            int n = get16(c);
            for (int i = 0; i < n && 4 + 2 * i + 2 <= clen; i++) {
                int id = get16(c + 4 + 2 * i);
                if (id > 0 && txflow_delete_mcast(p, id - 1) == 0) any = 1;
            }
        }
        r = any ? respond(req, 1, NULL, 0, out, max) : respond(req, 0xFFFF, NULL, 0, out, max);
        break;
    }
    case 0x2320: r = respond(req, 0x30, NULL, 0, out, max); break;
    case 0x3200: {   /* query RX flows */
        int st = start_index(c, clen);
        if (st < 0) { r = respond(req, 1, NULL, 0, out, max); break; }
        int more = serialize_items(p, MIN(p->caps_rxf, 16), st, p->caps_rxf, 2, qrx_item, &cx, b);
        r = respond_page(req, more, b, out, max);
        break;
    }
    case 0x1100: { uint8_t z[110] = { 0 }; r = respond(req, 1, z, 110, out, max); break; }
    case 0x1102: { uint8_t z[94] = { 0 }; r = respond(req, 1, z, 94, out, max); break; }
    case 0x3300: {   /* upstream: needed to avoid "clock domain mismatch" in Dante Controller */
        static const uint8_t v[8] = { 0x38, 0x00, 0x38, 0xfd, 0x38, 0xfe, 0x38, 0xff };
        r = respond(req, 1, v, 8, out, max);
        break;
    }
    case 0x3010: {   /* subscribe / unsubscribe: {u16 local id, u16 tx channel name offset, u16 tx host offset} */
        int n = clen >= 2 ? c[1] : 0;
        for (int i = 0; i < n && 2 + 6 * i + 6 <= clen; i++) {
            const uint8_t *it = c + 2 + 6 * i;
            uint16_t id = get16(it), cho = get16(it + 2), ho = get16(it + 4);
            if (!id || id - 1 >= p->channels) continue;
            if (cho && ho) {
                char ch[64], host[64];
                if (cstr(c, clen, cho, ch, sizeof(ch)) && cstr(c, clen, ho, host, sizeof(host))) sub_subscribe(p, id - 1, ch, host);
            } else sub_unsubscribe(p, id - 1);
        }
        r = respond(req, 1, NULL, 0, out, max);
        break;
    }
    case 0x3014: {   /* network-audio-controller's unsubscribe */
        if (clen >= 6) { int k = get16(c + 4); if (k > 0 && k - 1 < p->channels) sub_unsubscribe(p, k - 1); }
        r = respond(req, 1, NULL, 0, out, max);
        break;
    }
    default: {
        char hx[200];
        hexdump_str(c, (size_t)MIN(clen, 90), hx, sizeof(hx));
        LOGD("ARC: unknown opcode 0x%04x content %s", op, hx);
        break;
    }
    }
    free(b);
    return r;
}

static void on_arc(void *ctx, uint32_t sip, uint16_t sport, uint32_t dip, uint16_t dport, const uint8_t *d, int n,
                   const stamp_t *ts)
{
    (void)dip; (void)ts;
    port_t *p = ctx;
    uint8_t out[1500];
    int r = arc_handle(p, d, n, out, sizeof(out));
    if (r > 0) net_udp_send(p->net, dport, sip, sport, out, r, NULL);
}

void arc_init(port_t *p) { net_udp_bind(p->net, ARC_PORT, on_arc, p); }

/* ---- CMC (UDP 8800, cmc_server.rs / proto_cmc.rs) */
int cmc_handle(port_t *p, const uint8_t *req, int len, uint8_t *out, int max)
{
    if (len < RR_HDR || get16(req + 8) != 0) return 0;
    if (get16(req + 6) != 0x1001) { LOGD("CMC: unknown opcode 0x%04x", get16(req + 6)); return 0; }
    uint8_t a[24];
    put16(a, p->pid);
    memcpy(a + 2, p->devid, 8);
    put16(a + 10, 1); put16(a + 12, 0);
    put32(a + 14, net_ip(p->net));
    put16(a + 18, INFO_PORT);
    put16(a + 20, 0);
    return respond(req, 1, a, 22, out, max);
}

static void on_cmc(void *ctx, uint32_t sip, uint16_t sport, uint32_t dip, uint16_t dport, const uint8_t *d, int n,
                   const stamp_t *ts)
{
    (void)dip; (void)ts;
    port_t *p = ctx;
    uint8_t out[128];
    int r = cmc_handle(p, d, n, out, sizeof(out));
    if (r > 0) net_udp_send(p->net, dport, sip, sport, out, r, NULL);
}

void cmc_init(port_t *p) { net_udp_bind(p->net, CMC_PORT, on_cmc, p); }
