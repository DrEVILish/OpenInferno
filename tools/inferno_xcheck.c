// inferno_xcheck.c - an independent C model of upstream Inferno's media flow packets, to cross-check the golden
// packets of tools/golden.py (and through them the RTL, tb_inferno / tb_infcap compare against the same files).
//
// Provenance: written for OpenProSeries (W16). The packet assembly is a line-by-line transliteration of upstream
// Inferno (github.com/teodly/inferno, GPL-3.0, commit 9767558e0905c9e9c960f5a60e050fbdc38300b7):
//   inferno_aoip/src/device_server/flows_tx.rs  transmit():  pbuff[0] = 2u8; packet_ts = next_ts + clock_offset;
//       pbuff[1..5] = (packet_ts / sample_rate) as u32 BE; pbuff[5..9] = (packet_ts % sample_rate) as u32 BE;
//       per flow channel i: samples written from 9 + i x bps with stride nch x bps;
//   inferno_aoip/src/device_server/samples_utils.rs  write_s24_samples / write_s32_samples: the top bps bytes of the
//       left-justified i32 sample, big-endian (no dither in this model: our samples are 24-bit already);
//   inferno_aoip/src/device_server/flows_rx.rs  receive():  timestamp = sec.wrapping_mul(sample_rate)
//       .wrapping_add(subsec); samples_count = (len - 9) / stride; S24 / S32 readers;
// The Ethernet / IPv4 / UDP header is the operating system's in upstream; here it is RFC 791 / 1071 / 768 (DF, the
// IP id counting packets, UDP checksum 0), as inferno_tx.v sends it. `make xcheck` checks that the upstream lines this
// model follows are still in the pinned commit (tools/inferno_pin.sh) when the clone is present. It does not use
// any other project's code. Licence: GPL-3.0.
//
//   inferno_xcheck tx <tb|cap64> <n>   print the first n packets (send order) as hex lines, no preamble / FCS
//   inferno_xcheck rx <tb|cap64>       read such lines on stdin, decode them as flows_rx.rs receive() does and
//                                      check every timestamp and sample against the test pattern
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FS 48000u
#define MTU 1500u
#define S0 1760000000ull
#define U0 47990ull
#define T0 (S0 * FS + U0)
#define MAXCH 32

typedef struct {
    unsigned fpp, bps, nch;
    int      ch[MAXCH];           // port channel, -1 = silent (upstream: None)
    uint8_t  dmac[6], dip[4];
    uint16_t sport, dport;
    uint8_t  ttl, tos;
} flow_t;

static const uint8_t SRC_MAC[6] = { 0x02, 0, 0, 0, 0x01, 0x01 };
static const uint8_t SRC_IP[4] = { 192, 0, 2, 10 };

// the test pattern (golden.py sample()), as upstream's left-justified i32 Sample
static int32_t sample_at(uint64_t t, int ch)
{
    uint32_t f = (uint32_t)(t - T0);
    return (int32_t)((((f * 0x010203u) ^ ((uint32_t)ch << 16) ^ 0x5A5A5Au) & 0xFFFFFFu) << 8);
}

static void be32(uint8_t *p, uint32_t v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v; }
static uint32_t rd32(const uint8_t *p) { return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3]; }

// samples_utils.rs write_sN_samples: dst[pos..pos+bytes] = sample.to_be_bytes()[0..bytes]; pos += stride
static void write_samples(const int32_t *src, unsigned n, uint8_t *dst, size_t dst_len, size_t start, size_t stride,
                          unsigned bytes)
{
    size_t pos = start;
    for (unsigned i = 0; i < n && pos + bytes <= dst_len; i++, pos += stride) {
        uint8_t b[4];
        be32(b, (uint32_t)src[i]);
        memcpy(dst + pos, b, bytes);
    }
}

// flows_tx.rs transmit(), one packet of one flow; returns the UDP payload length
static size_t build_media(uint8_t *pbuff, const flow_t *f, uint64_t next_ts, int64_t clock_offset_samples)
{
    int32_t tmp[256];
    size_t stride = f->nch * f->bps;
    memset(pbuff + 9, 0, stride * f->fpp);
    pbuff[0] = 2u;
    uint64_t packet_ts = next_ts + (uint64_t)clock_offset_samples;
    uint64_t seconds = packet_ts / FS, subsec = packet_ts % FS;
    be32(pbuff + 1, (uint32_t)seconds);
    be32(pbuff + 5, (uint32_t)subsec);
    uint64_t start_ts = next_ts;                     // timestamp_shift: 0 (the samples of frame next_ts)
    for (unsigned i = 0; i < f->nch; i++) {
        if (f->ch[i] < 0) continue;                  // None: left zero
        for (unsigned k = 0; k < f->fpp; k++) tmp[k] = sample_at(start_ts + k, f->ch[i]);
        write_samples(tmp, f->fpp, pbuff, MTU, 9 + i * f->bps, stride, f->bps);
    }
    return 9 + stride * f->fpp;
}

static uint16_t ip_csum(const uint8_t *h)
{
    uint32_t s = 0;
    for (int i = 0; i < 20; i += 2) s += (uint32_t)h[i] << 8 | h[i + 1];
    while (s >> 16) s = (s & 0xFFFF) + (s >> 16);
    return (uint16_t)~s;
}

// Ethernet II + IPv4 (RFC 791: IHL 5, DF, TTL, UDP) + UDP (RFC 768: checksum 0) + media payload
static size_t build_frame(uint8_t *fr, const flow_t *f, uint64_t ts, uint16_t ip_id)
{
    size_t plen = build_media(fr + 42, f, ts, 0);
    memcpy(fr, f->dmac, 6); memcpy(fr + 6, SRC_MAC, 6); fr[12] = 0x08; fr[13] = 0x00;
    uint8_t *ip = fr + 14;
    uint16_t tot = (uint16_t)(20 + 8 + plen);
    ip[0] = 0x45; ip[1] = f->tos; ip[2] = tot >> 8; ip[3] = tot; ip[4] = ip_id >> 8; ip[5] = ip_id;
    ip[6] = 0x40; ip[7] = 0; ip[8] = f->ttl; ip[9] = 17; ip[10] = ip[11] = 0;
    memcpy(ip + 12, SRC_IP, 4); memcpy(ip + 16, f->dip, 4);
    uint16_t c = ip_csum(ip); ip[10] = c >> 8; ip[11] = c;
    uint8_t *u = fr + 34;
    uint16_t ulen = (uint16_t)(8 + plen);
    u[0] = f->sport >> 8; u[1] = f->sport; u[2] = f->dport >> 8; u[3] = f->dport; u[4] = ulen >> 8; u[5] = ulen;
    u[6] = u[7] = 0;
    return 42 + plen;
}

static unsigned make_flows(const char *cfg, flow_t *fl)
{
    memset(fl, 0, sizeof(flow_t) * 4);
    if (!strcmp(cfg, "tb")) {
        // the tb_inferno flows (golden.py INF_FLOWS)
        const int c0[] = { 0, 1, 2, 3, 4, 5, 6, 7 }, c1[] = { 40, 41, -1, 47, 3, 10 }, c2[] = { 44, 45 };
        fl[0] = (flow_t){ .fpp = 16, .bps = 3, .nch = 8, .dmac = { 2, 0, 0, 0, 2, 2 }, .dip = { 192, 0, 2, 20 },
                          .sport = 14400, .dport = 14336, .ttl = 64, .tos = 0xB8 };
        memcpy(fl[0].ch, c0, sizeof c0);
        fl[1] = (flow_t){ .fpp = 4, .bps = 4, .nch = 6, .dmac = { 0x01, 0x00, 0x5e, 0x7f, 0x01, 0x02 },
                          .dip = { 239, 255, 1, 2 }, .sport = 14401, .dport = 4321, .ttl = 32, .tos = 0xB8 };
        memcpy(fl[1].ch, c1, sizeof c1);
        fl[2] = (flow_t){ .fpp = 6, .bps = 3, .nch = 2, .dip = { 192, 0, 2, 10 }, .sport = 14402, .dport = 14338,
                          .ttl = 64, .tos = 0xB8 };
        memcpy(fl[2].dmac, SRC_MAC, 6);
        memcpy(fl[2].ch, c2, sizeof c2);
        return 3;
    }
    if (!strcmp(cfg, "cap64")) {
        // golden.py INF_CAP64: 4 flows x 16 channels, fpp 8, 24 bit
        for (unsigned f = 0; f < 4; f++) {
            fl[f] = (flow_t){ .fpp = 8, .bps = 3, .nch = 16, .dmac = { 2, 0, 0, 0, 2, 2 }, .dip = { 192, 0, 2, 20 },
                              .sport = (uint16_t)(14400 + f), .dport = (uint16_t)(14336 + f), .ttl = 64, .tos = 0xB8 };
            for (unsigned c = 0; c < 16; c++) fl[f].ch[c] = (int)(16 * f + c);
        }
        return 4;
    }
    fprintf(stderr, "unknown flow set %s\n", cfg);
    exit(2);
}

// the FPGA's schedule (inferno_tx.v), as golden.py inferno_schedule(): a power-of-two fpp starts at the next multiple
// of fpp after T0 (upstream bootstrap_next_ts: now + fpp - now % fpp), any other fpp at T0 + 1; a packet is due once
// its last frame is in; packets due at the same frame go out lower flow first
static void tx(const char *cfg, unsigned n)
{
    flow_t fl[4];
    unsigned nf = make_flows(cfg, fl);
    uint64_t next[4];
    for (unsigned i = 0; i < nf; i++)
        next[i] = (fl[i].fpp & (fl[i].fpp - 1)) == 0 ? T0 + fl[i].fpp - T0 % fl[i].fpp : T0 + 1;
    static uint8_t fr[MTU + 14];
    for (unsigned k = 0; k < n; k++) {
        unsigned best = 0;
        for (unsigned i = 1; i < nf; i++)
            if (next[i] + fl[i].fpp < next[best] + fl[best].fpp) best = i;
        size_t len = build_frame(fr, &fl[best], next[best], (uint16_t)k);
        for (size_t i = 0; i < len; i++) printf("%02x", fr[i]);
        printf("\n");
        next[best] += fl[best].fpp;
    }
}

// flows_rx.rs receive() on each line's UDP payload, matched to its flow by the destination port
static int rx(const char *cfg)
{
    flow_t fl[4];
    unsigned nf = make_flows(cfg, fl), npk = 0, nsmp = 0, errors = 0;
    static char line[2 * (MTU + 14) + 8];
    static uint8_t b[MTU + 14];
    while (fgets(line, sizeof line, stdin)) {
        size_t len = strlen(line) / 2;
        for (size_t i = 0; i < len; i++) sscanf(line + 2 * i, "%2hhx", &b[i]);
        uint16_t dport = (uint16_t)(b[36] << 8 | b[37]);
        uint16_t ulen = (uint16_t)(b[38] << 8 | b[39]);
        const flow_t *f = NULL;
        for (unsigned i = 0; i < nf; i++) if (fl[i].dport == dport) f = &fl[i];
        if (!f) { fprintf(stderr, "packet %u: no flow for port %u\n", npk, dport); errors++; continue; }
        const uint8_t *buf = b + 42;
        size_t recv_size = ulen - 8u;
        uint64_t timestamp = (uint64_t)rd32(buf + 1) * FS + rd32(buf + 5);     // wrapping in usize (64 bit)
        size_t stride = f->nch * f->bps, samples_count = (recv_size - 9) / stride;
        if (samples_count != f->fpp || buf[0] != 2) { fprintf(stderr, "packet %u: %zu frames\n", npk, samples_count); errors++; }
        for (unsigned i = 0; i < f->nch; i++) {
            for (size_t k = 0; k < samples_count; k++) {
                const uint8_t *p = buf + 9 + i * f->bps + k * stride;
                uint32_t s = (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | (f->bps == 4 ? p[3] : 0);
                uint32_t want = f->ch[i] < 0 ? 0 : (uint32_t)sample_at(timestamp + k, f->ch[i]);
                if (s != want) {
                    if (errors < 10) fprintf(stderr, "packet %u ch %u frame %zu: %08x want %08x\n", npk, i, k, s, want);
                    errors++;
                }
                nsmp++;
            }
        }
        npk++;
    }
    printf("%s: %u packets, %u samples decoded as upstream receive() does: %u errors\n", cfg, npk, nsmp, errors);
    return errors ? 1 : 0;
}

int main(int argc, char **argv)
{
    if (argc >= 4 && !strcmp(argv[1], "tx")) { tx(argv[2], (unsigned)atoi(argv[3])); return 0; }
    if (argc >= 3 && !strcmp(argv[1], "rx")) return rx(argv[2]);
    fprintf(stderr, "usage: inferno_xcheck tx <tb|cap64> <n> | rx <tb|cap64>\n");
    return 2;
}
