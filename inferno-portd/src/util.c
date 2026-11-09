/* util.c - see util.h. Part of OpenProSeries inferno-portd. Licence: GPL-3.0-or-later. */
#include "util.h"

#include <ctype.h>
#include <stdio.h>
#include <time.h>

int  g_log_level = LOG_INFO;
char g_log_tag[40] = "portd";

void bb_init(bbuf_t *b) { b->len = b->pos = 0; b->overflow = 0; }

static void bb_put(bbuf_t *b, const void *p, size_t n)
{
    if (b->pos + n > sizeof(b->d)) { b->overflow = 1; return; }
    if (p) memcpy(b->d + b->pos, p, n);
    else memset(b->d + b->pos, 0, n);
    b->pos += n;
    if (b->pos > b->len) b->len = b->pos;
}
void bb_u8(bbuf_t *b, uint8_t v) { bb_put(b, &v, 1); }
void bb_u16(bbuf_t *b, uint16_t v) { uint8_t t[2]; put16(t, v); bb_put(b, t, 2); }
void bb_u32(bbuf_t *b, uint32_t v) { uint8_t t[4]; put32(t, v); bb_put(b, t, 4); }
void bb_bytes(bbuf_t *b, const void *p, size_t n) { bb_put(b, p, n); }
void bb_zero(bbuf_t *b, size_t n) { bb_put(b, NULL, n); }
uint16_t bb_str0(bbuf_t *b, const char *s)
{
    uint16_t at = (uint16_t)b->pos;
    bb_put(b, s, strlen(s));
    bb_u8(b, 0);
    return at;
}
void bb_align(bbuf_t *b, size_t a) { while (b->pos % a) bb_u8(b, 0); }
void bb_seek(bbuf_t *b, size_t pos)
{
    if (pos > b->len) { size_t p0 = b->pos; b->pos = b->len; bb_zero(b, pos - b->len); (void)p0; }
    b->pos = pos;
}

const char *str_at(const uint8_t *buf, size_t len, size_t off, char *out, size_t outsz)
{
    if (off >= len || outsz == 0) return NULL;
    size_t n = 0;
    while (off + n < len && buf[off + n] != 0) n++;
    if (n >= outsz) n = outsz - 1;
    memcpy(out, buf + off, n);
    out[n] = 0;
    return out;
}

uint64_t mono_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

uint64_t (*g_now_ns)(void) = mono_ns;

static uint64_t rng_state = 0x9E3779B97F4A7C15ull;
void rnd_seed(uint64_t s) { rng_state = s ? s : 0x9E3779B97F4A7C15ull; }
uint32_t rnd32(void)
{
    /* xorshift64* */
    rng_state ^= rng_state >> 12;
    rng_state ^= rng_state << 25;
    rng_state ^= rng_state >> 27;
    return (uint32_t)((rng_state * 0x2545F4914F6CDD1Dull) >> 32);
}

void logm(int lvl, const char *fmt, ...)
{
    if (lvl > g_log_level) return;
    static const char *const L[] = { "E", "W", "I", "D" };
    char msg[1024];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    fprintf(stderr, "%ld.%03ld %s %s: %s\n", (long)ts.tv_sec % 100000, ts.tv_nsec / 1000000, L[lvl & 3], g_log_tag, msg);
}

void hexdump_str(const uint8_t *p, size_t n, char *out, size_t outsz)
{
    size_t o = 0;
    for (size_t i = 0; i < n && o + 3 < outsz; i++) o += (size_t)snprintf(out + o, outsz - o, "%02x", p[i]);
    if (outsz) out[o < outsz ? o : outsz - 1] = 0;
}

const char *ip_str(uint32_t ip, char *buf)
{
    snprintf(buf, 16, "%u.%u.%u.%u", ip >> 24, (ip >> 16) & 255, (ip >> 8) & 255, ip & 255);
    return buf;
}

int parse_ip(const char *s, uint32_t *ip)
{
    unsigned a, b, c, d;
    char tail;
    if (sscanf(s, "%u.%u.%u.%u%c", &a, &b, &c, &d, &tail) != 4 || a > 255 || b > 255 || c > 255 || d > 255) return -1;
    *ip = (a << 24) | (b << 16) | (c << 8) | d;
    return 0;
}

int parse_mac(const char *s, uint8_t mac[6])
{
    unsigned m[6];
    char tail;
    if (sscanf(s, "%x:%x:%x:%x:%x:%x%c", &m[0], &m[1], &m[2], &m[3], &m[4], &m[5], &tail) != 6) return -1;
    for (int i = 0; i < 6; i++) { if (m[i] > 255) return -1; mac[i] = (uint8_t)m[i]; }
    return 0;
}

void sanitize_name(const char *in, char *out, size_t max)
{
    size_t o = 0;
    for (const char *p = in; *p && o + 1 < max; p++) {
        unsigned char c = (unsigned char)*p;
        if (isalnum(c) || c == '-') out[o++] = (char)c;
        else if (c == ' ' || c == '_') out[o++] = '-';
    }
    out[o] = 0;
    size_t s = 0;
    while (out[s] == '-') s++;
    if (s) memmove(out, out + s, strlen(out + s) + 1);
    o = strlen(out);
    while (o && out[o - 1] == '-') out[--o] = 0;
}
