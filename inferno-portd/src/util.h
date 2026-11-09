/* util.h - byte order helpers, time, logging for inferno-portd.
 *
 * Part of OpenProSeries inferno-portd (the AMU control plane of the FPGA Inferno ports,
 * fpga/docs/inferno.md section 9). Licence: GPL-3.0-or-later.
 */
#ifndef PORTD_UTIL_H
#define PORTD_UTIL_H

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

static inline uint16_t get16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
static inline uint32_t get32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}
static inline void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }
static inline void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v;
}

/* a growable byte buffer with a write position (upstream Inferno's ByteBuffer, as the ARC code uses it) */
typedef struct {
    uint8_t d[1600];
    size_t  len;      /* bytes written (high-water) */
    size_t  pos;      /* write position */
    int     overflow;
} bbuf_t;
void   bb_init(bbuf_t *b);
void   bb_u8(bbuf_t *b, uint8_t v);
void   bb_u16(bbuf_t *b, uint16_t v);
void   bb_u32(bbuf_t *b, uint32_t v);
void   bb_bytes(bbuf_t *b, const void *p, size_t n);
void   bb_zero(bbuf_t *b, size_t n);
/* writes s and a 0, returns the offset it was written at */
uint16_t bb_str0(bbuf_t *b, const char *s);
void   bb_align(bbuf_t *b, size_t a);
void   bb_seek(bbuf_t *b, size_t pos);

/* a 0-terminated string at an offset of a buffer (upstream read_0term_str_from_buffer); NULL past the end */
const char *str_at(const uint8_t *buf, size_t len, size_t off, char *out, size_t outsz);

/* bounded copy that always terminates */
static inline void scopy(char *d, size_t sz, const char *s)
{
    size_t l = strlen(s);
    if (l >= sz) l = sz - 1;
    memmove(d, s, l);
    d[l] = 0;
}

uint64_t mono_ns(void);
/* the time source of the daemon (CLOCK_MONOTONIC; tests install a virtual clock) */
extern uint64_t (*g_now_ns)(void);
#define g_now() g_now_ns()
uint32_t rnd32(void);
void     rnd_seed(uint64_t s);

enum { LOG_ERR = 0, LOG_WARN = 1, LOG_INFO = 2, LOG_DEBUG = 3 };
extern int  g_log_level;
extern char g_log_tag[40];
void logm(int lvl, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
#define LOGE(...) logm(LOG_ERR, __VA_ARGS__)
#define LOGW(...) logm(LOG_WARN, __VA_ARGS__)
#define LOGI(...) logm(LOG_INFO, __VA_ARGS__)
#define LOGD(...) logm(LOG_DEBUG, __VA_ARGS__)

void hexdump_str(const uint8_t *p, size_t n, char *out, size_t outsz);
const char *ip_str(uint32_t ip, char *buf);   /* buf >= 16 */
int  parse_ip(const char *s, uint32_t *ip);
int  parse_mac(const char *s, uint8_t mac[6]);
/* Dante's device-name rules, as OMC's SanitizeDeskName: letters, digits, '-', not starting / ending with '-' */
void sanitize_name(const char *in, char *out, size_t max);

#endif
