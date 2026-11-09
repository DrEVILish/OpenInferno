/* fpga_model.c - the software model of one FPGA Inferno port, the desk clock and the CS2000 engine (fpga_model.h).
 *
 * Written from the OpenProSeries RTL's documented behaviour (fpga/rtl/aoip_regs.v, inferno_regs.v, inferno_tx.v,
 * inferno_rx.v, inferno_lat.vh, cpu_pktq.v, ptp_clock.v, fs_phase.v) and fpga/docs/inferno.md sections 3-6; the media
 * packet is upstream Inferno's (github.com/teodly/inferno, flows_tx.rs / flows_rx.rs, commit 9767558e).
 * Part of OpenProSeries inferno-portd. Licence: GPL-3.0-or-later.
 */
#define _GNU_SOURCE
#include "fpga_model.h"
#include "regs.h"
#include "util.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <net/if.h>
#include <netinet/in.h>
#include <linux/if_packet.h>
#include <net/ethernet.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <time.h>
#include <sys/uio.h>
#include <unistd.h>

#define F_CLK      50000000.0
#define NTXF       16
#define NRXF       16
#define MAX_CH     64
#define MAXFCH     32
#define MAX_FPP    32
#define NR_FRAMES  128
#define CQ_ND      16
#define CQ_RXBYTES 8192
#define CQ_TXWORDS 512

typedef struct {
    uint8_t  d[1520];
    int      len;
    uint32_t ts_sec, ts_ns;
} cq_frame_t;

typedef struct {
    uint8_t  d[1520];
    int      len;
    uint64_t arrive_ns;
} pend_t;

#define WIRE_N 8192
struct model_wire {
    struct { pend_t f[WIRE_N]; int head, n; } q[2];   /* q[s] = frames travelling to side s */
    int cut;
};

model_wire_t *model_wire_new(void) { return calloc(1, sizeof(model_wire_t)); }
void model_wire_cut(model_wire_t *w, int cut) { w->cut = cut; }

typedef struct {
    model_cfg_t c;
    uint64_t last_ns;
    double   tick_frac;
    /* ptp_clock */
    uint64_t sec;
    uint32_t ns, frac24;
    uint32_t incr, media_inc;
    uint32_t nco, media_ts, media_sec, media_sub;
    uint32_t set_sec_hi, set_sec_lo, set_ns, set_media, ns_latch;
    /* aoip_regs of this port */
    uint32_t mode, src_mac_hi, src_mac_lo, src_ip, link_offset;
    uint32_t own_incr, own_minc;   /* inert copies when this port is not the clock port */
    /* inferno_regs */
    uint32_t inf_ctrl, tbl_addr, tx_tbl[256], rx_tbl[256], rx_st[256];
    uint32_t media_fs, set_msec, set_msub, latency;
    uint32_t tx_packets, tx_overruns, rx_ok, rx_bad, rx_late, rx_early, rx_underruns, rx_gaps;
    struct { int run; uint32_t next_t; int fpp; } txf[NTXF];
    uint16_t ipid;
    uint32_t colsrc[MAX_CH];
    uint32_t colsrc_sel;
    uint64_t pat_ok, pat_bad;
    /* cpu_pktq */
    uint32_t cq_ctrl;
    uint32_t txw[CQ_TXWORDS];
    int      txw_n;
    cq_frame_t rxq[CQ_ND];
    int      rxq_head, rxq_n, rx_bytes;
    int      rx_rd;                /* words read of the head frame */
    uint32_t ts_valid, ts_tag, ts_sec, ts_ns;
    uint32_t cq_tx_frames, cq_tx_rejects, cq_rx_frames, cq_rx_over, cq_rx_filt;
    pend_t   pend[64];
    int      pend_head, pend_n;
    /* the desk audio clock and fs_phase */
    double   desk_pos;             /* desk frames (samples), not wrapped */
    double   desk_ratio_rel;       /* CS2000 ratio / nominal */
    uint32_t fp_ctrl, fp_frames, l_ts, l_ticks, l_period, l_dperiod;
    uint32_t s_frames, s_ts, s_ticks, s_period, s_dperiod;
    double   media_pos_unwrapped;  /* media frames since power-up + NCO fraction */
    /* CS2000 engine */
    uint8_t  cs_regs[0x20];
    uint32_t cs_cmd, cs_echo;
    uint32_t cs_ratio, cs_max_step;   /* the ratio in force (0: R0 from power-up), the largest change one write made */
    uint64_t cs_applied;
    /* stats */
    uint64_t st_txcpu, st_rxcpu, st_txmedia, st_rxmedia;
} model_t;

static double tick_rate(const model_t *m) { return F_CLK * (1.0 + m->c.xtal_ppm * 1e-6) * 1e-9; }   /* ticks per ns */
static double desk_rate(const model_t *m)
{
    return (double)m->media_fs * (1.0 + m->c.desk_ppm * 1e-6) * m->desk_ratio_rel * 1e-9;   /* frames per ns */
}

static void advance_ticks(model_t *m, uint64_t ticks)
{
    uint64_t f = (uint64_t)m->frac24 + ticks * (m->incr & 0xFFFFFFu);
    uint64_t nsadd = ticks * (m->incr >> 24) + (f >> 24);
    m->frac24 = (uint32_t)(f & 0xFFFFFFu);
    uint64_t ns = (uint64_t)m->ns + nsadd;
    m->sec += ns / 1000000000ull;
    m->ns = (uint32_t)(ns % 1000000000ull);
    uint64_t t = (uint64_t)m->nco + ticks * (uint64_t)m->media_inc;
    uint32_t samples = (uint32_t)(t >> 32);
    m->nco = (uint32_t)t;
    m->media_ts += samples;
    m->media_pos_unwrapped += samples;
    uint64_t sub = (uint64_t)m->media_sub + samples;
    uint32_t fs = m->media_fs ? m->media_fs : 48000;
    m->media_sec += (uint32_t)(sub / fs);
    m->media_sub = (uint32_t)(sub % fs);
}

static void advance(model_t *m)
{
    uint64_t now = g_now_ns();
    if (now <= m->last_ns) return;
    uint64_t dt = now - m->last_ns;
    m->last_ns = now;
    double tf = (double)dt * tick_rate(m) + m->tick_frac;
    uint64_t ticks = (uint64_t)tf;
    m->tick_frac = tf - (double)ticks;
    advance_ticks(m, ticks);
    /* the desk audio clock */
    double before = m->desk_pos;
    m->desk_pos += (double)dt * desk_rate(m);
    if ((m->fp_ctrl & 1) && floor(m->desk_pos) > floor(before)) {
        m->fp_frames += (uint32_t)(floor(m->desk_pos) - floor(before));
        double back_ns = (m->desk_pos - floor(m->desk_pos)) / desk_rate(m);
        double back_ticks = back_ns * tick_rate(m);
        double minc = m->media_inc ? (double)m->media_inc : 1.0;
        double pos_now = (double)m->media_ts + (double)m->nco / 4294967296.0;
        double pos_edge = pos_now - back_ticks * minc / 4294967296.0;
        double fl = floor(pos_edge);
        m->l_ts = (uint32_t)(int64_t)fl;
        m->l_ticks = (uint32_t)((pos_edge - fl) * 4294967296.0 / minc);
        m->l_period = (uint32_t)(4294967296.0 / minc);
        m->l_dperiod = (uint32_t)(tick_rate(m) / desk_rate(m));
    }
}

/* the time of day at a host time near now (linear in the oscillator) */
static void tod_at(model_t *m, uint64_t host_ns, uint32_t *sec_lo, uint32_t *ns_out)
{
    double nsper = (double)m->incr / 16777216.0 * tick_rate(m);   /* tod ns per host ns */
    double d = ((double)(int64_t)(host_ns - m->last_ns)) * nsper;
    double tod = (double)m->ns + (double)m->frac24 / 16777216.0 + d;
    int64_t s = (int64_t)m->sec;
    while (tod < 0) { tod += 1e9; s--; }
    while (tod >= 1e9) { tod -= 1e9; s++; }
    *sec_lo = (uint32_t)s;
    *ns_out = (uint32_t)tod;
}

static uint32_t pattern(int id, int col, uint32_t t)
{
    return ((uint32_t)(id & 0xF) << 20) | ((uint32_t)(col & 0x3F) << 14) | (t & 0x3FFF);
}

static int fpp_auto(const model_t *m)
{
    int hi = m->media_fs > 50000, mode = (int)(m->latency & 3);
    static const int lo_f[4] = { 8, 16, 32, 32 }, hi_f[4] = { 16, 32, 32, 32 };
    return hi ? hi_f[mode] : lo_f[mode];
}
static int lat_offset(const model_t *m)
{
    static const int f[4] = { 24, 48, 96, 240 };
    if (m->latency & 0x100) return (int)m->link_offset;
    return f[m->latency & 3] * (m->media_fs > 50000 ? 2 : 1);
}
static int mtu_fpp(int nchb) { return nchb <= 45 ? 32 : nchb <= 91 ? 16 : 8; }

static int tx_eff_fpp(const model_t *m, uint32_t w0)
{
    int nch = (int)((w0 >> 8) & 0x1F) + 1, bps = (w0 & 2) ? 4 : 3;
    int f = (int)((w0 >> 2) & 0x3F);
    if (f == 0) f = fpp_auto(m);
    if (f > MAX_FPP) f = MAX_FPP;
    int lim = mtu_fpp(nch * bps);
    return f < lim ? f : lim;
}

static void tx_restart(model_t *m, int f)
{
    uint32_t w0 = m->tx_tbl[16 * f];
    if (!(w0 & 1)) { m->txf[f].run = 0; return; }
    int fpp = tx_eff_fpp(m, w0);
    m->txf[f].fpp = fpp;
    m->txf[f].run = 1;
    uint32_t now = m->media_ts;
    if ((fpp & (fpp - 1)) == 0) m->txf[f].next_t = (now / (uint32_t)fpp + 1) * (uint32_t)fpp;
    else m->txf[f].next_t = now + 1;
}

static void phy_send(model_t *m, const uint8_t *frame, int len)
{
    if (m->c.wire) {
        int to = !m->c.wire_side;
        typeof(m->c.wire->q[0]) *q = &m->c.wire->q[to];
        if (q->n >= WIRE_N || m->c.wire->cut) return;
        pend_t *f = &q->f[(q->head + q->n) % WIRE_N];
        memcpy(f->d, frame, (size_t)len);
        f->len = len;
        f->arrive_ns = g_now_ns() + m->c.wire_delay_ns;
        q->n++;
        return;
    }
    if (m->c.phy_fd < 0) return;
    if (m->c.phy_packet) {
        if (send(m->c.phy_fd, frame, (size_t)len, MSG_DONTWAIT) < 0 && errno != EAGAIN) LOGD("model PHY send: %d", errno);
    } else {
        uint8_t b[1530];
        uint64_t t = g_now_ns();
        memcpy(b, &t, 8);
        memcpy(b + 8, frame, (size_t)len);
        if (send(m->c.phy_fd, b, (size_t)len + 8, MSG_DONTWAIT) < 0 && errno != EAGAIN) LOGD("model wire send: %d", errno);
    }
}

static uint16_t ip_csum(const uint8_t *p, int n)
{
    uint32_t s = 0;
    for (int i = 0; i + 1 < n; i += 2) s += get16(p + i);
    while (s >> 16) s = (s & 0xFFFF) + (s >> 16);
    return (uint16_t)~s;
}

/* (sec, sub) of media time t, from the media clock's own pair (t is near media_ts) */
static void media_secsub(const model_t *m, uint32_t t, uint32_t *sec, uint32_t *sub)
{
    int32_t d = (int32_t)(t - m->media_ts);
    int64_t s = (int64_t)m->media_sec, u = (int64_t)m->media_sub + d;
    int64_t fs = m->media_fs ? m->media_fs : 48000;
    while (u < 0) { u += fs; s--; }
    while (u >= fs) { u -= fs; s++; }
    *sec = (uint32_t)s;
    *sub = (uint32_t)u;
}

static void tx_packet(model_t *m, int f, uint32_t t)
{
    const uint32_t *w = &m->tx_tbl[16 * f];
    int nch = (int)((w[0] >> 8) & 0x1F) + 1, bps = (w[0] & 2) ? 4 : 3, fpp = m->txf[f].fpp;
    int pl = fpp * nch * bps;
    uint8_t p[1514];
    memset(p, 0, 42);
    p[0] = (uint8_t)(w[1] >> 8); p[1] = (uint8_t)w[1]; put32(p + 2, w[2]);
    p[6] = (uint8_t)(m->src_mac_hi >> 8); p[7] = (uint8_t)m->src_mac_hi; put32(p + 8, m->src_mac_lo);
    p[12] = 0x08; p[13] = 0x00;
    uint8_t *ip = p + 14;
    ip[0] = 0x45; ip[1] = (uint8_t)(w[0] >> 16);
    put16(ip + 2, (uint16_t)(20 + 8 + 9 + pl));
    put16(ip + 4, m->ipid++);
    put16(ip + 6, 0x4000);
    ip[8] = (uint8_t)(w[0] >> 24); ip[9] = 17;
    put32(ip + 12, m->src_ip); put32(ip + 16, w[3]);
    put16(ip + 10, ip_csum(ip, 20));
    uint8_t *u = ip + 20;
    put16(u, (uint16_t)(w[4] >> 16)); put16(u + 2, (uint16_t)w[4]); put16(u + 4, (uint16_t)(8 + 9 + pl)); put16(u + 6, 0);
    uint8_t *mp = u + 8;
    uint32_t adv = (m->latency >> 16) & 0xFF, sec, sub;
    media_secsub(m, t - adv, &sec, &sub);
    mp[0] = 2; put32(mp + 1, sec); put32(mp + 5, sub);
    uint8_t *s = mp + 9;
    for (int j = 0; j < fpp; j++)
        for (int c = 0; c < nch; c++) {
            uint8_t mb = (uint8_t)(w[5 + c / 4] >> (8 * (c % 4)));
            uint32_t v = (mb & 0x80) || (mb & 0x3F) >= MAX_CH ? 0 : pattern(m->c.id, mb & 0x3F, t + (uint32_t)j);
            if (bps == 4) { put32(s, v << 8); s += 4; }
            else { s[0] = (uint8_t)(v >> 16); s[1] = (uint8_t)(v >> 8); s[2] = (uint8_t)v; s += 3; }
        }
    phy_send(m, p, 42 + 9 + pl);
    m->tx_packets++;
    m->st_txmedia++;
}

static void tx_run(model_t *m)
{
    if (!(m->inf_ctrl & 1) || (m->mode & 3) != MODE_INFERNO) return;
    for (int f = 0; f < NTXF; f++) {
        if (!m->txf[f].run) continue;
        int fpp = m->txf[f].fpp, n = 0;
        while ((int32_t)(m->media_ts - (m->txf[f].next_t + (uint32_t)fpp)) >= 0) {
            if (++n > 64) {   /* fell far behind (a stalled host): resynchronise, as an overrun */
                m->tx_overruns++;
                tx_restart(m, f);
                break;
            }
            tx_packet(m, f, m->txf[f].next_t);
            m->txf[f].next_t += (uint32_t)fpp;
        }
    }
}

/* the RX depacketiser: 1 if the frame is an enabled flow's (claimed) */
static int rx_claim(model_t *m, const uint8_t *p, int len)
{
    if (len < 42 + 9 || p[12] != 0x08 || p[13] != 0x00) return 0;
    const uint8_t *ip = p + 14;
    if (ip[0] != 0x45 || (get16(ip + 6) & 0x3FFF) != 0 || ip[9] != 17) return 0;
    uint32_t dip = get32(ip + 16);
    const uint8_t *u = ip + 20;
    uint16_t dport = get16(u + 2), ulen = get16(u + 4);
    if (!(m->inf_ctrl & 2) || (m->mode & 3) != MODE_INFERNO) return 0;
    for (int f = 0; f < NRXF; f++) {
        const uint32_t *w = &m->rx_tbl[16 * f];
        if (!(w[0] & 1) || w[1] != dip || (w[2] & 0xFFFF) != dport) continue;
        int bps = (int)((w[0] >> 16) & 7), nch = (int)((w[0] >> 8) & 0x3F);
        int pl = (int)ulen - 8 - 9;
        if (bps < 2 || bps > 4 || nch < 1 || pl <= 0 || 42 + 9 + pl > len || pl % (nch * bps)) { m->rx_bad++; return 1; }
        const uint8_t *mp = u + 8;
        uint32_t fs = m->media_fs ? m->media_fs : 48000;
        uint32_t t = get32(mp + 1) * fs + get32(mp + 5) + w[3];
        int frames = pl / (nch * bps);
        int off = lat_offset(m), nf = m->media_fs > 50000 ? 512 : 256;
        uint32_t play = m->media_ts - (uint32_t)off;
        if ((int32_t)(play - (t + (uint32_t)frames - 1)) > 0) { m->rx_late++; return 1; }
        if ((int32_t)((t + (uint32_t)frames) - (play + (uint32_t)nf)) > 0) { m->rx_early++; return 1; }
        m->rx_ok++;
        m->st_rxmedia++;
        uint32_t *st = &m->rx_st[16 * f];
        st[0] = get32(ip + 12);
        st[1] = get16(u);
        st[2]++;
        uint32_t lat = m->media_ts - t;
        if ((int32_t)lat > (int32_t)st[3]) st[3] = lat;
        const uint8_t *s = mp + 9;
        for (int j = 0; j < frames; j++)
            for (int c = 0; c < nch; c++, s += bps) {
                uint8_t mb = c < MAXFCH ? (uint8_t)(w[4 + c / 4] >> (8 * (c % 4))) : 0x80;
                if ((mb & 0x80) || (mb & 0x3F) >= MAX_CH || bps == 2) continue;
                uint32_t v = bps == 4 ? get32(s) >> 8 : ((uint32_t)s[0] << 16) | ((uint32_t)s[1] << 8) | s[2];
                if ((v & 0x3FFF) == ((t + (uint32_t)j - w[3]) & 0x3FFF)) m->pat_ok++;
                else m->pat_bad++;
                m->colsrc[mb & 0x3F] = 0x80000000u | (((v >> 20) & 0xF) << 16) | ((v >> 14) & 0x3F);
            }
        return 1;
    }
    return 0;
}

static void cq_enqueue(model_t *m, const uint8_t *p, int len, uint64_t arrive_ns)
{
    if (!(m->cq_ctrl & 2)) return;
    uint8_t mac[6] = { (uint8_t)(m->src_mac_hi >> 8), (uint8_t)m->src_mac_hi, (uint8_t)(m->src_mac_lo >> 24),
                       (uint8_t)(m->src_mac_lo >> 16), (uint8_t)(m->src_mac_lo >> 8), (uint8_t)m->src_mac_lo };
    int uc = memcmp(p, mac, 6) == 0, bc = memcmp(p, "\xff\xff\xff\xff\xff\xff", 6) == 0, mc = p[0] & 1;
    int pass = (m->cq_ctrl & 16) || uc || bc || (mc && (m->cq_ctrl & 4));
    if (pass && (m->cq_ctrl & 8) && len >= 34 && p[12] == 8 && p[13] == 0 && p[30] == 239 &&
        !(p[31] == 255 && p[32] == 255 && p[33] == 255))
        pass = 0;
    if (!pass) { m->cq_rx_filt++; return; }
    if (m->rxq_n >= CQ_ND || m->rx_bytes + len + 3 > CQ_RXBYTES) { m->cq_rx_over++; return; }
    cq_frame_t *f = &m->rxq[(m->rxq_head + m->rxq_n) % CQ_ND];
    memcpy(f->d, p, (size_t)len);
    f->len = len;
    tod_at(m, arrive_ns, &f->ts_sec, &f->ts_ns);
    m->rxq_n++;
    m->rx_bytes += (len + 3) & ~3;
    m->cq_rx_frames++;
    m->st_rxcpu++;
}

static void phy_input(model_t *m, const uint8_t *p, int len, uint64_t arrive_ns)
{
    if (len < 14) return;
    if (rx_claim(m, p, len)) return;
    cq_enqueue(m, p, len, arrive_ns);
}

static void phy_poll(model_t *m)
{
    uint64_t now = g_now_ns();
    /* frames that have arrived on the delayed wire */
    while (m->pend_n && m->pend[m->pend_head].arrive_ns <= now) {
        pend_t *q = &m->pend[m->pend_head];
        phy_input(m, q->d, q->len, q->arrive_ns);
        m->pend_head = (m->pend_head + 1) % 64;
        m->pend_n--;
    }
    if (m->c.wire) {
        typeof(m->c.wire->q[0]) *q = &m->c.wire->q[m->c.wire_side];
        while (q->n && q->f[q->head].arrive_ns <= now) {
            pend_t *f = &q->f[q->head];
            phy_input(m, f->d, f->len, f->arrive_ns);
            q->head = (q->head + 1) % WIRE_N;
            q->n--;
        }
        return;
    }
    if (m->c.phy_fd < 0) return;
    for (int k = 0; k < 256; k++) {
        uint8_t b[1600];
        if (m->c.phy_packet) {
            struct sockaddr_ll sll;
            char cbuf[128];
            struct iovec iov = { b, sizeof(b) };
            struct msghdr mh;
            memset(&mh, 0, sizeof(mh));
            mh.msg_name = &sll; mh.msg_namelen = sizeof(sll);
            mh.msg_iov = &iov; mh.msg_iovlen = 1;
            mh.msg_control = cbuf; mh.msg_controllen = sizeof(cbuf);
            ssize_t n = recvmsg(m->c.phy_fd, &mh, MSG_DONTWAIT);
            if (n <= 0) break;
            if (sll.sll_pkttype == PACKET_OUTGOING) continue;
            /* the kernel's receive time (SO_TIMESTAMPNS, CLOCK_REALTIME) as our host time: the model's stand-in for
             * the SFD stamp, not the time this 1 ms poll ran */
            uint64_t at = now;
            for (struct cmsghdr *c = CMSG_FIRSTHDR(&mh); c; c = CMSG_NXTHDR(&mh, c))
                if (c->cmsg_level == SOL_SOCKET && c->cmsg_type == SCM_TIMESTAMPNS) {
                    struct timespec ts, rt;
                    memcpy(&ts, CMSG_DATA(c), sizeof(ts));
                    clock_gettime(CLOCK_REALTIME, &rt);
                    int64_t ago = ((int64_t)rt.tv_sec - ts.tv_sec) * 1000000000LL + (rt.tv_nsec - ts.tv_nsec);
                    if (ago >= 0 && ago < 100000000LL) at = now - (uint64_t)ago;
                }
            phy_input(m, b, (int)n, at);
        } else {
            ssize_t n = recv(m->c.phy_fd, b, sizeof(b), MSG_DONTWAIT);
            if (n <= 8) break;
            uint64_t sent;
            memcpy(&sent, b, 8);
            uint64_t arrive = sent + m->c.wire_delay_ns;
            if (arrive <= now && m->pend_n == 0) phy_input(m, b + 8, (int)n - 8, arrive);
            else if (m->pend_n < 64) {
                pend_t *q = &m->pend[(m->pend_head + m->pend_n) % 64];
                memcpy(q->d, b + 8, (size_t)n - 8);
                q->len = (int)n - 8;
                q->arrive_ns = arrive;
                m->pend_n++;
            }
        }
    }
}

/* ---- the CS2000 engine (the chip's ratio registers 0x06..0x09 and Freeze, Global Configuration 0x05 bit 3) */
static void cs2k_apply(model_t *m)
{
    if (m->cs_regs[5] & 0x08) return;   /* frozen: applied when Freeze clears */
    uint32_t r = ((uint32_t)m->cs_regs[6] << 24) | ((uint32_t)m->cs_regs[7] << 16) | ((uint32_t)m->cs_regs[8] << 8) | m->cs_regs[9];
    if (!r || !m->c.cs2k_r0) return;
    uint32_t was = m->cs_ratio ? m->cs_ratio : m->c.cs2k_r0, step = r > was ? r - was : was - r;
    if (step > m->cs_max_step) m->cs_max_step = step;
    m->cs_ratio = r;
    m->cs_applied++;
    m->desk_ratio_rel = (double)r / (double)m->c.cs2k_r0;
}

/* ---- registers */
static uint32_t inf_rd(model_t *m, uint32_t o)
{
    switch (o) {
    case INF_ID: return INF_ID_VALUE;
    case INF_CTRL: return 0x80000000u | (m->inf_ctrl & 0x7F03);
    case INF_TBL_ADDR: return m->tbl_addr;
    case INF_TBL_DATA: {
        uint32_t a = m->tbl_addr & 0x3FF, v;
        if (a < 0x100) v = m->tx_tbl[a];
        else if (a < 0x200) v = m->rx_tbl[a - 0x100];
        else if (a < 0x300) v = (a & 15) < 4 ? m->rx_st[a - 0x200] : 0;
        else v = 0;
        m->tbl_addr = (m->tbl_addr + 1) & 0x3FF;
        return v;
    }
    case INF_TX_PENDING: return 0;
    case INF_TX_RUNNING: { uint32_t r = 0; for (int f = 0; f < NTXF; f++) if (m->txf[f].run) r |= 1u << f; return r; }
    case INF_MEDIA_FS: return m->media_fs;
    case INF_SET_MSEC: return m->set_msec;
    case INF_SET_MSUB: return m->set_msub;
    case INF_MEDIA_SEC: return m->media_sec;
    case INF_MEDIA_SUB: return m->media_sub;
    case INF_LATENCY: return m->latency & 0xFF0103;
    case INF_LAT_STATUS: return ((uint32_t)fpp_auto(m) << 16) | ((uint32_t)lat_offset(m) & 0x7FF);
    case INF_JB_GEOM: return m->media_fs > 50000 ? ((32u << 16) | 512u) : ((64u << 16) | 256u);
    case INF_CAPS: return ((uint32_t)NRXF << 24) | ((uint32_t)NTXF << 16) | ((uint32_t)MAXFCH << 8) | MAX_CH;
    case INF_CAPS2: return ((uint32_t)NR_FRAMES << 16) | ((uint32_t)(m->c.maxlat & 3) << 8) | MAX_FPP;
    case INF_TX_PACKETS: return m->tx_packets;
    case INF_TX_OVERRUNS: return m->tx_overruns;
    case INF_RX_OK: return m->rx_ok;
    case INF_RX_BAD: return m->rx_bad;
    case INF_RX_LATE: return m->rx_late;
    case INF_RX_EARLY: return m->rx_early;
    case INF_RX_UNDERRUNS: return m->rx_underruns;
    case INF_RX_TS_GAPS: return m->rx_gaps;
    case INF_MODEL_RXPAT_OK: return (uint32_t)m->pat_ok;
    case INF_MODEL_RXPAT_BAD: return (uint32_t)m->pat_bad;
    case INF_MODEL_COLSRC: return m->colsrc[m->colsrc_sel & 63];
    default: return 0;
    }
}

static void inf_wr(model_t *m, uint32_t o, uint32_t v)
{
    switch (o) {
    case INF_CTRL: m->inf_ctrl = v & 0x7F03; break;
    case INF_TBL_ADDR: m->tbl_addr = v & 0x3FF; break;
    case INF_TBL_DATA: {
        uint32_t a = m->tbl_addr & 0x3FF;
        if (a < 0x100) {
            m->tx_tbl[a] = v;
            if ((a & 15) == 0) tx_restart(m, (int)(a >> 4));
        } else if (a < 0x200) m->rx_tbl[a - 0x100] = v;
        else if (a < 0x300 && (a & 15) == 3) m->rx_st[a - 0x200] = 0;   /* w3: clear the latency peak */
        m->tbl_addr = (m->tbl_addr + 1) & 0x3FF;
        break;
    }
    case INF_MEDIA_FS: m->media_fs = v & 0x1FFFF; break;
    case INF_SET_MSEC: m->set_msec = v; break;
    case INF_SET_MSUB: m->set_msub = v & 0x1FFFF; break;
    case INF_LATENCY: {
        uint32_t mode = v & 3;
        if ((int)mode > m->c.maxlat) mode = m->latency & 3;   /* refused: the field keeps its value */
        m->latency = (v & 0xFF0100) | mode;
        for (int f = 0; f < NTXF; f++) if (m->txf[f].run && tx_eff_fpp(m, m->tx_tbl[16 * f]) != m->txf[f].fpp) tx_restart(m, f);
        break;
    }
    case INF_MODEL_COLSRC: m->colsrc_sel = v; break;
    default: break;
    }
}

static uint32_t cq_rd(model_t *m, uint32_t o)
{
    switch (o) {
    case CQ_ID: return CQ_ID_VALUE;
    case CQ_CTRL: return m->cq_ctrl & 0x1F;
    case CQ_TX_FREE: return (uint32_t)(CQ_TXWORDS - m->txw_n);
    case CQ_TX_STATUS: return 0;
    case CQ_TX_TS_STATUS: return (m->ts_valid ? 0x80000000u : 0) | (m->ts_tag & 0xFF);
    case CQ_TX_TS_SEC: return m->ts_sec;
    case CQ_TX_TS_NS: { uint32_t r = m->ts_ns; m->ts_valid = 0; return r; }
    case CQ_RX_STATUS: return (uint32_t)m->rxq_n;
    case CQ_RX_LEN: return m->rxq_n ? 0x80000000u | (uint32_t)m->rxq[m->rxq_head].len : 0;
    case CQ_RX_TS_SEC: return m->rxq_n ? m->rxq[m->rxq_head].ts_sec : 0;
    case CQ_RX_TS_NS: return m->rxq_n ? m->rxq[m->rxq_head].ts_ns : 0;
    case CQ_RX_DATA: {
        if (!m->rxq_n) return 0;
        cq_frame_t *f = &m->rxq[m->rxq_head];
        int b = m->rx_rd * 4;
        if (b >= f->len) return 0;
        uint32_t w = 0;
        for (int i = 0; i < 4; i++) w = (w << 8) | (uint32_t)(b + i < f->len ? f->d[b + i] : 0);
        m->rx_rd++;
        return w;
    }
    case CQ_TX_FRAMES: return m->cq_tx_frames;
    case CQ_TX_REJECTS: return m->cq_tx_rejects;
    case CQ_RX_FRAMES: return m->cq_rx_frames;
    case CQ_RX_OVERFLOWS: return m->cq_rx_over;
    case CQ_RX_FILTERED: return m->cq_rx_filt;
    default: return 0;
    }
}

static void cq_wr(model_t *m, uint32_t o, uint32_t v)
{
    switch (o) {
    case CQ_CTRL:
        m->cq_ctrl = v & 0x1F;
        if (v & 0x100) { m->txw_n = 0; m->rxq_n = 0; m->rx_bytes = 0; m->rx_rd = 0; }
        break;
    case CQ_TX_DATA:
        if (m->txw_n < CQ_TXWORDS) m->txw[m->txw_n++] = v;
        break;
    case CQ_TX_SEND: {
        int len = (int)(v & 0x7FF);
        if (len < 14 || len > 1514 || (len + 3) / 4 != m->txw_n) { m->cq_tx_rejects++; m->txw_n = 0; break; }
        uint8_t f[1514];
        for (int i = 0; i < len; i++) f[i] = (uint8_t)(m->txw[i / 4] >> (24 - 8 * (i % 4)));
        m->txw_n = 0;
        if (!(m->cq_ctrl & 1)) break;
        phy_send(m, f, len);
        m->cq_tx_frames++;
        m->st_txcpu++;
        if (v & 0x8000) {
            tod_at(m, g_now_ns(), &m->ts_sec, &m->ts_ns);
            m->ts_valid = 1;
            m->ts_tag = (v >> 16) & 0xFF;
        }
        break;
    }
    case CQ_RX_POP:
        if (m->rxq_n) {
            m->rx_bytes -= (m->rxq[m->rxq_head].len + 3) & ~3;
            m->rxq_head = (m->rxq_head + 1) % CQ_ND;
            m->rxq_n--;
            m->rx_rd = 0;
        }
        break;
    default: break;
    }
}

static int is_clock(const model_t *m, uint32_t blk) { return blk == m->c.clock_base; }

static uint32_t aoip_rd(model_t *m, uint32_t blk, uint32_t o)
{
    int ck = is_clock(m, blk), own = blk == m->c.port_base;
    switch (o) {
    case AOIP_ID: return AOIP_ID_VALUE;
    case AOIP_MODE: return own ? ((m->mode & 3) << 4) | (m->mode & 3) : 0;
    case AOIP_SRC_MAC_HI: return own ? m->src_mac_hi : 0;
    case AOIP_SRC_MAC_LO: return own ? m->src_mac_lo : 0;
    case AOIP_SRC_IP: return own ? m->src_ip : 0;
    case AOIP_LINK_OFFSET: return own ? m->link_offset : 48;
    case AOIP_PTP_INCR: return ck ? m->incr : m->own_incr;
    case AOIP_MEDIA_INC: return ck ? m->media_inc : m->own_minc;
    case AOIP_SET_SEC_HI: return m->set_sec_hi;
    case AOIP_SET_SEC_LO: return m->set_sec_lo;
    case AOIP_SET_NS: return m->set_ns;
    case AOIP_SET_MEDIA: return m->set_media;
    case AOIP_MEDIA_TS: return m->media_ts;
    case AOIP_TOD_SEC_LO: m->ns_latch = m->ns; return (uint32_t)m->sec;
    case AOIP_TOD_NS: return m->ns_latch;
    default: return 0;
    }
}

static void aoip_wr(model_t *m, uint32_t blk, uint32_t o, uint32_t v)
{
    int ck = is_clock(m, blk), own = blk == m->c.port_base;
    switch (o) {
    case AOIP_MODE: if (own) m->mode = v & 3; break;
    case AOIP_SRC_MAC_HI: if (own) m->src_mac_hi = v & 0xFFFF; break;
    case AOIP_SRC_MAC_LO: if (own) m->src_mac_lo = v; break;
    case AOIP_SRC_IP: if (own) m->src_ip = v; break;
    case AOIP_LINK_OFFSET: if (own) m->link_offset = v; break;
    case AOIP_PTP_INCR: if (ck) m->incr = v; else m->own_incr = v; break;
    case AOIP_MEDIA_INC: if (ck) m->media_inc = v; else m->own_minc = v; break;
    case AOIP_SET_SEC_HI: m->set_sec_hi = v & 0xFFFF; break;
    case AOIP_SET_SEC_LO: m->set_sec_lo = v; break;
    case AOIP_SET_NS: m->set_ns = v & 0x3FFFFFFF; break;
    case AOIP_SET_MEDIA: m->set_media = v; break;
    case AOIP_CLOCK_CMD:
        if (!ck) break;
        if (v & 1) { m->sec = ((uint64_t)m->set_sec_hi << 32) | m->set_sec_lo; m->ns = m->set_ns; m->frac24 = 0; }
        if (v & 2) {
            m->media_pos_unwrapped += (double)(int32_t)(m->set_media - m->media_ts);
            m->media_ts = m->set_media; m->media_sec = m->set_msec; m->media_sub = m->set_msub;
        }
        break;
    default: break;
    }
}

static uint32_t m_rd(hw_t *h, uint32_t a)
{
    model_t *m = h->priv;
    advance(m);
    if (a >= m->c.port_base && a < m->c.port_base + 0x1000) {
        uint32_t o = a - m->c.port_base;
        if (o < 0x100) return aoip_rd(m, m->c.port_base, o);
        if (o < 0x200) return inf_rd(m, o);
        if (o < 0x300) return cq_rd(m, o);
        return 0;
    }
    if (a >= m->c.clock_base && a < m->c.clock_base + 0x100) return aoip_rd(m, m->c.clock_base, a - m->c.clock_base);
    if (a >= m->c.clock_base + INF_BASE && a < m->c.clock_base + INF_BASE + 0x100) {
        /* the clock port's media-clock registers: one desk clock in the model (the port's own block aliases it) */
        uint32_t o = a - m->c.clock_base;
        if (o == INF_MEDIA_FS || o == INF_SET_MSEC || o == INF_SET_MSUB || o == INF_MEDIA_SEC || o == INF_MEDIA_SUB || o == INF_ID)
            return inf_rd(m, o);
        return 0;
    }
    if (m->c.dclk_base != 0xFFFFFFFFu && a >= m->c.dclk_base && a < m->c.dclk_base + 0x20) {
        switch (a - m->c.dclk_base) {
        case FP_ID: return FP_ID_VALUE;
        case FP_CTRL: return m->fp_ctrl & 1;
        case FP_DESK_FRAMES: return m->s_frames;
        case FP_MEDIA_TS: return m->s_ts;
        case FP_TICKS: return m->s_ticks;
        case FP_PERIOD: return m->s_period;
        case FP_DESK_PERIOD: return m->s_dperiod;
        default: return 0;
        }
    }
    if (m->c.cs2k_base != 0xFFFFFFFFu && a >= m->c.cs2k_base && a < m->c.cs2k_base + 0x10) {
        if (a - m->c.cs2k_base == CS2K_ECHO) return m->cs_echo;
        if (a - m->c.cs2k_base == CS2K_CMD) return m->cs_cmd;
    }
    return 0;
}

static void m_wr(hw_t *h, uint32_t a, uint32_t v)
{
    model_t *m = h->priv;
    advance(m);
    if (a >= m->c.port_base && a < m->c.port_base + 0x1000) {
        uint32_t o = a - m->c.port_base;
        if (o < 0x100) aoip_wr(m, m->c.port_base, o, v);
        else if (o < 0x200) inf_wr(m, o, v);
        else if (o < 0x300) cq_wr(m, o, v);
        return;
    }
    if (a >= m->c.clock_base && a < m->c.clock_base + 0x100) { aoip_wr(m, m->c.clock_base, a - m->c.clock_base, v); return; }
    if (a >= m->c.clock_base + INF_BASE && a < m->c.clock_base + INF_BASE + 0x100) {
        uint32_t o = a - m->c.clock_base;
        if (o == INF_MEDIA_FS || o == INF_SET_MSEC || o == INF_SET_MSUB) inf_wr(m, o, v);
        return;
    }
    if (m->c.dclk_base != 0xFFFFFFFFu && a >= m->c.dclk_base && a < m->c.dclk_base + 0x20) {
        switch (a - m->c.dclk_base) {
        case FP_CTRL:
            m->fp_ctrl = v & 1;
            if (v & 2) m->fp_frames = 0;
            break;
        case FP_SNAP:
            m->s_frames = m->fp_frames; m->s_ts = m->l_ts; m->s_ticks = m->l_ticks;
            m->s_period = m->l_period; m->s_dperiod = m->l_dperiod;
            break;
        default: break;
        }
        return;
    }
    if (m->c.cs2k_base != 0xFFFFFFFFu && a >= m->c.cs2k_base && a < m->c.cs2k_base + 0x10) {
        uint32_t o = a - m->c.cs2k_base;
        if (o == CS2K_CMD) m->cs_cmd = v & 0xFFFF;
        else if (o == CS2K_STROBE && (v & 1)) {
            uint8_t reg = (uint8_t)(m->cs_cmd >> 8), d = (uint8_t)m->cs_cmd;
            if (reg < sizeof(m->cs_regs)) m->cs_regs[reg] = d;
            m->cs_echo = m->cs_cmd & 0xFFFF;   /* busy clears at once in the model */
            if (reg == 5 || (reg >= 6 && reg <= 9)) cs2k_apply(m);
        }
    }
}

static int m_fd(hw_t *h) { model_t *m = h->priv; return m->c.phy_fd; }

static void m_service(hw_t *h)
{
    model_t *m = h->priv;
    advance(m);
    phy_poll(m);
    tx_run(m);
}

static void m_close(hw_t *h)
{
    model_t *m = h->priv;
    if (m->c.phy_fd >= 0) close(m->c.phy_fd);
    free(m);
    free(h);
}

static const struct hw_ops model_ops = { m_rd, m_wr, m_fd, m_service, m_close };

hw_t *model_open(const model_cfg_t *cfg)
{
    model_t *m = calloc(1, sizeof(*m));
    hw_t *h = calloc(1, sizeof(*h));
    m->c = *cfg;
    m->last_ns = g_now_ns();
    m->incr = m->own_incr = 0x14000000u;
    m->media_inc = m->own_minc = 0x003EEA21u;
    m->link_offset = 48;
    m->media_fs = 48000;
    m->inf_ctrl = 64u << 8;
    m->cq_ctrl = 8;
    m->desk_ratio_rel = 1.0;
    m->sec = cfg->tod_start_ns / 1000000000ull;
    m->ns = (uint32_t)(cfg->tod_start_ns % 1000000000ull);
    h->ops = &model_ops;
    h->priv = m;
    h->port_base = cfg->port_base;
    h->clock_base = cfg->clock_base;
    h->dclk_base = cfg->dclk_base;
    h->cs2k_base = cfg->cs2k_base;
    return h;
}

int model_packet_socket(const char *ifname)
{
    int fd = socket(AF_PACKET, SOCK_RAW | SOCK_NONBLOCK, htons(ETH_P_ALL));
    if (fd < 0) { LOGE("AF_PACKET socket: %d", errno); return -1; }
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "%s", ifname);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) { LOGE("no interface %s", ifname); close(fd); return -1; }
    struct sockaddr_ll sll;
    memset(&sll, 0, sizeof(sll));
    sll.sll_family = AF_PACKET;
    sll.sll_protocol = htons(ETH_P_ALL);
    sll.sll_ifindex = ifr.ifr_ifindex;
    if (bind(fd, (struct sockaddr *)&sll, sizeof(sll)) < 0) { LOGE("bind %s: %d", ifname, errno); close(fd); return -1; }
    struct packet_mreq mr;
    memset(&mr, 0, sizeof(mr));
    mr.mr_ifindex = ifr.ifr_ifindex;
    mr.mr_type = PACKET_MR_PROMISC;
    setsockopt(fd, SOL_PACKET, PACKET_ADD_MEMBERSHIP, &mr, sizeof(mr));
    int on = 1;
    setsockopt(fd, SOL_SOCKET, SO_TIMESTAMPNS, &on, sizeof(on));
    int rcv = 4 << 20;
    setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &rcv, sizeof(rcv));
    return fd;
}

void model_get_stats(hw_t *h, model_stats_t *s)
{
    model_t *m = h->priv;
    advance(m);
    memset(s, 0, sizeof(*s));
    s->tx_frames_cpu = m->st_txcpu; s->rx_frames_cpu = m->st_rxcpu;
    s->tx_media = m->st_txmedia; s->rx_media = m->st_rxmedia;
    s->rx_late = m->rx_late; s->rx_early = m->rx_early; s->rx_bad = m->rx_bad;
    s->pat_ok = m->pat_ok; s->pat_bad = m->pat_bad;
    s->tod_s = (double)m->sec + ((double)m->ns + (double)m->frac24 / 16777216.0) * 1e-9;
    s->media_pos = m->media_pos_unwrapped + (double)m->nco / 4294967296.0;
    s->desk_pos = m->desk_pos;
    s->desk_rate_rel = (1.0 + m->c.desk_ppm * 1e-6) * m->desk_ratio_rel;
    s->tick_rate_rel = 1.0 + m->c.xtal_ppm * 1e-6;
    s->cs2k_ratio = m->cs_ratio ? m->cs_ratio : m->c.cs2k_r0;
    s->cs2k_max_step = m->cs_max_step;
    s->cs2k_applied = m->cs_applied;
}

int model_colsrc(hw_t *h, int col, int *tx_id, int *tx_col)
{
    model_t *m = h->priv;
    if (col < 0 || col >= MAX_CH || !(m->colsrc[col] & 0x80000000u)) return 0;
    *tx_id = (int)((m->colsrc[col] >> 16) & 0xF);
    *tx_col = (int)(m->colsrc[col] & 0x3F);
    return 1;
}
