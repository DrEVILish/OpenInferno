/* pktq.c - see pktq.h. Part of OpenProSeries inferno-portd. Licence: GPL-3.0-or-later. */
#include "pktq.h"
#include "regs.h"
#include "util.h"

/* The SFD stamps' pipeline offsets, against io build 29 (rmii_rx reading the pins, as OpenInferno's rtl). COMPUTED
 * from the RTL's clk50 register count (20 ns a stage), the same on every jack and engine and at 48 / 96 kHz; still
 * waiting for a desk measurement. RX: the stamp is late by the extra input stages; TX: the SFD reaches the pin after
 * the stamp by the extra output stages */
static const struct { int from_build, rx_late_ns; } IO_RX_STAGES[] = {
    { 31, 40 },   /* b31 (M438): aes50_port_mux's shared IOB stage, rmii_rx's own stage now a plain second register */
    { 30, 20 },   /* b30 (M423): rmii_rx's IOB input stage */
};
#define XBAR_RX_LATE_NS 40   /* port_xbar (xc3s5000.md 8.11.21): RMII RX 1 -> 3 stages */
#define XBAR_TX_LATE_NS 20   /* RMII TX 1 -> 2 stages */

static int io_rx_late_ns(int build)
{
    for (unsigned i = 0; i < sizeof(IO_RX_STAGES) / sizeof(IO_RX_STAGES[0]); i++)
        if (build >= IO_RX_STAGES[i].from_build) return IO_RX_STAGES[i].rx_late_ns;
    return 0;
}

static void stamp_add(stamp_t *t, int32_t ns)
{
    int64_t v = (int64_t)t->ns + ns;
    if (v < 0) { v += 1000000000; t->sec--; }
    else if (v >= 1000000000) { v -= 1000000000; t->sec++; }
    t->ns = (uint32_t)v;
}

int pktq_init(hw_t *h)
{
    uint32_t id = port_rd(h, CQ_ID);
    if (id != CQ_ID_VALUE) { LOGE("cpu_pktq: ID 0x%08x, not OCQ1", id); return -1; }
    port_wr(h, CQ_CTRL, CQ_CTRL_RUN | 0x100);   /* flush, then run */
    port_wr(h, CQ_CTRL, CQ_CTRL_RUN);
    /* no aes_fcap (OFCP): the build is unknown, taken as before b30 */
    int build = hw_rd(h, FCAP_ID) == FCAP_ID_VALUE ? (int)((hw_rd(h, FCAP_INFO) >> 16) & 0xFF) : 0;
    int xbar = hw_rd(h, XBAR_ID) == XBAR_ID_VALUE;
    h->rx_stamp_ns = -(io_rx_late_ns(build) + (xbar ? XBAR_RX_LATE_NS : 0));
    h->tx_stamp_ns = xbar ? XBAR_TX_LATE_NS : 0;
    if (h->rx_stamp_ns || h->tx_stamp_ns)
        LOGI("io build %d%s: SFD stamps corrected RX %+d ns, TX %+d ns", build, xbar ? " with the crossbar" : "", (int)h->rx_stamp_ns,
             (int)h->tx_stamp_ns);
    return 0;
}

int pktq_send(hw_t *h, const uint8_t *f, int len, int ts_req, uint8_t tag)
{
    if (len < 14 || len > 1514) return -1;
    int words = (len + 3) / 4;
    if ((int)port_rd(h, CQ_TX_FREE) < words) return -1;
    for (int i = 0; i < words; i++) {
        uint32_t w = 0;
        for (int b = 0; b < 4; b++) w = (w << 8) | (uint32_t)(4 * i + b < len ? f[4 * i + b] : 0);
        port_wr(h, CQ_TX_DATA, w);
    }
    port_wr(h, CQ_TX_SEND, (uint32_t)len | (ts_req ? 0x8000u : 0) | ((uint32_t)tag << 16));
    return 0;
}

int pktq_tx_stamp(hw_t *h, uint8_t *tag, stamp_t *ts)
{
    uint32_t st = port_rd(h, CQ_TX_TS_STATUS);
    if (!(st & 0x80000000u)) return 0;
    *tag = (uint8_t)st;
    ts->sec = port_rd(h, CQ_TX_TS_SEC);
    ts->ns = port_rd(h, CQ_TX_TS_NS);   /* clears the valid bit */
    stamp_add(ts, h->tx_stamp_ns);
    return 1;
}

int pktq_recv(hw_t *h, uint8_t *buf, int max, stamp_t *ts)
{
    uint32_t l = port_rd(h, CQ_RX_LEN);
    if (!(l & 0x80000000u)) return 0;
    int len = (int)(l & 0x7FF);
    ts->sec = port_rd(h, CQ_RX_TS_SEC);
    ts->ns = port_rd(h, CQ_RX_TS_NS);
    stamp_add(ts, h->rx_stamp_ns);
    int words = (len + 3) / 4;
    for (int i = 0; i < words; i++) {
        uint32_t w = port_rd(h, CQ_RX_DATA);
        for (int b = 0; b < 4; b++)
            if (4 * i + b < len && 4 * i + b < max) buf[4 * i + b] = (uint8_t)(w >> (24 - 8 * b));
    }
    port_wr(h, CQ_RX_POP, 1);
    return len > max ? max : len;
}
