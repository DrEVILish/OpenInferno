/* pktq.c - see pktq.h. Part of OpenProSeries inferno-portd. Licence: GPL-3.0-or-later. */
#include "pktq.h"
#include "regs.h"
#include "util.h"

int pktq_init(hw_t *h)
{
    uint32_t id = port_rd(h, CQ_ID);
    if (id != CQ_ID_VALUE) { LOGE("cpu_pktq: ID 0x%08x, not OCQ1", id); return -1; }
    port_wr(h, CQ_CTRL, CQ_CTRL_RUN | 0x100);   /* flush, then run */
    port_wr(h, CQ_CTRL, CQ_CTRL_RUN);
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
    return 1;
}

int pktq_recv(hw_t *h, uint8_t *buf, int max, stamp_t *ts)
{
    uint32_t l = port_rd(h, CQ_RX_LEN);
    if (!(l & 0x80000000u)) return 0;
    int len = (int)(l & 0x7FF);
    ts->sec = port_rd(h, CQ_RX_TS_SEC);
    ts->ns = port_rd(h, CQ_RX_TS_NS);
    int words = (len + 3) / 4;
    for (int i = 0; i < words; i++) {
        uint32_t w = port_rd(h, CQ_RX_DATA);
        for (int b = 0; b < 4; b++)
            if (4 * i + b < len && 4 * i + b < max) buf[4 * i + b] = (uint8_t)(w >> (24 - 8 * b));
    }
    port_wr(h, CQ_RX_POP, 1);
    return len > max ? max : len;
}
