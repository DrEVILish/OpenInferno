/* regs.h - the FPGA register map inferno-portd drives, from the OpenProSeries RTL:
 *   fpga/rtl/aoip_regs.v (OAP2), fpga/rtl/inferno_regs.v (OIN2), fpga/rtl/cpu_pktq.v (OCQ1),
 *   fpga/rtl/fs_phase.v (OFP1, the desk-clock phase detector) and the CS2000 command engine (docs/68 section 10.3).
 * Offsets are bytes inside a port's 4 KB block; the block's base is a daemon option (top_ecp5: 0x1000 x (n + 1)).
 * Licence: GPL-3.0-or-later.
 */
#ifndef PORTD_REGS_H
#define PORTD_REGS_H

/* ---- aoip_regs.v (port block + 0x000) */
#define AOIP_ID          0x000   /* "OAP2" */
#define AOIP_MODE        0x004   /* bits 1:0 requested: 0 AES50, 1 Inferno, 2 ACE, 3 off; RO 5:4 active, 8 switching */
#define AOIP_SRC_MAC_HI  0x014
#define AOIP_SRC_MAC_LO  0x018
#define AOIP_SRC_IP      0x01C
#define AOIP_LINK_OFFSET 0x03C
#define AOIP_PTP_INCR    0x040   /* ns per clock, 8.24 */
#define AOIP_MEDIA_INC   0x044   /* fs / f_clk x 2^32 */
#define AOIP_SET_SEC_HI  0x048
#define AOIP_SET_SEC_LO  0x04C
#define AOIP_SET_NS      0x050
#define AOIP_CLOCK_CMD   0x054   /* bit0 step the time of day, bit1 load the media clock */
#define AOIP_SET_MEDIA   0x058
#define AOIP_MEDIA_TS    0x05C
#define AOIP_TOD_SEC_LO  0x060   /* reading it latches TOD_NS */
#define AOIP_TOD_NS      0x064
#define AOIP_ID_VALUE    0x4F415032u
#define MODE_INFERNO     1

/* ---- inferno_regs.v (port block + 0x100) */
#define INF_BASE         0x100
#define INF_ID           (INF_BASE + 0x00)   /* "OIN2" */
#define INF_CTRL         (INF_BASE + 0x04)   /* bit0 TX en, bit1 RX en, bits 14:8 columns played */
#define INF_TBL_ADDR     (INF_BASE + 0x08)
#define INF_TBL_DATA     (INF_BASE + 0x0C)
#define INF_TX_PENDING   (INF_BASE + 0x10)
#define INF_TX_RUNNING   (INF_BASE + 0x14)
#define INF_MEDIA_FS     (INF_BASE + 0x18)
#define INF_SET_MSEC     (INF_BASE + 0x1C)
#define INF_SET_MSUB     (INF_BASE + 0x20)
#define INF_MEDIA_SEC    (INF_BASE + 0x24)
#define INF_MEDIA_SUB    (INF_BASE + 0x28)
#define INF_LATENCY      (INF_BASE + 0x2C)   /* bits 1:0 mode, bit 8 MANUAL, bits 23:16 TX_TS_ADVANCE */
#define INF_LAT_STATUS   (INF_BASE + 0x30)
#define INF_JB_GEOM      (INF_BASE + 0x34)
#define INF_CAPS         (INF_BASE + 0x38)   /* 7:0 MAX_CH, 15:8 MAXFCH, 23:16 TX flows, 31:24 RX flows */
#define INF_CAPS2        (INF_BASE + 0x3C)   /* 7:0 MAX_FPP, 9:8 MAXLAT, 31:16 TX ring frames */
#define INF_TX_PACKETS   (INF_BASE + 0x40)
#define INF_TX_OVERRUNS  (INF_BASE + 0x44)
#define INF_RX_OK        (INF_BASE + 0x48)
#define INF_RX_BAD       (INF_BASE + 0x4C)
#define INF_RX_LATE      (INF_BASE + 0x50)
#define INF_RX_EARLY     (INF_BASE + 0x54)
#define INF_RX_UNDERRUNS (INF_BASE + 0x58)
#define INF_RX_TS_GAPS   (INF_BASE + 0x5C)
/* software model only (tests): pattern checks of the received audio (fpga_model.c); hardware reads 0 */
#define INF_MODEL_RXPAT_OK  (INF_BASE + 0xE0)
#define INF_MODEL_RXPAT_BAD (INF_BASE + 0xE4)
#define INF_MODEL_COLSRC    (INF_BASE + 0xE8)   /* write the column, read {valid 31, port 23:16, tx column 7:0} */
#define INF_ID_VALUE     0x4F494E32u

/* table words (TBL_ADDR) */
#define TBL_TX(f, w)     (0x000 + 16 * (f) + (w))
#define TBL_RX(f, w)     (0x100 + 16 * (f) + (w))
#define TBL_RXST(f, w)   (0x200 + 16 * (f) + (w))
/* TX w0: TTL 31:24, TOS 23:16, channels-1 12:8, fpp 7:2 (0 = automatic), bit1 32-bit, bit0 enable */
#define TXW0(ttl, tos, nch, fpp, b32, en) \
    (((uint32_t)(ttl) << 24) | ((uint32_t)(tos) << 16) | ((uint32_t)((nch) - 1) << 8) | ((uint32_t)(fpp) << 2) | ((b32) ? 2u : 0u) | ((en) ? 1u : 0u))
/* RX w0: bps 18:16, channels 13:8, enable 0 */
#define RXW0(bps, nch, en) (((uint32_t)(bps) << 16) | ((uint32_t)(nch) << 8) | ((en) ? 1u : 0u))
#define MAP_SILENT       0x80    /* TX channel map: silence; RX channel map: ignore */

/* ---- cpu_pktq.v (port block + 0x200) */
#define CQ_BASE          0x200
#define CQ_ID            (CQ_BASE + 0x00)    /* "OCQ1" */
#define CQ_CTRL          (CQ_BASE + 0x04)    /* bit0 TX, bit1 RX, bit2 allmulti, bit3 drop239, bit4 promisc, bit8 FLUSH */
#define CQ_TX_FREE       (CQ_BASE + 0x08)
#define CQ_TX_DATA       (CQ_BASE + 0x0C)
#define CQ_TX_SEND       (CQ_BASE + 0x10)    /* 10:0 length, 15 TS request, 23:16 tag */
#define CQ_TX_STATUS     (CQ_BASE + 0x14)
#define CQ_TX_TS_STATUS  (CQ_BASE + 0x18)    /* bit31 a stamp waits, 7:0 tag */
#define CQ_TX_TS_SEC     (CQ_BASE + 0x1C)
#define CQ_TX_TS_NS      (CQ_BASE + 0x20)    /* reading clears bit31 */
#define CQ_RX_STATUS     (CQ_BASE + 0x30)
#define CQ_RX_LEN        (CQ_BASE + 0x34)    /* bit31 valid, 10:0 length */
#define CQ_RX_TS_SEC     (CQ_BASE + 0x38)
#define CQ_RX_TS_NS      (CQ_BASE + 0x3C)
#define CQ_RX_DATA       (CQ_BASE + 0x40)
#define CQ_RX_POP        (CQ_BASE + 0x44)
#define CQ_TX_FRAMES     (CQ_BASE + 0x50)
#define CQ_TX_REJECTS    (CQ_BASE + 0x54)
#define CQ_RX_FRAMES     (CQ_BASE + 0x58)
#define CQ_RX_OVERFLOWS  (CQ_BASE + 0x5C)
#define CQ_RX_FILTERED   (CQ_BASE + 0x60)
#define CQ_ID_VALUE      0x4F435131u
#define CQ_CTRL_RUN      (1u | 2u | 4u | 8u)  /* TX, RX, all multicast, drop other devices' 239/8 media */

/* ---- fs_phase.v: the desk audio clock against the media NCO (desk-global block; option --dclk-base) */
#define FP_ID            0x00    /* "OFP1" */
#define FP_CTRL          0x04    /* bit0 enable, bit1 clear counters */
#define FP_SNAP          0x08    /* write: copy the last edge's latches into the snapshot below */
#define FP_DESK_FRAMES   0x0C    /* snapshot: desk frame edges counted */
#define FP_MEDIA_TS      0x10    /* snapshot: media_ts at that edge */
#define FP_TICKS         0x14    /* snapshot: clocks from the last media frame strobe to that edge */
#define FP_PERIOD        0x18    /* snapshot: clocks between the last two media frame strobes */
#define FP_DESK_PERIOD   0x1C    /* snapshot: clocks between the last two desk frame edges */
#define FP_ID_VALUE      0x4F465031u

/* ---- CS2000 command engine (PRO1 IO FPGA global registers, docs/68 sections 10.3 / 10.4 item 4) */
#define CS2K_CMD         0x00    /* + base (IO 0xC0): reg << 8 | data */
#define CS2K_ECHO        0x04    /* IO 0xC4: bit31 busy, 15:0 echo of the last command */
#define CS2K_STROBE      0x08    /* IO 0xC8: write 1 to send */

#endif
