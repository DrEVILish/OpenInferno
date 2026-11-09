/* hw.h - the register backend: the IO FPGA window through /dev/mem on the AMU, or the software model of the
 * FPGA port (fpga_model.c) for host tests. Addresses are byte offsets in the window.
 * Part of OpenProSeries inferno-portd. Licence: GPL-3.0-or-later.
 */
#ifndef PORTD_HW_H
#define PORTD_HW_H

#include <stddef.h>
#include <stdint.h>

typedef struct hw hw_t;

struct hw_ops {
    uint32_t (*rd)(hw_t *h, uint32_t addr);
    void     (*wr)(hw_t *h, uint32_t addr, uint32_t v);
    int      (*pollfd)(hw_t *h);            /* a descriptor to wait on, or -1 */
    void     (*service)(hw_t *h);           /* let a model run (media, PHY); no-op on hardware */
    void     (*close)(hw_t *h);
};

struct hw {
    const struct hw_ops *ops;
    uint32_t port_base;      /* this port's 4 KB block */
    uint32_t clock_base;     /* the port block whose aoip_regs drive the desk clock (port 0's) */
    uint32_t dclk_base;      /* fs_phase.v block; 0xFFFFFFFF = none */
    uint32_t cs2k_base;      /* CS2000 command engine; 0xFFFFFFFF = none */
    void    *priv;
};

static inline uint32_t hw_rd(hw_t *h, uint32_t a) { return h->ops->rd(h, a); }
static inline void     hw_wr(hw_t *h, uint32_t a, uint32_t v) { h->ops->wr(h, a, v); }
/* this port's registers (regs.h offsets) */
static inline uint32_t port_rd(hw_t *h, uint32_t off) { return h->ops->rd(h, h->port_base + off); }
static inline void     port_wr(hw_t *h, uint32_t off, uint32_t v) { h->ops->wr(h, h->port_base + off, v); }
/* the desk clock's registers (aoip_regs of the clock port) */
static inline uint32_t clk_rd(hw_t *h, uint32_t off) { return h->ops->rd(h, h->clock_base + off); }
static inline void     clk_wr(hw_t *h, uint32_t off, uint32_t v) { h->ops->wr(h, h->clock_base + off, v); }

void tbl_write(hw_t *h, uint32_t word_addr, const uint32_t *w, int n);
uint32_t tbl_read(hw_t *h, uint32_t word_addr);

/* /dev/mem: phys = the window's physical address, size its length */
hw_t *hw_devmem_open(uint64_t phys, uint32_t size);

#endif
