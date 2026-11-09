/* hw_devmem.c - the IO FPGA window through /dev/mem (the AMU: MPC8315 local bus CS2, 16-bit GPCM; a 32-bit access
 * is two beats, high half first, which the FPGA's lbus_slave assembles; docs/68 section 10.1).
 * Part of OpenProSeries inferno-portd. Licence: GPL-3.0-or-later.
 */
#include "hw.h"
#include "regs.h"
#include "util.h"

#include <fcntl.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <unistd.h>

typedef struct {
    volatile uint32_t *base;
    uint32_t size;
    int fd;
} devmem_t;

static uint32_t dm_rd(hw_t *h, uint32_t a)
{
    devmem_t *d = h->priv;
    if (a + 4 > d->size) return 0;
    return d->base[a >> 2];
}
static void dm_wr(hw_t *h, uint32_t a, uint32_t v)
{
    devmem_t *d = h->priv;
    if (a + 4 > d->size) return;
    d->base[a >> 2] = v;
    __sync_synchronize();   /* the window is cache-inhibited and guarded; keep the order of the bus beats */
}
static int dm_fd(hw_t *h) { (void)h; return -1; }
static void dm_service(hw_t *h) { (void)h; }
static void dm_close(hw_t *h)
{
    devmem_t *d = h->priv;
    munmap((void *)d->base, d->size);
    close(d->fd);
    free(d);
    free(h);
}
static const struct hw_ops dm_ops = { dm_rd, dm_wr, dm_fd, dm_service, dm_close };

hw_t *hw_devmem_open(uint64_t phys, uint32_t size)
{
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) { LOGE("/dev/mem: cannot open"); return NULL; }
    void *p = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, (off_t)phys);
    if (p == MAP_FAILED) { LOGE("/dev/mem: cannot map 0x%llx", (unsigned long long)phys); close(fd); return NULL; }
    devmem_t *d = calloc(1, sizeof(*d));
    hw_t *h = calloc(1, sizeof(*h));
    d->base = p; d->size = size; d->fd = fd;
    h->ops = &dm_ops; h->priv = d;
    return h;
}

void tbl_write(hw_t *h, uint32_t word_addr, const uint32_t *w, int n)
{
    port_wr(h, INF_TBL_ADDR, word_addr);
    for (int i = 0; i < n; i++) port_wr(h, INF_TBL_DATA, w[i]);
}

uint32_t tbl_read(hw_t *h, uint32_t word_addr)
{
    port_wr(h, INF_TBL_ADDR, word_addr);
    return port_rd(h, INF_TBL_DATA);
}
