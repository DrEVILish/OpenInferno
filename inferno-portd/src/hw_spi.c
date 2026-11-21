/* hw_spi.c - Linux spidev register backend for desks whose host CPU reaches the FPGA over SPI
 * (e.g. OpenX32 i.MX25, embedded ARM/Linux boards). Speaks a simple 32-bit register framing:
 *   Write (7 bytes): [0xA5] [ADDR_HI] [ADDR_LO] [D31..24] [D23..16] [D15..8] [D7..0]
 *   Read  (8 bytes): [0x5A] [ADDR_HI] [ADDR_LO] [0x00]    [R31..24] [R23..16] [R15..8] [R7..0]
 *
 * Uses local Linux spidev UAPI struct/ioctl definitions instead of <linux/spi/spidev.h> so
 * musl cross-builds without kernel headers keep working. Optional sysfs CS toggle and flock(2)
 * serialization allow sharing one SPI bus cleanly with another console daemon.
 * Part of OpenProSeries inferno-portd. Licence: GPL-3.0-or-later.
 */
#include "hw.h"
#include "util.h"

#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define SPI_REG_CMD_WRITE    0xA5u
#define SPI_REG_CMD_READ     0x5Au
#define SPI_REG_WINDOW_SIZE  0x10000u

/* Linux spidev UAPI definitions (avoid <linux/spi/spidev.h> for musl sysroot portability) */
#define SPIDEV_MODE_0             0u
#define SPIDEV_IOC_MAGIC          'k'
#define SPIDEV_IOC_MESSAGE_1      _IOW(SPIDEV_IOC_MAGIC, 0, char[32])
#define SPIDEV_IOC_WR_MODE        _IOW(SPIDEV_IOC_MAGIC, 1, uint8_t)
#define SPIDEV_IOC_WR_BITS        _IOW(SPIDEV_IOC_MAGIC, 3, uint8_t)
#define SPIDEV_IOC_WR_MAX_SPEED   _IOW(SPIDEV_IOC_MAGIC, 4, uint32_t)

typedef struct {
    uint64_t tx_buf;
    uint64_t rx_buf;
    uint32_t len;
    uint32_t speed_hz;
    uint16_t delay_usecs;
    uint8_t  bits_per_word;
    uint8_t  cs_change;
    uint8_t  tx_nbits;
    uint8_t  rx_nbits;
    uint8_t  word_delay_usecs;
    uint8_t  pad;
} spidev_xfer_t;

_Static_assert(sizeof(spidev_xfer_t) == 32, "Linux spi_ioc_transfer ABI is 32 bytes");

typedef struct {
    int      spi_fd;
    int      cs_fd;
    int      lock_fd;
    uint32_t speed_hz;
} spi_hw_t;

static int spi_lock(int fd, int op)
{
    if (fd < 0) return 0;
    while (flock(fd, op) < 0) {
        if (errno == EINTR) continue;
        return -1;
    }
    return 0;
}

static void spi_cs(int fd, char level)
{
    if (fd < 0) return;
    ssize_t r;
    do { r = pwrite(fd, &level, 1, 0); } while (r < 0 && errno == EINTR);
}

static int spi_xfer(spi_hw_t *p, const uint8_t *tx, uint8_t *rx, uint32_t len)
{
    spidev_xfer_t tr;
    memset(&tr, 0, sizeof(tr));
    tr.tx_buf        = (uint64_t)(uintptr_t)tx;
    tr.rx_buf        = (uint64_t)(uintptr_t)rx;
    tr.len           = len;
    tr.speed_hz      = p->speed_hz;
    tr.bits_per_word = 8;

    if (spi_lock(p->lock_fd, LOCK_EX) < 0) return -1;
    spi_cs(p->cs_fd, '1');
    int ret;
    do { ret = ioctl(p->spi_fd, SPIDEV_IOC_MESSAGE_1, &tr); } while (ret < 0 && errno == EINTR);
    spi_cs(p->cs_fd, '0');
    (void)spi_lock(p->lock_fd, LOCK_UN);
    return ret;
}

static uint32_t sp_rd(hw_t *h, uint32_t a)
{
    spi_hw_t *p = h->priv;
    if (!hw_offset_valid(a, SPI_REG_WINDOW_SIZE)) return 0;
    uint8_t tx[8] = {
        SPI_REG_CMD_READ,
        (uint8_t)((a >> 8) & 0xFFu),
        (uint8_t)(a & 0xFFu),
        0, 0, 0, 0, 0
    };
    uint8_t rx[8] = {0};
    if (spi_xfer(p, tx, rx, sizeof(tx)) < 0) return 0;
    return ((uint32_t)rx[4] << 24) | ((uint32_t)rx[5] << 16) | ((uint32_t)rx[6] << 8) | rx[7];
}

static void sp_wr(hw_t *h, uint32_t a, uint32_t v)
{
    spi_hw_t *p = h->priv;
    if (!hw_offset_valid(a, SPI_REG_WINDOW_SIZE)) return;
    uint8_t tx[7] = {
        SPI_REG_CMD_WRITE,
        (uint8_t)((a >> 8) & 0xFFu),
        (uint8_t)(a & 0xFFu),
        (uint8_t)((v >> 24) & 0xFFu),
        (uint8_t)((v >> 16) & 0xFFu),
        (uint8_t)((v >> 8) & 0xFFu),
        (uint8_t)(v & 0xFFu)
    };
    uint8_t rx[7] = {0};
    (void)spi_xfer(p, tx, rx, sizeof(tx));
}

static int sp_fd(hw_t *h) { (void)h; return -1; }
static void sp_service(hw_t *h) { (void)h; }

static void sp_close(hw_t *h)
{
    spi_hw_t *p = h->priv;
    if (p) {
        if (p->spi_fd >= 0) close(p->spi_fd);
        if (p->cs_fd >= 0) close(p->cs_fd);
        if (p->lock_fd >= 0) close(p->lock_fd);
        free(p);
    }
    free(h);
}

static const struct hw_ops sp_ops = { sp_rd, sp_wr, sp_fd, sp_service, sp_close };

hw_t *hw_spi_open(const char *spidev_path, uint32_t speed_hz, const char *cs_path, const char *lock_path)
{
    if (!spidev_path || !*spidev_path || speed_hz == 0) {
        LOGE("invalid spidev path or speed");
        return NULL;
    }
    spi_hw_t *p = calloc(1, sizeof(*p));
    hw_t *h = calloc(1, sizeof(*h));
    if (!p || !h) {
        LOGE("cannot allocate spidev backend");
        free(p); free(h);
        return NULL;
    }
    p->spi_fd = -1; p->cs_fd = -1; p->lock_fd = -1;

    p->spi_fd = open(spidev_path, O_RDWR);
    if (p->spi_fd < 0) {
        LOGE("%s: cannot open spidev", spidev_path);
        goto fail;
    }
    uint8_t mode = SPIDEV_MODE_0, bits = 8;
    if (ioctl(p->spi_fd, SPIDEV_IOC_WR_MODE, &mode) < 0
        || ioctl(p->spi_fd, SPIDEV_IOC_WR_BITS, &bits) < 0
        || ioctl(p->spi_fd, SPIDEV_IOC_WR_MAX_SPEED, &speed_hz) < 0) {
        LOGE("%s: spidev configuration ioctl failed", spidev_path);
        goto fail;
    }
    if (cs_path && *cs_path) {
        p->cs_fd = open(cs_path, O_WRONLY);
        if (p->cs_fd < 0) {
            LOGE("%s: cannot open SPI CS control", cs_path);
            goto fail;
        }
    }
    if (lock_path && *lock_path) {
        p->lock_fd = open(lock_path, O_CREAT | O_RDWR, 0666);
        if (p->lock_fd < 0) {
            LOGE("%s: cannot open SPI lock file", lock_path);
            goto fail;
        }
    }
    p->speed_hz = speed_hz;
    h->ops = &sp_ops;
    h->priv = p;
    return h;

fail:
    if (p->lock_fd >= 0) close(p->lock_fd);
    if (p->cs_fd >= 0) close(p->cs_fd);
    if (p->spi_fd >= 0) close(p->spi_fd);
    free(p); free(h);
    return NULL;
}
