/* spidev backend framing, boundaries, CS/flock sequencing, and cleanup without hardware. Licence: GPL-3.0-or-later. */
#include "test.h"
#include "../src/hw.h"
#include "../src/util.h"
#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <string.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <unistd.h>

static int fail_at, calls, live, opens, closes, ioctls, locks, unlocks, cs_hi, cs_lo, locked, cs_active;
static uint32_t regs[16384];
static uint8_t last_tx[8];
static uint32_t last_len;

static void *spi_test_calloc(size_t n, size_t sz)
{
    if (++calls == fail_at) return NULL;
    void *p = calloc(n, sz);
    if (p) live++;
    return p;
}
static void spi_test_free(void *p) { if (p) live--; free(p); }
static int spi_test_open(const char *path, int flags, ...)
{
    (void)flags;
    opens++;
    if (!strcmp(path, "/dev/spidev0.0")) return 90;
    if (!strcmp(path, "/sys/cs")) return 91;
    if (!strcmp(path, "/run/lock")) return 92;
    return -1;
}
static int spi_test_close(int fd) { CHECK(fd >= 90 && fd <= 92); closes++; return 0; }
static int spi_test_flock(int fd, int op)
{
    CHECK(fd == 92);
    if (op == LOCK_EX) { CHECK(!locked && !cs_active); locked = 1; locks++; }
    else if (op == LOCK_UN) { CHECK(locked && !cs_active); locked = 0; unlocks++; }
    return 0;
}
static ssize_t spi_test_pwrite(int fd, const void *buf, size_t count, off_t off)
{
    CHECK(fd == 91 && count == 1 && off == 0 && locked);
    char c = *(const char *)buf;
    if (c == '1') { CHECK(!cs_active); cs_active = 1; cs_hi++; }
    else if (c == '0') { CHECK(cs_active); cs_active = 0; cs_lo++; }
    else CHECK(0);
    return 1;
}
static int spi_test_ioctl(int fd, unsigned long req, ...)
{
    va_list ap;
    va_start(ap, req);
    void *arg = va_arg(ap, void *);
    va_end(ap);
    CHECK(fd == 90);
    ioctls++;
    if (req == _IOW('k', 0, char[32])) {
        const uint64_t *u64 = (const uint64_t *)arg;
        const uint8_t *tx = (const uint8_t *)(uintptr_t)u64[0];
        uint8_t *rx = (uint8_t *)(uintptr_t)u64[1];
        uint32_t len = *(const uint32_t *)((const uint8_t *)arg + 16);
        last_len = len;
        memcpy(last_tx, tx, len <= 8 ? len : 8);
        if (len == 7 && tx[0] == 0xA5u) {
            uint16_t addr = ((uint16_t)tx[1] << 8) | tx[2];
            uint32_t val = ((uint32_t)tx[3] << 24) | ((uint32_t)tx[4] << 16) | ((uint32_t)tx[5] << 8) | tx[6];
            regs[addr >> 2] = val;
            return 7;
        }
        if (len == 8 && tx[0] == 0x5Au) {
            uint16_t addr = ((uint16_t)tx[1] << 8) | tx[2];
            uint32_t val = regs[addr >> 2];
            rx[4] = (uint8_t)(val >> 24);
            rx[5] = (uint8_t)(val >> 16);
            rx[6] = (uint8_t)(val >> 8);
            rx[7] = (uint8_t)val;
            return 8;
        }
        return -1;
    }
    return 0;
}

#define calloc spi_test_calloc
#define free   spi_test_free
#define open   spi_test_open
#define close  spi_test_close
#define flock  spi_test_flock
#define pwrite spi_test_pwrite
#define ioctl  spi_test_ioctl
#include "../src/hw_spi.c"
#undef calloc
#undef free
#undef open
#undef close
#undef flock
#undef pwrite
#undef ioctl

int main(void)
{
    g_log_level = LOG_ERR;

    TEST("failed spidev allocations never open files or leak memory");
    for (int i = 1; i <= 2; i++) {
        fail_at = i; calls = 0;
        CHECK(hw_spi_open("/dev/spidev0.0", 8000000u, "/sys/cs", "/run/lock") == NULL);
        CHECK(live == 0 && opens == 0);
    }
    fail_at = 0; calls = 0;

    TEST("failed CS or lock open cleans up already-opened descriptors");
    CHECK(hw_spi_open("/dev/spidev0.0", 8000000u, "/bad/cs", NULL) == NULL);
    CHECK(live == 0 && opens == 2 && closes == 1);
    opens = closes = ioctls = 0;

    TEST("7-byte write and 8-byte read framing with CS and flock sequencing");
    hw_t *h = hw_spi_open("/dev/spidev0.0", 8000000u, "/sys/cs", "/run/lock");
    CHECK(h != NULL);
    if (h) {
        hw_wr(h, 0x1004u, 0xDEADBEEFu);
        CHECK(last_len == 7 && last_tx[0] == 0xA5u && last_tx[1] == 0x10u && last_tx[2] == 0x04u);
        CHECK(hw_rd(h, 0x1004u) == 0xDEADBEEFu);
        CHECK(last_len == 8 && last_tx[0] == 0x5Au && last_tx[1] == 0x10u && last_tx[2] == 0x04u);
        CHECK(locks == 2 && unlocks == 2 && cs_hi == 2 && cs_lo == 2 && !locked && !cs_active);

        /* Boundary & alignment rejection */
        int prev_ioctls = ioctls;
        const uint32_t bad[] = {1u, 0x1001u, 0x10000u, 0xFFFFFFFCu};
        for (size_t i = 0; i < ARRAY_LEN(bad); i++) {
            hw_wr(h, bad[i], 0x12345678u);
            CHECK(hw_rd(h, bad[i]) == 0);
        }
        CHECK(ioctls == prev_ioctls);

        h->ops->close(h);
    }
    CHECK(live == 0 && opens == 3 && closes == 3);
    return t_done("test_hw_spi");
}
