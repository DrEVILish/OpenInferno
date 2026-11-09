/* MMIO boundaries and startup allocation cleanup, with no hardware access. Licence: GPL-3.0-or-later. */
#include "test.h"
#include "../src/hw.h"
#include "../src/util.h"
#include <sys/types.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

static int fail_at, calls, live, opens, closes, maps, unmaps;
static uint32_t memory[16];
void *hw_test_calloc(size_t n, size_t sz)
{
    if (++calls == fail_at) return NULL;
    void *p = calloc(n, sz); if (p) live++; return p;
}
void hw_test_free(void *p) { if (p) live--; free(p); }
int hw_test_open(const char *path, int flags, ...) { (void)path; (void)flags; opens++; return 99; }
int hw_test_close(int fd) { CHECK(fd == 99); closes++; return 0; }
void *hw_test_mmap(void *p, size_t n, int prot, int flags, int fd, off_t offset)
{
    (void)p; (void)prot; (void)flags; (void)offset;
    CHECK(fd == 99 && n == sizeof(memory)); maps++; return memory;
}
int hw_test_munmap(void *p, size_t n) { CHECK(p == memory && n == sizeof(memory)); unmaps++; return 0; }
/* Redirect this backend's calls only; never interpose on the C library. */
#define calloc hw_test_calloc
#define free hw_test_free
#define open hw_test_open
#define close hw_test_close
#define mmap hw_test_mmap
#define munmap hw_test_munmap
#include "../src/hw_devmem.c"
#undef calloc
#undef free
#undef open
#undef close
#undef mmap
#undef munmap

int main(void)
{
    g_log_level = LOG_ERR;
    TEST("failed backend allocations never open hardware and leave no allocated objects");
    for (int i = 1; i <= 2; i++) {
        fail_at = i; calls = 0;
        CHECK(hw_devmem_open(0, sizeof(memory)) == NULL);
        CHECK(live == 0 && opens == 0 && maps == 0);
    }
    TEST("aligned boundary accesses work; wraparound and unaligned offsets do not touch memory");
    fail_at = 0; calls = 0;
    hw_t *h = hw_devmem_open(0, sizeof(memory)); CHECK(h != NULL);
    if (h) {
        hw_wr(h, 60, 123); CHECK(hw_rd(h, 60) == 123);
        const uint32_t bad[] = {61, 64, 1, 0xfffffffcU, 0xffffffffU};
        for (size_t i = 0; i < ARRAY_LEN(bad); i++) {
            hw_wr(h, bad[i], 456); CHECK(hw_rd(h, bad[i]) == 0);
            CHECK(memory[0] == 0 && memory[15] == 123);
        }
        h->ops->close(h);
    }
    CHECK(live == 0 && opens == 1 && closes == 1 && maps == 1 && unmaps == 1);
    return t_done("test_hw");
}
