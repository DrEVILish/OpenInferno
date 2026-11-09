/* MMIO boundaries and startup allocation cleanup, with no hardware access. Licence: GPL-3.0-or-later. */
#include "test.h"
#include "../src/hw.h"
#include "../src/util.h"
#include <sys/types.h>
#include <sys/mman.h>

void *__real_calloc(size_t, size_t);
void __real_free(void *);
static int fail_at, calls, live, opens, closes, maps, unmaps;
static uint32_t memory[16];
void *__wrap_calloc(size_t n, size_t sz)
{
    if (++calls == fail_at) return NULL;
    void *p = __real_calloc(n, sz); if (p) live++; return p;
}
void __wrap_free(void *p) { if (p) live--; __real_free(p); }
int __wrap_open(const char *path, int flags, ...) { (void)path; (void)flags; opens++; return 99; }
int __wrap_close(int fd) { CHECK(fd == 99); closes++; return 0; }
void *__wrap_mmap(void *p, size_t n, int prot, int flags, int fd, off_t offset)
{
    (void)p; (void)prot; (void)flags; (void)offset;
    CHECK(fd == 99 && n == sizeof(memory)); maps++; return memory;
}
int __wrap_munmap(void *p, size_t n) { CHECK(p == memory && n == sizeof(memory)); unmaps++; return 0; }
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
