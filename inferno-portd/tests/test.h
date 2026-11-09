/* test.h - a tiny check harness for inferno-portd's tests. Licence: GPL-3.0-or-later. */
#ifndef PORTD_TEST_H
#define PORTD_TEST_H
#include <stdio.h>
#include <stdlib.h>

static int t_checks, t_fails;
#define CHECK(c) do { t_checks++; if (!(c)) { t_fails++; printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)
#define CHECKF(c, ...) do { t_checks++; if (!(c)) { t_fails++; printf("  FAIL %s:%d: %s: ", __FILE__, __LINE__, #c); printf(__VA_ARGS__); printf("\n"); } } while (0)
#define TEST(name) printf("- %s\n", name)
static inline int t_done(const char *suite)
{
    printf("%s %s: %d checks, %d failed\n", t_fails ? "FAIL" : "PASS", suite, t_checks, t_fails);
    return t_fails ? 1 : 0;
}
#endif
