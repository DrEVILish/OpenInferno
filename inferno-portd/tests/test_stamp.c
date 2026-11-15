/* Timestamp conversion boundary tests; no kernel or privileged sockets required. */
#define _GNU_SOURCE
#include "test.h"
#include <time.h>
#include <stdint.h>
static uint64_t mono = 1000000000, real = 10000000000ull;
static int stamp_clock(clockid_t id, struct timespec *ts)
{
    uint64_t n = id == CLOCK_REALTIME ? real : mono;
    if (id == CLOCK_MONOTONIC) mono += 100;
    ts->tv_sec = n / 1000000000; ts->tv_nsec = n % 1000000000;
    return 0;
}
#define clock_gettime stamp_clock
#include "../src/fpga_model.c"
#undef clock_gettime
int main(void)
{
    uint64_t at = 0;
    CHECK(stamp_convert(9999990000ull, 1000000000, 10000000000ull, 1000000100, 0, &at));
    CHECK(at == 999990050); /* cached poll time is deliberately absent */
    CHECK(!stamp_convert(1, 10, 20, 9, 0, &at));
    CHECK(!stamp_convert(1, 10, 20, 1011, 0, &at));
    CHECK(!stamp_convert(21, 10, 20, 11, 0, &at));
    CHECK(!stamp_convert(1, 10, 200000000, 11, 0, &at));
    CHECK(!stamp_convert(1, 10, 20, 11, 0, &at));
    CHECK(!stamp_convert(9999990000ull, 1000000000, 10000000000ull, 1000000100, 1000000000, &at));
    model_t m = {0};
    union { struct cmsghdr align; char buf[CMSG_SPACE(sizeof(struct timespec))]; } control;
    memset(&control, 0, sizeof(control));
    struct msghdr mh = {0};
    mh.msg_control = control.buf; mh.msg_controllen = sizeof(control.buf);
    struct cmsghdr *c = CMSG_FIRSTHDR(&mh);
    c->cmsg_level = SOL_SOCKET; c->cmsg_type = SCM_TIMESTAMPNS;
    c->cmsg_len = CMSG_LEN(sizeof(struct timespec));
    struct timespec ts = {9, 999990000};
    memcpy(CMSG_DATA(c), &ts, sizeof(ts));
    CHECK(rx_stamp(&m, &mh, &at));
    CHECK(at == 999990050);
    mh.msg_flags = MSG_CTRUNC; CHECK(!rx_stamp(&m, &mh, &at));
    mh.msg_flags = MSG_TRUNC; CHECK(!rx_stamp(&m, &mh, &at));
    mh.msg_flags = 0;
    c->cmsg_len--; CHECK(!rx_stamp(&m, &mh, &at));
    c->cmsg_len = sizeof(control.buf) + 1; CHECK(!rx_stamp(&m, &mh, &at));
    c->cmsg_len = CMSG_LEN(sizeof(ts));
    c->cmsg_type = 0; CHECK(!rx_stamp(&m, &mh, &at));
    c->cmsg_type = SCM_TIMESTAMPNS;
    ts.tv_nsec = 1000000000; memcpy(CMSG_DATA(c), &ts, sizeof(ts));
    CHECK(!rx_stamp(&m, &mh, &at));
    ts.tv_sec = 10; ts.tv_nsec = 999990000; memcpy(CMSG_DATA(c), &ts, sizeof(ts));
    real += 1000000000; /* queued event around a realtime step is discarded */
    CHECK(!rx_stamp(&m, &mh, &at));
    CHECK(m.stamp_epoch != 0);
    /* Reordered valid arrivals keep their own event time; a missing cmsg never falls back. */
    m = (model_t){0}; mono = 1000000000; real = 10000000000ull;
    ts = (struct timespec){9, 999980000}; memcpy(CMSG_DATA(c), &ts, sizeof(ts));
    CHECK(rx_stamp(&m, &mh, &at)); CHECK(at == 999980050);
    ts.tv_nsec = 999970000; memcpy(CMSG_DATA(c), &ts, sizeof(ts));
    CHECK(rx_stamp(&m, &mh, &at)); CHECK(at == 999970250);
    mh.msg_controllen = 0; CHECK(!rx_stamp(&m, &mh, &at));
    mh.msg_controllen = sizeof(control.buf);
    m.c.phy_packet = 1; m.c.clock_base = 0; m.last_ns = mono;
    aoip_wr(&m, 0, AOIP_PTP_INCR, 0x14000001);
    CHECK(m.stamp_epoch == mono);
    CHECK(!rx_stamp(&m, &mh, &at));
    m.last_ns += 1000;
    aoip_wr(&m, 0, AOIP_CLOCK_CMD, 1);
    CHECK(m.stamp_epoch == m.last_ns);
    CHECK(!rx_stamp(&m, &mh, &at));
    return t_done("stamp");
}
