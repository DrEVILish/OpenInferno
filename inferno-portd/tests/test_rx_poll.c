/* Replay the real AF_PACKET receive seam with a poll time 1 ms before the clock bracket. */
#define _GNU_SOURCE
#include "test.h"
#include <stdint.h>
#include <sys/socket.h>
#include <netpacket/packet.h>
#include <time.h>
#include <string.h>
static int reads, clocks;
static uint64_t poll_now(void) { return 999000000; }
static int test_clock(clockid_t id, struct timespec *t)
{
    uint64_t n = id == CLOCK_REALTIME ? 10000000000ull : 1000000000ull + (clocks++ ? 100 : 0);
    t->tv_sec = n / 1000000000; t->tv_nsec = n % 1000000000;
    return 0;
}
static ssize_t test_recvmsg(int fd, struct msghdr *mh, int flags)
{
    (void)fd; (void)flags;
    if (reads++) return -1;
    memset(mh->msg_name, 0, sizeof(struct sockaddr_ll));
    memset(mh->msg_iov[0].iov_base, 0xff, 60);
    struct cmsghdr *c = CMSG_FIRSTHDR(mh);
    c->cmsg_level = SOL_SOCKET; c->cmsg_type = SCM_TIMESTAMPNS;
    c->cmsg_len = CMSG_LEN(sizeof(struct timespec));
    struct timespec ts = {9, 999990000};
    memcpy(CMSG_DATA(c), &ts, sizeof(ts));
    mh->msg_controllen = CMSG_SPACE(sizeof(ts));
    return 60;
}
#define recvmsg test_recvmsg
#define clock_gettime test_clock
#ifndef MODEL_SOURCE
#define MODEL_SOURCE "../src/fpga_model.c"
#endif
#include MODEL_SOURCE
#undef clock_gettime
#undef recvmsg
int main(void)
{
    g_now_ns = poll_now;
    model_t m = {0};
    m.c.phy_fd = 42; m.c.phy_packet = 1;
    m.incr = 0x14000000; m.cq_ctrl = 2 | 4;
    phy_poll(&m);
    CHECK(m.rxq_n == 1);
    CHECK(m.rxq[0].ts_sec == 0);
    CHECK(m.rxq[0].ts_ns == 999990050);
    return t_done("rx_poll");
}
