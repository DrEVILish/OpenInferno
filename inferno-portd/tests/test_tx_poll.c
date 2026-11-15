#define _GNU_SOURCE
#include "test.h"
#include <stdint.h>
#include <sys/socket.h>
#include <time.h>
#include <string.h>
#include <errno.h>
static int event, mode, failed;
static uint8_t frame[60];
static uint64_t now(void) { return 1000000000; }
static int clocks(clockid_t id, struct timespec *t) {
 t->tv_sec = id == CLOCK_REALTIME ? 10 : 1; t->tv_nsec = 0; return 0;
}
static ssize_t sender(int fd, const void *p, size_t n, int flags) {
 (void)fd; (void)p; (void)flags;
 if (failed) { errno = failed==2 ? ENOBUFS : EAGAIN; return -1; } return n;
}
static ssize_t receiver(int fd, struct msghdr *mh, int flags) {
 (void)fd;
 if (!(flags & MSG_ERRQUEUE) || (event++ && mode!=4)) { errno=EAGAIN; return -1; }
 memcpy(mh->msg_iov[0].iov_base, frame, 60);
 struct cmsghdr *c=CMSG_FIRSTHDR(mh);
 c->cmsg_level=SOL_SOCKET; c->cmsg_type=SCM_TIMESTAMPING;
 struct timespec ts[3]={{9,999990000},{0},{0}};
 c->cmsg_len=CMSG_LEN(sizeof(ts)); memcpy(CMSG_DATA(c),ts,sizeof(ts));
 c=(struct cmsghdr *)((char *)c+CMSG_SPACE(sizeof(ts)));
 c->cmsg_level=SOL_PACKET; c->cmsg_type=16;
 struct { uint32_t error; uint8_t origin,type,code,pad; uint32_t info,id; } e=
  {ENOMSG,4,0,0,0,0,mode==2 ? 99 : 1};
 if(mode==5) e.origin=2;
 if(mode==6) e.type=1;
 if(mode==7) e.info=1;
 if(mode==8) e.error=EINVAL;
 c->cmsg_len=CMSG_LEN(sizeof(e)); memcpy(CMSG_DATA(c),&e,sizeof(e));
 mh->msg_controllen=CMSG_SPACE(sizeof(ts))+CMSG_SPACE(sizeof(e));
 if(mode==1) mh->msg_flags=MSG_CTRUNC;
 return 60;
}
#define recvmsg receiver
#define send sender
#define clock_gettime clocks
#include "../src/fpga_model.c"
#undef recvmsg
#undef send
#undef clock_gettime
int main(void) {
 g_now_ns=now;
 model_t m={0}; m.c.phy_fd=42; m.c.phy_packet=1; m.incr=0x14000000;
 m.cq_ctrl=1; m.txw_n=15;
 cq_wr(&m,CQ_TX_SEND,60|0x8000|(7<<16));
 CHECK(!m.ts_valid); CHECK(m.tx_id==1);
 phy_send(&m,frame,60,8); /* media and CQ share socket IDs */
 CHECK(m.tx_id==2);
 tx_poll(&m); CHECK(m.ts_valid && m.ts_tag==8); CHECK(m.ts_ns==999990000);
 m.ts_valid=0; event=0; tx_poll(&m); CHECK(!m.ts_valid); /* duplicate */
 failed=1; phy_send(&m,frame,60,9); CHECK(m.tx_id==2 && !m.tx_broken);
 failed=0;
 for(mode=1;mode<=8;mode++) {
  m.tx_pending[0].len=60; m.tx_pending[0].id=1; m.tx_pending[0].sent=now();
  event=0; m.stamp_epoch=0; if(mode==3) { m.last_ns=now(); aoip_wr(&m,m.c.clock_base,AOIP_PTP_INCR,m.incr+1); }
  if(mode==4) continue;
  tx_poll(&m); CHECK(!m.ts_valid);
 }
 m.tx_pending[0].sent=1; event=1; tx_poll(&m); CHECK(!m.tx_pending[0].len);
 mode=4; event=0; m.last_ns=now(); m.media_ts=100;
 m.mode=MODE_INFERNO; m.inf_ctrl=1; m.txf[0].run=1;
 m.txf[0].fpp=8; m.tx_tbl[0]=1;
 hw_t h={0}; h.priv=&m; m_service(&h);
 CHECK(event==256); CHECK(m.st_txmedia>0);
 failed=2; phy_send(&m,frame,60,1); CHECK(m.tx_broken);
 return t_done("tx_poll");
}
