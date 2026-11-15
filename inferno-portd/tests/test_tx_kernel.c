/* Privileged host integration: own veth only. Compile/run separately from AMU. */
#define _GNU_SOURCE
#include <sys/socket.h>
#include <time.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
static uint64_t raw_tx, raw_rx, rx_stamps[256];
static ssize_t observed_recv(int fd, struct msghdr *m, int flags) {
 ssize_t n=recvmsg(fd,m,flags);
 if(n>=0) for(struct cmsghdr *c=CMSG_FIRSTHDR(m);c;c=CMSG_NXTHDR(m,c)) {
  if(c->cmsg_level!=SOL_SOCKET) continue;
  if(((flags & MSG_ERRQUEUE) && c->cmsg_type==SCM_TIMESTAMPING) ||
     (!(flags & MSG_ERRQUEUE) && c->cmsg_type==SCM_TIMESTAMPNS)) {
   struct timespec t; memcpy(&t,CMSG_DATA(c),sizeof(t));
   uint64_t v=(uint64_t)t.tv_sec*1000000000+t.tv_nsec;
   if(flags&MSG_ERRQUEUE) raw_tx=v; else { raw_rx=v; rx_stamps[((uint8_t *)m->msg_iov[0].iov_base)[18]]=v; }
  }
 }
 return n;
}
#define recvmsg observed_recv
#ifndef MODEL_SOURCE
#define MODEL_SOURCE "../src/fpga_model.c"
#endif
#include MODEL_SOURCE
#undef recvmsg
static uint64_t rt_now(void) { struct timespec t; clock_gettime(CLOCK_REALTIME,&t); return (uint64_t)t.tv_sec*1000000000+t.tv_nsec; }
int main(int argc,char **argv) {
 if(argc!=3) return 2;
 int fd=model_packet_socket(argv[1]), peer=model_packet_socket(argv[2]);
 if(fd<0||peer<0) return 3;
 model_t a={0},b={0}; a.c.phy_fd=fd;b.c.phy_fd=peer;
 a.c.phy_packet=b.c.phy_packet=1; a.cq_ctrl=1;b.cq_ctrl=2|4;
 a.incr=b.incr=0x14000000;
 uint64_t x=g_now_ns(),rt=rt_now(),y=g_now_ns();
 a.last_ns=b.last_ns=x+(y-x)/2;
 int count=0,rxcount=0; int64_t maxerr=0,rxerr=0;
 for(int i=0;i<3000;i++) {
  raw_rx=0;
  /* Exercise CQ send, competing media sends, and real phy_poll. */
  uint8_t f[60]; memset(f,0xff,sizeof(f));f[6]=2;f[12]=0x88;f[13]=0xb5;put32(f+14,(uint32_t)i);
  for(int media=0;media<16;media++) { f[18]=(uint8_t)media; phy_send(&a,f,60,-1); }
  f[18]=16;
  for(int j=0;j<15;j++) a.txw[j]=get32(f+4*j);
  a.txw_n=15; cq_wr(&a,CQ_TX_SEND,60|0x8000|((i&255)<<16));
  if(a.ts_valid) { fprintf(stderr,"unexpected CQ stamp before errorqueue consumption\n"); return 5; }
  for(int k=0;k<10 && !a.ts_valid;k++) { phy_poll(&a); phy_poll(&b); }
  if(a.ts_valid) {
   if(a.ts_tag!=(uint32_t)(i&255)) return 4;
   int64_t e=(int64_t)((uint64_t)a.ts_sec*1000000000+a.ts_ns)-(int64_t)(raw_tx-rt);
   if(llabs(e)>maxerr) maxerr=llabs(e); count++; a.ts_valid=0;
  }
  while(b.rxq_n) {
   cq_frame_t *q=&b.rxq[b.rxq_head];
   /* The first queued frame corresponds to the first observed RX record. */
   if(raw_rx) {
    int64_t e=(int64_t)((uint64_t)q->ts_sec*1000000000+q->ts_ns)-(int64_t)(rx_stamps[q->d[18]]-rt);
    if(llabs(e)>rxerr) rxerr=llabs(e);rxcount++;raw_rx=0;
   }
   cq_wr(&b,CQ_RX_POP,1);
  }
 }
 printf("actual CQ TX %d/3000 RX batches %d; independent initial clock bracket %llu ns; max TX/RX conversion difference %lld/%lld ns (includes clock pairing uncertainty)\n",count,rxcount,(unsigned long long)(y-x),(long long)maxerr,(long long)rxerr);
 close(fd);close(peer); return count<2900 || maxerr>1000 || rxerr>1000;
}
