/* Production servo safety regressions (CLK-02/03/04). GPL-3.0-or-later. */
#include "test.h"
#include "../src/clock.h"
#include "../src/fpga_model.h"
#include "../src/regs.h"
#include "../src/util.h"
#include <math.h>
#include <string.h>
static void scenario(int k, int jitter)
{
    const double xtal[] = {50,50,50,100,-2.8,-2.8,-38.3,50,50};
    servo_t s = {0}; servo_init(&s);
    double off = k == 1 ? 3000000 : 34.0*86400*1e9, f=0, step=0;
    uint64_t t=5000000000ull, start=t, bad=0, good=0;
    double ready=-1, hist[20000]={0}; uint64_t times[20000]={0}; int locks[20000]={0}, n=0;
    int changed=0, waslocked=0, steps=0;
    while(t-start < 500000000000ull) {
        uint64_t delta = jitter ? (n%17 == 0 ? 900000000ull : (n%3==0 ? 125000000ull : 250000000ull)) : 250000000ull;
        double dt=delta*1e-9, sec=(t-start)*1e-9;
        double drift=xtal[k];
        if (k>=5 && sec>=200) {
            if(k==5) drift-=fmin((sec-200)*.07,2.4);
            if(k==7) drift-=5;
            if(k==8) drift-=50;
            if(!changed) { off-= k==5 ? 12000 : k==7 ? 2000 : k==8 ? 500000 : 0; changed=1; }
        }
        off+=(drift*1000+f)*dt; t+=delta;
        double meas=off + (double)((n*173)%401)-200;
        if(s.state==SERVO_LOCKED && fabs(meas)>200000) {
            if(!bad) bad=t;
            if(t-bad>=500000000ull) { servo_init(&s); good=0; }
        } else bad=0;
        double prev=f; waslocked=s.state==SERVO_LOCKED;
        int state=servo_sample(&s,meas,t,&f,&step);
        CHECK(fabs(f-prev)<=(waslocked?1000:5000)*fmin(dt,.25)+1e-6);
        if(state==SERVO_JUMP) { CHECK(!waslocked); off+=step; steps++; }
        if(state==SERVO_LOCKED && fabs(meas)<5000 && delta<=500000000ull) {
            if(!good) good=t;
            if(t-good>=1000000000ull && ready<0) ready=(t-start)*1e-9;
        } else good=0;
        hist[n]=f; times[n]=t; locks[n]=state==SERVO_LOCKED;
        double lo=f,hi=f; int all=locks[n];
        for(int j=n-1;j>=0 && t-times[j]<=1000000000ull;j--) {
            lo=fmin(lo,hist[j]); hi=fmax(hi,hist[j]); all &= locks[j];
        }
        CHECK(hi-lo<=5000.000001); if(all) CHECK(hi-lo<=1000.000001);
        n++;
    }
    printf("case %d jitter %d ready %.3f steps %d final offset %.1f ns\n",k,jitter,ready,steps,off);
    CHECK(ready>0 && ready<60); CHECK(fabs(off)<2000);
}

static uint64_t vt;
static uint64_t now(void) { return vt; }
static void increments(void)
{
    g_now_ns=now;
    model_cfg_t cfg={0}; cfg.phy_fd=-1; cfg.port_base=cfg.clock_base=0x1000;
    cfg.dclk_base=cfg.cs2k_base=0xffffffff;
    hw_t *h=model_open(&cfg);
    dclock_cfg_t c={0}; c.owner=1; c.fs=48000;
    dclock_t *d=dclock_new(h,&c); dclock_set_mode(d,DCLK_FOLLOWER);
    dclock_set_freq(d, -12345.678);
    double sum=0, target=335544320.0*(1-12345.678e-9)*48000*256/1e9;
    int carries=0;
    for(int i=0;i<10000;i++) {
        vt+=1000000; dclock_tick(d,vt);
        double reg=clk_rd(h,AOIP_MEDIA_INC);
        CHECK(reg==floor(target) || reg==ceil(target));
        carries += reg==ceil(target); sum+=reg;
    }
    CHECK(carries>0 && carries<10000);
    CHECK(fabs(sum/10000-target)<.0001);
    printf("MEDIA_INC target %.9f actual average %.9f; dither ceil writes %d/10000\n",target,sum/10000,carries);
    dclock_free(d); h->ops->close(h);
}

int main(void)
{
    servo_t s = {0}; double f, step, prev = 0, off = 3000000;
    servo_init(&s);
    int jumps = 0, locked = 0;
    for (int i = 0; i < 240; i++) {
        off += (50000 + prev) * .25;
        int r = servo_sample(&s, off, 5000000000ull + i * 250000000ull, &f, &step);
        CHECK(fabs(f-prev) <= 1250.000001);
        CHECK(fabs(f) <= 150000);
        if (r == SERVO_JUMP) { CHECK(!locked); off += step; jumps++; }
        locked = r == SERVO_LOCKED;
        prev = f;
    }
    CHECK(locked); CHECK(jumps <= 2); CHECK(fabs(off)<1000);
    increments();
    for(int k=0;k<9;k++) { scenario(k,0); scenario(k,1); }
    return t_done("D4");
}
