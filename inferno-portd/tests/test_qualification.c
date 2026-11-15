/* Full production PTP/model qualification, CLK-02/03/04. GPL-3.0-or-later.
 * Reuse the packet/subscription fixtures; readiness is exclusively portd's gate. */
#define main inproc_fixture_main
#include "test_inproc.c"
#undef main

static uint64_t noise_rng;
static double uniform(void)
{
    noise_rng ^= noise_rng >> 12;
    noise_rng ^= noise_rng << 25;
    noise_rng ^= noise_rng >> 27;
    return ((double)(uint32_t)((noise_rng * 0x2545F4914F6CDD1Dull) >> 32) + .5) / 4294967296.;
}
static int64_t noise(const uint8_t *p, int n)
{
    if (n < 42 || p[12] != 8 || p[13] != 0 || p[23] != 17 || p[36] != 1 || p[37] != 63) return 0;
    double v = 200 * sqrt(-2 * log(uniform())) * cos(2 * M_PI * uniform());
    if (uniform() < .01) v += uniform() * 5000;
    return llround(v);
}
static hw_t *device(int side, double ppm, uint64_t tod)
{
    model_cfg_t m = {0};
    m.id = side + 1; m.xtal_ppm = ppm; m.maxlat = 3;
    m.port_base = m.clock_base = 0x1000;
    m.dclk_base = m.cs2k_base = 0xffffffff;
    m.phy_fd = -1; m.wire = W; m.wire_side = side; m.wire_delay_ns = 5000;
    m.tod_start_ns = tod; m.rx_noise_ns = side ? noise : NULL;
    return model_open(&m);
}
static int fault_case;
static uint64_t fault_start;
static int sync_frame(const uint8_t *p, int n)
{
    return n>=75 && p[12]==8 && p[13]==0 && p[23]==17 &&
           p[36]==1 && p[37]==63 && p[74]==PTP1_SYNC;
}
static int lose_sync(const uint8_t *p, int n)
{
    if(!sync_frame(p,n)) return 0;
    uint64_t ms=(vt-fault_start)/1000000;
    if(fault_case==10) return ms>=45000 && ms<47000;
    /* Irregular missing observations during acquisition and recovery. */
    return fault_case==11 && ((ms/250)%13==3 || (ms/250)%17==4);
}
static uint64_t delay_sync(const uint8_t *p, int n)
{
    if(!sync_frame(p,n)) return 0;
    uint64_t ms=(vt-fault_start)/1000000;
    if(fault_case==9 && ms>=45000 && ms<47000) return 600000000ull;
    if(fault_case==11) return (ms/250%3)*200ull;
    return 0;
}
static void qualify(int k)
{
    const double xo[] = {50,50,0,50,-8.5,-8.5,-44,50,50,0,0,0,0};
    const double leader[] = {0,0,-50,-50,-5.7,-5.7,-5.7,0,0,0,0,0,0};
    const int duration[] = {240,240,300,300,240,400,240,400,500,100,100,100,100};
    if(A.hw) { portd_stop(&A); A.hw->ops->close(A.hw); }
    if(B.hw) { portd_stop(&B); B.hw->ops->close(B.hw); }
    W = model_wire_new(); port_defaults(&A); port_defaults(&B);
    A.n=0; B.n=1; A.mac[5]=0x11; B.mac[5]=0x22;
    snprintf(A.desk,sizeof(A.desk),"FOH"); snprintf(B.desk,sizeof(B.desk),"MON");
    A.lat_mode=B.lat_mode=3;
    portd_opts_t oa={1,0,0,0,0,PTP_ROLE_LEADER,1,-2,0,0,0};
    portd_opts_t ob={1,0,0,0,0,PTP_ROLE_FOLLOWER,1,-2,0,0,0};
    noise_rng=11+k; fault_case=k; fault_start=vt;
    model_wire_drop(W,lose_sync); model_wire_delay(W,delay_sync);
    CHECK(portd_start(&A,device(0,leader[k],TOD_L),&oa)==0);
    CHECK(portd_start(&B,device(1,xo[k],TOD_L+(k==1 ? 3000000ull : 34ull*86400*1000000000)),&ob)==0);
    for(int c=0;c<8;c++) {
        char ch[8]; snprintf(ch,sizeof(ch),"%02d",c+1);
        sub_subscribe(&B,c,ch,"FOH-1"); sub_subscribe(&A,c,ch,"MON-2");
    }
    double first=-1, prev=0, maxrate=0, sumreg=0, sumtarget=0, lastavg=0, lastcommandavg=0;
    uint64_t start=vt, change_t=vt, steps=0, leak=0;
    int oldlocked=0, played=0, muted=0, resumed=0;
    double history[1001]={0}; int lockhistory[1001]={0};
    double errors[1000]={0}, rollerror=0;
    double lastchange[4]={0}; uint64_t changetime[4]={0}; int changes=0;
    model_stats_t before={0}, after;
    for(int i=0;i<duration[k]*1000;i++) {
        double sec=i*.001;
        if(k==12 && i==45000) {
            ptp_stats_t before; ptp_get_stats(B.ptp,&before);
            CHECK(before.leader_uuid[5]==0x11);
            hw_t *h=A.hw; portd_stop(&A); A.mac[5]=0x33;
            CHECK(portd_start(&A,h,&oa)==0);
        }
        if(i==200000 && (k==5 || k>=7)) dclock_step(A.clk,k==5 ? 12000 : k==7 ? 2000 : 500000);
        if(k==5 && i>=200000) dclock_set_freq(A.clk, fmin((sec-200)*.07,2.4)*1000);
        if(k>=7 && i==200000) dclock_set_freq(A.clk,k==7 ? 5000 : 50000);
        int on=enables(&B);
        model_get_stats(B.hw,&before);
        run(.001);
        model_get_stats(B.hw,&after);
        dclock_stats_t s; dclock_get_stats(B.clk,&s);
        if(s.ppb!=prev) {
            double dt=(vt-change_t)*1e-9, rate=fabs(s.ppb-prev)/fmin(dt,.25);
            CHECKF(rate <= (oldlocked ? 1000 : 5000)+1e-4,"case %d t %.3f rate %.6f locked %d",k,sec,rate,oldlocked);
            maxrate=fmax(maxrate,rate); change_t=vt; prev=s.ppb;
            changetime[changes%4]=vt-start; lastchange[changes%4]=s.ppb; changes++;
        }
        CHECK(fabs(s.ppb)<=150000.000001);
        int boundary = history[i%1001]!=s.ppb || lockhistory[i%1001]!=s.locked ||
                       history[(i+1)%1001]!=history[i%1001] ||
                       lockhistory[(i+1)%1001]!=lockhistory[i%1001];
        history[i%1001]=s.ppb; lockhistory[i%1001]=s.locked;
        /* Commands and lock state change only on these 1ms ticks. Check every
         * change and every left-edge expiry; between them extrema are constant. */
        if(boundary && i>=1000) {
            double lo=s.ppb,hi=s.ppb; int all=1;
            for(int j=0;j<1001;j++) { lo=fmin(lo,history[j]); hi=fmax(hi,history[j]); all &= lockhistory[j]; }
            CHECKF(hi-lo<=5000.00001,"rolling case %d t %.3f span %.9f",k,sec,hi-lo);
            if(all) CHECKF(hi-lo<=1000.00001,"locked rolling case %d t %.3f span %.9f",k,sec,hi-lo);
        }
        if(s.steps!=steps) CHECKF(!enables(&B) && !s.locked,"step case %d t %.3f oldlocked %d newlocked %d gate %d",k,sec,oldlocked,s.locked,enables(&B));
        steps=s.steps; oldlocked=s.locked;
        if(enables(&B) && first<0) first=(vt-start)*1e-9;
        if(!on && !enables(&B)) leak += after.rx_media-before.rx_media + after.tx_media-before.tx_media;
        if(on && after.pat_ok>before.pat_ok && after.tx_media>before.tx_media) {
            played=1; if(muted) resumed=1;
        }
        if(first>0 && !enables(&B)) muted=1;
        double reg=clk_rd(B.hw,AOIP_MEDIA_INC);
        double target=335544320.*(1+s.ppb*1e-9)*48000*256/1e9;
        rollerror+=(reg-target)-errors[i%1000];
        errors[i%1000]=reg-target;
        /* Every rolling 1s quantized mean tracks the integrated command to
         * <1 count in its sum, hence <1e9/(1000*nominal) ppb in its mean. */
        if(i>=999) CHECKF(fabs(rollerror)<1.00001,"quant case %d t %.3f residual %.9f",k,sec,rollerror);
        sumreg+=reg; sumtarget+=target;
        if(i%250==249) {
            /* Sigma-delta error is bounded by one register count per window.
             * Compare moving average to the actual integrated command, including transitions. */
            CHECK(fabs(sumreg-sumtarget)<1.00001);
            double commandavg=(sumtarget/250/(335544320.*48000*256/1e9)-1)*1e9;
            double avg=(sumreg/250/(335544320.*48000*256/1e9)-1)*1e9;
            double bound=1e9/(250*(335544320.*48000*256/1e9));
            CHECK(fabs(avg-commandavg)<bound+1e-6);
            if(i>=499 && fabs(commandavg-lastcommandavg)>1250.01) {
                printf("250ms diagnostic case %d t %.3f actual delta %.9f command delta %.9f residual bound %.9f\n",
                       k,sec,avg-lastavg,commandavg-lastcommandavg,2*bound);
                for(int j=0;j<4;j++) printf("  staircase t %.9f ppb %.9f\n",changetime[j]*1e-9,lastchange[j]);
            }
            lastavg=avg; lastcommandavg=commandavg; sumreg=sumtarget=0;
        }
    }
    printf("production case %d first-ready %.3f max ppb/s %.6f steps %llu played %d mute/resume %d/%d leak %llu\n",
           k,first,maxrate,(unsigned long long)steps,played,muted,resumed,(unsigned long long)leak);
    CHECK(first>0 && first<60); CHECK(played); CHECK(leak==0);
    if(k==8 || k==9 || k==10 || k==12) CHECK(muted && resumed);
    if(k==12) { ptp_stats_t end; ptp_get_stats(B.ptp,&end); CHECK(end.leader_uuid[5]==0x33); }
    CHECK(enables(&B)==3);
}

/* Independent edge checks use the production servo API, never force ready. */
static void boundaries(void)
{
    for(int delta=-1;delta<=1;delta++) {
        servo_t s={0}; double f=0,step=0; servo_init(&s);
        int r=servo_sample(&s,100000+delta,1000000000ull,&f,&step);
        CHECK((r==SERVO_JUMP)==(delta>0));
        servo_init(&s); f=step=0;
        for(int j=0;j<=8;j++) r=servo_sample(&s,20000+delta,1000000000ull+j*250000000ull,&f,&step);
        CHECK((r==SERVO_JUMP)==(delta>0));
    }
    servo_t s={0}; double f=0,step=0; servo_init(&s);
    uint64_t t=1000000000ull;
    for(int j=0;j<=8;j++,t+=250000000ull) servo_sample(&s,0,t,&f,&step);
    /* Acquisition landing does not count as the first PI lock sample. */
    for(int j=0;j<8;j++,t+=100000000ull) {
        servo_sample(&s,0,t,&f,&step); CHECK(s.state!=SERVO_LOCKED);
    }
    servo_sample(&s,1000,t,&f,&step); CHECK(s.lock_good==0); t+=250000000ull;
    for(int j=0;j<7;j++,t+=250000000ull) {
        servo_sample(&s,999,t,&f,&step); CHECK(s.state!=SERVO_LOCKED);
    }
    servo_sample(&s,999,t,&f,&step); CHECK(s.state==SERVO_LOCKED);
    servo_init(&s); t+=1000000000ull;
    for(int j=0;j<=8;j++,t+=250000000ull) servo_sample(&s,0,t,&f,&step);
    for(int j=0;j<7;j++,t+=250000000ull) servo_sample(&s,0,t,&f,&step);
    t+=600000000ull; servo_sample(&s,0,t,&f,&step);
    CHECK(s.state!=SERVO_LOCKED && s.lock_good==1);
}
static void saturation(void)
{
    for(int sign=-1;sign<=1;sign+=2) {
        servo_t s={0}; servo_init(&s);
        s.count=1; s.acq=0; s.t0=1000000000ull;
        s.last_ppb=s.integ=sign*130000;
        double f=s.last_ppb, step;
        uint64_t t=s.t0;
        for(int j=0;j<80;j++) {
            t+=250000000ull;
            double prev=f;
            servo_sample(&s,-sign*80000,t,&f,&step);
            CHECK(fabs(f-prev)<=1250.000001);
            CHECK(fabs(s.integ)<=130000 && fabs(f)<=150000);
        }
        CHECK(fabs(f-sign*150000)<1e-6);
        /* With estimate far ahead of output, error driving it farther freezes;
         * reversed error unwinds immediately instead of staying saturated. */
        s.integ=sign*130000; s.last_ppb=sign*100000;
        memset(s.expires,0,sizeof(s.expires));
        t+=250000000ull; servo_sample(&s,-sign*80000,t,&f,&step);
        CHECK(s.integ==sign*130000);
        t+=250000000ull; servo_sample(&s,sign*80000,t,&f,&step);
        CHECK(fabs(s.integ)<130000);
        double keep=s.last_ppb;
        servo_init(&s); CHECK(s.last_ppb==keep);
        t+=250000000ull; servo_sample(&s,0,t,&f,&step);
        CHECK(f==keep && s.state!=SERVO_LOCKED);
        for(int j=0;j<8;j++) {
            t+=250000000ull; double prev=f;
            servo_sample(&s,0,t,&f,&step);
            CHECK(fabs(f-prev)<=1250.000001);
        }
    }
}
/* Both leaders keep sending: replacing a qualified parent is not source loss. */
static void overlapping_parent(void)
{
    if(A.hw) { portd_stop(&A); A.hw->ops->close(A.hw); }
    if(B.hw) { portd_stop(&B); B.hw->ops->close(B.hw); }
    W=model_wire_new(); port_defaults(&A); port_defaults(&B);
    A.n=0; B.n=1; A.mac[5]=0x11; B.mac[5]=0x22;
    snprintf(A.desk,sizeof(A.desk),"FOH");
    snprintf(B.desk,sizeof(B.desk),"MON");
    A.lat_mode=B.lat_mode=3;
    portd_opts_t oa={1,0,0,0,0,PTP_ROLE_LEADER,1,-2,0,0,0};
    portd_opts_t ob={1,0,0,0,0,PTP_ROLE_FOLLOWER,1,-2,0,0,0};
    CHECK(portd_start(&A,mk(0,1,0,TOD_L),&oa)==0);
    CHECK(portd_start(&B,mk(1,2,0,TOD_L),&ob)==0);
    sub_subscribe(&B,0,"01","FOH-1"); run(30);
    CHECK(ptp_clock_ready(B.ptp) && enables(&B)==3);
    ptp_stats_t old, s; ptp_get_stats(B.ptp,&old);
    model_stats_t before, after; model_get_stats(B.hw,&before);
    CHECK(before.pat_ok>0);
    ptp_stats_t a_before; ptp_get_stats(A.ptp,&a_before);
    port_t C; port_defaults(&C); C.n=2; C.mac[5]=0x05;
    snprintf(C.desk,sizeof(C.desk),"NEW");
    portd_opts_t oc=oa; oc.stratum=1;
    uint32_t sec,ns; dclock_now(A.clk,&sec,&ns);
    CHECK(portd_start(&C,mk(0,3,0,(uint64_t)sec*1000000000ull+ns),&oc)==0);
    uint64_t selected=0, first_locked=0, reopened=0, leak=0;
    for(uint64_t end=vt+30000000000ull; vt<end;) {
        int off=selected && !enables(&B);
        model_get_stats(B.hw,&before);
        vt+=1000000ull;
        /* C services shared upstream requests; A remains a live transmitting leader. */
        port_tick(&C,vt); port_tick(&A,vt); port_tick(&B,vt);
        ptp_get_stats(B.ptp,&s); model_get_stats(B.hw,&after);
        if(!selected && memcmp(s.leader_uuid,old.leader_uuid,6)) {
            selected=vt;
            CHECK(s.state==PTP_SLAVE && ptp_state(A.ptp)==PTP_MASTER);
            ptp_stats_t live; ptp_get_stats(A.ptp,&live);
            CHECK(live.syncs_tx>a_before.syncs_tx);
            CHECK(!ptp_clock_ready(B.ptp) && enables(&B)==0);
            dclock_stats_t d; dclock_get_stats(B.clk,&d);
            CHECK(!d.locked && s.servo_state!=SERVO_LOCKED);
        }
        if(selected && s.servo_state==SERVO_LOCKED && !first_locked) first_locked=vt;
        if(selected && enables(&B)) { reopened=vt; break; }
        if(off && !enables(&B)) leak+=after.rx_media-before.rx_media+after.tx_media-before.tx_media;
    }
    CHECK(selected && first_locked && reopened);
    CHECK(first_locked>=selected+1750000000ull);
    CHECK(reopened>=first_locked+PTP_READY_HOLD_NS);
    CHECK(leak==0);
    portd_stop(&C); C.hw->ops->close(C.hw);
}
int main(void)
{
    g_now_ns=vnow; g_log_level=LOG_WARN; rnd_seed(7);
    boundaries(); saturation(); overlapping_parent();
    for(int k=0;k<13;k++) qualify(k);
    return t_done("production qualification");
}
