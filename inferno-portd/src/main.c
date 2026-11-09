/* main.c - inferno-portd: the control plane of one FPGA Inferno port (OpenProSeries fpga/docs/inferno.md section 9).
 *
 *   inferno-portd --port N [options]          N = 1..6 (port A = 1); the device is "<DESK>-<N>"
 *     --desk NAME           desk name (OMC's console preference "deskname"; default OpenProSeries)
 *     --mac XX:..:XX        the port's MAC (default 02:4f:50:53:00:0N)
 *     --ip A.B.C.D[/BITS][,GW]   static address (default: IPv4 link-local, RFC 3927)
 *     --backend devmem|model     devmem: the IO FPGA window through /dev/mem (the AMU); model: the software FPGA
 *     --mem-base ADDR --mem-size BYTES   the window (devmem; default 0xF2000000, 0x100000)
 *     --port-stride BYTES   port n's block = stride x (n + 1) (top_ecp5: 0x1000; the PRO1 io map: 0x10000)
 *     --clock-port N        the port whose aoip_regs drive the desk clock (default 1 = A)
 *     --dclk-base ADDR      fs_phase.v (default none)       --cs2k-base ADDR   the CS2000 engine (default none)
 *     --owner               this daemon writes the desk clock (exactly one daemon per desk)
 *     --ptp off|auto|leader|follower   (default auto)   --steer: this port's PTP offsets steer the desk clock
 *     --log-sync N          leader Sync interval, log2 s (default -2)
 *     --audio-loop --cs2k --cs2k-r0 0xRATIO --fail-leader   the desk audio clock loop (clock.h; --fail-master = old alias)
 *     --phy IFNAME          model: its PHY is an AF_PACKET socket on IFNAME
 *     --model-id N --model-ppm P --model-desk-ppm P --model-tod NS   model: pattern id, oscillator errors, start time
 *     --ctl PATH            control socket (ctl.c)      --state FILE   persistent state
 *     --rate 48000|96000 --latency MS --channels N --nchan N (TX channels per flow advertised, default 16)  -v / -q
 * Part of OpenProSeries inferno-portd. Licence: GPL-3.0-or-later.
 */
#include "ctl.h"
#include "daemon.h"
#include "fpga_model.h"
#include "util.h"

#include <poll.h>
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

extern int g_adv_nchan;
static volatile sig_atomic_t g_stop;
static void on_sig(int s) { (void)s; g_stop = 1; }

static int bad_number;
static uint32_t num(const char *s)
{
    char *end; errno = 0;
    unsigned long long value = strtoull(s, &end, 0);
    if (errno || end == s || *end || *s == '-' || value > UINT32_MAX) bad_number = 1;
    return (uint32_t)value;
}

/* inferno-portd ctl <socket> <command ...>: one command to a running daemon, its answer to stdout */
static int ctl_client(const char *path, int argc, char **argv)
{
    char line[1024];
    size_t used = 0;
    for (int i = 0; i < argc; i++) {
        size_t len = strlen(argv[i]);
        if (used >= sizeof(line) - 1 || len > sizeof(line) - used - 2) { fprintf(stderr, "control command too long\n"); return 2; }
        memcpy(line + used, argv[i], len); used += len;
        line[used++] = i + 1 < argc ? ' ' : '\n';
    }
    line[used] = 0;
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    struct sockaddr_un sa;
    memset(&sa, 0, sizeof(sa));
    sa.sun_family = AF_UNIX;
    if (strlen(path) >= sizeof(sa.sun_path)) { if (fd >= 0) close(fd); return 2; }
    snprintf(sa.sun_path, sizeof(sa.sun_path), "%s", path);
    if (fd < 0 || connect(fd, (struct sockaddr *)&sa, sizeof(sa))) {
        fprintf(stderr, "cannot connect to %s\n", path); if (fd >= 0) close(fd); return 1;
    }
    for (size_t sent = 0; sent < used;) {
        ssize_t w = send(fd, line + sent, used - sent, MSG_NOSIGNAL);
        if (w < 0 && errno == EINTR) continue;
        if (w <= 0) { close(fd); return 1; }
        sent += (size_t)w;
    }
    char buf[16384] = "";
    size_t n = 0;
    int complete = 0, failed = 0;
    while (n < sizeof(buf) - 1) {
        ssize_t r = read(fd, buf + n, sizeof(buf) - 1 - n);
        if (r < 0 && errno == EINTR) continue;
        if (r <= 0) break;
        n += (size_t)r; buf[n] = 0;
        if (strstr(buf, "err ")) { failed = 1; complete = 1; break; }
        if (n >= 3 && !memcmp(buf + n - 3, "ok\n", 3)) { complete = 1; break; }
    }
    fputs(buf, stdout);
    close(fd);
    return !complete || failed ? 1 : 0;
}

int main(int argc, char **argv)
{
    if (argc >= 4 && !strcmp(argv[1], "ctl")) return ctl_client(argv[2], argc - 3, argv + 3);
    static port_t P;
    port_t *p = &P;
    port_defaults(p);
    portd_opts_t o = { 0, 0, 0, 0, 0, 1, 0, -2, 0 };
    int portn = 1, clock_port = 1, model = 0, model_id = 1;
    uint64_t mem_base = 0xF2000000ull, model_tod = 0;
    uint32_t mem_size = 0x100000, stride = 0x1000, dclk = 0xFFFFFFFFu, cs2k = 0xFFFFFFFFu;
    double mppm = 0, mdppm = 0;
    const char *phy = NULL, *ctlp = NULL;
    int have_mac = 0;
    double lat = -1;
    for (int i = 1; i < argc; i++) {
        const char *a = argv[i], *v = i + 1 < argc ? argv[i + 1] : NULL;
#define ARG(n) (!strcmp(a, n) && v && ++i)
        if (ARG("--port")) portn = atoi(v);
        else if (ARG("--desk")) snprintf(p->desk, sizeof(p->desk), "%s", v);
        else if (ARG("--mac")) { if (parse_mac(v, p->mac)) { fprintf(stderr, "bad --mac\n"); return 2; } have_mac = 1; }
        else if (ARG("--ip")) {
            char ip[32] = "", gw[32] = "";
            int bits = 16;
            if (sscanf(v, "%31[0-9.]/%d,%31s", ip, &bits, gw) < 2) sscanf(v, "%31[0-9.],%31s", ip, gw);
            if (parse_ip(ip, &p->cfg_ip)) { fprintf(stderr, "bad --ip\n"); return 2; }
            if (bits < 0 || bits > 32) { fprintf(stderr, "bad --ip prefix (0..32)\n"); return 2; }
            p->cfg_mask = bits == 0 ? 0 : 0xFFFFFFFFu << (32 - bits);
            if (gw[0]) parse_ip(gw, &p->cfg_gw);
        } else if (ARG("--backend")) {
            if (strcmp(v,"model") && strcmp(v,"devmem")) { fprintf(stderr,"bad --backend\n"); return 2; }
            model = !strcmp(v,"model");
        }
        else if (ARG("--mem-base")) mem_base = strtoull(v, NULL, 0);
        else if (ARG("--mem-size")) mem_size = num(v);
        else if (ARG("--port-stride")) stride = num(v);
        else if (ARG("--clock-port")) clock_port = atoi(v);
        else if (ARG("--dclk-base")) dclk = num(v);
        else if (ARG("--cs2k-base")) cs2k = num(v);
        else if (!strcmp(a, "--owner")) o.clock_owner = 1;
        else if (ARG("--ptp")) o.ptp_role = !strcmp(v, "off") ? 0 : !strcmp(v, "leader") ? 2 : !strcmp(v, "follower") ? 3 : 1;
        else if (!strcmp(a, "--steer")) o.ptp_steer = 1;
        else if (ARG("--log-sync")) o.log_sync = (int8_t)atoi(v);
        else if (!strcmp(a, "--audio-loop")) o.audio_loop = 1;
        else if (!strcmp(a, "--cs2k")) o.cs2k = 1;
        else if (ARG("--cs2k-r0")) o.cs2k_r0 = num(v);
        else if (!strcmp(a, "--fail-leader") || !strcmp(a, "--fail-master")) o.fail_leader = 1;
        else if (ARG("--phy")) phy = v;
        else if (ARG("--model-id")) model_id = atoi(v);
        else if (ARG("--model-ppm")) mppm = atof(v);
        else if (ARG("--model-desk-ppm")) mdppm = atof(v);
        else if (ARG("--model-tod")) model_tod = strtoull(v, NULL, 0);
        else if (ARG("--ctl")) ctlp = v;
        else if (ARG("--state")) snprintf(p->state_path, sizeof(p->state_path), "%s", v);
        else if (ARG("--rate")) p->fs = atoi(v) == 96000 ? 96000 : 48000;
        else if (ARG("--latency")) lat = atof(v);
        else if (ARG("--channels")) p->channels = atoi(v);
        else if (ARG("--nchan")) g_adv_nchan = atoi(v);
        else if (!strcmp(a, "-v")) g_log_level = LOG_DEBUG;
        else if (!strcmp(a, "-q")) g_log_level = LOG_WARN;
        else { fprintf(stderr, "inferno-portd: unknown option %s (see the head of main.c)\n", a); return 2; }
#undef ARG
    }
    if (bad_number) { fprintf(stderr, "invalid unsigned register value\n"); return 2; }
    if (portn < 1 || portn > 6) { fprintf(stderr, "--port 1..6\n"); return 2; }
    p->n = portn - 1;
    if (!have_mac) p->mac[5] = (uint8_t)portn;
    if (lat >= 0) p->lat_mode = lat <= 0.5 ? 0 : lat <= 1 ? 1 : lat <= 2 ? 2 : 3;
    if (p->channels < 1 || p->channels > MAX_CH) p->channels = MAX_CH;
    if (p->fs > 50000 && p->channels > MAX_CH / 2) p->channels = MAX_CH / 2;
    rnd_seed(mono_ns() ^ ((uint64_t)p->mac[5] << 40) ^ ((uint64_t)getpid() << 20));

    hw_t *hw;
    uint64_t port_offset = (uint64_t)stride * (unsigned)portn;
    uint64_t clock_offset = (uint64_t)stride * (unsigned)clock_port;
    if (!model && (clock_port < 1 || clock_port > 6 || stride < 0x400 || (stride & 3u)
        || port_offset > UINT32_MAX - 0x400u || clock_offset > UINT32_MAX - 0x400u
        || mem_size < 0x400 || (mem_size & 3u) || port_offset + 0x400 > mem_size || clock_offset + 0x400 > mem_size
        || (dclk != UINT32_MAX && (!hw_offset_valid(dclk, mem_size) || (uint64_t)dclk + 0x100 > mem_size))
        || (cs2k != UINT32_MAX && (!hw_offset_valid(cs2k, mem_size) || (uint64_t)cs2k + 0x100 > mem_size)))) {
        fprintf(stderr, "invalid hardware window or block offsets\n"); return 2;
    }
    uint32_t pbase = (uint32_t)port_offset, cbase = (uint32_t)clock_offset;
    if (model) {
        model_cfg_t mc;
        memset(&mc, 0, sizeof(mc));
        mc.id = model_id; mc.xtal_ppm = mppm; mc.desk_ppm = mdppm; mc.maxlat = 3;
        mc.port_base = pbase; mc.clock_base = cbase; mc.dclk_base = dclk; mc.cs2k_base = cs2k;
        mc.phy_fd = phy ? model_packet_socket(phy) : -1;
        mc.phy_packet = 1;
        mc.cs2k_r0 = o.cs2k_r0 ? o.cs2k_r0 : 0x00200000u;
        mc.tod_start_ns = model_tod;
        if (phy && mc.phy_fd < 0) return 1;
        hw = model_open(&mc);
    } else {
        hw = hw_devmem_open(mem_base, mem_size);
        if (!hw) return 1;
        hw->port_base = pbase; hw->clock_base = cbase; hw->dclk_base = dclk; hw->cs2k_base = cs2k;
    }
    if (portd_start(p, hw, &o)) return 1;
    ctl_t ctl;
    ctl.lfd = -1;
    if (ctlp && ctl_listen(&ctl, ctlp)) { LOGE("cannot listen on %s", ctlp); return 1; }
    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);
    signal(SIGPIPE, SIG_IGN);
    while (!g_stop) {
        struct pollfd pf[2];
        int n = 0;
        if (hw->ops->pollfd(hw) >= 0) { pf[n].fd = hw->ops->pollfd(hw); pf[n].events = POLLIN; n++; }
        if (ctl.lfd >= 0) { pf[n].fd = ctl.lfd; pf[n].events = POLLIN; n++; }
        poll(pf, (nfds_t)n, 1);   /* the CPU queue is polled every 1 ms (no FPGA -> AMU interrupt is known) */
        port_tick(p, g_now());
        if (ctl.lfd >= 0 && ctl_poll(&ctl, p)) g_stop = 1;
    }
    LOGI("stopping");
    portd_stop(p);
    if (ctl.lfd >= 0) ctl_close(&ctl);
    hw->ops->close(hw);
    return 0;
}
