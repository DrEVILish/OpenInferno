/* ctl.c - the local control socket: OMC (through the AMU relay, or directly in host tests) sets the port up with
 * one-line text commands and reads its state. Protocol (each command answers lines, then "ok" or "err <why>"):
 *   deskname <NAME>            the device becomes "<NAME>-<n>"
 *   rate 48000|96000           latency 0.5|1|2|5 (ms; or the mode 0..3 as "mode <m>")
 *   channels <1..64>           txname <k> <name>          rxname <k> <name>
 *   sub <k> <txchannel>@<device>   unsub <k>
 *   mcast <id> <ch,ch,...>     mcastdel <id>
 *   ptp off|auto|leader|follower [steer]    clock owner|none
 *   clockfail holdover|leader  the desk clock on source loss (GO LEADER: also "master", "internal"); at once if lost now
 *   status                     key=value lines            quit
 * Part of OpenProSeries inferno-portd. Licence: GPL-3.0-or-later.
 */
#include "ctl.h"
#include "clock.h"
#include "fpga_model.h"
#include "ptp.h"
#include "regs.h"
#include "util.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

int settings_apply_line(port_t *p, const char *line, int at_start);

static void outf(char *out, size_t sz, const char *fmt, ...) __attribute__((format(printf, 3, 4)));
static void outf(char *out, size_t sz, const char *fmt, ...)
{
    size_t l = strlen(out);
    if (l + 1 >= sz) return;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(out + l, sz - l, fmt, ap);
    va_end(ap);
}

static void status(port_t *p, char *o, size_t sz)
{
    char b[16];
    outf(o, sz, "name=%s\nport=%d\nip=%s\nrate=%d\nlatency_ns=%u\nchannels=%d\n", p->name, p->n + 1, ip_str(net_ip(p->net), b), p->fs,
         port_latency_ns(p), p->channels);
    if (p->ptp) {
        ptp_stats_t s;
        ptp_get_stats(p->ptp, &s);
        outf(o, sz, "ptp_state=%s\nptp_offset_ns=%.0f\nptp_delay_ns=%.0f\nptp_samples=%llu\nptp_syncs_rx=%llu\nptp_syncs_tx=%llu\n",
             ptp_state_name(s.state), s.offset_ns, s.delay_ns, (unsigned long long)s.samples, (unsigned long long)s.syncs_rx,
             (unsigned long long)s.syncs_tx);
        outf(o, sz, "ptp_fups_rx=%llu\nptp_dreq_tx=%llu\nptp_dreq_rx=%llu\nptp_dresp_tx=%llu\nptp_dresp_rx=%llu\nptp_servo=%d\n",
             (unsigned long long)s.fups_rx, (unsigned long long)s.dreq_tx, (unsigned long long)s.dreq_rx, (unsigned long long)s.dresp_tx,
             (unsigned long long)s.dresp_rx, s.servo_state);
        outf(o, sz, "ptp_leader=%02x%02x%02x%02x%02x%02x\n", s.leader_uuid[0], s.leader_uuid[1], s.leader_uuid[2], s.leader_uuid[3],
             s.leader_uuid[4], s.leader_uuid[5]);
    }
    if (p->clk) {
        dclock_stats_t c;
        dclock_get_stats(p->clk, &c);
        outf(o, sz, "clock_mode=%d\nclock_locked=%d\nclock_ppb=%.1f\nclock_steps=%llu\nmedia_loads=%llu\nmedia_err=%.2f\n", c.mode,
             c.locked, c.ppb, (unsigned long long)c.steps, (unsigned long long)c.media_loads, c.media_err);
        outf(o, sz, "audio_phase=%.4f\naudio_ppm=%.3f\ncs2k_ratio=0x%08x\ncs2k_ppm=%.3f\ncs2k_writes=%llu\nclock_fail=%s\n", c.audio_phase,
             c.audio_ppm, c.cs2k_ratio, c.cs2k_ppm, (unsigned long long)c.cs2k_writes, c.fail_leader ? "leader" : "holdover");
    }
    int ntx = 0, nrx = 0;
    for (int f = 0; f < NTXF; f++)
        if (p->tx[f].used) {
            ntx++;
            outf(o, sz, "txflow%d=%s %s:%u %dch fpp%d%s\n", f + 1, p->tx[f].multicast ? "mcast" : "ucast", ip_str(p->tx[f].dst_ip, b),
                 p->tx[f].dst_port, p->tx[f].nch, p->tx[f].fpp, p->tx[f].programmed ? "" : " (waiting)");
        }
    for (int f = 0; f < NRXF; f++)
        if (p->rx[f].used) {
            nrx++;
            outf(o, sz, "rxflow%d=%s %s %dch fpp%d state%d pkts%u\n", f + 1, p->rx[f].multicast ? "mcast" : "ucast", p->rx[f].host,
                 p->rx[f].nch, p->rx[f].fpp, p->rx[f].state, p->rx[f].pkts);
        }
    outf(o, sz, "txflows=%d\nrxflows=%d\n", ntx, nrx);
    int subs = 0, rcv = 0;
    for (int k = 0; k < MAX_CH; k++)
        if (p->sub[k].active) {
            subs++;
            rcv += p->sub[k].status == SUB_RX_UNICAST || p->sub[k].status == SUB_RX_MULTICAST;
        }
    outf(o, sz, "subs=%d\nsubs_receiving=%d\n", subs, rcv);
    outf(o, sz, "hw_tx_packets=%u\nhw_rx_ok=%u\nhw_rx_late=%u\nhw_rx_early=%u\nhw_rx_bad=%u\n", port_rd(p->hw, INF_TX_PACKETS),
         port_rd(p->hw, INF_RX_OK), port_rd(p->hw, INF_RX_LATE), port_rd(p->hw, INF_RX_EARLY), port_rd(p->hw, INF_RX_BAD));
    uint32_t pok = port_rd(p->hw, INF_MODEL_RXPAT_OK), pbad = port_rd(p->hw, INF_MODEL_RXPAT_BAD);
    if (pok || pbad) outf(o, sz, "model_pattern_ok=%u\nmodel_pattern_bad=%u\n", pok, pbad);
    net_stats_t ns;
    net_get_stats(p->net, &ns);
    outf(o, sz, "net_rx=%llu\nnet_tx=%llu\nnet_drops=%llu\n", (unsigned long long)ns.rx, (unsigned long long)ns.tx,
         (unsigned long long)ns.drops);
}

int ctl_exec(port_t *p, const char *line, char *out, size_t sz)
{
    out[0] = 0;
    char cmd[32] = "", a[128] = "", b[128] = "";
    int n = sscanf(line, "%31s %127s %127s", cmd, a, b);
    if (n < 1) { outf(out, sz, "err empty\n"); return -1; }
    if (!strcmp(cmd, "status")) { status(p, out, sz); outf(out, sz, "ok\n"); return 0; }
    if (!strcmp(cmd, "colsrc") && n >= 2) {
        /* model only: which transmitter column column k plays */
        int k = atoi(a) - 1, id, col;
        port_wr(p->hw, INF_MODEL_COLSRC, (uint32_t)k);
        uint32_t v = port_rd(p->hw, INF_MODEL_COLSRC);
        (void)id; (void)col;
        if (v & 0x80000000u) outf(out, sz, "colsrc=%u:%u\n", (v >> 16) & 15, (v & 63) + 1);
        else outf(out, sz, "colsrc=none\n");
        outf(out, sz, "ok\n");
        return 0;
    }
    if (!strcmp(cmd, "latency") && n >= 2) {
        double ms = atof(a);
        int m = ms <= 0.5 ? 0 : ms <= 1 ? 1 : ms <= 2 ? 2 : 3;
        if (port_set_latency(p, m)) { outf(out, sz, "err refused by the build\n"); return -1; }
        outf(out, sz, "ok\n");
        return 0;
    }
    if (!strcmp(cmd, "mode") && n >= 2) {
        char *end; long mode = strtol(a, &end, 10);
        if (*end || mode < 0 || mode > 3 || port_set_latency(p, (int)mode)) {
            outf(out, sz, "err refused latency mode\n"); return -1;
        }
        outf(out, sz, "ok\n"); return 0;
    }
    if (!strcmp(cmd, "channels") && n >= 2) {
        int k = atoi(a), lim = p->fs > 50000 ? MAX_CH / 2 : MAX_CH;
        if (k < 1 || k > lim) { outf(out, sz, "err 1..%d\n", lim); return -1; }
        port_unadvertise(p);
        p->channels = k;
        port_apply_hw(p);
        port_advertise(p);
        p->dirty = 1;
        outf(out, sz, "ok\n");
        return 0;
    }
    if (!strcmp(cmd, "rate") && n >= 2) {
        int fs = atoi(a);
        if (fs != 48000 && fs != 96000) { outf(out, sz, "err 48000 or 96000\n"); return -1; }
        if (port_set_rate(p, fs)) { outf(out, sz, "err rate exceeds multicast budget\n"); return -1; }
        outf(out, sz, "ok\n");
        return 0;
    }
    if (!strcmp(cmd, "unsub") && n >= 2) { sub_unsubscribe(p, atoi(a) - 1); outf(out, sz, "ok\n"); return 0; }
    if (!strcmp(cmd, "rxname") && n >= 3) {
        port_rename_rx(p, atoi(a) - 1, line + (strstr(line, b) - line));
        outf(out, sz, "ok\n");
        return 0;
    }
    if (!strcmp(cmd, "mcastdel") && n >= 2) {
        int r = txflow_delete_mcast(p, atoi(a) - 1);
        outf(out, sz, r ? "err no such flow\n" : "ok\n");
        return r;
    }
    if (!strcmp(cmd, "ptp") && n >= 2 && p->ptp) {
        int role = !strcmp(a, "off") ? PTP_ROLE_OFF : !strcmp(a, "leader") ? PTP_ROLE_LEADER : !strcmp(a, "follower") ? PTP_ROLE_FOLLOWER : PTP_ROLE_AUTO;
        ptp_set_role(p->ptp, role, n >= 3 && !strcmp(b, "steer"));
        outf(out, sz, "ok\n");
        return 0;
    }
    if (!strcmp(cmd, "clockfail") && n >= 2 && p->clk) {
        /* as the start option --fail-leader (--fail-master: the old alias), at run time; the words of OMC's ClockFailFromKey */
        int lead = !strcmp(a, "leader") || !strcmp(a, "master") || !strcmp(a, "internal");
        if (!lead && strcmp(a, "holdover")) { outf(out, sz, "err holdover or leader\n"); return -1; }
        dclock_set_fail(p->clk, lead);
        outf(out, sz, "ok\n");
        return 0;
    }
    if (!strcmp(cmd, "quit")) { outf(out, sz, "ok\n"); return 1; }
    if (settings_apply_line(p, line, 0) == 0) { outf(out, sz, "ok\n"); return 0; }
    outf(out, sz, "err unknown command\n");
    return -1;
}

/* ---- the socket */
int ctl_listen(ctl_t *c, const char *path)
{
    memset(c, 0, sizeof(*c));
    for (int i = 0; i < CTL_CLIENTS; i++) c->cl[i].fd = -1;
    c->lfd = socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK, 0);
    if (c->lfd < 0) return -1;
    struct sockaddr_un sa;
    memset(&sa, 0, sizeof(sa));
    sa.sun_family = AF_UNIX;
    snprintf(sa.sun_path, sizeof(sa.sun_path), "%s", path);
    unlink(path);
    if (bind(c->lfd, (struct sockaddr *)&sa, sizeof(sa)) || listen(c->lfd, 4)) { close(c->lfd); c->lfd = -1; return -1; }
    snprintf(c->path, sizeof(c->path), "%s", path);
    return 0;
}

/* 1: fully sent; 0: backpressure; -1: connection failed. */
static int ctl_flush(struct ctl_client *k)
{
    while (k->out_pos < k->out_len) {
        ssize_t n = send(k->fd, k->out + k->out_pos, (size_t)(k->out_len - k->out_pos), MSG_NOSIGNAL);
        if (n < 0 && errno == EINTR) continue;
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return 0;
        if (n <= 0) return -1;
        k->out_pos += (int)n;
    }
    k->out_pos = k->out_len = 0;
    return 1;
}

int ctl_poll(ctl_t *c, port_t *p)
{
    int quit = 0;
    if (c->lfd < 0) return 0;
    int fd = accept(c->lfd, NULL, NULL);
    if (fd >= 0) {
        int ok = 0;
        if (fcntl(fd, F_SETFL, O_NONBLOCK) == 0)
            for (int i = 0; i < CTL_CLIENTS; i++) if (c->cl[i].fd < 0) {
                memset(&c->cl[i], 0, sizeof(c->cl[i])); c->cl[i].fd = fd; ok = 1; break;
            }
        if (!ok) close(fd);
    }
    for (int i = 0; i < CTL_CLIENTS; i++) {
        struct ctl_client *k = &c->cl[i];
        if (k->fd < 0) continue;
        int sent = ctl_flush(k);
        if (sent < 0) { close(k->fd); k->fd = -1; continue; }
        if (!sent) continue;
        if (k->quit) { quit = 1; continue; }
        ssize_t r = read(k->fd, k->buf + k->n, sizeof(k->buf) - 1 - (size_t)k->n);
        if ((r == 0 && !memchr(k->buf, '\n', (size_t)k->n)) || (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) {
            close(k->fd); k->fd = -1; continue;
        }
        if (r > 0) k->n += (int)r;
        k->buf[k->n] = 0;
        char *nl;
        while ((nl = strchr(k->buf, '\n'))) {
            *nl = 0;
            if (nl > k->buf && nl[-1] == '\r') nl[-1] = 0;
            k->quit = ctl_exec(p, k->buf, k->out, sizeof(k->out)) == 1;
            k->out_len = (int)strlen(k->out);
            int used = (int)(nl + 1 - k->buf);
            memmove(k->buf, nl + 1, (size_t)(k->n - used) + 1); k->n -= used;
            sent = ctl_flush(k);
            if (sent < 0) { close(k->fd); k->fd = -1; break; }
            if (!sent) break;
            if (k->quit) { quit = 1; break; }
        }
        if (k->fd >= 0 && k->n >= (int)sizeof(k->buf) - 1) { close(k->fd); k->fd = -1; }
    }
    return quit;
}

void ctl_close(ctl_t *c)
{
    for (int i = 0; i < CTL_CLIENTS; i++) if (c->cl[i].fd >= 0) close(c->cl[i].fd);
    if (c->lfd >= 0) { close(c->lfd); unlink(c->path); }
}
