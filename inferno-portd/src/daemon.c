/* daemon.c - brings one port up on a register backend (main.c and the in-process tests use it).
 * Part of OpenProSeries inferno-portd. Licence: GPL-3.0-or-later.
 */
#include "daemon.h"
#include "clock.h"
#include "pktq.h"
#include "ptp.h"
#include "regs.h"
#include "util.h"

#include <stdio.h>

static void on_load(void *ctx) { txflows_reanchor((port_t *)ctx); }

int portd_start(port_t *p, hw_t *hw, const portd_opts_t *o)
{
    p->hw = hw;
    uint32_t id = port_rd(hw, AOIP_ID), iid = port_rd(hw, INF_ID);
    if (id != AOIP_ID_VALUE || iid != INF_ID_VALUE) {
        LOGE("port block: IDs 0x%08x / 0x%08x (want OAP2 / OIN2): not an Inferno-capable port", id, iid);
        return -1;
    }
    uint32_t caps = port_rd(hw, INF_CAPS), caps2 = port_rd(hw, INF_CAPS2);
    p->caps_ch = (int)(caps & 0xFF);
    p->caps_fch = (int)((caps >> 8) & 0xFF);
    p->caps_txf = (int)((caps >> 16) & 0xFF);
    p->caps_rxf = (int)(caps >> 24);
    p->maxlat = (int)((caps2 >> 8) & 3);
    if (p->caps_ch < p->channels) p->channels = p->caps_ch;
    /* the caps are the run-time limits; refuse only what the static tables cannot hold, or a port with no flows */
    if (p->caps_ch < 1 || p->caps_ch > MAX_CH || p->caps_fch < 1 || p->caps_fch > MAXFCH || p->caps_txf > NTXF ||
        p->caps_rxf > NRXF || p->caps_txf + p->caps_rxf == 0) {
        LOGE("incompatible FPGA: %d columns, %d TX / %d RX flows, %d ch per flow (supported: 1..%d columns, 0..%d / 0..%d flows "
             "but not none, 1..%d ch per flow)", p->caps_ch, p->caps_txf, p->caps_rxf, p->caps_fch, MAX_CH, NTXF, NRXF, MAXFCH);
        return -1;
    }
    if (pktq_init(hw)) return -1;
    p->net = net_new(hw, p->mac);
    net_set_ip(p->net, p->cfg_ip, p->cfg_mask, p->cfg_gw);
    p->mdns = mdns_new(p->net);
    port_set_identity(p);
    if (p->state_path[0]) settings_load(p);
    int limit = p->fs > 50000 ? p->caps_ch / 2 : p->caps_ch;   /* 96 kHz plays half the build's columns */
    if (p->channels > limit) p->channels = limit;
    port_set_identity(p);
    snprintf(g_log_tag, sizeof(g_log_tag), "%s", p->name);
    arc_init(p);
    cmc_init(p);
    flowctl_init(p);
    sub_init(p);
    if (p->state_path[0]) {   /* subscriptions and bundles of the state file (they need the stack) */
        FILE *f = fopen(p->state_path, "r");
        char line[256];
        int settings_apply_line(port_t *, const char *, int);
        if (f) {
            while (fgets(line, sizeof(line), f))
                if (!strncmp(line, "sub ", 4) || !strncmp(line, "mcast ", 6)) settings_apply_line(p, line, 1);
            fclose(f);
        }
        p->dirty = 0;
    }
    dclock_cfg_t dc = { o->clock_owner, p->fs, o->audio_loop, o->cs2k, o->cs2k_r0, o->fail_leader };
    p->clk = dclock_new(hw, &dc);
    dclock_on_load(p->clk, on_load, p);
    if (o->clock_owner) hw_wr(hw, hw->clock_base + INF_MEDIA_FS, (uint32_t)p->fs);   /* port 0's rate drives the desk clock */
    ptp_cfg_t pc = { o->ptp_role, o->log_sync, o->stratum ? o->stratum : 4, (int16_t)0xF060, o->ptp_steer };
    p->ptp = ptp_new(p->net, p->clk, p->mac, &pc);
    port_apply_hw(p);
    if (net_ready(p->net)) {
        info_init(p);
        port_advertise(p);
    }
    LOGI("port %d up: %d TX / %d RX channels at %d Hz, latency %u ns, PTP role %d%s", p->n + 1, port_tx_channels(p), p->channels, p->fs,
         port_latency_ns(p), o->ptp_role, o->clock_owner ? ", desk clock owner" : "");
    return 0;
}

void portd_stop(port_t *p)
{
    port_unadvertise(p);
    sub_shutdown(p);
    uint32_t z = 0;
    for (int f = 0; f < p->caps_txf; f++) tbl_write(p->hw, TBL_TX(f, 0), &z, 1);
    for (int f = 0; f < p->caps_rxf; f++) tbl_write(p->hw, TBL_RX(f, 0), &z, 1);
    if (p->state_path[0]) settings_save(p);
    if (p->ptp) ptp_free(p->ptp);
    if (p->clk) dclock_free(p->clk);
    mdns_free(p->mdns);
    net_free(p->net);
}
