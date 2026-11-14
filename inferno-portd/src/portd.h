/* portd.h - inferno-portd: the AMU control plane of one FPGA Inferno port (OpenProSeries fpga/docs/inferno.md
 * section 9). One process per port in mode 1; the port is the NetAudio device "<DESK>-<n>" (port A = 1).
 *
 * The Dante / Inferno protocol code is translated into C from upstream Inferno (github.com/teodly/inferno, GPL-3.0-or-
 * later, commit 9767558e0905c9e9c960f5a60e050fbdc38300b7, inferno_aoip/src): protocol/req_resp.rs, proto_arc.rs,
 * proto_cmc.rs, flows_control.rs, mcast.rs; device_server/arc_server.rs, cmc_server.rs, flows_control_server.rs,
 * channels_subscriber.rs, flows_tx.rs, tx_multicasts.rs, info_mcast_server.rs, mdns_server.rs, mdns_client.rs. The
 * media packets themselves are the FPGA's (inferno_tx.v / inferno_rx.v): this daemon programs their flow tables.
 * Licence: GPL-3.0-or-later (as upstream).
 */
#ifndef PORTD_H
#define PORTD_H

#include "hw.h"
#include "mdns.h"
#include "net.h"

/* compile-time maxima: they size the tables; the build's INF_CAPS values (port_t caps_*) are the run-time limits */
#define MAX_CH      64      /* columns per port at 48 kHz (half at 96 kHz) */
#define NTXF        16      /* FPGA TX flows */
#define NRXF        16      /* FPGA RX flows */
#define MAXFCH      32      /* channels per flow, either way */
#define ARC_PORT    4440
#define CMC_PORT    8800
#define FLOWCTL_PORT 4455
#define INFO_PORT   8700
#define MEDIA_PORT  4321    /* multicast bundles (upstream tx_multicasts.rs) */
#define DBCP1       0x1102
#define FPP_MIN     2
#define FPP_MAX_ADV 32      /* upstream FPP_MAX_ADVERTISED; also the FPGA's MAX_FPP */
#define KEEPALIVE_NS        250000000ull    /* upstream KEEPALIVE_INTERVAL */
#define KEEPALIVE_TIMEOUT_NS 4000000000ull  /* upstream KEEPALIVE_TIMEOUT_SECONDS */
#define TX_BUDGET_BPS 95000000.0            /* admission control: 95 % of 100 Mb/s (inferno.md section 5) */

/* upstream SubscriptionStatus */
enum {
    SUB_NONE = 0, SUB_UNRESOLVED = 1, SUB_IN_PROGRESS = 8, SUB_TOO_MANY = 0x14, SUB_TX_FAIL = 0x15,
    SUB_LATENCY = 0x1A,   /* Dante RX status 26, channel latency: the transmitter needs more than our buffer holds */
    SUB_RX_UNICAST = 0x01010009, SUB_RX_MULTICAST = 0x0101000a,
};

typedef struct {
    int      used, multicast;
    uint16_t cookie;
    char     rx_host[64], rx_flow[64];
    uint32_t dst_ip;
    uint16_t dst_port, src_port;
    int      nch, bps, fpp;          /* fpp 0 = automatic (the latency mode's) */
    int16_t  ch[MAXFCH];             /* local TX channel index, -1 = none */
    uint64_t expires;                /* unicast: keepalive deadline */
    int      programmed;
    uint8_t  mac[6];
    int      mc_state;               /* multicast: 0 probing the address, 1 active */
    uint64_t mc_t;
    int      mc_probes;
} txflow_t;

typedef struct {
    int      used, multicast;
    char     host[64];               /* the transmitting device */
    uint32_t ctl_ip;
    uint16_t ctl_port, dbcp1;
    uint8_t  handle[6];
    int      state;                  /* 0 requesting, 1 running, 2 updating */
    uint16_t rx_port;                /* unicast: our port the flow is sent to */
    uint32_t group;                  /* multicast: the bundle's group and port */
    uint16_t gport;
    int      bundle_id;
    int      nch, bps, fpp, shift;
    uint16_t txch[MAXFCH];           /* the transmitter's channel id (1-based), 0 = none */
    int16_t  col[MAXFCH];            /* our column (local RX channel index), -1 = none */
    uint16_t pend_txch[MAXFCH];      /* an update in flight */
    int16_t  pend_col[MAXFCH];
    int      pend_nch;
    int      per;                    /* the most channels this flow may carry */
    uint32_t pkts;
    uint64_t pkts_seen_at, created, next_ka, req_at;
    uint16_t req_seq;
    int      req_tries;
    uint32_t src_ip;
    uint16_t src_port;
    uint32_t latency_ns, peer_latency_ns;
} rxflow_t;

typedef struct {
    int      active;
    char     txch[32], txhost[64];
    int      status;
    int      flow, slot;             /* -1: not in a flow */
    uint64_t next_try;
    uint64_t ready_at;               /* resolved, waiting to be gathered into a new flow */
} sub_t;

typedef struct ptp ptp_t;
typedef struct dclock dclock_t;

typedef struct port {
    /* configuration */
    int      n;                      /* port index (0 = A): device "<DESK>-<n + 1>" */
    char     desk[32];
    char     name[40];               /* friendly hostname */
    char     factory[40];
    uint8_t  mac[6];
    uint8_t  devid[8];
    uint16_t pid;
    uint32_t cfg_ip, cfg_mask, cfg_gw;
    int      fs;
    int      lat_mode;               /* 0..3 = 0.5 / 1 / 2 / 5 ms */
    int      channels;               /* columns: the RX channels; TX channels = port_tx_channels() */
    int      maxlat, caps_ch, caps_fch, caps_txf, caps_rxf;
    char     txname[MAX_CH][32], rxname[MAX_CH][32];
    char     state_path[256];
    int      dirty;
    /* run time */
    hw_t    *hw;
    net_t   *net;
    mdns_t  *mdns;
    int      advertised;
    txflow_t tx[NTXF];
    rxflow_t rx[NRXF];
    sub_t    sub[MAX_CH];
    uint16_t fc_seq, info_seq;
    int      fc_sock;                /* our flow-control client port */
    uint64_t next_sub_scan, next_ka, next_heartbeat, next_flow_scan, next_save;
    int      resolve_now;
    ptp_t   *ptp;
    dclock_t *clk;
    int      changed_rx[MAX_CH];     /* channels for the next change notification */
    int      any_changed;
} port_t;

/* a build without TX flows (receive-only) has no TX channels */
static inline int port_tx_channels(const port_t *p) { return p->caps_txf ? p->channels : 0; }
/* a build without RX flows (transmit-only) has no RX channels */
static inline int port_rx_channels(const port_t *p) { return p->caps_rxf ? p->channels : 0; }

/* identity and adverts (port.c) */
void port_defaults(port_t *p);
void port_set_identity(port_t *p);              /* name, factory name, device id from desk / n / MAC */
void port_advertise(port_t *p);                 /* (re)announce every mDNS record */
void port_unadvertise(port_t *p);
uint32_t port_latency_ns(const port_t *p);
int  port_offset_frames(const port_t *p);
int  port_auto_fpp(const port_t *p);
void port_apply_hw(port_t *p);                  /* mode, addresses, rate, latency, enables */
void port_tick(port_t *p, uint64_t now);
int  port_set_latency(port_t *p, int mode);
int  port_set_rate(port_t *p, int fs);
void port_rename(port_t *p, const char *desk);
void port_rename_tx(port_t *p, int k, const char *name);
void port_rename_rx(port_t *p, int k, const char *name);
void sanitize_chname(const char *in, char *out, size_t sz);

/* the req / resp packet (protocol/req_resp.rs): start, length, seq, opcode1, opcode2 */
#define RR_HDR 10
int  rr_make(uint8_t *buf, uint16_t start, uint16_t seq, uint16_t op1, uint16_t op2, const uint8_t *c, int clen);

/* ARC server (arc.c, from arc_server.rs) */
void arc_init(port_t *p);
/* the response to one ARC request (tests call it directly): returns the length of the packet built into out */
int  arc_handle(port_t *p, const uint8_t *req, int len, uint8_t *out, int max);
/* CMC (cmc.c) */
void cmc_init(port_t *p);
int  cmc_handle(port_t *p, const uint8_t *req, int len, uint8_t *out, int max);

/* TX flows (flows_tx.c, from flows_tx.rs / flows_control_server.rs / tx_multicasts.rs) */
void flowctl_init(port_t *p);
int  flowctl_handle(port_t *p, uint32_t sip, const uint8_t *req, int len, uint8_t *out, int max);
void txflows_tick(port_t *p, uint64_t now);
void txflows_rate_changed(port_t *p);
void txflows_reanchor(port_t *p);               /* after a media clock load: rewrite every w0 */
int  txflow_create_mcast(port_t *p, int flow_index, const int16_t *ch, int nch);
int  txflow_delete_mcast(port_t *p, int flow_index);
double tx_load_bps(const port_t *p, int extra_nch, int extra_fpp, int extra_bps);
int  tx_eff_fpp(const port_t *p, int fpp, int nch, int bps);

/* RX: subscriptions (subscriber.c, from channels_subscriber.rs / flows_rx.rs / flows_control.rs client) */
void sub_init(port_t *p);
int  sub_subscribe(port_t *p, int k, const char *txch, const char *txhost);
void sub_unsubscribe(port_t *p, int k);
void sub_tick(port_t *p, uint64_t now);
int  sub_status(const port_t *p, int k);
int  sub_latency_fits(const port_t *p, uint32_t their_ns);   /* our jitter buffer holds a peer at this latency */
void sub_retime(port_t *p);
void sub_reset_rate(port_t *p);
void sub_shutdown(port_t *p);                  /* stops every RX flow (the daemon exits) */
/* flow request body (flows_control.rs request_flow), for tests */
int  fc_build_request(const port_t *p, uint16_t seq, uint32_t rate, uint32_t bps, uint16_t fpp, const uint16_t *ch, int nch,
                      uint16_t rx_port, const char *flow_name, uint8_t *out, int max);

/* info / heartbeat multicasts (info.c, from info_mcast_server.rs, mcast.rs) */
void info_init(port_t *p);
void info_tick(port_t *p, uint64_t now);
void info_channel_change(port_t *p, const int *chans, int n);
int  mcast_make(uint8_t *buf, uint16_t start, uint16_t seq, uint16_t process, const uint8_t devid[8], const char *vendor,
                const uint8_t opcode[8], const uint8_t *c, int clen);

/* persistence (settings.c) */
int  settings_load(port_t *p);
int  settings_save(port_t *p);

#endif
