/* ptp.h - PTPv1 (IEEE 1588-2002, as Dante uses it) on one port: the follower (Sync / Follow_Up / Delay_Req /
 * Delay_Resp, offset and delay for the servo), our own leader (two-step Sync + Follow_Up, Delay_Resp; upstream
 * Inferno has no PTPv1 leader) and the best-master choice between them.
 *
 * Message layouts are translated from the PTPv1 code of upstream Inferno's clock daemon, the Statime fork it ships
 * with (github.com/teodly/statime, branch inferno-dev, Apache-2.0 / MIT: statime/src/datastructures/messages_v1/ (*.rs),
 * common/grandmaster_v1.rs, timestamp_v1.rs, bmc/dataset_comparison.rs `from_sync_v1_message`), checked against the
 * octet positions of IEEE 1588-2002. Follow_Up is read and written as 1588-2002 lays it out (associatedSequenceId at
 * octets 42-43, preciseOriginTimestamp at 44-51: Statime's deserialiser; its serialiser is not used by Statime).
 * Part of OpenProSeries inferno-portd. Licence: GPL-3.0-or-later.
 */
#ifndef PORTD_PTP_H
#define PORTD_PTP_H

#include "net.h"
#include <stdint.h>

#define PTP1_GROUP   IP4(224, 0, 1, 129)
#define PTP1_EVENT   319
#define PTP1_GENERAL 320

enum { PTP1_SYNC = 0, PTP1_DELAY_REQ = 1, PTP1_FOLLOW_UP = 2, PTP1_DELAY_RESP = 3, PTP1_MGMT = 4 };
#define PTP1_FLAG_ASSIST 0x08   /* two-step: a Follow_Up carries the precise origin */

typedef struct {
    uint8_t  subdomain[16];
    uint8_t  msgtype;            /* 1 event, 2 general */
    uint8_t  comm;               /* 1 = Ethernet */
    uint8_t  uuid[6];
    uint16_t port, seq;
    uint8_t  control;
    uint16_t flags;
} ptp1_hdr_t;

typedef struct {
    uint8_t  comm, uuid[6];
    uint16_t port, seq;
    uint8_t  stratum;
    char     ident[4];
    int16_t  variance;
    uint8_t  preferred, boundary;
} ptp1_gm_t;

typedef struct {
    uint32_t sec, ns;
    uint16_t epoch;
    int16_t  utc_offset;
    ptp1_gm_t gm;
    int8_t   sync_interval;
    int16_t  local_variance;
    uint16_t steps;
    uint8_t  local_stratum;
    char     local_ident[4];
    uint8_t  parent_comm, parent_uuid[6];
    uint16_t parent_port;
    int16_t  est_variance;
    int32_t  est_drift;
    uint8_t  utc_reasonable;
} ptp1_sync_t;

typedef struct {
    uint32_t sec, ns;
    uint8_t  req_comm, req_uuid[6];
    uint16_t req_port, req_seq;
} ptp1_dresp_t;

typedef struct {
    uint16_t assoc_seq;
    uint32_t sec, ns;
} ptp1_fup_t;

int ptp1_put_sync(uint8_t *b, const ptp1_hdr_t *h, const ptp1_sync_t *s);          /* Sync / Delay_Req: 124 bytes */
int ptp1_put_followup(uint8_t *b, const ptp1_hdr_t *h, const ptp1_fup_t *f);       /* 52 bytes */
int ptp1_put_delay_resp(uint8_t *b, const ptp1_hdr_t *h, const ptp1_dresp_t *r);   /* 60 bytes */
int ptp1_get_hdr(const uint8_t *b, int n, ptp1_hdr_t *h);
int ptp1_get_sync(const uint8_t *b, int n, ptp1_sync_t *s);
int ptp1_get_followup(const uint8_t *b, int n, ptp1_fup_t *f);
int ptp1_get_delay_resp(const uint8_t *b, int n, ptp1_dresp_t *r);

/* the data set a Sync announces, for the best-master choice */
typedef struct {
    uint8_t  preferred, stratum;
    char     ident[4];
    int16_t  variance;
    uint8_t  uuid[6];
    uint16_t steps;
} ptp1_ds_t;
/* < 0: a is the better master, > 0: b is, 0: the same clock */
int ptp1_compare(const ptp1_ds_t *a, const ptp1_ds_t *b);

/* ---- one port's PTP engine */
enum { PTP_ROLE_OFF = 0, PTP_ROLE_AUTO = 1, PTP_ROLE_LEADER = 2, PTP_ROLE_FOLLOWER = 3 };
enum { PTP_LISTENING = 0, PTP_MASTER = 1, PTP_SLAVE = 2, PTP_PASSIVE = 3, PTP_DISABLED = 4 };

typedef struct ptp ptp_t;
struct dclock;

typedef struct {
    int      role;
    int8_t   log_sync;           /* the leader's sync interval, log2 s (-2 = 250 ms) */
    uint8_t  stratum;            /* our data set */
    int16_t  variance;
    int      steer;              /* this port's offsets steer the desk clock (the desk follows this port) */
} ptp_cfg_t;

ptp_t *ptp_new(net_t *n, struct dclock *clk, const uint8_t mac[6], const ptp_cfg_t *cfg);
void   ptp_free(ptp_t *p);
void   ptp_set_role(ptp_t *p, int role, int steer);
void   ptp_tick(ptp_t *p, uint64_t now);
int    ptp_state(const ptp_t *p);
const char *ptp_state_name(int s);

typedef struct {
    int      state;
    uint8_t  leader_uuid[6];      /* the clock followed */
    double   offset_ns, delay_ns;   /* the last follower measurement */
    uint64_t syncs_rx, syncs_tx, fups_rx, fups_tx, dreq_tx, dreq_rx, dresp_tx, dresp_rx, samples;
    int      servo_state;
} ptp_stats_t;
void   ptp_get_stats(const ptp_t *p, ptp_stats_t *s);

#endif
