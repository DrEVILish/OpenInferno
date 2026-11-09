/* pktq.h - the port's CPU packet queue (fpga/rtl/cpu_pktq.v, OCQ1): whole Ethernet frames to and from the PHY with
 * SFD time stamps. Part of OpenProSeries inferno-portd. Licence: GPL-3.0-or-later.
 */
#ifndef PORTD_PKTQ_H
#define PORTD_PKTQ_H

#include "hw.h"

typedef struct {
    uint32_t sec;   /* the time of day's seconds (low 32 bits) */
    uint32_t ns;
} stamp_t;

int  pktq_init(hw_t *h);   /* checks the ID, enables TX / RX with all-multicast and the 239/8 drop */
/* queues one frame (14..1514 bytes without FCS); ts_req asks for the TX stamp under `tag`. 0 or -1 (no room) */
int  pktq_send(hw_t *h, const uint8_t *frame, int len, int ts_req, uint8_t tag);
/* the oldest TX stamp, if one waits: 1 and its tag / time */
int  pktq_tx_stamp(hw_t *h, uint8_t *tag, stamp_t *ts);
/* the head received frame into buf (max bytes); returns its length (0: none) and pops it */
int  pktq_recv(hw_t *h, uint8_t *buf, int max, stamp_t *ts);

#endif
