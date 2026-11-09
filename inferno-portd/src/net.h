/* net.h - a minimal IPv4 stack on the port's CPU packet queue (fpga/docs/inferno.md section 9.2, "a minimal ARP /
 * IGMP / UDP layer in the daemon"): Ethernet II, ARP (cache, resolve, reply, RFC 5227 probes), IPv4 link-local
 * addressing (RFC 3927), UDP sockets, IGMPv2 joins (RFC 2236), ICMP echo. SFD stamps reach the UDP callbacks.
 * Part of OpenProSeries inferno-portd. Licence: GPL-3.0-or-later.
 */
#ifndef PORTD_NET_H
#define PORTD_NET_H

#include "hw.h"
#include "pktq.h"

#define IP4(a, b, c, d) (((uint32_t)(a) << 24) | ((uint32_t)(b) << 16) | ((uint32_t)(c) << 8) | (uint32_t)(d))

typedef void (*udp_cb)(void *ctx, uint32_t sip, uint16_t sport, uint32_t dip, uint16_t dport,
                       const uint8_t *d, int n, const stamp_t *ts);
typedef void (*txstamp_cb)(void *ctx, uint8_t tag, const stamp_t *ts);

typedef struct {
    uint8_t ttl, tos;
    uint8_t ts_req, tag;
} txopt_t;

typedef struct net net_t;

net_t *net_new(hw_t *h, const uint8_t mac[6]);
void   net_free(net_t *n);
/* a static address (0: RFC 3927 link-local, chosen from the MAC and probed) */
void   net_set_ip(net_t *n, uint32_t ip, uint32_t mask, uint32_t gw);
int    net_ready(const net_t *n);           /* the address is ours (probed) */
uint32_t net_ip(const net_t *n);
uint32_t net_mask(const net_t *n);
uint32_t net_gw(const net_t *n);
const uint8_t *net_mac(const net_t *n);

/* port 0 = a free ephemeral port; returns the port or -1 */
int    net_udp_bind(net_t *n, uint16_t port, udp_cb cb, void *ctx);
void   net_udp_unbind(net_t *n, uint16_t port);
int    net_udp_send(net_t *n, uint16_t sport, uint32_t dip, uint16_t dport, const void *d, int len, const txopt_t *o);
/* the MAC of an address (multicast mapped, unicast from ARP): 1 = known; 0 = asked (an ARP request went out) */
int    net_mac_of(net_t *n, uint32_t ip, uint8_t mac[6]);
void   net_join(net_t *n, uint32_t group);
void   net_leave(net_t *n, uint32_t group);
void   net_set_txstamp_cb(net_t *n, txstamp_cb cb, void *ctx);

void   net_input(net_t *n, const uint8_t *f, int len, const stamp_t *ts);
void   net_poll(net_t *n);                 /* drains the RX queue and the TX stamps */
void   net_tick(net_t *n, uint64_t now_ns);

typedef struct { uint64_t rx, tx, arp_rx, arp_tx, udp_rx, udp_tx, drops, igmp_tx, icmp; } net_stats_t;
void   net_get_stats(const net_t *n, net_stats_t *s);

/* RFC 1071 */
uint16_t inet_csum(const uint8_t *p, int n, uint32_t seed);

#endif
