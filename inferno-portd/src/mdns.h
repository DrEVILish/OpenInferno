/* mdns.h - a small mDNS / DNS-SD responder and resolver per port (RFC 6762 / 6763), for the records upstream Inferno
 * advertises (inferno_aoip/src/device_server/mdns_server.rs, searchfire's broadcaster: one response per service,
 * PTR + SRV + TXT answers, the A record of "<service name>.local" as additional) and the lookups its subscriber makes
 * (mdns_client.rs: SRV + TXT of "<channel>@<device>._netaudio-chan._udp.local", the bundle, the A record).
 * Each port is its own host "<DESK>-<n>", which is why this is not avahi (one host name per machine).
 * Part of OpenProSeries inferno-portd. Licence: GPL-3.0-or-later.
 */
#ifndef PORTD_MDNS_H
#define PORTD_MDNS_H

#include "net.h"

#define MDNS_PORT  5353
#define MDNS_GROUP IP4(224, 0, 0, 251)
#define MDNS_MAXTXT 16

typedef struct mdns mdns_t;

typedef struct {
    char     inst[96];          /* the service name label: "FOH-1", "01@FOH-1", "1@FOH-1"; or "d.c.b.a" for in-addr */
    char     type[32];          /* "_netaudio-arc", ...; "in-addr" = a multicast address reservation (A only) */
    uint16_t port;
    uint32_t ttl;
    int      ntxt;
    char     txt[MDNS_MAXTXT][100];
    char     origin[64];        /* in-addr: the additional "_inferno-response-origin.local" TXT */
} mdns_svc_t;

mdns_t *mdns_new(net_t *n);
void    mdns_free(mdns_t *m);
/* add or replace (same inst + type) a service; announced on the next tick. Returns its id */
int     mdns_add(mdns_t *m, const mdns_svc_t *s);
/* withdraw (a goodbye with TTL 0) */
void    mdns_remove(mdns_t *m, const char *inst, const char *type);
void    mdns_remove_all(mdns_t *m);
void    mdns_tick(mdns_t *m, uint64_t now);

/* resolver: sends a query (rate-limited per name) and reads the cache */
void    mdns_query(mdns_t *m, const char *fqdn, uint16_t qtype);
/* SRV of an instance FQDN: 1 = found */
int     mdns_srv(mdns_t *m, const char *fqdn, uint16_t *port, char *target, size_t tsz);
/* one TXT key's value: 1 = found; txt_any = 1 if the record exists at all */
int     mdns_txt(mdns_t *m, const char *fqdn, const char *key, char *val, size_t vsz, int *txt_any);
int     mdns_a(mdns_t *m, const char *fqdn, uint32_t *ip);
/* drops cached records of a name (a re-resolve after a timeout) */
void    mdns_forget(mdns_t *m, const char *fqdn);

/* DNS wire helpers (tests) */
int     dns_put_name(uint8_t *p, int max, const char *fqdn);
int     dns_get_name(const uint8_t *msg, int len, int off, char *out, int outsz);
/* builds the response packet of a service into buf (id, ttl override < 0: the service's); returns its length */
int     mdns_build_response(const mdns_svc_t *s, uint32_t ip, uint16_t id, long ttl, uint8_t *buf, int max);
void    mdns_fqdn(const mdns_svc_t *s, char *out, size_t sz);   /* "<inst>.<type>._udp.local" */

typedef struct { uint64_t rx_queries, tx_responses, tx_queries, rx_responses, cache; } mdns_stats_t;
void    mdns_get_stats(const mdns_t *m, mdns_stats_t *s);

#endif
