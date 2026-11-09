/* ctl.h - the control socket (ctl.c). Part of OpenProSeries inferno-portd. Licence: GPL-3.0-or-later. */
#ifndef PORTD_CTL_H
#define PORTD_CTL_H

#include "portd.h"

#define CTL_CLIENTS 4
typedef struct {
    int lfd;
    char path[108];
    struct ctl_client { int fd, n; char buf[2048]; } cl[CTL_CLIENTS];
} ctl_t;

/* runs one command line; out gets the answer. Returns 1 for "quit", 0 ok, -1 error */
int  ctl_exec(port_t *p, const char *line, char *out, size_t sz);
int  ctl_listen(ctl_t *c, const char *path);
int  ctl_poll(ctl_t *c, port_t *p);   /* 1 = a client asked to quit */
void ctl_close(ctl_t *c);

#endif
