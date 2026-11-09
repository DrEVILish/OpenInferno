/* settings.c - the port's persistent state (upstream saved_settings.rs / state_storage.rs, as a text file next to
 * OMC's console prefs): channel names, subscriptions, multicast bundles, latency, rate, desk name.
 * Part of OpenProSeries inferno-portd. Licence: GPL-3.0-or-later.
 */
#include "portd.h"
#include "util.h"

#include <stdio.h>
#include <stdlib.h>

int settings_save(port_t *p)
{
    char tmp[300];
    snprintf(tmp, sizeof(tmp), "%s.tmp", p->state_path);
    FILE *f = fopen(tmp, "w");
    if (!f) { LOGW("cannot write %s", tmp); return -1; }
    fprintf(f, "# inferno-portd state, port %d\n", p->n + 1);
    fprintf(f, "deskname %s\nrate %d\nlatency %d\nchannels %d\n", p->desk, p->fs, p->lat_mode, p->channels);
    for (int k = 0; k < MAX_CH; k++) {
        char d[8];
        snprintf(d, sizeof(d), "%02d", k + 1);
        if (strcmp(p->txname[k], d)) fprintf(f, "txname %d %s\n", k + 1, p->txname[k]);
        if (strcmp(p->rxname[k], d)) fprintf(f, "rxname %d %s\n", k + 1, p->rxname[k]);
        if (p->sub[k].active) fprintf(f, "sub %d %s@%s\n", k + 1, p->sub[k].txch, p->sub[k].txhost);
    }
    for (int i = 0; i < NTXF; i++) {
        const txflow_t *t = &p->tx[i];
        if (!t->used || !t->multicast) continue;
        fprintf(f, "mcast %d", i + 1);
        for (int c = 0; c < t->nch; c++) fprintf(f, "%c%d", c ? ',' : ' ', t->ch[c] + 1);
        fprintf(f, "\n");
    }
    int failed = ferror(f);
    if (fclose(f)) failed = 1;
    if (failed) { LOGW("cannot finish %s", tmp); remove(tmp); return -1; }
    if (rename(tmp, p->state_path)) { LOGW("cannot replace %s", p->state_path); return -1; }
    return 0;
}

int settings_apply_line(port_t *p, const char *line, int at_start);
int settings_apply_line(port_t *p, const char *line, int at_start)
{
    char key[32], a[96], b[96];
    int k;
    if (sscanf(line, "%31s", key) != 1 || key[0] == '#') return 0;
    if (!strcmp(key, "deskname") && sscanf(line, "%*s %31s", a) == 1) {
        if (at_start) { snprintf(p->desk, sizeof(p->desk), "%s", a); port_set_identity(p); }
        else port_rename(p, a);
    } else if (!strcmp(key, "rate") && sscanf(line, "%*s %d", &k) == 1) {
        if (at_start) p->fs = k == 96000 ? 96000 : 48000;
        else port_set_rate(p, k);
    } else if (!strcmp(key, "latency") && sscanf(line, "%*s %d", &k) == 1) {
        if (at_start) p->lat_mode = k & 3;
        else port_set_latency(p, k);
    } else if (!strcmp(key, "channels") && sscanf(line, "%*s %d", &k) == 1) {
        if (k >= 1 && k <= MAX_CH) p->channels = k;
    } else if (!strcmp(key, "txname") && sscanf(line, "%*s %d %95[^\n]", &k, a) == 2) {
        if (at_start) { if (k >= 1 && k <= MAX_CH) sanitize_chname(a, p->txname[k - 1], sizeof(p->txname[0])); }
        else port_rename_tx(p, k - 1, a);
    } else if (!strcmp(key, "rxname") && sscanf(line, "%*s %d %95[^\n]", &k, a) == 2) {
        if (k >= 1 && k <= MAX_CH) sanitize_chname(a, p->rxname[k - 1], sizeof(p->rxname[0]));
    } else if (!strcmp(key, "sub") && sscanf(line, "%*s %d %95[^@]@%95s", &k, a, b) == 3) {
        return sub_subscribe(p, k - 1, a, b);
    } else if (!strcmp(key, "mcast") && sscanf(line, "%*s %d %95s", &k, a) == 2) {
        int16_t ch[MAXFCH];
        int n = 0;
        for (char *s = a; *s && n < MAXFCH;) {
            ch[n++] = (int16_t)(atoi(s) - 1);
            char *c = strchr(s, ',');
            if (!c) break;
            s = c + 1;
        }
        return txflow_create_mcast(p, k - 1, ch, n);
    } else return -1;
    return 0;
}

int settings_load(port_t *p)
{
    FILE *f = fopen(p->state_path, "r");
    if (!f) return -1;
    char line[256];
    while (fgets(line, sizeof(line), f)) settings_apply_line(p, line, 1);
    fclose(f);
    return 0;
}
