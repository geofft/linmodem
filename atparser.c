/* 
 * Simple AT command parser
 * 
 * Copyright (c) 2000 Fabrice Bellard.
 *
 * This code is released under the GNU General Public License version
 * 2. Please read the file COPYING to know the exact terms of the
 * license.
 * 
 */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>

#include "lm.h"

#define DEBUG

static void parse_at_line(struct lm_at_state *s, const char *line);
static void at_putc(struct lm_at_state *s, int c);
static void at_printf(struct lm_at_state *s, char *fmt, ...);

/*
 * This AT command parser is not intended to be complete nor accurate
 * but to provide the minimum functionality so that the programs can
 * make a data/voice/fax connection. 
 */

void lm_at_parser_init(struct lm_at_state *s, struct sm_state *sm)
{
    s->sm = sm;
    s->at_line_ptr = 0;
    s->at_state = AT_MODE_COMMAND;
}

void lm_at_parser(struct lm_at_state *s)
{
    int state, c;

    switch(s->at_state) {
    case AT_MODE_COMMAND:
        for(;;) {
            /* handle incoming character */
            c = sm_get_bit(&s->sm->tx_fifo);
            if (c == -1)
                break;
            
            if (c == '\n' || c == '\r') {
                /* line validation */
                s->at_line[s->at_line_ptr] = '\0';
                if (s->at_line_ptr != 0) {
                    at_putc(s, '\n');
                    parse_at_line(s, s->at_line);
                }
                s->at_line_ptr = 0;
            } else if (c == '\b') {
                /* backspace */
                if (s->at_line_ptr > 0) {
                    s->at_line_ptr--;
                    at_printf(s, "\b \b");
                }
            } else {
                /* add new char */
                if (s->at_line_ptr < (sizeof(s->at_line) - 1)) {
                    s->at_line[s->at_line_ptr++] = c;
                    at_putc(s, c);
                }
            }
        }
        break;
    case AT_MODE_DIALING:
        state = lm_get_state(s->sm);
        if (state == LM_STATE_IDLE) {
            at_printf(s, "ERROR\n");
            s->at_state = AT_MODE_COMMAND;
        } else if (state == LM_STATE_CONNECTED) {
            at_printf(s, "CONNECT\n");
            s->at_state = AT_MODE_CONNECTED;
        }
        break;

    case AT_MODE_CONNECTED:
        if (lm_get_state(s->sm) == LM_STATE_IDLE) {
            at_printf(s, "OK\n");
            s->at_state = AT_MODE_COMMAND;
        }
        break;
    }
}

/* return TRUE if val is a prefix of str. If it returns TRUE, ptr is
   set to the next character in 'str' after the prefix */
#if 0
static int strstart(const char *str, const char *val, const char **ptr)
{
    const char *p, *q;
    p = str;
    q = val;
    while (*q != '\0') {
        if (*p != *q)
            return 0;
        p++;
        q++;
    }
    *ptr = p;
    return 1;
}
#endif

static int strcasestart(const char *str, const char *val, const char **ptr)
{
    const char *p, *q;
    p = str;
    q = val;
    while (*q != '\0') {
        if (toupper(*p) != toupper(*q))
            return 0;
        p++;
        q++;
    }
    *ptr = p;
    return 1;
}

static void at_putc(struct lm_at_state *s, int c)
{
    if (c == '\n') 
        sm_put_bit(&s->sm->rx_fifo, '\r');
    sm_put_bit(&s->sm->rx_fifo, c);
}

static void at_printf(struct lm_at_state *s, char *fmt, ...)
{
    char buf[256], *p;
    int c;

    va_list ap;

    va_start(ap, fmt);
    snprintf(buf, sizeof(buf), fmt, ap);
    p = buf;
    while (*p) {
        c = *p++;
        at_putc(s, c);
    }
    va_end(ap);
}

static void parse_at_line(struct lm_at_state *s, const char *line)
{
    const char *p;
    struct sm_state *sm = s->sm;

    if (strcasestart(line, "ATD", &p)) {
        int pulse;

        pulse = sm->lm_config->pulse_dial;
        if (strcasestart(p, "P", &p))
            pulse = 1;
        if (strcasestart(p, "T", &p))
            pulse = 0;
        lm_start_dial(sm, pulse, p);
        s->at_state = AT_MODE_DIALING;
    } else if (strcasestart(line, "ATI", &p)) {
        at_printf(s, "Linmodem " LM_VERSION "\n");
    } else if (strcasestart(line, "ATZ", &p)) {
        at_printf(s, "OK\n");
    } else if (strcasestart(line, "ATA", &p)) {
        lm_start_receive(sm);
        s->at_state = AT_MODE_DIALING;
    } else if (strcasestart(line, "ATH", &p)) {
        lm_hangup(sm);
    }
}
