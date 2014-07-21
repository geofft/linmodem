/* 
 * The Generic Linux Soft Modem
 * 
 * Copyright (c) 1999,2000 Fabrice Bellard.
 * Copyright (c) 1999 Pavel Machek
 *
 * This code is released under the GNU General Public License version
 * 2. Please read the file COPYING to know the exact terms of the
 * license.
 * 
 * This implementation is totally clean room. It was written by
 * reading the ITU specification and by using basic signal processing
 * knowledge.  
 */
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/time.h>
#include <signal.h>

#include "lm.h"
#include "v34.h"
#include "v90.h"

#define DEBUG

int lm_debug = 0;
int debug_state = 0;

/* default parameters */
static LinModemConfig default_lm_config = 
{
    pulse_dial: 0,
    dtmf_level: -9, 
    dtmf_digit_length: 150,
    dtmf_pause_length: 100,
    available_modulations: V8_MOD_V21 | V8_MOD_V23,
};

/* fifo handling */

void sm_init_fifo(struct sm_fifo *f, u8 *buf, int size)
{
    f->sptr = buf;
    f->eptr = buf + size;
    f->wptr = f->rptr = f->sptr;
    f->size = 0;
    f->max_size = size;
}

void sm_flush(struct sm_fifo *f)
{
    f->wptr = f->rptr = f->sptr;
    f->size = 0;
}

int sm_size(struct sm_fifo *f)
{
    return f->size;
}

void sm_put_bit(struct sm_fifo *f, int v)
{
    if (f->size < f->max_size) {
        *f->wptr++ = v;
        if (f->wptr == f->eptr) f->wptr = f->sptr;
        f->size++;
    }
}

/* put bits, from MSB to LSB */
void sm_put_bits(struct sm_fifo *f, int v, int n)
{
    int i;
    for(i=n-1;i>=0;i--) {
        sm_put_bit(f, (v >> i) & 1);
    }
}

int sm_peek_bit(struct sm_fifo *f)
{
    if (f->size > 0)
	return *f->rptr;
    else
	return -1;
}

int sm_get_bit(struct sm_fifo *f)
{
    int v;
    
    if (f->size > 0) {
        f->size--;
        v = *f->rptr++;
        if (f->rptr == f->eptr) f->rptr = f->sptr;
        return v;
    } else {
        /* fifo empty */
        return -1;
    }
}

/* return -1 if not enough bits */
int sm_get_bits(struct sm_fifo *f, int n)
{
    int v,i;

    if (f->size < n)
        return -1;

    v = 0;
    for(i=n-1;i>=0;i--) {
        v |= (sm_get_bit(f) << i);
    }
    return v;
}

/* timer handling */

/* note: the current time handling is horrible because we use a global
   state which is initialized when entering in sm_process() */
static unsigned int sim_time;


/* current time, in samples */
int sm_time(void)
{
    return sim_time;
}

/* delay is in ms */
void sm_set_timer(struct sm_timer *t, int delay)
{
    t->timeout = sm_time() + (delay * 8000) / 1000;
}

/* return 1 if timer expired */
int sm_check_timer(struct sm_timer *t)
{
    long timeout;
    
    timeout = sm_time();
    return (timeout >= t->timeout);
}

/* 
   Main modem state machine. It handles the dialing & rings, and then
   call the corresponding protocol handlers.  
 */

char *sm_states_str[] = {
#define TAG(s) #s ,
#include "lmstates.h"
};

static int dtmf_get_digit(void *opaque)
{
    struct sm_state *sm = opaque;
    if (sm->dtmf_ptr < sm->dtmf_len) {
        return sm->call_num[sm->dtmf_ptr++];
    } else {
        return -1;
    }
}

static void dtmf_put_digit(void *opaque, int digit)
{
    printf("DTMF: got digit '%c'\n", digit);
}

void sm_process(struct sm_state *sm, s16 *output, s16 *input, int nb_samples)
{
    /* XXX: time hack */
    sim_time = sm->time;

    /* modulation */
    switch(sm->state) {
    case SM_DTMF_DIAL_WAIT:
    case SM_DTMF_DIAL_WAIT1:
        DTMF_mod(&sm->dtmf_tx, output, nb_samples);
        break;
    default:
        memset(output, 0, nb_samples * sizeof(s16));
        break;
    }

    /* demodulation */
    switch(sm->state) {
    case SM_TEST_RING2:
        DTMF_demod(&sm->dtmf_rx, input, nb_samples);
        break;
    }

    /* write state transition */
    if (lm_debug) {
        if (sm->state != sm->debug_laststate) {
            sm->debug_laststate = sm->state;
            printf("%s: state: %s\n", sm->name, sm_states_str[sm->state]);
        }
    }

    switch(sm->state) {
        
        /* nothing to do (except waiting for a connection) */
    case SM_IDLE:
        break;

    case SM_GO_ONHOOK:
        {
            sm->hw->set_offhook(sm->hw_state, 0);
            sm->state = SM_IDLE;
        }
        break;

        /* calling modem */
    case SM_CALL:
        {
            sm->calling = 1;
            sm->hangup_request = 0;
            sm->state = SM_DTMF_DIAL;
            sm->hw->set_offhook(sm->hw_state, 1);
	    sm_set_timer(&sm->dtmf_timer, 2000);
        }
        break;

    case SM_DTMF_DIAL:
        if (sm->hangup_request) {
            sm->state = SM_GO_ONHOOK;
        } else if (sm_check_timer(&sm->dtmf_timer)) {
            DTMF_mod_state *p = &sm->dtmf_tx;

            sm->dtmf_ptr = 0;
            sm->dtmf_len = strlen(sm->call_num);

            p->get_digit = dtmf_get_digit;
            p->opaque = sm;
            p->dtmf_level = sm->lm_config->dtmf_level; 
            p->digit_length_ms = sm->lm_config->dtmf_digit_length; 
            p->digit_pause_ms = sm->lm_config->dtmf_pause_length;
            DTMF_mod_init(p);

            sm->state = SM_DTMF_DIAL_WAIT;
        }
        break;

    case SM_DTMF_DIAL_WAIT:
        {
            if (sm->dtmf_ptr >= sm->dtmf_len) {
                /* wait some time after dialing */
                sm_set_timer(&sm->dtmf_timer, 1000);
                sm->state = SM_DTMF_DIAL_WAIT1; 
            }
        }
        break;

    case SM_DTMF_DIAL_WAIT1:
        {
            if (sm_check_timer(&sm->dtmf_timer)) {
                /* start of V8 */
                V8_init(&sm->u.v8_state, 1, sm->lm_config->available_modulations);
                sm->state = SM_V8;
            }
        }
        break;

        /* answer modem */

        /* wait 2 sec until ring detected (for simulation only) */
        /* we try to recognize the DTMF value for fun :-) */
    case SM_TEST_RING:
        {
            sm->calling = 0;
            sm->dtmf_rx.opaque = sm;
            sm->dtmf_rx.put_digit = dtmf_put_digit;
            DTMF_demod_init(&sm->dtmf_rx);

            sm_set_timer(&sm->ring_timer, 5000); 
            sm->state = SM_TEST_RING2;
        }
        break;

    case SM_TEST_RING2:
        {
            if (sm_check_timer(&sm->ring_timer)) {
                sm->state = SM_RECEIVE;
            }
        }
        break;

        /* entry point to receive a connection */

    case SM_RECEIVE:
        {
            sm->calling = 0;
            sm->hangup_request = 0;
            sm->hw->set_offhook(sm->hw_state, 1);
            V8_init(&sm->u.v8_state, 0, sm->lm_config->available_modulations);
            sm->state = SM_V8;
        }
        break;

        /* V8 handling (both calling & receive) */
    case SM_V8:
        {
            int ret;
            if (sm->hangup_request) {
                printf("ddezde\n");
                sm->state = SM_GO_ONHOOK;
            } else {
                ret = V8_process(&sm->u.v8_state, output, input, nb_samples);
                switch(ret) {
                case V8_MOD_HANGUP:
                    sm->state = SM_GO_ONHOOK;
                    break;
                case V8_MOD_V21:
                    V21_init(&sm->u.v21_state, sm->calling,
                             serial_get_bit, serial_put_bit, sm);
                    sm->state = SM_V21;
                    break;
                case V8_MOD_V23:
                    V23_init(&sm->u.v23_state, sm->calling,
                             serial_get_bit, serial_put_bit, sm);
                    sm->state = SM_V23;
                    break;
                }
            }
        }
        break;

        /* V21 handling (both calling & receive) */
    case SM_V21:
        {
            int ret;
            ret = V21_process(&sm->u.v21_state, output, input, nb_samples);
            if (ret || sm->hangup_request)
                sm->state = SM_GO_ONHOOK;
        }
        break;

        /* V23 handling (both calling & receive) */
    case SM_V23:
        {
            int ret;
            ret = V23_process(&sm->u.v23_state, output, input, nb_samples);
            if (ret || sm->hangup_request)
                sm->state = SM_GO_ONHOOK;
        }
        break;
    }

    /* XXX: time hack */
    sm->time = sim_time + nb_samples;
}

/*
 * Send a dial request.
 */
int lm_start_dial(struct sm_state *s, int pulse, const char *number)
{
    if (s->state != SM_IDLE)
        return -1;
    s->pulse_dial = pulse;
    strcpy(s->call_num, number);
    s->state = SM_CALL;
    return 0;
}

/*
 * Send a receive request (for example to respond to a ring)
 */
int lm_start_receive(struct sm_state *s)
{
    if (s->state != SM_IDLE)
        return -1;
    s->state = SM_RECEIVE;
    return 0;
}

/* 
 * disconnect the modem
 */
int lm_hangup(struct sm_state *s)
{
    if (s->state == SM_IDLE)
        return -1;
    s->hangup_request = 1;
    return 0;
}

/* 
 * return a simplified state of the modem.
 */
enum lm_get_state_val lm_get_state(struct sm_state *s)
{
    switch(s->state) {
    case SM_IDLE:
        return LM_STATE_IDLE;
    case SM_V21:
    case SM_V23: 
        return LM_STATE_CONNECTED;
    default:
        return LM_STATE_CONNECTING;
    }
}


void lm_init(struct sm_state *sm, struct sm_hw_info *hw, const char *name)
{
    memset(sm, 0, sizeof(*sm));
    sm->hw = hw;
  
    /* pretty name */
    strcpy(sm->name, name);
    
    /* init fifos */
    sm_init_fifo(&sm->tx_fifo, sm->tx_fifo_buf, SM_FIFO_SIZE);
    sm_init_fifo(&sm->rx_fifo, sm->rx_fifo_buf, SM_FIFO_SIZE);
    
    /* we open the hardware driver */
    sm->hw_state = malloc(sizeof(struct lm_interface_state)); 
    sm->hw_state->sm = sm;
    sm->hw->open(sm->hw_state);
    
    sm->debug_laststate = -1;
    sm->state = SM_IDLE;

    /* config */
    sm->lm_config = &default_lm_config;
}


void sigusr1_debug(int dummy)
{
    printf( "<<<Debugging signal came>>>\n" );
    debug_state = 1;
}

void help(void)
{
    printf("Linux Generic Software Modem\n"
           "Copyright (c) 1999, 2000 Fabrice Bellard\n"
           "\n"
           "usage: lm [options]"
           "Test options:\n"
           "-v : verbose mode (additive)\n"
           "-s : modem test with the line simulator\n"
           "-m mod: test the modulation 'mod'. 'mod' can be:\n"
           "        v21, v23, v22, v34, v90\n"
           "\n"
           "Sound card support\n"
           "-t : use sound card as modem\n"
           "\n"
           "LTmodem support:\n"
           "-a : test answer mode with ltmodem\n"
           "-c command: use 'command' as modem driver\n"
           "-d number: dial 'number'\n"
           );
}

enum {
    MODE_NONE,
    MODE_LINESIM,
    MODE_V21TEST,
    MODE_V22TEST,
    MODE_V23TEST,
    MODE_V34TEST,
    MODE_V90TEST,
    MODE_LTMODEM_CALL,
    MODE_LTMODEM_ANSWER,
    MODE_SOUNDCARD,
};


extern char *modem_command, *dial_number;

int main(int argc, char **argv)
{
    int c, mode, calling;
    
    signal(SIGUSR1, sigusr1_debug);
    
    mode = MODE_NONE;
    calling = 0;

    for(;;) {
        c = getopt(argc, argv, "hvstrac:d:m:");
        if (c == -1) break;
        switch(c) {
        case 'v':
          lm_debug++;
          break;
	case 'm':
            if (!strcasecmp(optarg, "v21"))
                mode = MODE_V21TEST;
            else if (!strcasecmp(optarg, "v22"))
                mode = MODE_V22TEST;
            else if (!strcasecmp(optarg, "v23"))
                mode = MODE_V23TEST;
            else if (!strcasecmp(optarg, "v34"))
                mode = MODE_V34TEST;
            else if (!strcasecmp(optarg, "v90"))
                mode = MODE_V90TEST;
            else {
                fprintf(stderr, "incorrect modulation: '%s'\n", optarg);
                exit(1);
            }
	    break;
        case 's':
            mode = MODE_LINESIM;
            break;
        case 't':
            mode = MODE_SOUNDCARD;
            break;

            /* LTmodem options */
	case 'c':
	    modem_command = optarg;
	    break;
        case 'r':
            break;
	case 'a':	/* "Answer" mode */
            mode = MODE_LTMODEM_ANSWER;
	    break;
	case 'd':	/* Dial given number and exit mode */
            mode = MODE_LTMODEM_CALL;
	    dial_number = optarg;
	    break;
        default:
            help();
            exit(1);
        }
    }

    if (mode == MODE_NONE) {
        help();
        exit(1);
    }

    srandom(0); /* we want a deterministic test */
    dsp_init();
    V34_static_init();

    switch(mode) {
    case MODE_V21TEST:
        FSK_test(0);
        break;
    case MODE_V22TEST:
        V22_test();
        break;
    case MODE_V23TEST:
        FSK_test(1);
        break;
    case MODE_V34TEST:
        V34_test();
        break;
    case MODE_V90TEST:
        V90_test();
        break;
    case MODE_LINESIM:
        line_simulate();
        break;
    case MODE_LTMODEM_CALL:
    case MODE_LTMODEM_ANSWER:
        real_test(mode == MODE_LTMODEM_CALL);
        system("ltmodem -c");
        break;
    case MODE_SOUNDCARD:
        soundcard_modem();
        break;
    }

    return 0;
}





