/* 
 * V8 protocol handler
 * 
 * Copyright (c) 1999,2000 Fabrice Bellard.
 *
 * This code is released under the GNU General Public License version
 * 2. Please read the file COPYING to know the exact terms of the
 * license.
 * 
 * This implementation is totally clean room. It was written by
 * reading the ITU specification and by using basic signal processing
 * knowledge.  
 */
#include "lm.h"

static void V8_mod_init(V8_mod_state *s)
{
    s->sample_rate = V8_SAMPLE_RATE;

    /* ANSam tone: 2100 Hz, amplitude modulated at 15 Hz, with phase
       reversal every 450 ms */
    s->phase = 0;
    s->phase_incr = (int) (PHASE_BASE * 2100.0 / s->sample_rate);
    s->mod_phase = 0;
    s->mod_phase_incr = (int) (PHASE_BASE * 15.0 / s->sample_rate);
    s->phase_reverse_samples = (int) (s->sample_rate * 0.450);
    s->phase_reverse_left = 0;
    /* XXX: incorrect power */
    s->amp = (int) (pow(10, s->tone_level / 20.0) * 32768.0);
}

static void V8_mod(V8_mod_state *s, s16 *samples, unsigned int nb)
{
    int amp,i;

    for(i=0;i<nb;i++) {
        /* handle phase reversal every 450 ms */
        if (s->phase_reverse_left == 0) {
            s->phase_reverse_left = s->phase_reverse_samples;
            s->phase += PHASE_BASE / 2;
        }

        amp = (dsp_cos(s->mod_phase) * (int)(0.2 * COS_BASE)) >> COS_BITS;
        amp += COS_BASE; /* between 0.8 and 1.2 */
        samples[i] = (amp * dsp_cos(s->phase)) >> COS_BITS;
        s->mod_phase += s->mod_phase_incr;
        s->phase += s->phase_incr;
    }
}

/* Recognize the V8 ANSam tone. Some other tones (in particular V21
   tone) may be added later. We compute the DFT for every interesting
   frequency and do a threshold with the power. Not the best method,
   but easy to implement and quite reliable. */

static void V8_demod_init(V8_demod_state *s)
{
    int i;

    for(i=0;i<V8_N;i++) {
        s->cos_tab[i] = (int) (cos( 2 * M_PI * i / V8_N) * COS_BASE);
        s->sin_tab[i] = (int) (sin( 2 * M_PI * i / V8_N) * COS_BASE);
    }

    s->buf_ptr = 0;
    s->v8_ANSam_detected = 0;
}

static void V8_demod(V8_demod_state *s, const s16 *samples, unsigned int nb)
{
    int i, p0, p1;

    for(i=0;i<nb;i++) {
        s->buf[s->buf_ptr++] = samples[i];
        if (s->buf_ptr >= V8_N) {
            s->buf_ptr = 0;
            dsp_sar_tab(s->buf, V8_N, 8);
            p0 = dsp_norm2(s->buf, V8_N, 0);
            p1 = compute_DFT(s->cos_tab, s->sin_tab, s->buf, DFT_COEF_2100, V8_N);
            /* XXX: this test is incorrect (not homogenous) */
            if ((p0 > 1000) && (p1 > (5*p0))) {
                /* V8 tone recognized */
                s->v8_ANSam_detected = 1;
            }
        }
    }
}

/* V8 stream decoding */
static void ci_decode(V8State *s)
{
    int data = s->rx_data[0];
    if (data == 0x83) {
        printf("CI: data call\n");
    } 
}

/* CM or JM decoding */
static void cm_decode(V8State *s)
{
    u8 *p;
    int c;

    if (s->got_cm)
        return;

    if (s->cm_count > 0) {
        /* we must receive two identical CM sequences */
        if (s->cm_count == s->rx_data_ptr &&
            !memcmp(s->cm_data, s->rx_data, s->rx_data_ptr)) {
            /* got CM !! */
            s->got_cm = 1;
            /* decode it */
            /* XXX: this decoding is sufficient for modulations, but
               not exhaustive */
            s->decoded_modulations = 0;
            p = s->cm_data;
            /* zero is used to indicate the end */
            s->cm_data[s->cm_count] = 0;

            c = *p++;
            /* call function */
            if ((c & 0xf8) != 0x80) 
                return;
            if (c != V8_CALL_FUNC_DATA)
                return;

            /* modulation */
            c = *p++;
            if ((c & 0xf8) != V8_MODN0) 
                return;

            if (c & V8_MODN0_V90)
                s->decoded_modulations |= V8_MOD_V90;
            if (c & V8_MODN0_V34)
                s->decoded_modulations |= V8_MOD_V34;

            c = *p++;
            if ((c & 0x1c) == V8_EXT) {
                /* ignored */
                c = *p++;
                if ((c & 0x1c) == V8_EXT) {
                    if (c & V8_MODN2_V23)
                        s->decoded_modulations |= V8_MOD_V23;
                    if (c & V8_MODN2_V21)
                        s->decoded_modulations |= V8_MOD_V21;
                    /* skip other extensions */
                    do {
                        c = *p++;
                    } while ((c & 0x1c) == V8_EXT);
                }
            }
            return;
        }
    }
            
    /* save the current CM sequence */
    s->cm_count = s->rx_data_ptr;
    memcpy(s->cm_data, s->rx_data, s->rx_data_ptr);
}

static void put_bit(void *opaque, int bit)
{
    V8State *s = opaque;
    int new_state, i;

    /* wait ten ones & synchro */
    s->bit_sync = ((s->bit_sync << 1) | bit) & ((1 << 20) - 1);
    if (s->bit_sync == ((V8_TEN_ONES << 10) | V8_CI_SYNC)) {
        new_state = V8_CI_SYNC;
        goto data_init;
    } else if (s->bit_sync == ((V8_TEN_ONES << 10) | V8_CM_SYNC)) {
        new_state = V8_CM_SYNC;
    data_init:
        /* debug */
        if (s->data_state == V8_CI_SYNC) {
            printf("CI: ");
        } else if (s->data_state == V8_CM_SYNC) {
            if (s->calling)
                printf("JM: ");
            else
                printf("CM: ");
        }
        for(i=0;i<s->rx_data_ptr;i++) printf(" %02x", s->rx_data[i]);
        printf("\n");
        
        /* decode previous sequence */
        switch(s->data_state) {
        case V8_CI_SYNC:
            ci_decode(s);
            break;
        case V8_CM_SYNC:
            cm_decode(s);
            break;
        }
        s->data_state = new_state;
        s->bit_buf = 0;
        s->bit_cnt = 0;
        s->rx_data_ptr = 0;
    }
    
    /* parse octets with 1 bit start, 1 bit stop */
    if (s->data_state) {
        s->bit_buf = ((s->bit_buf << 1) | bit) & ((1 << 10) - 1);
        s->bit_cnt++;
        /* start, stop ? */
        if ((s->bit_buf & 0x201) == 0x001 && s->bit_cnt >= 10) {
            int data;
            /* store the available data */
            data = (s->bit_buf >> 1) & 0xff;
            printf("got data: %d %02x\n", s->data_state, data);
            /* CJ detection */
            if (data == 0) {
                if (++s->data_zero_count == 3) {
                    s->got_cj = 1;
                    printf("got CJ\n");
                }
            } else {
                s->data_zero_count = 0;
            }

            if (s->rx_data_ptr < (sizeof(s->rx_data)-1)) {
                s->rx_data[s->rx_data_ptr++] = data;
            }
            
            s->bit_cnt = 0;
        }
    }
}

static void v8_decode_init(V8State *s)
{
    V21_demod_init(&s->v21_rx, s->calling, put_bit, s);
    s->data_state = 0;
    s->bit_sync = 0;
    s->cm_count = 0;
    s->got_cm = 0;

    s->got_cj = 0;
    s->data_zero_count = 0;
    s->rx_data_ptr = 0;
}


static int get_bit(void *opaque)
{
    V8State *s = opaque;
    int bit;

    bit = sm_get_bit(&s->tx_fifo);
    if (bit < 0)
        bit = 1;
    return bit;
}

static void v8_put_byte(V8State *s, int data)
{
    /* insert start & stop bits */
    sm_put_bits(&s->tx_fifo, ((data & 0xff) << 1) | 1, 10);
}


void V8_init(V8State *sm, int calling, int mod_mask)
{
    sm->debug_laststate = -1;
    sm->calling = calling;
    if (sm->calling) {
        sm->state = V8_WAIT_1SECOND;
        sm_set_timer(&sm->v8_start_timer, 1000);
    } else {
        /* wait 200 ms */
        sm_set_timer(&sm->v8_connect_timer, 200);
        sm->state = V8_WAIT;
    }
    sm_init_fifo(&sm->rx_fifo, sm->rx_buf, sizeof(sm->rx_buf));
    sm_init_fifo(&sm->tx_fifo, sm->tx_buf, sizeof(sm->tx_buf));
    sm->modulation_mask = mod_mask;
}

/* send CM or JM */
static void cm_send(V8State *s, int mod_mask)
{
    int val;

    sm_put_bits(&s->tx_fifo, V8_TEN_ONES, 10);
    sm_put_bits(&s->tx_fifo, V8_CM_SYNC, 10);
    
    /* data call */
    v8_put_byte(s, V8_CALL_FUNC_DATA);
    
    /* supported modulations */
    val = V8_MODN0;
    if (mod_mask & V8_MOD_V90)
        val |= V8_MODN0_V90;
    if (mod_mask & V8_MOD_V34)
        val |= V8_MODN0_V34;
    v8_put_byte(s, val);
    v8_put_byte(s, V8_EXT);
    val = V8_EXT;
    if (mod_mask & V8_MOD_V23)
        val |= V8_MODN2_V23;
    if (mod_mask & V8_MOD_V21)
        val |= V8_MODN2_V21;
    v8_put_byte(s, val);
    
    /* for now, no LAPM */
    //v8_put_byte(s, V8_DATA_LAPM);
    
    /* We are not on celullar connection. What is that,
       anyway? GSM?  Don't send this - we don't what it is
       for, anyway. */
    //v8_put_byte(s, V8_DATA_NOCELULAR);
}

/* selection the modulation according to V8 priority from the bits in 'mask' */
static int select_modulation(int mask)
{
    int val;

    val = V8_MOD_HANGUP;
    /* use modulations in this order */
    if (mask & V8_MOD_V21)
        val = V8_MOD_V21;
    if (mask & V8_MOD_V23)
        val = V8_MOD_V23;
    if (mask & V8_MOD_V34)
        val = V8_MOD_V34;
    if (mask & V8_MOD_V90)
        val = V8_MOD_V90;
    return val;
}


/* V8 protocol handler */
int V8_process(V8State *s, s16 *output, s16 *input, int nb_samples)
{
    int ret = 0;

    /* modulation part */
    switch (s->state) {
    case V8_CI_SEND:
    case V8_CM_SEND:
    case V8_JM_SEND:
    case V8_CJ_SEND:
        /* modulate with V21 */
        FSK_mod(&s->v21_tx, output, nb_samples);
        break;

    case V8_CM_WAIT:
        /* send ANSam modulation */
        V8_mod(&s->v8_tx, output, nb_samples);
        break;

    default:
        /* output nothing */ 
        memset(output, 0, nb_samples * sizeof(s16));
        break;
    }

    /* demodulation part */
    switch (s->state) {
    case V8_CI:
    case V8_CI_OFF:
    case V8_CI_SEND:
        /* detect ANSam */
        V8_demod(&s->v8_rx, input, nb_samples);
        break;
        
    case V8_CM_WAIT:
    case V8_CM_SEND:
    case V8_JM_SEND:
        /* V21 receive */
        FSK_demod(&s->v21_rx, input, nb_samples);
        break;
        
    default:
        break;
    }

    /* state machine */

    if (lm_debug) {
        if (s->state != s->debug_laststate) {
            s->debug_laststate = s->state;
            printf("%s: V8: state: %s\n", 
                   s->calling ? "cal" : "ans", sm_states_str[s->state]);
        }
    }

    switch(s->state) {

    case V8_WAIT_1SECOND:
        {
            /* wait 1 second before sending the first CI packet */
            if (sm_check_timer(&s->v8_start_timer)) {
                s->state = V8_CI;
                s->v8_ci_count = 0;
                V8_demod_init(&s->v8_rx); /* init ANSam detection */
                V21_mod_init(&s->v21_tx, 1, get_bit, s);
            }
        }
        break;

        /* send the CI packets */
    case V8_CI:
        {
            int i;
            
            /* send 4 CI packets (at least 3 must be sent) */
            for(i=0;i<4;i++) {
                sm_put_bits(&s->tx_fifo, V8_TEN_ONES, 10);
                sm_put_bits(&s->tx_fifo, V8_CI_SYNC, 10);
                v8_put_byte(s, V8_CALL_FUNC_DATA);
            }
            s->state = V8_CI_SEND;
        }
        break;

    case V8_CI_SEND:
        {
            if (sm_size(&s->tx_fifo) == 0) {
                s->state = V8_CI_OFF;
                sm_set_timer(&s->v8_ci_timer, 500); /* 0.5 s off */
            }
        }
        break;
 
    case V8_CI_OFF:
        {
            /* check if an ANSam tone is detected */
            if (s->v8_rx.v8_ANSam_detected) {
		sm_set_timer(&s->v8_ci_timer, V8_TE);
                s->state = V8_GOT_ANSAM;
            } else if (sm_check_timer(&s->v8_ci_timer)) {
                if (++s->v8_ci_count == V8_MAX_CI_SEQ) {
                    ret = V8_MOD_HANGUP;
                }
                else {
                    s->state = V8_CI;
                }
            }
        }
        break;

    case V8_GOT_ANSAM:
    	{
	    if (sm_check_timer(&s->v8_ci_timer)) {
                v8_decode_init(s);
		s->state = V8_CM_SEND;
	    }
	}
	break;

    case V8_CM_SEND:
    	{
            if (s->got_cm) {
                /* if JM detected, we send CJ & wait for 75 ms before exiting V8 */

                s->selected_mod_mask = s->modulation_mask & s->decoded_modulations;
                s->selected_modulation = select_modulation(s->selected_mod_mask);

                /* flush tx queue */
                sm_flush(&s->tx_fifo);
                v8_put_byte(s, 0);
                v8_put_byte(s, 0);
                v8_put_byte(s, 0);
                /* a few more bytes to fill the time */
                v8_put_byte(s, 0);
                v8_put_byte(s, 0);
                v8_put_byte(s, 0);
                v8_put_byte(s, 0);
                v8_put_byte(s, 0);
                v8_put_byte(s, 0);
                s->state = V8_CJ_SEND;
            } else if (sm_size(&s->tx_fifo) == 0) {
                /* send CM */
                cm_send(s, s->modulation_mask);
            }
	}
        break;

    case V8_CJ_SEND:
        /* wait until CJ is sent */
        if (sm_size(&s->tx_fifo) == 0) {
            sm_set_timer(&s->v8_start_timer, 75);
            s->state = V8_SIGC;
        }
        break;

    case V8_SIGC:
        if (sm_check_timer(&s->v8_start_timer)) {
            /* it's OK, let's start the wanted modulation */
            ret = s->selected_modulation;
        }
        break;


        /* V8 answer */

    case V8_WAIT:
        {
            if (sm_check_timer(&s->v8_connect_timer)) {
                /* send the ANSam tone */
                s->v8_tx.tone_level = -3; /* XXX: fix it */
                V8_mod_init(&s->v8_tx);
                
                /* prepare V21 to receive CI or CM */
                v8_decode_init(s);

                /* wait at most 5 seconds */
                sm_set_timer(&s->v8_connect_timer, 5000);
                s->state = V8_CM_WAIT;
            }
        }
        break;

    case V8_CM_WAIT:
        {
            if (sm_check_timer(&s->v8_connect_timer)) {
                /* timeout */
                ret = V8_MOD_HANGUP;
            } else {
                if (s->got_cm) {
                    /* stop sending ANSam & send JM */
                    V21_mod_init(&s->v21_tx, 0, get_bit, s);
                    /* timeout for JM */
                    sm_set_timer(&s->v8_connect_timer, 5000); 
                    s->state = V8_JM_SEND;
                    s->selected_mod_mask = s->modulation_mask & s->decoded_modulations;
                    s->selected_modulation = select_modulation(s->selected_mod_mask);
                }
            }
        }
        break;

    case V8_JM_SEND:
        {
            if (sm_check_timer(&s->v8_connect_timer)) {
                /* timeout */
                ret = V8_MOD_HANGUP;
            } else if (s->got_cj) {
                /* stop sending JM & wait 75 ms */
                sm_set_timer(&s->v8_connect_timer, 75); 
                s->state = V8_SIGA;
            } else if (sm_size(&s->tx_fifo) == 0) {
                /* Send JM */
                cm_send(s, s->selected_mod_mask);
            }
        }
        break;

    case V8_SIGA:
        if (sm_check_timer(&s->v8_connect_timer)) {
            ret = s->selected_modulation;
        }
        break;
    }
    return ret;
}
