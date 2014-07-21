/* 
 * generic FSK modulator & demodulator
 * 
 * Copyright (c) 1999,2000 Fabrice Bellard.
 *
 * This code is released under the GNU General Public License version
 * 2. Please read the file COPYING to know the exact terms of the
 * license.
 */
#include "lm.h"

void FSK_mod_init(FSK_mod_state *s)
{
    int b;

    s->omega[0] = (PHASE_BASE * s->f_lo) / s->sample_rate;
    s->omega[1] = (PHASE_BASE * s->f_hi) / s->sample_rate;
    s->baud_incr = (s->baud_rate * 0x10000) / s->sample_rate;
    s->phase = 0;
    s->baud_frac = 0;
    b = 0;
    s->current_bit = b;
}

void FSK_mod(FSK_mod_state *s, s16 *samples, unsigned int nb)
{
    int phase,baud_frac,b,i;

    phase = s->phase;
    baud_frac = s->baud_frac;
    b = s->current_bit;

    for(i=0;i<nb;i++) {
        baud_frac += s->baud_incr;
        if (baud_frac >= 0x10000) {
            baud_frac -= 0x10000;
            b = s->get_bit(s->opaque);
        }
        samples[i] = dsp_cos(phase);
        phase += s->omega[b];
    }
    s->phase = phase;
    s->baud_frac = baud_frac;
    s->current_bit = b;
}

void FSK_demod_init(FSK_demod_state *s)
{
    float phase;
    int i, a;

    s->baud_incr = (s->baud_rate * 0x10000) / s->sample_rate;
    s->baud_pll = 0;
    s->baud_pll_adj = s->baud_incr / 4;

    s->filter_size = s->sample_rate / s->baud_rate;

    memset(s->filter_buf, 0, sizeof(s->filter_buf));
    s->buf_ptr = s->filter_size;
    s->lastsample = 0;

    /* compute the filters */
    for(i=0;i<s->filter_size;i++) {
        phase = 2 * M_PI * s->f_lo * i / (float)s->sample_rate;
        s->filter_lo_i[i] = (int) (cos(phase) * COS_BASE);
        s->filter_lo_q[i] = (int) (sin(phase) * COS_BASE);

        phase = 2 * M_PI * s->f_hi * i / (float)s->sample_rate;
        s->filter_hi_i[i] = (int) (cos(phase) * COS_BASE);
        s->filter_hi_q[i] = (int) (sin(phase) * COS_BASE);
    }

    s->shift = -2;
    a = s->filter_size;
    while (a != 0) {
        s->shift++;
        a /= 2;
    }
    printf("shift=%d\n", s->shift);
}

void FSK_demod(FSK_demod_state *s, const s16 *samples, unsigned int nb)
{
    int buf_ptr, corr, newsample, baud_pll, i;
    int sum;

    baud_pll = s->baud_pll;
    buf_ptr = s->buf_ptr;

    for(i=0;i<nb;i++) {
        /* add a new sample in the demodulation filter */
        s->filter_buf[buf_ptr++] = samples[i] >> s->shift;
        if (buf_ptr == FSK_FILTER_BUF_SIZE) {
            memmove(s->filter_buf, 
                    s->filter_buf + FSK_FILTER_BUF_SIZE - s->filter_size, 
                    s->filter_size * sizeof(s16));
            buf_ptr = s->filter_size;
        }
        
        /* non coherent FSK demodulation - not optimal, but it seems
           very difficult to do another way */
        corr = dsp_dot_prod(s->filter_buf + buf_ptr - s->filter_size,
                            s->filter_hi_i, s->filter_size, 0);
        corr = corr >> COS_BITS;
        sum = corr * corr;
        
        corr = dsp_dot_prod(s->filter_buf + buf_ptr - s->filter_size,
                            s->filter_hi_q, s->filter_size, 0);
        corr = corr >> COS_BITS;
        sum += corr * corr;

        corr = dsp_dot_prod(s->filter_buf + buf_ptr - s->filter_size,
                            s->filter_lo_i, s->filter_size, 0);
        corr = corr >> COS_BITS;
        sum -= corr * corr;
        
        corr = dsp_dot_prod(s->filter_buf + buf_ptr - s->filter_size,
                            s->filter_lo_q, s->filter_size, 0);
        corr = corr >> COS_BITS;
        sum -= corr * corr;

        lm_dump_sample(CHANNEL_SAMPLESYNC, sum / 32768.0);
        //        printf("sum=%0.3f\n", sum / 65536.0);
        newsample = sum > 0;

        /* baud PLL synchronisation : when we see a transition of
           frequency, we tend to modify the baud phase so that it is
           in the middle of two bits */
        if (s->lastsample != newsample) {
            s->lastsample = newsample;
            //            printf("pll=%0.3f (%d)\n", baud_pll / 65536.0, newsample);
            if (baud_pll < 0x8000)
                baud_pll += s->baud_pll_adj;
            else
                baud_pll -= s->baud_pll_adj;
        }
        
        baud_pll += s->baud_incr;

        if (baud_pll >= 0x10000) {
            baud_pll -= 0x10000;
            //            printf("baud=%f (%d)\n", baud_pll / 65536.0, s->lastsample);
            s->put_bit(s->opaque, s->lastsample);
        }
    }

    s->baud_pll = baud_pll;
    s->buf_ptr = buf_ptr;
}

/* test for FSK using V21 or V23 */

#define NB_SAMPLES 40

#define MAXDELAY 32

static int tx_bits[MAXDELAY], tx_ptr = 0, rx_ptr = 0;
static int tx_blank = 32;

/* transmit random bits with a sync header (31 ones, 1 zero) */
static int test_get_bit(void *opaque)
{
    int bit;

    if (tx_blank != 0) {
        /* send 1 at the beginning for synchronization */
        bit = (tx_blank > 1);
        tx_blank--;
    } else {
        bit = random() % 2;
        tx_bits[tx_ptr] = bit;
        if (++tx_ptr == MAXDELAY)
            tx_ptr = 0;
    }
    return bit;
}

static int nb_bits = 0, errors = 0, sync_count = 0, got_sync = 0;

static void test_put_bit(void *opaque, int bit)
{
    int tbit;
    
    if (!got_sync) {
        
        if (bit) {
            sync_count++;
        } else {
            if (sync_count > 16)
                got_sync = 1;
            sync_count = 0;
        }
    } else {
        tbit = tx_bits[rx_ptr];
        if (++rx_ptr == MAXDELAY)
            rx_ptr = 0;
        if (bit != tbit) {
            errors++;
        }
        nb_bits++;
    }
}

void FSK_test(int do_v23)
{
    FSK_mod_state tx;
    FSK_demod_state rx;
    int err, calling;
    struct LineModelState *line_state;
    s16 buf[NB_SAMPLES];
    s16 buf1[NB_SAMPLES];
    s16 buf2[NB_SAMPLES];
    s16 buf3[NB_SAMPLES];
    FILE *f1;
    
    err = lm_display_init();
    if (err < 0) {
        fprintf(stderr, "Could not init X display\n");
        exit(1);
    }

    line_state = line_model_init();

    f1 = fopen("cal.sw", "wb");
    if (f1 == NULL) {
        perror("cal.sw");
        exit(1);
    }

    calling = 0;
    if (do_v23) {
        V23_mod_init(&tx, calling, test_get_bit, NULL);
        V23_demod_init(&rx, 1 - calling, test_put_bit, NULL);
    } else {
        V21_mod_init(&tx, calling, test_get_bit, NULL);
        V21_demod_init(&rx, 1 - calling, test_put_bit, NULL);
    }

    nb_bits = 0;
    errors = 0;
    for(;;) {
        if (lm_display_poll_event())
            break;
        
        FSK_mod(&tx, buf, NB_SAMPLES);
        memset(buf3, 0, sizeof(buf3));

        line_model(line_state, buf1, buf, buf2, buf3, NB_SAMPLES);

        fwrite(buf, 1, NB_SAMPLES * 2, f1);
        
        FSK_demod(&rx, buf1, NB_SAMPLES);
    }

    fclose(f1);

    printf("errors=%d nb_bits=%d Pe=%f\n", 
           errors, nb_bits, (float) errors / (float)nb_bits);
}
