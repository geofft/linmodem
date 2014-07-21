/*
 * DTMF demodulator, inspirated from the multimon project of Thomas
 * Sailer (sailer@ife.ee.ethz.ch, hb9jnx@hb9w.che.eu). It is different
 * in the sense that it uses a true block based DFT estimator.
 * 
 * Copyright (c) 1999 Fabrice Bellard.
 *
 * This code is released under the GNU General Public License version
 * 2. Please read the file COPYING to know the exact terms of the
 * license.
 */

#include "lm.h"

/*
 * DTMF frequencies
 *
 *      1209 1336 1477 1633
 *  697   1    2    3    A
 *  770   4    5    6    B
 *  852   7    8    9    C
 *  941   *    0    #    D
 *  */

static const char *dtmf_transl = "123A456B789C*0#D";

static int dtmf_freq[] = {
    1209, 1336, 1477, 1633,
    697, 770, 852, 941,
};

#define SAMPLE_RATE 8000

/* Each DTMF digit is estimed on N samples by estimating the DFT of
   the signal. It is quite reliable, but the frequency resolution is
   not accurate enough to meet the very strict ITU requirements. */

/* DTMF modulation */

void DTMF_mod_init(DTMF_mod_state *s)
{
    s->t1 = s->t2 = 0;
    s->omega1 = 0;
    s->samples_left = 0;
}

/* compute parameters for a new digit */
static void compute_params(DTMF_mod_state *s)
{
    const char *p;
    int v,digit,f1,f2;

    /* if we were playing a digit, we make a pause */
    if (s->omega1 != 0)
        goto nodigit;

    digit = s->get_digit(s->opaque);
    if (digit == -1)
        goto nodigit;
    p = dtmf_transl;
    while (*p != digit && *p) p++;
    if (*p) {
        v = p - dtmf_transl;
        f1 = dtmf_freq[v & 3];
        f2 = dtmf_freq[4 + (v >> 2)];
        s->omega1 = (PHASE_BASE * f1) / SAMPLE_RATE;
        s->omega2 = (PHASE_BASE * f2) / SAMPLE_RATE;
        /* amplitude */
        s->amp = (int) (pow(10, s->dtmf_level / 20.0) * 32768.0);
        /* number of samples to play */
        s->samples_left = (s->digit_length_ms * SAMPLE_RATE) / 1000;
    } else {
    nodigit:
        s->omega1 = 0;
        s->samples_left = (s->digit_pause_ms * SAMPLE_RATE) / 1000;
    }
}

void DTMF_mod(DTMF_mod_state *s, s16 *samples, unsigned int nb)
{
    int len, t1, t2, amp, i;

    while (nb > 0) {
        if (s->samples_left == 0) {
            compute_params(s);
        }

        len = nb;
        if (len > s->samples_left)
            len = s->samples_left;
        
        if (s->omega1 != 0) {
            t1 = s->t1;
            t2 = s->t2;
            amp = s->amp;
            for(i=0;i<len;i++) {
                int v;
                v = ((dsp_cos(t1) + dsp_cos(t2)) * amp) >> COS_BITS;
                samples[i] = v;
                t1 += s->omega1;
                t2 += s->omega2;
            }
            s->t1 = t1;
            s->t2 = t2;
        } else {
            memset(samples, 0, nb * 2); /* silence between digits */
        }

        nb -= len;
        samples += len;
        s->samples_left -= len;
    }
}

/* DTMF demodulation */

void DTMF_demod_init(DTMF_demod_state *s)
{
    int i;
    for(i=0;i<DTMF_N;i++) {
        s->cos_tab[i] = (int) (cos( 2 * M_PI * i / DTMF_N) * COS_BASE);
        s->sin_tab[i] = (int) (sin( 2 * M_PI * i / DTMF_N) * COS_BASE);
    }
    for(i=0;i<8;i++) {
        float v;
        v = (float)dtmf_freq[i] / (float)SAMPLE_RATE * (float)DTMF_N;
        s->dtmf_coefs[i] = (int)rint(v);
    }

    s->buf_ptr = 0;
    s->last_digit = 0;
}

int find_max(int *val,int n)
{
    int i,j,max;
    
    j = 0;
    max = val[0];
    for(i=1;i<n;i++) {
        if (val[i] > max) {
            j = i;
            max = val[i];
        }
    }
    return j;
}

void DTMF_demod(DTMF_demod_state *s, 
                const s16 *samples, unsigned int nb)
{
    int i, j, power[8], power0, v1, v2, digit, bits, p1, p2, p0;

    for(j=0;j<nb;j++) {
        s->buf[s->buf_ptr++] = samples[j];

        if (s->buf_ptr >= DTMF_N) {
            /* decision based on one block */
            bits = dsp_max_bits(s->buf, DTMF_N);
            if (bits < 8) bits = 8;
            dsp_sar_tab(s->buf, DTMF_N, bits - 8);
            
            power0 = dsp_norm2(s->buf, DTMF_N, 0);
            
            for(i=0;i<8;i++) {
                power[i] = compute_DFT(s->cos_tab, s->sin_tab, s->buf, 
                                       s->dtmf_coefs[i], DTMF_N);
            }
            
            v1 = find_max(power, 4);
            v2 = find_max(power + 4, 4);
            p0 = (int)(power0 * 0.3);
            p1 = (2 * power[v1]) / DTMF_N;
            p2 = (2 * power[v2 + 4]) / DTMF_N;
#if 0            
            printf("%d %d %d %f %f\n",p0, v1, v2, (float) p1 / p0,(float) p2 / p0);
#endif
            if (p1 > p0 && p2 > p0) 
                digit = dtmf_transl[v1 | (v2 << 2)];
            else
                digit = 0;
            
            if (digit != s->last_digit) {
                if (digit != 0) {
                    s->put_digit(s->opaque, digit);
                }
                s->last_digit = digit;
            }
            s->buf_ptr = 0;
        }
    }
}
