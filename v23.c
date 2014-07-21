/* 
 * generic V23 modulator & demodulator
 * 
 * Copyright (c) 1999 Fabrice Bellard.
 *
 * This code is released under the GNU General Public License version
 * 2. Please read the file COPYING to know the exact terms of the
 * license.
 */
#include <stdlib.h>
#include <stdio.h>

#include "lm.h"
#include "fsk.h"

#define SAMPLE_RATE 8000

void V23_mod_init(FSK_mod_state *s, int calling, get_bit_func get_bit, void *opaque)
{
    if (calling) {
        /* 75 bauds */
        s->f_lo = 390;
        s->f_hi = 450;
        s->baud_rate = 75;
    } else {
        /* 1200 bauds */
        s->f_lo = 1300;
        s->f_hi = 2100;
        s->baud_rate = 1200;
    }
    s->sample_rate = SAMPLE_RATE;
    s->get_bit = get_bit;
    s->opaque = opaque;

    FSK_mod_init(s);
}


void V23_demod_init(FSK_demod_state *s, int calling, put_bit_func put_bit, void *opaque)
{
    if (!calling) {
        /* 75 bauds */
        s->f_lo = 390;
        s->f_hi = 450;
        s->baud_rate = 75;
    } else {
        /* 1200 bauds */
        s->f_lo = 1300;
        s->f_hi = 2100;
        s->baud_rate = 1200;
    }
    s->sample_rate = SAMPLE_RATE;
    s->put_bit = put_bit;
    s->opaque = opaque;
 
    FSK_demod_init(s);
}


void V23_init(V23State *s, int calling, 
              get_bit_func get_bit, put_bit_func put_bit, void *opaque)
{
    V23_mod_init(&s->tx, calling, get_bit, opaque);
    V23_demod_init(&s->rx, calling, put_bit, opaque);
}

int V23_process(V23State *s, s16 *output, s16 *input, int nb_samples)
{
    FSK_mod(&s->tx, output, nb_samples);
    FSK_demod(&s->rx, input, nb_samples);
    return 0;
}
