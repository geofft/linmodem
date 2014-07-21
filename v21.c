/* 
 * Implementation of the V21 modulation/demodulation
 * 
 * Copyright (c) 1999,2000 Fabrice Bellard.
 *
 * This code is released under the GNU General Public License version
 * 2. Please read the file COPYING to know the exact terms of the
 * license.
 */
#include "lm.h"

#define SAMPLE_RATE 8000

void V21_mod_init(FSK_mod_state *s, int calling, get_bit_func get_bit, void *opaque)
{
    if (calling) {
        /* channel 1 */
        s->f_lo = 1080 + 100;
        s->f_hi = 1080 - 100;
    } else {
        /* channel 2 */
        s->f_lo = 1750 + 100;
        s->f_hi = 1750 - 100;
    }
    s->baud_rate = 300;
    s->sample_rate = SAMPLE_RATE;
    s->get_bit = get_bit;
    s->opaque = opaque;

    FSK_mod_init(s);
}


void V21_demod_init(FSK_demod_state *s, int calling, put_bit_func put_bit, void *opaque)
{
    if (!calling) {
        /* channel 1 */
        s->f_lo = 1080 + 100;
        s->f_hi = 1080 - 100;
    } else {
        /* channel 2 */
        s->f_lo = 1750 + 100;
        s->f_hi = 1750 - 100;
    }
    s->baud_rate = 300;
    s->sample_rate = SAMPLE_RATE;
    s->put_bit = put_bit;
    s->opaque = opaque;
    FSK_demod_init(s);
}

void V21_init(V21State *s, int calling, 
              get_bit_func get_bit, put_bit_func put_bit, void *opaque)
{
    V21_mod_init(&s->tx, calling, get_bit, opaque);
    V21_demod_init(&s->rx, calling, put_bit, opaque);
}

int V21_process(V21State *s, s16 *output, s16 *input, int nb_samples)
{
    /* XXX: handle disconnect detection by looking at the power */
    FSK_mod(&s->tx, output, nb_samples);
    FSK_demod(&s->rx, input, nb_samples);
    return 0;
}
