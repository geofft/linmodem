/* 
 * V22 modulator & demodulator
 * 
 * Copyright (c) 1999,2000 Fabrice Bellard.
 *
 * This code is released under the GNU General Public License version
 * 2. Please read the file COPYING to know the exact terms of the
 * license.
 */
#include "lm.h"

/*
 * This code is also used by the V34 phase 2 at 600 bits/s. V22bis
 * 2400 bps is implemented in the modulation but not in the
 * demodulation.  
 */

void V22_mod_init(V22ModState *s)
{
    s->baud_phase = 0;
    s->baud_num = 3;
    s->baud_denom = 40;
    s->tx_filter_wsize = V22_TX_FILTER_SIZE / s->baud_denom;
    memset(s->tx_buf, 0, sizeof(s->tx_buf));
    s->tx_outbuf_ptr = 0;
    s->carrier_phase = 0;
    s->carrier2_phase = 0;
    s->Z = 0;
    
    if (s->calling) {
        /* call modem DPSK: 600 bps, carrier at 1200 Hz, 0 db */
        s->carrier_incr = (PHASE_BASE * 1200.0) / V34_SAMPLE_RATE;
    } else {
        /* answer modem DPSK: 600 bps, carrier at 2400 Hz, -1 db, 
           guard tone at 1800 Hz, -7db */
        s->carrier_incr = (PHASE_BASE * 2400.0) / V34_SAMPLE_RATE;
        s->carrier2_incr = (PHASE_BASE * 1800.0) / V34_SAMPLE_RATE;
    }
}

static void V22_mod_baseband(V22ModState *s, s16 *x_ptr, s16 *y_ptr)
{
    int x, y, x1, y1, b1, b2;

    /* handle each kind of modulation */
    switch(s->mod_type) {
    default:
    case V34_MOD_600:
        b1 = s->get_bit(s->opaque);
        /* rotation by 0 or 180 degrees */
        s->Z = s->Z ^ (b1 << 1);
        x1 = 0x2000;
        y1 = 0x2000;
        break;
    case V22_MOD_600:
        b1 = s->get_bit(s->opaque);
        /* rotation by 90 or 270 degrees */
        s->Z = (s->Z + ((b1 << 1) | 1)) & 3;
        x1 = 0x2000;
        y1 = 0x2000;
        break;
    case V22_MOD_1200:
        b1 = s->get_bit(s->opaque);
        b2 = s->get_bit(s->opaque);
        b2 ^= (1 - b1);
        s->Z = (s->Z + ((b1 << 1) | b2)) & 3;
        x1 = 0x2000;
        y1 = 0x2000;
        break;
    case V22_MOD_2400:
        /* quadrant selection */
        b1 = s->get_bit(s->opaque);
        b2 = s->get_bit(s->opaque);
        b2 ^= (1 - b1);
        s->Z = (s->Z + ((b1 << 1) | b2)) & 3;
        /* 4 positions inside the quadrant */
        b1 = s->get_bit(s->opaque);
        b2 = s->get_bit(s->opaque);
        /* XXX: normalize */
        x1 = 0x1000;
        if (b2) 
            x1 += 0x2000;
        y1 = 0x1000;
        if (b1)
            y1 += 0x2000;
        break;
    }
    
    /* rotate counter clockwise */
    switch(s->Z) {
    case 0:
        x = x1;
        y = y1;
        break;
    case 1:
        x = -y1;
        y = x1;
        break;
    case 2:
        x = -x1;
        y = -y1;
        break;
    default:
    case 3:
        x = y1;
        y = -x1;
        break;
    }
    *x_ptr = x;
    *y_ptr = y;
}

void V22_mod(V22ModState *s, s16 *samples, unsigned int nb)
{
    int i, j, k, val, si, sq, ph;
    
    for(i=0;i<nb;i++) {

        /* apply the spectrum shaping filter */
        ph = s->baud_phase;
        si = sq = 0;
        for(j=0;j<s->tx_filter_wsize;j++) {
            k = (s->tx_outbuf_ptr - j - 1) & 
                (V22_TX_BUF_SIZE - 1);
            si += s->tx_buf[k][0] * v22_tx_filter[ph];
            sq += s->tx_buf[k][1] * v22_tx_filter[ph];
            ph += s->baud_denom;
        }
        si = si >> 14;
        sq = sq >> 14;

        /* get next baseband symbol ? */
        s->baud_phase += s->baud_num;
        if (s->baud_phase >= s->baud_denom) {
            s->baud_phase -= s->baud_denom;
            V22_mod_baseband(s, 
                             &s->tx_buf[s->tx_outbuf_ptr][0],
                             &s->tx_buf[s->tx_outbuf_ptr][1]);

            s->tx_outbuf_ptr = (s->tx_outbuf_ptr + 1) & (V22_TX_BUF_SIZE - 1);
        }
        
        val = (si * dsp_cos(s->carrier_phase) - 
               sq * dsp_cos((PHASE_BASE/4) - s->carrier_phase)) >> COS_BITS;
        s->carrier_phase += s->carrier_incr;
        if (!s->calling) {
            /* a 1800 Hz tone is added for answer modem modulation at 6 dB below it */
            val += (dsp_cos(s->carrier2_phase) >> 1);
            s->carrier2_phase += s->carrier2_incr;
        }
        samples[i] = val;
    }
}

void V22_demod_init(V22DemodState *s)
{
    s->baud_phase = 0;
    s->baud_num = 3;
    s->baud_denom = 40;

    s->carrier_phase = 0;

    if (!s->calling) {
        /* call modem DPSK: 600 bps, carrier at 1200 Hz, 0 db */
        s->carrier_incr = (PHASE_BITS * 1200) / 8000;
    } else {
        /* answer modem DPSK: 600 bps, carrier at 2400 Hz, -1 db, 
           guard tone at 1800 Hz, -7db */
        s->carrier_incr = (PHASE_BITS * 2400) / 8000;
    }
}

void V22_demod(V22DemodState *s, s16 *samples, unsigned int nb)
{
    

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

void V22_test(void)
{
    V22ModState tx;
    V22DemodState rx;
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

    tx.calling = calling;
    tx.opaque = NULL;
    tx.get_bit = test_get_bit;
    tx.mod_type = V34_MOD_600;
    V22_mod_init(&tx);

    rx.calling = !calling;
    rx.opaque = NULL;
    rx.put_bit = test_put_bit;
    rx.mod_type = tx.mod_type;
    V22_demod_init(&rx);

    nb_bits = 0;
    errors = 0;
    for(;;) {
        if (lm_display_poll_event())
            break;
        
        V22_mod(&tx, buf, NB_SAMPLES);
        memset(buf3, 0, sizeof(buf3));

        line_model(line_state, buf1, buf, buf2, buf3, NB_SAMPLES);

        fwrite(buf, 1, NB_SAMPLES * 2, f1);
        
        V22_demod(&rx, buf1, NB_SAMPLES);
    }

    fclose(f1);

    printf("errors=%d nb_bits=%d Pe=%f\n", 
           errors, nb_bits, (float) errors / (float)nb_bits);
}
