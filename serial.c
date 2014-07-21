/*
 * Serial encoder/decoder.
 * 
 * Copyright (c) 2000 Fabrice Bellard.
 *
 * This code is released under the GNU General Public License version
 * 2. Please read the file COPYING to know the exact terms of the
 * license.
 * 
 */

#include "lm.h"

/* Init the serial encoder & decoder. 5 <= data_bits <= 8 and parity
   can be 'E', 'O', or 'N' */
void serial_init(struct sm_state *s, int data_bits, int parity)
{
    s->serial_data_bits = data_bits;
    s->serial_use_parity = (parity == 'E' || parity == 'O');
    s->serial_parity = (parity == 'O');
    s->serial_wordsize = data_bits + 2 + s->serial_use_parity;

    /* rx init */
    s->serial_buf = 0;
    s->serial_cnt = 0;

    /* tx init */
    s->serial_tx_buf = 0;
    s->serial_tx_cnt = 0;
}

/* return a bit from the tx fifo with serial encoding */
int serial_get_bit(void *opaque)
{
    struct sm_state *s = opaque;
    int data, j, bit, p;

    if (s->serial_tx_cnt == 0) {
        data = sm_get_bit(&s->tx_fifo);
        if (data == -1)
            return 1;
        s->serial_tx_cnt = s->serial_wordsize;
        if (s->serial_use_parity) {
            p = s->serial_parity;
            for(j=0;j<s->serial_data_bits;j++) p ^= (data >> j) & 1;
            s->serial_tx_buf = (data << 2) | (p << 1) | 1;
        } else {
            s->serial_tx_buf = (data << 1) | 1;
        }
    }
    s->serial_tx_cnt--;
    bit = (s->serial_tx_buf >> s->serial_tx_cnt) & 1;
    return bit;
}

/* decode a serial stream and put it in rx_fifo. Not optimized */
void serial_put_bit(void *opaque, int bit)
{
    struct sm_state *s = opaque;
    int mask, p, j, data;

    s->serial_buf = (s->serial_buf << 1) | bit;
    if (s->serial_cnt >= (s->serial_wordsize-1)) {
        mask = 1 | (1 << (s->serial_wordsize-1));

        if ((s->serial_buf & mask) == 0x1) {
            data = (s->serial_buf & ((1 << s->serial_wordsize) - 1)) >> 1;
            
            if (s->serial_use_parity) {
                p = s->serial_parity;
                for(j=0;j<=s->serial_data_bits;j++) p ^= (data >> j) & 1;
                if (!p)
                    sm_put_bit(&s->rx_fifo, data >> 1);
            } else {
                sm_put_bit(&s->rx_fifo, data);
            }
            
            s->serial_cnt = 0;
        }
    } else {
        s->serial_cnt++;
    }
}

