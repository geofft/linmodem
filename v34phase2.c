/* 
 * Implementation of the V34 phase 2
 * 
 * Copyright (c) 1999,2000 Fabrice Bellard.
 *
 * This code is released under the GNU General Public License version
 * 2. Please read the file COPYING to know the exact terms of the
 * license.
 * 
 * This implementation is totally clean room. It was written by
 * reading the V34 specification and by using basic signal processing
 * knowledge.  
 */
#include "lm.h"
#include "v34priv.h"

#define DEBUG

#define INFO_SYNC 0x72

/* send the v34 info0 sequence */
void V34_send_info0(V34State *s, int ack)
{
    u8 buf[48], *p, *q;
    int crc;

    p = buf;
    put_bits(&p, 4, 0xf); /* fill bits */
    put_bits(&p, 8, INFO_SYNC); /* sync word */
    
    put_bits(&p, 1, 1); /* symbol rate 2743 supported ? */
    put_bits(&p, 1, 1); /* symbol rate 2800 supported ? */
    put_bits(&p, 1, 1); /* symbol rate 3429 supported ? */
    put_bits(&p, 1, 1); /* symbol rate 3000, low carrier supported ? */
    put_bits(&p, 1, 1); /* symbol rate 3000, high carrier supported ? */
    put_bits(&p, 1, 1); /* symbol rate 3200, low carrier supported ? */
    put_bits(&p, 1, 1); /* symbol rate 3200, high carrier supported ? */
    
    put_bits(&p, 1, 1); /* symbol rate 3429 disallowed ? */
    put_bits(&p, 1, 1); /* can power reduce ? */
    put_bits(&p, 3, 5); /* difference between emit & receive sym rate */
    put_bits(&p, 1, 0); /* from CME modem ? */
    put_bits(&p, 1, 0); /* reserved by ITU */
    put_bits(&p, 2, 0); /* tx clock source: 0=internal */
    put_bits(&p, 1, ack); /* ack reception */

    /* crc */
    crc = calc_crc(buf + 12, p - (buf + 12));
    put_bits(&p, 16, crc);

    put_bits(&p, 4, 0xf); /* fill bits */

    for(q=buf;q<p;q++) {
        put_sym(s, buf[i], 0);
    }
}

/* send the v34 info1c sequence */
void V34_send_info1c(V34State *s)
{
    u8 buf[109], *p;
    int crc;
    
    p = buf;

    put_bits(&p, 4, 0xf); /* fill bits */
    put_bits(&p, 8, INFO_SYNC); /* sync word */

    put_bits(&p, 3, 1); /* minimum power reduction (XXX) */
    put_bits(&p, 3, 1); /* additional power reduction (XXX) */
    put_bits(&p, 7, 0); /* length of MD sequence (35 ms incr) */

    for(i=0;i<6;i++) {
        /* for each symbol speed (increasing order) */
        put_bits(&p, 1, 0); /* use high carrier ? (XXX) */
        put_bits(&p, 4, 0); /* pre emphasis filter (XXX) */
        put_bits(&p, 4, 12); /* projected data rate (XXX) */
    }

    put_bits(&p, 10, 0); /* frequency offset (XXX) */

    /* crc */
    crc = calc_crc(buf + 12, p - (buf + 12));
    put_bits(&p, 16, crc);

    put_bits(&p, 4, 0xf); /* fill bits */
}

/* send the v34 info1a sequence */
void V34_send_info1a(V34State *s)
{
    u8 buf[70], *p;
    int crc;
    
    p = buf;

    put_bits(&p, 4, 0xf); /* fill bits */
    put_bits(&p, 8, INFO_SYNC); /* sync word */

    put_bits(&p, 3, 1); /* minimum power reduction (XXX) */
    put_bits(&p, 3, 1); /* additional power reduction (XXX) */
    put_bits(&p, 7, 0); /* length of MD sequence (35 ms incr) */

    put_bits(&p, 1, 0); /* high carrier used (XXX) */
    put_bits(&p, 4, 0); /* pre emphasis filter (XXX) */
    put_bits(&p, 4, 12); /* proj max data rate (XXX) */
    
    put_bits(&p, 3, 4); /* sym rate ans->cal (XXX : 3200) */
    put_bits(&p, 3, 4); /* sym rate cal->ans (XXX : 3200) */
    
    put_bits(&p, 10, 0); /* frequency offset (XXX) */

    /* crc */
    crc = calc_crc(buf + 12, p - (buf + 12));
    put_bits(&p, 16, crc);

    put_bits(&p, 4, 0xf); /* fill bits */
}


void V34_send_L1(V34State *s, int rep)
{
    static char ph[25] = {
        /* 150 */  1,-1, 1, 1,
        /* 750 */  1, 0, 1, 0,
        /*1350 */  1, 1,-1, 0,
        /*1950 */  1, 1,-1, 0,
        /*2550 */  1,-1, 1,-1,
        /*3150 */ -1,-1,-1, 1,
        /*3750 */  1 };

    for(n=0;n<rep;n++) {
        for(i=0;i<25;i++) {
            if (ph[i]) {
                put_sym(s, i * 150 + 150, ph[i]);
            }
        }
    }
}


int V34P2_process(V34State *s, s16 *output, s16 *input, int nb_samples)
{
    /* modulation */
    switch(s->state) {
    case V34_P2_INFO0_SEND:
        V34_send_info0(
        
    }
    
    /* demodulation */
    switch(s->state) {
    case V34_P2_INFO0_SEND:
        
        
    }
}

