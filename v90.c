/* 
 * Implementation of the V90 modulation/demodulation
 * 
 * Copyright (c) 1999,2000 Fabrice Bellard.
 *
 * This code is released under the GNU General Public License version
 * 2. Please read the file COPYING to know the exact terms of the
 * license.
 * 
 * This implementation is totally clean room. It was written by
 * reading the V90 specification and by using basic signal processing
 * knowledge.  
 */

#include "lm.h"
#include "v90priv.h"
#include "v34priv.h"

#define DEBUG

/* Compute the average power of a given constellation :
   nothing complicated here, except that each ucode must be counted
   correctly. We took the algorithm of the spec. It could be very
   optimized and simplified.
 */
int compute_power(V90EncodeState *s)
{
    int m,v,i,j;
    u64 p,n,r[6],k[6],a;

    n = ((u64)1 << s->K) - 1;
    for(i=0;i<6;i++) {
        r[i] = n;
        k[i] = n % s->M[i];
        n = n / s->M[i];
    }

    p = 0;
    a = 1;
    for(i=0;i<6;i++) {
        m = s->M[i];
        for(j=0;j<m;j++) {
            if (j < k[i]) {
                n = a * (r[i+1] + 1);
            } else if (j == k[i]) {
                n = ((u64)1 << s->K) - a * (r[i] - r[i+1]);
            } else {
                n = a * r[i+1];
            }
            v = s->ucode_to_linear[s->m_to_ucode[i][j]];
            p += v * v * n;
        }
        a = a * m;
    }
    return p / ((u64) 6 << s->K);
}

static const int sign_op[4] = { 0, 0x55, 0xff, 0xaa };

/* Select the best method for the current shaping frame
   We find the the shortest path in a treillis of depth ld
 */
static void select_best_signs(V90EncodeState *s, int sp_frame_size, int pp)
{
    int ucode_ptr, depth, i;
    int state, lstate, lstate_min;
    int t_min, x_min, y_min, v_min, w_min, sg;
    int x1, y1, v1, w1, x, y, v, w;
    u8  mem_l[2][TREILLIS_MAX_DEPTH+2], 
        mem_t[2][TREILLIS_MAX_DEPTH+2];
    s16 mem_x[2][TREILLIS_MAX_DEPTH+2], 
        mem_y[2][TREILLIS_MAX_DEPTH+2], 
        mem_v[2][TREILLIS_MAX_DEPTH+2], 
        mem_w[2][TREILLIS_MAX_DEPTH+2];
    
    /* save new pp signs */
    s->pp[s->ucode_ptr] = pp;

    /* Find best path starting at state Q */

    ucode_ptr = (s->ucode_ptr - (sp_frame_size * s->ld)) & (UCODE_BUF_SIZE-1);

    /* fill the treillis memory */
    mem_t[s->Q][0] = s->t;
    mem_x[s->Q][0] = s->x;
    mem_y[s->Q][0] = s->y;
    mem_v[s->Q][0] = s->v;
    mem_w[s->Q][0] = 0;

    for(depth = 0; depth <= s->ld; depth++) {
        
        for(state = 0; state < 2; state++) {
            
            w_min = 0x7fffffff;
            lstate_min = t_min = x_min = y_min = v_min = 0;
            
            /* we select the current state at the beginning of the treillis */
            if (depth == 0) 
                lstate = s->Q;
            else 
                lstate = 0;

            do {
                /* compute the signs */
                sg = mem_t[lstate][depth] ^ s->pp[ucode_ptr];
                /* apply the bit invertion method */
                sg ^= sign_op[(state << 1) | lstate];
                
                x1 = mem_x[lstate][depth];
                y1 = mem_y[lstate][depth];
                v1 = mem_v[lstate][depth];
                w1 = mem_w[lstate][depth];
                
                /* apply the spectral shaping filter to the frame */
                for(i=0;i<sp_frame_size;i++) {
                    x = s->ucode_to_linear[s->ucode[(ucode_ptr + i) & (UCODE_BUF_SIZE-1)]];
                    if (((sg >> i) & 1) == 0)
                        x = -x;
                    
                    y = x - ((s->b1 * x1 + s->a1 * y1) >> 6);
                    v = y - ((s->b2 * y1 + s->a2 * v1) >> 6);
                    w = ((v * v) >> 4) + w1;
                    
                    x1 = x;
                    y1 = y;
                    v1 = v;
                    w1 = w;
                }
                
                /* best transition ? */
                if (w1 < w_min) {
                    t_min = sg;
                    x_min = x1;
                    y_min = y1;
                    v_min = v1;
                    w_min = w1;
                    lstate_min = lstate;
                }
                lstate++;
            } while (lstate < 2 && depth != 0);
        
            /* selects the best transition */
            mem_t[state][depth+1] = t_min;
            mem_x[state][depth+1] = x_min;
            mem_y[state][depth+1] = y_min;
            mem_v[state][depth+1] = v_min;
            mem_w[state][depth+1] = w_min;
            mem_l[state][depth+1] = lstate_min;
        }

        ucode_ptr = (ucode_ptr + sp_frame_size) & (UCODE_BUF_SIZE-1);
    }

    /* now select the best next state by recursing thru the treillis */
    if (mem_w[1][s->ld + 1] < mem_w[0][s->ld + 1])
        state = 1;
    else
        state = 0;

    for(depth=s->ld;depth>0;depth--)
        state = mem_l[state][depth];
    
    s->Q = state;

    /* update current values */
    s->t = mem_t[state][1];
    s->x = mem_x[state][1];
    s->y = mem_y[state][1];
    s->v = mem_v[state][1];

    s->ucode_ptr = (s->ucode_ptr + sp_frame_size) & (UCODE_BUF_SIZE - 1);
}

/*
 * converts (S+K) data bits to 6 a/u law values (note: we output
 * linear values which may be converted back to u/a law). A delay of
 * (ld * frame_size) is introduced due to the shaping treillis.  
 */
static void v90_encode_mapping_frame(V90EncodeState *s, s16 *samples, u8 *data)
{
    int k, l, i, j, frame_size, nb_frames, p, signs;
    u64 v;
    int pv[3];

    /* modulo mapping to ucodes */
    v = 0;
    for(i=0;i<s->K;i++) v |= (data[i+s->S] << i);
    for(i=0;i<6;i++) {
        k = v % s->M[i];
        v /= s->M[i];
        /* compute the corresponding ucode */
        s->ucode[(s->ucode_ptr + i) & (UCODE_BUF_SIZE - 1)] = s->m_to_ucode[i][k];
    }
    
    /* computation of the sign of the ucodes */
    switch(s->S) {
    case 6:
    default:
        /* no redundancy & spectral shaping */
        l = s->last_sign;
        signs = 0;
        for(i=0;i<6;i++) {
            l = data[i] ^ l;
            signs |= (l << i);
        }
        s->last_sign = l;
        s->ucode_ptr = (s->ucode_ptr + 6) & (UCODE_BUF_SIZE - 1);
        frame_size = 6;
        goto skip_spectral_shaping;

    case 5:
        /* 1 bit of redundancy */
        {
            int pp1, pp3, pp5;
            
            pp1 = data[0] ^ s->last_sign;
            pp3 = data[2] ^ pp1;
            pp5 = data[4] ^ pp3;
            s->last_sign = pp5;
            
            pv[0] = (pp1 << 1) | (data[1] << 2) | (pp3 << 3) |
                (data[3] << 4) | (pp5 << 5);

            frame_size = 6;
        }
        break;
    case 4:
        /* 2 bits of redundancy */
        {
            int pp1, pp4;

            pp1 = data[0] ^ s->last_sign;
            pp4 = data[2] ^ pp1;

            s->last_sign = pp4;

            pv[0] = (pp1 << 1) | (data[1] << 2);
            pv[1] = (pp4 << 1) | (data[3] << 2);
            
            frame_size = 3;
        }
        break;

    case 3:
        /* 3 bits of redundancy */
        {
            int pp1, pp3, pp5;

            pp1 = data[0] ^ s->last_sign;
            pp3 = data[1] ^ pp1;
            pp5 = data[2] ^ pp3;
            
            s->last_sign = pp5;

            pv[0] = (pp1 << 1);
            pv[1] = (pp3 << 1);
            pv[2] = (pp5 << 1);

            frame_size = 2;
        }
        break;
    }

    /* select the best signs for each frame (with a delay of ld) */
    nb_frames = 6-s->S;
    signs = 0;
    for(i=0,j=0; i<nb_frames; i++, j+=frame_size) {
        select_best_signs(s, frame_size, pv[i]);
        l = s->t & ((1 << frame_size) - 1);
        signs |= (l << j);
    }

 skip_spectral_shaping:

    /* now the signs are computed, we can compute the pcm values from
       the ucodes. It may be faster to use the convertion already done
       for spectral shaping */
    p = (s->ucode_ptr - 6 - (frame_size * s->ld)) & (UCODE_BUF_SIZE - 1);
    for(i=0;i<6;i++) {
        int x;
        x = s->ucode_to_linear[s->ucode[p]];
        if (((signs >> i) & 1) == 0)
            x = -x;

        samples[i] = x;
        p = (p + 1) & (UCODE_BUF_SIZE - 1);
    }
}

/*
 * converts 6 a/u law samples to (S+K) data bits 
 */
static void v90_decode_mapping_frame(V90DecodeState *s, u8 *data, s16 *samples)
{
    s16 *tab;
    int d1, d2, i, j, l, val, v, m_min, m_max, m;
    u8 signs[6];
    u64 bitbuf;

    /* find signs & ucode */
    bitbuf = 0;
    for(j=5;j>=0;j--) {
        val = samples[j];
        if (val < 0) {
            val = - val;
            signs[j] = 0;
        } else {
            signs[j] = 1;
        }
        /* We look for the value closest to val with a binary search:
           see Knuth section 6.2 exercice 1. */
        tab = &s->m_to_linear[j][0];
        m_min = 0;
        m_max = s->M[j] - 1;
        while (m_min <= m_max) {
            m = (m_min + m_max) >> 1;
            v = tab[m];
            if (v == val)
                goto found;
            else if (val > v) {
                m_max = m - 1;
            } else {
                m_min = m + 1;
            }
        } 
        d1 = tab[m_max] - val;
        d2 = val - tab[m_min];
        if (d1 < d2) 
            m = m_max;
        else
            m = m_min;
    found:
        /* now we update the modulo value */
        bitbuf = bitbuf * s->M[j] + m;
    }

    /* output the K bits */
    for(i=0;i<s->K;i++) 
        data[i + s->S] = (bitbuf >> i) & 1;

    switch(s->S) {
    default:
    case 6:
        /* no redundant bit */
        l = s->last_sign;
        for(i=0;i<6;i++) {
            data[i] = l ^ signs[i];
            l = signs[i];
        }
        s->last_sign = l;
        break;
    case 5:
        /* 1 redundant bit */
        {
            int pp1, pp3, pp5, t0, pv0, Q1;

            t0 = 0;
            for(i=0;i<6;i++)
                t0 |= (signs[i] << i);
            
            pv0 = s->t ^ t0;
            Q1 = (pv0 & 1) ^ s->Q;
            pv0 ^= sign_op[s->Q | (Q1 << 1)] & 0x3f;
            s->t = t0;
            s->Q = Q1;
            
            /* extract the data bits */
            data[1] = (pv0 >> 2) & 1;
            data[3] = (pv0 >> 4) & 1;
            pp1 = ((pv0 >> 1) & 1);
            pp3 = ((pv0 >> 3) & 1);
            pp5 = ((pv0 >> 5) & 1);
            data[0] = pp1 ^ s->last_sign;
            data[2] = pp1 ^ pp3;
            data[4] = pp3 ^ pp5;
            s->last_sign = pp5;
        }
        break;
    case 4:
        /* 2 redundant bits */
        {
            int pp1, pp4, t0, t1, pv0, pv1, Q1, Q2;
            
            t0 = signs[0] | (signs[1] << 1) | (signs[2] << 2);
            t1 = signs[3] | (signs[4] << 1) | (signs[5] << 2);

            pv0 = s->t ^ t0;
            Q1 = (pv0 & 1) ^ s->Q;
            pv0 ^= sign_op[s->Q | (Q1 << 1)] & 7;

            pv1 = t0 ^ t1;
            Q2 = (pv1 & 1) ^ Q1;
            pv1 ^= sign_op[Q1 | (Q2 << 1)] & 7;

            s->t = t1;
            s->Q = Q2;
            
            data[1] = (pv0 >> 2) & 1;
            data[3] = (pv1 >> 2) & 1;
            pp1 = (pv0 >> 1) & 1;
            pp4 = (pv1 >> 1) & 1;
            data[0] = pp1 ^ s->last_sign;
            data[2] = pp4 ^ pp1;
            s->last_sign = pp4;
        }
        break;
    case 3:
        /* 3 redundant bits */
        {
            int pp1, pp3, pp5, t0, t1, t2, pv0, pv1, pv2, Q1, Q2, Q3;
            
            t0 = signs[0] | (signs[1] << 1);
            t1 = signs[2] | (signs[3] << 1);
            t2 = signs[4] | (signs[5] << 1);

            pv0 = s->t ^ t0;
            Q1 = (pv0 & 1) ^ s->Q;
            pv0 ^= sign_op[s->Q | (Q1 << 1)] & 3;

            pv1 = t0 ^ t1;
            Q2 = (pv1 & 1) ^ Q1;
            pv1 ^= sign_op[Q1 | (Q2 << 1)] & 3;

            pv2 = t1 ^ t2;
            Q3 = (pv2 & 1) ^ Q2;
            pv2 ^= sign_op[Q2 | (Q3 << 1)] & 3;

            s->t = t2;
            s->Q = Q3;
            
            pp1 = (pv0 >> 1) & 1;
            pp3 = (pv1 >> 1) & 1;
            pp5 = (pv2 >> 1) & 1;
            
            data[0] = pp1 ^ s->last_sign;
            data[1] = pp3 ^ pp1;
            data[2] = pp5 ^ pp3;
            s->last_sign = pp5;
        }
        break;
    }
}

static void compute_constellation(V90EncodeState *s, 
                                  u8 m_index[6], u8 ucode_used[6][128])
{
    int i,j,k,m;

    /* the ucode are taken from the bigger value down to the smaller value */
    for(j=0;j<6;j++) {
        k = m_index[j];
        m = 0;
        for(i=127;i>=0;i--) {
            if (ucode_used[k][i]) {
                s->m_to_ucode[j][m] = i;
                m++;
            }
        }
        s->M[j] = m;
    }

#ifdef DEBUG
    for(j=0;j<6;j++) {
        printf("M[%d]: ", j);
        for(i=0;i<s->M[j];i++) {
            printf("%3d ", s->m_to_ucode[j][i]);
        }
        printf("\n");
    }
#endif
}


void v90_encode_init(V90EncodeState *s)
{
}



void v90_decode_init(V90DecodeState *s)
{
    int i,j,m;

    /* some test values (You can modify them to test the
       modulator/demodulator) */
    for(i=0;i<6;i++) {
        for(j=0;j<16;j++) 
            s->ucode_used[i][10 + j * 6] = 1;
    }

    s->K = 4 * 6;
    s->S = 5;
    s->alaw = 1;
    s->ld = 2;
    s->a1 = (int) (0.1 * 64);
    s->a2 = (int) (0.2 * 64);
    s->b1 = (int) (-0.1 * 64);
    s->b2 = (int) (0.1 * 64);


    /* we suppose that all other values are set to zero */

    
    /* compute the decode tables */
    if (s->alaw)
        s->ucode_to_linear = v90_alaw_ucode_to_linear;
    else
        s->ucode_to_linear = v90_ulaw_ucode_to_linear;

    for(j=0;j<6;j++) {
        m = 0;
        for(i=127;i>=0;i--) {
            if (s->ucode_used[j][i]) {
                s->m_to_linear[j][m] = s->ucode_to_linear[i];
                m++;
            }
        }
        s->M[j] = m;
    }
}

static u8 test_buf[1000];

/* send a CP frame (analog modem) */
static void v90_send_CP(V90DecodeState *s, int is_CP, int ack)
{
    u8 *buf, *p;
    int i, crc, drn;

    buf = test_buf;

    p = buf;
    put_bits(&p, 17, 0x1ffff); /* frame sync */

    put_bits(&p, 1, 0); /* start bit */
    put_bits(&p, 1, 0); /* reserved */
    put_bits(&p, 1, is_CP); /* 0=CPt 1=CP frame */
    drn = s->K + s->S;
    if (is_CP) 
        drn -= 20;
    else
        drn -= 8;
    put_bits(&p, 5, drn); /* drn: speed */
    put_bits(&p, 5, 0); /* reserved */
    put_bits(&p, 1, 0); /* 1 if silence asked */
    put_bits(&p, 2, 6 - s->S); /* Sr */
    put_bits(&p, 1, ack); /* ack */
    
    put_bits(&p, 1, 0); /* start bit */
    put_bits(&p, 1, s->alaw); /* u/a law selection */
    for(i=0;i<13;i++) {
        put_bits(&p, 1, 1); /* speed (i+2) * 2400 supported (V34 part) */
    }
    put_bits(&p, 2, s->ld);
    
    put_bits(&p, 1, 0); /* start bit */
    put_bits(&p, 16, 0); /* RMS value for TRN1d */
    
    put_bits(&p, 1, 0); /* start bit */
    put_bits(&p, 8, s->a1 & 0xff); /* spectral shaping parameters */
    put_bits(&p, 8, s->a2 & 0xff); /* spectral shaping parameters */
    put_bits(&p, 1, 0); /* start bit */
    put_bits(&p, 8, s->b1 & 0xff); /* spectral shaping parameters */
    put_bits(&p, 8, s->b2 & 0xff); /* spectral shaping parameters */
    
    put_bits(&p, 1, 0); /* start bit */
    for(i=0;i<6;i++) {
        if (i == 4) 
            put_bits(&p, 1, 0); /* start bit */
        put_bits(&p, 4, 0); /* modulation index */
    }
    
    put_bits(&p, 1, 0); /* different different tx - D/A constellations ? */
    put_bits(&p, 7, 0); /* reserved */
    
    /* transmit the constellation */
    for(i=0;i<128;i++) {
        if ((i & 15) == 0) 
            put_bits(&p, 1, 0); /* start bit */
        put_bits(&p, 1, s->ucode_used[0][i]);
    }

    /* CRC */
    put_bits(&p, 1, 0); /* start bit */
    crc = calc_crc(buf + 17, p - (buf+17));
    put_bits(&p, 16, crc);

    put_bits(&p, 3, 0); /* fill */
    printf("CP size= %d\n", p - buf);
}

static int get_bit(u8 **pp)
{
    u8 *p;
    int v;

    p = *pp;
    v = *p++;
    *pp = p;
    return v;
}


static int get_bits(u8 *p, int n)
{
    int i, v;

    v = 0;
    for(i=n-1;i>=0;i--) {
        v |= *p++ << i;
    }
    return v;
}

/* parse the CP packet & compute the V90 parameters */
static void v90_parse_CP(V90EncodeState *s, u8 *buf)
{
    u8 m_index[6];
    u8 ucode_used[7][128];
    int drn, i, j, k, nb_constellations;

    /* now the whole packet is can be read */

    s->S = 6 - get_bits(buf + 31, 2);
    s->alaw = get_bits(buf + 35, 1);
    s->ld = get_bits(buf + 49, 2);
    s->a1 = (s8) get_bits(buf + 69, 8);
    s->a2 = (s8) get_bits(buf + 77, 8);
    s->b1 = (s8) get_bits(buf + 86, 8);
    s->b2 = (s8) get_bits(buf + 94, 8);
    
    for(i=0;i<4;i++)
        m_index[i] = get_bits(buf + 103 + i * 4, 4);

    for(i=0;i<2;i++)
        m_index[4 + i] = get_bits(buf + 120 + i * 4, 4);
    nb_constellations = 0;
    for(i=0;i<6;i++) {
        if (m_index[i] > nb_constellations) nb_constellations = m_index[i];
    }
    nb_constellations++;
    if (buf[128])
        nb_constellations++;
    
    for(i=0;i<nb_constellations;i++) {
        for(j=0;j<128;j++) {
            k = i * 128 + j;
            ucode_used[i][j] = buf[137 + (17 * (k >> 4)) + (k & 15)];
        }
    }
    
    /* compute K */
    drn = get_bits(buf + 20, 5);
    if (buf[19]) 
        drn += 20;
    else 
        drn += 8;
    s->K = drn - s->S;

    if (s->alaw)
        s->ucode_to_linear = v90_alaw_ucode_to_linear;
    else
        s->ucode_to_linear = v90_ulaw_ucode_to_linear;

    printf("V90_received_CP:\n");
    compute_constellation(s, m_index, ucode_used);

    printf("S=%d K=%d R=%d alaw=%d ld=%d a1=%d a2=%d b1=%d b2=%d\n", 
           s->S, s->K, ((s->S + s->K) * 8000) / 6,
           s->alaw, s->ld, s->a1, s->a2, s->b1, s->b2);
}

/* received & parse the CP packet */
static void v90_receive_CP(V90EncodeState *s)
{
    u8 buf[1024], *p, *q;
    int b, i, frame_index, one_count, frame_count, nb_constellations;
    int crc1;
    u8 m_index[6];

    p = test_buf;
     
 wait_sync:
    one_count = 0;
    
    while (get_bit(&p)) {
        one_count++;
    }
    if (one_count != 17)
        goto wait_sync;

#ifdef DEBUG
    printf("got CP sync\n");
#endif

    frame_index = 0;
    frame_count = 8;
    crc1 = 1;
    q = buf + 17;
    while (frame_index < frame_count) {
        printf("%2d: ", frame_index);
        *q++ = 0;
        for(i=0;i<16;i++) {
            b = get_bit(&p);
            *q++ = b;
            printf("%d", b);
        }
        printf("\n");

        if (frame_index == 6) {
            /* compute the number of constellation to read */
            for(i=0;i<4;i++) {
                m_index[i] = get_bits(buf + 103 + i * 4, 4);
            }
            for(i=0;i<2;i++) {
                m_index[4 + i] = get_bits(buf + 120 + i * 4, 4);
            }
            nb_constellations = 0;
            for(i=0;i<6;i++) {
                if (m_index[i] > nb_constellations) nb_constellations = m_index[i];
            }
            nb_constellations++;
            if (buf[128])
                nb_constellations++;
            frame_count += 8 * nb_constellations;
        }
        
        if (frame_index == (frame_count - 1)) {
            /* check the crc (it must be zero because we include the crc itself) */
            crc1 = calc_crc(buf + 17, q - (buf+17));
        }

        if (get_bit(&p) != 0) {
            printf("start bit expected\n");
            goto wait_sync;
        }
        frame_index++;
    }
    if (crc1 != 0)
        goto wait_sync;

    v90_parse_CP(s, buf);
}

/* simple test of V90 algebraic computations */
/* Note: if ld != 0 and 3 <= S <= 4, the delay introduced with data[][]
   is not correct */

void V90_test(void)
{
    int i,j,n,l;
    V90EncodeState v90_enc;
    V90DecodeState v90_dec;
    u8 data[5][48], data1[48];
    s16 samples[6];

    /* init modem state */
    memset(&v90_enc, 0, sizeof(v90_enc));
    v90_encode_init(&v90_enc);

    memset(&v90_dec, 0, sizeof(v90_dec));
    v90_decode_init(&v90_dec);
    
    /* send the CP sequence which contains the modulation parameters */
    v90_send_CP(&v90_dec, 1, 0);
    
    /* "receive" it ! */
    v90_receive_CP(&v90_enc);

    /* number of data bits per mapping frame */
    n = v90_enc.S + v90_enc.K;

    /* transmit & receive 1000 mapping frames */
    memset(data, 0, sizeof(data));
    l = 0;
    for(i=0;i<1000;i++) {
        for(j=0;j<n;j++) data[l][j] = random() & 1;
        
        v90_encode_mapping_frame(&v90_enc, samples, data[l]);

        // for(j=0;j<6;j++) samples[j] += (random() % 32) - 16;

        printf("%4d: ", i);
        for(j=0;j<6;j++) printf("%6d,", samples[j]);
        printf("\n");

        v90_decode_mapping_frame(&v90_dec, data1, samples);

        l = (l + 1) % (v90_dec.ld+1);

        for(j=0;j<n;j++) {
            if (data[l][j] != data1[j]) {
                printf("error mapping frame=%d bit=%d\n", i, j);
            }
        }

    }
}
