/* 
 * Implementation of the V34 equalizer
 * 
 * Copyright (c) 2000 Fabrice Bellard.
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

//#define DEBUG

#define SQRT3_2 (int)(0.8660254 * 0x4000)

#define CMUL(a,b,c) \
{\
    (a).re=((b).re*(c).re-(b).im*(c).im) >> 14;\
    (a).im=((b).im*(c).re+(b).re*(c).im) >> 14;\
}

#define SCALE(a,b,shift) \
{\
    (a).re=(b).re >> (shift);\
    (a).im=(b).im >> (shift);\
}

#define FFT23_SIZE (EQ_FRAC * V34_PP_SIZE)
#define RENORM 256.0
#define FRAC   16384.0

static icomplex fft144_table[FFT23_SIZE];
static s16 fft144_reverse[FFT23_SIZE];

icomplex tabPP[V34_PP_SIZE]; /* PP is used to generate the PP signal */
static icomplex tabPP_fft[V34_PP_SIZE];

/* Compute an fft on an array whose size n = 2^k.3^l. The algorithm is
   not the most efficient, but it is simple. Each stage of the fft
   normalize the result: it is divided by 2 (for fft2) or 4 (for fft3)
   at each stage. For 144, the renormalization is 2^8 */
static void fft23(icomplex *output, icomplex *tab, unsigned int n)
{
    unsigned int s, i, j, k;
    icomplex *p, *q, *r, *c1_ptr, *c2_ptr;
    s = n;
    k = 1;
    while (s != 1) {
        if ((s % 3) == 0) {
            /* we handle first the '3' factors */
            s /= 3;
            for(p = tab;p<tab + n;p+=2 * s) {
                c1_ptr = fft144_table;
                c2_ptr = fft144_table;
                q = p + s;
                r = p + 2*s;
                for(j=0;j<s;j++) {
                    icomplex a,b,c;
                    int tmp1, tmp2, tmp3, tmp4, tmp5, tmp6, tmp7;
                    int tmp8, tmp9, tmp10, tmp11, tmp12;
                    
                    SCALE(a, *p, 2);
                    SCALE(b, *q, 2);
                    SCALE(c, *r, 2);

                    /* fft on 3 points */
                    tmp1 = a.re;
                    tmp10 = a.im;
                    
                    tmp2 = b.re;
                    tmp3 = c.re;
                    tmp4 = tmp2 + tmp3;
                    tmp9 = (SQRT3_2 * (tmp3 - tmp2)) >> 14;
                    tmp6 = b.im;
                    tmp7 = c.im;
                    tmp8 = (SQRT3_2 * (tmp6 - tmp7)) >> 14;
                    tmp11 = tmp6 + tmp7;
                    
                    p->re = (tmp1 + tmp4);
                    tmp5 = tmp1 - (tmp4 >> 1);
                    c.re = (tmp5 - tmp8);
                    b.re = (tmp5 + tmp8);
                    p->im = tmp10 + tmp11;
                    tmp12 = tmp10 - (tmp11 >> 1);
                    b.im = (tmp9 + tmp12);
                    c.im = (tmp12 - tmp9);
                    
                    /* post multiplications */
                    CMUL(*q, b, *c1_ptr);
                    CMUL(*r, c, *c2_ptr);

                    p++;
                    q++;
                    r++;
                    c1_ptr += k;
                    c2_ptr += 2 * k;
                    if (c2_ptr >= (fft144_table + n))
                        c2_ptr -= n;
                }
            }
            k *= 3;
        } else {
            /* '2' factors */
            s /= 2;
            for(p=tab;p<tab + n;p += s) {
                c1_ptr = fft144_table;
                q = p + s;
                for(j=0;j<s;j++) {
                    icomplex a, b;

                    SCALE(a, *p, 1);
                    SCALE(b, *q, 1);

                    /* fft on 2 points */
                    p->re = (a.re + b.re);
                    p->im = (a.im + b.im);
                    b.re = (a.re - b.re);
                    b.im = (a.im - b.im);

                    /* post multiplication */
                    CMUL(*q, b, *c1_ptr);
                    p++;
                    q++;
                    c1_ptr += k;
                }
            }
            k *= 2;
        }
    }

    /* now we reverse the indices : cannot permute in place because
       fft_reverse is not involutive */
    for(i=0;i<n;i++) {
        output[fft144_reverse[i]] = tab[i];
    }
}

static s16 cos12[12] = { 16384, 14188, 8192, 0, -8192, -14188, 
                         -16384, -14188, -8192, 0, 8192, 14188 };


static icomplex tabtmp[48];

/* Init some constants for the equalizer. May be moved to v34gen.c if
   it is too complicated */
void V34eq_init(void)
{
    int i, j, k, n;
    float a, carrier;
    complex tab1[V34_PP_SIZE], tab2[V34_PP_SIZE];

    for(i=0;i<FFT23_SIZE;i++) {
        a = - 2 * M_PI * i / (float) FFT23_SIZE;
        fft144_table[i].re = (int)(cos(a) * 0x4000);
        fft144_table[i].im = (int)(sin(a) * 0x4000);
    }

    /* reverse table */
    for(i=0;i<FFT23_SIZE;i++) {
        int base;
        j = i;
        k = 0;
        n = FFT23_SIZE;
        while (n != 1) {
            if ((n % 2) == 0)
                base = 2;
            else
                base = 3;
            
            k = base * k + (j % base);
            j /= base;
            n /= base;
        }
        fft144_reverse[i] = k;
    }


    /* compute the V34 PP sequence */
    for(k=0;k<12;k++) {
        for(i=0;i<4;i++) {
            j = k * i;
            if ((k % 3) == 1)
                    j += 4;
            j = j % 12;
            tabPP[4*k+i].re = cos12[j];
            tabPP[4*k+i].im = cos12[(15 - j) % 12];
        }
    }

    /* fft of PP sequence */
    carrier = 0.0;
    for(i=0;i<V34_PP_SIZE;i++) {
        icomplex a, b;
#if 0
        b.re = (int)(cos(carrier) * 0x4000);
        b.im = (int)(sin(carrier) * 0x4000);
        CMUL(a, tabPP[i], b);
#else
        a = tabPP[i];
#endif
        tabtmp[i] = a;
        tab1[i].re = a.re / 16384.0 / sqrt(48);
        tab1[i].im = a.im / 16384.0 / sqrt(48);
        carrier -= 2 * M_PI * 1920.0 / 3200.0;
        //        carrier -= 2 * M_PI * 1800.0 / 2400.0;
    }
    slow_fft(tab2, tab1, V34_PP_SIZE, 0);
    
    for(i=0;i<V34_PP_SIZE;i++) {
        tabPP_fft[i].re = (int)(tab2[i].re * 0x4000);
        tabPP_fft[i].im = (int)(tab2[i].im * 0x4000);
#if 0
        printf("%3d: %7.4f %7.4f\n", 
               i, tab2[i].re, tab2[i].im);
#endif
    }
}


/* Fast training of the equalizer based on the PP sequence */
void V34_fast_equalize1(V34DSPState *s, s16 *input)
{
    int i,k,j, vmax, v, lshift, renorm;
    icomplex tab[FFT23_SIZE], tab1[FFT23_SIZE];
    float carrier;

    for(i=0;i<FFT23_SIZE;i++) {
        tab[i].re = input[i];
        tab[i].im = 0;
    }

#if 0
    /* test: modulate a PP sequence */
    {
        icomplex a, b;
        float carrier, carrier_incr;
        int p;

        p = 0;
        carrier_incr = (2 * M_PI * 0.25);
        for(i=0;i<FFT23_SIZE/3;i++) {
            a = tabPP[i];
            b = tabPP[(i+1) % (FFT23_SIZE/3)];
            
            tab[p].re = a.re;
            tab[p].im = a.im;
            p++;
            tab[p].re = 0;
            tab[p].im = 0;
            p++;
            tab[p].re = 0;
            tab[p].im = 0;
            p++;
        }
    }
#endif

#if 0
    for(i=0;i<FFT23_SIZE;i++) {
        tab[i].re = 0;
        tab[i].im = 0;
        if (i == 2)
            tab[i].re = FRAC;
    }
#endif

#ifdef DEBUG
    printf("Fast equalizer Input:\n");
    for(i=0;i<FFT23_SIZE;i++) {
        printf("%3d: %7.4f %7.4f\n", 
               i, tab[i].re / FRAC, tab[i].im / FRAC);
    }
#endif

    fft23(tab1, tab, FFT23_SIZE);
    
    /* find best renormalization shift (the fft prefers to have its
       inputs as close as 2^14 as possible) */

    for(i=0;i<FFT23_SIZE;i++)
        tab[i] = tab1[i];
        
    vmax = 0;
    for(i=0;i<FFT23_SIZE/2;i++) {
        v = abs(tab[i].re);
        if (v > vmax)
            vmax = v;
        v = abs(tab[i].im);
        if (v > vmax)
            vmax = v;
    }
    lshift = 0;
    while (vmax < 0x4000) {
        vmax <<= 1;
        lshift++;
    }
#if defined(DEBUG) || 1
    printf("vmax=%d lshift=%d\n", vmax, lshift);
#endif

    for(i=0;i<FFT23_SIZE/2;i++) {
        tab[i].re <<= lshift;
        tab[i].im <<= lshift;
    }

    for(i=0;i<24;i++) {
        icomplex a, b, c;
        int norm, d, e;

        c = tabPP_fft[i];
        b = tab[i];
        norm = b.re * b.re + b.im * b.im;
        b = tab[i+ 48];
        norm += b.re * b.re + b.im * b.im;
        norm = norm >> 14;
        if (norm == 0)
            norm = 1;
        c.re = (c.re << 14) / norm;
        c.im = (c.im << 14) / norm;
        
        b.re = tab[i].re;
        b.im = - tab[i].im;
        CMUL(a, c, b);
        tab1[i] = a;
        
        b.re = tab[48 + i].re;
        b.im = - tab[48 + i].im;
        CMUL(a, c, b);
        tab1[48 + i] = a;
    }

    for(i=24;i<48;i++) {
        icomplex a, b, c;
        int norm;

        c = tabPP_fft[i];
        b = tab[i];
        norm = b.re * b.re + b.im * b.im;
        norm = norm >> 14;
        if (norm == 0)
            norm = 1;
        c.re = (c.re << 14) / norm;
        c.im = (c.im << 14) / norm;
        
        b.re = tab[i].re;
        b.im = - tab[i].im;
        CMUL(a, c, b);
        tab1[i] = a;
    }

    for(i=FFT23_SIZE/2;i<FFT23_SIZE;i++) {
        tab1[i].re = 0;
        tab1[i].im = 0;
    }
    for(i=0;i<FFT23_SIZE;i++)
        tab[i] = tab1[i];

#ifdef DEBUG
    printf("After FFT and division:\n");
    for(i=0;i<FFT23_SIZE;i++) {
        printf("%3d: %7.4f %7.4f\n", 
               i, tab[i].re / FRAC, tab[i].im / FRAC);
    }
#endif

    /* inverse FFT (we assume the size is a multiple of two) */
    for(i=1;i<FFT23_SIZE/2;i++) {
        icomplex a;
        j = FFT23_SIZE - i;
        a = tab[i];
        tab[i] = tab[j];
        tab[j] = a;
    }
    fft23(tab1, tab, FFT23_SIZE);

    /* find the maximum real value & center the equalizer on that value */
    vmax = 0;
    j = 0;
    for(i=0;i<FFT23_SIZE;i++) {
        v = abs(tab1[i].re);
        if (v > vmax) {
            vmax = v;
            j = i;
        }
    }
    
    /* center & renormalize */
    renorm = (int) (256.0 / FFT23_SIZE * RENORM * RENORM * 4.0 / 
                    (1 << (14 - lshift)));
    
#if defined(DEBUG)
    printf("Equalizer:\n");
    printf("center=%d renorm=%d\n", j, renorm);
#endif

#if 0
    k = FFT23_SIZE/2;
    while (((j - k) % 3) != 0) {
        if (++j == FFT23_SIZE)
            j = 0;
    }
#else
    k = 0;
    j = 0;
#endif

    for(i=0;i<FFT23_SIZE;i++) {
        //        s->eq_filter[k][0] = (tab1[j].re * renorm) << 8;
        //        s->eq_filter[k][1] = (tab1[j].im * renorm) << 8;
        if (++k == FFT23_SIZE)
            k = 0;
        if (++j == FFT23_SIZE)
            j = 0;
    }

#ifdef DEBUG
    for(i=0;i<FFT23_SIZE;i++) {
        printf("%3d: %7.4f %7.4f\n", 
               i, 
               (float)s->eq_filter[i][0] / (1 << 30),  
               (float)s->eq_filter[i][1] / (1 << 30));
    }
#endif
}

#undef CMUL

#define CMUL(a,b,c) \
{\
    (a).re=((b).re*(c).re-(b).im*(c).im);\
    (a).im=((b).im*(c).re+(b).re*(c).im);\
}

void V34_fast_equalize(V34DSPState *s, s16 *input)
{
    complex tab[144], tab1[144];
    complex a, b, c;
    float norm, d;
    int i;

    for(i=0;i<FFT23_SIZE;i++) {
        tab1[i].re = input[i];
        tab1[i].im = 0;
    }

    slow_fft(tab, tab1, 144, 0);

    for(i=0;i<24;i++) {
        c.re = tabPP_fft[i].re / FRAC;
        c.im = tabPP_fft[i].im / FRAC;

        b = tab[i];
        norm = b.re * b.re + b.im * b.im;
        b = tab[i+ 48];
        norm += b.re * b.re + b.im * b.im;
        c.re = (c.re) / norm;
        c.im = (c.im) / norm;
        
        b.re = tab[i].re;
        b.im = - tab[i].im;
        CMUL(a, c, b);
        tab1[i] = a;
        
        b.re = tab[48 + i].re;
        b.im = - tab[48 + i].im;
        CMUL(a, c, b);
        tab1[48 + i] = a;
    }

    for(i=24;i<48;i++) {
        c.re = tabPP_fft[i].re / FRAC;
        c.im = tabPP_fft[i].im / FRAC;
        b = tab[i];
        norm = b.re * b.re + b.im * b.im;
        c.re = (c.re) / norm;
        c.im = (c.im) / norm;
        
        b.re = tab[i].re;
        b.im = - tab[i].im;
        CMUL(a, c, b);
        tab1[i] = a;
    }
    
    for(i=FFT23_SIZE/2;i<FFT23_SIZE;i++) {
        tab1[i].re = 0;
        tab1[i].im = 0;
    }

    for(i=0;i<FFT23_SIZE;i++) {
        printf("%3d: %7.4f %7.4f\n", 
               i, tab[i].re / FRAC, tab[i].im / FRAC);
    }

    slow_fft(tab, tab1, 144, 1);

    
    for(i=0;i<FFT23_SIZE;i++) {
        s->eq_filter[i][0] = (int)(tab[i].re * FRAC * FRAC / 12) << 16;
        s->eq_filter[i][1] = (int)(tab[i].im * FRAC * FRAC / 12) << 16;
    }
}
