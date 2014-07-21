/* 
 * V34 constant data generator
 * 
 * Copyright (c) 1999,2000 Fabrice Bellard.
 *
 * This code is released under the GNU General Public License version
 * 2. Please read the file COPYING to know the exact terms of the
 * license.
 */
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>

#include "dsp.h"

#define V34_SAMPLE_RATE_NUM 10
#define V34_SAMPLE_RATE_DEN 3
#define V34_SAMPLE_RATE ((2400*V34_SAMPLE_RATE_NUM)/V34_SAMPLE_RATE_DEN)
#define V22_TX_FILTER_SIZE (20 * 40)

#define RC_FILTER_SIZE 40

void find_data_rot(int *data_ptr, int *rot_ptr, int x0, int y0)
{
    int xx,yy,data,rot,x,y;

    /* find the data & rotation */
    for(data=0;data<4;data++) {
        x = -3 + (data & 1) * 4;
        y = -3 + (data >> 1) * 4;
        for(rot=0;rot<4;rot++) {
            if (x == x0 && y == y0) {
                *data_ptr = data;
                *rot_ptr = rot;
                return;
            }
            /* rotate by 90 */
            xx = y;
            yy = -x;
            x = xx;
            y = yy;
        }
    }
}


/* 0 : rotation by 180, 1 = rotation by 90 */
int classify(int x[2][2])
{
    int x0, y0, x1, y1,d0,d1,r0,r1;

    x0 = 2 * x[0][0] - 3;
    y0 = 2 * x[0][1] - 3;
    x1 = 2 * x[1][0] - 3;
    y1 = 2 * x[1][1] - 3;
    
    find_data_rot(&d0,&r0,x0,y0);
    find_data_rot(&d1,&r1,x1,y1);
    if (((r0 - r1) & 1) == 0)
        return 0;
    else
        return 1;
}


void gen_table(int nb_trans)
{
    int Y[5],Yt[5], trans, ss[2][3], xx[2][2];
    int res, i, y0;

    printf("u8 trellis_trans_%d[256][4] = {\n",
           nb_trans);
    
    for(y0=0;y0<2;y0++) {

    for(trans=0;trans<nb_trans;trans++) {
        printf(" /* trans=%d y0=%d */\n", trans, y0);
        Yt[1] = trans & 1;
        Yt[2] = (trans >> 1) & 1;
        Yt[4] = (trans >> 2) & 1;
        Yt[3] = (trans >> 3) & 1;

    for(xx[0][0] = 0; xx[0][0] < 4; xx[0][0]++)
    for(xx[0][1] = 0; xx[0][1] < 4; xx[0][1]++)
    for(xx[1][0] = 0; xx[1][0] < 4; xx[1][0]++)
    for(xx[1][1] = 0; xx[1][1] < 4; xx[1][1]++) {
        
        /* (§ 9.6.3.1) find Y vector. We traducted the table into binary expressions */
        for(i=0;i<2;i++) {
            int x,y,y0,x0,y1,x1;
            
            /* XXX: is it right to suppose that we use figure 9 as a periodic mapping ? */
            x = xx[i][0];
            y = xx[i][1];
            
            x0 = x & 1;
            x1 = ((x & 2) >> 1);
            y0 = y & 1;
            y1 = ((y & 2) >> 1);
            
            ss[i][2] = x1 ^ y1 ^ y0 ^ x0;
            ss[i][1] = y0;
            ss[i][0] = y0 ^ x0;
        }
        
        /* table 13 traducted into binary operations */
        Y[4] = ss[0][2] ^ ss[1][2];
        Y[3] = ss[0][1];
        Y[2] = ss[0][0];
        Y[1] = (ss[0][0] & ~ss[1][0] & 1) ^ ss[0][1] ^ ss[1][1];

        res = 0;
        switch(nb_trans) {
        case 4:
            res = (Yt[1] == Y[1] && 
                   Yt[2] == Y[2]);
            break;
        case 8:
            res = (Yt[1] == Y[1] && 
                   Yt[2] == Y[2] &&
                   Yt[4] == Y[4]);
            break;
        case 16:
            res = (Yt[1] == Y[1] && 
                   Yt[2] == Y[2] &&
                   Yt[3] == Y[3] &&
                   Yt[4] == Y[4]);
            break;
        }

        if (res && classify(xx) == y0) {
            printf(" { %d, %d, %d, %d },\n", 
                   xx[0][0],
                   xx[0][1],
                   xx[1][0],
                   xx[1][1]);
        }
    }

    }
    }
    printf("};\n");
}

static u8 S_tab[6][8] = {
  /* a, c, d1, e1, d2, e2, J, P */
    { 1, 1, 2, 3, 3, 4, 7, 12, }, /* S=2400 */
    { 8, 7, 3, 5, 2, 3, 8, 12, }, /* S=2743 */
    { 7, 6, 3, 5, 2, 3, 7, 14, }, /* S=2800 */
    { 5, 4, 3, 5, 2, 3, 7, 15, }, /* S=3000 */
    { 4, 3, 4, 7, 3, 5, 7, 16, }, /* S=3200 */
    {10, 7, 4, 7, 4, 7, 8, 15, }, /* S=3429 */
};

/* this table depends on the sample rate. We have S=a1/c1 * V34_SAMPLE_RATE */
static u8 baud_tab[6][2] = {
    /* a1, c1 */
    { 3, 10 },
    { 12, 35 },
    { 7, 20 },
    { 3, 8 },
    { 2, 5 },
    { 3, 7 },
};

#define FFT_SIZE 2048

/* build a square raised cosine nyquist filter of n coefficients
   centered on f0, with coefficients alpha and beta.
*/

void build_sqr_nyquist_filter(float *filter, 
                              float f0, float alpha, float beta, int n)
{
    float f, f1, f2, val, tau, norm;
    int i,j;
    complex tab[FFT_SIZE];
    
    f1 = (1.0 - beta) * alpha;
    f2 = (1.0 + beta) * alpha;
    tau = 0.5 / alpha;
    norm = tau / sqrt(FFT_SIZE);
    if (f0 != 0) 
        norm *= 0.5;
    
    for(i=0;i<=FFT_SIZE/2;i++) {
	f = i / (float)FFT_SIZE;

        /* center on f0 */
        f = fabs(f - f0);

        if (f <= f1)
            val = 1;
        else if (f <= f2) {
            val = 0.5 * (1.0 + cos((M_PI * tau / beta) * (f - f1)));
        } else {
            val = 0;
        }
        val = sqrt(val);

        tab[i].re = val * norm;
        tab[i].im = 0;
    }

    for(i=1;i<FFT_SIZE;i++) tab[FFT_SIZE - i] = tab[i];
    
    fft_calc(tab, FFT_SIZE, 0);

    j = FFT_SIZE - ((n-1)/2);
    for(i=0;i<n;i++) {
        filter[i] = tab[j].re;
        if (++j == FFT_SIZE)
            j = 0;
    }
}

void write_filter(char *name, float *filter, int n)
{
    int i;

    printf("s16 %s[%d]=\n{\n", 
           name, n);
    for(i=0;i<n;i++) {
        printf("%6d, ", (int)(filter[i] * 0x4000));
        if ((i % 8) == 7) 
            printf("\n");
    }
    printf("\n};\n");
}


int main(int argc, char **argv)
{
    int i, j, n;
    float filter[FFT_SIZE];
    char buf[512];

    printf("/* THIS SOURCE CODE IS AUTOMATICALLY GENERATED - DO NOT MODIFY */\n");
    printf("/*\n"
           " * V34 tables\n"
           " * \n"
           " * Copyright (c) 1999,2000 Fabrice Bellard.\n"
           " *\n"
           " * This code is released under the GNU General Public License version\n"
           " * 2. Please read the file COPYING to know the exact terms of the\n"
           " * license.\n"
           " */\n");

    printf("#include \"lm.h\"\n"
           "#include \"v34priv.h\"\n"
           "\n");

    /* tables for trellis coded modulation */
    gen_table(4);
    gen_table(8);
    gen_table(16);

    /* rx filters which convert the 8000 Hz flow to (symbol_rate * 3)
       with a nyquist filter */

    for(j=0;j<6;j++) {
        float symbol_rate, carrier, alpha, beta, freq;

        symbol_rate = 2400.0 * 
            (float)S_tab[j][0] / (float)S_tab[j][1];
        
        for(i=0;i<2;i++) {
            if (i == 1 &&
                S_tab[j][2] == S_tab[j][4] &&
                S_tab[j][3] == S_tab[j][5])
                break;

            carrier = symbol_rate * 
                (float)S_tab[j][2 + 2*i] / (float)S_tab[j][3 + 2*i];
            
            alpha = symbol_rate / (2.0 * symbol_rate * 3.0 * baud_tab[j][1]);
            beta = 0.1;
            freq = carrier / (symbol_rate * 3.0 * baud_tab[j][1]);
            n = RC_FILTER_SIZE * baud_tab[j][1] + 1;

            printf("/* S=%d carrier=%d alpha=%f beta=%f f0=%f */\n", 
                   (int)rint(symbol_rate), 
                   (int)rint(carrier), 
                   alpha, beta, freq);

            build_sqr_nyquist_filter(filter, freq, alpha, beta, n);
            
            sprintf(buf, "v34_rx_filter_%d_%d", 
                    (int)rint(symbol_rate),
                    (int)rint(carrier));

            write_filter(buf, filter, n);
        }
    }

    /* tx filters */

    for(j=0;j<6;j++) {
        float alpha, beta;

        alpha = 1.0 / (2.0 * baud_tab[j][1]);
        beta = 0.1;
        n = RC_FILTER_SIZE * baud_tab[j][1] + 1;
        
        build_sqr_nyquist_filter(filter, 0.0, alpha, beta, n);

        sprintf(buf, "v34_rc_%d_filter", baud_tab[j][1]);
        write_filter(buf, filter, n);
    }

    /* V22 filters */

    /* 600 sym/s for 8000 Hz, beta=0.75 */
    build_sqr_nyquist_filter(filter, 0.0, 1.0 / (2.0 * 40.0), 0.75, 
                             V22_TX_FILTER_SIZE);
    
    write_filter("v22_tx_filter", filter, V22_TX_FILTER_SIZE);

    return 0;
}
