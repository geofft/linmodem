/* 
 * Implementation of the V34 modulation/demodulation
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

void print_bits(int val, int n)
{
    int i;

    for(i=n-1;i>=0;i--) {
        putchar('0' + ((val >> i) & 1));
    }
}

void print_bit_vector(char *str, u8 *tab, int n)
{
  int i;
  printf("%s[%d]= ", str, n);
  for(i=n-1;i>=0;i--) putchar(tab[i] + '0');
  printf("\n");
}


static void agc_init(V34DSPState *s);
static void baseband_decode(V34DSPState *s, int si, int sq);
static void put_sym(V34DSPState *s, int si, int sq);

/* divide the bit sequence by poly */
#define SCRAMBLER_DEG 23
#define V34_GPC       (1 | (1 << (23-18)))
#define V34_GPA       (1 | (1 << (23-5)))

int scramble_bit(V34DSPState *s, int b, int poly)
{
  int b1, reg;
 
  reg = s->scrambler_reg;

  b1 = (reg >> (SCRAMBLER_DEG-1)) ^ b;
  reg = (reg << 1) & ((1 << SCRAMBLER_DEG) - 1);
  if (b1)
    reg ^= poly;
  
  s->scrambler_reg = reg;

  return b1;
}

int unscramble_bit(V34DSPState *s, int b, int poly)
{
  int b1, reg;
 
  reg = s->scrambler_reg;

  b1 = (reg >> (SCRAMBLER_DEG-1)) ^ b;
  reg = (reg << 1) & ((1 << SCRAMBLER_DEG) - 1);
  if (b)
    reg ^= poly;
  
  s->scrambler_reg = reg;

  return b1;
}

/* build the constellation (no need to store it completely, we are lazy!) */

static int constellation_cmp(const void *a_ptr, const void *b_ptr)
{
  int x1,y1,x2,y2,d;
  x1= ((s8 *)a_ptr)[0];
  y1= ((s8 *)a_ptr)[1];
  x2= ((s8 *)b_ptr)[0];
  y2= ((s8 *)b_ptr)[1];
  
  d = (x1 * x1 + y1 * y1) - (x2 * x2 + y2 * y2) ;
  if (d != 0) 
    return d;
  else
    return y2 - y1;
}

#define rotate_clockwise(x, y, x1, y1, z)\
{\
      switch(z) {\
      case 0:\
        x = x1;\
        y = y1;\
        break;\
      case 1:\
        x = -y1;\
        y = x1;\
        break;\
      case 2:\
        x = -x1;\
        y = -y1;\
        break;\
      default:\
        x = y1;\
        y = -x1;\
        break;\
      }\
}

static void build_constellation(V34DSPState *s)
{
  int x,y,i,j,k;

  k = 0;
  for(y=C_MIN; y<= C_MAX; y++) {
      for(x=C_MIN; x<= C_MAX; x++) {
          s->constellation[k][0] = 4*x+1;
          s->constellation[k][1] = 4*y+1;
          k++;
      }
  }

  /* now sort the constellation */
  qsort(s->constellation, C_MAX_SIZE, 2, constellation_cmp);

#if 0
  for(i=0;i<L_MAX/4;i++) printf("%d: x=%d y=%d\n",
                            i, s->constellation[i][0], s->constellation[i][1]);
#endif

  /* build the table for the decoder (not the best table, the corners
     are not ok) */
  
  memset(s->constellation_to_code, 0, sizeof(s->constellation_to_code));
  for(j=0;j<4;j++) {
    for(i=0;i<L_MAX/4;i++) {
      int x1,y1;

      x1 = s->constellation[i][0];
      y1 = s->constellation[i][1];

      rotate_clockwise(x, y, x1, y1, j);

      x = (x + C_RADIUS) >> 1;
      y = (y + C_RADIUS) >> 1;
      
      s->constellation_to_code[x][y] = i | (j << 14);
    }
  }
}

/* index to ring utilities */

static inline int g2(V34DSPState *st, int p, int m)
{
  if (p >= 0 && p <= 2*(m-1))
    return m - abs(p-(m-1));
  else {
    return 0;
  }
}

static inline int g4(V34DSPState *st, int p, int m)
{
  int s,i;
  
  s = 0;
  if (p >= 0 && p <= 4*(m-1)) {
    for(i=0;i<=p;i++) s += st->g2_tab[i] * st->g2_tab[p-i];
  }
  return s;
}

static inline int g8(V34DSPState *st, int p, int m)
{
  int s,i;

  s = 0;
  if (p >= 0 && p <= 8*(m-1)) {
    for(i=0;i<=p;i++) s += st->g4_tab[i] * st->g4_tab[p-i];
  }
  return s;
}

static void index_to_rings(V34DSPState *s, int ring[4][2], int r0)
{
  int a,b,c,d,e,f,g,h,r1,r2,r3,r4,r5,tmp,m;
  
  m = s->M;

  a = -1;
  r1 = 0;
  for(;;) {
    tmp = r0 - s->z8_tab[a+1];
    if (tmp < 0) break;
    r1 = tmp;
    a++;
  }
  
  b = 0;
  for(;;) {
    tmp = r1 - s->g4_tab[b] * s->g4_tab[a-b];
    if (tmp < 0) break;
    r1 = tmp;
    b++;
  }
  
  tmp = s->g4_tab[b];
  r2 = r1 % tmp;
  r3 = (r1 - r2) / tmp;

  c = 0;
  r4 = r2;
  for(;;) {
    tmp = r4 - s->g2_tab[c] * s->g2_tab[b-c];
    if (tmp < 0) break;
    r4 = tmp;
    c++;
  }

  d = 0;
  r5 = r3;
  for(;;) {
    tmp = r5 - s->g2_tab[d] * s->g2_tab[a-b-d];
    if (tmp < 0) break;
    r5 = tmp;
    d++;
  }

  tmp = s->g2_tab[c];
  e = r4 % tmp;
  f = (r4 - e) / tmp;
  
  tmp = s->g2_tab[d];
  g = r5 % tmp;
  h = (r5 - g) / tmp;
  
  if (c < m) {
    ring[0][0] = e;
    ring[0][1] = c - ring[0][0];
  } else {
    ring[0][1] = m - 1 - e;
    ring[0][0] = c - ring[0][1];
  }
    
  if ((b-c) < m) {
    ring[1][0] = f;
    ring[1][1] = b - c - ring[1][0];
  } else {
    ring[1][1] = m - 1 - f;
    ring[1][0] = b - c - ring[1][1];
  }
  
  if (d < m) {
    ring[2][0] = g;
    ring[2][1] = d - ring[2][0];
  } else {
    ring[2][1] = m - 1 - g;
    ring[2][0] = d - ring[2][1];
  }

  if ((a-b-d) < m) {
    ring[3][0] = h;
    ring[3][1] = a - b - d - ring[3][0];
  } else {
    ring[3][1] = m - 1 - h;
    ring[3][0] = a - b - d - ring[3][1];
  }
}

/* return the K bit index corresponding to the rings */
static int rings_to_index(V34DSPState *s, int ring[4][2])
{
  int a,b,c,d,e,f,g,h,r0,r1,r2,r3,r4,r5,m,i;

  m = s->M;

  /* find back the parameters */
  c = ring[0][0] + ring[0][1];
  if (c < m) e = ring[0][0]; else e = m - 1 - ring[0][1];
  
  b = ring[1][0] + ring[1][1];
  if (b < m) f = ring[1][0]; else f = m - 1 - ring[1][1];
  b += c;
  
  d = ring[2][0] + ring[2][1];
  if (d < m) g = ring[2][0]; else g = m - 1 - ring[2][1];
  
  a = ring[3][0] + ring[3][1];
  if (a < m) h = ring[3][0]; else h = m - 1 - ring[3][1];
  a += b + d;

  r5 = h * s->g2_tab[d] + g;
  r4 = f * s->g2_tab[c] + e;
  
  r3 = r5;
  for(i=0;i<d;i++) r3 += s->g2_tab[i] * s->g2_tab[a-b-i];
  
  r2 = r4;
  for(i=0;i<c;i++) r2 += s->g2_tab[i] * s->g2_tab[b-i];

  r1 = r3 * s->g4_tab[b] + r2;
  
  for(i=0;i<b;i++) r1 += s->g4_tab[i] * s->g4_tab[a-i];

  r0 = r1 + s->z8_tab[a];

  return r0;
}

/* initialize the g2, g4, g8 & z8 tables */
static void build_rings(V34DSPState *s)
{
  int n,i,m;
  m = s->M;
  n = 8*(m - 1) + 1;
  for(i=0;i<n;i++) s->g2_tab[i] = g2(s,i,m);
  for(i=0;i<n;i++) s->g4_tab[i] = g4(s,i,m);
  for(i=0;i<n;i++) s->g8_tab[i] = g8(s,i,m);
  
  s->z8_tab[0] = 0;
  for(i=1;i<n;i++) {
    s->z8_tab[i] = s->z8_tab[i-1] + s->g8_tab[i-1];
  }

#if 0
  {
    int i;
    for(i=0;i<=(1 << s->K);i++) {
      int m[4][2],j, r1,r0;
      r0 = random() % (1 << s->K);
      index_to_rings(s, m, r0);
      r1 = rings_to_index(s, m);
      printf("%d:", r0);
      for(j=0;j<4;j++) printf(" %d %d",m[j][0], m[j][1]);
      printf("\n");
      if (r0 != r1) {
        printf("error r0=%d r1=%d\n" , r0, r1);
        exit(1);
      }
    }
  }
#endif

}

/* parameters for each symbol rate */
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

static s16 *rc_filter[6] = {
    v34_rc_10_filter,
    v34_rc_35_filter,
    v34_rc_20_filter,
    v34_rc_8_filter,
    v34_rc_5_filter,
    v34_rc_7_filter,
};
    
static void build_tx_filter(V34DSPState *s)
{
    /* sampled at every symbol */
    
    s->tx_filter = rc_filter[s->S];
    s->baud_incr = s->symbol_rate * (float)0x10000 / (float)V34_SAMPLE_RATE;
    s->baud_phase = 4;

    s->carrier_phase = 0;
    s->carrier_incr = s->carrier_freq * (float)0x10000 / (float)V34_SAMPLE_RATE;
    /* init TX fifo */
    s->tx_filter_wsize = RC_FILTER_SIZE;
    s->tx_buf_ptr = 0;
    s->tx_outbuf_ptr = s->tx_filter_wsize;
    s->tx_buf_size = 0;
}

float hilbert[156] = 
{
/* alpha=0.000000 beta=0.000000 */
  0.0000000000e+00,
  1.0438059849e-03,
  0.0000000000e+00,
  1.1113855722e-03,
  0.0000000000e+00,
  1.2232369871e-03,
  0.0000000000e+00,
  1.3825587134e-03,
  0.0000000000e+00,
  1.5926453386e-03,
  0.0000000000e+00,
  1.8569074639e-03,
  0.0000000000e+00,
  2.1788971424e-03,
  0.0000000000e+00,
  2.5623400268e-03,
  0.0000000000e+00,
  3.0111756977e-03,
  0.0000000000e+00,
  3.5296080377e-03,
  0.0000000000e+00,
  4.1221680212e-03,
  0.0000000000e+00,
  4.7937919718e-03,
  0.0000000000e+00,
  5.5499192449e-03,
  0.0000000000e+00,
  6.3966145267e-03,
  0.0000000000e+00,
  7.3407216217e-03,
  0.0000000000e+00,
  8.3900579330e-03,
  0.0000000000e+00,
  9.5536620984e-03,
  0.0000000000e+00,
  1.0842111879e-02,
  0.0000000000e+00,
  1.2267936058e-02,
  0.0000000000e+00,
  1.3846153846e-02,
  0.0000000000e+00,
  1.5594989773e-02,
  0.0000000000e+00,
  1.7536833977e-02,
  0.0000000000e+00,
  1.9699551684e-02,
  0.0000000000e+00,
  2.2118299263e-02,
  0.0000000000e+00,
  2.4838091053e-02,
  0.0000000000e+00,
  2.7917505894e-02,
  0.0000000000e+00,
  3.1434171201e-02,
  0.0000000000e+00,
  3.5493106154e-02,
  0.0000000000e+00,
  4.0239829657e-02,
  0.0000000000e+00,
  4.5881743462e-02,
  0.0000000000e+00,
  5.2724604849e-02,
  0.0000000000e+00,
  6.1238171887e-02,
  0.0000000000e+00,
  7.2182437365e-02,
  0.0000000000e+00,
  8.6871563629e-02,
  0.0000000000e+00,
  1.0778971907e-01,
  0.0000000000e+00,
  1.4026261876e-01,
  0.0000000000e+00,
  1.9814073999e-01,
  0.0000000000e+00,
  3.3221536070e-01,
  0.0000000000e+00,
  9.9962693916e-01,
  0.0000000000e+00,
 -9.9962693916e-01,
 -0.0000000000e+00,
 -3.3221536070e-01,
 -0.0000000000e+00,
 -1.9814073999e-01,
 -0.0000000000e+00,
 -1.4026261876e-01,
 -0.0000000000e+00,
 -1.0778971907e-01,
 -0.0000000000e+00,
 -8.6871563629e-02,
 -0.0000000000e+00,
 -7.2182437365e-02,
 -0.0000000000e+00,
 -6.1238171887e-02,
 -0.0000000000e+00,
 -5.2724604849e-02,
 -0.0000000000e+00,
 -4.5881743462e-02,
 -0.0000000000e+00,
 -4.0239829657e-02,
 -0.0000000000e+00,
 -3.5493106154e-02,
 -0.0000000000e+00,
 -3.1434171201e-02,
 -0.0000000000e+00,
 -2.7917505894e-02,
 -0.0000000000e+00,
 -2.4838091053e-02,
 -0.0000000000e+00,
 -2.2118299263e-02,
 -0.0000000000e+00,
 -1.9699551684e-02,
 -0.0000000000e+00,
 -1.7536833977e-02,
 -0.0000000000e+00,
 -1.5594989773e-02,
 -0.0000000000e+00,
 -1.3846153846e-02,
 -0.0000000000e+00,
 -1.2267936058e-02,
 -0.0000000000e+00,
 -1.0842111879e-02,
 -0.0000000000e+00,
 -9.5536620984e-03,
 -0.0000000000e+00,
 -8.3900579330e-03,
 -0.0000000000e+00,
 -7.3407216217e-03,
 -0.0000000000e+00,
 -6.3966145267e-03,
 -0.0000000000e+00,
 -5.5499192449e-03,
 -0.0000000000e+00,
 -4.7937919718e-03,
 -0.0000000000e+00,
 -4.1221680212e-03,
 -0.0000000000e+00,
 -3.5296080377e-03,
 -0.0000000000e+00,
 -3.0111756977e-03,
 -0.0000000000e+00,
 -2.5623400268e-03,
 -0.0000000000e+00,
 -2.1788971424e-03,
 -0.0000000000e+00,
 -1.8569074639e-03,
 -0.0000000000e+00,
 -1.5926453386e-03,
 -0.0000000000e+00,
 -1.3825587134e-03,
 -0.0000000000e+00,
 -1.2232369871e-03,
 -0.0000000000e+00,
 -1.1113855722e-03,
 -0.0000000000e+00,
 -1.0438059849e-03,
};


s16 *v34_rx_filters[12] = {
    v34_rx_filter_2400_1600,
    v34_rx_filter_2400_1800,
    v34_rx_filter_2743_1646,
    v34_rx_filter_2743_1829,
    v34_rx_filter_2800_1680,
    v34_rx_filter_2800_1867,
    v34_rx_filter_3000_1800,
    v34_rx_filter_3000_2000,
    v34_rx_filter_3200_1829,
    v34_rx_filter_3200_1920,
    v34_rx_filter_3429_1959,
    v34_rx_filter_3429_1959,
};
    
static void build_rx_filter(V34DSPState *s)
{
    float a, f_low, f_high;
    int i;


    s->rx_filter = v34_rx_filters[s->S * 2 + s->use_high_carrier];

    /* XXX: temporary hack to synchronize */
    if (s->S == V34_S3429)
        s->baud_phase = 1;
    else
        s->baud_phase = 2;
    s->baud_num = (s->baud_num * 3);

    s->carrier_incr = s->carrier_freq * (float)0x10000 / s->symbol_rate;
    s->carrier_phase = 0;
    s->rx_buf1_ptr = 0;
    s->rx_filter_wsize = (s->baud_denom * RC_FILTER_SIZE) / s->baud_num;
    printf("cincr=%d baudincr=%d\n", s->carrier_incr, s->baud_incr);

    s->baud_phase = s->baud_phase << 16;
    s->baud_num = s->baud_num << 16;
    s->baud_denom = s->baud_denom << 16;

    /* equalizer : init to identity */
    s->eq_filter[EQ_SIZE/2][0] = 0x4000 << 16;
    /* XXX: hilbert normalization ? */
    for(i=0;i<EQ_SIZE;i++) 
        s->eq_filter[i][1] = (int)(hilbert[i] * 0.61475 * (0x4000 << 16));
    
    /* adaptation shift : big at the beginning, should be small after. */
    s->eq_shift = 0;

    /* synchronization : Nyquist filters at the upper & lower frequencies */
    a = 0.99;
    f_low = 2 * M_PI * (s->carrier_freq - s->symbol_rate / 2.0) / 
        (3.0 * s->symbol_rate);
    f_high = 2 * M_PI * (s->carrier_freq + s->symbol_rate / 2.0) / 
        (3.0 * s->symbol_rate);
    printf("%f %f\n", f_low, f_high);

    s->sync_low_coef[0] = (int)(2 * a * cos(f_low) * 0x4000);
    s->sync_low_coef[1] = (int)( - a * a * 0x4000);

    s->sync_high_coef[0] = (int)(2 * a * cos(f_high) * 0x4000);
    s->sync_high_coef[1] = (int)(- a * a * 0x4000);

    /* precomputed constants to compute the cross correlation */
    s->sync_A = (int)( - a * a * sin(f_high - f_low) * 0x4000);
    s->sync_B = (int)( a * sin(f_high) * 0x4000);
    s->sync_C = (int)( - a * sin(f_low) * 0x4000);
}

int V34_init_low(V34DSPState *s, V34State *p, int transmit)
{
  int S,d,e;
  
  /* copy the params */
  s->calling = p->calling;
  s->S = p->S;
  s->expanded_shape = p->expanded_shape;
  s->R = p->R;
  if (p->use_aux_channel)
      s->R += 200;
  s->conv_nb_states = p->conv_nb_states;
  s->use_non_linear = p->use_non_linear;
  s->use_high_carrier = p->use_high_carrier;
  memcpy(s->h, p->h, sizeof(s->h));

  /* superframe & data frame size */
  S = s->S;
  if (!s->use_high_carrier) {
    d = S_tab[S][2];
    e = S_tab[S][3];
  } else {
    d = S_tab[S][4];
    e = S_tab[S][5];
  }
  s->symbol_rate = 2400.0 * (float)S_tab[S][0] / (float)S_tab[S][1];
  s->carrier_freq = s->symbol_rate * (float)d / (float)e;

  s->J = S_tab[S][6];
  s->P = S_tab[S][7];
  s->N = (s->R * 28) / (s->J * 100); 
  /* max length of a mapping frame (no need for table 8 as in the spec !) */
  s->b = s->N / s->P; 
  if ((s->b * s->P) < s->N) s->b++;
  s->r = s->N - (s->b - 1) * s->P;
  
  /* aux channel */
  if (p->use_aux_channel)
      s->W = 15 - s->J; /* no need to test as in the spec ! */
  else
      s->W = 0; /* no aux channel */

  /* mapping parameters */
  s->q = 0;
  if (s->b <= 12) {
    s->K = 0;
  } else {
    s->K = s->b - 12;
    while (s->K >= 32) {
      s->K -= 8;
      s->q++;
    }
  }

  /* XXX: use integer arith ! */
  if (!s->expanded_shape) {
    s->M = (int) ceil(pow(2.0, s->K / 8.0));
  } else {
    s->M = (int) rint(1.25 * pow(2.0, s->K / 8.0));
  }
  s->L = 4 * s->M * (1 << s->q);

#ifdef DEBUG
  printf("S_index=%d (S=%0.0f carrier=%0.0f)\n"
         "R=%d J=%d P=%d N=%d b=%d r=%d W=%d\n", 
         s->S, s->symbol_rate, s->carrier_freq,
         s->R, s->J, s->P, s->N, s->b, s->r, s->W);
  printf("K=%d q=%d M=%d L=%d\n", s->K, s->q, s->M, s->L);
#endif

  build_constellation(s);
  
  build_rings(s);

  s->baud_num = baud_tab[S][0];
  s->baud_denom = baud_tab[S][1];

  if (transmit) {
      build_tx_filter(s);
  } else {
      build_rx_filter(s);
      agc_init(s);
      s->phase_4d = 0;
  }

  s->Z_1 = 0;
  s->U0 = 0; /* trellis coder memory */
  memset(s->x, 0, sizeof(s->x));
  s->half_data_frame_count = 0;
  s->sync_count = 0;
  s->conv_reg = 0;
  s->scrambler_reg = 0;

  s->mapping_frame = 0; /* mapping frame counters */
  s->acnt = 0;
  s->rcnt = 0;

  return 0;
}

/* shift by n & round toward zero */ 
static inline int shr_round0(int val, int n)
{
  int offset;

  offset = (1 << (n-1)) - 1;
  if (val >= 0) {
    val = (val + offset) >> n;
  } else {
    val = -val;
    val = (val + offset) >> n;
    val = -val;
  }
  return val;
}

/* clamp a between -v & v */
static inline int clamp(int a, int v)
{
  if (a > v) 
    return v;
  else if (a < -v) 
    return -v;
  else
    return a;
}

/* compute the next state in the trellis (Y[0] is ignored) */
static int trellis_next_state(int conv_nb_states, int conv_reg, int trans)
{
    int i,Y0;
    int Y[5];
    
    Y[1] = (trans & 1);
    Y[2] = ((trans >> 1) & 1);
    Y[4] = ((trans >> 2) & 1);
    Y[3] = ((trans >> 3) & 1);

    Y0 = conv_reg & 1;
    switch(conv_nb_states) {
    case 16:
        /* figure 10 */
        conv_reg ^= (Y[1] << 1) | (Y[2] << 2) | ((Y[2] ^ Y0) << 3) | (Y0 << 4);
        conv_reg >>= 1;
        break;
    case 32:
        /* figure 11 */
        conv_reg ^= (Y[2] << 1) | (Y[4] << 2) | (Y[1] << 3) | (Y[2] << 4) | (Y0 << 5);
        conv_reg >>= 1;
        break;
    default:
    case 64:
        /* figure 12 */
        {
            int r[6], s[6], tmp1, tmp2;
            
            for(i=0;i<6;i++) r[i] = (conv_reg >> i) & 1;
            
            s[0] = r[1] ^ r[3] ^ Y[2];
            s[1] = r[0];
            s[2] = r[3];
            tmp2 = Y[1] ^ r[4];
            s[3] = r[3] ^ tmp2;
            tmp1 = r[4] ^ r[5];
            s[4] = r[2] ^ Y[3] ^ (r[3] & Y[2]) ^ tmp1;
            s[5] = Y[4] ^ tmp1 ^ (r[3] & tmp2);
            
            conv_reg = 0;
            for(i=0;i<6;i++) conv_reg |= s[i] << i;
        }
        break;
    }
    return conv_reg;
}

/* (§ 9.6.3) trellis encoder, return U0 */

static int trellis_encoder(V34DSPState *s, int c0, int yy[2][2])
{
  int v0, ss[2][3], Y[5], i, trans;

  /* convolutional coder */

  /* (§ 9.6.3.1) find Y vector. We traducted the table into binary expressions */
  for(i=0;i<2;i++) {
    int x,y,y0,x0,y1,x1;

    /* XXX: is it right to suppose that we use figure 9 as a periodic mapping ? */
    x = yy[i][0];
    y = yy[i][1];
    x = ((x + 3) >> 1) & 3;
    y = ((y + 3) >> 1) & 3;
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

  /* compute the next trellis state */
  trans = (Y[3] << 3) | (Y[4] << 2) | (Y[2] << 1) | Y[1];

  s->conv_reg = trellis_next_state(s->conv_nb_states, s->conv_reg, trans);
  Y[0] = s->conv_reg & 1;

  /* super frame synchronisation pattern */
  if (s->sync_count == 0) {
    v0 = (SYNC_PATTERN >> (15 - s->half_data_frame_count)) & 1;
  } else {
    v0 = 0;
  }

  return Y[0] ^ c0 ^ v0;
}


static int get_bit(V34DSPState *s)
{
    int b, poly;

    b = s->get_bit(s->opaque);
    if (b == -1) b = 1;
    if (s->calling)
        poly = V34_GPC;
    else
        poly = V34_GPA;
    b = scramble_bit(s, b, poly);
    return b;
}

/* auxilary channel bit */
static int aux_get_bit(V34DSPState *s)
{
    return 0;
}

/* encode size bits into the corresponding symbols of the constellation */
/* return exactly 8 baseband complex samples */
static void encode_mapping_frame(V34DSPState *s)
{
  int m[4][2], r0; /* rings */
  u8 I[3][4];
  int Q[4][2], Z[2], Y[2][2];
  u8 *ptr;
  int t,i,j,k,x,y,x1,y1,w,mp_size;
  int u_re, u_im, p_re, p_im, c_re, c_im, C0, x_re, x_im, xp_re, xp_im;
  u8 data[MAX_MAPPING_FRAME_SIZE];

  /* compute mapping frame size */
  s->rcnt += s->r;
  if (s->rcnt < s->P) {
      mp_size = s->b - 1;
  } else {
      s->rcnt -= s->P;
      mp_size = s->b;
  }
    
  /* send an auxilary channel bit if needed */
  s->acnt += s->W;
  if (s->acnt < s->P) {
      data[0] = get_bit(s);
  } else {
      s->acnt -= s->P;
      data[0] = aux_get_bit(s); 
  }

  for(i=1;i<mp_size;i++) data[i] = get_bit(s); 

  //  print_bit_vector("sent", data, mp_size);
  
  if (s->b <= 12) {
    /* (§ 9.3.2) simple case: no shell mapping */
    memset(m, 0, sizeof(m));
    for(i=0;i<s->b;i++) ((u8 *)I)[i] = data[i];
    for(i=s->b;i<12;i++) ((u8 *)I)[i] = 0;
    memset(Q, 0, sizeof(Q));
  } else {
    /* (§ 9.3.1) */
    /* leave one bit if low frame */
    int n;

    n = s->K;
    if (mp_size < s->b) n--;
    
    ptr = data;
    r0 = 0;
    for(i=0;i<n;i++) r0 |= *ptr++ << i;
    index_to_rings(s, m, r0);

    for(j=0;j<4;j++) {
      I[0][j] = ptr[0];
      I[1][j] = ptr[1];
      I[2][j] = ptr[2];
      ptr += 3;

      t = 0;
      for(i=0;i<s->q;i++) t |= *ptr++ << i;
      Q[j][0] = t;

      t = 0;
      for(i=0;i<s->q;i++) t |= *ptr++ << i;
      Q[j][1] = t;
    }
  }
  
  if (s->b < 56) 
    w = 1;
  else 
    w = 2;

  for(j=0;j<4;j++) {
    /* for each 4D symbol */

    /* (§ 9.5) differential coding */
    Z[0] = (I[1][j] + 2 * I[2][j] + s->Z_1) & 3;
    s->Z_1 = Z[0];
    
    /* (§ 9.6.1) mapping to 2D symbols */
    Z[1] = (Z[0] + 2 * I[0][j] + s->U0) & 3;

    C0 = 0; /* for trellis coding */
    for(i=0;i<2;i++) {
      t = Q[j][i] + (m[j][i] << s->q);

      assert(t >= 0 && t < L_MAX/4);
      x1 = s->constellation[t][0];
      y1 = s->constellation[t][1];
      /* rotation by Z[i] * 90 degress clockwise */
      rotate_clockwise(x, y, x1, y1, Z[i]);
      u_re = x;
      u_im = y;

      /* (§ 9.6.2) precoder */
      x = 0;
      y = 0;
      for(k=0;k<3;k++) {
        x += s->x[k][0] * s->h[k][0] - s->x[k][1] * s->h[k][1];
        y += s->x[k][1] * s->h[k][0] + s->x[k][0] * s->h[k][1];
      }
      /* round to 2^-7 */
      p_re = shr_round0(x, 14);
      p_im = shr_round0(y, 14);
      
      /* compute c(n) */
      c_re = shr_round0(p_re, 7 + w) << w;
      c_im = shr_round0(p_im, 7 + w) << w;
      C0 += c_re + c_im;

      Y[i][0] = clamp(u_re + c_re, 255);
      Y[i][1] = clamp(u_im + c_im, 255);
      
      x_re = (Y[i][0] << 7) - p_re;
      x_im = (Y[i][1] << 7) - p_im;

      for(k=2;k>=1;k--) {
        s->x[k][0] = s->x[k-1][0];
        s->x[k][1] = s->x[k-1][1];
      }
      s->x[0][0] = x_re;
      s->x[0][1] = x_im;

      /* (§ 9.7) non linear encoder */

      if (!s->use_non_linear) {
        xp_re = x_re;
        xp_im = x_im;
      } else {
        int x2;
        float dzeta,theta;
        /* XXX: average power ? */

        x2 = (x_re * x_re + x_im * x_im) >> 7;
        dzeta = 0.3125 /* x2 / 128.0 */;
        theta = (1 + dzeta / 6.0 + dzeta * dzeta / 120.0);
        
        xp_re = rint(theta * x_re);
        xp_im = rint(theta * x_im);
      }

      put_sym(s, xp_re, xp_im);
    }
    s->U0 = trellis_encoder(s, (C0 >> 1) & 1, Y);

    /* 4D symbol count & data frame count for synchronisation */
    if (++s->sync_count == 2*s->P) {
      s->sync_count = 0;
      if (++s->half_data_frame_count == 2*s->J) {
        s->half_data_frame_count = 0;
      }
    }
  }

  if (++s->mapping_frame >= s->P) {
      /* new data frame */
      s->mapping_frame = 0;
      s->rcnt = 0;
      s->acnt = 0;
  }
}


/* put a new baseband symbol in the tx queue */
static void put_sym(V34DSPState *s, int si, int sq)
{
    s->tx_buf[s->tx_buf_ptr][0] = (si * s->tx_amp) >> 7;
    s->tx_buf[s->tx_buf_ptr][1] = (sq * s->tx_amp) >> 7;

    s->tx_buf_ptr = (s->tx_buf_ptr + 1) & (TX_BUF_SIZE - 1);
    s->tx_buf_size++;

    assert(s->tx_buf_size <= TX_BUF_SIZE);
}


/* write at most nb samples, return the number of samples
   written. Stops if no more baseband symbols in tx queue */
static int V34_baseband_to_carrier(V34DSPState *s, 
                                   s16 *samples, unsigned int nb)
{
    int si, sq, ph, i, j, k;

    for(i=0;i<nb;i++) {
        /* is there enough symbols in the queue ? */
        if (s->tx_buf_size < s->tx_filter_wsize)
            break;

        /* apply the spectrum shaping filter */
        ph = s->baud_phase;
        si = sq = 0;
        for(j=0;j<s->tx_filter_wsize;j++) {
            k = (s->tx_outbuf_ptr - j - 1) & 
                (TX_BUF_SIZE - 1);
            si += s->tx_buf[k][0] * s->tx_filter[ph];
            sq += s->tx_buf[k][1] * s->tx_filter[ph];
            ph += s->baud_denom;
        }
        si = si >> 14;
        sq = sq >> 14;
        if ( si != (short)si || sq != (short)sq) {
            printf("error %d %d\n", si, sq);
        }
        // printf("M: phase=%04X %d %d\n", s->baud_phase, si, sq);
        /* get next baseband symbols */
        s->baud_phase += s->baud_num;
        if (s->baud_phase >= s->baud_denom) {
            s->baud_phase -= s->baud_denom;
            s->tx_outbuf_ptr = (s->tx_outbuf_ptr + 1) & (TX_BUF_SIZE - 1);
            s->tx_buf_size--;
        }
        
        /* center on the carrier */
        samples[i] = (si * dsp_cos(s->carrier_phase) - 
            sq * dsp_cos((PHASE_BASE/4) - s->carrier_phase)) >> COS_BITS;
        s->carrier_phase += s->carrier_incr;
    }
    return i;
}

#define S_POWER     (1 + 1)
#define TRN4_POWER  (1 + 1)
#define TRN16_POWER ((1 + 1 + 2*(9 + 1) + 9 + 9) / 4.0)

/* compute the normalized amplitude */
#define CALC_AMP(x) (int)( (128.0 * 128.0) / sqrt(x) )

/* send the V34 S sequence, duration: 128 T */
static void V34_send_S(V34DSPState *s)
{
    int i;

    /* transmit amplitude multiplier */
    s->tx_amp = CALC_AMP(S_POWER);

    for(i=0;i<64;i++) {
        put_sym(s, 128, 128); /* 0 deg */
        put_sym(s, -128, 128); /* -90 deg */
    }
}

/* send the V34 S bar sequence, duration: 16 T */
static void V34_send_Sinv(V34DSPState *s)
{
    int i;

    s->tx_amp = CALC_AMP(S_POWER);
    for(i=0;i<8;i++) {
        put_sym(s, -128, -128); /* 180 deg */
        put_sym(s, 128, -128); /* 90 deg */
    }
}

/* send the PP sequence, duration: 288 T */
static void V34_send_PP(V34DSPState *s)
{
    int k,p;

    s->tx_amp = 128;

    /* 6 periods */
    for(p=0;p<6;p++) {
        for(k=0;k<V34_PP_SIZE;k++) {
            put_sym(s, tabPP[k].re, tabPP[k].im);
        }
    }
}

/* send the TRN sequence, (4 or 16 states), sent for 2048 T (XXX: this
   time was choosen randomly */
static void V34_send_TRN(V34DSPState *s)
{
    int i,x,y,x1,y1,z,poly,I1,I2,Q1,Q2,q;

    if (s->calling)
        poly = V34_GPC;
    else
        poly = V34_GPA;

    if (s->is_16states) 
        s->tx_amp = CALC_AMP(TRN16_POWER);
    else
        s->tx_amp = CALC_AMP(TRN4_POWER);
        
    for(i=0;i<1024;i++) {
        I1 = scramble_bit(s, 1, poly);
        I2 = scramble_bit(s, 1, poly);
        if (s->is_16states) {
            Q1 = scramble_bit(s, 1, poly);
            Q2 = scramble_bit(s, 1, poly);
            q = (Q2 << 1) | Q1;
        } else {
            q = 0;
        }
        x1 = s->constellation[q][0] << 7;
        y1 = s->constellation[q][1] << 7;
        z = (I2 << 1) | I1;
        rotate_clockwise(x, y, x1, y1, z);
        put_sym(s, x, y);
    }
    s->Z_1 = z; /* the last value z is used for the next sequence */
}

void put_bits(u8 **pp, int n, int bits)
{
    u8 *p;
    int i;

    p = *pp;
    for(i=n-1;i>=0;i--) {
        *p++ = (bits >> i) & 1;
    }
    *pp = p;
}

/* from § 10.1.2.3.2 */
int calc_crc(u8 *buf, int size)
{
    int crcinv, crc,i,b;

    crc = 0xffff;
    
    for(i=0;i<size;i++) {
        b = (crc & 1) ^ buf[i];
        crc = (crc >> 1) ^ ((b << 15) | (b << 10) | (b << 3));
    }

    /* invert the order of the crc bits (could be done while
       computing, but who cares ? */
    crcinv = 0;
    for(i=0;i<16;i++) {
        crcinv |= ((crc >> i) & 1) << (15-i);
    }
    return crcinv;
}

/* modulate an MP sequence */
static void V34_mod_MP(V34DSPState *s, u8 *buf, int size, int is_16states)
{
    int x,y,x1,y1,z,poly,I1,I2,Q1,Q2,q;
    u8 *p;
    
    if (s->calling)
        poly = V34_GPC;
    else
        poly = V34_GPA;

    if (is_16states) 
        s->tx_amp = CALC_AMP(TRN16_POWER);
    else
        s->tx_amp = CALC_AMP(TRN4_POWER);
        
    p = buf;
    z = s->Z_1;
    while ((p - buf) < size) {
        I1 = scramble_bit(s, *p++, poly);
        I2 = scramble_bit(s, *p++, poly);
        if (is_16states) {
            Q1 = scramble_bit(s, *p++, poly);
            Q2 = scramble_bit(s, *p++, poly);
            q = (Q2 << 1) | Q1;
        } else {
            q = 0;
        }
        x1 = s->constellation[q][0] << 7;
        y1 = s->constellation[q][1] << 7;
        z = (((I2 << 1) | I1) + z) & 3;
        rotate_clockwise(x, y, x1, y1, z);
        put_sym(s, x, y);
    }
    s->Z_1 = z;
}


/* send MP sequence. 'type' select its type (0 or 1). 'do_ack' selects
   if it is an acknowledge sequence */
static void V34_send_MP(V34DSPState *s, int type, int do_ack)
{
    u8 buf[188],*p;
    int i,j,crc;

    p = buf;
    put_bits(&p, 17, 0x1ffff); /* frame sync */
    put_bits(&p, 1, 0); /* start bit */
    put_bits(&p, 1, type); /* type: 1 */
    put_bits(&p, 1, 0); /* reserved */
    put_bits(&p, 4, 12); /* call to answer max rate: 28800 */
    put_bits(&p, 4, 12); /* answer to call max rate: 28800 */
    put_bits(&p, 1, 0); /* no aux channel */
    put_bits(&p, 2, 0); /* 0=16 state trellis */
    put_bits(&p, 1, 0); /* non linear encoder disabled */
    put_bits(&p, 1, 0); /* constellation shaping, 0=minimum, 1=expanded */
    put_bits(&p, 1, do_ack); /* acknowledge bit */
    
    put_bits(&p, 1, 0); /* start bit */
    put_bits(&p, 15, 0x7ff8); /* all rates enabled up to 28800 */
    put_bits(&p, 1, 1); /* asymetric data rate enable */
    
    if (type == 1) {
        for(i=0;i<3;i++) {
            for(j=0;j<2;j++) {
                put_bits(&p, 1, 0); /* start bit */
                put_bits(&p, 16, s->h[i][j]); /* precoding coef */
            }
        }
    }
                     
    put_bits(&p, 1, 0); /* start bit */
    put_bits(&p, 16, 0); /* reserved by ITU */

    put_bits(&p, 1, 0); /* start bit */

    crc = calc_crc(buf + 17, p - (buf+17));
    put_bits(&p, 16, crc);

    put_bits(&p, 1, 0); /* fill bit */
    if (type == 0) {
        put_bits(&p, 2, 0); /* fill bit */
    }

    /* now we transmit the buffer */
    V34_mod_MP(s, buf, p - buf, s->is_16states);
}

/* send E sequence */
static void V34_send_E(V34DSPState *s)
{
    u8 buf[20];

    memset(buf, 1, 20);

    V34_mod_MP(s, buf, 20, s->is_16states);
}

/* J sequence */
#define J4POINTS   0x0991
#define J16POINTS  0x0D91
#define JEND       0xF991

static void V34_send_J(V34DSPState *s, int length)
{
    int i,val;
    u8 buf[16],*p;

    if (s->is_16states) 
        val = J16POINTS;
    else
        val = J4POINTS;
    p = buf;
    put_bits(&p, 16, val);

    for(i=0;i<length;i++) {
        V34_mod_MP(s, buf, 16, 0); /* use 4 state modulation */
    }
}

static void V34_send_JP(V34DSPState *s)
{
    u8 buf[16],*p;

    p = buf;
    put_bits(&p, 16, JEND);
    
    V34_mod_MP(s, buf, 16, 0);
}



static void V34_mod(V34DSPState *s, s16 *samples, unsigned int nb)
{
    int n;

    for(;;) {
        /* modulate the symbols in the TX queue */
        switch(s->state) {
        case V34_STARTUP3_WAIT_J:
            /* silence */
            if (nb > 0) {
                *samples++ = 0;
                nb--;
            }
            break;
        
#if 0
        case V34_START:
            /* 600 bps DPSK modulation */
            

#endif
        default:
            /* V34 modulation */
            while (s->tx_buf_size >= s->tx_filter_wsize && nb > 0) {
                n = V34_baseband_to_carrier(s, samples, nb);
                samples += n;
                nb -= n;
            }
            break;
        }

        if (nb == 0) break;

        /* protocol state machine */

        switch(s->state) {
#if 0
            /* phase 2 */
        case V34_START:
            V34_send_info0(s, 0);
            break;
#endif

            /* phase 3 */
        case V34_STARTUP3_S1:
            V34_send_S(s);
            s->state = V34_STARTUP3_SINV1;
            break;
        case V34_STARTUP3_SINV1:
            V34_send_Sinv(s);
            s->state = V34_STARTUP3_S2;
            break;
        case V34_STARTUP3_S2:
            V34_send_S(s);
            s->state = V34_STARTUP3_SINV2;
            break;
        case V34_STARTUP3_SINV2:
            V34_send_Sinv(s);
            s->state = V34_STARTUP3_PP;
            break;
        case V34_STARTUP3_PP:
            V34_send_PP(s);
            s->state = V34_STARTUP3_TRN;
            break;
        case V34_STARTUP3_TRN:
            V34_send_TRN(s);
            //            s->state = V34_STARTUP3_J;
            break;
        case V34_STARTUP3_J:
            V34_send_J(s, 10); /* XXX: must wait for S on other end */
            if (s->calling) {
                s->state = V34_STARTUP3_JP;
            } else {
                /* the anwser modem wait for the S signal */
                s->state = V34_STARTUP3_WAIT_J;
            }
            break;
        case V34_STARTUP3_JP:
            V34_send_JP(s); 
            //s->state = V34_STARTUP4_TRN;
            s->state = V34_STARTUP3_S1;
            break;

        case V34_STARTUP3_WAIT_J:
            if (s->J_received)
                s->state = V34_STARTUP4_S;
            break;
                
            /* phase 4 */
        case V34_STARTUP4_S:
            V34_send_S(s);
            s->state = V34_STARTUP4_WAIT_JP;
            break;
        case V34_STARTUP4_WAIT_JP:
            if (s->JP_received)
                s->state = V34_STARTUP4_SINV;
            break;
        case V34_STARTUP4_SINV:
            V34_send_S(s);
            s->state = V34_STARTUP4_TRN;
            break;

        case V34_STARTUP4_TRN:
            V34_send_TRN(s);
            s->state = V34_STARTUP4_MP;
            break;
        case V34_STARTUP4_MP:
            V34_send_MP(s, 1, 0); /* type 1, no ack */
            s->state = V34_STARTUP4_MPP;
            break;
        case V34_STARTUP4_MPP:
            V34_send_MP(s, 1, 1); /* type 1, ack */
            s->state = V34_STARTUP4_E;
            break;
        case V34_STARTUP4_E:
            V34_send_E(s);
            s->state = V34_DATA;
            break;
        case V34_DATA:
            /* compute the next 8 baseband symbols */
            encode_mapping_frame(s);
            break;
        }
    }
}

static void V34_mod_init(V34DSPState *s, V34State *p)
{
    V34_init_low(s, p, 1);
    s->state = V34_STARTUP3_S1;
    s->JP_received = 1;
    //    s->state = V34_DATA;
    s->is_16states = 0; /* use 16 states */
}

/*****************************************************/
/* Here begins the real fun ! */

static inline int tcm_decision(int level, int sample)
{
    int x, xs;

    /* find the 4x4 subset */
    x = (sample + (7 * 128) - (level << 8)) >> 10;
    /* convert back to sample */
    xs = (((x<<2) + level - 2) << 8) + 128;
    return xs;
}

static int tcm_dist(int level, int sample)
{
    int xs, e;

    xs = tcm_decision(level,sample);

    e = sample - xs;
#if 0
    printf("level=%d sample=%d(%d)  xs=%d e=%d e1=%d e2=%d\n", 
           level, sample, (sample >> 8) * 2 + 1, xs, e, 
           sample - (xs + 1024), sample-(xs - 1024));
#endif
    return (e*e) >> 8;
}

/* Viterbi decoder for the V34 trellis coded modulation */

static void trellis_decoder(V34DSPState *s, s16 yout[2][2], s16 yy[2][2], 
                             int *mse)
{
    int i, j, k, n, nbbt, nb_trans, state, next_state, error, trellis_ptr;
    int error_table[32],decision_table[32],emin,jmin,u0,x,y;
    u8 *p,*q;

    trellis_ptr = s->trellis_ptr;

    /* compute the number of bits used in the transitions from each state */
    switch(s->conv_nb_states) {
    case 16:
        nbbt = 2;
        p = &trellis_trans_4[0][0];
        break;
    case 32:
        nbbt = 3;
        p = &trellis_trans_8[0][0];
        break;
    default:
        nbbt = 4;
        p = &trellis_trans_16[0][0];
        break;
    }
    nb_trans = 1 << nbbt;

    /* write a previous decoded symbol : extract a decoded bit from
       the beginning of a path */

    k = trellis_ptr;
    k--;
    if (k < 0) k = TRELLIS_LENGTH-1;
    j = 0; /* arbitrary path selected : all the paths converge to same
              decoded bits*/
    for(i=0;i<(TRELLIS_LENGTH-1);i++) {
        j = s->state_path[j][k];
        k--;
        if (k < 0) k = TRELLIS_LENGTH-1;
    }
    u0 = s->u0_memory[trellis_ptr];
    q = p + (s->state_decision[j][k] * 4);
#if 1
    yout[0][0] = tcm_decision(q[0], s->state_memory[trellis_ptr][0]);
    yout[0][1] = tcm_decision(q[1], s->state_memory[trellis_ptr][1]);
    yout[1][0] = tcm_decision(q[2], s->state_memory[trellis_ptr][2]);
    yout[1][1] = tcm_decision(q[3], s->state_memory[trellis_ptr][3]);
    /* undo the rotation */    
    if ((s->state_decision[j][k] >> 7)) {
        x = yout[1][1];
        y = - yout[1][0];
        yout[1][0] = x;
        yout[1][1] = y;
    }
    /* rotate only if u0 is set */
    if (u0 ^ (s->state_decision[j][k] >> 7)) {
        x = - yout[1][1];
        y = yout[1][0];
        yout[1][0] = x;
        yout[1][1] = y;
    }
#else
    /* no trellis */
    yout[0][0] = s->state_memory[trellis_ptr][0];
    yout[0][1] = s->state_memory[trellis_ptr][1];
    yout[1][0] = s->state_memory[trellis_ptr][2];
    yout[1][1] = s->state_memory[trellis_ptr][3];
#endif
    /* compute the mean square error (normalized to 2^7) */
    *mse = (dsp_sqr(yout[0][0] - s->state_memory[trellis_ptr][0]) + 
        dsp_sqr(yout[0][1] - s->state_memory[trellis_ptr][1]) + 
        dsp_sqr(yout[1][0] - s->state_memory[trellis_ptr][2]) + 
        dsp_sqr(yout[1][1] - s->state_memory[trellis_ptr][3])) >> 7;

    s->state_memory[trellis_ptr][0] = yy[0][0];
    s->state_memory[trellis_ptr][1] = yy[0][1];
    s->state_memory[trellis_ptr][2] = yy[1][0];
    s->state_memory[trellis_ptr][3] = yy[1][1];

    /* compute the error table */
    /* XXX: may be optimized by using the algebraic properties of the mapping */    
    n = 128 >> nbbt;
    jmin = 0; /* no warning */
    for(i=0;i<(nb_trans*2);i++) {
        emin = 0x7fffffff;
        for(j=0;j<n;j++) {
            int e;
            e = tcm_dist(p[0], yy[0][0]) + 
                tcm_dist(p[1], yy[0][1]) +
                tcm_dist(p[2], yy[1][0]) +
                tcm_dist(p[3], yy[1][1]);
            if (e < emin) {
                emin = e;
                jmin = j;
            }
            p+=4;
        }
        error_table[i] = emin;
        decision_table[i] = i * n + jmin;
    }

    /* we compute the bit u0 (needed for synchronization & c0 estimation) */
    jmin = 0;
    emin = 0x7fffffff;
    for(i=0;i<nb_trans*2;i++) {
        if (error_table[i] < emin) {
            jmin = i;
            emin = error_table[i];
        }
    }
    s->u0_memory[trellis_ptr] = (jmin >= nb_trans);

    /* init the error table to +infinity */
    for(state=0;state<s->conv_nb_states;state++) {
        s->state_error1[state] = 0x7fffffff;
    }

    for(state=0;state<s->conv_nb_states;state++) {
        /* select the value of y0 depending on the current state */
        /* for each state, we update the next state entry by selecting
           the shortest path */
        /* XXX: should handle error overflow */
        if (state & 1)
            n = nb_trans;
        else 
            n = 0;
        for(j=0;j<nb_trans;j++) {
            next_state = trellis_next_state(s->conv_nb_states, state, j);
            error = s->state_error[state] + error_table[j + n];
            if (error < s->state_error1[next_state]) {
                s->state_error1[next_state] = error;
                s->state_decision[next_state][trellis_ptr] = decision_table[j + n];
                s->state_path[next_state][trellis_ptr] = state;
            }
        }
    }

    /* XXX: this copy is not needed. Permute the two tables */
    memcpy(s->state_error, s->state_error1, sizeof(s->state_error));

    trellis_ptr = (trellis_ptr + 1) % TRELLIS_LENGTH;
    s->trellis_ptr = trellis_ptr;
}

static void put_bit(V34DSPState *s, int b)
{
    int poly;

    if (!s->calling)
        poly = V34_GPC;
    else
        poly = V34_GPA;
    b = unscramble_bit(s, b, poly);
    //    printf("recv: %d\n", b);
    s->put_bit(s->opaque, b);
}

/* auxilary channel bit */
static void aux_put_bit(V34DSPState *s, int b)
{
    /* not used now */
}

static void decode_mapping_frame(V34DSPState *s, s16 rx_mapping_frame[8][2])
{
  int m[4][2], r0; /* rings */
  u8 I[3][4];
  int Q[4][2], Z[2];
  u8 *ptr;
  int t,i,j,x,y,n,mp_size,xout_ptr;
  u8 data[MAX_MAPPING_FRAME_SIZE];

  xout_ptr = 0;
  for(j=0;j<4;j++) {
    /* for each 4D symbol */

    for(i=0;i<2;i++) {
      x = rx_mapping_frame[xout_ptr][0];
      y = rx_mapping_frame[xout_ptr][1];
      xout_ptr++;

      /* decision */
      x = (x >> 8) * 2 + 1;
      x = clamp(x, C_RADIUS);
      y = (y >> 8) * 2 + 1;
      y = clamp(y, C_RADIUS);
      
      t = s->constellation_to_code[(x+C_RADIUS) >> 1][(y+C_RADIUS) >> 1];
      /* mapping to the symbol */
      Z[i] = t >> 14;
      t = t & 0xff;

      Q[j][i] = t & ((1 << s->q)-1);
      m[j][i] = t >> s->q;
    }

    t = (Z[0] - s->Z_1) & 3;
    s->Z_1 = Z[0];
    I[1][j] = t & 1;
    I[2][j] = t >> 1;
    
    t = (Z[1] - Z[0]) & 3;
    I[0][j] = t >> 1;
  }

  /* compute mapping frame size */
  s->rcnt += s->r;
  if (s->rcnt < s->P) {
      mp_size = s->b - 1;
  } else {
      s->rcnt -= s->P;
      mp_size = s->b;
  }
    
  /* now everything is "decoded", we can write the data */
  ptr = data;
  if (s->b <= 12) {
    for(i=0;i<s->b;i++) *ptr++ = ((u8 *)I)[i];
  } else {
    r0 = rings_to_index(s, m);

    n = s->K;
    if (mp_size < s->b) n--;
    for(i=0;i<n;i++) *ptr++ = (r0 >> i) & 1;
    
    for(j=0;j<4;j++) {
      ptr[0] = I[0][j];
      ptr[1] = I[1][j];
      ptr[2] = I[2][j];
      ptr += 3;
  
      t=Q[j][0];
      for(i=0;i<s->q;i++) *ptr++ = (t >> i) & 1;

      t=Q[j][1];
      for(i=0;i<s->q;i++) *ptr++ = (t >> i) & 1;
    }
  }

  /* send an auxilary channel bit if needed */
  s->acnt += s->W;
  if (s->acnt < s->P) {
      put_bit(s, data[0]);
  } else {
      s->acnt -= s->P;
      aux_put_bit(s, data[0]); 
  }

  /* send all the decoded bits */
  for(i=1;i<mp_size;i++) put_bit(s, data[i]); 
  //  print_bit_vector("recv", data, mp_size);

  if (++s->mapping_frame >= s->P) {
      /* new data frame */
      s->mapping_frame = 0;
      s->rcnt = 0;
      s->acnt = 0;
  }
}

static void baseband_decode(V34DSPState *s, int si, int sq)
{
    s16 y[2][2];
    static int delay = 0;
    int mse,v0;

    lm_dump_qam(si / (10.0 * 128.0), sq / (10.0 * 128.0));

    s->yy[s->phase_4d][0] = si;
    s->yy[s->phase_4d][1] = sq;
    
    if (++s->phase_4d == 2) {

        trellis_decoder(s, y, s->yy , &mse);
        s->phase_mse += mse;
        s->phase_mse_cnt++;
        if (s->phase_mse_cnt >= 8) {
            s->phase_mse_cnt = 0;
            s->phase_mse = 0;
        }

        /* synchronization bit */
        if (s->sync_count == 0) {
            v0 = (SYNC_PATTERN >> (15 - s->half_data_frame_count)) & 1;
        } else {
            v0 = 0;
        }

        /* synchronization bit */
        if (++s->sync_count == 2*s->P) {
            s->sync_count = 0;
            if (++s->half_data_frame_count == 2*s->J) {
                s->half_data_frame_count = 0;
            }
        }

        memcpy(&s->rx_mapping_frame[s->rx_mapping_frame_count][0], 
               &y[0][0], 4 * sizeof(s16));
        delay++;
        if (delay > TRELLIS_LENGTH) {

            s->rx_mapping_frame_count += 2;
            if (s->rx_mapping_frame_count == 8) {
                /* a complete mapping frame was read */
                decode_mapping_frame(s, s->rx_mapping_frame); 
                s->rx_mapping_frame_count = 0;
            }
        }
        s->phase_4d = 0;
    }
}


/* AGC */

#define AGC_COEF 0.99
#define AGC_BITS 24

static void agc_init(V34DSPState *s)
{
    s->agc_coef = (int) (AGC_COEF * (1 << AGC_BITS));
    s->agc_mem = 0;
}

static void agc_estimate(V34DSPState *s, int sample)
{
    float power;
    
    s->agc_mem = s->agc_mem * AGC_COEF + (sample * sample);

    power = (float) s->agc_mem * (1.0 - AGC_COEF);
    /* XXX: the constant here depends on the modulation parameters */
    // s->agc_gain = (16384.0/2 * 1495) / sqrt(power);
    s->agc_gain = 16384.0 * 0.80;

    //    lm_dump_agc(power / 16384.0);
    lm_dump_agc(sqrt(power));
}

#if 0
static void test_nyq(float *a_ptr, float *b_ptr, float f, float val)
{
    float a,b,c,d;

    a = *a_ptr;
    b = *b_ptr;
    c = 0.99 * cos(f);
    d = 0.99 * sin(f);
    
    *a_ptr = a * c - b * d + val;
    *b_ptr = a * d + b * c;
}

static float al, bl, ah, bh;
static float f_low, f_high;
static float low_mem[2], low_coef[2];
#endif

#define SYNC_THR 32

/* Fast symbol timing recovery */
static void v34_symbol_sync(V34DSPState *s, int spl)
{
    int a, b, c, v;
#if 0
    int tmp0, tmp1;
    static s16 dc_filter[2];
    static s16 ac_filter[3];
#endif

#if 0
    {
        static int k = 0;
        float v;

        f_low = 2 * M_PI * (s->carrier_freq - s->symbol_rate / 2.0) / (3.0 * s->symbol_rate);
        f_high = 2 * M_PI * (s->carrier_freq + s->symbol_rate / 2.0) / (3.0 * s->symbol_rate);
        test_nyq(&al, &bl, f_low, spl);
        test_nyq(&ah, &bh, f_high, spl);
    }
#endif
    /* sync_low_mem has an amplitude of about spl / ( 1 - a), so we
       store it with a scaling of 2^6 to reduce its amplitude */

    v = ((spl << 8) + s->sync_low_mem[0] * s->sync_low_coef[0] + 
         s->sync_low_mem[1] * s->sync_low_coef[1]) >> 14;
    s->sync_low_mem[1] = s->sync_low_mem[0];
    s->sync_low_mem[0] = v;
    
    v = ((spl << 8) + s->sync_high_mem[0] * s->sync_high_coef[0] + 
         s->sync_high_mem[1] * s->sync_high_coef[1]) >> 14;
    s->sync_high_mem[1] = s->sync_high_mem[0];
    s->sync_high_mem[0] = v;

    if (s->baud3_phase == 0) {
        /* high & low nyquist filters for symbol timing recovery */
        
        /* XXX: the shift should adapt to the power */
        a = (s->sync_low_mem[1] * s->sync_high_mem[1]) >> 14;
        b = (s->sync_high_mem[1] * s->sync_low_mem[0]) >> 14;
        c = (s->sync_low_mem[1] * s->sync_high_mem[0]) >> 14;
        
        v = (a * s->sync_A + b * s->sync_B + c * s->sync_C) >> 14;
        printf("v=%d\n", v);
        s->baud_phase -= v << 3;

#if 0
        /* DC filter h(z) = z - z^-2 */
        tmp0 = v - dc_filter[1];
        dc_filter[1] = dc_filter[0];
        dc_filter[0] = v;

        /* AC filter h(z) = z + z^-3 */
        tmp1 = tmp0 + ac_filter[2];
        ac_filter[2] = ac_filter[1];
        ac_filter[1] = ac_filter[0];
        ac_filter[0] = tmp0;

        if (tmp1 < -SYNC_THR)
            tmp1 = -SYNC_THR;
        else if (tmp1 > SYNC_THR) 
            tmp1 = SYNC_THR;
        else
            tmp1 =0;
#endif
#if 1
        //        lm_dump_sample(CHANNEL_SAMPLE, v);

        //        printf("corr=%0.0f\n", 
        //               corr * 32.0 / 100.0);
#else
        printf("al=%0.1f ah=%0.1f al1=%0.1f ah1=%0.1f\n",
               ah / 100.0, bh / 100.0, 
               (s->sync_high_mem[0] - s->sync_high_mem[1] * 0.99 * cos(f_high)) * 32.0 / 100.0,
               (s->sync_high_mem[1] * 0.99 * sin(f_high)) * 32.0 / 100.0);
#endif        
    }
}


/* equalize & adapt the equalizer */
static int v34_equalize(V34DSPState *s, 
                        int *ri_ptr, int *rq_ptr, int spl)
{
    int p,q,i;
    int ri, rq, fi, fq, q_ri, q_rq, ei, eq, ei1, eq1, si, sq;
    int cosw, sinw, dphi, norm;

    /* add the sample in the equalizer ring buffer */
    p = s->eq_buf_ptr;

    s->eq_buf[p] = spl;

    if (++p == EQ_SIZE)
        p = 0;
    s->eq_buf_ptr = p;

    if (s->baud3_phase != 0)
        return 0;

    /* apply the equalizer filter to the data */
    ri = rq = 0;
    q = p;
    for(i=0;i<EQ_SIZE;i++) {
        fi = s->eq_filter[i][0] >> 16;
        fq = s->eq_filter[i][1] >> 16;
        
        ri += fi * s->eq_buf[q];
        rq += fq * s->eq_buf[q];

        q++;
        if (q == EQ_SIZE)
            q = 0;
    }
    si = ri >> 14;
    sq = rq >> 14;
    
    /* rotate by the carrier phase */
    /* translate back to baseband */

    cosw = dsp_cos(s->carrier_phase);
    sinw = - dsp_cos((PHASE_BASE/4) - s->carrier_phase);
    ri = ( si * cosw - sq * sinw ) >> COS_BITS;
    rq = ( si * sinw + sq * cosw ) >> COS_BITS;
    
    *ri_ptr = ri;
    *rq_ptr = rq;

    /* compute the error */

    /* quantification */
    q_ri = ((ri >> 8) * 2 + 1) << 7;
    q_rq = ((rq >> 8) * 2 + 1) << 7;

    /* error computation */
    ei1 = - (ri - q_ri);
    eq1 = - (rq - q_rq);

    /**** phase tracking */

    /* normalized derivative of the phase shift */
    /* XXX: avoid sqrt : slow !!! */
    norm = (int) sqrt(ri * ri + rq * rq );
    if (norm > 0) {
        dphi = (ri * eq1 - rq * ei1) / norm;
    } else {
        dphi = 0;
    }
    s->carrier_phase += s->carrier_incr - dphi;

    /* remodulate (because the equalizer is done before converting to
       baseband) */
    ei = ( ei1 * cosw + eq1 * sinw ) >> COS_BITS;
    eq = ( - ei1 * sinw + eq1 * cosw ) >> COS_BITS;

#if 1
    /* update the coefficients with the error */
    q = p;
    for(i=0;i<EQ_SIZE;i++) {
        int di, dq;
        di = ei * s->eq_buf[q];
        dq = eq * s->eq_buf[q];
        
        s->eq_filter[i][0] += (di >> s->eq_shift) * 16;
        s->eq_filter[i][1] += (dq >> s->eq_shift) * 16;

        q++;
        if (q == EQ_SIZE)
            q = 0;
    }
#endif

    lm_dump_equalizer(s->eq_filter, 1 << 30, EQ_SIZE);

    return 1;
}

static void V34_demod(V34DSPState *s, 
                      const s16 *samples, unsigned int nb)
{
    int si, sq, i, j, k , ph, spl;
    int v, frac, ph1;

    for(i=0;i<nb;i++) {
        /* Automatic Gain Control */
        spl = samples[i];

        agc_estimate(s, spl);
        spl = (spl * s->agc_gain) >> 14;

        /* insert the new sample in the ring buffer */
        s->rx_buf1[s->rx_buf1_ptr] = spl;
        s->rx_buf1_ptr = (s->rx_buf1_ptr + 1) & (RX_BUF1_SIZE-1);

        /* sample rate convertion, timing correction & matched filter
           (root raised cosine) */
        s->baud_phase += s->baud_num;
        while (s->baud_phase >= s->baud_denom) {
            s->baud_phase -= s->baud_denom;
            
            ph = s->baud_phase;
            si = 0;
            for(j=0;j<s->rx_filter_wsize;j++) {
                k = (s->rx_buf1_ptr - s->rx_filter_wsize + j) & (RX_BUF1_SIZE-1);
                /* XXX: verify that there is no overflow */

                /* interpolation of the filter coefficient */
                ph1 = ph >> 16;
                frac = ph & 0xffff;
                v = ((0x10000 - frac) * s->rx_filter[ph1] + 
                     frac * s->rx_filter[ph1+1]) >> 16;
                si += v * s->rx_buf1[k];
                ph += s->baud_num;
            }
            si = (si >> 14);
            lm_dump_sample(CHANNEL_SAMPLESYNC, si / 32768.0);

            /* we have here EQ_FRAC = 3 symbols per baud */

            switch(s->state) {
            case V34_STARTUP3_WAIT_S1:
                /* wait for the S signal */
                printf("waiting S1 %d\n", si);
                /* XXX: find a better test ! */
                if (abs(si) > 13000) {
                    s->state = V34_STARTUP3_S1;
                    s->sym_count = 0;
                }
                break;

            case V34_STARTUP3_S1:
                /* S signals are mainly used to recover the symbol clock */
                v34_symbol_sync(s, si);
                if (++s->sym_count >= 128 * EQ_FRAC) {
                    s->state = V34_STARTUP3_SINV1;
                    s->sym_count = 0;
                }
                break;
            case V34_STARTUP3_SINV1:
                v34_symbol_sync(s, si);
                if (++s->sym_count >= 16 * EQ_FRAC) {
                    s->state = V34_STARTUP3_S2;
                    s->sym_count = 0;
                }
                break;

            case V34_STARTUP3_S2:
                v34_symbol_sync(s, si);
                if (++s->sym_count >= 128 * EQ_FRAC) {
                    s->state = V34_STARTUP3_SINV2;
                    s->sym_count = 0;
                }
                break;

            case V34_STARTUP3_SINV2:
                v34_symbol_sync(s, si);
                if (++s->sym_count >= 100 * EQ_FRAC) {
                    s->state = V34_STARTUP3_PP;
                    s->sym_count = 0;
                }
                break;

            case V34_STARTUP3_PP:
#if 1
                /* PP is used to fast train the equalizer */

                /* store the 144 samples at the middle of the PP
                   frame. We do this because we suppose in the fast
                   equalizer that the sequence is periodic */
                if (s->sym_count >= (120) * EQ_FRAC && 
                    s->sym_count < (168) * EQ_FRAC) {
                    s->eq_buf[s->sym_count - (120) * EQ_FRAC] = si;
                }

                if (s->sym_count == (168) * EQ_FRAC) {
                    /* XXX: this call takes a long time. Is it a
                       problem ? */
                    V34_fast_equalize(s, s->eq_buf);
                    /* reset eq_buf to avoid potential problems when the
                       adaptive is started */
                    memset(s->eq_buf, 0, sizeof(s->eq_buf));
                }
                
                if (++s->sym_count == 288 * EQ_FRAC) {
                    s->state = V34_STARTUP3_TRN;
                }
#else
                memmove(s->eq_buf, &s->eq_buf[1], 2 * EQ_SIZE);
                s->eq_buf[EQ_SIZE - 1] = si;
                
                if ((++s->sym_count % EQ_SIZE) == 0) {
                    V34_fast_equalize(s, s->eq_buf);
                    lm_dump_equalizer(s->eq_filter, 1 << 30, EQ_SIZE);
                }
#endif

                break;

            case V34_STARTUP3_TRN:
                si = (float)si * 128.0 / CALC_AMP(TRN4_POWER);
                if (v34_equalize(s, &si, &sq, si)) {
                    static int ptr = 0;
                    
                    if (++ptr > (28 * 2)) {
                        baseband_decode(s, si, sq);
                    }
                }
                break;
            }


            
            if (++s->baud3_phase == EQ_FRAC)
                s->baud3_phase = 0;
        }
    }
}

static void V34_demod_init(V34DSPState *s, V34State *p)
{
    memset(s, 0, sizeof(V34DSPState));

    V34_init_low(s, p, 0);
    s->state = V34_STARTUP3_WAIT_S1;
}

/* init the V34 constants. Should be launched once */
void V34_static_init(void)
{
    V34eq_init();
}


void V34_init(struct V34State *s, int calling)
{
    

}

int V34_process(struct V34State *s, s16 *output, s16 *input, int nb_samples)
{
    

    return 0;
}

/* V34 test: half duplex with fixed parameters */

#define NB_SAMPLES 40 /* 5 ms */

static int test_get_bit(void *opaque)
{
    return 1;
}

static int nb_bits, errors;

static void test_put_bit(void *opaque, int bit)
{
    nb_bits++;
    if (bit != 1) {
        errors++;
    }
}

void V34_test(void)
{
    V34State s;
    V34DSPState v34_rx, v34_tx;
    int err;
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

    /* fill the test V34 parameters */
#if 0
    s.S = V34_S3200;
    s.R = 28800;
#else
    s.S = V34_S2400;
    s.R = 19200;
#endif
    s.expanded_shape = 0;
    s.conv_nb_states = 16;
    s.use_non_linear = 0;
    s.use_high_carrier = 1;
    s.use_aux_channel = 0;
    memset(s.h, 0, sizeof(s.h));

    f1 = fopen("cal.sw", "wb");
    if (f1 == NULL) {
        perror("cal.sw");
        exit(1);
    }

    s.calling = 1;
    V34_mod_init(&v34_tx, &s);
    v34_tx.opaque = NULL;
    v34_tx.get_bit = test_get_bit;

    s.calling = 0;
    V34_demod_init(&v34_rx, &s);
    v34_rx.opaque = NULL;
    v34_rx.put_bit = test_put_bit;

    nb_bits = 0;
    errors = 0;
    for(;;) {
        if (lm_display_poll_event())
            break;
        
        V34_mod(&v34_tx, buf, NB_SAMPLES);
        
        memset(buf3, 0, sizeof(buf3));

        line_model(line_state, buf1, buf, buf2, buf3, NB_SAMPLES);
        
        fwrite(buf, 1, NB_SAMPLES * 2, f1);

        V34_demod(&v34_rx, buf, NB_SAMPLES);
    }

    fclose(f1);

    printf("errors=%d nb_bits=%d Pe=%f\n", 
           errors, nb_bits, (float) errors / (float)nb_bits);
}
