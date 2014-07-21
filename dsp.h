
typedef signed char s8;
typedef unsigned char u8;
typedef short s16;
typedef unsigned short u16;
typedef int s32;
typedef unsigned int u32;
typedef long long s64;
typedef unsigned long long u64;

#define PHASE_BITS 16
#define PHASE_BASE (1 << PHASE_BITS)

#define COS_BITS   14
#define COS_BASE   (1 << COS_BITS)

#define COS_TABLE_BITS 13
#define COS_TABLE_SIZE (1 << COS_TABLE_BITS)

extern s16 cos_tab[COS_TABLE_SIZE];

void dsp_init(void);



/* unoptimized DSP C functions */
/* XXX: optimize them for each architecture */

static inline int dsp_cos(int phase) 
{
    return cos_tab[(phase >> (PHASE_BITS - COS_TABLE_BITS)) & (COS_TABLE_SIZE-1)];
}

static inline int dsp_dot_prod(const s16 *tab1, const s16 *tab2, 
                               int n, int sum)
{
    int i;

    for(i=0;i<n;i++) {
        sum += tab1[i] * tab2[i];
    }

    return sum;
}

static inline int dsp_norm2(s16 *tab, int n, int sum)
{
    int i;
    for(i=0;i<n;i++) {
        sum += tab[i] * tab[i];
    }
    return sum;
}

static inline void dsp_sar_tab(s16 *tab, int n, int shift)
{
    int i;
    for(i=0;i<n;i++) {
        tab[i] >>= shift;
    }
}

static inline int dsp_max_bits(s16 *tab, int n)
{
    int i, max, v, b;
    max = 0;
    for(i=0;i<n;i++) {
        v = abs(tab[i]);
        if (v > max) 
            max = v;
    }
    b = 0;
    while (max != 0) {
        b++;
        max>>=1;
    }
    return b;
}

static inline int dsp_sqr(int n)
{
    return n*n;
}

int compute_DFT(s16 *cos_tab, s16 *sin_tab, s16 *x, int k,int n);

typedef struct {
	float re,im;
} complex;

/* medium speed FFT */
void fft_calc(complex *x,int n, int r);

/* slow FFT for any size */
void slow_fft(complex *output, complex *input, int n, int r);

/* compute the hamming window */
void calc_hamming(float *ham, int NF);
