#include "lm.h"

s16 cos_tab[COS_TABLE_SIZE];

void dsp_init(void)
{
    int i;

    for(i=0;i<COS_TABLE_SIZE;i++) {
        cos_tab[i] = (int) (cos( 2 * M_PI * i / COS_TABLE_SIZE) * COS_BASE);
    }
}

/* DFT computation with Goertzel algorithm */

int compute_DFT(s16 *cos_tab, s16 *sin_tab, s16 *x, int k,int n)
{
    int y_re,y_im,i,j,y_2;

    j = 0;
#if 0
    int s0,s1,s2;
    s0 = s1 = 0;
    for(i=0;i<n;i++) {
        s2 = x[i] + ((cos_tab[j] * s1) >> (COS_BITS-1)) - s0;
        s0 = s1;
        s1 = s2;
        j += k;
        if (j >= n) j -= n;
    }
    
    y_re = s1 - ((cos_tab[k] * s0) >> COS_BITS);
    /* cannot use cos_tab because n is even */
    y_im = s1 - ((sin_tab[k] * s0) >> COS_BITS); 
#else
    y_re = y_im = 0;
    for(i=0;i<n;i++) {
        y_re += cos_tab[j] * x[i];
        y_im += sin_tab[j] * x[i];
        j += k;
        if (j >= n) j -= n;
    }
    y_re = y_re >> COS_BITS;
    y_im = y_im >> COS_BITS;
#endif
    y_2 = y_re * y_re + y_im * y_im;
    return y_2;
}

/* horrible fft code - only needed to debug or fixed table generation */

#define FFT_MAX_SIZE 2048

static float norm;
static float wfft[FFT_MAX_SIZE];
static short tinv[FFT_MAX_SIZE];
static int nf = 0, nf2, nf4;

int fft_init(int n)
{
    float p,k;
    int i,j,a,c, nk;
    
    nf=n;
    nf2=nf/2;
    nf4=nf/4;
    nk=0;
    while (n>>=1) nk++;
    norm=1/sqrt(nf);
    
    k=0;
    p=(2*M_PI)/nf;
    for(i=0;i<=nf4;i++) {
        wfft[i]=cos(k);
        k+=p;
    }
    
    for(i=0;i<nf;i++) {
        a=i;
        c=0;
        for(j=0;j<nk;j++) {
            c=(c<<1)|(a&1);
            a>>=1;
        }
        tinv[i]= c<=i ? -1 : c;
    }
    
    return 0;
}

/* r = TRUE : reverse FFT */
void fft_calc(complex *x,int n, int r)
{
    int i,j,k,l;
    complex a,b,c;
    complex *p,*q;	

    /* auto init of coefficients */
    if (n != nf) {
        fft_init(n);
    }

    k=nf2;
    l=1;
    do {
        p=x;
        q=x+k;
        i=l;
        do {
            j=0;
            do {
                a=*p;
                b=*q;
                p->re=a.re+b.re;
                p->im=a.im+b.im;
                b.re=a.re-b.re;
                b.im=a.im-b.im;
                if (j==0) {
                    *q=b;
                } else if (j==nf4) {
                    if (r) {
                        q->re=b.im;
                        q->im=-b.re;
                    } else {
                        q->re=-b.im;
                        q->im=b.re;
                    }
                    q->re=-b.im;
                    q->im=b.re;
                } else if (j<nf4) {
                    c.re=wfft[j];
                    c.im=wfft[nf4-j];
                    if (r) c.im=-c.im;
                    q->re=b.re*c.re-b.im*c.im;
                    q->im=b.im*c.re+b.re*c.im;
                } else {
                    c.re=-wfft[nf2-j];
                    c.im=wfft[j-nf4];
                    if (r) c.im=-c.im;
                    q->re=b.re*c.re-b.im*c.im;
                    q->im=b.im*c.re+b.re*c.im;
                }
                p++;
                q++;
                j+=l;
            } while (j<nf2);
            p+=k;
            q+=k;
        } while (--i);
        k>>=1;
        l<<=1;
    } while (k);
    
    for(i=0,p=x;i<nf;i++,p++) {
        p->re*=norm;
        p->im*=norm;
    }
    
    for(i=0,p=x;i<nf;i++,p++) if ((j=tinv[i])!=-1) {
        a=*p;
        *p=x[j];
        x[j]=a;
    }
}

/* hamming window */

void calc_hamming(float *ham, int NF)
{
   int i;
   for(i=0;i<NF;i++) {
      ham[i]=0.54-0.46*cos((2*M_PI*i)/NF);
   }
}

/* slow floating point fft, for testing & init */

void slow_fft(complex *output, complex *input, int n, int r)
{
    complex coef[n], a, b, c;
    int i,j;

    for(i=0;i<n;i++) {
        float a;
        a = - 2 * M_PI * i / (float) n;
        if (r) 
            a = -a;
        coef[i].re = cos(a);
        coef[i].im = sin(a);
    }
    
    for(i=0;i<n;i++) {
        a.re = 0;
        a.im = 0;
        for(j=0;j<n;j++) {
            b = input[j];
            c = coef[(j * i) % n];
            a.re += b.re*c.re-b.im*c.im;
            a.im += b.im*c.re+b.re*c.im;
        }
        output[i] = a;
    }
}

