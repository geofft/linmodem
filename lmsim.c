/* 
 * Implementation of the phone line simulator
 * 
 * Copyright (c) 1999,2000 Fabrice Bellard.
 *
 * This code is released under the GNU General Public License version
 * 2. Please read the file COPYING to know the exact terms of the
 * license.
 * 
 */
#include "lm.h"

#define NB_SAMPLES 40 /* 5 ms */
#define SAMPLE_RATE 8000

#define SAMPLE_REF  0x4000  /* 0 dB reference (sample level) */

/* 'calling' is used to know in which direction we transmit for echo cancellation */
struct sm_hw_info sm_hw_null;


/* transmit from 'A' to 'B' */
static void tx_rx(struct sm_state *A, struct sm_state *B, int dump)
{
        /* transmit data from call_dce */
        if (sm_size(&A->tx_fifo) < 10) {
            int i;

            for(i=0;i<256;i++)
                sm_put_bit(&A->tx_fifo, i);
        }

        /* receive data from answer_dce */
        for(;;) {
            int c;
            c = sm_get_bit(&B->rx_fifo);
            if (c == -1)
                break;
            if (dump) {
                printf("[%02x]", c);
                fflush(stdout);
            }
        }
}

void line_simulate(void)
{
    struct sm_state sm1, sm2, *call_dce = &sm1, *answer_dce = &sm2;
    s16 answer_buf[NB_SAMPLES], call_buf[NB_SAMPLES];
    s16 answer_buf1[NB_SAMPLES], call_buf1[NB_SAMPLES];
    FILE *f1,*f2;
    struct LineModelState *line_state;
    int err;

    err = lm_display_init();
    if (err < 0) {
        fprintf(stderr, "Could not init X display\n");
        exit(1);
    }

    line_state = line_model_init();

    /* init two modems */
    lm_init(call_dce, &sm_hw_null, "cal");
    lm_init(answer_dce, &sm_hw_null, "ans");

    /* start calls */
    lm_start_dial(call_dce, 0, "1234567890");
    lm_start_receive(answer_dce);
    answer_dce->state = SM_TEST_RING;

    f1 = fopen("cal.sw", "wb");
    if (f1 == NULL) {
        perror("cal.sw");
        exit(1);
    }

    f2 = fopen("ans.sw", "wb");
    if (f1 == NULL) {
        perror("ans.sw");
        exit(1);
    }

    memset(answer_buf, 0, sizeof(answer_buf));

    /* init asynchronous tx & rx */
    serial_init(call_dce, 8, 'N');
    serial_init(answer_dce, 8, 'N');

    for(;;) {
        lm_display_poll_event();

        /* transmit & receive in both direction & dump call to ans modem */
        tx_rx(call_dce, answer_dce, 0);
        tx_rx(answer_dce, call_dce, 1);

        /* exit connection if ONHOOK state */

        if (lm_get_state(call_dce) == LM_STATE_IDLE ||
            lm_get_state(answer_dce) == LM_STATE_IDLE)
            break;

        /* simulate both modem & output the samples to disk */

        sm_process(call_dce, call_buf, answer_buf1, NB_SAMPLES);
        fwrite(call_buf, 1, NB_SAMPLES * 2, f1);
        
        sm_process(answer_dce, answer_buf, call_buf1, NB_SAMPLES);
        fwrite(answer_buf, 1, NB_SAMPLES * 2, f2);

        /* simulate the phone line */
        line_model(line_state, 
                   call_buf1, call_buf,
                   answer_buf1, answer_buf, NB_SAMPLES);
    }

    fclose(f1);
    fclose(f2);

    for(;;) {
        if (lm_display_poll_event())
            break;
    }

    lm_display_close();
}

static int sim_open(struct lm_interface_state *s)
{
    return 0;
}

static void sim_close(struct lm_interface_state *s)
{
}

static void sim_set_offhook(struct lm_interface_state *s, int v)
{
    if (v) {
        printf("%s: offhook\n",s->sm->name);
    } else {
        printf("%s: onhook\n",s->sm->name);
    }
}

static void sim_set_ring(struct lm_interface_state *s, int v)
{
}

static void sim_main_loop(struct lm_interface_state *s)
{
}

struct sm_hw_info sm_hw_null = 
{
    sim_open,
    sim_close,
    sim_set_offhook,
    sim_set_ring,
    sim_main_loop,
};

/* simple phone line model */

#define LINE_FILTER_SIZE 129
#define A 0.99 /* constant for exponential averaging for power estimations */
#define FFT_SIZE    1024

/* state of a uni directional line */
typedef struct {
    float buf[LINE_FILTER_SIZE]; /* last transmitted samples (ring buffer, 
                               used by the line filter) */
    int buf_ptr;            /* pointer of the last transmitted sample in buf */
    float tx_pow;  /* estimated transmit power */
    float rx_pow;  /* estimated receive power */
    float noise_pow; /* estimated noise power */
} UniDirLineState;


typedef struct LineModelState {
    UniDirLineState line1, line2;
    float fout1, fout2; 
} LineModelState;

static float line_filter[LINE_FILTER_SIZE];
static float sigma;    /* gaussian noise sigma */
static int nb_clamped; /* number of overflows */
static float modem_hybrid_echo; /* echo level created by the modem hybrid */
static float cs_hybrid_echo; /* echo level created by the central site hybrid */

#define RANDMAX 0x7fffffff

float random_unif(void)
{
  return (float) random() / RANDMAX;
}

float random_gaussian(void)
{
  float v1, v2, s , m;
  do {
    v1 = 2 * random_unif() - 1; 
    v2 = 2 * random_unif() - 1;
    s = v1 * v1 + v2 * v2;
  } while (s >= 1);
  m = sqrt(-2 * log(s)/s);
  return v1 * m;
}

/* tabulated medium range telephone line respond (from p 537, Digital
   Communication, John G. Proakis */

/* amp 1.0 -> 2.15, freq = 3000 Hz -> 3.2, by 0.2 increments 
   delay = 4 ms -> 2.2
 */

float phone_amp[23] = {
    0,
    0.9,
    1.4,
    1.8,
    2.0,
    2.1,
    2.3,
    2.3,
    2.2,
    2.1,
    2.0,
    1.85,
    1.75,
    1.55,
    1.3,
    1.1,
    0.8,
    0.55,
    0.25,
    0.05,
    0.05, 
    0.05, 
    0.00,
};

float phone_delay[23] = {
    2.2, /* NA */
    2.2, /* NA */
    2.2,
    0.9,
    0.5,
    0.25,
    0.1,
    0.05,
    0.0,
    0.0,
    0.0,
    0.05,
    0.1,
    0.2,
    0.4,
    0.5,
    0.9,
    1.2,
    2.2,
    2.2, /* NA */
    2.2, /* NA */
    2.2, /* NA */
    2.2, /* NA */
};

static void build_line_impulse_response(void)
{
    float f, f1, a, amp, phase, delay;
    int index, i, j;
    complex tab[FFT_SIZE];
    FILE *outfile;

    for(i=0;i<FFT_SIZE/2;i++) {
        f = (float) i / FFT_SIZE;
        f = f * SAMPLE_RATE;
        f1 = f / 3000.0 * 3.2 / 0.2;
        a = f1 - floor(f1);
        index = (int)floor(f1);

        /* linear interpolation */
        amp = (1 - a) * phone_amp[index] + a * phone_amp[index+1];
        amp = amp / 2.15;

        delay = (1 - a) * phone_delay[index] + a * phone_delay[index+1];
        delay = delay / 2.2 * 4;

        phase = 2 * M_PI * f * delay * 0.001;
#if 0
        printf("index=%d a=%0.3f f=%0.0f amp=%0.2f delay=%0.2f %0.1f\n", 
               index, a, f, amp, delay, phase);
#endif        
        tab[i].re = amp * cos(phase);
        tab[i].im = amp * sin(phase);
    }
    tab[FFT_SIZE/2].im = 0;
    for(i=1;i<FFT_SIZE/2;i++) {
        tab[FFT_SIZE - i].re = tab[i].re;
        tab[FFT_SIZE - i].im = - tab[i].im;
    }
    
    fft_calc(tab, FFT_SIZE, 1);

    outfile = fopen("a", "w");
    j = FFT_SIZE - (LINE_FILTER_SIZE - 1)/2;
    for(i=0;i<LINE_FILTER_SIZE;i++) {
        line_filter[i] = tab[j].re;
        fprintf(outfile, "%f\n", tab[j].re);
        if (++j == FFT_SIZE)
            j = 0;
    }
    fclose(outfile);
}

LineModelState *line_model_init(void)
{
    float N0,SNR, p, echo_level;
    int i;
    LineModelState *s;

    s = malloc(sizeof(LineModelState));
    memset(s, 0, sizeof(LineModelState));

    SNR = 25; /* wanted SNR */
    N0 = pow(10,-SNR/10.0);
    sigma=sqrt(N0/2) * (float)SAMPLE_REF;

    /* echos */
    echo_level = -15; /* in dB */
    cs_hybrid_echo = pow(10, echo_level/20.0);
    modem_hybrid_echo = pow(10, echo_level/20.0);
    
#if 0
    build_line_impulse_response();
#else
    /* simple filter */
    line_filter[LINE_FILTER_SIZE/2+1] = 0.3;
    line_filter[LINE_FILTER_SIZE/2] = 1.0;
    line_filter[LINE_FILTER_SIZE/2-1] = 0.3;
#endif
    /* normalize the filter to a power of 1.0 */
    p = 0;
    for(i=0;i<LINE_FILTER_SIZE;i++) {
        p += line_filter[i] * line_filter[i];
    }
    p = sqrt(p);
    for(i=0;i<LINE_FILTER_SIZE;i++) line_filter[i] /= p;

#if 0
    for(i=0;i<LINE_FILTER_SIZE;i++) 
        printf("%5d %0.3f\n", i, line_filter[i]);
#endif

    return s;
}


float compute_db(float a)
{
    return 10.0 * log(a) / log(10.0);
}

static int dump_count;

static float calc_line_filter(UniDirLineState *s, 
                              float v, int calling)
{
    float sum, noise;
    int j, p;

    /* compute tx power */
    s->tx_pow = (v * v) * (1.0 - A) + A * s->tx_pow;
        
    /* add the sample in the filter buffer */
    p = s->buf_ptr;
    s->buf[p] = v;
    if (++p == LINE_FILTER_SIZE)
        p = 0;
    s->buf_ptr = p;
    
    /* apply the filter */
    sum = 0;
    for(j=0;j<LINE_FILTER_SIZE;j++) {
        sum += line_filter[j] * s->buf[p];
        if (++p == LINE_FILTER_SIZE)
            p = 0;
    }
    
    /* add noise */
    noise = random_gaussian() * sigma;
    sum += noise;
    
    /* (testing only: noise power) */
    s->noise_pow = (noise * noise) * (1.0 - A) + A * s->noise_pow;
    
    /* compute rx_power */
    s->rx_pow = (sum * sum) * (1.0 - A) + A * s->rx_pow;
    
    /* dump estimations */
    if (calling && ++dump_count == 50) {
        float ref_db;
        
        dump_count = 0;
        ref_db = compute_db(SAMPLE_REF * SAMPLE_REF);
        lm_dump_linesim_power(compute_db(s->tx_pow) - ref_db, 
                              compute_db(s->rx_pow) - ref_db,
                              compute_db(s->noise_pow) - ref_db);
    }
    
    return sum;
}

static int clamp(float a)
{
    if (a < -32768) {
        a = -32768;
        nb_clamped++;
    } else if (a > 32767) {
        a = 32767;
        nb_clamped++;
    }
    return (int)rint(a);
}

/* modem (cal)                         modem (ans)
 *
 * input1                        ->   output1  (line1)
 * output2                       <-    input2  (line2)
 *
 */
void line_model(LineModelState *s, 
                s16 *output1, const s16 *input1,
                s16 *output2, const s16 *input2,
                int nb_samples)
{
    int i;
    float in1, in2, out1, out2;
    float tmp1, tmp2;

    for(i=0;i<nb_samples;i++) {
        in1 = input1[i];
        in2 = input2[i];

        /* echo from cal modem central site hybrid */
        tmp1 = in1 + s->fout2 * cs_hybrid_echo;

        /* echo from ans modem central site hybrid */
        tmp2 = in2 + s->fout1 * cs_hybrid_echo;

        /* line filters & noise */
        s->fout1 = calc_line_filter(&s->line1, tmp1, 1);

        s->fout2 = calc_line_filter(&s->line2, tmp2, 0);

        /* echo from ans modem hybrid */
        out1 = s->fout1 + in2 * modem_hybrid_echo;
        lm_dump_sample(CHANNEL_SAMPLE, out1 / 32768.0);

        /* echo from cal modem hybrid */
        out2 = s->fout2 + in1 * modem_hybrid_echo;

        output1[i] = clamp(out1);
        output2[i] = clamp(out2);
    }
}
