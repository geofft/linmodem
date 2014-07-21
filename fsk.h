#ifndef FSK_H
#define FSK_H

typedef struct {
    /* parameters */
    int f_lo,f_hi;
    int sample_rate;
    int baud_rate;

    /* local variables */
    int phase, baud_frac, baud_incr;
    int omega[2];
    int current_bit;
    void *opaque;
    get_bit_func get_bit;
} FSK_mod_state;

/* max = 106 for 75 bauds */
#define FSK_FILTER_SIZE 128 
#define FSK_FILTER_BUF_SIZE 256

typedef struct {
    /* parameters */
    int f_lo,f_hi;
    int sample_rate;
    int baud_rate;

    /* local variables */
    int filter_size;
    s16 filter_lo_i[FSK_FILTER_SIZE];
    s16 filter_lo_q[FSK_FILTER_SIZE];
    
    s16 filter_hi_i[FSK_FILTER_SIZE];
    s16 filter_hi_q[FSK_FILTER_SIZE];

    s16 filter_buf[FSK_FILTER_BUF_SIZE];
    int buf_ptr;

    int baud_incr;
    int baud_pll, baud_pll_adj, baud_pll_threshold;
    int lastsample;
    int shift;

    void *opaque;
    put_bit_func put_bit;
} FSK_demod_state;

void FSK_mod_init(FSK_mod_state *s);
void FSK_mod(FSK_mod_state *s, s16 *samples, unsigned int nb);
void FSK_demod_init(FSK_demod_state *s);
void FSK_demod(FSK_demod_state *s, const s16 *samples, unsigned int nb);

void FSK_test(int do_v23);

#endif
