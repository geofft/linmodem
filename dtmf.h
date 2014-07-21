
/* modulation */
typedef struct {
    /* parameters */
    int dtmf_level; /* in dB */
    int digit_length_ms;
    int digit_pause_ms;
    void *opaque;
    int (*get_digit)(void *opaque);

    /* internal state */
    int omega1,omega2,t1,t2;
    int samples_left;
    int amp;
} DTMF_mod_state;

void DTMF_mod_init(DTMF_mod_state *s);
void DTMF_mod(DTMF_mod_state *s, s16 *samples, unsigned int nb);

/* demodulation */
#define DTMF_N 205

typedef struct {
    /* parameters */
    void *opaque;
    void (*put_digit)(void *opaque, int digit);
    
    /* internal state */
    s16 buf[DTMF_N];
    int buf_ptr;
    s16 cos_tab[DTMF_N];
    s16 sin_tab[DTMF_N];
    int last_digit;
    int dtmf_coefs[8]; /* coefficient number of the DFT */
} DTMF_demod_state;

void DTMF_demod_init(DTMF_demod_state *s);
void DTMF_demod(DTMF_demod_state *s, 
                const s16 *samples, unsigned int nb);
