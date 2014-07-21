
#define V8_SAMPLE_RATE 8000

/* V8 tone ANSam tone synthesis */
typedef struct {
    /* parameters */
    int tone_level;

    /* internal */
    int sample_rate;
    int phase, phase_incr;
    int mod_phase, mod_phase_incr;
    int phase_reverse_samples;
    int phase_reverse_left;
    int amp;
} V8_mod_state;


/* ANSam tone detection */
/* 25 ms window */
#define V8_N  200
#define DFT_COEF_2100 ((int)((2100.0 / V8_SAMPLE_RATE * V8_N) + 0.5))


typedef struct {
    int buf_ptr;
    s16 buf[V8_N];
    s16 cos_tab[V8_N];
    s16 sin_tab[V8_N];
    int v8_ANSam_detected; /* true if ANSam detected */
} V8_demod_state;

/* Te period, as in the V8 spec 500 <= Te  */
#define V8_TE 800

/* V8 main state */
typedef struct {
    int calling;   /* true if we are the calling modem */
    int state;     /* current state of the V8 protocol */
    int debug_laststate; 
    struct sm_timer v8_start_timer;
    struct sm_timer v8_ci_timer;
    struct sm_timer v8_connect_timer;
    int v8_ci_count;
    FSK_mod_state v21_tx;
    FSK_demod_state v21_rx;
    struct sm_fifo rx_fifo;
    struct sm_fifo tx_fifo;
    u8 rx_buf[256];
    u8 tx_buf[256];
    V8_mod_state v8_tx;
    V8_demod_state v8_rx;

    /* available modulations */
    int modulation_mask;

    /* V8 data parsing */
    unsigned int bit_buf;
    unsigned int bit_sync;
    int bit_cnt;
    int data_state; /* indicates the type of synchro */
    u8 rx_data[64];
    int rx_data_ptr;
    
    /* CM/JM parsing */
    u8 cm_data[64];
    int cm_count;
    int got_cm;
    int decoded_modulations; /* modulation mask decoded from CM/JM */

    /* CJ parsing */
    int got_cj;
    int data_zero_count;

    int selected_mod_mask;   /* selected modulation mask */
    int selected_modulation; /* see V8_MOD_xxx */
} V8State;

/* V8 protocol definitions */
#define V8_MAX_CI_SEQ 10  /* maximum number of CI packets sent */

#define V8_TEN_ONES 0x3ff
#define V8_CI_SYNC  0x001
#define V8_CM_SYNC  0x00f

#define V8_CALL_FUNC_DATA 0x83

#define V8_MODN0     0xA0
#define V8_EXT       0x08

#define V8_MODN0_V90 0x04   /* FIXME */
#define V8_MODN0_V34 0x02
#define V8_MODN2_V21 0x01
#define V8_MODN2_V23 0x20

#define V8_DATA_NOCELULAR 0xB0
#define V8_DATA_LAPM	  0x54

#define V8_MOD_V90 (1 << 0)  /* FIXME */
#define V8_MOD_V34 (1 << 1)  /* V34 duplex */
#define V8_MOD_V23 (1 << 10) /* V23 duplex */
#define V8_MOD_V21 (1 << 12) /* V21 duplex */

#define V8_MOD_HANGUP 0x8000 /* indicate hangup */

void V8_init(V8State *sm, int calling, int mod_mask);
int V8_process(V8State *sm, s16 *output, s16 *input, int nb_samples);
