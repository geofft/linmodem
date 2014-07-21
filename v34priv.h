#ifndef V34PRIV_H
#define V34PRIV_H

#define MAX_MAPPING_FRAME_SIZE 79
#define M_MAX 18

/* symbol rate */
enum {
  V34_S2400,
  V34_S2743,
  V34_S2800,
  V34_S3000,
  V34_S3200,
  V34_S3429,
};

/* constellation parameters */
#define L_MAX 1664   /* max number of points in the constellation */
#define C_MIN   -11
#define C_MAX   11
#define C_MAX_SIZE ((C_MAX-C_MIN+1)*(C_MAX-C_MIN+1))
#define C_RADIUS (2*(C_MAX-C_MIN)+1) /* max coordinate of the constellation */
#define SYNC_PATTERN 0x77FA /* (table 12) synchronisation pattern for J=8 */

#define V34_SAMPLE_RATE_NUM 10
#define V34_SAMPLE_RATE_DEN 3
#define V34_SAMPLE_RATE ((2400*V34_SAMPLE_RATE_NUM)/V34_SAMPLE_RATE_DEN)

/* size of the raised root cosine filter (for both rx & tx) */
#define RC_FILTER_SIZE 40

#define TX_BUF_SIZE (2048)

#define RX_BUF1_SIZE   256
#define RX_BUF2_SIZE   256

/* size of the complex equalizer filter */
#define EQ_FRAC        3
#define EQ_SIZE        (52*EQ_FRAC)

#define AGC_WINDOW_SIZE 512 /* must be a power of two, in input samples */

#define TRELLIS_MAX_STATES 64
/* 5 times the constraint length */
#define TRELLIS_LENGTH (6*5)

/* 10 fractional bits for nyquist filters */
#define NQ_BITS 10
#define NQ_BASE (1 << NQ_BITS)

/* state of the signal processing part of the V34 transmitter */
typedef struct V34DSPState {
  /* V34 parameters */
  int calling; /* true if we are the caller */ 
  int S; /* index for symbol rate */
  int expanded_shape; /* true if expanded shape used */
  int R; /* transmit rate (in bits/s, including aux channel) */
  int conv_nb_states; /* number of states of the convolutional coder */
  int use_non_linear;
  int use_high_carrier;
  s16 h[3][2]; /* precoding coefficients (14 bits fractional part) */

    void *opaque;
    get_bit_func get_bit;  
    
    put_bit_func put_bit;  
  
  /* do not modify after this */

  int N; /* total number of bits in a data frame */
  int W; /* number of aux bits in a data frame (0 = no aux channel) */
  int J; /* number of data frame in a super frame */
  int P; /* number of mapping frame in a data frame */
  int b; /* max length of a mapping frame */
  int r; /* counter to know the length of the mapping frame */
  int K; /* mapping parameters */
  int q;
  int L; /* current number of points of the constellation */
  int M; /* current number of rings */ 
  int Z_1; /* previous Z value (see § 9.5) */

    int mapping_frame; /* number of the mapping frame */
    int rcnt, acnt;    /* fractional counters to know the number of bits
                          in a mapping frame */
  int half_data_frame_count;    /* number of half data frame */
  int sync_count; /* counter mod 2P for synchronisation */
  s16 x[3][2]; /* 3 most recent samples for precoding (7 bit fractional part) */
  int U0;
  int conv_reg; /* memory of the convolutional coder */
  int scrambler_reg; /* state of the self synchronizing scrambler */
  float carrier_freq; 
  float symbol_rate; 
    s8 constellation[C_MAX_SIZE][2];

    /* precomputed bases for the ring computation */
    int g2_tab[8*(M_MAX - 1) + 1];
    int g4_tab[8*(M_MAX - 1) + 1];
    int g8_tab[8*(M_MAX - 1) + 1];
    int z8_tab[8*(M_MAX - 1) + 1];
    
    /* for decoding only */
    u16 constellation_to_code[C_RADIUS+1][C_RADIUS+1];

    /* for encoding only */
    s16 *tx_filter;
    s16 tx_buf[TX_BUF_SIZE][2];
    int tx_buf_ptr, tx_outbuf_ptr, tx_buf_size;
    int tx_filter_wsize;
    int baud_num, baud_denom;
    int baud_incr;
    int baud_phase;
    int carrier_phase;
    int carrier_incr;

    s16 tx_amp; /* amplitude for transmit : each symbol is multiplied
                   by it (1:8:7) */

    int baud3_phase;
    s16 *rx_filter;
    int rx_filter_wsize;
    s16 rx_buf1[RX_BUF1_SIZE];
    int rx_buf1_ptr;
    
    /* symbol synchronization */
    s16 sync_low_mem[2];
    s16 sync_low_coef[2];
    s16 sync_high_mem[2];
    s16 sync_high_coef[2];
    s16 sync_A, sync_B, sync_C;

    /* equalizer */
    s32 eq_filter[EQ_SIZE][2];
    s16 eq_buf[EQ_SIZE];
    int eq_buf_ptr;
    int eq_shift;

    /* AGC */
    float agc_mem;
    float agc_coef;
    int agc_power;
    int agc_gain;
    
    /* Viterbi decoder */

    /* the previous decoded decision comming to this path. Each
       decision Y[5] is coded on one byte */

    u8 state_decision[TRELLIS_MAX_STATES][TRELLIS_LENGTH];
    u8  state_path[TRELLIS_MAX_STATES][TRELLIS_LENGTH];
    s16 state_memory[TRELLIS_LENGTH][4];
    u8  u0_memory[TRELLIS_LENGTH];
    int state_error[TRELLIS_MAX_STATES];
    int state_error1[TRELLIS_MAX_STATES];
    int trellis_ptr;

    /* decoder synchronization */
    int phase_4d; /* index of the current 2d symbol in the 4D symbol
                     (0 or 1) */
    int phase_mse; /* MSE to find if we are synchronized on a 4D symbol */
    int phase_mse_cnt;

    s16 yy[2][2]; /* current 4D symbol */
    s16 rx_mapping_frame[2*4][2]; 
    int rx_mapping_frame_count;

    /* rx state */
    int sym_count;

    /* current V34 protocol state */
    int state;
    int is_16states; /* 16 states required in the startup sequences */

    /* interaction with receiver */
    int J_received;
    int JP_received;
} V34DSPState;

u8 trellis_trans_4[256][4];
u8 trellis_trans_8[256][4];
u8 trellis_trans_16[256][4];

/* V34 states */
enum {
    V34_STARTUP3_S1,
    V34_STARTUP3_SINV1,
    V34_STARTUP3_S2,
    V34_STARTUP3_SINV2,
    V34_STARTUP3_PP,
    V34_STARTUP3_TRN,
    V34_STARTUP3_J,
    V34_STARTUP3_JP,
    V34_STARTUP3_WAIT_J,

    V34_STARTUP4_S,
    V34_STARTUP4_WAIT_JP,
    V34_STARTUP4_SINV,
    V34_STARTUP4_TRN,
    V34_STARTUP4_MP,
    V34_STARTUP4_MPP,
    V34_STARTUP4_E,
    V34_DATA,

    /* receive only */
    V34_STARTUP3_WAIT_S1,
};

void put_bits(u8 **pp, int n, int bits);
int calc_crc(u8 *buf, int size);
void v34_send_info0(V34DSPState *s, int ack);

#define DSPK_TX_FILTER_SIZE 321
extern s16 v34_dpsk_tx_filter[DSPK_TX_FILTER_SIZE];

extern s16 v34_rc_5_filter[];
extern s16 v34_rc_7_filter[];
extern s16 v34_rc_8_filter[];
extern s16 v34_rc_10_filter[];
extern s16 v34_rc_20_filter[];
extern s16 v34_rc_35_filter[];

extern s16 v34_rx_filter_2400_1600[];
extern s16 v34_rx_filter_2400_1800[];
extern s16 v34_rx_filter_2743_1646[];
extern s16 v34_rx_filter_2743_1829[];
extern s16 v34_rx_filter_2800_1680[];
extern s16 v34_rx_filter_2800_1867[];
extern s16 v34_rx_filter_3000_1800[];
extern s16 v34_rx_filter_3000_2000[];
extern s16 v34_rx_filter_3200_1829[];
extern s16 v34_rx_filter_3200_1920[];
extern s16 v34_rx_filter_3429_1959[];

/* v34eq.c */
typedef struct {
    s16 re, im;
} icomplex;

#define V34_PP_SIZE 48
extern icomplex tabPP[V34_PP_SIZE];

void V34eq_init(void);
void V34_fast_equalize(V34DSPState *s, s16 *input);

typedef struct V34State {
    /* V34 parameters test */
    int calling; /* true if we are the caller */
    int S; /* index for symbol rate */
    int expanded_shape; /* true if expanded shape used */
    int R; /* transmit rate (in bits/s, excluding aux channel) */
    int conv_nb_states; /* number of states of the convolutional coder */
    int use_non_linear;
    int use_high_carrier;
    int use_aux_channel;
    s16 h[3][2]; /* precoding coefficients (14 bits fractional part) */

    V34DSPState v34_tx;
    V34DSPState v34_rx;
} V34State;

#endif

