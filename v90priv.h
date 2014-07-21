#ifndef V90PRIV_H
#define V90PRIV_H

#define V90_SAMPLE_RATE 8000
#define TREILLIS_MAX_DEPTH   4  
/* ring buffer size of the previous ucodes : must be a power of two */
#define UCODE_BUF_SIZE   (TREILLIS_MAX_DEPTH * 8)

/* state of the signal processing part of the V90 modem */
typedef struct V90EncodeState {
    struct sm_state *sm;

    int alaw;    /* true=alaw, false=ulaw */

    /* frame parameters (a frame = 6 PAM values) */
    int S;       /* sign bits (3 <= S <= 6) */
    int K;       /* number of bits for the ring coder */
    int a1, a2, b1, b2; /* spectral conformer parameters */

    int M[6];    /* ring size */
    u8 m_to_ucode[6][128]; /* encoding table */
    int ld;      /* depth for spectral shaping (0 <= ld <= 3) */

    /* sign mapper */
    int ucode_ptr;                 /* index of the first pam value of the 
                                      current mapping frame */
    u8  ucode[UCODE_BUF_SIZE]; /* current output of the modulator */
    u8  pp[UCODE_BUF_SIZE];    /* corresponding signs (grouped by frame) */
    int  last_sign;                /* sign of the last computed frame */

    /* spectral shaping treillis */
    u8  t;      /* signs of the frame */
    s16 x;      /* last x value: not stricly needed, but simply the treillis computation */
    s16 y;      /* memory for spectral shaping filter */
    s16 v;      /* idem */
    s16 w;      /* idem */
    int Q;      /* current shaping treillis state
                   (with a delay of ld) */
    const s16 *ucode_to_linear;    /* table to retrieve the linear values from ucodes */
} V90EncodeState;

const s16 v90_ulaw_ucode_to_linear[128];
const s16 v90_alaw_ucode_to_linear[128];

typedef struct V90DecodeState {
    struct sm_state *sm;
    
    int alaw;    /* true=alaw, false=ulaw */

    /* frame parameters (a frame = 6 PAM values) */
    int S;       /* sign bits (3 <= S <= 6) */
    int K;       /* number of bits for the ring coder */

    int M[6];    /* ring size */
    u8 ucode_used[6][128];
    s16 m_to_linear[6][128]; /* give the estimated linear value of each received modulo */
    int ld;
    s8 a1, a2, b1, b2;
    int last_sign;
    int t;
    int Q;
    const s16 *ucode_to_linear;    /* table to retrieve the linear values from ucodes */
} V90DecodeState;

#endif
