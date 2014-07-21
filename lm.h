#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#include "dsp.h"

#define LM_VERSION "0.2.5"

/* bit fifo */

struct sm_fifo {
    unsigned char *sptr, *wptr, *rptr, *eptr;
    int size, max_size;
};

void sm_flush(struct sm_fifo *f);
int sm_size(struct sm_fifo *f);
void sm_put_bit(struct sm_fifo *f, int v);
void sm_put_bits(struct sm_fifo *f, int v, int n);
int sm_get_bit(struct sm_fifo *f);
int sm_get_bits(struct sm_fifo *f, int n);
void sm_init_fifo(struct sm_fifo *f, u8 *buf, int size);

/* bit I/O for data pumps */
typedef void (*put_bit_func)(void *opaque, int bit);
typedef int (*get_bit_func)(void *opaque);

/* timer */
struct sm_timer {
    long timeout;
};

int sm_time(void);
void sm_set_timer(struct sm_timer *t, int delay);
int sm_check_timer(struct sm_timer *t);

/* debug */
extern int lm_debug;
extern char *sm_states_str[];

/* protocol description */

#include "dtmf.h"
#include "fsk.h"
#include "v21.h"
#include "v22.h"
#include "v23.h"
#include "v8.h"
#include "v34.h"

/* modem state */
#define SM_FIFO_SIZE 4096

struct sm_state {
    /* pretty name of the modem (to debug) */
    char name[16];

    struct sm_hw_info *hw;
    struct lm_interface_state *hw_state;

    /* bytes to be transmitted */
    struct sm_fifo tx_fifo;
    u8 tx_fifo_buf[SM_FIFO_SIZE];

    /* received chars */
    struct sm_fifo rx_fifo;
    u8 rx_fifo_buf[SM_FIFO_SIZE];

    /* true if we are the caller */
    int calling;
    /* phone number to call */
    char call_num[64];
    int pulse_dial; /* TRUE if we must use pulse dialing */

    /* dialing */
    struct sm_timer dtmf_timer;
    DTMF_mod_state dtmf_tx;
    int dtmf_ptr, dtmf_len;   /* pointer in call_num */

    /* dtmf receive: for testing (or voice mode in the future) */
    struct sm_timer ring_timer;
    DTMF_demod_state dtmf_rx;

    /* modulation state */
    union {
        V8State v8_state;
        V21State v21_state;
        V23State v23_state;
    } u;

    /* serial state */
    int serial_data_bits; /* 5 to 8 */
    int serial_parity, serial_use_parity;
    int serial_wordsize;

    unsigned int serial_buf;
    int serial_cnt;
    
    unsigned int serial_tx_buf;
    int serial_tx_cnt;

    /* main modem state */
    int state;
    int debug_laststate;
    int hangup_request;
    unsigned int time; /* current time (in samples) */

    /* config */
    struct LinModemConfig *lm_config;
};

/* linmodem configuration registers */
typedef struct LinModemConfig {
    int pulse_dial;   /* default dial type */
    int dtmf_level;   /* in dB */
    int dtmf_digit_length; /* in ms */
    int dtmf_pause_length; /* in ms */
    int available_modulations; /* mask of available modulations */
} LinModemConfig;

/* abstract line interface driver */
struct lm_interface_state {
    struct sm_state *sm;
    int handle;
};

/* modem hardware interface abstraction */
/* XXX: must be slightly modified for kernel mode operation */
struct sm_hw_info {
    int (*open)(struct lm_interface_state *s);
    void (*close)(struct lm_interface_state *s);

    /* off hook or on hook modem */
    void (*set_offhook)(struct lm_interface_state *s, int v);

    /* when the ring is enabled, an event E_RING is sent to the
       process if a ring occured */
    void (*set_ring)(struct lm_interface_state *s, int v);

    /* the main modem loop is here */
    void (*main_loop)(struct lm_interface_state *s);
};

/* general state of the modem */

enum {
#define TAG(s) s,
#include "lmstates.h"
};

void lm_init(struct sm_state *sm, struct sm_hw_info *hw, const char *name);

/* main modem process */
void sm_process(struct sm_state *sm, s16 *output, s16 *input, int nb_samples);

int lm_start_dial(struct sm_state *s, int pulse, const char *number);
int lm_start_receive(struct sm_state *s);
int lm_hangup(struct sm_state *s);

enum lm_get_state_val {
    LM_STATE_IDLE,
    LM_STATE_CONNECTING,
    LM_STATE_CONNECTED,
};

enum lm_get_state_val lm_get_state(struct sm_state *s);

/* lmsim.c */

void line_simulate(void);

struct LineModelState;

struct LineModelState *line_model_init(void);
void line_model(struct LineModelState *s, 
                s16 *output1, const s16 *input1,
                s16 *output2, const s16 *input2,
                int nb_samples);

/* lmreal.c */

void real_test(int calling);

/* lmsoundcard.c */
void soundcard_modem(void);

/* serial.c */

void serial_init(struct sm_state *s, int data_bits, int parity);
int serial_get_bit(void *opaque);
void serial_put_bit(void *opaque, int bit);

/* atparser.c */

enum lm_at_state_type {
    AT_MODE_COMMAND,
    AT_MODE_DIALING,
    AT_MODE_CONNECTED,
};

struct lm_at_state {
    char at_line[256];
    int at_line_ptr;
    int at_state; 
    struct sm_state *sm; /* corresponding modem state */
};

void lm_at_parser_init(struct lm_at_state *s, struct sm_state *sm);
void lm_at_parser(struct lm_at_state *s);

#include "display.h"
