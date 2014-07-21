/* display.c */

int lm_display_init(void);
void lm_display_close(void);
int lm_display_poll_event(void);

void lm_dump_qam(float si, float sq);
void lm_dump_eye(int channel, float time, float val);

#define NB_CHANNELS 2
enum {
    CHANNEL_SAMPLE = 0,
    CHANNEL_SAMPLESYNC,
};
void lm_dump_sample(int channel, float val);
void lm_dump_agc(float gain);
void lm_dump_equalizer(s32 eq_filter1[][2], int norm, int size);
void lm_dump_echocancel(s32 eq_filter1[][2], int norm, int size);
void lm_dump_linesim_power(float tx_db, float rx_db, float noise_db);
