
void V23_mod_init(FSK_mod_state *s, int calling, get_bit_func get_bit, void *opaque);
void V23_demod_init(FSK_demod_state *s, int calling, put_bit_func put_bit, void *opaque);

typedef struct {
    FSK_mod_state tx;
    FSK_demod_state rx;
} V23State;

void V23_init(V23State *s, int calling, 
              get_bit_func get_bit, put_bit_func put_bit, void *opaque);
int V23_process(V23State *s, s16 *output, s16 *input, int nb_samples);
