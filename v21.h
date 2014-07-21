
void V21_mod_init(FSK_mod_state *s, int calling, get_bit_func get_bit, void *opaque);
void V21_demod_init(FSK_demod_state *s, int calling, put_bit_func put_bit, void *opaque);

typedef struct {
    FSK_mod_state tx;
    FSK_demod_state rx;
} V21State;

void V21_init(V21State *s, int calling, 
              get_bit_func get_bit, put_bit_func put_bit, void *opaque);
int V21_process(V21State *sm, s16 *output, s16 *input, int nb_samples);

