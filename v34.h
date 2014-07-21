#ifndef V34_H
#define V34_H

#include "v34priv.h"

/* should be called once to init some V34 static tables */
void V34_static_init(void);

struct V34State; 

void V34_init(struct V34State *s, int calling);
int V34_process(struct V34State *s, s16 *output, s16 *input, int nb_samples);

/* V34 half duplex test with line simulator */
void V34_test(void);

#endif

