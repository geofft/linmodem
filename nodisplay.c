/*
 * Stubs for X11 interface for linmodem
 *
 * Copyright (c) 2000 Fabrice Bellard, 2014 Geoffrey Thomas
 *
 * This code is released under the GNU General Public License version
 * 2. Please read the file COPYING to know the exact terms of the
 * license.
 */

#include "lm.h"

int lm_display_init(void)
{
    return 0;
}

void lm_display_close(void)
{
}

void lm_dump_qam(float si, float sq)
{
}

void draw_samples(int channel)
{
}

void lm_dump_sample(int channel, float val)
{
}

void lm_dump_equalizer(s32 eq_filter1[][2], int norm, int size)
{
}

void lm_dump_agc(float gain)
{
}

void lm_dump_linesim_power(float tx_db, float rx_db, float noise_db)
{
}

int lm_display_poll_event(void)
{
    return 0;
}
