/* 
 * X11 interface for linmodem
 * 
 * Copyright (c) 2000 Fabrice Bellard.
 *
 * This code is released under the GNU General Public License version
 * 2. Please read the file COPYING to know the exact terms of the
 * license.
 */

#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/extensions/XShm.h>
#include <X11/keysym.h>

#include "lm.h"


#define NB_MODES 5

enum {
    DISP_MODE_SAMPLE, /* display the brut samples */
    DISP_MODE_ECHOCANCEL, /* display the echo cancellor info */
    DISP_MODE_SAMPLESYNC, /* display the samples after freq normalize
                             & symbol recovery */
    DISP_MODE_EQUALIZER, /* display the equalizer filter */
    DISP_MODE_QAM, /* display the qam */
} disp_state;

char *mode_str[NB_MODES] = {
    "Sample",
    "Echo Cancel",
    "Sample Sync",
    "Equalizer",
    "QAM",
};


#define QAM_SIZE 460

static void set_state(int state);

/**************************/

#define RGB(r, g, b) ((((r) >> 3) << 11) | (((g) >> 2) << 5) | ((b) >> 3))

Display *display;
Window window;
GC gc;
XFontStruct *xfont;
unsigned int fg, bg;
int minx, miny;

int font_xsize, font_ysize;

int lm_display_init(void)
{
    XSizeHints hint;
    int screen;
    XVisualInfo vinfo;
    int size_x = 640;
    int size_y = 480;
    XSetWindowAttributes xwa;
    char *fontname = "fixed";

    display = XOpenDisplay("");
    if (display == NULL) {
	return -1;
    }
    screen = DefaultScreen(display);

    bg = RGB(0, 0, 0);
    fg = RGB(255, 255, 255);

    /* Fill in hint structure */

    hint.x = 0;
    hint.y = 0;
    hint.width = size_x;
    hint.height = size_y;
    hint.flags = PPosition | PSize;

    /* Make the window */
    if (!XMatchVisualInfo(display, screen, 16, TrueColor, &vinfo)) {
        printf("A 16 bit visual is need by this program\n");
        return -1;
    }

    window = XCreateSimpleWindow(display,
                                 DefaultRootWindow(display),
                                 hint.x, hint.y,
                                 hint.width, hint.height,
                                 4, fg, bg);
    /* Enable backing store */
    xwa.backing_store = Always;
    XChangeWindowAttributes(display, window, CWBackingStore, &xwa);

    XSelectInput(display, window, StructureNotifyMask);

    /* Tell other applications about this window */

    XSetStandardProperties(display, window,
			   "linmodem", "linmodem",
			   None, NULL, 0, &hint);

    /* Map window. */

    XMapWindow(display, window);

    /* Wait for map. */
    while (1) {
	XEvent xev;
	XNextEvent(display, &xev);
	if (xev.type == MapNotify && xev.xmap.event == window)
	    break;
    }
    XSelectInput(display, window, KeyPressMask
		 | ButtonPressMask);

    gc = XCreateGC(display, window, 0, 0);

    xfont = XLoadQueryFont (display, fontname);
    if (!xfont) {
        fprintf(stderr, "Could not load font '%s'\n", fontname);
        return -1;
    }
    font_xsize = xfont->max_bounds.rbearing - xfont->min_bounds.lbearing;
    font_ysize = xfont->max_bounds.ascent + xfont->max_bounds.descent;

    set_state(DISP_MODE_SAMPLE);

    return 0;
}

void lm_display_close(void)
{
    XCloseDisplay(display);
}

void printf_at(int x, int y, char *fmt, ...)
{
    va_list ap;
    char buf[1024];
    
    va_start(ap, fmt);

    vsnprintf(buf, sizeof(buf), fmt, ap);
    
    XSetFont(display, gc, xfont->fid);

    XSetForeground(display, gc, bg);
    XFillRectangle(display, window, gc, 
                   x * font_xsize, y * font_ysize, 
                   font_xsize * strlen(buf), font_ysize);
                   
    XSetForeground(display, gc, fg);
    XDrawString(display, window, gc, x * font_xsize, (y + 1) * font_ysize - 1, 
                buf, strlen(buf));
    
    va_end(ap);
}

/* draw a graph at (x1, y1, x1 + w, y1 + h). The points are computed
   by calc_func */

#define DG_AUTOSCALE_YMIN 0x0001
#define DG_AUTOSCALE_YMAX 0x0002

void draw_graph(char *title,
                int x1, int y1, int w, int h,
                float xmin, float xmax, float ymin, float ymax,
                int flags, float (*calc_func)(float)) 
{
    int i, yy, xx;
    float x, y;
    int ly;
    float tab[1024];
    
    /* auto scale */
    for(i=0;i<w;i++) {
        x = ((float)i / (float)(w-1)) * (xmax - xmin) + xmin;
        y = calc_func(x);
        tab[i] = y;
        if (i == 0) {
            if (flags & DG_AUTOSCALE_YMIN)
                ymin = y;
            if (flags & DG_AUTOSCALE_YMAX)
                ymax = y;
        } else {
            if (flags & DG_AUTOSCALE_YMIN && y < ymin)
                ymin = y;
            if (flags & DG_AUTOSCALE_YMAX && y > ymax)
                ymax = y;
        }
    }

    /* draw ! */

    XSetForeground(display, gc, bg);
    XFillRectangle(display, window, gc, x1, y1, w, h);
    
    XSetForeground(display, gc, RGB(255, 0, 0));
    XDrawRectangle(display, window, gc, x1, y1, w, h);
    
    /* axis */
    XSetForeground(display, gc, RGB(0, 255, 0));
    if (xmin <= 0.0 && 0.0 <= xmax) {
        xx = (int)rint(((0.0 - xmin) / (xmax - xmin)) * (w-1)) + x1;
        XDrawLine(display, window, gc, 
                          xx, y1, xx, y1 + h - 1);
    }
    
    if (ymin <= 0.0 && 0.0 <= ymax) {
        yy = y1 + h - 1 - (int)rint(((0.0 - ymin) / (ymax - ymin)) * (h-1));
        XDrawLine(display, window, gc, 
                          x1, yy, x1 + w - 1, yy);
    }

    XSetForeground(display, gc, RGB(255, 255, 255));
    ly = -1;
    for(i=0;i<w;i++) {
        y = tab[i];
        if (y >= ymin && y <= ymax) {
            yy = y1 + h - 1 - (int)rint(((y - ymin) / (ymax - ymin)) * (h-1));
            if (ly >= 0) {
                XDrawLine(display, window, gc, 
                          i - 1, ly, i, yy);
            } else {
                XDrawPoint(display, window, gc, 
                           i, yy);
            }
            ly = yy;
        } else {
            ly = -1;
        }
    }


    /* title */
    XSetForeground(display, gc, RGB(0, 255, 0));
    
    printf_at((x1+font_xsize-1) / font_xsize,
              (y1+font_ysize-1) / font_ysize,
              "%s - ymin=%6.3e ymax=%6.3e", title, ymin, ymax);
             
}

/***************************************************/
/* utilities */


/***************************************************/

/* si and sq must be betwen -1.0 and 1.0 */

int nb_samples = 0;

void lm_dump_qam(float si, float sq)
{
    int x, y;

    if (disp_state != DISP_MODE_QAM)
        return;

    x = (int)(si * (QAM_SIZE/2)) + (QAM_SIZE/2);
    y = (int)(sq * (QAM_SIZE/2)) + (QAM_SIZE/2);
    if (x < 0 || x >= QAM_SIZE ||
        y < 0 || y >= QAM_SIZE)
        return;

    XSetForeground(display, gc, RGB(255, 255, 255));
    XDrawPoint(display, window, gc, x, y);

    nb_samples++;
    printf_at(minx, 1, "# samples: %d", nb_samples);
}

/* print samples */
#define NB_SAMPLES 512

float sample_mem[NB_CHANNELS][NB_SAMPLES];
int sample_pos[NB_CHANNELS];

float sample_hamming[NB_SAMPLES];
int sample_hamming_init = 0;
complex sample_fft[NB_SAMPLES];
int sample_channel;

float calc_sample(float x)
{
    return sample_mem[sample_channel][(int)x];
}

float calc_sample_pow(float x)
{
    complex *p = &sample_fft[(int)x];
    return p->re * p->re + p->im * p->im;
}

void draw_samples(int channel)
{
    int i;
    sample_channel = channel;

    draw_graph("Sample",
               0, 0, QAM_SIZE, QAM_SIZE/2,  
               0.0, NB_SAMPLES - 1, 0.0, 0.0, 
               DG_AUTOSCALE_YMAX | DG_AUTOSCALE_YMIN,
               calc_sample);
    
    if (!sample_hamming_init) {
        calc_hamming(sample_hamming, NB_SAMPLES);
        sample_hamming_init = 1;
    }
    
    for(i=0;i<NB_SAMPLES;i++) {
        sample_fft[i].re = sample_mem[channel][i] * sample_hamming[i];
        sample_fft[i].im = 0;
    }
    
    fft_calc(sample_fft, NB_SAMPLES, 0);
    
    draw_graph("Spectral power",
               0, QAM_SIZE/2, QAM_SIZE, QAM_SIZE/2,  
               0.0, NB_SAMPLES/2 - 1, 0.0, 0.0, 
               DG_AUTOSCALE_YMAX, calc_sample_pow);
}

void lm_dump_sample(int channel, float val)
{

    sample_mem[channel][sample_pos[channel]] = val;
    if (++sample_pos[channel] == NB_SAMPLES) {
        sample_pos[channel] = 0;
        
        if ((disp_state == DISP_MODE_SAMPLE &&
             channel == CHANNEL_SAMPLE) ||
            (disp_state == DISP_MODE_SAMPLESYNC &&
             channel == CHANNEL_SAMPLESYNC)) {
            
            draw_samples(channel);
        }
    }
}

#if 0
int last_eye_y[2];
int last_eye_x[2];

/* print an eye diagram (each symbol is sampled at integer time
   values). We print 1 centered period */
void lm_dump_eye(int channel, float time, float val)
{
    int x, y;

    if (disp_state != DISP_MODE_EYE)
        return;

    if (val < -1)
        val = -1;
    else if (val > 1)
        val = 1;

    y = (int)(val * (QAM_SIZE/2)) + (QAM_SIZE/2);
    time += 0.5;
    if (time >= 1.0)
        time -= 1.0;
    x = (int)(time * QAM_SIZE);
    
    XSetForeground(display, gc, RGB(255, 255, 255));
    if (x > last_eye_x[channel] && 0) {
        XDrawLine(display, window, gc, 
                  last_eye_x[channel], last_eye_y[channel], x, y);
    } else {
        XDrawPoint(display, window, gc, x, y);
    }
    last_eye_x[channel] = x;
    last_eye_y[channel] = y;
}
#endif

/* print equalizer */

#define EQ_FFT_SIZE 144

static int eq_count = 0;
static s32 (*eq_filter)[2];
static int eq_norm;
static complex eq_fft[EQ_FFT_SIZE];

float calc_eq_re(float x)
{
    return (float)eq_filter[(int)rint(x)][0] / (float)eq_norm;
}

float calc_eq_im(float x)
{
    return (float)eq_filter[(int)rint(x)][1] / (float)eq_norm;
}

float calc_eq_pow(float x)
{
    complex *p = &eq_fft[(int)x];
    return p->re * p->re + p->im * p->im;
}

float calc_eq_phase(float x)
{
    complex *p = &eq_fft[(int)x];
    return atan2(p->im, p->re);
}

void lm_dump_equalizer(s32 eq_filter1[][2], int norm, int size)
{
    int i;
    
    if (disp_state != DISP_MODE_EQUALIZER)
        return;
    
    if (++eq_count == 1) {
        eq_count = 0;
        eq_filter = eq_filter1;
        eq_norm = norm;

        draw_graph("Eqz real",
                   0, 0, QAM_SIZE, QAM_SIZE/4,  
                   0.0, size - 1, -1.5, 1.5, 
                   0,
                   calc_eq_re);

        draw_graph("Eqz imag",
                   0, QAM_SIZE/4, QAM_SIZE, QAM_SIZE/4,  
                   0.0, size - 1, -1.5, 1.5, 
                   0,
                   calc_eq_im);

        for(i=0;i<EQ_FFT_SIZE;i++) {
            if (i < size) {
                eq_fft[i].re = eq_filter[i][0] / (float)norm;
                eq_fft[i].im = eq_filter[i][1] / (float)norm;
            } else {
                eq_fft[i].re = eq_fft[i].im = 0;
            }
        }
        
        if (EQ_FFT_SIZE == 144) {
            complex eq_fft1[144];

            slow_fft(eq_fft1, eq_fft, EQ_FFT_SIZE, 0);
            for(i=0;i<EQ_FFT_SIZE;i++)
                eq_fft[i] = eq_fft1[i];
            
        } else {
            fft_calc(eq_fft, EQ_FFT_SIZE, 0);
        }

        draw_graph("Eqz spec pow",
                   0, 2*QAM_SIZE/4, QAM_SIZE, QAM_SIZE/4,  
                   0.0, EQ_FFT_SIZE - 1, 0.0, 0.0,
                   DG_AUTOSCALE_YMAX, calc_eq_pow);

        draw_graph("Eqz spec phase",
                   0, 3*QAM_SIZE/4, QAM_SIZE, QAM_SIZE/4,  
                   0.0, EQ_FFT_SIZE - 1, -M_PI, M_PI,
                   0, calc_eq_phase);
    }
}


/* agc */

void lm_dump_agc(float gain)
{
    printf_at(minx, 2, "AGC: %10.5f", gain);
}

void lm_dump_linesim_power(float tx_db, float rx_db, float noise_db)
{
    printf_at(minx, 3, "TX: %6.2f dB SNR: %6.2f dB", tx_db, rx_db - noise_db);
    printf_at(minx, 4, "RX: %6.2f dB  N0: %6.2f dB", rx_db, noise_db);
}

static void set_state(int state)
{
    int i;

    minx = ((QAM_SIZE + font_xsize) / font_xsize) + 1;
    miny = ((QAM_SIZE + font_ysize) / font_ysize);
    
    disp_state = state;

    XSetForeground(display, gc, bg);
    XFillRectangle(display, window, gc, 0, 0, 640, 480);

    XSetForeground(display, gc, RGB(255, 0, 0));
    XDrawRectangle(display, window, gc, 0, 0, QAM_SIZE, QAM_SIZE);

    switch(disp_state) {
    case DISP_MODE_QAM:
        XDrawLine(display, window, gc, 0, QAM_SIZE/2, QAM_SIZE-1, QAM_SIZE/2);
        XDrawLine(display, window, gc, QAM_SIZE/2, 0, QAM_SIZE/2, QAM_SIZE-1);
        break;
    default:
        break;
    }
    printf_at(minx, 0, "Mode: %s", mode_str[disp_state]);
    
    for(i=0;i<NB_MODES;i++) {
        printf_at(1 + 15 * i, miny, "F%d:%s", i + 1, mode_str[i]);
    }
}

int lm_display_poll_event(void)
{
    char buf[80];
    XEvent xev;
    KeySym keysym;
    XComposeStatus status;


    if (XPending(display) <= 0)
        return 0;

    XNextEvent(display, &xev);
    switch(xev.type) {
    case KeyPress:
	XLookupString((XKeyEvent *) & xev, buf, 80, &keysym, &status);
        switch(keysym) {
        case XK_q:
            return 1;
        case XK_F1:
        case XK_F2:
        case XK_F3:
        case XK_F4:
        case XK_F5:
        case XK_F6:
        case XK_F7:
        case XK_F8:
            {
                int mode;

                mode = keysym - XK_F1;
                if (mode < NB_MODES) {
                    set_state(mode);
                }
            }
            break;
        }
        break;
    }
    return 0;
}


