/* sample interface code to use a linux soundcard */
#define _XOPEN_SOURCE
#define _XOPEN_SOURCE_EXTENDED
#include <stdlib.h>
#include <sys/select.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/soundcard.h>

#include "lm.h"

/* OSS buffer size */
#define FRAGMENT_BITS 8
#define NB_FRAGMENTS  (32768 >> FRAGMENT_BITS)

/* 128 samples : 16 ms, maybe too much ? */
#define NB_SAMPLES ((1 << FRAGMENT_BITS)/2)

/* default pty */

extern struct sm_hw_info sm_hw_soundcard;

void soundcard_modem(void)
{
    struct sm_state sm1, *dce = &sm1;
    struct sm_hw_info *hw = &sm_hw_soundcard;
    s16 in_buf[NB_SAMPLES];
    s16 out_buf[NB_SAMPLES];
    u8 buf[1024];
    struct lm_at_state at_parser;
    fd_set rfds, wfds;
    int hw_handle, tty_handle, max_handle, n;
    int out_buf_flushed, i, len;
    
    if (!lm_debug) {
        tty_handle = open("/dev/ptmx", O_RDWR);
        if (tty_handle < 0) {
            perror("/dev/ptmx");
            return;
        }
        if (grantpt(tty_handle) < 0) {
            perror("grantpt");
            return;
        }
        if (unlockpt(tty_handle) < 0) {
            perror("unlockpt");
            return;
        }
        printf("linmodem tty is '%s'\n", ptsname(tty_handle));
    } else {
        printf("linmodem tty is stdout\n");
        tty_handle = 0;
    }
    fcntl(tty_handle, F_SETFL, O_NONBLOCK);

    lm_init(dce, hw, "sc");
    lm_at_parser_init(&at_parser, dce);


    hw_handle = dce->hw_state->handle;
    out_buf_flushed = 1;
    /* XXX: non block mode ? */

    /* test call */
    printf("linmodem started.\n");
    for(;;) {
        /* sound card & tty handling */
        FD_ZERO(&rfds);
        FD_SET(hw_handle, &rfds);
        FD_SET(tty_handle, &rfds);
        FD_ZERO(&wfds);
        if (!out_buf_flushed) 
            FD_SET(hw_handle, &wfds);
        if (sm_size(&dce->rx_fifo) > 0)
            FD_SET(tty_handle, &wfds);
        max_handle = tty_handle;
        if (hw_handle > tty_handle)
            max_handle = hw_handle;

        n = select(max_handle + 1, &rfds, &wfds, NULL, NULL);
        if (n > 0) {
            /* process at commands */
            lm_at_parser(&at_parser);

            /* read from tty */
            if (FD_ISSET(tty_handle, &rfds)) {
                len = read(tty_handle, buf, sizeof(buf));
                for(i=0;i<len;i++) {
                    sm_put_bit(&dce->tx_fifo, buf[i]);
                }
            }
            /* write to tty */
            if (FD_ISSET(tty_handle, &wfds)) {
                struct sm_fifo *f = &dce->rx_fifo;
                int size;
                size = f->eptr - f->rptr;
                if (size > f->size)
                    size = f->size;
                len = write(tty_handle, f->rptr, size);
                if (len > 0) {
                    f->rptr += len;
                    f->size -= len;
                    if (f->rptr == f->eptr)
                        f->rptr = f->sptr;
                }
            }

            /* we assume that the modem read & write per block of
               NB_SAMPLES. It makes no sense the underlying hardware
               does something else */

            /* read from modem */
            if (FD_ISSET(hw_handle, &rfds) && out_buf_flushed) {
                len = read(hw_handle, in_buf, NB_SAMPLES * 2);
                if (len == NB_SAMPLES * 2) {
                    /* process the modem samples */
                    sm_process(dce, out_buf, in_buf, NB_SAMPLES);
                    out_buf_flushed = 0;
                }
            }
            /* write to modem */
            if (FD_ISSET(hw_handle, &wfds)) {
                len = write(hw_handle, out_buf, NB_SAMPLES * 2);
                if (len == NB_SAMPLES * 2) {
                    out_buf_flushed = 1;
                }
            }
        }
    }
}

static int soundcard_open(struct lm_interface_state *s)
{
    int tmp, err;
    /* init the sound card to 8000 Hz, Mono, 16 bits */

    s->handle = open("/dev/dsp", O_RDWR);
    if (s->handle < 0) {
        perror("/dev/dsp");
        exit(1);
    }

    /* set the card to duplex */
    tmp=0;
    err=ioctl(s->handle, SNDCTL_DSP_SETDUPLEX, &tmp);
    if (err < 0) {
        perror("SNDCTL_DSP_SETDUPLEX");
    }
    
    /* buffer size */
    tmp=(NB_FRAGMENTS << 16) | FRAGMENT_BITS;
    err=ioctl(s->handle, SNDCTL_DSP_SETFRAGMENT, &tmp);
    if (err < 0) {
        perror("set fragment");
    }
    
    tmp=AFMT_S16_LE;
    err=ioctl(s->handle,SNDCTL_DSP_SETFMT,&tmp);
    if (err < 0) goto error;
    
    /* should be last */
    tmp = 8000;
    err=ioctl(s->handle,SNDCTL_DSP_SPEED,&tmp);
    if (err < 0) goto error;

    return 0;
 error:
    close(s->handle);
    return -1;
}

static void soundcard_close(struct lm_interface_state *s)
{
    close(s->handle);
}

static void soundcard_set_offhook(struct lm_interface_state *s, int v)
{
    if (v) {
        printf("%s: offhook\n",s->sm->name);
    } else {
        printf("%s: onhook\n",s->sm->name);
    }
}

static void soundcard_set_ring(struct lm_interface_state *s, int v)
{
}

static void soundcard_main_loop(struct lm_interface_state *s)
{


}

struct sm_hw_info sm_hw_soundcard = 
{
    soundcard_open,
    soundcard_close,
    soundcard_set_offhook,
    soundcard_set_ring,
    soundcard_main_loop,
};

