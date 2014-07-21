/* real modem: suitable for ltmodem */

/* Copyright 1999 Pavel Machek <pavel@suse.cz>, distribute under GPL v2 */

#include "lm.h"
#include <unistd.h>

#define NB_SAMPLES 0x50

extern struct sm_hw_info sm_hw_null;

void readit(int f, void *buf, int len)
{
        int res;
        while (len > 0) {
		if ((res = read(f, buf, len)) <= 0) {
                        printf("Read failed: %m");
			exit(5);
		}
                len -= res;
                buf += res;
        }
}

char *modem_command = "tee /tmp/delme.xmit | ltmodem -u 2> /dev/null | tee /tmp/delme.rec\n",
     *dial_number = "31415926";

void real_test(int calling)
{
    int fildesin[2];
    int fildesout[2];
    int i;
    struct sm_state sm, *dce = &sm;
    s16 in_buf[NB_SAMPLES];
    s16 out_buf[NB_SAMPLES];

    lm_init(dce, &sm_hw_null, "real");

    strcpy(dce->call_num,dial_number);

    pipe(fildesin);
    pipe(fildesout);

    printf( "Starting communication with ltmodem\n" );
    fflush(stdout); fflush(stderr);

    if (!fork()) {
      close(0);
      dup(fildesin[0]);
      close(1);
      dup(fildesout[1]);
      close(fildesin[0]);
      close(fildesin[1]);
      close(fildesout[0]);
      close(fildesout[1]);
      system( modem_command );
      exit(0);
    }

    /* test call */

    if (calling) {
        lm_start_dial(dce, 0, dial_number);
    } else {
        lm_start_receive(dce);
    }

    close(fildesin[0]);
    close(fildesout[1]);

    printf( "Kicking modem..." ); fflush(stdout);
    bzero(out_buf, 2*NB_SAMPLES);
    {
      int i;
      for (i=0; i<100; i++) {
	write(fildesin[1], out_buf, 2*NB_SAMPLES);
      }
    }
    printf( "Modem kicked\n" );

    /* answer_dce->state = SM_TEST_RING; */
    for(;;) {
        if (lm_get_state(dce) == LM_STATE_IDLE)
            break;

	readit(fildesout[0], in_buf, NB_SAMPLES*2);

        sm_process(dce, out_buf, in_buf, NB_SAMPLES);

	if ((i = write(fildesin[1], out_buf, NB_SAMPLES*2)) != NB_SAMPLES*2) {
	    printf( "Error writing -- got short sample (%d,%m)\n",i );
	} 
    }
    printf( "Looks like we are done\n" );
}
