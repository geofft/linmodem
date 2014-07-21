# uncomment to use X11 debug interface
#USE_X11=y

CFLAGS= -O2 -Wall -g
LDFLAGS= -g
OBJS= lm.o lmsim.o lmreal.o lmsoundcard.o serial.o atparser.o \
      dsp.o fsk.o v8.o v21.o v23.o dtmf.o \
      v34.o v34table.o v22.o v34eq.o \
      v90.o v90table.o
INCLUDES= display.h   fsk.h       v21.h       v34priv.h   v90priv.h \
          dsp.h       lm.h        v23.h       v8.h \
          dtmf.h      lmstates.h  v34.h       v90.h
PROG= lm

ifdef USE_X11
OBJS += display.o
LDFLAGS += -L/usr/X11R6/lib -lX11
else
OBJS += nodisplay.o
endif

all: $(PROG) 

$(PROG): $(OBJS)
	gcc -o $(PROG) $(OBJS) -lm $(LDFLAGS)

v34gen: v34gen.o dsp.o
	gcc -o $@ v34gen.o dsp.o -lm $(LDFLAGS)

v34table.c: v34gen
	./v34gen > $@

v90gen: v90gen.o
	gcc -o $@ $< -lm $(LDFLAGS)

v90table.c: v90gen
	./v90gen > $@

linmodem.dvi: linmodem.tex
	latex2e linmodem.tex

clean:
	rm -f *.o *~ *.dat core gmon.out *.sw $(PROG) v34gen v90gen *.aux *.dvi *.log

tar:
	( cd .. ; tar zcvf linmodem.tgz linmodem --exclude CVS )

%.o: %.c $(INCLUDES)
	gcc $(CFLAGS) -c $*.c
