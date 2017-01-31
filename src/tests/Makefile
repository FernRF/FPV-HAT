ifeq (1, $(DEBUG))
DBG=-g3 -D_DEBUG
else
DBG=-O2
endif

CC    = gcc
INCLUDE = -I../
CFLAGS  = $(DBG) -Wall $(INCLUDE) -Winline -pipe -flto -march=armv6j -ftree-vectorize -mfloat-abi=hard -mfpu=vfp -mstructure-size-boundary=64 

LDFLAGS = -L/usr/local/lib 
LDLIBS  = ../si4463.o ../fec.o -lwiringPi -lpthread -lm 
#-lwiringPiDev -lpthread -lm

SRC = connect.c info.c rx_long.c tx_long.c tx_fix.c rx_fix.c fec_test.c sleep.c

OBJ = $(SRC:.c=.o)

BINS = $(SRC:.c=)

.PHONY: tests

all: tests

.PHONY: clean

clean:
	rm -rf $(OBJ) $(BINS)

tests: $(BINS)
	cp info si4463info
	cp sleep si4463sleep

