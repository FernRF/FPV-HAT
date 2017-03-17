CC    = gcc
INCLUDE = -I/usr/local/include

DEBUG?=0

HALF_DUPLEX_FIFO?=1

VERSION=1.0
DEB_NAME=video-transfer

ifeq (1, $(DEBUG))
DBG=-g3 -D_DEBUG
else
DBG=-O2
endif

ifdef DEBUG_FILE
DBG += -DDEBUG_FILE=$(DEBUG_FILE)
endif

CFLAGS  = -std=gnu99 -D_GNU_SOURCE $(DBG) -Wall $(INCLUDE) -Winline -pipe -flto -march=armv6j -ftree-vectorize -mfloat-abi=hard -mfpu=vfp -mstructure-size-boundary=64

ifeq (1, $(HALF_DUPLEX_FIFO))
CFLAGS += -DHALF_DUPLEX_FIFO
endif


LDFLAGS = -L/usr/local/lib
LDLIBS  = -lwiringPi -lpthread -lm -lcrypto
#-lwiringPiDev -lpthread -lm

SI4463_SRC = si4463.c fec.c sender_2fsk.c sender_2gfsk.c sender_4fsk.c sender_4gfsk.c receiver_2fsk.c receiver_2gfsk.c receiver_4fsk.c receiver_4gfsk.c aes_crypto.c
SI4463_OBJ = $(SI4463_SRC:.c=.o)

LDLIBS += $(SI4463_OBJ)

SRC = receiver.c sender.c

OBJ = $(SRC:.c=.o)

BINS = $(SRC:.c=)

.PHONY: all

all: $(SI4463_OBJ) $(BINS) tests

.PHONY: clean

clean:
	rm -rf $(OBJ) $(BINS) $(SI4463_OBJ)
	cd tests && $(MAKE) clean

.PHONY: tests
tests:
	cd tests && $(MAKE)


.PHONY: deb

deb:
	tar cfz $(DEB_NAME)_$(VERSION).orig.tar.gz *.[ch] Makefile
	mv $(DEB_NAME)_$(VERSION).orig.tar.gz ../
	dpkg-buildpackage -us -uc

#tests: $(TESTS)
#$(TEST_SRC:.c=.o)

