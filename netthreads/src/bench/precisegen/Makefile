TARGETS= precisegen

include ../bench.mk

# Chooses which port will be used to send packets
OUTPUT_PORT?=0

EXTRA_DEFINES+=-DOUTPUT_PORT=$(OUTPUT_PORT)

ifeq ($(CONTEXT),sim)
	EXTRA_DEFINES+=-DINPUT_PORT=0
else
	EXTRA_DEFINES+=-DINPUT_PORT=1
endif

#CFLAGS+=-Werror

precisegen: precisegen.o pktbuff.o workqueue.o mtdeque.o deque.o memcpy.o mif