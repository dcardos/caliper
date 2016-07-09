TARGET=pipe
include ../bench.mk

pipe: process.o pktbuff.o memcpy.o

cp:
	scp -P 19999 pipe.instr.mif pipe.instr.enc pipe.data.mif ${SKY}$(CPFOLDER)
