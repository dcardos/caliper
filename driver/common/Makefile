#
# $Id: Makefile 3087 2007-12-08 05:07:12Z jnaous $
#

all: nf2util.o

libnf2.so: nf2util.c nf2util.h nf2.h
	gcc -fpic -c nf2util.c
	gcc -shared nf2util.o -o $@

clean :
	rm -rf nf2util.o libnf2.so

.PHONY: clean
