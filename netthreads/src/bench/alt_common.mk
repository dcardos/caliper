HOME=/localdisk/work/my
SPE_PREFIX=$(HOME)/bin/mips-mips-elf
SPE_COMPILER=$(SPE_PREFIX)-gcc
SPE_AS=$(SPE_PREFIX)-as 
SPE_AR=$(SPE_PREFIX)-ar 
SPE_LD=$(SPE_PREFIX)-ld 
SPE_DIS=$(SPE_PREFIX)-objdump 

#################  DEFAULTS ##############
 # override these in the individual Makefiles if you wish
 ##########################################

DLY=b
MULT=o
START = nf2.o
LSI = ../common/nf2_linker.ld
UDEFINCS = -I. -I../common -I/usr/include -I$(BASE)/compiler/lib/gcc/mips-mips-elf/4.0.2/include/
VPATH=../common


EXTRA=-march=sp -mbias${BYPASS} -mmult${MULT} -mnodly${DLY} -Wall


GCCLIB=$(HOME)/lib/gcc/mips-mips-elf/4.0.2
TOOLS=$(SPE_HOME)/applications/tools
MULTI_TOOLS=$(SPE_HOME)/applications/multi_tools

FUNC_LIB=$(TOOLS)/libspe.a
FLOATING= $(TOOLS)/libgcc/soft-float/libgcc.a
MATH= $(TOOLS)/mylibm.a
EXTRA_LIB=$(FUNC_LIB) $(FUNC_LIB) $(TOOLS)/mylibc.a

#ORIG_LIB = -L $(GCCLIB)/soft-float -lm -lc  -lgcc  $(FUNC_LIB)
ORIG_LIB = $(FUNC_LIB) $(TOOLS)/mylibc.a  $(MATH) $(TOOLS)/mylibc.a $(FLOATING) $(FUNC_LIB)
LDFLAGS= -T $(LSI) $(ORIG_LIB) --print-map 


ifndef SINGLE_PROGRAM
CFLAGS= -O3  -mgp32 -msoft-float -mlong32 $(EXTRA)  -fno-builtin  -DINTERNAL_NEWLIB -DDEFINE_MALLOC  -DREENTRANT_SYSCALLS_PROVIDED # -mno-rnames  -mips1
else
CFLAGS= -O3  -mgp32 -msoft-float -mlong32 -G 100000 $(EXTRA) -DDEFINE_MALLOC  -DINTERNAL_NEWLIB -DREENTRANT_SYSCALLS_PROVIDED # -mno-rnames  -mips1
endif

ASFLAGS= -O3 -march=sp
INCS = $(UDEFINCS)

#cancel this implicit rule, saves putting the command line option
#this causes the compilation of c files to go through assembly
#which is needed to get rid of shifts
%.o : %.c
%.o : %.C
%.s : %.S

#.c.o: 
#	$(SPE_COMPILER) $(CFLAGS) -c -o $*.o $<


MACROS = $(SPE_HOME)/applications/multi_tools/c.m4.sp

%.h: %.H
	m4 ${MACROS} $*.H > $*.h

%.c: %.C
	m4 $(MACROS) $*.C > $*.c

# this makes software only implementations
PATCH_ORIG= awk -f $(SPE_HOME)/applications/patch.awk -f $(SPE_HOME)/applications/patch_match.awk 

#this makes use of hard shifters
AUX=$(subst _, ,$(HARDSHIFTS))
PATCH= awk -f $(SPE_HOME)/applications/patch.awk -f $(SPE_HOME)/applications/patch_sub.awk $(AUX)
LOFIXER=awk -f $(SPE_HOME)/applications/multi_tools/lo_gp_fixer.awk

# if you change this, don't forget to change scripts in tool/libgcc
%.o:%.s
ifdef NOSHIFT
	echo "HARDSHIFTS is "$(HARDSHIFTS)
	$(PATCH)  $< > $*.S
	$(SPE_AS) $(ASFLAGS) -o $*.o $*.S
	rm $*.S
else
ifdef DATA_SHIFTING
	$(LOFIXER) $< > $*.S; mv $*.S $*.s 
endif
	$(SPE_AS) $(ASFLAGS) -o $*.o $<
endif



%.s:%.c
	$(SPE_COMPILER) $(CFLAGS) $(INCS) -S -o $*.s $<


