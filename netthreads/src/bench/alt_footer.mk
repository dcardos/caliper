OBJS=${SRC:.c=.o}

# START must be linked first Since RTL model blindly executes from 400000

########################################################
ifdef MULTIPROC 
MASTER_START=$(MULTI_TOOLS)/start_master.o
SLAVE_START=$(MULTI_TOOLS)/start_slave.o
$(TARGET) $(TARGET)_slave: $(OBJS) $(FUNC_LIB) $(MASTER_START) $(SLAVE_START) $(MATH) $(FLOATING) $(EXTRA_LIB)
	$(SPE_LD) $(MASTER_START) $(OBJS) $(LDFLAGS) $(EXTRA_LIB) -o $(TARGET)
	$(SPE_LD) $(SLAVE_START) $(OBJS) $(LDFLAGS) $(EXTRA_LIB) -o $(TARGET)_slave

#############################################################
else 
ifndef MULTITHREAD
$(TARGET): $(OBJS) $(FUNC_LIB) $(START) $(MATH) $(FLOATING) $(EXTRA_LIB) $(ULIBS)
	$(SPE_LD) $(START) $(OBJS) $(ULIBS) $(LDFLAGS) $(EXTRA_LIB) -o $(TARGET) > $(TARGET).map
	egrep "^ *0x0000000004" $(TARGET).map |grep -v PROVIDE > $(TARGET).static

#############################################################
else

include ../mt.mk

$(TARGET)_t1: $(OBJS) $(FUNC_LIB) $(START) $(MATH) $(FLOATING) $(EXTRA_LIB)
	make -C $(SPE_HOME)/applications/multi_tools rebuild
	echo -e "_es = $(ST1);\n_gp = $(GP1);\n_sp = $(SP1);\n" > $(LSI)
	cat ../linker_mt.ld >> $(LSI)
	$(SPE_LD) $(LDFLAGS_T1) $(OBJS) $(EXTRA_LIB) -o $(TARGET)_t1.t
	$(SPE_HOME)/applications/multi_tools/recompile_loader.sh $(TARGET)_t1.t 2000 $(DATA_SHIFTING)
	$(SPE_LD) $(LDFLAGS_T1) $(OBJS) $(EXTRA_LIB) -o $(TARGET)_t1.t

$(TARGET)_t2: $(OBJS) $(FUNC_LIB) $(START) $(MATH) $(FLOATING) $(EXTRA_LIB)
	echo -e "_es = $(ST2);\n_gp = $(GP2);\n_sp = $(SP2);\n" > $(LSI)
	cat ../linker_mt.ld >> $(LSI)
	$(SPE_LD) $(LDFLAGS_T2) $(OBJS) $(EXTRA_LIB) -o $(TARGET)_t2.t 
$(TARGET)_t3: $(OBJS) $(FUNC_LIB) $(START) $(MATH) $(FLOATING) $(EXTRA_LIB)
	echo -e "_es = $(ST3);\n_gp = $(GP3);\n_sp = $(SP3);\n" > $(LSI)
	cat ../linker_mt.ld >> $(LSI)
	$(SPE_LD) $(LDFLAGS_T3) $(OBJS)  $(EXTRA_LIB) -o $(TARGET)_t3.t
$(TARGET)_t4: $(OBJS) $(FUNC_LIB) $(START) $(MATH) $(FLOATING) $(EXTRA_LIB)
	echo -e "_es = $(ST4);\n_gp = $(GP4);\n_sp = $(SP4);\n" > $(LSI)
	cat ../linker_mt.ld >> $(LSI)
	$(SPE_LD) $(LDFLAGS_T4) $(OBJS) $(EXTRA_LIB) -o $(TARGET)_t4.t
$(TARGET)_t5: $(OBJS) $(FUNC_LIB) $(START) $(MATH) $(FLOATING) $(EXTRA_LIB)
	echo -e "_es = $(ST5);\n_gp = $(GP5);\n_sp = $(SP5);\n" > $(LSI)
	cat ../linker_mt.ld >> $(LSI)
	$(SPE_LD) $(LDFLAGS_T5) $(OBJS) $(EXTRA_LIB) -o $(TARGET)_t5.t
$(TARGET)_t6: $(OBJS) $(FUNC_LIB) $(START) $(MATH) $(FLOATING) $(EXTRA_LIB)
	echo -e "_es = $(ST6);\n_gp = $(GP6);\n_sp = $(SP6);\n" > $(LSI)
	cat ../linker_mt.ld >> $(LSI)
	$(SPE_LD) $(LDFLAGS_T6) $(OBJS) $(EXTRA_LIB) -o $(TARGET)_t6.t
$(TARGET)_t7: $(OBJS) $(FUNC_LIB) $(START) $(MATH) $(FLOATING) $(EXTRA_LIB)
	echo -e "_es = $(ST7);\n_gp = $(GP7);\n_sp = $(SP7);\n" > $(LSI)
	cat ../linker_mt.ld >> $(LSI)
	$(SPE_LD) $(LDFLAGS_T7) $(OBJS) $(EXTRA_LIB) -o $(TARGET)_t7.t
$(TARGET)_t8: $(OBJS) $(FUNC_LIB) $(START) $(MATH) $(FLOATING) $(EXTRA_LIB)
	echo -e "_es = $(ST8);\n_gp = $(GP8);\n_sp = $(SP8);\n" > $(LSI)
	cat ../linker_mt.ld >> $(LSI)
	$(SPE_LD) $(LDFLAGS_T8) $(OBJS) $(EXTRA_LIB) -o $(TARGET)_t8.t

endif
endif
#############################################################

clean:
	rm -f $(TARGET) $(OBJS) $(TARGET).s *.S debug_sched* $(START)

dis:
	$(SPE_DIS) -D $(TARGET)
	$(SPE_DIS) -h $(TARGET)

$(FUNC_LIB):
	make -C $(TOOLS)

$(START):
	make -C $(TOOLS)

$(MATH):
	make -C $(TOOLS)

$(FLOATING):
	make -C $(TOOLS)

OBJDUMP=$(SPE_DIS)
OBJCOPY=$(SPE_PREFIX)-objcopy
RAW2HEX=$(TOOLS)/raw2hex
RAW2MIF=$(TOOLS)/raw2mif

check:
	-$(SPE_DIS) -d $(TARGET) | grep -f $(TOOLS)/pat.txt

embed:  rif # $(TARGET).read

######################## Rif creation (Quartus)  ##########################
ifndef MULTITHREAD
rif: instr.rif data.rif 
else
rif: instr.rif data.rif ddr.rif
endif
# rif: instr_slave.rif instr.rif data.rif

ifndef MULTITHREAD
instr.rif: $(TARGET).instr.mif
	mif2rif $< $@ >&/dev/null; [ -s $@ ]


data.rif: $(TARGET).data.mif
	mif2rif $< $@ >&/dev/null; [ -s $@ ]
else

RI=$(addsuffix .instr.rif,$(TGT)) 
RD=$(addsuffix .data.rif,$(TGT))

ifndef SINGLE_PROGRAM
instr.rif: $(RI)
	$(MULTI_TOOLS)/mergerif $(RI) i $(PIPE) $(PIPE)
	mv out.rif $@
else
instr.rif: $(RI)
	cp $(TARGET)_t1.instr.rif $@
endif

data.rif: $(RD)
ifndef DATA_SHIFTING
	$(MULTI_TOOLS)/mergerif $(RD) d $(PIPE) $(PIPE)
else
	$(MULTI_TOOLS)/mergerif $(RD) ds$(DATA_SHIFTING) $(PIPE) $(PIPE)
endif
	mv out.rif $@

ddr.rif:
	$(MULTI_TOOLS)/mergerif instr.rif data.rif ia $(PIPE) $(PIPE)
	mv out.rif ddr32.rif
	$(MULTI_TOOLS)/rif32to64 ddr32.rif
	mv out.rif ddr64.rif
endif

#instr_slave.rif: $(TARGET)_slave
#	$(OBJDUMP) --full-content $(TARGET)_slave > $(TARGET)_slave.raw
#	$(RAW2MIF) -width=32 -depth=65536 $(TARGET)_slave.raw 
#	mif2rif $(TARGET)_slave.instr.mif instr_slave.rif >&/dev/null; [ -s instr_slave.rif ]

%.instr.rif: %.instr.mif
	mif2rif $< $@ >&/dev/null; [ -s $@ ]

%.data.rif: %.data.mif
	mif2rif $< $@ >&/dev/null; [ -s $@ ]

######################## Mif creation  ##########################

%.instr.mif %.data.mif: %.raw
	$(RAW2MIF) -width=32 -depth=65536 $*.raw

######################## Srec creation (GCC)  ##########################
srec: $(TARGET).srec
$(TARGET).srec: $(TARGET)
	$(OBJCOPY) --output-target=srec $(TARGET) $(TARGET).srec

######################## For cross compiled GCC ##########################

$(TARGET).raw: $(TARGET)
	$(OBJDUMP) --full-content $(TARGET) > $(TARGET).raw

%.raw: %.t
	$(OBJDUMP) --full-content $< > $@

$(TARGET).read: 
	$(OBJDUMP) -D $(TARGET) > $(TARGET).read


#$(TARGET): $(START)
#$(LD) $(LDFLAGS) $(OBJS) -o $(TARGET)
#
#$(START):
#$(AS) $(ASFLAGS) $(STARTSRC) -o $(START)


embed_clean:
	rm -f $(TARGET).raw $(TARGET).read $(TARGET).srec $(TARGET).instr.mif $(TARGET).data.mif  instr.rif data.rif *.t *.rif *.mif *.raw debug_sched*.txt 

redo: clean all embed

