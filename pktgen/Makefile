# DRIVER_ADDR specifies the netlink address to send packets to
# 2 is the fakedriver. This is the default.
# 0 is the real driver. Override this on the cmd line with 'make DRIVER_ADDR=0'
DRIVER_ADDR?=0

CFLAGS := -O3 -Wall -DNETLINK_DRIVER_ADDR=$(DRIVER_ADDR) 
LDFLAGS :=
INCS := -I../common

OBJS := netlink.o pktgen.o args.o send_nl.o send.o
pktgen: $(OBJS)
	g++ $(LDFLAGS) $^ -o $@

.PHONY: clean
clean:
	rm -f pktgen $(OBJS) $(OBJS:.o=.d)

%.o: %.cc
	g++ $(CFLAGS) $(INCS) -c $<

%.d: %.cc
	@set -e; rm -f $@; \
	g++ -MM $(CFLAGS) $(INCS) $< > $@.$$$$; \
	sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' < $@.$$$$ > $@; \
	rm -f $@.$$$$

include $(OBJS:.o=.d)
