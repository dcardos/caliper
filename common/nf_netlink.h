#ifndef NF_NETLINK_H
#define NF_NETLINK_H

#define NETLINK_NF_PKTGEN 17

#include <linux/netlink.h>

#include "pktdescr.h"

#ifdef __KERNEL__
#include <linux/time.h>
#else
#include <time.h>
#endif


/* Message types */

#define NF2_NL_DELAYS NLMSG_MIN_TYPE 
#define NF2_NL_PKT NLMSG_MIN_TYPE + 1
#define NF2_NL_SIM_START NLMSG_MIN_TYPE + 2

/* Send to the driver to add a special token in the outgoing
   packets. Once all the packets that were sent prior to the mark (and
   probably a few more) have been sent to the NetFPGA, the driver will
   send a NFMarkHitMsg back to the process that set the mark with the
   same sequence number. */
#define NF2_NL_MARK NLMSG_MIN_TYPE + 3
#define NF2_NL_MARK_HIT NLMSG_MIN_TYPE + 4


struct NFDelayMsg {
  int type;
  /* Variable length data follows here */
};

struct NFDelayResultMsg {
  int type;
  int data;
};

struct NFSimStartMsg {
  struct timespec start;
  /*  unsigned int start_sec;
      unsigned int start_nsec;*/
};

#endif
