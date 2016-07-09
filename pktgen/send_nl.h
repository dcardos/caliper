#ifndef SEND_NL_H
#define SEND_NL_H

#include "netlink.h"
#include "nf_netlink.h"
#include "args.h"


/*
  Send packets over netlink to the NetFPGA driver and packet
  generator
*/


class Send_NL {

 public:
  Send_NL(Args &args);
  bool init();
  void go();

 private:
  unsigned long long send_sim_start();
  int send_burst(NetLinkMsg &pkt, struct pkt_descr *pdesc, struct send_state &state, unsigned int burst_size, delay_source delays, length_source lengths, unsigned int num_pkts);
  int mark_pkts();
  bool wait_for_mark(unsigned int seq);

  Args &args;
  NetLink nl;

  NetLinkMsg mark_msg;
  int mark_seq;
};




#endif
