#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>


#include "send_nl.h"


using namespace std;

Send_NL::Send_NL(Args &args) 
  : args(args), nl(getpid()), mark_msg(4096), mark_seq(0)
{}

#define BURST_SIZE  100

bool Send_NL::init() {
  return nl.init();
}

struct send_state {

  unsigned int pkts_sent;
  unsigned long long final_tx_ns;


  send_state() : pkts_sent(0), final_tx_ns(0) 
  {}
};

void Send_NL::go() {

    /*
    The goal here is to keep the NetFPGA card busy. It should always
    have a packet that it's waiting to send. To do this, the driver
    should also always have packets to send, but we cannot constantly
    send packets without filling all of the TX buffers in the
    driver. Instead, aim to keep between BURST_SIZE and 2*BURST_SIZE
    worth of packets in the system waiting to transmit at all times.

    1. Send inital burst of packets.
    2. Send a burst of packets
    3. Wait long enough for 1 burst to be sent
    4. Goto step 2

  */

  NetLinkMsg pkt_msg(sizeof(struct pkt_descr));
  pkt_msg.header()->nlmsg_type = NF2_NL_PKT;

  struct pkt_descr *pdesc = (struct pkt_descr*) pkt_msg.payload();
  pdesc->payload_len = args.payload_len;
  pdesc->src_ip = args.src_ip;
  pdesc->src_port = args.src_port;
  pdesc->dst_ip = args.dst_ip;
  pdesc->dst_port = args.dst_port;

  memcpy(pdesc->dst_mac, args.dst_mac, 6);
  memset(pdesc->src_mac, 0, 6);

  /* first packet is absolute, the rest are relative */
  pdesc->is_absolute = 1;
  pdesc->custom_data_len = 0;

  /*uint *payload = (uint*) (pdesc + 1);
  payload[0] = htonl(0x4444feef);
  payload[1] = htonl(0x1234abcd);*/

  unsigned long long simtime_start = send_sim_start();
  
  cout << "Will send " << args.num_pkts << " packets." << endl;
  //cout << "sim time now " << ( get_now_ns() - simtime_start) << "us" << endl;

  struct send_state state;
  // Start sending packets 1/2 sec from now
  state.final_tx_ns = (get_now_ns() - simtime_start) + 1000000000LL;
  cout << "first pkt sim tx time " << state.final_tx_ns << "ns" << endl;


  if (send_burst(pkt_msg, pdesc, state, BURST_SIZE, args.delays, args.lengths, args.num_pkts) == 1) {
    cout << "Complete during first burst of " << state.pkts_sent << " packets" << endl;
  } else {
    while (1) {
      int seq = mark_pkts();
      //unsigned long long after_prev_burst = state.final_tx_ns;
      if (send_burst(pkt_msg, pdesc, state, BURST_SIZE, args.delays, args.lengths, args.num_pkts) == 1) {
	cout << "Complete. sent " << state.pkts_sent << " packets" << endl;
	break;
      }

      wait_for_mark(seq);
    
      /*
      // Wait until the burst before the previous one is done sending
      // Calculate what the current simulation time is
      unsigned long long simtime_now = get_now_ns() - simtime_start;

      if (simtime_now < after_prev_burst) {
	// How long do we need to sleep until the previous burst is sent?
	unsigned long long sleep_time = after_prev_burst - simtime_now;
	// convert nanos to mircoseconds
	sleep_time /= 1000;

	if (sleep_time > 5) {
	  // subtract fudge factor because sleeping isn't accurate
	  sleep_time -= 5;
	  //cout << "sleep time " << sleep_time << "us\n";
	
	  usleep(sleep_time);
	}
	}*/
    }
  }

  // Wait until all packets should be sent
  // Calculate what the current simulation time is
  unsigned long long simtime_now = get_now_ns() - simtime_start;

  if (simtime_now < state.final_tx_ns) {
    // How long do we need to sleep until the previous burst is sent?
    unsigned long long sleep_time = state.final_tx_ns - simtime_now;
    // convert nanos to mircoseconds
    sleep_time /= 1000;
    
    cout << "final sleep time " << sleep_time << "us\n";
    usleep(sleep_time);
  }
}

unsigned long long Send_NL::send_sim_start() {
  NetLinkMsg msg(sizeof(struct NFSimStartMsg));
  msg.header()->nlmsg_type = NF2_NL_SIM_START;

  //  cout << "sec size " << sizeof(tv.tv_sec) << " usec size " << sizeof(tv.tv_usec) << endl;
  
  struct NFSimStartMsg *sim_start = (struct NFSimStartMsg*) msg.payload();


  struct timeval tv;
  gettimeofday(&tv, 0);
  sim_start->start.tv_sec = tv.tv_sec;
  sim_start->start.tv_nsec = tv.tv_usec * 1000;

  nl.send(msg, NETLINK_DRIVER_ADDR);

  unsigned long long ns = tv.tv_sec;
  ns = ns * 1000000000 + tv.tv_usec * 1000;

  return ns;
}

int burst_id = 1;

int Send_NL::send_burst(NetLinkMsg &pkt, struct pkt_descr *pdesc, struct send_state &state, unsigned int burst_size, delay_source delays, length_source lengths, unsigned int num_pkts) {
  if (state.pkts_sent >= num_pkts) {
    return 1;
  }

  int pkts = std::min(num_pkts - state.pkts_sent, burst_size);
  if (pkts <= 0) {
    return 1;
  }
  
  unsigned long long tx_ns = state.final_tx_ns;
  for (int i = 0; i < pkts; i++) {
    pdesc->payload_len = lengths();
    if (pdesc->is_absolute) {
      pdesc->departure_time = tx_ns;
      cout << "first departure_time " << pdesc->departure_time << endl;
    } else {
      pdesc->departure_time = delays();
      //cout << "sample departure_time " << pdesc->departure_time << endl;
    }
    tx_ns += pdesc->departure_time;
    nl.send(pkt, NETLINK_DRIVER_ADDR);

    /* all but the first packet use relative timing */
    pdesc->is_absolute = 0;
  }

  //printf("Sent burst %d\n", burst_id++);
  
  state.pkts_sent += pkts;
  state.final_tx_ns = tx_ns;

  return state.pkts_sent == num_pkts;
}



int Send_NL::mark_pkts() {
  mark_msg.setLength(0);

  mark_seq++;
  struct nlmsghdr *nh = mark_msg.header();
  nh->nlmsg_type = NF2_NL_MARK;
  nh->nlmsg_seq = mark_seq;
  nh->nlmsg_flags = 0;

  if (nl.send_custom_hdr(mark_msg, NETLINK_DRIVER_ADDR)) {
    return mark_seq;
  } else {
    return -1;
  }
}

bool Send_NL::wait_for_mark(unsigned int seq) {
  int err = nl.receive(mark_msg, 0, 0);

  if (err == -1) {
    cout << "ERROR RECEIVING" << endl;
    return false;
  }

  unsigned int len = (unsigned int) err;

  //cout << "Got NL msg of length " << len << endl;

  for (struct nlmsghdr *nh = mark_msg.header(); NLMSG_OK (nh, len);
       nh = NLMSG_NEXT (nh, len)) 
  {
    /* The end of multipart message. */
    if (nh->nlmsg_type == NLMSG_DONE) {
      //cout << "DONE\n";
      break;
    }

    if (nh->nlmsg_type == NLMSG_ERROR) {
      /* Do some error handling. */
      cout << "ERROR\n";
    }

    /* Continue with parsing payload. */
    //cout << "msg type " << nh->nlmsg_type << " seq " << nh->nlmsg_seq << "\n";

    if (nh->nlmsg_type == NF2_NL_MARK_HIT) {
      if (nh->nlmsg_seq == seq) {
	return true;
      } else {
	//cout << "Received wrong mark seq. Expected " << seq << " got " << nh->nlmsg_seq << endl;
      }
    }
  }
  //cout << endl;

  return false;
}
