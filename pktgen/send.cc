#include <string.h>
#include <sys/time.h>
#include <time.h>

#include <iostream>
#include <iomanip>

//#include "cycle.h"
#include "send.h"       

using namespace std;

Send::Send(Args &args) 
  : args(args)
{
}


bool Send::init() {
  if ((sk = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
    perror("socket");
    return false;
  }
  
  memset(&si, 0, sizeof(si));
  si.sin_family = AF_INET;
  si.sin_port = htons(args.dst_port);
  si.sin_addr.s_addr = htonl(args.dst_ip);

  return true;
  
}

#define BUFLEN 1500

static inline void adddelay(struct timeval *tv, unsigned long long nanos) {
  unsigned int sec = (unsigned int) (nanos / 1000000000);
  unsigned int usec = (unsigned int) ((nanos % 1000000000) / 1000);

  tv->tv_sec += sec;
  tv->tv_usec += usec;

  // check for overflow
  if (tv->tv_usec > 1000000) {
    tv->tv_sec++;
    tv->tv_usec -= 1000000;
  }
}

void Send::go() {
  int num = args.num_pkts;
  int slen = sizeof(si);

  char buf[BUFLEN];
  
  length_source lengths = args.lengths;
  delay_source delays = args.delays;

  // first packet is delayed a fixed amount
  unsigned long long delay = 250000000;

  struct timeval to_send;
  struct timeval sent;
  gettimeofday(&sent, 0);

  for (int i = 0; i < num; i++) {
    to_send = sent;
    adddelay(&to_send, delay);
    //cout << "Will send at: " << to_send.tv_sec << setw(6) << setfill('0') << to_send.tv_usec << endl;
    unsigned int length = lengths();
    /* Simply busy wait */
    do {
      gettimeofday(&sent, 0);
    } while (sent.tv_sec < to_send.tv_sec ||
	     (sent.tv_sec == to_send.tv_sec && sent.tv_usec < to_send.tv_usec));
    if (sendto(sk, buf, length, 0, (sockaddr*)&si, slen) == -1) {
      perror("sendto");
      goto done;
    }
    
    delay = delays();

    
  }

 done:
  close(sk);
}
