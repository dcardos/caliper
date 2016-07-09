#ifndef PKTDESCR_H
#define PKTDESCR_H

//#include <sys/time.h>

struct pkt_descr {
  unsigned int src_ip;
  unsigned int dst_ip;
  unsigned short src_port;
  unsigned short dst_port;
  unsigned char src_mac[6];
  unsigned char dst_mac[6];
  unsigned long long departure_time;
  int is_absolute;
  int payload_len;
  int custom_data_len; /* the number of bytes that follow this struct
			  that should be included in the packet
			  payload */
};

#endif
