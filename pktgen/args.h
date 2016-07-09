#ifndef ARGS_H
#define ARGS_H

#include <iostream>
#include <fstream>

#include "common.h"

class Args {

 public:
  Args();

  bool parse(int *argc, char **argv);

  bool use_netfpga;

  unsigned int src_ip;
  unsigned int dst_ip;
  unsigned char dst_mac[6];
  unsigned short dst_port;
  unsigned short src_port;
  unsigned int payload_len;
  unsigned int fixed_interval;
  unsigned int runtime_seconds;
  unsigned int num_pkts;

  bool src_ip_set;
  bool dst_ip_set;
  bool dst_mac_set;
  bool interval_set;
  bool runtime_set;
  bool num_pkts_set;
  bool payload_len_set;

  std::ifstream payload_file;
  std::ifstream interval_file;

  delay_source delays;
  length_source lengths;
};



#endif
