#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <ctype.h>

#include "args.h"

using namespace std;

Args::Args() :
  use_netfpga(false),

  src_ip(0xa000001),
  dst_ip(0xa001001),
  dst_port(2001),
  src_port(2000),
  payload_len(150),
  fixed_interval(100000000),
  runtime_seconds(5),
  num_pkts(0),

  src_ip_set(false),
  dst_ip_set(false),
  dst_mac_set(false),
  interval_set(false),
  runtime_set(false),
  num_pkts_set(false),
  payload_len_set(false)

  {
    memset(dst_mac, 0, 6);
    dst_mac[5] = 0x1;
  }

static bool parse_num(const char *str, unsigned int *value) {
  *value = 0;
  if (str == 0 || *str == 0) {
    return false;
  }

  char *last_char;
  *value = strtoul(str, &last_char, 0);
  if (*last_char != 0) {
    // didn't parse entire string
    *value = 0;
    return false;
  }
  return true;
}

static bool parse_ip(const char *str, unsigned int *ip) {
  struct in_addr inp;
  if (inet_aton(str, &inp)) {
    *ip = ntohl(inp.s_addr);
    return true;
  }
  return false;
}

static bool parse_mac(const char *str, unsigned char *mac) {
  // verify it's a MAC address
  if (strlen(str) != 17) {
    return false;
  }
  for (int i = 1; i <= 5; i++) {
    if (str[i * 3 - 1] != ':') {
      //cout << "not : " << (i*3) -1 << endl;
      return false;
    }
  }
  for (int i = 0; i < 6; i++) {
    if (!isxdigit(str[i * 3]) || !isxdigit(str[i * 3 + 1])) {
      //cout << "not xdigit: " << (i*3) << " or " << (i*3+1) << endl;
      return false;
    }
  }

  int err = sscanf(str, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", 
                   mac+0, mac+1, mac+2, mac+3, mac+4, mac+5);

  return err == 6;
}

static void usage() {
  cout << "\
Usage: pktgen [-N] [-n num packets] [-t runtime sec]\n\
              [-i destination ip] [-m destination MAC]\n\
              [-I source ip]\n\
              [-d interval ns] [-D interval file ns]\n\
              [-l payload_length] [-L payload_length_file]\n\
where\n\
    -N Sends packets to the NetFPGA (the default is to send\n\
       normal UDP packets through the network stack)"
       << endl;
 }


bool Args::parse(int *argc_in, char **argv) {
  int argc = *argc_in;
  int c;
  opterr = 0;

  while ((c = getopt (argc, argv, "l:L:d:D:t:n:I:i:m:Nh")) != -1)
    switch (c)
      {
      case 'h':
        usage();
        exit(0);
        break;
      case 'N':
        use_netfpga = true;
        break;
      case 'I':
	if (!parse_ip(optarg, &src_ip)) {
	  fprintf(stderr, "Invalid source IP: %s\n", optarg);
	  return false;
	}
	src_ip_set = true;
	break;
      case 'i':
	if (!parse_ip(optarg, &dst_ip)) {
	  fprintf(stderr, "Invalid destination IP: %s\n", optarg);
	  return false;
	}
	dst_ip_set = true;
	break;
      case 'm':
	if (!parse_mac(optarg, dst_mac)) {
	  fprintf(stderr, "Invalid destination MAC addresss: %s\n", optarg);
	  return false;
	}
	dst_mac_set = true;
	break;
      case 'l':
	if (!parse_num(optarg, &payload_len)) {
	  fprintf(stderr, "Invalid payload length: %s\n", optarg);
	  return false;
	}
	payload_len_set = true;
	break;
      case 'L':
	if (payload_file.is_open()) {
	  fprintf(stderr, "Can not set multiple payload length files (-L)\n");
	}
	payload_file.open(optarg);
	if (payload_file.fail()) {
	  fprintf(stderr, "Could not open payload length file: %s\n", optarg);
	  return false;
	}
	break;
      case 'd':
	if (!parse_num(optarg, &fixed_interval)) {
	  fprintf(stderr, "Invalid delay amount: %s\n", optarg);
	  return false;
	}
	interval_set = true;
	break;
      case 'D':
	if (interval_file.is_open()) {
	  fprintf(stderr, "Can not set multiple interval files (-D)\n");
	}
	interval_file.open(optarg);
	if (interval_file.fail()) {
	  fprintf(stderr, "Could not open interval file: %s\n", optarg);
	  return false;
	}
	break;
      case 't':
	if (!parse_num(optarg, &runtime_seconds)) {
	  fprintf(stderr, "Invalid runtime: %s\n", optarg);
	  return false;
	}
	runtime_set = true;
	break;
      case 'n':
	if (!parse_num(optarg, &num_pkts)) {
	  fprintf(stderr, "Invalid number of packets: %s\n", optarg);
	  return false;
	}
	num_pkts_set = true;
	break;
      case '?':
	if (isprint (optopt))
	  fprintf (stderr, "Unknown option `-%c'.\n", optopt);
	else
	  fprintf (stderr,
		   "Unknown option character `\\x%x'.\n",
		   optopt);
	return false;
      default:
	abort ();
      }

  if (interval_file.is_open() && interval_set) {
    fprintf (stderr, "Cannot set both fixed delay (-d) and delay file (-D).\n");
    return false;
  }

  if (payload_file.is_open() && payload_len_set) {
    fprintf (stderr, "Cannot set both fixed payload length (-l) and length file (-L).\n");
    return false;
  }

  if (num_pkts_set && runtime_set) {
    fprintf (stderr, "Cannot set both num pkts (-n) and runtime (-t).\n");
    return false;
  }

  if (interval_file.is_open() && runtime_set) {
    fprintf (stderr, "When reading intervals from a file (-D) cannot set runtime (-t). Instead set the number of packets (-n).\n");
    return false;
  }

  if (!use_netfpga && dst_mac_set) {
    fprintf (stderr, "Can only set destination mac (-m) when sending through the NetFPGA (-N).\n");
    return false;
  }

  if (!use_netfpga && src_ip_set) {
    fprintf (stderr, "Can only set source IP (-I) when sending through the NetFPGA (-N).\n");
    return false;
  }

  if (optind < argc) {
    fprintf (stderr, "Unknown non-option argument.\n");
  }

  *argc_in = optind;




  

  return true;
}
