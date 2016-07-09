#include <iostream>
#include <iomanip>
#include <fstream>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "args.h"
#include "send_nl.h"
#include "send.h"

using namespace std;


static Args args;

char* ip(unsigned int i) {
  struct in_addr addr = { htonl(i) };
  return inet_ntoa(addr);
}

unsigned long long get_now_ns() {
  struct timeval tv;
  gettimeofday(&tv, 0);

  return (unsigned long long)tv.tv_sec * 1000000000 + tv.tv_usec * 1000;
}






unsigned int fixed_length() {
  return args.payload_len;
}

unsigned int file_length() {
  unsigned int length;
  ifstream &payload_file = args.payload_file;

  payload_file >> length;

  if (payload_file.eof()) {
    // start reading from the beginning again
    payload_file.clear();
    payload_file.seekg(0);
    payload_file >> length;
  } else if (!payload_file.good()) {
    fprintf(stderr, "Error reading payload length file\n");
    exit(1);
  }
  return length;
}



unsigned long long file_departure() {
  unsigned long long result;
  ifstream &interval_file = args.interval_file;

  interval_file >> result;

  if (interval_file.eof()) {
    // start reading from the beginning again
    interval_file.clear();
    interval_file.seekg(0);
    interval_file >> result;
  } else if (!interval_file.good()) {
    fprintf(stderr, "Error reading interval file\n");
    exit(1);
  }
  return result;
}

unsigned long long fixed_departure() {
  return args.fixed_interval;
}

int main(int argc, char **argv) {
  if (!args.parse(&argc, argv)) {
    return 1;
  }

  if (args.runtime_set || !args.num_pkts_set) {
    unsigned long long total_time_ns = (unsigned long long) args.runtime_seconds * 1000000000LL;
    args.num_pkts = total_time_ns / args.fixed_interval;
  }

  args.delays = fixed_departure;
  if (args.interval_file.is_open()) {
    args.delays = file_departure;
  }

  args.lengths = fixed_length;
  if (args.payload_file.is_open()) {
    args.lengths = file_length;
  }

  cout << "Sending " << args.num_pkts << " packets" << endl;
  cout << "Destination IP: " << ip(args.dst_ip) << endl;
  if (args.use_netfpga) {
    cout << "Destination MAC: ";
    for (int i = 0; i < 6; i++) {
      cout << hex << setfill('0') << setw(2) << (unsigned short)args.dst_mac[i] << ":";
    }
    cout << dec << endl;
  }
  //cout << "Delay " << args.fixed_interval << " payload length " << args.payload_len << " seconds " << args.runtime_seconds << endl;
  
  if (args.use_netfpga) {
    cout << "Sending to NetFPGA" << endl;

    Send_NL nl(args);
    if (!nl.init()) {
      cout << "Error initing Send_NL" << endl;
      return 1;
    }
    nl.go();

  } else {
    cout << "Sending to network stack" << endl;
    Send send(args);
    if (!send.init()) {
      cout << "Error initing Send" << endl;
      return 1;
    }
    send.go();

  }

  return 0;
}
