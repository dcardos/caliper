#ifndef DRIVERCOM_H
#define DRIVERCOM_H

/*
Defines all of the messages that are sent between the driver and
NetThreads packet generator, in both directions.
 */

#define D2N_TYPE_RESET 33
#define D2N_TYPE_PKTS 42

struct driver_to_netfpga {
  int type;  
  /* All driver_to_netfpga messages need to be longer than this
     because the hardware pipeline can't handle packets smaller than 9
     bytes (I think?). */
};

struct packet_hdr {
  unsigned short len;
  unsigned short wire_len;
  unsigned int clock_hi;
  unsigned int clock_lo;
  short is_absolute;
  unsigned short next_pkt;
  /* Ensure this struct's size is a multiple of 4 bytes */
};

#define N2D_TYPE_RESET 88
#define N2D_TYPE_QUOTA 100

struct netfpga_to_driver {
  int type;
  int pkt_quota;
  unsigned int time_hi;
  unsigned int time_lo;
  int filler[8];
};


#endif
