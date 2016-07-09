#include "sw_port.h"
#include <iostream>
#include <stdlib.h>

#define log(f, a...) printf(f, ##a)
#define PACKET_TRACE "PACKET_TRACE"


#ifndef SW_NETWORK

char * FILENAME;
#define PCap_PORT_NUM 0

PcapPort *port = NULL;

char buffer[2000];
char* nf_pktin_poputk(char*);
int packet_number =0;
#else

PktPort rawSocket("tap0","tap1");
#endif

char* sw_pktin_pop()
{
#ifndef SW_NETWORK
  if (port == NULL)
    {
      FILENAME = getenv(PACKET_TRACE);
      if ( FILENAME == NULL )
        {
	  fprintf(stderr, "ERROR: sw_pktin_pop. Please set environment "
		  "variable:\n\t" PACKET_TRACE "\n");
	  exit(-1);
        }
        
      printf("Creating new port with file %s\n", FILENAME);
      port = new PcapPort(PCap_PORT_NUM, FILENAME);
      log("New Port created\n");
	
    }
	
  bool isNewFile;
  int size;
  log("Fetching Packet\n");
  if ((size = port->fetch_pkt(buffer,  isNewFile )) <= 0)
    {
      exit(0);
    }
	
  else
    {
      log("Packet %d Fetched. Returning Buffer.\n", packet_number);
      packet_number++;

      return buffer;
    }
#else
  return(rawSocket.fetch_pkt());
#endif
}



void sw_pktout_send(char* start_addr, char* end_addr) /*** Sends out a file name pcap#.trace (where # is whatever Pcap_PORT_NUM is defined above) to pubsub directory.***/
{
#ifndef SW_NETWORK
  log("IN sw_pktout_send");
  int size = end_addr - start_addr;
  port->send_pkt(start_addr + 8, size );
#else
  rawSocket.send_pkt( start_addr,  end_addr);	
#endif
}


