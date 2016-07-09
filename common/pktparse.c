
#ifdef __KERNEL__

#include <net/udp.h>
#include <net/ip.h>
#include <linux/if_ether.h>
#include <asm/string.h>

#else

#include <net/if.h>
#include <net/ethernet.h>
#include <arpa/inet.h>
#include <linux/if_ether.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <netinet/udp.h>
#include <string.h>

#endif

#include "pktparse.h"
#include "checksum.h"


#define pull_hdr(name, type, data, len)          \
  do {                                           \
    if (len < sizeof(type)) {                    \
      name = 0;                                  \
    } else {                                     \
      name = (type*) data;                       \
      data += sizeof(type);                      \
      len -= sizeof(type);                       \
    }                                            \
  } while (0)

static unsigned short ip_id = 1;

int build_packet(struct pkt_descr *pdesc, char *buf, int len) {
  char *data = buf;
  int size;
  struct ethhdr *eth;
  struct iphdr *ip;
  struct udphdr *udp;

  // Convert packet description to an actual packet
  pull_hdr(eth, struct ethhdr, data, len);
  pull_hdr(ip, struct iphdr, data, len);
  pull_hdr(udp, struct udphdr, data, len);

  if (udp == 0) {
    /* buf wasn't large enough for the headers */
    return -1;
  }
  size = (data - buf) + pdesc->payload_len;

  // fill the headers
  udp->source = htons(pdesc->src_port);
  udp->dest = htons(pdesc->dst_port);
  udp->len = htons(sizeof(struct udphdr) + pdesc->payload_len);
  udp->check = 0;

  ip->ihl = 5; // no tcp options
  ip->version = 4;
  ip->tos = 0;
  ip->tot_len = htons(size - sizeof(struct ethhdr));
  ip->id = htons(ip_id++); // each packet gets unique ip id
  ip->frag_off = 0;
  ip->ttl = 64;
  ip->protocol = IPPROTO_UDP;
  ip->check = 0;
  ip->saddr = htonl(pdesc->src_ip);
  ip->daddr = htonl(pdesc->dst_ip);

  /* Calculate checksums */
  ip->check = ip_checksum(ip);
  udp->check = 0; //udp_checksum(ip, udp);
  
  eth->h_proto = htons(ETH_P_IP);
  memcpy(eth->h_dest, pdesc->dst_mac, 6);
  memcpy(eth->h_source, pdesc->src_mac, 6);

  return (data - buf);
}
