#include "checksum.h"

/* Only sums in 16-bit shorts. If count is odd, then the final byte
   will not be summed! */
static unsigned int ones_sum(void *vaddr, int count) {
  unsigned short *addr = vaddr;
  unsigned int sum = 0;
  count >>= 1;
  while (count) {
    sum += *(addr++);
    count--;
  }

  /*  Add left-over byte, if any */
  /*  if( count > 0 )
      sum += * (unsigned char *) addr;*/

  return sum;
}

static unsigned short fold_sum(unsigned int sum) {
   /*  Fold 32-bit sum to 16 bits */
  while (sum>>16)
    sum = (sum & 0xffff) + (sum >> 16);
  
  return (unsigned short)~sum;
}

static unsigned short checksum(void *addr, int count) {
  int sum = ones_sum(addr, count);
  return fold_sum(sum);
}

unsigned short udp_checksum(struct iphdr *ip, struct udphdr *udp) {
  unsigned short udp_len = ntohs(udp->len);
  unsigned int sum = ones_sum(&ip->saddr, 8);
  sum += ntohs(17); // protocol
  sum += udp->len;
  sum += ones_sum(udp, udp_len);
  if (udp_len & 1) {
    // check sum the final byte with padded zeroes
    sum += ((unsigned char*)udp)[udp_len - 1] << 8; 
  }
  sum = fold_sum(sum);
  /* Special case: 0 is not a valid checksum */
  if (sum == 0) {
    sum = 0xFFFF;
  }
  return sum;
}

unsigned short ip_checksum(struct iphdr *ip) {
  return checksum(ip, ip->ihl * 4);
}
