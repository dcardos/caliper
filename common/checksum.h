#ifndef CHECKSUM_H
#define CHECKSUM_H

#ifdef __KERNEL__
#include <linux/ip.h>
#include <linux/udp.h>
#else
#include <netinet/ip.h>
#include <netinet/udp.h>
#endif

unsigned short udp_checksum(struct iphdr *ip, struct udphdr *udp);
unsigned short ip_checksum(struct iphdr *ip);

#endif
