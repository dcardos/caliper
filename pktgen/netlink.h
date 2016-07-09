#ifndef NETLINK_H
#define NETLINK_H

extern "C" {
#include <asm/types.h>
#include <sys/socket.h>
#include <linux/netlink.h>
}

class NetLinkMsg {
 public:
  NetLinkMsg(int payloadLength);
  ~NetLinkMsg();

  nlmsghdr *header();
  void *payload();

  int length();
  void setLength(int length);
  int capacity();
 private:
  nlmsghdr *m_msg;
  int m_length;
  int m_capacity;
};


class NetLink {

 public:
  NetLink(int address);
  ~NetLink();
  bool init();
  
  bool send(NetLinkMsg &msg, int destAddr);
  bool send_custom_hdr(NetLinkMsg &msg, int destAddr);
  int receive(NetLinkMsg &nlm, long timeout_sec, long timeout_usec);

 private:
  int m_address;
  int m_nlfd;
  int rawfd;
};



#endif
