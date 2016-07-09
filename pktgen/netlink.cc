
#include "netlink.h"

#include <stdio.h>
#include <cstring>
#include <unistd.h>

#include <iostream>

#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <fcntl.h>
#include <errno.h>

#include "nf_netlink.h"

#define MY_DEBUG 0

NetLink::NetLink(int address)
  : m_address(address), m_nlfd(-1)
{
  //printf("NetLink constructor\n");
}

NetLink::~NetLink() {
  if (m_nlfd != -1) {
    if (close(m_nlfd) != 0) {
      perror("close socket");
    }
  }
}

bool NetLink::init() {

  /* Ben's modification BEGIN */
  //Initializing a raw socket to read a packet from eth1
  rawfd = socket(PF_INET, SOCK_DGRAM, 0);

  //printf("netlink.cc: init\n");

  m_nlfd = socket(PF_NETLINK, SOCK_RAW, NETLINK_NF_PKTGEN);
  if (m_nlfd == -1) {
    perror("socket");
    return false;
  }

  // Set non-blocking to deal with spurious events that may wake up
  // select()
  if (fcntl(m_nlfd, F_SETFL, O_NONBLOCK) == -1) {
      perror("fcntl() set O_NONBLOCK failed");
      close(m_nlfd);
      return false;
  }

  // Bind to the netlink address
  struct sockaddr_nl sa;
  sa.nl_family = AF_NETLINK;
  sa.nl_pid = m_address;
  sa.nl_groups = 0;

  if (bind(m_nlfd, (struct sockaddr*)&sa, sizeof(sa)) == -1) {
    perror("bind");
    close(m_nlfd);
    m_nlfd = -1;
    return false;
  }

  return true;
}

unsigned int sequence_number = 0;

bool NetLink::send(NetLinkMsg &msg, int destAddr) {

  if(MY_DEBUG){
    printf("[netlink.cc] NetLink::send:  destAddr is %d\n", destAddr);
  }

  struct nlmsghdr *nh = msg.header();
  nh->nlmsg_seq = ++sequence_number;
  nh->nlmsg_flags = 0;

  return send_custom_hdr(msg, destAddr);
}

bool NetLink::send_custom_hdr(NetLinkMsg &msg, int destAddr) {
  if (m_nlfd == -1) {
    return false;
  }

  struct nlmsghdr *nh = msg.header();
  nh->nlmsg_pid = m_address;

  struct iovec iov = { (void *) nh, nh->nlmsg_len };

  struct sockaddr_nl sa;
  memset(&sa, 0, sizeof(sa));
  sa.nl_family = AF_NETLINK;
  sa.nl_pid = destAddr;

  struct msghdr msghdr;
  memset(&msghdr, 0, sizeof(msghdr));
  msghdr.msg_name = (void*) &sa;
  msghdr.msg_namelen = sizeof(sa);
  msghdr.msg_iov = &iov;
  msghdr.msg_iovlen = 1;

  if (sendmsg(m_nlfd, &msghdr, 0) == -1) {
    perror("sendmsg");
    return false;
  } else {
    return true;
  }
}

/*
  Receive a message.

  This method blocks until either one message is received, the timeout
  is reached, or some other event occurs. Note: the units of
  timeout_usec are microseconds. If both timeout_sec and timeout_usec
  are 0 then no timeout is used.

  If a packet is received it is written into the NetLinkMsg nlm which
  is passed in. If the NetLinkMsg isn't large enough the message will
  be truncated.

  Returns: -1 if an error occurred.
           0 if a timeout occurs, or some other strange thing happened.
           >= 1  if a message was received and data was written into nlm.
                 The number returned is the amount of data written, but this
                 value is also set as the length property of the NetLinkMsg.

 */
int NetLink::receive(NetLinkMsg &nlm, long timeout_sec, long timeout_usec) {
  if (m_nlfd == -1) {
    return -1;
  }

  // validate timeout
  if (timeout_sec < 0 || timeout_usec < 0 || timeout_usec >= 1000000) {
    printf("NetLink::receive: Invalid timeout sec=%ld usec=%ld\n", timeout_sec, timeout_usec);
    return -1;
  }
  nlm.setLength(0);

  struct iovec iov = { (void *) nlm.header(), nlm.capacity() };
  struct sockaddr_nl sa;
  struct msghdr msg = { (void *)&sa, sizeof(sa), &iov, 1, NULL, 0, 0 };

  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(m_nlfd, &rfds);

  struct timeval tv;
  tv.tv_sec = timeout_sec;
  tv.tv_usec = timeout_usec;

  struct timeval *timeout = &tv;
  if (timeout_sec == 0 && timeout_usec == 0) {
    timeout = NULL;
  }

  int result = select(m_nlfd + 1, &rfds, NULL, NULL, timeout);
  if (result == -1) {
    if (errno == EINTR) {
      // process received a signal?
      return 0;
    }
    perror("select()");
    return -1;
  } else if (result) {
    // data is waiting to read
    if (!FD_ISSET(m_nlfd, &rfds)) {
      printf("select returned non-zero but read not ready?\n");
      return -1;
    }

    int len = recvmsg(m_nlfd, &msg, 0);
    if (len == -1) {
      if (errno == EAGAIN || errno == EINTR) {
        // either the process received a signal or the read would have
        // blocked? select() man page describes these as spurious
        // ready reports, which we'll ignore.
        return 0;
      }
      perror("recvmsg");
      return -1;
    }
    nlm.setLength(len);
    return len;

  } else {
    // timeout
    return 0;
  }
}

NetLinkMsg::NetLinkMsg(int payloadLength) {
  int bytes = NLMSG_SPACE(payloadLength);
  m_capacity = bytes;
  m_length = 0;

  m_msg = reinterpret_cast<nlmsghdr*>(new char[bytes]);
  if (m_msg) {
    memset(m_msg, 0, bytes);
    m_msg->nlmsg_len = NLMSG_LENGTH(payloadLength);
  }
}

NetLinkMsg::~NetLinkMsg() {
  delete [](m_msg);
}

nlmsghdr *NetLinkMsg::header() {
  return m_msg;
}

void *NetLinkMsg::payload() {
  return NLMSG_DATA(m_msg);
}

int NetLinkMsg::length() {
  return m_length;
}

int NetLinkMsg::capacity() {
  return m_capacity;
}

void NetLinkMsg::setLength(int length) {
  m_length = length;
}
