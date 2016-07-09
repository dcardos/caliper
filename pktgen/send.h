#ifndef SEND_H
#define SEND_H

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

#include "args.h"

class Send {
 public:
  Send(Args &args);
  bool init();
  void go();


 private:
  Args &args;

  int sk;
  struct sockaddr_in si;

};





#endif
