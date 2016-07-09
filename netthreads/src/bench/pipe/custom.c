#include "pktbuff.h"
#include "support.h"
#define SIZE 128
int main(void)
{
  if (nf_tid() != 0) {
    while (1) {}
  }
  nf_lock(LOCK_INIT);
  nf_pktout_init();
  nf_pktin_init();
  nf_unlock(LOCK_INIT);

  t_addr *pkt = nf_pktout_alloc(SIZE);
  fill_ioq((struct ioq_header *)pkt, 0, SIZE-sizeof(struct ioq_header));

  nf_pktout_send(pkt, pkt + SIZE);
  nf_pktout_send(pkt, pkt + SIZE);
  nf_pktout_send(pkt, pkt + SIZE);
  nf_pktout_send(pkt, pkt + SIZE);

  while (1) {}
  return 0;
}

