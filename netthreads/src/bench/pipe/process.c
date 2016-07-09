#include "support.h"
#include "pktbuff.h"

static void process_pkt(void* data) {
  struct pkt_buff pkt;
  struct ioq_header *ioq = data;
  unsigned int size = ntohs(ioq->byte_length);
  unsigned short dest = ntohs(ioq->src_port) ^ 2;

  //print_pkt(data, size + sizeof(struct ioq_header));
  log("%hd -> %hd (%d)\n", ntohs(ioq->src_port), dest, size);

  //  log("ioq_hdr: dst=%hx words=%hu src=%hu bytes=%hu\n", ntohs(ioq->dst_port), ntohs(ioq->word_length), ntohs(ioq->src_port), size);
  pkt_fill(&pkt, data, size + sizeof(struct ioq_header));

  struct pkt_buff reply;
  struct ioq_header *dioq;

  pkt_push_all(&pkt);
  while (pkt_alloc(&reply, pkt.len) == 0) {
    log("Failed to alloc\n");
  }

  memcpy32(reply.data+ sizeof(struct ioq_header),
	   pkt.data  + sizeof(struct ioq_header), 
	   reply.len - sizeof(struct ioq_header));

  nf_pktin_free(data);
	
  dioq = pkt_pull_type(&reply, struct ioq_header);
  fill_ioq(dioq, dest, size);

  nf_pktout_send(reply.data, reply.data + reply.total_size);
}


int main(void)
{
  t_addr* next_packet;
  int i = 0;
  
  int mytid = nf_tid();

  if(mytid != 0)
    {
      nf_stall_a_bit();
      nf_lock(LOCK_INIT); // should not get it
    }
  else
    {
      nf_lock(LOCK_INIT); // should get it immediately
      nf_pktout_init();
      nf_pktin_init();      
    }

  nf_unlock(LOCK_INIT); 


  while(1) 
    {
      next_packet = nf_pktin_pop();
      if (nf_pktin_is_valid(next_packet)) {
        i++;
        log("thread %d sending packet %d from %p\n", nf_tid(), i, next_packet);
        process_pkt(next_packet);
      }
    }

  while(1); 

  return 0;
}
