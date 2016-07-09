#include "support.h"
#include "pktbuff.h"

#include "workqueue.h"
#include "drivercom.h"

/*
#define LOCK_INIT 1
#define LOCK_PO_MEM 2
#define MALLOCFREE_LOCK  3
#define SENDING_LOCK  4

#define LOCK_DS0  5
#define LOCK_DS1  6
#define LOCK_DS2  7
#define LOCK_DS3  8
#define LOCK_DS4  9
#define LOCK_DS5  10
#define LOCK_PREDEF 11
*/

#define RCV_QUOTA_START 8
#define RCV_QUOTA_STEP 4

#define LOCK_PREPARED_PKT LOCK_INIT

#define NUM_PKT_OUTS 8

/*
 * Time is a 32-bit unsigned int that continually increments.
 */
#define TIME_LT(a,b)     ((int)((a)-(b)) < 0)
#define TIME_LEQ(a,b)    ((int)((a)-(b)) <= 0)
#define TIME_GT(a,b)     ((int)((a)-(b)) > 0)
#define TIME_GEQ(a,b)    ((int)((a)-(b)) >= 0)

/*
  Contains information associated with one of the slots in input
  memory.
*/
struct pktin_mem {
  t_addr *addr;
  int ref_count;
  int id;
};

/*
  Represents a packet that is either being prepared and written to the
  output memory by a jobs_queue worker, or a packet that's waiting
  to be sent by the send_queue worker.
 */
struct pkt_out {
  unsigned int id;
  struct packet_hdr hdr;
  struct pktin_mem *in_mem;
  t_addr *input_mem_payload;
  t_addr *output_mem;
  
  
  struct work_task task;
  struct deque_item item;
};


/*
  One thread pops packets containing packet descriptions sent by the
  host PC. For each packet description a task is put on the
  jobs_queue to build that specific packet in the output
  memory. Multiple threads serve this queue.
 */
struct work_queue jobs_queue;

/*
  A single sender thread serves this queue sending packets in order at
  the correct time. Once a jobs_queue worker has moved a thread to
  the output memory it might add a task to this queue.
 */
struct mtdeque send_pkts;
/* Circular queue of packets that will be enqueued on the
   send_pkt. */
struct pkt_out *prepared_pkts[NUM_PKT_OUTS];
unsigned int next_pkt_tosend = 0;


/*
  Holds threads waiting for pkt_out objects that are ready to be reused.
 */
struct mtdeque free_pkts;

volatile int packets_processed = 0;

/* Points into output memory where the next quota packet is being
   built to send to the driver */
t_addr *quota_pkt = 0;

struct pkt_out pkt_outs[NUM_PKT_OUTS];


static struct work_task pop_task;
static unsigned int pkt_in_order = 0;


struct pktin_mem pktin_mems[10];
static inline struct pktin_mem *get_pktin_data(t_addr *pkt);


static void pop_work(unsigned long data);
static void prepare_work(unsigned long data);

/* Signals that threads should stop their work and reset everything as
   it was when the system first booted. Once the sending thread
   determines a reset is complete, it will send a packet to the host
   signalling that the NetFPGA is ready. */
static volatile int reset_system = 0;

/*
 Free a packet in the input memory. This may cause the sending thread
 to send a packet to the driver to tell it there are free buffers.
 */
static volatile int pkts_in_processed = 0;
static void free_input_pkt(struct pktin_mem *pktin) {
  log("t%d: Freeing pktin %p\n", nf_tid(), pktin->addr);
   
  pkts_in_processed++;
  nf_pktin_free(pktin->addr);
}

int verify_pkt_out(t_addr *pkt) {
  int i = 0;
  for (i = 0; i < 10; i++) {
    if (pkt == (t_addr*)((i*1600) | (1<<PACKETOUT_SEL))) 
      return 1;
  }
  return 0;
}

static void process_pkt_pkts(t_addr *data, t_addr *head, uint size) {
  /* move head up to the start of the pkt, which is aligned on 8 byte boundaries */

  t_addr *old_head = head;
  head = (t_addr*)(((unsigned long)head + 7) & ~7); 
  size -= head - old_head;

  struct packet_hdr* hdr = (struct packet_hdr*) head;
  t_addr *end = head + size - sizeof(struct packet_hdr);
  int pkt_num = 1;
  // count the number of packets

  log("pkthdr: len=%hu wire_len=%hu clock_hi=%u clock_lo=%u next=%hu\n", hdr->len, hdr->wire_len, hdr->clock_hi, hdr->clock_lo, hdr->next_pkt);

  while (hdr->next_pkt) {
    //log("hdr->next_pkt %hu\n", hdr->next_pkt);
    pkt_num++;

    /* Move ahead to next header by given offset */
    hdr = (struct packet_hdr*)((t_addr*)hdr + hdr->next_pkt);
    log("pkthdr: len=%hu wire_len=%hu clock_hi=%u clock_lo=%u next=%hu\n", hdr->len, hdr->wire_len, hdr->clock_hi, hdr->clock_lo, hdr->next_pkt);
    if ((t_addr*)hdr > end) {
      log("ERROR:t%d: Dropping corrupt packet\n", nf_tid());
      nf_pktin_free(data);
      return;
    }
  }

  log("Num packets %d\n", pkt_num);

  if ((t_addr*)hdr + hdr->len > end) {
    log("ERROR:t%d: Dropping truncated packet %d >= %u\n", nf_tid(),
        ((t_addr*)hdr + hdr->len + sizeof(struct packet_hdr)) - head,
        size
        );
    nf_pktin_free(data);
    return;
  }

  log("T%d: Processing pkt containing %d pkts\n", nf_tid(), pkt_num);

  struct pktin_mem *pm = get_pktin_data(data);
  pm->ref_count = pkt_num;
  pm->id = packets_processed;
  packets_processed++;

  hdr = (struct packet_hdr*) head;

  while (1) {
    /* Pop multiple free packets at once */
    struct deque_item *item;
    pkt_num -= mtdeque_pops(&free_pkts, &item, pkt_num, CAN_BLOCK);
    log("T%d: Still need %d free pkts\n", nf_tid(), pkt_num);

    workq_lock(&jobs_queue);
    while (item) {
      struct pkt_out *po = deque_entry(item, struct pkt_out, item);
    
      /* prepare the packet */
      po->id = pkt_in_order++;
      po->in_mem = pm;
      po->hdr = *hdr;
      po->input_mem_payload = (t_addr*)hdr + sizeof(struct packet_hdr);
    
      workq_add_task_nolock(&jobs_queue, &po->task, prepare_work, (unsigned long) po);
      //    i++;
      if (hdr->next_pkt == 0) {
        workq_unlock(&jobs_queue);
        goto done;
      }

      /* Move ahead to next header by given offset */
      hdr = (struct packet_hdr*)((t_addr*)hdr + hdr->next_pkt);
      item = item->next;
    }
    workq_unlock(&jobs_queue);
  }
 done: ;
}

static void process_pkt(t_addr *data) {
  struct ioq_header *ioq = (struct ioq_header*) data;
  log("t%d: process_pkt of size %u from port %u\n", nf_tid(), ntohs(ioq->byte_length), ntohs(ioq->src_port));
  unsigned int size = ntohs(ioq->byte_length);
  if (size < sizeof(struct driver_to_netfpga)) {
    log("ERROR:t%d: Dropping corrupt packet. Too short. size = %hu\n", nf_tid(), size);
    nf_pktin_free(data);
    return;
  }
  struct driver_to_netfpga *msg = (struct driver_to_netfpga*) (ioq + 1);

  t_addr *head = (t_addr *) (msg + 1);

  log("ioq: %p msg: %p head: %p\n", ioq, msg, head);
  size -= sizeof(struct driver_to_netfpga);

  /*  int i,j; 
  for (i = 0; i < 4; i++) {
    for (j = 0; j < 8; j++) {
      log("0x%02hhx,", data[i * 8 + j]);
    }
    log("\n");
    }*/
  
  switch (msg->type) {
  case D2N_TYPE_RESET:
    log("t%d: received RESET command from hardware\n", nf_tid());
    reset_system = 1;
    nf_pktin_free(data);
    break;
  case D2N_TYPE_PKTS:
    process_pkt_pkts(data, head, size);
    /* schedule the another pkt pop */
    workq_add_task(&jobs_queue, &pop_task, pop_work, 0);
    break;
  default:
    log("t%d: process_pkt with unknown command %d\n", nf_tid(), msg->type);
    /* schedule the another pkt pop */
    workq_add_task(&jobs_queue, &pop_task, pop_work, 0);
    nf_pktin_free(data);
    break;
  }
}

static void pop_work(unsigned long data) {
  log("t%d: pop_work\n", nf_tid());

  while (1) {
    t_addr *pkt = nf_pktin_pop();
    if (nf_pktin_is_valid(pkt)) {
      struct ioq_header *ioq = (struct ioq_header*) pkt;
      /* Only look at packets that came from the host PC */
      //log("Popped pktin at %p\n", pkt);
      if (ntohs(ioq->src_port) == INPUT_PORT) {
        log("Popped pktin %p\n", pkt);
        process_pkt(pkt);
        
        return;
      }
      nf_pktin_free(pkt);
    }
  }
}


static void prepare_work(unsigned long data) {
  struct pkt_out *po = (struct pkt_out*) data;

  log("t%d: prepare_work %d\n", nf_tid(), po->id);

  /* Prepare the packet in output memory */
  /*struct ioq_header *ioq = (struct ioq_header*)po->output_mem;
  fill_ioq(ioq, OUTPUT_PORT, po->hdr.wire_len);
  memcpy32(ioq + 1, po->input_mem_payload, po->hdr.len);*/

  struct pkt_buff tosend;
  pkt_fill(&tosend, po->output_mem, po->hdr.wire_len + sizeof(struct ioq_header));
  fill_ioq(pkt_pull_type(&tosend, struct ioq_header), OUTPUT_PORT, po->hdr.wire_len);
  log("t%d: copying pkt of size %d from %p to %p\n", nf_tid(), po->hdr.len, tosend.head, po->input_mem_payload);
  memcpy32(tosend.head, po->input_mem_payload, po->hdr.len);

  unsigned int index = po->id % NUM_PKT_OUTS;

  /*
    Lock to put packet in prepared packet array.
   */
  nf_lock(LOCK_PREPARED_PKT);
  if (next_pkt_tosend == po->id) {
    /* The packet we just prepared is next to go to the sending
       thread. Push it and any following packets that are also ready
       to the sending thread */
    int num_ready = 1;
    struct pkt_out *ready_pkt;

    mtdeque_lock(&send_pkts);
    mtdeque_push_tail_nolock(&send_pkts, &po->item);

    while (1) {
      index = (index + 1) % NUM_PKT_OUTS;
      ready_pkt = prepared_pkts[index];
      if (ready_pkt == 0) {
        break;
      }
      prepared_pkts[index] = 0;
      mtdeque_push_tail_nolock(&send_pkts, &ready_pkt->item);
      num_ready++;
    }
    log("t%d: Pushed %d packets to the sender\n", nf_tid(), num_ready);
    next_pkt_tosend += num_ready;

    mtdeque_unlock(&send_pkts);
  } else {
    prepared_pkts[index] = po;
  }

  /* If this was the last work using the packet in input memory then
     free it */
  struct pktin_mem *in_mem = po->in_mem;
  log("t%d: prepare from packet %p refcount %d\n", nf_tid(), in_mem->addr, in_mem->ref_count);
  int cnt = --in_mem->ref_count;
  if (cnt == 0) {
    free_input_pkt(in_mem);
  }
  nf_unlock(LOCK_PREPARED_PKT);
}

/* The current clock time observed by the sending thread. */
uint time_hi; /* counts how many times time_lo wraps */
uint time_lo;

static inline void update_time() {
  uint time_now = nf_time();
  if (time_now < time_lo) {
    /* time has wrapped */
    time_hi++;
  }
  time_lo = time_now;
}

#define LATE 0
#define FAR 0xFFFFFFFFu

static uint time_until(uint target_time_hi, uint target_time_lo) {
  if (target_time_hi < time_hi || (target_time_hi == time_hi && target_time_lo <= time_lo)) {
    return LATE;
  } else {
    switch (target_time_hi - time_hi) {
    case 0:
      return target_time_lo - time_lo;
    case 1:
      if (target_time_lo >= time_lo) {
        return FAR;
      } else {
        return (0xFFFFFFFFu - time_lo + 1) + target_time_lo;
      }
    default:
      return FAR;
    }
  }
}


static inline t_addr *nf_pktout_send_finish_nocollect() {
  return (t_addr*) ((1<<PACKETOUT_SEL) | *(uint volatile *)SEND_OUT_W);
}

static int pkts_in_processed_seen = 0;

/* Handle sending special packets. Special packets are either quota
   packets sent to the host or incoming packets sent out port 1 */
static int send_special() {
  /* May need to notify the host that we can accept more
     packets */
  int difference = pkts_in_processed - pkts_in_processed_seen;
  if (difference >= RCV_QUOTA_STEP) {
    /*if (!verify_pkt_out(quota_pkt)) {
      return 0;
      }*/

    pkts_in_processed_seen += difference;

    int size = sizeof(struct ioq_header) + sizeof(struct netfpga_to_driver);
    log("t%d: Sending reset command back to driver of size %d\n", nf_tid(), size);
    struct pkt_buff cmd;
    pkt_fill(&cmd, quota_pkt, size);
    fill_ioq(pkt_pull_type(&cmd, struct ioq_header), 1, sizeof(struct netfpga_to_driver));
    /*struct ioq_header *ioq = (struct ioq_header *) cmd.data;
      ioq->dst_port = 2 | 8;*/
    struct netfpga_to_driver *n2d = pkt_pull_type(&cmd, struct netfpga_to_driver);
    n2d->type = N2D_TYPE_QUOTA;
    n2d->pkt_quota = difference;
    
    nf_pktout_send_setup(cmd.data, cmd.head);
    /* Update the time just before sending the command. Most accurate. */
    update_time();
    n2d->time_hi = time_hi;
    n2d->time_lo = time_lo;
    quota_pkt = nf_pktout_send_finish_nocollect();
    return 1;
  }
  return 0;
}

static uint send_time_hi;
static uint send_time_lo;

static void tight_send_loop(struct pkt_out *pkt, int send_now) {
  /* prepare to send packet */
  t_addr *pkt_start = pkt->output_mem;
  uint pkt_size = pkt->hdr.wire_len + sizeof(struct ioq_header);
  t_addr *pkt_end = pkt_start + pkt_size;

  /*if (!verify_pkt_out(pkt_start)) {
    return;
    }*/

  /*  ((unsigned*)pkt_start)[2] = pkt_size;
  ((unsigned*)pkt_start)[3] = (uint)pkt_start;

  if (send_now) {
    ((unsigned*)pkt_start)[4] = 0xdeadbeef;
  } else {
    ((unsigned*)pkt_start)[4] = 0xabababab;
    }*/

      
  nf_pktout_send_setup(pkt_start, pkt_end);
  if (!send_now) {
    //send_time -= 8;
    /* Here is the tight send loop! */
    //while (TIME_LT(nf_time(), send_time)) {}
    nf_pktout_send_schedule(send_time_lo);
  } 
  pkt->output_mem = nf_pktout_send_finish_nocollect();
  if (send_now) {
    /* Missed a deadline probably, record the send time */
    update_time();
    send_time_hi = time_hi;
    send_time_lo = time_lo;
  }

  mtdeque_push_tail(&free_pkts, &pkt->item);

  send_special();
}

static void reset_output_packets(struct deque_item *item, int output_queue) {
  //log("t%d: resetting waiting for free packtets. have %d\n", nf_tid(), free_pkts.size);
  while (1) {
    mtdeque_lock(&free_pkts);
    while (item) {
      struct deque_item *next = item->next;
      mtdeque_push_tail_nolock(&free_pkts, item);
      item = next;
    }
    mtdeque_unlock(&free_pkts);
    
    //log("t%d: resetting waiting for free packtets. have %d\n", nf_tid(), free_pkts.size);
    if (free_pkts.size == NUM_PKT_OUTS) {
      break;
    }
    /* Wait until the other threads preparing packets finish. */
    mtdeque_pops(&send_pkts, &item, 0xFFFF, CAN_BLOCK);        
  }

  /* Send a reset message to the driver. Use one of the free packets
     we just waited for */
  item = mtdeque_pop(&free_pkts, CAN_BLOCK);
  struct pkt_out *pkt = deque_entry(item, struct pkt_out, item);
  

  int size = sizeof(struct ioq_header) + sizeof(struct netfpga_to_driver);
  log("t%d: Sending reset command back to driver of size %d\n", nf_tid(), size);
  struct pkt_buff cmd;
  pkt_fill(&cmd, pkt->output_mem, size);
  fill_ioq(pkt_pull_type(&cmd, struct ioq_header), output_queue, sizeof(struct netfpga_to_driver));
  /*struct ioq_header *ioq = (struct ioq_header *) pkt->output_mem;
    ioq->dst_port = 2 | 32 | 128;*/
  struct netfpga_to_driver *n2d = pkt_pull_type(&cmd, struct netfpga_to_driver);
  n2d->type = N2D_TYPE_RESET;
  n2d->pkt_quota = RCV_QUOTA_START;

  nf_pktout_send_setup(cmd.data, cmd.head);
  /* Update the time just before sending the command. Most accurate. */
  update_time();
  n2d->time_hi = time_hi;
  n2d->time_lo = time_lo;
  pkt->output_mem = nf_pktout_send_finish_nocollect();
  mtdeque_push_tail(&free_pkts, &pkt->item);
}

static inline void relax_for(int i) {
  while (i > 0) {
    i--;
    relax();
  }
}

static void manage_sending() {
  /* store time as two 32-bit numbers */
  time_hi = 0;
  time_lo = nf_time();

  send_time_hi = 0;
  send_time_lo = time_lo;

  struct deque_item *item = 0;
  struct pkt_out *pkt = 0;

  while (1) {
    /* outer loop - update time while checking for packets to send */
    update_time();

    if (reset_system) {
      /* Stop sending packets. */
      log("t%d: Noticed RESET!\n", nf_tid());
      
      /* reclaim all the output memory buffers */
      if (pkt) {
        /* ensure it's linked to the rest of the pkts we've already
           got from the send_pkts queue (This probably isn't necessary) */
        pkt->item.next = item;
        item = &pkt->item;
        pkt = 0;
      }
      reset_output_packets(item, 1);
      item = 0;
      reset_system = 0;
      
      log("RESET COMPLETE\n");
      
      /* Start popping packets again */
      workq_add_task(&jobs_queue, &pop_task, pop_work, 0);
    } else if (pkt) {
      /* have a packet to send */

      uint time_left = time_until(send_time_hi, send_time_lo);

      //log("Have a packet. time_left = %d\n", time_left);

      if (time_left == LATE) {
        /* Missed a deadline. Send now! */
        tight_send_loop(pkt, 1);
        pkt = 0;
      } else if (time_left < 5000) {
        /* need to send soon, go into tight inner loop */
        tight_send_loop(pkt, 0);
        pkt = 0;
      } else {
        /* lots of time to wait */
        if (send_special() == 0) {
          relax_for(10);
        }
      }
    } else if (item) {
      pkt = deque_entry(item, struct pkt_out, item);
      item = item->next;

      log("WILL SEND PKT abs? %hd hi: %u lo: %u\n", pkt->hdr.is_absolute, pkt->hdr.clock_hi, pkt->hdr.clock_lo);

      if (pkt->hdr.is_absolute) {
        send_time_hi = pkt->hdr.clock_hi;
        send_time_lo = pkt->hdr.clock_lo;
      } else {
        uint old_time_lo = send_time_lo;
        send_time_lo += pkt->hdr.clock_lo;
        send_time_hi += pkt->hdr.clock_hi;
        if (pkt->hdr.clock_hi == 0 && pkt->hdr.clock_lo < 5000) {
          /* this packet needs to be sent soon. Go straight to tight loop. */
          tight_send_loop(pkt, 0);
          pkt = 0;
        }
        if (send_time_lo < old_time_lo) {
          /* keep track of wraps */
          send_time_hi++;
        }
      }
    } else if (send_pkts.size) {
      mtdeque_pops(&send_pkts, &item, 0xFFFF, NO_BLOCK);
    } else {
      /* nothing to send. delay for a bit to avoid hammering memory
         controller */
      if (send_special() == 0) {
        relax_for(10);
      }
    }
  }
}

static inline struct pktin_mem *get_pktin_data(t_addr *pkt) {
    unsigned long addr = (unsigned long) pkt;
#ifndef CONTEXT_NF
  // verify address

    if (((addr & ~0x3FFF) ^ (unsigned long) header_mem) != 0) {
      log("ERROR:t%d: invalid pktin address %p\n", nf_tid(), pkt);
    } else {
      addr &= 0x3FFF;
      if (addr % 1600 != 0) {
        log("ERROR:t%d:invalid pktin address %p\n", nf_tid(), pkt);
      }
    }
    addr = (unsigned long) pkt;
#endif

    int index = (addr & 0x3FFF) / 1600;
#ifndef CONTEXT_NF
    if (index >= 10) {
      log("ERROR:t%d: invalid pktin address %p index %d\n", nf_tid(), pkt, index);
    }
#endif
    log("got packet in data %d\n", index);
    return &pktin_mems[index];
}

/*int delay_val = 1;
static void waste_time() {
  int i;
  int v;

  v = delay_val;
  for (i = 0; i < 10; i++) {
    v += v + 13;
  }
  delay_val = v;
  }*/

int main() {
  if (nf_tid() == 0) {
    int i;
    nf_lock(LOCK_INIT);
    nf_pktout_init();
    nf_pktin_init();

    workq_init(&jobs_queue, LOCK_DS0, LOCK_DS1);
    mtdeque_init(&send_pkts, LOCK_DS2, LOCK_DS3);
    mtdeque_init(&free_pkts, LOCK_DS4, LOCK_DS5);

    quota_pkt = nf_pktout_alloc(1520);

    /* Add all pkt_out structs to the free list */
    mtdeque_lock(&free_pkts);
    for (i = 0; i < NUM_PKT_OUTS; i++) {
      struct pkt_out *po = pkt_outs + i;
      po->output_mem = nf_pktout_alloc(1520);
      mtdeque_push_nolock(&free_pkts, &po->item);
    }
    mtdeque_unlock(&free_pkts);

    workq_add_task(&jobs_queue, &pop_task, pop_work, 0);

    
    for (i = 0; i < 10; i++) {
      struct pktin_mem *pm = &pktin_mems[i];
      pm->addr = header_mem + (1600 * i);
      pm->ref_count = 0;
    }

    /*    t_addr *pktin, *pktout;
    while (!nf_pktin_is_valid(pktin = nf_pktin_pop())) {}

    pktout = nf_pktout_alloc(1520);
    fill_ioq((struct ioq_header*)pktout, 1, 98);
    
    nf_pktout_send(pktout, pktout + sizeof(struct ioq_header) + 98);
    while (1) {}*/

   } else {
    nf_stall_a_bit();
    nf_lock(LOCK_INIT);
  }
  nf_unlock(LOCK_INIT);

  if (nf_tid() == 0) {
    manage_sending();
  } else {
    workq_serve(&jobs_queue, -1);
  }

  while (1) {}
  return 0;
}
