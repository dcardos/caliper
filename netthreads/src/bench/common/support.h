#ifndef SUPPORT_H
#define SUPPORT_H

#include "common.h"

typedef char t_addr;

#define      HEADER_MEM_SEL 28
#define      PACKETOUT_SEL 27
#define      CONNSTATE_SEL_COMM  26
#define      CONNSTATE_SEL_HP CONNSTATE_SEL_COMM
#define      CONNSTATE_SEL_PO CONNSTATE_SEL_COMM
#define      NOT_A_PACKET 0xffffffff

#define TOBS  31
#define LOBS 29

#define  EXTENDED_BIT (1<<2)
#define  GET_LOCK    ((1<<LOBS)|EXTENDED_BIT)
#define  UN_LOCK     ((2<<LOBS)|EXTENDED_BIT)
#define  PRINTF_W    ((3<<LOBS)|EXTENDED_BIT)
#define  SEND_TIME_TRIG ((4<<LOBS)|EXTENDED_BIT)

#define	 CTRL_START (1<<LOBS)
#define	 CTRL_END   (2<<LOBS)
#define  CTRL_END_W (3<<LOBS)
#define  SEND_OUT_W (4<<LOBS)
#define  HEADER_FLUSH_W (5<<LOBS)
#define  COMMON_TIME (6<<LOBS)
#define  HEADER_RETURN_W   (7<<LOBS)

#define STACK_IN_BYTES 32768 // 1024
// don't use the small starting stack
#define STACK_SKIP     256  // 8 bytes, 2 cpus, 4 threads 
#define DCACHE         16384
#define THREADS_PER_CPU    4

#define PKTIN_BACKOFF 0

#ifdef MAIN_SUPPORT
t_addr* header_mem = (t_addr*)(1<<HEADER_MEM_SEL); 
t_addr* conn_mem = (t_addr*)(1<<CONNSTATE_SEL_COMM); 
t_addr* packetout_mem = (t_addr*)(1<<PACKETOUT_SEL); 
#else
extern t_addr* header_mem; 
extern t_addr* conn_mem; 
extern t_addr* packetout_mem; 
#endif


/*
  Defining INLINE_API to be non-zero will cause some functions in here
  to be inlined. The actual implementation of these functions is at
  the bottom of this header.

  By default we do inline the functions. Add
  "EXTRA_DEFINES=-DINLINE_API=0" to your Makefile before including
  ../bench.mk to disable the inlining.
 */
#ifndef INLINE_API
#define INLINE_API 1
#endif

#if INLINE_API
  #define INLINE_FUNC static inline
  #define INCLUDE_IMPL
#else
  #define INLINE_FUNC

  #ifdef MAIN_SUPPORT
    #define INCLUDE_IMPL
  #endif
#endif


// ****** incoming packets ******
// Initialize the packet input memory, dividing into 10 packets. This
// is optional, and you can call nf_pktin_free directly.
void nf_pktin_init();
INLINE_FUNC int nf_pktin_is_valid(t_addr* addr);

// ****** outgoing packets ******
#define NUM_OUTPUT_BUFS 10
void nf_pktout_init();
t_addr* nf_pktout_alloc(uint size);
void nf_pktout_send(char* start_addr, char* end_addr);
void nf_pktout_free(t_addr* addr);

#ifdef DEBUG
uint nf_time();
#else
INLINE_FUNC uint nf_time();
#endif

void nf_lock(int id);
void nf_unlock(int id);

int nf_tid();
int nf_stall_a_bit();
void relax();

// my locks
#define HAS_LOCKS 1
#define LOCK_INIT 0
#define LOCK_PKTIN 0
#define LOCK_MISC 1
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

// memory.c
void sp_init_mem_single();
void sp_init_mem_pool();
void *sp_malloc(size_t size);
void sp_free(void *ptr);
void* sp_realloc(void *ptr, int new_size);
// memcpy.c
void *memcpy32(void *dst, const void *src, size_t count);

#ifdef CONTEXT_NF
#define print_pkt(pkt, len)
#else
void print_pkt(char *pkt, int len);
#endif



typedef struct {
  volatile int lock;
  volatile int writing;
  volatile int writer_pending;
  volatile int writer_id;
  volatile int reader_cnt;
} rw_lock;

void rw_lock_init(rw_lock* rw, int lock);
void reader_lock(rw_lock* rw);
void reader_unlock(rw_lock* rw);
void writer_lock(rw_lock* rw);
void writer_unlock(rw_lock* rw);


/***************** Alternative to nf_pktout_send ************************

These functions divide sending into two parts to give more control
over when a packet is actually sent. Be careful to ALWAYS call finish
after calling setup. Avoid using this function if you're sending
packets from multiple threads.
 */
void nf_pktout_send_setup(char* start_addr, char* end_addr);
void nf_pktout_send_schedule(unsigned scheduled_time);
void nf_pktout_send_finish();
/*****************************************/


#ifdef INCLUDE_IMPL

#if !PKTIN_BACKOFF
INLINE_FUNC t_addr* nf_pktin_pop()
{
#ifndef DEBUG
  uint volatile*loc1 = (uint*)(HEADER_FLUSH_W);
  return (t_addr*)((*loc1)|(1<<HEADER_MEM_SEL)); // or the mem-id bit to the base address
#else
  extern char* sw_pktin_pop();
  return (sw_pktin_pop()); 	/** sw_pkt_io.cc**/
#endif
}

INLINE_FUNC void nf_pktin_free(t_addr* val)
{
#ifndef DEBUG
      uint volatile*loc1 = (uint*)(HEADER_RETURN_W);
      *loc1 = (uint)val;
#else
  return;
#endif
}
#else
t_addr* nf_pktin_pop();
void nf_pktin_free(t_addr* val);
#endif

INLINE_FUNC int nf_pktin_is_valid(t_addr* addr)
{
  return (((unsigned int)addr) != NOT_A_PACKET);
}

#ifndef DEBUG
// the DEBUG implementation of nf_time() is in support.c
INLINE_FUNC uint nf_time()
{
  return* (uint volatile*)((uint)(COMMON_TIME));  
}
#endif 

#endif


#endif
