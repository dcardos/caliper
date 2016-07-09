/* ****************************************************************************
 * $Id: nf2kernel.h 644 2005-10-20 17:35:01Z gwatson $
 *
 * Module: nf2kernel.h
 * Project: NetFPGA 2 Linux Kernel Driver
 * Description: Header file for kernel driver
 *
 * Change history:
 *
 */

#ifndef _NF2KERNEL_H
#define _NF2KERNEL_H	1

#ifdef __KERNEL__

#include <linux/cdev.h>
#include <linux/sockios.h>
#include <linux/netdevice.h>
#include <linux/fs.h>
#include <asm/atomic.h>

#include <asm/semaphore.h>
#include <linux/kfifo.h>
#include <linux/list.h>

/* Define PCI Vendor and device IDs for the NF2 card */
#define PCI_VENDOR_ID_STANFORD		0xFEED
#define PCI_DEVICE_ID_STANFORD_NF2	0x0001

/* Prefix for device names 
 * - will have either c or u appended to indicate control or user
 */
#define NF2_DEV_NAME	"nf2"

/* How many interfaces does a single card support */
#ifndef MAX_IFACE
#define MAX_IFACE	1
#endif

/* Transmit timeout period */
#define NF2_TIMEOUT	(1000 * HZ)

/* How many transmit buffers to allocate */
#define NUM_TX_BUFFS	16

/* How many transmit buffers to allocate */
#define NUM_RX_BUFFS	16

/* How large is the largest DMA transfer */
#define MAX_DMA_LEN	2048

/* Major device number for user devices */
#define NF2_MAJOR 0   /* dynamic major by default */

/* Maximum transmission size -- this should be 
 * max packet size - 14 for the Ethernet header */
#define MTU 1986

#define NF2_DELAY_FIFO_LEN 1024

/*
 * Debugging diagnostic printk
 */
#ifdef NF2_DEBUG
#  define PDEBUG(fmt, args...) printk(fmt, ## args)
#else
#  define PDEBUG(fmt, args...)	/* Don't do anything */
#endif

/*
 * Interface data structure
 * - an instance of this structure exists for each interface/port
 *   on a control card.
 *
 *   Not used for user cards.
 */

struct nf2_iface_priv {
	/* Which card does this IF belong to? */
	struct nf2_card_priv *card;

	/* What number interface is this? */
	unsigned int iface;

	/* Statistics for the interface */
	struct net_device_stats stats;
};


/*
 * User card private data
 * - an instance of this structure exists for each user card.
 *
 *   Not used for control cards.
 */
struct nf2_user_priv {
	/* The card corresponding to this structure */
	struct nf2_card_priv *card;

	/* How many times has this been opened? */
	int open_count;

	/* dev_t for user cards */
	dev_t dev;

	/* Char device */
	struct cdev cdev;

	/* Mutual exclusion semaphore */
	struct semaphore sem;

	/* Track the number of bytes available for reading */
	u32 rx_wr_pos, rx_rd_pos;

	/* Actual read position */
	unsigned char *rx_buf_rd_pos;

	/* Read and write queues */
        wait_queue_head_t inq, outq;
};


/*
 * Card data structrue
 * - an instance of this data structure exists for each card.
 */
struct nf2_card_priv {
	/* PCI device corresponding to the card */
	struct pci_dev *pdev;

	/* Address in memory of board */
	void *ioaddr;

	/* Control card */
	char is_ctrl;

	/* Transmit Buffers */
	struct txbuff *txbuff;

	/* Spinlock for the buffer variables */
	spinlock_t txbuff_lock;

        struct list_head free_txbuffs;
        struct list_head transmit_txbuffs;
        struct txbuff *txbuff_building;

        int allow_send;
        int send_immediately;

	/* Address of the DMA transfer */
	u32 dma_tx_addr;
	u32 dma_rx_addr;

        unsigned int dma_tx_len;

        struct timer_list timer;

	/* Is a DMA transfer in progress? */
        // bit flags determining if tx or rd DMA is in progress
        unsigned long dma_in_progress_flags;

	/* === Control Card Variables === */
	/* Network devices */
	struct net_device *ndev[MAX_IFACE];

	/* Which interfaces are currently up? 
	 * Note: This is a bitmask*/
	unsigned int ifup;

	/* Semaphore for the state variables */
	struct semaphore state_lock;


	/* === User Card Variables === */
	struct nf2_user_priv *upriv;

	/* The current buffer to process and the last 
	 * buffer used from the pool */
	struct nf2_packet *wr_pool;

        struct timespec pkt_avail_time;

        s64 ns2_sim_origin_ns;
        s64 nf_clock_origin_ns;

        s64 ns2_sim_to_nf_clock;

        s64 old_clock;
        s64 old_time;
        s64 avg_clock_drift;
};

struct packet_hdr;

/*
 * Buffer to hold packets to be transmitted
 */
struct txbuff {
  	u8 buff[MAX_DMA_LEN];
  	int len;
        unsigned int iface;
  struct packet_hdr *last_hdr;
  struct list_head list;

  u32 mark_pid;
  struct sk_buff *mark_skb;
};


/*
 * A structure representing an in-flight packet being received.
 */
struct nf2_packet {
	int len;
	u8 data[MAX_DMA_LEN + 2];
};


/*
 * Functions
 */
int nf2u_probe(struct pci_dev *pdev, const struct pci_device_id *id, struct nf2_card_priv *card);
void nf2u_remove(struct pci_dev *pdev, struct nf2_card_priv *card);
int nf2c_probe(struct pci_dev *pdev, const struct pci_device_id *id, struct nf2_card_priv *card);
void nf2c_remove(struct pci_dev *pdev, struct nf2_card_priv *card);

struct pkt_descr;
struct NFSimStartMsg;
int nf2c_inject_pkt(struct pkt_descr* pkt);
int nf2c_sim_start(struct NFSimStartMsg* sim);
int nf2c_mark(u32 pid, u32 seq);

/*
 * Variables
 */
extern uint pkt_delay;
extern int timeout;
extern int rx_pool_size;
extern int tx_pool_size;
extern int nf2_major;
extern int nf2_minor;

extern struct sock *nf2_nl_sk;
extern struct kfifo *nf2_delay_fifo;
extern struct semaphore nf2_delay_wr_sem;


/* Fixes for older kernels */
#if LINUX_VERSION_CODE <  KERNEL_VERSION(2,6,22)

static inline struct nlmsghdr *nlmsg_hdr(const struct sk_buff *skb) {
  return (struct nlmsghdr *)skb->data;
}

#define list_first_entry(ptr, type, member) \
  list_entry((ptr)->next, type, member)

#endif

#endif	/* __KERNEL__ */

#endif	/* _NF2KERNEL_H */
