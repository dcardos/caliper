/* ****************************************************************************
 * vim:set shiftwidth=2 softtabstop=2 expandtab:
 * $Id: nf2_control.c 1547 2007-03-27 19:56:58Z grg $
 *
 * Module: nf2_util.c
 * Project: NetFPGA 2 Linux Kernel Driver
 * Description: Control card functionality
 *
 * Change history:
 *   7/8/2008 - Jad Naous: - fixed problem with newer kenrels where SA_SHIRQ is
 *                           not defined
 *                         - Fixed various warnings
 *
 * To Do: - Check that the timeout handler works okay when multiple ports
 *          are enabled
 *
 */

#include <linux/version.h>
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,8) 
#include <linux/config.h>
#endif

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/interrupt.h>

#include <linux/in.h>
#include <linux/netdevice.h>   /* struct device, and other headers */
#include <linux/etherdevice.h> /* eth_type_trans */
#include <linux/if_ether.h>


#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/semaphore.h>
#include <asm/bitops.h>
#include <asm/div64.h>

#include "../common/nf2.h"
#include "nf2kernel.h"
#include "nf2util.h"
#include "nf2_export.h"
#include "nf_netlink.h"
#include "pktparse.h"

#include "drivercom.h"

#define KERN_DFLT_DEBUG KERN_INFO

/* JN: If we are working with an older kernel, it would probably
 * still use the SA_SHIRQ */
#ifndef IRQF_SHARED
#define IRQF_SHARED SA_SHIRQ
#endif

#define DROP_PAYLOAD

/* Control card device number */
static int devnum = 0;

/* Delays buffer */
static u32 partial_delay;
static unsigned int valid_delay_bytes = 0;


static struct net_device *inject_dev = NULL;

/* Function declarations */
static int nf2c_attempt_send(struct net_device *dev);
static void nf2c_rx(struct net_device *dev, struct nf2_packet *pkt);
static int nf2c_create_pool(struct nf2_card_priv *card);
static void nf2c_destroy_pool(struct nf2_card_priv *card);
static irqreturn_t nf2c_intr(int irq, void *dev_id
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
                             , struct pt_regs * regs
#endif
                             );
static void nf2c_clear_dma_flags(struct nf2_card_priv *card);

/*static void timer_func(unsigned long arg);*/

static int nf2c_tx_cmd(u8 *cmd, uint len, struct net_device *dev);


static inline int start_tx_dma(struct nf2_card_priv *card) {
  return !test_and_set_bit(0,  &card->dma_in_progress_flags);
}

static inline int start_rd_dma(struct nf2_card_priv *card) {
  return !test_and_set_bit(1,  &card->dma_in_progress_flags);
}

static inline void finish_tx_dma(struct nf2_card_priv *card) {
  clear_bit(0, &card->dma_in_progress_flags);
}

static inline void finish_rd_dma(struct nf2_card_priv *card) {
  clear_bit(1, &card->dma_in_progress_flags);
}

static inline int dma_tx_in_progress(struct nf2_card_priv *card) {
  return test_bit(0, &card->dma_in_progress_flags);
}

static inline int dma_rd_in_progress(struct nf2_card_priv *card) {
  return test_bit(1, &card->dma_in_progress_flags);
}

/*
 * open method - called when the interface is brought up
 *
 * Locking: state_lock - prevent the state variable and the corresponding
 *                       register from getting out of sync
 */
static int nf2c_open(struct net_device *dev)
{
  int err = 0;
  u32 mac_reset;
  u32 enable;
  struct nf2_iface_priv *iface = (struct nf2_iface_priv *)netdev_priv(dev);
  struct nf2_card_priv *card = iface->card;
  int go_up = 0;


  PDEBUG(KERN_DFLT_DEBUG "nf2: bringing up card\n");

  /* Aquire the mutex for the state variables */
  if (down_interruptible(&card->state_lock))
    return -ERESTARTSYS;

  /* Tell the driver the carrier is down... */
  netif_carrier_off(dev);

  /* Check if any other interfaces are active.
   * If no other interfaces are up install the IRQ handler
   * Always attach the interrupt to the first device in the pool 
   */
  if (!card->ifup)
  {
    nf2_hw_reset(card);
    if((err = request_irq(card->pdev->irq, nf2c_intr, IRQF_SHARED,
                          card->ndev[0]->name, card->ndev[0])))
      goto out;
    nf2_enable_irq(card);

    /*setup_timer(&card->timer, timer_func, (unsigned long) dev);*/
    /*printk(KERN_ALERT "Arming timer\n");
      mod_timer(&card->timer, jiffies + HZ/2);*/
    go_up = 1;
  }

  /* Modify the ifup flag */
  card->ifup |= 1 << iface->iface;
  PDEBUG(KERN_DFLT_DEBUG "nf2: ifup: %x\n", card->ifup);

  /* Perform the necessary actions to enable the MAC */
  mac_reset = CNET_RESET_MAC_0 << iface->iface;
  iowrite32(mac_reset, card->ioaddr + CNET_REG_RESET);

  enable = ioread32(card->ioaddr + CNET_REG_ENABLE);
  enable |= (CNET_ENABLE_RX_FIFO_0 | CNET_ENABLE_TX_MAC_0) << 
    iface->iface | 
    CNET_ENABLE_INGRESS_ARBITER | 
    CNET_ENABLE_RX_DMA;
  iowrite32(enable, card->ioaddr + CNET_REG_ENABLE);

  netif_carrier_on(dev);
  netif_wake_queue(dev);

 out:
  up(&card->state_lock);
  if (go_up) {

    u8 buffer[60];
    struct driver_to_netfpga *cmd = (struct driver_to_netfpga *)buffer;

    inject_dev = dev;

    /* allow one reset packet to be sent */
    card->allow_send = 1;
    
    /* reset time values so we can recognize the first sample */
    card->old_clock = 0;
    card->old_time = 0;
    card->avg_clock_drift = 0;
    
    cmd->type = htonl(D2N_TYPE_RESET);
    nf2c_tx_cmd(buffer, sizeof(buffer), dev);
  }

  return err;
}

/*
 * release method - called when the interface is brought down
 *
 * Locking: state_lock - prevent the state variable and the corresponding
 *                       register from getting out of sync
 *
 *          no locking for txbuff variables as other functions that modify
 *          these variables will not execute concurrently if nf2c_release
 *          has been called
 */
static int nf2c_release(struct net_device *dev)
{
  u32 mac_reset;
  u32 enable;
  struct nf2_iface_priv *iface = (struct nf2_iface_priv *)netdev_priv(dev);
  struct nf2_card_priv *card = iface->card;
  int i;

  /* Aquire the mutex for the state variables */
  if (down_interruptible(&card->state_lock))
    return -ERESTARTSYS;

  /* Prevent transmission at the software level */
  netif_carrier_off(dev); 
  netif_stop_queue(dev);
        
  /* Perform the necessary actions to disable the MAC */
  enable = ioread32(card->ioaddr + CNET_REG_ENABLE);
  enable &= ~((CNET_ENABLE_RX_FIFO_0 | CNET_ENABLE_TX_MAC_0) << 
              iface->iface);
  iowrite32(enable, card->ioaddr + CNET_REG_ENABLE);

  mac_reset = CNET_RESET_MAC_0 << iface->iface;
  iowrite32(mac_reset, card->ioaddr + CNET_REG_RESET);

  /* Update the ifup flag */
  card->ifup &= ~(1 << iface->iface);
  PDEBUG(KERN_ALERT "nf2: ifup: %x\n", card->ifup);

  /* Check if any other interfaces are active.
   * If no other interfaces are up uninstall the IRQ handler
   */
  if (!card->ifup)
    {
      // if (!netif_running(bp->dev)) takes care of not rescheduling timer

      /*del_timer_sync(&card->timer);*/
      
      free_irq(card->pdev->irq, card->ndev[0]);
      nf2_hw_reset(card);
      /* No need to call nf2_disable_irq(card) as the reset will
       * disable the interrupts */

      nf2c_clear_dma_flags(card);

      inject_dev = 0;

      /* Return all txbuffs to the free list */
      INIT_LIST_HEAD(&card->free_txbuffs);
      INIT_LIST_HEAD(&card->transmit_txbuffs);
      card->txbuff_building = 0;
      for (i = 0; i < tx_pool_size; i++)
	{
	  struct txbuff *txbuff = &card->txbuff[i];
	  txbuff->mark_pid = 0;
	  kfree_skb(txbuff->mark_skb);
	  txbuff->mark_skb = 0;
	  list_add(&txbuff->list, &card->free_txbuffs);
	}
    }

  PDEBUG(KERN_DFLT_DEBUG "nf2: Queue stopped\n");

  up(&card->state_lock);
  return 0;
}

/*
 * Configuration changes (passed on by ifconfig)
 */
static int nf2c_config(struct net_device *dev, struct ifmap *map)
{
  if (dev->flags & IFF_UP) /* can't act on a running interface */
    return -EBUSY;

  /* Don't allow changing the I/O address */
  if (map->base_addr != dev->base_addr) {
    printk(KERN_WARNING "nf2: Can't change I/O address\n");
    return -EOPNOTSUPP;
  }

  /* Allow changing the IRQ */
  if (map->irq != dev->irq) {
    printk(KERN_WARNING "nf2: Can't change IRQ\n");
    return -EOPNOTSUPP;
  }

  /* ignore other fields */
  return 0;
}

/*static void timer_func(unsigned long v) {
  struct net_device *dev = (struct net_device *) v;
  struct nf2_iface_priv *iface = (struct nf2_iface_priv *)netdev_priv(dev);
  struct nf2_card_priv *card = iface->card;

  if (!netif_running(dev)) {
    // interface is down
    return;
  }
  netif_wake_queue(dev);
  
  printk(KERN_ALERT "TIMER rearming\n");
  mod_timer(&card->timer, jiffies + 250);

}*/

static void print_data(unsigned char* data, int len) {
  int i;
  char line[50];
  char *pos = line;

  for (i = 0; i < len; i++) {
    snprintf(pos, 3, "%02x", *(data++));
    pos += 2;
    if (i) {
      if (i % 16 == 15) {
	*pos = '\0';
	printk("nf2: 0x%04X:  %s\n", i & ~15, line);
	pos = line;
      } else if (i % 8 == 7) {
	*pos = ' ';
	pos++;
	*pos = ' ';
	pos++;
      } else if (i % 2 == 1) {
	*pos = ' ';
	pos++;
      }	  
    }
  }
  if (pos != line) {
    *pos = '\0';
    printk("nf2: 0x%04X:  %s\n", (i-1) & ~15, line);
  }
}

static void print_data_bare(unsigned char* data, int len) {
  //#if 0
      {
      int i;
      char line[50];
      char *pos = line;

      printk("Sending packet:\n");
      for (i = 0; i < len; i++) {
	snprintf(pos, 6, "0x%02x,", *(data++));
	pos += 5;
	if (i) {
	  if (i % 8 == 7) {
	    *pos = '\0';
	    printk("%s\n", line);
	    pos = line;
	  } /*else if (i % 8 == 7) {
	    *pos = ' ';
	    pos++;
	    *pos = ' ';
	    pos++;
	  } else if (i % 2 == 1) {
	    *pos = ' ';
	    pos++;
	    }	*/  
	}
      }
      if (pos != line) {
	*pos = '\0';
	printk("%s\n", line);
      }
      }
      //#endif

}

static void print_pkt(struct sk_buff *skb)
{
  print_data(skb->data, skb->len);
}

#define MAX_PKT_SIZE (1600 - 8)
//static int freeze_cnt = 5;

static inline int remaining_space(struct txbuff *txbuff) {
  /* Use the existing size rounded up to nearest 8 because packets
     need to be aligned. */
  unsigned int used = (txbuff->len + 7) & ~7;
  return MAX_PKT_SIZE - used;
}

static struct txbuff *nf2c_get_free_txbuff(struct net_device *dev, struct nf2_card_priv *card) {
  struct txbuff *txbuff;
  struct list_head *head;
  if (list_empty(&card->free_txbuffs)) {
    /* Shouldn't happen ever! */
    if (printk_ratelimit())
      printk(KERN_ALERT "nf2: no available transmit/receive buffers\n");
    netif_stop_queue(dev);
    return 0;
  }
  /* remove the head off the free list */
  head = card->free_txbuffs.next;
  list_del_init(head);
  txbuff = list_entry(head, struct txbuff, list);
  txbuff->len = 0;
  txbuff->last_hdr = 0;

  /* Stop the queue if the number of free txbuffs drops to 0 */
  if (list_empty(&card->free_txbuffs)) {
    //	    PDEBUG(KERN_DFLT_DEBUG "nf2: stopping queue %d\n", iface->iface);
    //if (printk_ratelimit())
    //  printk(KERN_ALERT "nf2: stopping queue\n");
    netif_stop_queue(dev);
  }

  return txbuff;
}

static int nf2c_tx_cmd(u8 *cmd, uint len, struct net_device *dev)
{
  unsigned long flags;
  struct nf2_iface_priv *iface = netdev_priv(dev);
  struct nf2_card_priv *card = iface->card;
  struct txbuff *txbuff;

  /* Aquire a spinlock for the buffs vbles */
  spin_lock_irqsave(&card->txbuff_lock, flags);

  txbuff = nf2c_get_free_txbuff(dev, card);
  if (txbuff == 0) {
    spin_unlock_irqrestore(&card->txbuff_lock, flags);
    return NETDEV_TX_BUSY; 
  }
  /* There is only one iface for each card now, but set it anyway. */
  txbuff->iface = iface->iface;
  txbuff->len = len;
  memcpy(txbuff->buff, cmd, len);

  /* Put the command packet after the packet at the head of the
     transmit queue.  The head packet is probably being sent to the
     NetFPGA right now. */
  list_add(&txbuff->list, card->transmit_txbuffs.next);

  spin_unlock_irqrestore(&card->txbuff_lock, flags);

  /* Start transmission if possible */
  nf2c_attempt_send(dev);

  return NETDEV_TX_OK;
}


static int nf2c_tx_pkt(u8 *pkt, uint len, uint keep_len, unsigned int clock_hi, unsigned int clock_lo, int is_absolute, struct net_device *dev)
{
  unsigned long flags;
  struct nf2_iface_priv *iface = netdev_priv(dev);
  struct nf2_card_priv *card = iface->card;
  struct txbuff *txbuff;
  struct packet_hdr *hdr;
  int call_send = 0;
  u16 hdr_index;

  if (len > 1514) {
    //printk(KERN_ERR "nf2c_tx_pkt: pkt len %d > 1514 is too large\n", len);
    len = 1514;
  }
  if (keep_len > len) {
    printk(KERN_ERR "nf2c_tx_pkt: pkt keep_len %d is larger than len %d\n", keep_len, len);
    keep_len = len;
  }
  if (len < 60) {
    // printk(KERN_ERR "nf2c_tx_pkt: pkt len %d < 60 is too small\n", len);
    len = 60;
  }

  /* Aquire a spinlock for the buffs vbles */
  spin_lock_irqsave(&card->txbuff_lock, flags);

  txbuff = card->txbuff_building;
  /* Clear txbuff_building and the send_immediately flag. If an
     interrupt wants to flush the packets being built it will set the
     send_immediately flag and queue the building packet for
     transmission when we're ready.
  */
  /*card->txbuff_building = 0;
    card->send_immediately = 0;*/

  if (txbuff != 0) {
    if (remaining_space(txbuff) < sizeof(struct packet_hdr) + keep_len) {
      //      printk(KERN_ALERT "Building pkt is full, adding to transmit queue\n");
      /* Does not fit. Queue this txbuff for sending and get a new one. */
      list_add_tail(&txbuff->list, &card->transmit_txbuffs);
      call_send = 1;

      /* Allow code below to grab the free pkt. */
      card->txbuff_building = txbuff = 0;
    }
  } 

  if (txbuff == 0)
    {
      struct driver_to_netfpga *cmd;
      txbuff = nf2c_get_free_txbuff(dev, card);
      if (txbuff == 0) {
	spin_unlock_irqrestore(&card->txbuff_lock, flags);
	return NETDEV_TX_BUSY; 
      }
      /* There is only one iface for each card now, but set it anyway. */
      txbuff->iface = iface->iface;

      cmd = (struct driver_to_netfpga *)txbuff->buff;
      cmd->type = htonl(D2N_TYPE_PKTS);
      /* start rest of packet after the struct driver_to_netfpga */
      txbuff->len = sizeof(struct driver_to_netfpga); 
    }

  //printk("nf2: injecting into txbuff %p\n", txbuff);

  /* Keep lock while copying data into the packet. 
  spin_unlock_irqrestore(&card->txbuff_lock, flags);*/

  /* put this new header on the next 8-byte aligned address */
  hdr_index = (txbuff->len + 7) & ~7;
  /*printk("Placing header at %d\n", hdr_index);*/

  hdr = (struct packet_hdr*) (txbuff->buff + hdr_index);
  hdr->len = htons(keep_len);
  hdr->wire_len = htons(len);
  memcpy(((char*)hdr) + sizeof(struct packet_hdr), pkt, keep_len);

  hdr->next_pkt = 0;
  /* Set pointer in previous header to this pkt header */
  if (txbuff->last_hdr) {
    short offset = (u8*)hdr - (u8*)(txbuff->last_hdr);
    txbuff->last_hdr->next_pkt = htons(offset);
  }
  txbuff->last_hdr = hdr;
  txbuff->len = hdr_index + sizeof(struct packet_hdr) + keep_len;
  
  //printk("add pkt abs? %hd hi: %u lo: %u\n", is_absolute, clock_hi, clock_lo);

  hdr->clock_hi = htonl(clock_hi);
  hdr->clock_lo = htonl(clock_lo);
  hdr->is_absolute = htons((short)is_absolute);

  /* Grab lock while setting txbuff_building. nf2c_tx may have set the
     send_immediately flag while we were 
   */
  /*spin_lock_irqsave(&card->txbuff_lock, flags);*/
  /* If the packet is nearly full, we should enqueue it for transmission. */  

  /* It may be necessary to send this packet immediately.
     - send_immediately flag was set by an interrupt
     - packet is almost full
     - there is only zero or one packet on the transmit queue
       (if there is only one, it is probably being DMAed right now)
  */
  if (/*card->send_immediately 
	|| */remaining_space(txbuff) - sizeof(struct packet_hdr) < 60
	     || list_empty(&card->transmit_txbuffs)) {
    //      || card->transmit_txbuffs.next == card->transmit_txbuffs.prev) {
    /*printk(KERN_ALERT "nf2: sending building pkt txbuff %p now\n", txbuff);*/
    /*    if (card->send_immediately) {
      printk(KERN_ALERT "tx noticed send_immediately\n");
      }*/

    /*if (remaining_space(txbuff) - sizeof(struct packet_hdr) < 60) {
      printk("Big packet:\n");
      print_data_bare(txbuff->buff, txbuff->len);
      }*/

    card->txbuff_building = 0;
    list_add_tail(&txbuff->list, &card->transmit_txbuffs);
    /*card->send_immediately = 0;*/

    /* Ensure send is called */
    call_send = 1;

    if (list_empty(&card->free_txbuffs)) {
      PDEBUG(KERN_DFLT_DEBUG "nf2: stopping queue %d after space caused send\n", iface->iface);
      //if (printk_ratelimit())
	//printk(KERN_ALERT "nf2: stopping queue %d after space caused send\n", iface->iface);
      netif_stop_queue(dev);
    }
  } else {
    /*printk(KERN_ALERT "nf2: holding onto building pkt\n");*/
    /* Hold onto txbuff for now and possibly add the next packet to
       it */
    card->txbuff_building = txbuff;
  }
  spin_unlock_irqrestore(&card->txbuff_lock, flags);

  /* save the timestamp */
  dev->trans_start = jiffies;

  if (call_send) {
    /* Attempt to send the actual packet */
    nf2c_attempt_send(dev);
  }
  return NETDEV_TX_OK;
}

/*
 * Transmit a packet (called by the kernel)
 */
static int nf2c_tx(struct sk_buff *skb, struct net_device *dev)
{
 //printk("nf2c: packet seen at nf2c_tx \n");
 struct nf2_iface_priv *iface;
 struct nf2_card_priv *card;

 uint keep_len;
 unsigned int clock_hi;
 unsigned int clock_lo;
 int is_absolute;
 int err;

 iface = netdev_priv(dev);
 card = iface->card;

 /* Examine the skb to decide how much to keep. For now just take
    first 60 bytes.  */
 keep_len = skb->len;
 if (keep_len > 200)
 {
   keep_len = 200;
   // printk("nf2c: large keep_len:  %d bytes\n", skb->len);
 }

 /*
    Decide on a relative delay or absolute transmission time. For now
    just use a hardcoded relative delay
  */
 is_absolute = 0;
 clock_hi = 0;
 // Define packet pacing rate:
 double pacing_rate = 950; //Mbps
 int packet_size = 1518;
 double cl = 1000 * packet_size / pacing_rate;
 clock_lo = cl; //12304/8;  //The units is GigE clock ticks(8ns). clock_lo = 1 means 8 ns interval

 err = nf2c_tx_pkt(skb->data, skb->len, keep_len, clock_hi, clock_lo,
is_absolute, dev);
 if (err != NETDEV_TX_OK) {
   printk(KERN_ERR "nf2: inject packet failed. nf2c_tx_pkt result %d\n", err);
 } else {
   //printk(KERN_ERR "nf2: Pkt injected\n");
 }

 /* finished with skb */
 dev_kfree_skb(skb);
 return err;
}

int nf2c_inject_pkt(struct pkt_descr* pkt) {
  struct nf2_iface_priv *iface;
  struct nf2_card_priv *card;
  int len, keep_len;
  char buff[2000];
  char *data = buff + 2; /* Align IP headers when building packet */
  u32 clock_hi;
  u32 clock_lo;
  s64 clock;

  int err;
  int custom_len;

  /*if (printk_ratelimit())*/
    

  /*printk(KERN_ERR "nf2: Inject pkt with sim time %llu\n", pkt->departure_time);*/
  
  if (inject_dev == 0) {
    return -ENXIO;
  }

  iface = netdev_priv(inject_dev);
  card = iface->card;

  keep_len = build_packet(pkt, data, sizeof(buff) - 2);
  if (keep_len < 0) {
    return -ENOMEM;
  }

  len = keep_len + pkt->payload_len;
  custom_len = pkt->custom_data_len;
  if (custom_len > pkt->payload_len) {
    custom_len = pkt->payload_len;
  }

  /* copy custom payload into the packet */
  memcpy(data + keep_len, pkt + 1, custom_len);
  keep_len += custom_len;

  clock = (s64)pkt->departure_time;
  if (pkt->is_absolute) {
    /* convert to absolute clock ticks */
    clock += card->ns2_sim_to_nf_clock;
  }
        
  clock /= 8;
  /*printk(KERN_ERR "nf2: NF2 clock ns %llu\n", clock);*/
  clock_lo = (u32)clock;
  clock_hi = (u32)(clock >> 32);

  err = nf2c_tx_pkt(data, len, keep_len, clock_hi, clock_lo, pkt->is_absolute, inject_dev);
  if (err != 0) {
    printk(KERN_ERR "nf2: inject packet failed. nf2c_tx_pkt result %d\n", err);
  } else {
    //printk(KERN_ERR "nf2: Pkt injected\n");
  }
  return err;
}


/*
  Marks the packets that are still waiting to be sent to the NetFPGA
  card. Once all of the marked packets are sent, the process that set
  the mark will be sent a NetLink message. This allows process that
  are sending packets to avoid overflowing buffers within the driver.
 */
int nf2c_mark(u32 pid, u32 seq) {
  unsigned int size;
  struct sk_buff *skb;
  unsigned long flags;
  struct nlmsghdr *nlh;
  struct nf2_iface_priv *iface;
  struct nf2_card_priv *card;
  struct txbuff *tomark = 0;
  int sendnow = 0;

  if (inject_dev == 0) {
    return -ENXIO;
  }
  
  iface = netdev_priv(inject_dev);
  card = iface->card;
  
  /* Build the skb that eventually will be sent. Doing the work here
     avoids doing it in the interrupt context later. */

  size = NLMSG_SPACE(0);
  skb = alloc_skb(size, GFP_KERNEL);
  if (!skb) {
    printk(KERN_WARNING "nf2: Unable to mark TX packets. Cannot allocate reply skb\n");
    return -ENOMEM;
  }
  nlh = NLMSG_PUT(skb, 0, seq, NF2_NL_MARK_HIT, size - sizeof(*nlh));
  
  //NETLINK_CB(skb).groups = 0; /* not in mcast group */
  NETLINK_CB(skb).pid = 0;      /* from kernel */
  //NETLINK_CB(skb).dst_pid = pid;
  NETLINK_CB(skb).dst_group = 0;  /* unicast */

  /* Message doesn't actually have a payload...*/
  /*data = NLMSG_DATA(nlh);
    memcpy(data, msg, sizeof(*data) + msg->len);*/

  /* Grab the txbuff holding the most recently injected packet. */


  spin_lock_irqsave(&card->txbuff_lock, flags);
    
  /*  The last packet injected is either in txbuff_building or in
      the last txbuff on the the TX queue */
  tomark = card->txbuff_building;
  if (tomark == 0 && !list_empty(&card->transmit_txbuffs)) {
    /*printk(KERN_ERR "nf2: marking a txbuff in the transmit queue\n");*/
    tomark = list_entry(card->transmit_txbuffs.prev, struct txbuff, list);
  } else if (tomark) {
    /*printk(KERN_ERR "nf2: marking txbuff_building\n");*/
  }
  if (tomark) {
    if (tomark->mark_skb) {
      /* this txbuff is already marked. Can't mark it again, so fail */
      spin_unlock_irqrestore(&card->txbuff_lock, flags);
      kfree_skb(skb);
      printk(KERN_WARNING "nf2: txbuff already marked!\n");
      return -EBUSY;
    }
      
    /*printk("nf2: Marked txbuff %p\n", tomark);*/
    tomark->mark_pid = pid;
    tomark->mark_skb = skb;
  } else { 
    /* There are no packets waiting to send. Notify the process
       immediately. */
    sendnow = 1;
    printk(KERN_ERR "nf2: No waiting tx buffs. notifying immediately\n");
  }
  
  spin_unlock_irqrestore(&card->txbuff_lock, flags);

  if (sendnow) {
    int err = netlink_unicast(nf2_nl_sk, skb, pid, MSG_DONTWAIT);
    if (err < 0) {
      printk(KERN_ERR "nf2: Failed to notify process %d immediately. err=%d\n", pid, err);
      return err;
    }
  }

  return 0;


 nlmsg_failure: 
  /* NLMSG_PUT gotos here if the skb is too small. Should never
     happen. */
  kfree_skb(skb);
  return -ENOMEM;
}

static inline void update_time_conversions(struct nf2_card_priv *card) {
  card->ns2_sim_to_nf_clock =  card->ns2_sim_origin_ns - card->nf_clock_origin_ns;

  /*printk("nf2c: update_time card->ns2_sim_to_nf_clock: %lld card->nf_clock_origin_ns: %lld  card->ns2_sim_origin_ns: %lld\n", 
    card->ns2_sim_to_nf_clock, card->nf_clock_origin_ns, card->ns2_sim_origin_ns);*/
}

int nf2c_sim_start(struct NFSimStartMsg* sim) {
  struct nf2_iface_priv *iface;
  struct nf2_card_priv *card;

  if (inject_dev == 0) {
    return -ENXIO;
  }

  iface = netdev_priv(inject_dev);
  card = iface->card;

  card->ns2_sim_origin_ns = timespec_to_ns(&sim->start);
  printk(KERN_ERR "nf2: sim start sec %lu nsec %lu. origin %lld\n", sim->start.tv_sec, sim->start.tv_nsec, card->ns2_sim_origin_ns);
  update_time_conversions(card);
  
  return 0;
}

static int print_it = 1;

/*
 * Send an actual packet
 *
 * Atomic variable dma_tx_lock is used to prevent multiple packets from
 * being sent simultaneously (the hardware can only transfer one at once)
 *
 * Note: The txbuff lock is not used as an incorrect read of free_txbuffs is
 *       not fatal (this function will be called again).
 */
static int nf2c_attempt_send(struct net_device *dev)
{
  int err = 0;
  int dma_len;
  unsigned long flags;
  struct nf2_iface_priv *iface = netdev_priv(dev);
  struct nf2_card_priv *card = iface->card;
  struct txbuff *txbuff;

  /*printk(KERN_ERR "nf2: attempt send\n");*/

  /* Check if a DMA transfer is in progress and record the fact that
   * we have started a transfer
   */
  if (!start_tx_dma(card))
    {
      //printk("attempt send: dma already in progess\n");
      return 1;
    }

  spin_lock_irqsave(&card->txbuff_lock, flags);

  /*if (list_empty(&card->transmit_txbuffs)) {
    printk(KERN_ERR "nf2: attempt send, but transmit_txbuffs is empty\n");
  } 
  if (card->allow_send == 0) {
    printk(KERN_ERR "nf2: attempt send, but no allow_send\n");
    }*/

  /* Check if there's something to send */
  if (list_empty(&card->transmit_txbuffs) || card->allow_send == 0)
    {
      //printk("attempt send blocked: list_empty(&card->transmit_txbuffs)== %d || card->allow_send == %d\n", list_empty(&card->transmit_txbuffs), card->allow_send);
      finish_tx_dma(card);
      spin_unlock_irqrestore(&card->txbuff_lock, flags);
      return 1;
    }

  dev->trans_start = jiffies;
  
  txbuff = list_first_entry(&card->transmit_txbuffs, struct txbuff, list);
  
  /*printk("nf2: start sending txbuff %p\n", txbuff);*/
  spin_unlock_irqrestore(&card->txbuff_lock, flags);
  
  card->dma_tx_len = dma_len = txbuff->len;


  
  /*  if (print_it) {
      print_data_bare(txbuff->buff, dma_len);
    print_it = 0;
    }*/

  /* Map the buffer into DMA space */
  card->dma_tx_addr = pci_map_single(card->pdev, 
                                     txbuff->buff, dma_len, PCI_DMA_TODEVICE);
  /* Start the transfer */
  iowrite32(card->dma_tx_addr, 
            card->ioaddr + CPCI_REG_DMA_E_ADDR);
  iowrite32(dma_len,
            card->ioaddr + CPCI_REG_DMA_E_SIZE);
  iowrite32(NF2_SET_DMA_CTRL_MAC(txbuff->iface) | DMA_CTRL_OWNER,
            card->ioaddr + CPCI_REG_DMA_E_CTRL);
        
  return err;
}

static int nf2c_flush_transmit(struct net_device *dev, struct nf2_card_priv *card) {
  if (list_empty(&card->transmit_txbuffs)) {
    struct txbuff *building = card->txbuff_building;
    if (building) {
      //printk(KERN_ALERT "Flush building packet\n");
      list_add_tail(&building->list, &card->transmit_txbuffs);
      card->txbuff_building = 0;
      if (list_empty(&card->free_txbuffs)) {
	PDEBUG(KERN_DFLT_DEBUG "nf2: stopping queue after transmit flush\n");
	//if (printk_ratelimit())
	  //printk(KERN_ALERT "nf2: stopping queue after transmit flush\n");
	netif_stop_queue(dev);
      }
      return 1;
    } else {
      //printk(KERN_ALERT "Mark to flush building packet\n");
      /* nf2c_tx may be building a packet now. tell it to send it right away. */
      /*card->send_immediately = 1;*/
      return 0;
    }
  }
  return 0;
}

/*
 * Receive a packet: retrieve, encapsulate and pass over to upper levels
 *
 * Note: This is called from the interrupt handler. netif_rx schedules
 * the skb to be delivered to the kernel later (ie. bottom-half delivery).
 */
static void nf2c_rx(struct net_device *dev, struct nf2_packet *pkt)
{
  unsigned long flags;
  int call_send;
  /*struct sk_buff *skb;*/
  struct nf2_iface_priv *iface = netdev_priv(dev);
  struct nf2_card_priv *card = iface->card;
  struct netfpga_to_driver *msg = (struct netfpga_to_driver*)pkt->data;
  int msg_type = ntohl(msg->type);

  /*static s64 old_clock;
    static s64 old_time;*/
  s64 clocktime;
  s64 realtime;


  //struct timeval tv;

  //do_gettimeofday(&tv);
  realtime = timespec_to_ns(&card->pkt_avail_time);

  if (pkt->len < sizeof(struct netfpga_to_driver)) {
    //if (printk_ratelimit())
      printk("nf2c: ERROR nf2c_rx got pkt too small: %d bytes\n", pkt->len);
      //print_data_bare(pkt->data, pkt->len);
    goto out;
  }

  //print_data(pkt->data, pkt->len);

  if (msg_type != N2D_TYPE_RESET && msg_type != N2D_TYPE_QUOTA) {
    if (printk_ratelimit())
      printk("nf2c: ERROR nf2c_rx got unknown pkt type: %d\n", msg_type);

    goto out;
  }

  clocktime = ((u64)ntohl(msg->time_hi) << 32) | (u64)ntohl(msg->time_lo);

  if (0 && card->old_time != 0) {
    s64 clock_drift;
    s64 clock_delta = clocktime - card->old_clock;
    s64 time_delta = realtime - card->old_time;
    s32 remainder;
    s32 difference = (s32)(clock_delta - time_delta/8);
    printk("\n");
    if (difference == 0) {
      // drift may as well be infinite
      clock_drift = 0x7fffffffffffffffLL;
    } else {
      //clock_drift = (s32)clock_delta / clock_drift;
      //clock_delta =  536870912;
      clock_drift = clock_delta;
      remainder = do_div(clock_drift, abs(difference));
      if (difference < 0) clock_drift *= -1;
      printk("%lld / %d = %lld\n", clock_delta, difference, clock_drift);
    }
    
    printk("nf2c: nfclock moved %lld ticks expected %lld ticks (diff %d)in cpu time moved %lld ns.\n", 
	   clock_delta, time_delta/8, difference , time_delta); 
    printk("nf2c: Every %lld ticks the netfpga drifted by %dns (rem: %d)\n", clock_drift, 
	   ( clock_drift < 0? -8 :( clock_drift > 0?8 : 0)), remainder);

    if (card->avg_clock_drift == 0) {
      card->avg_clock_drift = clock_drift;
      printk("nf2c: clock drift %lld.\n", clock_drift);
    } else {
      /* keep running average */
      card->avg_clock_drift = (card->avg_clock_drift / 8) * 7 + clock_drift / 8;

      printk("nf2c: clock drift %lld. Averaged %lld\n", clock_drift, card->avg_clock_drift);
    }
  } else {
    /*printk("nf2c: ignoring first time sample\n");*/
  }
  card->old_clock = clocktime;
  card->old_time = realtime;

  /* shift extra 3 to account for 1 clock = 8 ns */
  clocktime *= 8; 

  spin_lock_irqsave(&card->txbuff_lock, flags);

  card->nf_clock_origin_ns = realtime - clocktime;
  update_time_conversions(card);

  call_send = (card->allow_send == 0);
  if (msg_type == N2D_TYPE_RESET) {
    card->allow_send = ntohl(msg->pkt_quota);
    printk("nf2c: nf2c_rx got rst pkt: time_hi %u time_lo %u time now is %lld\n", ntohl(msg->time_hi), ntohl(msg->time_lo), realtime);
  } else {
    //printk("nf2c: Increased quota from %d to %d\n", card->allow_send, card->allow_send + ntohl(msg->pkt_quota));
    card->allow_send += ntohl(msg->pkt_quota);
  }
  call_send &= (card->allow_send != 0);


  if (call_send) {
    nf2c_flush_transmit(dev, card);
  }
  spin_unlock_irqrestore(&card->txbuff_lock, flags);

  /*printk("nf2c: nf2c_rx got pkt type: %d quota now %d\n", msg_type, card->allow_send);*/

  if (call_send) {
    //    printk("nf2c_rx restarting sending\n");
    nf2c_attempt_send(dev);
  }
  return;

#if 0
  /*
   * The packet has been retrieved from the transmission
   * medium. Build an skb around it, so upper layers can handle it
   */
  skb = dev_alloc_skb(pkt->len + 2);
  if (!skb) {
    if (printk_ratelimit())
      printk(KERN_NOTICE "nf2 rx: low on mem - packet dropped\n");
    iface->stats.rx_dropped++;
    goto out;
  }

  skb_reserve(skb, 2); /* align IP on 16B boundary */  
  memcpy(skb_put(skb, pkt->len), pkt->data, pkt->len);

  /* Write metadata, and then pass to the receive level */
  skb->dev = dev;
  skb->protocol = eth_type_trans(skb, dev);
  skb->ip_summed = CHECKSUM_NONE; /* Check the checksum */
  iface->stats.rx_packets++;
  iface->stats.rx_bytes += pkt->len;
  netif_rx(skb);
#endif
 out:
  return;
}
    
/*
 * Clear the DMA flags that record that a DMA transfer is in progress
 */
static void nf2c_clear_dma_flags(struct nf2_card_priv *card)
{
  /*  unsigned long flags;
      unsigned int ifnum;*/

  PDEBUG(KERN_DFLT_DEBUG "nf2: clearing dma flags\n");

  /* Clear the dma_rd_in_progress flag */
  if (dma_rd_in_progress(card))
    {
      pci_unmap_single(card->pdev, card->dma_rx_addr,
                       MAX_DMA_LEN, 
                       PCI_DMA_FROMDEVICE);

      finish_rd_dma(card);
    }

  /* Clear the dma_tx_in_progress flag
   * Note: also frees the skb */
  if (dma_tx_in_progress(card))
    {
      pci_unmap_single(card->pdev, card->dma_tx_addr,
                       card->dma_tx_len, 
                       PCI_DMA_TODEVICE);

      finish_tx_dma(card);
    }
}

/*
 * Handle ioctl calls
 */
static int nf2c_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
  struct nf2reg reg;

  switch(cmd) {
    /* Read a register */
  case SIOCREGREAD:
    if (copy_from_user(&reg, rq->ifr_data, 
		       sizeof(struct nf2reg)))
      {
        printk(KERN_ERR "nf2: Unable to copy data from user space\n");
        return -EFAULT;
      }
    nf2k_reg_read(dev, reg.reg, &reg.val);
    if (copy_to_user(rq->ifr_data, &reg, 
		     sizeof(struct nf2reg)))
      {
        printk(KERN_ERR "nf2: Unable to copy data to user space\n");
        return -EFAULT;
      }
    return 0;
                          
    /* Write a register */
  case SIOCREGWRITE:
    if (copy_from_user(&reg, rq->ifr_data, 
		       sizeof(struct nf2reg)))
      {
        printk(KERN_ERR "nf2: Unable to copy data from user space\n");
        return -EFAULT;
      }
    nf2k_reg_write(dev, reg.reg, &(reg.val));
    return 0;

  default:
    return -EOPNOTSUPP;
  }

  /* Should never reach here, but anyway :) */
  return -EOPNOTSUPP;
}

/*
 * handle register writes
 */
int nf2k_reg_read(struct net_device *dev, unsigned int addr, void* data) {
  struct nf2_iface_priv *iface = netdev_priv(dev);
  struct nf2_card_priv *card = iface->card;
  void *from_addr = card->ioaddr + addr;

  if (!data) {
    printk(KERN_WARNING "nf2:  WARNING: register read with data address 0 requested\n");
    return 1;
  }

  if (!card->ioaddr) {
    printk(KERN_WARNING "nf2:  WARNING: card IO address is NULL during register read\n");
    return 1;
  }

  if (addr >= pci_resource_len(card->pdev, 0)) {
    printk(KERN_ERR "nf2:  ERROR: address exceeds bounds (0x%lx) during register read\n",
           (long unsigned int)pci_resource_len(card->pdev, 0) - 1);
    return 1;
  }

  memcpy_fromio(data, from_addr, sizeof(uint32_t));
  
  return 0;
}
EXPORT_SYMBOL(nf2k_reg_read);

/*
 * handle register writes
 */
int nf2k_reg_write(struct net_device *dev, unsigned int addr, void* data) {
  struct nf2_iface_priv *iface = netdev_priv(dev);
  struct nf2_card_priv *card = iface->card;
  void *to_addr = card->ioaddr + addr;
  
  if (!data) {
    printk(KERN_WARNING "nf2:  WARNING: register write with data address 0 requested\n");
    return 1;
  }
 
  if (!card->ioaddr) {
    printk(KERN_WARNING "nf2:  WARNING: card IO address is NULL during register write\n");
    return 1;
  }

  if (addr >= pci_resource_len(card->pdev, 0)) {
    printk(KERN_ERR "nf2:  ERROR: address exceeds bounds (0x%lx) during register write\n",
           (long unsigned int)pci_resource_len(card->pdev, 0) - 1);
    return 1;
  }

  memcpy_toio(to_addr, data, sizeof(uint32_t));

  return 0;
}
EXPORT_SYMBOL(nf2k_reg_write);

/*
 * Return statistics to the caller
 */
static struct net_device_stats *nf2c_stats(struct net_device *dev)
{
  struct nf2_iface_priv *iface = netdev_priv(dev);
  return &iface->stats;
}

/*
 * Set the MAC address of the interface
 */
static int nf2c_set_mac_address(struct net_device *dev, void *a)
{
  struct sockaddr *addr = (struct sockaddr *) a;

  /* Verify that the address is valid */
  if (!is_valid_ether_addr(addr->sa_data))
    return -EADDRNOTAVAIL;

  /* Copy the MAC address into the dev */
  memcpy(dev->dev_addr, addr->sa_data, dev->addr_len);

  return 0;
}

/*
 * Deal with a transmit timeout.
 *
 * FIXME: Consider adding locking to protect the enable register
 * (what if ifup is modified while this function is executing?)
 * Also What happens if this is executed while the ISR is running?
 * We need a lock on the intmask reg.
 */
static void nf2c_tx_timeout (struct net_device *dev)
{
  struct nf2_iface_priv *iface = netdev_priv(dev);
  struct nf2_card_priv *card = iface->card;
  u32 enable, intmask;

  printk(KERN_ALERT "nf2: Transmit timeout on %s at %lu, latency %lu\n", 
         dev->name, jiffies, jiffies - dev->trans_start);

  iface->stats.tx_errors++;

  /* Read the current status of enable and interrupt mask registers */
  enable = ioread32(card->ioaddr + CNET_REG_ENABLE);
  intmask = ioread32(card->ioaddr + CPCI_REG_INTERRUPT_MASK);

  /* Reset the card! */
  //nf2_hw_reset(card);

  /* Write the status of the enable registers */
  iowrite32(enable, card->ioaddr + CNET_REG_ENABLE);
  iowrite32(intmask, card->ioaddr + CPCI_REG_INTERRUPT_MASK);

  /* Clear the DMA flags */
  //nf2c_clear_dma_flags(card);

  /* Call the send function if there's packets to send */
  nf2c_attempt_send(dev);

  /* Don't explicitly wake queue. Retrying the send will make it wake naturally. */
  /* actually, do force it to be safe. throttling is tricky! */
  /*netif_wake_queue(dev);*/
}

static inline void nf2_restart_dma(struct net_device *netdev) {
  struct nf2_iface_priv *iface = netdev_priv(netdev);
  struct nf2_card_priv *card = iface->card;
  nf2_reset_cpci(card);
  nf2c_clear_dma_flags(card);
  nf2c_attempt_send(netdev);
}


static int notify_of_mark(struct txbuff *txbuff) {
  int err;

  /*printk("nf2: notify_of_mark txbuff %p\n", txbuff);*/

  if (txbuff->mark_skb == 0) {
    return 0;
  }
  
  err = netlink_unicast(nf2_nl_sk, txbuff->mark_skb, txbuff->mark_pid, MSG_DONTWAIT);

  if (err < 0) {
    printk(KERN_WARNING "nf2: Notifying process %u of mark failed. err %d\n", txbuff->mark_pid, err);
  } else {
    /*printk(KERN_ERR "nf2: Notifying process %u of mark. Sent %d bytes.\n", txbuff->mark_pid, err);*/
    err = 0;
  }

  txbuff->mark_pid = 0;
  txbuff->mark_skb = 0;
  return err;
}

/*
 * Handle an interrupt
 *
 * Note: Keep this as short as possible!
 */
static irqreturn_t nf2c_intr(int irq, void *dev_id
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
				  , struct pt_regs * regs
#endif
				  )
{
  struct net_device *netdev = dev_id;
  struct nf2_iface_priv *iface = netdev_priv(netdev);
  struct nf2_card_priv *card = iface->card;
  /*struct nf2_iface_priv *tx_iface;*/
  unsigned long flags;
  unsigned int ifnum;
  u32 err;
  u32 ctrl;
  u32 status;
  u32 prog_status;
  u32 int_mask;
  u32 cnet_err;


  struct timespec int_time;
  getnstimeofday(&int_time);

  /* get the interrupt mask */
  int_mask = ioread32(card->ioaddr + CPCI_REG_INTERRUPT_MASK);
  PDEBUG(KERN_DFLT_DEBUG "nf2: intr mask vector: 0x%08x\n", int_mask);

  /* disable interrupts so we don't get race conditions */
  nf2_disable_irq(card);
  smp_mb();

  /* Grab the interrupt status */
  status = ioread32(card->ioaddr + CPCI_REG_INTERRUPT_STATUS);
  PDEBUG(KERN_DFLT_DEBUG "nf2: intr status vector: 0x%08x\n", status);

  /* only consider bits that are not masked plus INT_PKT_AVAIL*/
  status &= ~(int_mask & ~INT_PKT_AVAIL); 
 
  /* Check if the interrrupt was generated by us */
  if (status)
    {
      PDEBUG(KERN_DFLT_DEBUG "nf2: intr to be handled: 0x%08x\n", status);
           
      /* Handle packet RX complete 
       * Note: don't care about the rx pool here as the packet is copied to an skb
       * immediately so there is no need to have multiple packets in the rx pool
       */
      if (status & INT_DMA_RX_COMPLETE)
        {
          PDEBUG(KERN_DFLT_DEBUG "nf2: intr: INT_DMA_RX_COMPLETE\n");

	  if (dma_rd_in_progress(card)) {
               
	    pci_unmap_single(card->pdev, card->dma_rx_addr,
			     MAX_DMA_LEN, 
			     PCI_DMA_FROMDEVICE);
               
	    card->wr_pool->len = ioread32(card->ioaddr + CPCI_REG_DMA_I_SIZE);
               
	    ctrl = ioread32(card->ioaddr + CPCI_REG_DMA_I_CTRL);
               
	    finish_rd_dma(card);
               
	    if ((ctrl & 0x300) >> 8 != 0) {
	      if (printk_ratelimit())
		printk(KERN_ERR "nf2: intr: INT_DMA_RX_COMPLETE, CPCI_REG_DMA_I_CTRL returned %u which is dev %u\n", ctrl, (ctrl & 0x300) >> 8);
	    }
	    nf2c_rx(card->ndev[0], card->wr_pool);
	  }
	  /* reenable PKT_AVAIL interrupts */
	  int_mask &= ~INT_PKT_AVAIL;
        }
           
      /* Handle packet TX complete */
      if (status & INT_DMA_TX_COMPLETE)
        {
	  //int more_pkts = 0;
          PDEBUG(KERN_DFLT_DEBUG "nf2: intr: INT_DMA_TX_COMPLETE\n");
          
          /* make sure there is a tx dma in progress */
          if(dma_tx_in_progress(card)) {
	    struct txbuff *txbuff;
            pci_unmap_single(card->pdev, card->dma_tx_addr,
                             card->dma_tx_len,
                             PCI_DMA_TODEVICE);


	    /* TODO: Determine if the following statistics changes can be used */
	    /* Establish which iface we were sending the packet on */
	    /*            ifnum = card->txbuff[card->rd_txbuff].iface;
			  tx_iface = netdev_priv(card->ndev[ifnum]);*/

            /* Update the statistics */
	    /*            tx_iface->stats.tx_packets++;
			  tx_iface->stats.tx_bytes += card->txbuff[card->rd_txbuff].skb->len;*/
               
            /* Aquire a spinlock for the buffs vbles */
            spin_lock_irqsave(&card->txbuff_lock, flags);

	    if (list_empty(&card->transmit_txbuffs)) {
	      printk(KERN_ERR "nf2: intr: INT_DMA_TX_COMPLETE, but no tx packets waiting!\n");
	    } else {
	      card->allow_send--;

	      //printk("nf2: Finished TX. card->allow_send = %d\n", card->allow_send);

	      if (card->allow_send < 0){
		card->allow_send = 0;
	      }
	      /*if (card->allow_send == 0){
		printk("ALLOW_SEND = 0\n");
		}*/
	    txbuff = list_first_entry(&card->transmit_txbuffs, struct txbuff, list);

	    /* If the txbuff was marked. Send a netlink message saying
	       it's been sent. */
	    notify_of_mark(txbuff);

	    ifnum = txbuff->iface;
	    list_move(&txbuff->list, &card->free_txbuffs);
	    
	    nf2c_flush_transmit(netdev, card);
	    /* Send again if there are more packets waiting */
	    //more_pkts = !list_empty(&card->transmit_txbuffs) && card->allow_send;
	    finish_tx_dma(card);
               
            /* Re-enable the queues if necessary */
	    netif_wake_queue(card->ndev[ifnum]);
	    }

            spin_unlock_irqrestore(&card->txbuff_lock, flags);
               
            /* Call the send function in case there are other packets to send */
	    nf2c_attempt_send(netdev);
          }
        }
           
      /* Handle PHY interrupts */
      if (status & INT_PHY_INTERRUPT)
        {
          PDEBUG(KERN_DFLT_DEBUG "nf2: intr: INT_PHY_INTERRUPT\n");
               
          int_mask |= INT_PHY_INTERRUPT;
          printk(KERN_ALERT "nf2: Phy Interrrupt - masked off.\n");
        }
           
      /* Handle a packet RX notification 
       *
       * Should not need to worry about this interrupt being asserted while
       * a DMA transfer is in progress as the hardware should prevent this.
       *
       * ie. no need to do: !card->dma_rx_in_progress
       */
      if (status & INT_PKT_AVAIL)
        {
          PDEBUG(KERN_DFLT_DEBUG "nf2: intr: INT_PKT_AVAIL\n");

	  if ( start_rd_dma(card) ) 
	    {
	      //	      PDEBUG(KERN_DFLT_DEBUG "nf2: dma_rx_in_progress is %d\n", atomic_read(&card->dma_rx_in_progress));
	      card->dma_rx_addr = pci_map_single(card->pdev,
						 card->wr_pool->data,
						 MAX_DMA_LEN,
						 PCI_DMA_FROMDEVICE);
	      /* Start the transfer */
	      iowrite32(card->dma_rx_addr, 
			card->ioaddr + CPCI_REG_DMA_I_ADDR);
	      iowrite32(DMA_CTRL_OWNER,
			card->ioaddr + CPCI_REG_DMA_I_CTRL);

	      /* Store the time that the interrupt occurred to observe
		 clock drift in the NetFPGA */
	      card->pkt_avail_time = int_time;
	    }
	  else 
	    {
	      PDEBUG(KERN_DFLT_DEBUG "nf2: received interrupt for new rx packet avail while still\n");
	      PDEBUG(KERN_DFLT_DEBUG "     processing last packet - TODO for performance, that's ok for now.\n");
	    }
	  /* mask off subsequent PKT_AVAIL interrupts */
	  int_mask |= INT_PKT_AVAIL;
        }
           
      /* The cnet is asserting an error */
      if (status & INT_CNET_ERROR)
        {
          cnet_err = ioread32(card->ioaddr + CNET_REG_ERROR);
          int_mask |= INT_CNET_ERROR;
               
          printk(KERN_ERR "nf2: CNET error (CNET reg 0x%x : %08x).\n", CNET_REG_ERROR, cnet_err);
        }
           
      /* CNET read timeout */
      if (status & INT_CNET_READ_TIMEOUT)
        {
          printk(KERN_ERR "nf2: CNET read timeout occurred\n");
        }
           
      /* Programming error occured */
      if (status & INT_PROG_ERROR)
        {
          printk(KERN_ERR "nf2: CNET programming error\n");
        }
           
      /* DMA transfer error */
      if (status & INT_DMA_TRANSFER_ERROR)
        {
          err = ioread32(card->ioaddr + CPCI_REG_ERROR);
          printk(KERN_ERR "nf2: DMA transfer error: 0x%08x\n", err);
	  if (err & ERR_DMA_RETRY_CNT_EXPIRED) printk(KERN_ERR "\t ERR_DMA_RETRY_CNT_EXPIRED - Too many unsuccessful retries.\n");
          if (err & ERR_DMA_TIMEOUT) printk(KERN_ERR "\t ERR_DMA_TIMEOUT - DMA transfer took too long.\n");

          /* Check the programming status */
          prog_status = ioread32(card->ioaddr + CPCI_REG_PROG_STATUS);
          if (prog_status & ~PROG_DONE)
            {
              printk(KERN_ERR "\t Note: Virtex is not currently programmed.\n");
            }
              
	  nf2_restart_dma(netdev);
        }
           
      /* DMA setup error */
      if (status & INT_DMA_SETUP_ERROR)
        {
          err = ioread32(card->ioaddr + CPCI_REG_ERROR);
	  printk(KERN_ERR "nf2: DMA setup error: 0x%08x\n", err);
          if (err & ERR_DMA_RD_MAC_ERROR) printk(KERN_ERR "\t ERR_DMA_RD_MAC_ERROR - No data to read from MAC.\n");
          if (err & ERR_DMA_WR_MAC_ERROR) printk(KERN_ERR "\t ERR_DMA_WR_MAC_ERROR - MAC Tx is full.\n");
          if (err & ERR_DMA_WR_ADDR_ERROR) printk(KERN_ERR "\t ERR_DMA_WR_ADDR_ERROR - not on word boundary.\n");
          if (err & ERR_DMA_RD_ADDR_ERROR) printk(KERN_ERR "\t ERR_DMA_RD_ADDR_ERROR - not on word boundary.\n");
          if (err & ERR_DMA_WR_SIZE_ERROR) printk(KERN_ERR "\t ERR_DMA_WR_SIZE_ERROR - egress pkt too big (>2kB)\n");
          if (err & ERR_DMA_RD_SIZE_ERROR) printk(KERN_ERR "\t ERR_DMA_RD_SIZE_ERROR - ingress pkt too big (>2kB)\n");
          if (err & ERR_DMA_BUF_OVERFLOW) printk(KERN_ERR "\t ERR_DMA_BUF_OVERFLOW - CPCI internal buffer overflow\n");
               
	  nf2_restart_dma(netdev);
        }
           
      /* DMA fatal error */
      if (status & INT_DMA_FATAL_ERROR)
        {
          err = ioread32(card->ioaddr + CPCI_REG_ERROR);
          printk(KERN_ERR "nf2: DMA fatal error: 0x%08x\n", err);
          
	  nf2_restart_dma(netdev);
        }
           
      /* Check for unknown errors */
      if (status & INT_UNKNOWN)
        {
          printk(KERN_ERR "nf2: Unknown interrupt(s): 0x%08x\n", status);
        }
           
    }

  PDEBUG(KERN_DFLT_DEBUG "nf2: Reenabling interrupts: mask is 0x%08x\n", int_mask);
  /* Rewrite the interrupt mask including any changes */
  iowrite32(int_mask, card->ioaddr + CPCI_REG_INTERRUPT_MASK);

  if(status)
    {
      return IRQ_HANDLED;
    }
  else
    {
      return IRQ_NONE;
    }
}

/*
 * The init function (sometimes called probe).
 * It is invoked by register_netdev()
 */
static void nf2c_init(struct net_device *dev)
{
  struct nf2_iface_priv *iface;

  ether_setup(dev); /* assign some of the fields */

  dev->open            = nf2c_open;
  dev->stop            = nf2c_release;
  dev->set_config      = nf2c_config;
  dev->hard_start_xmit = nf2c_tx;
  dev->do_ioctl        = nf2c_ioctl;
  dev->get_stats       = nf2c_stats;
  dev->tx_timeout      = nf2c_tx_timeout;
  dev->watchdog_timeo  = timeout;
  dev->set_mac_address = nf2c_set_mac_address;
  dev->mtu             = MTU;

  iface = netdev_priv(dev);
  memset(iface, 0, sizeof(struct nf2_iface_priv));
}

/*
 * Create the pool of buffers for DMA transfers
 *
 * Only one is used by the control card
 */
static int nf2c_create_pool(struct nf2_card_priv *card)
{
  card->wr_pool = kmalloc (sizeof (struct nf2_packet), GFP_KERNEL);
  if (card->wr_pool == NULL)
    {
      printk (KERN_NOTICE "nf2: Out of memory while allocating packet pool\n");
      return -ENOMEM;
    }
  return 0;
}

/*
 * Destroy the pool of buffers available for DMA transfers
 */
static void nf2c_destroy_pool(struct nf2_card_priv *card)
{
  kfree(card->wr_pool);
}


/* Process one packet of messages. */
/*static inline int nfnetlink_rcv_skb(struct sk_buff *skb)
{
	int err;
	struct nlmsghdr *nlh;

	while (skb->len >= NLMSG_SPACE(0)) {
		u32 rlen;

		nlh = (struct nlmsghdr *)skb->data;
		if (!NLMSG_OK(nlh))
			return 0;
		rlen = NLMSG_ALIGN(nlh->nlmsg_len);
		if (rlen > skb->len)
			rlen = skb->len;
		if (nfnetlink_rcv_msg(skb, nlh, &err)) {
			if (!err)
				return -1;
			netlink_ack(skb, nlh, err);
		} else
			if (nlh->nlmsg_flags & NLM_F_ACK)
				netlink_ack(skb, nlh, 0);
		skb_pull(skb, rlen);
	}

	return 0;
}*/

/*
 * nf2_probe:
 * Identifies the card, performs initialization and sets up the necessary
 * data structures.
 */
int nf2c_probe(struct pci_dev *pdev, const struct pci_device_id *id, struct nf2_card_priv *card)
{
  int ret = -ENODEV;

  struct net_device *netdev;
  struct nf2_iface_priv *iface;

  int i;
  int result;

  int err;

  char *devname = "nf2c%d";

  /* Create the rx pool */
  if ((err = nf2c_create_pool(card)) != 0)
    {
      ret = err;
      goto err_out_free_none;
    }

  /* Create the tx pool */
  PDEBUG(KERN_DFLT_DEBUG "nf2: kmallocing memory for tx buffers\n");
  card->txbuff = kmalloc(sizeof(struct txbuff) * tx_pool_size, GFP_KERNEL | __GFP_DMA);
  if (card->txbuff == NULL)
    {
      printk(KERN_ERR "nf2: Could not allocate nf2 user card tx buffers.\n");
      ret = -ENOMEM;
      goto err_out_free_rx_pool;
    }
  /* Zero out the buffers */
  memset(card->txbuff, 0, sizeof(struct txbuff) * tx_pool_size);

  /* Add all of the buffers to the free list */
  for (i = 0; i < tx_pool_size; i++)
    {
      list_add(&card->txbuff[i].list, &card->free_txbuffs);
    }

  /* Set up the network device... */
  for (i = 0; i < MAX_IFACE; i++)
    {
      netdev = card->ndev[i] = alloc_netdev(
                                            sizeof(struct nf2_iface_priv),
                                            devname, nf2c_init);
      if (netdev == NULL)
        {
          printk(KERN_ERR "nf2: Could not allocate ethernet device.\n");

          ret = -ENOMEM;
          goto err_out_free_etherdev;
        }
      netdev->irq = pdev->irq;
      iface = (struct nf2_iface_priv*)netdev_priv(netdev);

      iface->card = card;
      iface->iface = i;

      /* 
       * Assign the hardware address of the board: use "\0NF2Cx", 
       * where x is the device number.
       */
      memcpy(netdev->dev_addr, "\0NF2C0", ETH_ALEN);
      netdev->dev_addr[ETH_ALEN - 1] = devnum++;
    }

  /* Register the network devices */
  PDEBUG(KERN_DFLT_DEBUG "nf2: registering network devices\n");
  for (i = 0; i < MAX_IFACE; i++)
    {
      if (card->ndev[i])
        {
          if ((result = register_netdev(card->ndev[i])))
            {
              printk(KERN_ERR "nf2: error %i registering device \"%s\"\n",
                     result, 
                     card->ndev[i]->name);
            }
          else
            {
              PDEBUG(KERN_ALERT "nf2: registered netdev %d\n", i);
            }
        }
    }


  {
    /* Initialize the origin times to now */
    struct timespec ts;
    getnstimeofday(&ts);
    card->ns2_sim_origin_ns = timespec_to_ns(&ts);
    card->nf_clock_origin_ns = card->ns2_sim_origin_ns;
    update_time_conversions(card);
  }

  /* If we make it here then everything has succeeded */
  return 0;

  /* Error handling points. Undo any resource allocation etc */
 err_out_free_etherdev:
  for (i = 0; i < MAX_IFACE; i++)
    if (card->ndev[i])
      free_netdev(card->ndev[i]);
  if (card->txbuff != NULL) 
      kfree(card->txbuff);

 err_out_free_rx_pool:
  nf2c_destroy_pool(card);

 err_out_free_none:
  return ret;
}

/*
 * Called when the device driver is unloaded
 */
void nf2c_remove(struct pci_dev *pdev, struct nf2_card_priv *card)
{
  int i;

  /* Release the ethernet data structures */
  for (i = 0; i < MAX_IFACE; i++)
    {
      if (card->ndev[i])
        {
          unregister_netdev(card->ndev[i]);
          free_netdev(card->ndev[i]);
        }
    }

  /* Destroy the txbuffs */
  if (card->txbuff != NULL) 
    {
      kfree(card->txbuff);
    }

  /* Destroy the rx pool */
  nf2c_destroy_pool(card);
}

