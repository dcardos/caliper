/* ****************************************************************************
 * $Id: nf2main.c 1798 2007-05-21 09:28:49Z grg $
 *
 * Module: nf2main.c
 * Project: NetFPGA 2 Linux Kernel Driver
 * Description: Main source file for kernel driver
 *		Code for control and user cards is in separate files.
 *
 * Change history: 9/1/05 - Semi-functional driver :-)
 * 		   9/2/05 - Split driver into multiple modules
 * 		   	    (user and control mode in separate modules)
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

#include <linux/in.h>
#include <linux/netdevice.h>   /* struct device, and other headers */
#include <linux/etherdevice.h> /* eth_type_trans */
#include <linux/if_ether.h>
#include <linux/moduleparam.h>
#include <linux/stat.h>
#include <linux/netlink.h>
#include <net/sock.h>

#include <linux/timer.h>

#include <asm/uaccess.h>
#include <asm/semaphore.h>

#include "../common/nf2.h"
#include "nf2kernel.h"
#include "checksum.h"
#include "nf_netlink.h"

MODULE_AUTHOR("Maintainer: Glen Gibb <grg@stanford.edu>");
MODULE_DESCRIPTION("Stanford NetFPGA 2 Driver");
MODULE_LICENSE("GPL");


unsigned int pkt_delay = 125000000/4;
module_param(pkt_delay, uint, S_IRUGO | S_IWUSR | S_IWGRP);

/*
 * Timeout parameter
 */
int timeout = NF2_TIMEOUT;
module_param(timeout, int, S_IRUGO);

/*
 * Size of receive buffer pool
 */
int rx_pool_size = NUM_RX_BUFFS;
module_param(rx_pool_size, int, S_IRUGO);

/*
 * Size of transmit buffer pool
 */
int tx_pool_size = NUM_TX_BUFFS;
module_param(tx_pool_size, int, S_IRUGO);

/*
 * Major and minor device numbers. Defaults to dynamic allocation.
 */

int nf2_major = NF2_MAJOR;
int nf2_minor = 0;
module_param(nf2_major, int, S_IRUGO);
module_param(nf2_minor, int, S_IRUGO);

struct sock *nf2_nl_sk = NULL;
struct kfifo *nf2_delay_fifo = NULL;
DECLARE_MUTEX(nf2_delay_wr_sem);

/*
 * Function prototypes
 */
static void nf2_validate_params(void);


/*
 * pci_device_id - table of Vendor and Device IDs for the kernel to match
 *                 to identify the card(s) supported by this driver
 */
static struct pci_device_id ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_STANFORD, PCI_DEVICE_ID_STANFORD_NF2), },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, ids);


/* 
 * Get the revision from the config space
 */
static unsigned char nf2_get_revision(struct pci_dev *pdev)
{
	u8 revision;

	pci_read_config_byte(pdev, PCI_REVISION_ID, &revision);
	return revision;
}

/*
 * Check if the board is a control board 
 */
/*static unsigned char nf2_is_control_board(void *ioaddr)
{
	unsigned int board_id;

	board_id = ioread32(ioaddr + CPCI_REG_BOARD_ID);
	return board_id & ID_VERSION;
}*/

/*
 * nf2_probe:
 * Identifies the card, performs initialization and sets up the necessary
 * data structures.
 */
static int nf2_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	int ret = -ENODEV;
	struct nf2_card_priv *card;
	int rev;
	int err;

	/* Enable the device */
	if((err = pci_enable_device(pdev))) {
		printk(KERN_ERR "nf2: Unable to enable the PCI device, aborting.\n");
		goto err_out_none;
	}

	
	/* Grab the revision and make sure we know about it */
	rev = nf2_get_revision(pdev);
	printk(KERN_INFO "nf2: Found an NF2 device (cfg revision %d)...\n", rev);
	if (rev != 0x00)
		return -ENODEV;

	/* Enable bus mastering */
	PDEBUG(KERN_INFO "nf2: Enabling bus mastering\n");
	pci_set_master(pdev);

	/* Test to make sure we can correctly set the DMA mask */
	PDEBUG(KERN_INFO "nf2: Setting DMA mask\n");
	if((err = pci_set_dma_mask(pdev, 0xFFFFFFFFULL))) {
		printk(KERN_ERR "nf2: No usable DMA configuration, aborting.\n");
		goto err_out_none;
	}

	/* Request the memory region corresponding to the card */
	PDEBUG(KERN_INFO "nf2: Requesting memory region for NF2\n");
	if (!request_mem_region(pci_resource_start(pdev, 0),
			pci_resource_len(pdev, 0), "nf2")) {
		printk (KERN_ERR "nf2: cannot reserve MMIO region\n");
		goto err_out_none;
	}

	/* Create the card private data structure */
	PDEBUG(KERN_INFO "nf2: kmallocing memory for nf2_card_priv\n");
	card = (struct nf2_card_priv*)kmalloc(sizeof(struct nf2_card_priv),
						  GFP_KERNEL);
	if (card == NULL) {
		printk(KERN_ERR "nf2: Could not allocate memory for card private data.\n");

		ret = -ENOMEM;
		goto err_out_free_mem_region;
	}
	/* Clear the contents of the data structure */
	memset(card, 0, sizeof(struct nf2_card_priv));
	
	/* Record the pdev corresponding to the card */
	card->pdev = pdev;

	/* Initialize the locking mechanisms */
	PDEBUG(KERN_INFO "nf2: initializing data structures in card\n");
	init_MUTEX(&card->state_lock);
	spin_lock_init(&card->txbuff_lock);
	INIT_LIST_HEAD(&card->free_txbuffs);
	INIT_LIST_HEAD(&card->transmit_txbuffs);

	/* Store the netdevice associated with the pdev */
	pci_set_drvdata(pdev, card);

	/* Map the memory region */
	PDEBUG(KERN_INFO "nf2: mapping I/O space\n");
	card->ioaddr = ioremap(pci_resource_start(pdev, 0),
			       pci_resource_len(pdev, 0));
	if (!card->ioaddr)
	{
		printk (KERN_ERR "nf2: cannot remap mem region %lx @ %lx\n",
			(long unsigned int)pci_resource_len(pdev, 0), 
			(long unsigned int)pci_resource_start(pdev, 0));
		goto err_out_free_card;
	}

	/* Disable all MACs */
	iowrite32(0, card->ioaddr + CNET_REG_ENABLE);

	/* Work out whether the card is a control or user card */
	PDEBUG(KERN_INFO "nf2: calling control/user probe function\n");
	card->is_ctrl = 1;
	ret = nf2c_probe(pdev, id, card);
	/*card->is_ctrl = nf2_is_control_board(card->ioaddr);
	if (card->is_ctrl)
	{
		ret = nf2c_probe(pdev, id, card);
	}
	else
	{
		ret = nf2u_probe(pdev, id, card);
	}*/

	/* Check for errors from the control/user probes */
	if (ret < 0)
	{
		goto err_out_iounmap;
	}
	else
	{
		/* If we make it here then everything has succeeded */
		PDEBUG(KERN_INFO "nf2: device probe succeeded\n");
		return ret;
	}

	/* Error handling */
err_out_iounmap:
	iounmap(card->ioaddr);

err_out_free_card:
	pci_set_drvdata(pdev, NULL);
	kfree(card);

err_out_free_mem_region:
	release_mem_region(pci_resource_start(pdev, 0), 
			   pci_resource_len(pdev, 0));

err_out_none:
	pci_disable_device(pdev);
	return ret;
}

static void nf2_remove(struct pci_dev *pdev)
{
	struct nf2_card_priv *card;

	/* clean up any allocated resources and stuff here.
	 * like call release_region();
	 */
	printk(KERN_ALERT "nf2: Unloading driver\n");

	/* Get the private data */
	card = (struct nf2_card_priv*)pci_get_drvdata(pdev);
	if (card)
	{
		/* Call the control/user release function */
		nf2c_remove(pdev, card);
		/*if (card->is_ctrl)
			nf2c_remove(pdev, card);
		else
			nf2u_remove(pdev, card);*/

		/* Unmap the IO memory region */
		if (card->ioaddr)
		{
			printk(KERN_ALERT "nf2: unmapping ioaddr\n");
			iounmap(card->ioaddr);
		}

		/* Free the private data */
		printk(KERN_ALERT "nf2: freeing card\n");
		kfree(card);
	}

	/* Unset the driver data */
	printk(KERN_ALERT "nf2: setting drvdata to NULL\n");
	pci_set_drvdata(pdev, NULL);

	/* Release the memory */
	printk(KERN_ALERT "nf2: releasing mem region\n");
	release_mem_region(pci_resource_start(pdev, 0), 
			   pci_resource_len(pdev, 0));

	/* Disable the device */
	printk(KERN_ALERT "nf2: disabling device\n");
	pci_disable_device(pdev);

	printk(KERN_ALERT "nf2: finished removing\n");
}

/*static void nfnetlink_rcv(struct sock *sk, int len)
{
	do {
		struct sk_buff *skb;

		if (nfnl_shlock_nowait())
			return;

		while ((skb = skb_dequeue(&sk->sk_receive_queue)) != NULL) {
			if (nfnetlink_rcv_skb(skb)) {
				if (skb->len)
					skb_queue_head(&sk->sk_receive_queue,
						       skb);
				else
					kfree_skb(skb);
				break;
			}
			kfree_skb(skb);
		}
*/
		/* don't call nfnl_shunlock, since it would reenter
		 * with further packet processing */
/*	up(&nfnl_sem);
	} while(nfnl && nfnl->sk_receive_queue.qlen);
}*/


static int nl_receive_msg(struct sk_buff *skb, struct nlmsghdr *nlh) {
  u8 *payload;
  unsigned int used;

  /*if (printk_ratelimit())
    printk(KERN_ERR "nf2: Received NL message type:%hu flags:%hu seq:%u pid:%u\n", nlh->nlmsg_type, nlh->nlmsg_flags, nlh->nlmsg_seq, nlh->nlmsg_pid);*/

  payload = NLMSG_DATA(nlh);

  switch (nlh->nlmsg_type) {
  case NF2_NL_DELAYS:
    break;
  case NF2_NL_PKT:
    return nf2c_inject_pkt((struct pkt_descr*) payload);
  case NF2_NL_SIM_START:
    return nf2c_sim_start((struct NFSimStartMsg*) payload);
  case NF2_NL_MARK:
    return nf2c_mark(nlh->nlmsg_pid, nlh->nlmsg_seq);
  }
   //used = __kfifo_put(nf2_delay_fifo, payload, NLMSG_PAYLOAD(nlh, 0));

  //printk(KERN_ERR "nf2: Received NL message type:%hu flags:%hu seq:%u pid:%u\n", nlh->nlmsg_type, nlh->nlmsg_flags, nlh->nlmsg_seq, nlh->nlmsg_pid);
      
  /* process netlink message with header pointed by
   * nlhand payload pointed by payload
   */
  //printk(KERN_ERR "nf2: Received NL message of len %u and payload len %d. Put %u in fifo.\n", nlh->nlmsg_len, NLMSG_PAYLOAD(nlh, 0), used);

  return 0;
}

static void nl_skb_in(struct sk_buff *skb) 
{
  int             err;
  struct nlmsghdr *nlh;
  u32             rlen;
  
  while (skb->len >= NLMSG_SPACE(0)) {
    nlh = nlmsg_hdr(skb);
    if (nlh->nlmsg_len < sizeof(*nlh) || skb->len < nlh->nlmsg_len)
      return;

    rlen = NLMSG_ALIGN(nlh->nlmsg_len);
    if (rlen > skb->len)
      rlen = skb->len;

    if ((err = nl_receive_msg(skb, nlh))) {
      /*printk("ERROR ACKING\n");*/
      netlink_ack(skb, nlh, err);
    } else if (nlh->nlmsg_flags & NLM_F_ACK) {
      /*printk("ACKING\n");*/
      netlink_ack(skb, nlh, 0);
    } else {
      /*printk("NO ACKING\n");*/
    }
    skb_pull(skb, rlen);
  }
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
static void nl_sock_in(struct sock *sk, int len) 
{
  struct sk_buff *skb;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22) 
  if (down_interruptible(&nf2_delay_wr_sem) != 0) {
    /*  Interrupted while getting semaphore. Be sure to free the skb's */
    while ((skb = skb_dequeue(&sk->sk_receive_queue)) != NULL) {
      kfree_skb(skb);
    }
    return;
  }
#endif

  while ((skb = skb_dequeue(&sk->sk_receive_queue)) != NULL) {
    nl_skb_in(skb);
    kfree_skb(skb);
  }

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22) 
  up(&nf2_delay_wr_sem);
#endif
}
#endif

/*
 * Validate the value of the params passed in and adjust them if necessary
 */
static void nf2_validate_params(void)
{
	if (timeout <= 0)
	{
		printk(KERN_WARNING "nf2: Value of timeout param must be positive. Value: %d\n", timeout);
		timeout = NF2_TIMEOUT;
	}

	if (rx_pool_size <= 0)
	{
		printk(KERN_WARNING "nf2: Value of rx_pool_size param must be positive. Value: %d\n", rx_pool_size);
		rx_pool_size = NUM_RX_BUFFS;
	}

	if (tx_pool_size <= 0)
	{
		printk(KERN_WARNING "nf2: Value of tx_pool_size param must be positive. Value: %d\n", tx_pool_size);
		tx_pool_size = NUM_TX_BUFFS;
	}

	if (nf2_major < 0)
	{
		printk(KERN_WARNING "nf2: Value of nf2_major param cannot be negative. Value: %d\n", nf2_major);
		rx_pool_size = NF2_MAJOR;
	}

	if (nf2_minor < 0)
	{
		printk(KERN_WARNING "nf2: Value of nf2_minor param cannot be negative. Value: %d\n", nf2_minor);
		rx_pool_size = 0;
	}
}

/*
 * pci_driver structure to set up callbacks for various PCI events
 */ 
static struct pci_driver pci_driver = {
	.name = "nf2",
	.id_table = ids,
	.probe = nf2_probe,
	.remove = nf2_remove,
};

static void tfunc(unsigned long v); 

static DEFINE_TIMER(my_timer, tfunc, 0, 5);

static void tfunc(unsigned long v) {
  printk(KERN_ALERT "!!!!!!!!!1tfunc called!!!!!!! expires is %lu, jiffies is %lu\n", my_timer.expires, jiffies);
  if (v) {
    my_timer.data = v-1;
    my_timer.expires = jiffies + 1000;
    add_timer(&my_timer);
  }
}





static int pci_skel_init(void)
{
  int ret;

  /*  my_timer.expires = jiffies + 1000;
      add_timer(&my_timer);*/
  

	/* Validate the params */
	nf2_validate_params();


	/* Open the netlink socket to communicate with userspace */
	nf2_nl_sk = netlink_kernel_create(
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
					  &init_net,
#endif
					  NETLINK_NF_PKTGEN, 
					  0, 

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
					  nl_skb_in,
#else
					  nl_sock_in,				
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,22) 
					  NULL,
#endif
					  THIS_MODULE);
	if (nf2_nl_sk == NULL)
	{
	  printk(KERN_ERR "nf2: Could not allocate netlink socket.\n");
	  ret = -ENOMEM;
	  goto fail_nl;
	}
	printk(KERN_ERR "nf2: Allocated netlink socket.\n");

	nf2_delay_fifo = kfifo_alloc(NF2_DELAY_FIFO_LEN, GFP_KERNEL, NULL);
	if (IS_ERR(nf2_delay_fifo)) {
	  ret = PTR_ERR(nf2_delay_fifo);
	  goto fail_fifo;
	}

	/* Register the driver */
	ret = pci_register_driver(&pci_driver);
	if (ret != 0) {
	  ret = ENODEV;
	  goto fail_pci_reg;
	}

	/* Success */
	return 0;

 fail_pci_reg:
	kfifo_free(nf2_delay_fifo);
 fail_fifo:
	nf2_delay_fifo = NULL;
	sock_release(nf2_nl_sk->sk_socket);
	nf2_nl_sk = NULL;
 fail_nl:
	return ret;
}

static void pci_skel_exit(void)
{
	pci_unregister_driver(&pci_driver);
	kfifo_free(nf2_delay_fifo);
	nf2_delay_fifo = NULL;
	sock_release(nf2_nl_sk->sk_socket);
	nf2_nl_sk = NULL;
	del_timer_sync(&my_timer);
}

module_init(pci_skel_init);
module_exit(pci_skel_exit);
