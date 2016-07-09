#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
 .name = KBUILD_MODNAME,
 .init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
 .exit = cleanup_module,
#endif
 .arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0xf44298a2, "struct_module" },
	{ 0x4c3af445, "__request_region" },
	{ 0xb0c8677f, "pci_bus_read_config_byte" },
	{ 0x12da5bb2, "__kmalloc" },
	{ 0xba2513f6, "__mod_timer" },
	{ 0xa5423cc4, "param_get_int" },
	{ 0xdc3eaf70, "iomem_resource" },
	{ 0xdb9ff32c, "malloc_sizes" },
	{ 0xa7c35c6b, "getnstimeofday" },
	{ 0xa0910339, "pci_disable_device" },
	{ 0xdee7300c, "sock_release" },
	{ 0x681a249a, "netif_carrier_on" },
	{ 0x7dd8a396, "down_interruptible" },
	{ 0x356b3c46, "netif_carrier_off" },
	{ 0xcb32da10, "param_set_int" },
	{ 0xeaa456ed, "_spin_lock_irqsave" },
	{ 0x7d11c268, "jiffies" },
	{ 0xa13798f8, "printk_ratelimit" },
	{ 0x576dea38, "pci_set_master" },
	{ 0xfa593d6a, "del_timer_sync" },
	{ 0x6ec877dd, "netlink_kernel_create" },
	{ 0xed9b195c, "pci_set_dma_mask" },
	{ 0xb5b9d827, "kfifo_alloc" },
	{ 0x1b7d4074, "printk" },
	{ 0xf45b18b7, "free_netdev" },
	{ 0x2da418b5, "copy_to_user" },
	{ 0xaa1c0ec7, "register_netdev" },
	{ 0x27147e64, "_spin_unlock_irqrestore" },
	{ 0xfafd0fdc, "netlink_unicast" },
	{ 0x9c09bd99, "skb_pull" },
	{ 0x98e03e2, "init_net" },
	{ 0xbc91f04f, "boot_tvec_bases" },
	{ 0xcdd63367, "kmem_cache_alloc" },
	{ 0xda05202, "__alloc_skb" },
	{ 0xfbed20d8, "ioremap_nocache" },
	{ 0xedd14538, "param_get_uint" },
	{ 0x26c495e6, "alloc_netdev_mq" },
	{ 0x1fc91fb2, "request_irq" },
	{ 0xb415af6e, "kfree_skb" },
	{ 0x8bb33e7d, "__release_region" },
	{ 0x585fd37, "pci_unregister_driver" },
	{ 0x7323c956, "ether_setup" },
	{ 0xa65df142, "netlink_ack" },
	{ 0x37a0cba, "kfree" },
	{ 0x126970ed, "param_set_uint" },
	{ 0xedc03953, "iounmap" },
	{ 0x381da1, "up" },
	{ 0xd38092d7, "__pci_register_driver" },
	{ 0xe3262722, "unregister_netdev" },
	{ 0x96fa7700, "__netif_schedule" },
	{ 0x436c2179, "iowrite32" },
	{ 0xfe4075f2, "skb_put" },
	{ 0xd49c1f60, "pci_enable_device" },
	{ 0xf2a644fb, "copy_from_user" },
	{ 0x9e35dbae, "dma_ops" },
	{ 0xe484e35f, "ioread32" },
	{ 0xf20dabd8, "free_irq" },
	{ 0xbaf117e, "kfifo_free" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

MODULE_ALIAS("pci:v0000FEEDd00000001sv*sd*bc*sc*i*");
