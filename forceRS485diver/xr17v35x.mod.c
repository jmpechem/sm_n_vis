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
	{ 0x7a4e47, "module_layout" },
	{ 0x5b6cef95, "pci_unregister_driver" },
	{ 0x601b53be, "uart_unregister_driver" },
	{ 0xb80179fc, "__pci_register_driver" },
	{ 0x7bc618e8, "uart_register_driver" },
	{ 0xdb12d9ae, "dev_get_drvdata" },
	{ 0xedc03953, "iounmap" },
	{ 0x6778465, "uart_remove_one_port" },
	{ 0x1c3f314e, "pci_disable_device" },
	{ 0x47eb68c7, "dev_set_drvdata" },
	{ 0xfb578fc5, "memset" },
	{ 0xd2b09ce5, "__kmalloc" },
	{ 0x4280e37d, "pci_enable_device" },
	{ 0xae43fea4, "uart_add_one_port" },
	{ 0x593a99b, "init_timer_key" },
	{ 0x888ce236, "uart_match_port" },
	{ 0x4f8b5ddb, "_copy_to_user" },
	{ 0x4f6b400b, "_copy_from_user" },
	{ 0xa1c76e0a, "_cond_resched" },
	{ 0x2072ee9b, "request_threaded_irq" },
	{ 0xdcc9020c, "kmem_cache_alloc_trace" },
	{ 0xccc8d061, "kmalloc_caches" },
	{ 0x8834396c, "mod_timer" },
	{ 0x7d11c268, "jiffies" },
	{ 0x4c4696fc, "uart_write_wakeup" },
	{ 0xd52bf1ce, "_raw_spin_lock" },
	{ 0xd062d1e1, "tty_flip_buffer_push" },
	{ 0xa7160ea, "uart_insert_char" },
	{ 0x69acdf38, "memcpy" },
	{ 0xf20dabd8, "free_irq" },
	{ 0x5d9600aa, "mutex_unlock" },
	{ 0x29610b44, "mutex_lock" },
	{ 0x37a0cba, "kfree" },
	{ 0xbbdebe8, "ipipe_unstall_root" },
	{ 0x43261dca, "_raw_spin_lock_irq" },
	{ 0x42c8de35, "ioremap_nocache" },
	{ 0x7695876b, "uart_handle_dcd_change" },
	{ 0x5e8b8c65, "uart_handle_cts_change" },
	{ 0xcf21d241, "__wake_up" },
	{ 0x8f64aa4, "_raw_spin_unlock_irqrestore" },
	{ 0x89615b53, "uart_update_timeout" },
	{ 0x9327f5ce, "_raw_spin_lock_irqsave" },
	{ 0x27e1a049, "printk" },
	{ 0xf9e00df7, "uart_get_baud_rate" },
	{ 0xbdfb6dbb, "__fentry__" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";


MODULE_INFO(srcversion, "62E052D36B4420226D1ACDD");
