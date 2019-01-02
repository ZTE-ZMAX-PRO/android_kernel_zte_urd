/* drivers/android/ram_console.c
 *
 * Copyright (C) 2007-2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/console.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/persistent_ram.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/slab.h>
#include "ram_console.h"
#include "linux/of_fdt.h"

static struct persistent_ram_zone *ram_console_zone;
static const char *bootinfo;
static size_t bootinfo_size;

#ifndef EXPORT_COMPAT
#define EXPORT_COMPAT(x)
#endif

/*
 * RAM console support by ZTE_BOOT_JIA_20130121 jia.jia
 */
#ifdef CONFIG_ZTE_RAM_CONSOLE
static struct persistent_ram ram;

/*
 * To fix compiling warning of 'Section mismatch in reference from the function'
 * with the macro of 'CONFIG_DEBUG_SECTION_MISMATCH=y'
 * by ZTE_BOOT_JIA_20121010 jia.jia
 */
#if 0
static int ram_console_persistent_ram_init(struct platform_device *pdev)
#else
static int ram_console_persistent_ram_init(struct platform_device *pdev)
#endif
{
	int ret;
	int i;
	struct device_node *node = pdev->dev.of_node;
	struct device_node *pnode = NULL;
	int len = 0;
	const __be32 *cell;

	memset(&ram, 0, sizeof(struct persistent_ram));

	pnode = of_parse_phandle(node, "linux,contiguous-region", 0);
	if (!pnode) {
		pr_err("ram_console: cannot get node\n");
		ret = -EINVAL;
		goto bail0;
	}

	cell = of_get_property(pnode, "reg", &len);
	if (!cell || len <= 0) {
		pr_err("ram_console: cannot find property\n");
		ret = -EINVAL;
		goto bail0;
	}

	if ((len / sizeof(u64)) % 2) {
		pr_err("ram_console: unexpected number of memory region\n");
		ret = -EINVAL;
		goto bail0;
	}

	ram.num_descs = (len / sizeof(u64)) / 2;

	ram.descs = (struct persistent_ram_descriptor *)kzalloc(ram.num_descs *
			sizeof(struct persistent_ram_descriptor),
			GFP_KERNEL);
	if (!ram.descs) {
		pr_err("ram_console: allocate ram descriptor memory failed\n");
		ret = -ENOMEM;
		goto bail0;
	}

	for (i = 0; i < ram.num_descs; ++i) {
		/*
		 * of_read_number always returns a 64-bit number.
		 * But we can truncate it on a 32-bit system.
		 */
		if (i == 0)
			ram.start = (phys_addr_t)of_read_number(cell, 2);
		cell += 2;
		ram.descs[i].name = pdev->name;
		ram.descs[i].size = (phys_addr_t)of_read_number(cell, 2);
		cell += 2;
		ram.size += ram.descs[i].size;
	}
	printk(KERN_ERR "persistent_ram_init, addr: %lx\n", (long)ram.start);
	printk(KERN_ERR "persistent_ram_init, size: %lx\n", (long)ram.size);

	ret = persistent_ram_early_init(&ram);
	if (ret) {
		pr_err("ram_console: persistent ram early init failed\n");
		goto bail1;
	}

	return 0;

bail1:
	kfree(ram.descs);
bail0:
	return ret;
}
#endif /* CONFIG_ZTE_RAM_CONSOLE */

static void ram_console_write(struct console *co,const char *s, unsigned int count)
{
	struct persistent_ram_zone *prz = co->data;
	persistent_ram_write(prz, s, count);
}

static struct console ram_console = {
	.name	= "ram",
	.write	= ram_console_write,
	.flags	= CON_PRINTBUFFER | CON_ENABLED | CON_ANYTIME,
         .flags        = CON_PRINTBUFFER | CON_ENABLED,

	.index	= -1,
};

void ram_console_enable_console(int enabled)
{
	if (enabled)
		ram_console.flags |= CON_ENABLED;
	else
		ram_console.flags &= ~CON_ENABLED;
}

static int ram_console_probe(struct platform_device *pdev)
{
	struct ram_console_platform_data *pdata = pdev->dev.platform_data;
	struct persistent_ram_zone *prz;

/*
 * RAM console support by ZTE_BOOT_JIA_20130121 jia.jia
 * Disable ECC here
 */
#ifdef CONFIG_ZTE_RAM_CONSOLE
	int ret;

	ret = ram_console_persistent_ram_init(pdev);
	if (ret < 0) {
		return ret;
	}

	prz = persistent_ram_init_ringbuffer(&pdev->dev, false);
#else
	prz = persistent_ram_init_ringbuffer(&pdev->dev, true);
#endif /* CONFIG_ZTE_RAM_CONSOLE */

	if (IS_ERR(prz))
		return PTR_ERR(prz);

	if (pdata) {
		bootinfo = kstrdup(pdata->bootinfo, GFP_KERNEL);
		if (bootinfo)
			bootinfo_size = strlen(bootinfo);
	}

	prz->ecc = 0;
	ram_console_zone = prz;
	ram_console.data = prz;

	register_console(&ram_console);

	return 0;
}

static struct of_device_id msm_ram_console[] = {
	{.compatible = "qcom,msm-ram-console"},
	{},
};

static struct platform_driver ram_console_driver = {
	.driver		= {
		.name	= "msm-ram-console",
		.owner = THIS_MODULE,
		.of_match_table = msm_ram_console,
	},
	.probe = ram_console_probe,
};

static int __init ram_console_module_init(void)
{
	return platform_driver_register(&ram_console_driver);
}

#ifndef CONFIG_PRINTK
#define dmesg_restrict	0
#endif

static ssize_t ram_console_read_old(struct file *file, char __user *buf,
				    size_t len, loff_t *offset)
{
	loff_t pos = *offset;
	ssize_t count;
	struct persistent_ram_zone *prz = ram_console_zone;
	size_t old_log_size = persistent_ram_old_size(prz);
	const char *old_log = persistent_ram_old(prz);
	char *str;
	int ret;

	if (dmesg_restrict && !capable(CAP_SYSLOG))
		return -EPERM;

	/* Main last_kmsg log */
	if (pos < old_log_size) {
		count = min(len, (size_t)(old_log_size - pos));
		if (copy_to_user(buf, old_log + pos, count))
			return -EFAULT;
		goto out;
	}

	/* ECC correction notice */
	pos -= old_log_size;
	count = persistent_ram_ecc_string(prz, NULL, 0);
	if (pos < count) {
		str = kmalloc(count, GFP_KERNEL);
		if (!str)
			return -ENOMEM;
		persistent_ram_ecc_string(prz, str, count + 1);
		count = min(len, (size_t)(count - pos));
		ret = copy_to_user(buf, str + pos, count);
		kfree(str);
		if (ret)
			return -EFAULT;
		goto out;
	}

	/* Boot info passed through pdata */
	pos -= count;
	if (pos < bootinfo_size) {
		count = min(len, (size_t)(bootinfo_size - pos));
		if (copy_to_user(buf, bootinfo + pos, count))
			return -EFAULT;
		goto out;
	}

	/* EOF */
	return 0;

out:
	*offset += count;
	return count;
}

static const struct file_operations ram_console_file_ops = {
	.owner = THIS_MODULE,
	.read = ram_console_read_old,
};

static int __init ram_console_late_init(void)
{
	struct proc_dir_entry *entry;
	struct persistent_ram_zone *prz = ram_console_zone;

	if (!prz)
		return 0;

	if (persistent_ram_old_size(prz) == 0)
		return 0;

	entry = proc_create("last_kmsg", S_IFREG | S_IRUGO, NULL, &ram_console_file_ops);
	if (!entry) {
		printk(KERN_ERR "ram_console: failed to create proc entry\n");
		persistent_ram_free_old(prz);
		return 0;
	}

	return 0;
}

late_initcall(ram_console_late_init);
postcore_initcall(ram_console_module_init);

EXPORT_COMPAT("qcom,msm-ram-console");
