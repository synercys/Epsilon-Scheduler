/**
 * argon-mem.c - character device access to the Argon decoder registers
 *
 * Based on bcm2835-gpiomem.c. Provides IO memory access to the decoder
 * register blocks such that ffmpeg plugins can access the hardware.
 *
 * Jonathan Bell <jonathan@raspberrypi.org>
 * Copyright (c) 2019, Raspberry Pi (Trading) Ltd.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The names of the above-listed copyright holders may not be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2, as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/pagemap.h>
#include <linux/io.h>

#define DRIVER_NAME "argon-mem"
#define DEVICE_MINOR 0

struct argon_mem_priv {
	dev_t devid;
	struct class *class;
	struct cdev argon_mem_cdev;
	unsigned long regs_phys;
	unsigned long mem_window_len;
	struct device *dev;
	const char *name;
};

static int argon_mem_open(struct inode *inode, struct file *file)
{
	int dev = iminor(inode);
	int ret = 0;
	struct argon_mem_priv *priv;
	if (dev != DEVICE_MINOR)
		ret = -ENXIO;

	priv = container_of(inode->i_cdev, struct argon_mem_priv,
				argon_mem_cdev);
	if (!priv)
		return -EINVAL;
	file->private_data = priv;
	return ret;
}

static int argon_mem_release(struct inode *inode, struct file *file)
{
	int dev = iminor(inode);
	int ret = 0;

	if (dev != DEVICE_MINOR)
		ret = -ENXIO;

	return ret;
}

static const struct vm_operations_struct argon_mem_vm_ops = {
#ifdef CONFIG_HAVE_IOREMAP_PROT
	.access = generic_access_phys
#endif
};

static int argon_mem_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct argon_mem_priv *priv;
	unsigned long pages;

	priv = file->private_data;
	pages = priv->regs_phys >> PAGE_SHIFT;
	/*
	 * The address decode is far larger than the actual number of registers.
	 * Just map the whole lot in.
	 */
	vma->vm_page_prot = phys_mem_access_prot(file, pages,
						 priv->mem_window_len,
						 vma->vm_page_prot);
	vma->vm_ops = &argon_mem_vm_ops;
	if (remap_pfn_range(vma, vma->vm_start,
			pages,
			priv->mem_window_len,
			vma->vm_page_prot)) {
		return -EAGAIN;
	}
	return 0;
}

static const struct file_operations
argon_mem_fops = {
	.owner = THIS_MODULE,
	.open = argon_mem_open,
	.release = argon_mem_release,
	.mmap = argon_mem_mmap,
};

static const struct of_device_id argon_mem_of_match[];
static int argon_mem_probe(struct platform_device *pdev)
{
	int err;
	void *ptr_err;
	const struct of_device_id *id;
	struct device *dev = &pdev->dev;
	struct device *argon_mem_dev;
	struct resource *ioresource;
	struct argon_mem_priv *priv;


	/* Allocate buffers and instance data */

	priv = kzalloc(sizeof(struct argon_mem_priv), GFP_KERNEL);

	if (!priv) {
		err = -ENOMEM;
		goto failed_inst_alloc;
	}
	platform_set_drvdata(pdev, priv);

	priv->dev = dev;
	id = of_match_device(argon_mem_of_match, dev);
	if (!id)
		return -EINVAL;
	priv->name = id->data;

	ioresource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (ioresource) {
		priv->regs_phys = ioresource->start;
		priv->mem_window_len = ioresource->end - ioresource->start;
	} else {
		dev_err(priv->dev, "failed to get IO resource");
		err = -ENOENT;
		goto failed_get_resource;
	}

	/* Create character device entries */

	err = alloc_chrdev_region(&priv->devid,
				  DEVICE_MINOR, 1, priv->name);
	if (err != 0) {
		dev_err(priv->dev, "unable to allocate device number");
		goto failed_alloc_chrdev;
	}
	cdev_init(&priv->argon_mem_cdev, &argon_mem_fops);
	priv->argon_mem_cdev.owner = THIS_MODULE;
	err = cdev_add(&priv->argon_mem_cdev, priv->devid, 1);
	if (err != 0) {
		dev_err(priv->dev, "unable to register device");
		goto failed_cdev_add;
	}

	/* Create sysfs entries */

	priv->class = class_create(THIS_MODULE, priv->name);
	ptr_err = priv->class;
	if (IS_ERR(ptr_err))
		goto failed_class_create;

	argon_mem_dev = device_create(priv->class, NULL,
					priv->devid, NULL,
					priv->name);
	ptr_err = argon_mem_dev;
	if (IS_ERR(ptr_err))
		goto failed_device_create;

	dev_info(priv->dev, "%s initialised: Registers at 0x%08lx length 0x%08lx",
		priv->name, priv->regs_phys, priv->mem_window_len);

	return 0;

failed_device_create:
	class_destroy(priv->class);
failed_class_create:
	cdev_del(&priv->argon_mem_cdev);
	err = PTR_ERR(ptr_err);
failed_cdev_add:
	unregister_chrdev_region(priv->devid, 1);
failed_alloc_chrdev:
failed_get_resource:
	kfree(priv);
failed_inst_alloc:
	dev_err(priv->dev, "could not load argon_mem");
	return err;
}

static int argon_mem_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct argon_mem_priv *priv = platform_get_drvdata(pdev);

	device_destroy(priv->class, priv->devid);
	class_destroy(priv->class);
	cdev_del(&priv->argon_mem_cdev);
	unregister_chrdev_region(priv->devid, 1);
	kfree(priv);

	dev_info(dev, "%s driver removed - OK", priv->name);
	return 0;
}

static const char argon_hevc_name[] = "argon-hevcmem";
static const char argon_h264_name[] = "argon-h264mem";
static const char argon_vp9_name[] = "argon-vp9mem";
static const char argon_intc_name[] = "argon-intcmem";

static const struct of_device_id argon_mem_of_match[] = {
	{
		.compatible = "raspberrypi,argon-hevc-decoder",
		.data = &argon_hevc_name,
	},
	{
		.compatible = "raspberrypi,argon-h264-decoder",
		.data = &argon_h264_name,
	},
	{
		.compatible = "raspberrypi,argon-vp9-decoder",
		.data = &argon_vp9_name,
	},
	/* The "intc" is included as this block of hardware contains the
	 * "frame done" status flags.
	 */
	{
		.compatible = "raspberrypi,argon-local-intc",
		.data = &argon_intc_name,
	},
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, argon_mem_of_match);

static struct platform_driver argon_mem_driver = {
	.probe = argon_mem_probe,
	.remove = argon_mem_remove,
	.driver = {
		   .name = DRIVER_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = argon_mem_of_match,
		   },
};

module_platform_driver(argon_mem_driver);

MODULE_ALIAS("platform:argon-mem");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Driver for accessing Argon decoder registers from userspace");
MODULE_AUTHOR("Jonathan Bell <jonathan@raspberrypi.org>");
