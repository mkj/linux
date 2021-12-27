// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) Nuvoton Technology corporation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i3c/device.h>
#include <linux/i3c/master.h>

#define SPD5118_EEPROM_SIZE		1024
#define SPD5118_PAGE_COUNT		16
#define SPD5118_PAGE_SIZE		64
#define SPD5118_PAGE_SHIFT		6

#define PAGE_ADDR0(page)	(((page) & 0x1) << 6)
#define PAGE_ADDR1_4(page)	(((page) & GENMASK_ULL(4, 1)) >> 1)

#define REG_MR48	0x30
#define WR_OP		BIT(3)

static DEFINE_MUTEX(spd5118_bus_lock);

static const struct i3c_device_id spd5118_i3c_ids[] = {
	I3C_DEVICE(0x0266, 0x5118, NULL),
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(i3c, spd5118_i3c_ids);

static int spd5118_reg_read(struct device *dev, char reg
			, u8 *val)
{
	struct i3c_device *i3c = dev_to_i3cdev(dev);
	struct i3c_priv_xfer xfers[2];
	int status;
	u8 addr[2];

	addr[0] = reg; /* MemReg=0 */
	addr[1] = 0;
	xfers[0].rnw = false;
	xfers[0].len = 2;
	xfers[0].data.out = (u8 *)addr;

	xfers[1].rnw = true;
	xfers[1].len = 1;
	xfers[1].data.in = val;

	status = i3c_device_do_priv_xfers(i3c, xfers, 2);

	return status;
}

static int spd5118_prep_wr(struct device *dev)
{
	unsigned long timeout = jiffies + HZ;
	u8 val;
	int ret;

	while (time_before(jiffies, timeout)) {
		ret = spd5118_reg_read(dev, REG_MR48, &val);
		if (ret)
			return -EINVAL;
		if ((val & WR_OP) == 0)
			return 0;
		cond_resched();
	}

	return -EBUSY;
}

static ssize_t spd5118_eeprom_read(struct device *dev, char *buf,
				  unsigned int offset, size_t count, int page)
{
	struct i3c_device *i3c = dev_to_i3cdev(dev);
	struct i3c_priv_xfer xfers[2];
	int status;
	u8 addr[2];

	if (count > SPD5118_EEPROM_SIZE)
		count = SPD5118_EEPROM_SIZE;
	/* Can't cross page boundaries */
	if (unlikely(offset + count > SPD5118_PAGE_SIZE))
		count = SPD5118_PAGE_SIZE - offset;

	addr[0] = 0x80 | PAGE_ADDR0(page) | offset; /* MemReg=1 */
	addr[1] = PAGE_ADDR1_4(page);
	xfers[0].rnw = false;
	xfers[0].len = 2;
	xfers[0].data.out = (u8 *)&addr[0];

	xfers[1].rnw = true;
	xfers[1].len = count;
	xfers[1].data.in = buf;

	status = i3c_device_do_priv_xfers(i3c, xfers, 2);
	if (status == 0)
		status = count;

	dev_dbg(dev, "read nvm %lu@%d.%d --> %d\n", count, page, offset, status);

	return status;
}

static ssize_t spd5118_eeprom_write(struct device *dev, char *buf,
				  unsigned int offset, size_t count, int page)
{
	struct i3c_device *i3c = dev_to_i3cdev(dev);
	struct i3c_priv_xfer xfer;
	int status, i;
	u8 *out;

	if (count > SPD5118_EEPROM_SIZE)
		count = SPD5118_EEPROM_SIZE;
	/* Can't cross page boundaries */
	if (unlikely(offset + count > SPD5118_PAGE_SIZE))
		count = SPD5118_PAGE_SIZE - offset;

	out = kmalloc(count, GFP_KERNEL);
	if (!out)
		return -ENOMEM;

	out[0] = 0x80 | PAGE_ADDR0(page) | offset; /* MemReg=1 */
	out[1] = PAGE_ADDR1_4(page);

	for (i = 0; i < count; i++) {
		status = spd5118_prep_wr(dev);
		if (status)
			break;

		out[2] = buf[i];
		xfer.rnw = false;
		xfer.len = 3;
		xfer.data.out = out;

		status = i3c_device_do_priv_xfers(i3c, &xfer, 1);
		if (status) {
			status = -EIO;
			break;
		}
		out[0]++;
	}

	kfree(out);
	dev_dbg(dev, "write nvm %lu@%d.%d --> %ld\n",
			count, page, offset, (status ? status: count));

	return (status ? status: count);
}

static ssize_t spd5118_read(struct file *filp, struct kobject *kobj,
			   struct bin_attribute *bin_attr,
			   char *buf, loff_t off, size_t count)
{
	struct device *dev = kobj_to_dev(kobj);
	size_t requested = count;
	int page;

	if (unlikely(!count))
		return count;

	page = off >> SPD5118_PAGE_SHIFT;
	if (unlikely(page >= SPD5118_PAGE_COUNT))
		return 0;
	off &= (1 << SPD5118_PAGE_SHIFT) - 1;

	mutex_lock(&spd5118_bus_lock);

	while (count) {
		int status;

		status = spd5118_eeprom_read(dev, buf, off, count, page);
		if (status < 0) {
			mutex_unlock(&spd5118_bus_lock);
			return status;
		}
		buf += status;
		off += status;
		count -= status;

		if (off == SPD5118_PAGE_SIZE) {
			page++;
			off = 0;
		}
	}

	mutex_unlock(&spd5118_bus_lock);

	return requested;
}

static ssize_t spd5118_write(struct file *filp, struct kobject *kobj,
				   struct bin_attribute *attr,
				   char *buf, loff_t off, size_t count)
{
	struct device *dev = kobj_to_dev(kobj);
	size_t requested = count;
	int page;

	if (unlikely(!count))
		return count;

	page = off >> SPD5118_PAGE_SHIFT;
	if (unlikely(page >= SPD5118_PAGE_COUNT))
		return 0;
	off &= (1 << SPD5118_PAGE_SHIFT) - 1;

	mutex_lock(&spd5118_bus_lock);

	while (count) {
		int status;

		status = spd5118_eeprom_write(dev, buf, off, count, page);
		if (status < 0) {
			mutex_unlock(&spd5118_bus_lock);
			return status;
		}
		buf += status;
		off += status;
		count -= status;

		if (off == SPD5118_PAGE_SIZE) {
			page++;
			off = 0;
		}
	}

	mutex_unlock(&spd5118_bus_lock);

	return requested;
}



static const struct bin_attribute eeprom_attr = {
	.attr = {
		.name = "eeprom",
		.mode = S_IRUGO | S_IWUSR,
	},
	.size = SPD5118_EEPROM_SIZE,
	.read = spd5118_read,
	.write = spd5118_write,
};

static int spd5118_i3c_probe(struct i3c_device *i3cdev)
{
	int err;

	/* Create the sysfs eeprom file */
	err = sysfs_create_bin_file(&i3cdev->dev.kobj, &eeprom_attr);
	if (err)
		return err;

	dev_info(&i3cdev->dev, "%u byte SPD EEPROM\n",
			SPD5118_EEPROM_SIZE);

	return 0;
}

static int spd5118_i3c_remove(struct i3c_device *i3cdev)
{
	sysfs_remove_bin_file(&i3cdev->dev.kobj, &eeprom_attr);

	return 0;
}

static struct i3c_driver spd5118_driver = {
	.driver = {
		.name = "spd5118",
	},
	.probe = spd5118_i3c_probe,
	.remove = spd5118_i3c_remove,
	.id_table = spd5118_i3c_ids,
};
module_i3c_driver(spd5118_driver);

MODULE_DESCRIPTION("Driver for JESD403-compliant DDR5 SPD EEPROMs");
MODULE_AUTHOR("Stanley Chu <yschu@nuvoton.com>");
MODULE_LICENSE("GPL v2");
