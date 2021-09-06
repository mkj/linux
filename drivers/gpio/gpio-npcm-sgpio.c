// SPDX-License-Identifier: GPL-2.0
/*
 * Nuvoton NPCM Serial GPIO Driver
 *
 * Copyright (C) 2021 Nuvoton Technologies
 */
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/gpio/driver.h>
#include <linux/hashtable.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#define MAX_NR_HW_SGPIO			64

#define  IOXCFG1 0x2A
#define  IOXCFG1_SFT_CLK GENMASK(3, 0)
#define  IOXCFG1_SFT_CLK_2	0x0E
#define  IOXCFG1_SFT_CLK_3	0x0D
#define  IOXCFG1_SFT_CLK_4	0x0C
#define  IOXCFG1_SFT_CLK_8	0x07
#define  IOXCFG1_SFT_CLK_16	0x06
#define  IOXCFG1_SFT_CLK_32	0x05
#define  IOXCFG1_SFT_CLK_1024	0x00
#define  IOXCFG1_SCLK_POL BIT(4)
#define  IOXCFG1_LDSH_POL BIT(5)

#define  IOXCTS 0x28
#define  IOXCTS_IOXIF_EN BIT(7)
#define  IOXCTS_RD_MODE GENMASK(2, 1)
#define  IOXCTS_RD_MODE_PERIODIC BIT(2)
#define  IOXCTS_RD_MODE_CONTINUOUS GENMASK(2, 1)

#define  IOXCFG2 0x2B
#define  IXOEVCFG_MASK 0x3

#define GPIO_BANK(x)    ((x) / 8)
#define GPIO_BIT(x)     ((x) % 8)

struct nuvoton_sgpio {
	struct gpio_chip chip;
	struct clk *pclk;
	spinlock_t lock;
	void __iomem *base;
	u8 nin_sgpio;
	u8 nout_sgpio;
	u8 in_port;
	u8 out_port;
	int irq;
	struct irq_domain	*domain;
	u8 int_type[64];
};

struct nuvoton_sgpio_bank {
	u8    rdata_reg;
	u8    wdata_reg;
	u8    event_config;
	u8    event_status;
};

enum nuvoton_sgpio_reg {
	rdata_reg,
	wdata_reg,
	event_config,
	event_status,
};

static const struct nuvoton_sgpio_bank nuvoton_sgpio_banks[] = {
	{
		.rdata_reg = 0x08,
		.wdata_reg = 0x00,
		.event_config = 0x10,
		.event_status = 0x20,
	},
	{
		.rdata_reg = 0x09,
		.wdata_reg = 0x01,
		.event_config = 0x12,
		.event_status = 0x21,
	},
	{
		.rdata_reg = 0x0a,
		.wdata_reg = 0x02,
		.event_config = 0x14,
		.event_status = 0x22,
	},
	{
		.rdata_reg = 0x0b,
		.wdata_reg = 0x03,
		.event_config = 0x16,
		.event_status = 0x23,
	},
	{
		.rdata_reg = 0x0c,
		.wdata_reg = 0x04,
		.event_config = 0x18,
		.event_status = 0x24,
	},
	{
		.rdata_reg = 0x0d,
		.wdata_reg = 0x05,
		.event_config = 0x1a,
		.event_status = 0x25,
	},
	{
		.rdata_reg = 0x0e,
		.wdata_reg = 0x06,
		.event_config = 0x1c,
		.event_status = 0x26,
	},
	{
		.rdata_reg = 0x0f,
		.wdata_reg = 0x07,
		.event_config = 0x1e,
		.event_status = 0x27,
	},

};

static void __iomem *bank_reg(struct nuvoton_sgpio *gpio,
			      const struct nuvoton_sgpio_bank *bank,
				const enum nuvoton_sgpio_reg reg)
{
	switch (reg) {
	case rdata_reg:
		return gpio->base + bank->rdata_reg;
	case wdata_reg:
		return gpio->base + bank->wdata_reg;
	case event_config:
		return gpio->base + bank->event_config;
	case event_status:
		return gpio->base + bank->event_status;
	default:
		/* acturally if code runs to here, it's an error case */
		BUG();
	}
}

static const struct nuvoton_sgpio_bank *to_bank(unsigned int offset)
{
	unsigned int bank = GPIO_BANK(offset);

	return &nuvoton_sgpio_banks[bank];
}

static void irqd_to_nuvoton_sgpio_data(struct irq_data *d,
				       struct nuvoton_sgpio **gpio,
					const struct nuvoton_sgpio_bank **bank,
					u8 *bit, int *offset)
{
	struct nuvoton_sgpio *internal;

	*offset = irqd_to_hwirq(d);
	internal = irq_data_get_irq_chip_data(d);
	WARN_ON(!internal);

	*gpio = internal;
	*offset -= internal->nout_sgpio;
	*bank = to_bank(*offset);
	*bit = GPIO_BIT(*offset);
}

static int nuvoton_sgpio_init_valid_mask(struct gpio_chip *gc,
					 unsigned long *valid_mask, unsigned int ngpios)
{
	struct nuvoton_sgpio *gpio = gpiochip_get_data(gc);
	u8 in_port, out_port, set_port;

	if (gpio->nin_sgpio % 8 > 0)
		in_port = gpio->nin_sgpio / 8 + 1;
	else
		in_port = gpio->nin_sgpio / 8;

	if (gpio->nout_sgpio % 8 > 0)
		out_port = gpio->nout_sgpio / 8 + 1;
	else
		out_port = gpio->nout_sgpio / 8;

	gpio->in_port = in_port;
	gpio->out_port = out_port;
	set_port = ((out_port & 0xF) << 4) | (in_port & 0xF);
	iowrite8(set_port, gpio->base + IOXCFG2);

	return 0;
}

static int nuvoton_sgpio_dir_in(struct gpio_chip *gc, unsigned int offset)
{
	struct nuvoton_sgpio *gpio = gpiochip_get_data(gc);

	if (offset < gpio->nout_sgpio)
		return -EINVAL;

	return 0;
}

static int nuvoton_sgpio_dir_out(struct gpio_chip *gc, unsigned int offset, int val)
{
	struct nuvoton_sgpio *gpio = gpiochip_get_data(gc);

	if (offset < gpio->nout_sgpio) {
		gc->set(gc, offset, val);
		return 0;
	} else {
		return -EINVAL;
	}
}

static int nuvoton_sgpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct nuvoton_sgpio *gpio = gpiochip_get_data(gc);

	if (offset < gpio->nout_sgpio)
		return 0;
	else
		return 1;
}

static void nuvoton_sgpio_set(struct gpio_chip *gc, unsigned int offset, int val)
{
	struct nuvoton_sgpio *gpio = gpiochip_get_data(gc);
	const struct  nuvoton_sgpio_bank *bank = to_bank(offset);
	void __iomem *addr;
	u8 reg = 0;

	addr = bank_reg(gpio, bank, wdata_reg);
	reg = ioread8(addr);

	if (val) {
		reg |= (val << GPIO_BIT(offset));
		iowrite8(reg, addr);
	} else {
		reg &= ~(1 << GPIO_BIT(offset));
		iowrite8(reg, addr);
	}
}

static int nuvoton_sgpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct nuvoton_sgpio *gpio = gpiochip_get_data(gc);
	const struct  nuvoton_sgpio_bank *bank;
	void __iomem *addr;
	u8 dir, reg;

	dir = nuvoton_sgpio_get_direction(gc, offset);
	if (dir == 0) {
		bank = to_bank(offset);

		addr = bank_reg(gpio, bank, wdata_reg);
		reg = ioread8(addr);
		reg = (reg >> GPIO_BIT(offset)) & 0x01;
	} else {
		offset -= gpio->nout_sgpio;
		bank = to_bank(offset);

		addr = bank_reg(gpio, bank, rdata_reg);
		reg = ioread8(addr);
		reg = (reg >> GPIO_BIT(offset)) & 0x01;
	}

	return reg;
}

static void nuvoton_sgpio_setup_enable(struct nuvoton_sgpio *gpio, u8 enable)
{
	u8 reg = 0;

	reg = ioread8(gpio->base + IOXCTS);
	reg = reg & ~IOXCTS_RD_MODE;
	reg = reg | IOXCTS_RD_MODE_PERIODIC;

	if (enable == 1) {
		reg |= IOXCTS_IOXIF_EN;
		iowrite8(reg, gpio->base + IOXCTS);
	} else {
		reg &= ~IOXCTS_IOXIF_EN;
		iowrite8(reg, gpio->base + IOXCTS);
	}
}

static int nuvoton_sgpio_setup_clk(struct nuvoton_sgpio *gpio, u32 sgpio_freq)
{
	unsigned long apb_freq;
	u32 sgpio_clk_div;
	u8 tmp;

	apb_freq = clk_get_rate(gpio->pclk);
	sgpio_clk_div = (apb_freq / sgpio_freq);
	if ((apb_freq % sgpio_freq) != 0)
		sgpio_clk_div += 1;

	tmp = ioread8(gpio->base + IOXCFG1) & ~IOXCFG1_SFT_CLK;

	if (sgpio_clk_div >= 1024)
		iowrite8(IOXCFG1_SFT_CLK_1024 | tmp, gpio->base + IOXCFG1);
	else if (sgpio_clk_div >= 32)
		iowrite8(IOXCFG1_SFT_CLK_32 | tmp, gpio->base + IOXCFG1);
#ifndef CONFIG_ARCH_NPCM7XX
	else if (sgpio_clk_div >= 16)
		iowrite8(IOXCFG1_SFT_CLK_16 | tmp, gpio->base + IOXCFG1);
#endif
	else if (sgpio_clk_div >= 8)
		iowrite8(IOXCFG1_SFT_CLK_8 | tmp, gpio->base + IOXCFG1);
	else if (sgpio_clk_div >= 4)
		iowrite8(IOXCFG1_SFT_CLK_4 | tmp, gpio->base + IOXCFG1);
#ifdef CONFIG_ARCH_NPCM7XX
	else if (sgpio_clk_div >= 3)
		iowrite8(IOXCFG1_SFT_CLK_3 | tmp, gpio->base + IOXCFG1);
	else if (sgpio_clk_div >= 2)
		iowrite8(IOXCFG1_SFT_CLK_2 | tmp, gpio->base + IOXCFG1);
#endif
	else
		return -EINVAL;
	return 0;
}

static void nuvoton_sgpio_irq_init_valid_mask(struct gpio_chip *gc,
					      unsigned long *valid_mask, unsigned int ngpios)
{
	struct nuvoton_sgpio *gpio = gpiochip_get_data(gc);
	int n = gpio->nin_sgpio;

	/* input GPIOs in the high range */
	bitmap_set(valid_mask, gpio->nout_sgpio, n);
	bitmap_clear(valid_mask, 0, gpio->nout_sgpio);
}

static void nuvoton_sgpio_irq_set_mask(struct irq_data *d, bool set)
{
	const struct nuvoton_sgpio_bank *bank;
	struct nuvoton_sgpio *gpio;
	unsigned long flags;
	u16 reg;
	u8 bit, type;
	void __iomem *addr;
	int offset;

	irqd_to_nuvoton_sgpio_data(d, &gpio, &bank, &bit, &offset);
	addr = bank_reg(gpio, bank, event_config);

	spin_lock_irqsave(&gpio->lock, flags);

	nuvoton_sgpio_setup_enable(gpio, 0);

	reg = ioread16(addr);
	if (set) {
		reg &= ~(IXOEVCFG_MASK << (bit * 2));
	} else {
		type = gpio->int_type[offset - gpio->nout_sgpio];
		reg |= (type << (bit * 2));
	}

	iowrite16(reg, addr);

	nuvoton_sgpio_setup_enable(gpio, 1);

	addr = bank_reg(gpio, bank, event_status);
	reg = ioread8(addr);
	reg |= BIT(bit);
	iowrite8(reg, addr);

	spin_unlock_irqrestore(&gpio->lock, flags);
}

static void nuvoton_sgpio_irq_ack(struct irq_data *d)
{
	const struct nuvoton_sgpio_bank *bank;
	struct nuvoton_sgpio *gpio;
	unsigned long flags;
	void __iomem *status_addr;
	int offset;
	u8 bit;

	irqd_to_nuvoton_sgpio_data(d, &gpio, &bank, &bit, &offset);
	status_addr = bank_reg(gpio, bank, event_status);
	spin_lock_irqsave(&gpio->lock, flags);
	iowrite8(BIT(bit), status_addr);
	spin_unlock_irqrestore(&gpio->lock, flags);
}

static void nuvoton_sgpio_irq_mask(struct irq_data *d)
{
	nuvoton_sgpio_irq_set_mask(d, true);
}

static void nuvoton_sgpio_irq_unmask(struct irq_data *d)
{
	nuvoton_sgpio_irq_set_mask(d, false);
}

static int nuvoton_sgpio_set_type(struct irq_data *d, unsigned int type)
{
	u32 val;
	u8 bit;
	u16 reg;
	const struct nuvoton_sgpio_bank *bank;
	irq_flow_handler_t handler;
	struct nuvoton_sgpio *gpio;
	unsigned long flags;
	void __iomem *addr;
	int offset;

	irqd_to_nuvoton_sgpio_data(d, &gpio, &bank, &bit, &offset);

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_BOTH:
		val = 3;
		handler = handle_edge_irq;
		break;
	case IRQ_TYPE_EDGE_RISING:
		val = 1;
		handler = handle_edge_irq;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		val = 2;
		handler = handle_edge_irq;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		val = 1;
		handler = handle_level_irq;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		val = 2;
		handler = handle_level_irq;
		break;
	default:
		return -EINVAL;
	}

	gpio->int_type[offset - gpio->nout_sgpio] = val;

	spin_lock_irqsave(&gpio->lock, flags);
	nuvoton_sgpio_setup_enable(gpio, 0);
	addr = bank_reg(gpio, bank, event_config);
	reg = ioread16(addr);

	reg |= (val << (bit * 2));

	iowrite16(reg, addr);
	nuvoton_sgpio_setup_enable(gpio, 1);
	spin_unlock_irqrestore(&gpio->lock, flags);

	irq_set_handler_locked(d, handler);

	return 0;
}

static void nuvoton_sgpio_irq_handler(struct irq_desc *desc)
{
	struct gpio_chip *gc = irq_desc_get_handler_data(desc);
	struct irq_chip *ic = irq_desc_get_chip(desc);
	struct nuvoton_sgpio *gpio = gpiochip_get_data(gc);
	unsigned int i, j, girq;
	unsigned long reg;

	chained_irq_enter(ic, desc);

	for (i = 0; i < ARRAY_SIZE(nuvoton_sgpio_banks); i++) {
		const struct nuvoton_sgpio_bank *bank = &nuvoton_sgpio_banks[i];

		reg = ioread8(bank_reg(gpio, bank, event_status));
		for_each_set_bit(j, &reg, 8) {
			girq = irq_find_mapping(gc->irq.domain, i * 8 + gpio->nout_sgpio + j);
			generic_handle_irq(girq);
		}
	}

	chained_irq_exit(ic, desc);
}

static struct irq_chip nuvoton_sgpio_irqchip = {
	.name       = "nuvoton-sgpio",
	.irq_ack    = nuvoton_sgpio_irq_ack,
	.irq_mask   = nuvoton_sgpio_irq_mask,
	.irq_unmask = nuvoton_sgpio_irq_unmask,
	.irq_set_type   = nuvoton_sgpio_set_type,
};

static int nuvoton_sgpio_setup_irqs(struct nuvoton_sgpio *gpio,
				    struct platform_device *pdev)
{
	int rc;
	struct gpio_irq_chip *irq;
	struct device *dev = &pdev->dev;

	rc = platform_get_irq(pdev, 0);
	if (rc < 0)
		return rc;

	gpio->irq = rc;

	irq = &gpio->chip.irq;
	irq->chip = &nuvoton_sgpio_irqchip;
	irq->init_valid_mask = nuvoton_sgpio_irq_init_valid_mask;
	irq->handler = handle_bad_irq;
	irq->default_type = IRQ_TYPE_NONE;
	irq->parent_handler = nuvoton_sgpio_irq_handler;
	irq->parent_handler_data = gpio;
	irq->parents = &gpio->irq;
	irq->num_parents = 1;

	return 0;
}

static const struct of_device_id nuvoton_sgpio_of_table[] = {
	{ .compatible = "nuvoton,npcm7xx-sgpio" },
	{ .compatible = "nuvoton,npcm845-sgpio" },
	{}
};

MODULE_DEVICE_TABLE(of, nuvoton_sgpio_of_table);

static int __init nuvoton_sgpio_probe(struct platform_device *pdev)
{
	struct nuvoton_sgpio *gpio;
	u32 nin_gpios, nout_gpios, sgpio_freq;
	int rc;

	gpio = devm_kzalloc(&pdev->dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	gpio->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(gpio->base))
		return PTR_ERR(gpio->base);

	rc = of_property_read_u32(pdev->dev.of_node, "nin_gpios", &nin_gpios);
	if (rc < 0) {
		dev_err(&pdev->dev, "Could not read ngpios property\n");
		return -EINVAL;
	}
	rc = of_property_read_u32(pdev->dev.of_node, "nout_gpios", &nout_gpios);
	if (rc < 0) {
		dev_err(&pdev->dev, "Could not read ngpios property\n");
		return -EINVAL;
	}

	gpio->nin_sgpio = nin_gpios;
	gpio->nout_sgpio = nout_gpios;
	if (gpio->nin_sgpio > MAX_NR_HW_SGPIO || gpio->nout_sgpio > MAX_NR_HW_SGPIO)
		return -EINVAL;

	rc = of_property_read_u32(pdev->dev.of_node, "bus-frequency", &sgpio_freq);
	if (rc < 0) {
		dev_err(&pdev->dev, "Could not read bus-frequency property\n");
		return -EINVAL;
	}

	gpio->pclk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(gpio->pclk)) {
		dev_err(&pdev->dev, "devm_clk_get failed\n");
		return PTR_ERR(gpio->pclk);
	}

	if (sgpio_freq == 0)
		return -EINVAL;

	rc = nuvoton_sgpio_setup_clk(gpio, sgpio_freq);
	if (rc < 0)
		return -EINVAL;

	spin_lock_init(&gpio->lock);
	gpio->chip.parent = &pdev->dev;
	gpio->chip.ngpio = nin_gpios + nout_gpios;
	gpio->chip.init_valid_mask = nuvoton_sgpio_init_valid_mask;
	gpio->chip.direction_input = nuvoton_sgpio_dir_in;
	gpio->chip.direction_output = nuvoton_sgpio_dir_out;
	gpio->chip.get_direction = nuvoton_sgpio_get_direction;
	gpio->chip.request = NULL;
	gpio->chip.free = NULL;
	gpio->chip.get = nuvoton_sgpio_get;
	gpio->chip.set = nuvoton_sgpio_set;
	gpio->chip.set_config = NULL;
	gpio->chip.label = dev_name(&pdev->dev);
	gpio->chip.base = -1;

	rc = nuvoton_sgpio_setup_irqs(gpio, pdev);
	if (rc < 0)
		return rc;

	rc = devm_gpiochip_add_data(&pdev->dev, &gpio->chip, gpio);
	if (rc < 0)
		return rc;

	nuvoton_sgpio_setup_enable(gpio, 1);
	return 0;
}

static struct platform_driver nuvoton_sgpio_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = nuvoton_sgpio_of_table,
	},
};

module_platform_driver_probe(nuvoton_sgpio_driver, nuvoton_sgpio_probe);
MODULE_AUTHOR("Jim Liu <jjliu0@nuvoton.com>");
MODULE_AUTHOR("Joseph Liu <kwliu@nuvoton.com>");
MODULE_DESCRIPTION("Nuvoton NPCM Serial GPIO Driver");
MODULE_LICENSE("GPL v2");
