// SPDX-License-Identifier: GPL-2.0
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/pci-ecam.h>
#include <linux/delay.h>
#include <linux/msi.h>
#include <linux/of_address.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/reset.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/of_irq.h>
#include <asm/irq.h>
#include <asm/cputype.h>

/* PCIe Root Complex Register */
#define LINKSTAT			0x92
#define RCCFGNUM			0x140
#define IMSI_ADDR			0x190
#define PCIERC_IMASK_LOCAL_ADDR		0x180
#define PCIERC_ISTATUS_LOCAL_ADDR	0x184
#define PCIERC_ISTATUS_MSI_ADDR		0x194
#define PCIERC_AXI_ERROR_REPORT		0x3E0

#define PCIERC_ISTATUS_LOCAL_MSI_BIT	BIT(28)
#define PCIERC_ISTATUS_LOCAL_INTA_BIT	BIT(24)
#define PCIERC_CFG_NO_SLVERR		BIT(0)

#define NPCM_CORE_SELECT		(15)

/* PCIe-to-AXI Window 0 Registers */
#define RCPA0SAL	0x600
#define RCPA0SAH	0x604
#define RCPA0TAL	0x608
#define RCPA0TAH	0x60C
#define RCPA0TP		0x610
#define RCPA0TM		0x618

/* AXI-to-PCIe Window 1 to 4 Registers */
#define RCAPnSAL(n) (0x800 + (0x20 * (n)))
#define RCAPnSAH(n) (0x804 + (0x20 * (n)))
#define RCAPnTAL(n) (0x808 + (0x20 * (n)))
#define RCAPnTAH(n) (0x80C + (0x20 * (n)))
#define RCAPnTP(n)  (0x810 + (0x20 * (n)))

/* RCAPnSAL register fields */
#define CFG_SIZE_4K     11

/* RCAPnTP register fields */
#define TRSF_PARAM_MEMORY    (0L << 16)
#define TRSF_PARAM_CONFIG    (1L << 16)
#define TRSF_PARAM_IO        (2L << 16)
#define TRSL_ID_PCIE_TX_RX   0 
#define TRSL_ID_PCIE_CONFIG  1 
#define RCA_WIN_EN	     BIT(0)

#define PLDA_XPRESS_RICH_MEMORY_WINDOW 	0
#define PLDA_XPRESS_RICH_CONFIG_WINDOW 	1
#define PLDA_XPRESS_RICH_IO_WINDOW 		2
//#define PLDA_XPRESS_RICH_IO_WINDOW 		1
#define PLDA_XPRESS_RICH_MESSAGE_WINDOW	4

#define PLDA_XPRESS_RICH_TARGET_PCI_TX_RX	0
#define PLDA_XPRESS_RICH_TARGET_PCI_CONFIG	1
#define PLDA_XPRESS_RICH_TARGET_AXI_MASTER	4

#define PCI_RC_ATTR_WIN_EN_POS			0
#define PCI_RC_ATTR_WIN_SIZE_POS		1
#define PCI_RC_ATTR_AP_ADDR_L_POS		12

#define PCI_RC_ATTR_TRSF_PARAM_POS		16
#define PCI_RC_ATTR_TRSL_ID_POS			0

#define PCI_RC_MAX_AXI_PCI_WIN			5

#define IRQ_REQUEST
#define PCI_BAR_REG 0x10
#define PCI_CMD_STATUS_REG  0x4

#define  PCI_COMMAND_IO		0x1
#define  PCI_COMMAND_MEMORY	0x2
#define  PCI_COMMAND_MASTER	0x4

//#define DBG_DELAY

/* Other rgister for initialzing root complex */
#define MFSEL1			0x0C
#define MFSEL3			0x64
#define MFSEL3_PCIEPUSE_BIT	BIT(17)

#define INTCR3			0x9c
#define INTCRPCE3		0x128
#define INTCR3_RCCORER_BIT	BIT(22)

#define LINK_UP_FIELD		(0x3F << 20)
#define LINK_RETRAIN_BIT	BIT(27)

#define RC_TO_EP_DELAY		15

#define NPCM_MSI_MAX		32

struct npcm_pcie {
	u32 			irq;
	u32 			bar0;
	struct device		*dev;
	struct resource 	*res;
	spinlock_t		used_msi_lock;
	void __iomem		*reg_base;
	void __iomem		*config_base;
	struct irq_domain 	*msi_domain;
	struct reset_control	*reset;
	struct regmap		*gcr_regmap;
	DECLARE_BITMAP(msi_irq_in_use, NPCM_MSI_MAX);
};

static void npcm_pcie_destroy_msi(unsigned int irq)
{
	struct msi_desc *msi = irq_get_msi_desc(irq);
	struct npcm_pcie *pcie = msi_desc_to_pci_sysdata(msi);
	struct irq_data *d = irq_get_irq_data(irq);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);
		
	if (!test_bit(hwirq, pcie->msi_irq_in_use))
		dev_err(pcie->dev, "Trying to free unused MSI#%d\n", irq);
	else
		clear_bit(hwirq, pcie->msi_irq_in_use);
}

static int npcm_pcie_assign_msi(struct npcm_pcie *pcie)
{
	int pos;

	pos = find_first_zero_bit(pcie->msi_irq_in_use, NPCM_MSI_MAX);
	if (pos < NPCM_MSI_MAX)
		set_bit(pos, pcie->msi_irq_in_use);
	else
		return -ENOSPC;

	return pos;
}

void npcm_msi_teardown_irq(struct msi_controller *chip,
				    unsigned int irq)
{
	npcm_pcie_destroy_msi(irq);
	irq_dispose_mapping(irq);
}

int npcm_pcie_msi_setup_irq(struct msi_controller *chip, struct pci_dev *pdev,
			    struct msi_desc *desc)
{
	struct npcm_pcie *pcie = pdev->bus->sysdata;
	unsigned int irq, id;
	struct msi_msg msg;
	int hwirq;

	hwirq = npcm_pcie_assign_msi(pcie);
	if (hwirq < 0)
		return hwirq;

	irq = irq_create_mapping(pcie->msi_domain, hwirq);
	if (!irq)
		return -EINVAL;

	irq_set_msi_desc(irq, desc);

	msg.address_hi = 0x0;
	msg.address_lo = (pcie->bar0 + IMSI_ADDR) & 0xfffffff0;

	id = read_cpuid_id();
	msg.data = (id << NPCM_CORE_SELECT) | hwirq;

	pci_write_msi_msg(irq, &msg);

	return 0;
}

/* MSI Chip Descriptor */
static struct msi_controller npcm_pcie_msi_chip = {
	.setup_irqs = NULL,
	.setup_irq = npcm_pcie_msi_setup_irq,
	.teardown_irq = npcm_msi_teardown_irq,
};

static struct irq_chip npcm_msi_irq_chip = {
	.name = "NPCM PCI-MSI",
	.irq_enable = pci_msi_unmask_irq,
	.irq_disable = pci_msi_mask_irq,
	.irq_mask = pci_msi_mask_irq,
	.irq_unmask = pci_msi_unmask_irq,
};

static int npcm_pcie_msi_map(struct irq_domain *domain, unsigned int irq,
			     irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &npcm_msi_irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

static const struct irq_domain_ops msi_domain_ops = {
	.map = npcm_pcie_msi_map,
};

static irqreturn_t npcm_msi_handler(int irq, void *data)
{
	struct npcm_pcie *pcie = (struct npcm_pcie *)data;
	unsigned int dt_irq;
	unsigned int index;
	unsigned long status;
	u32 global_pcie_status = ioread32(pcie->reg_base + PCIERC_ISTATUS_LOCAL_ADDR);

	if (global_pcie_status & PCIERC_ISTATUS_LOCAL_MSI_BIT)
	{
		status = ioread32(pcie->reg_base + PCIERC_ISTATUS_MSI_ADDR);
		if (!status)
			return IRQ_HANDLED;
		do {
			index = find_first_bit(&status, 32);
			iowrite32(1 << index, pcie->reg_base + PCIERC_ISTATUS_MSI_ADDR);

			dt_irq = irq_find_mapping(pcie->msi_domain, index);
			if (test_bit(index, pcie->msi_irq_in_use))
				generic_handle_irq(dt_irq);
			else
				dev_info(pcie->dev, "unhandled MSI\n");

			status = ioread32(pcie->reg_base + PCIERC_ISTATUS_MSI_ADDR);

		} while (status);
	}
	iowrite32(global_pcie_status, pcie->reg_base + PCIERC_ISTATUS_LOCAL_ADDR);

	return IRQ_HANDLED;
}

static int npcm_pcie_init_irq_domain(struct npcm_pcie *pcie)
{
	struct device *dev = pcie->dev;
	struct device_node *node = dev->of_node;

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		pcie->msi_domain = irq_domain_add_linear(node, NPCM_MSI_MAX,
							 &msi_domain_ops,
							 &npcm_pcie_msi_chip);
		if (!pcie->msi_domain) {
			dev_err(dev, "Failed to get a MSI IRQ domain\n");
			return PTR_ERR(pcie->msi_domain);
		}
	}

	return 0;
}

static int npcm_pcie_rc_device_connected(struct npcm_pcie *pcie)
{
	u32 val;

	/*
	 * Check the Link status register on the bridge 
	 * configuration space at:
	 * Bus 0, Device 0, function 0 offset 0x92 bit 4
	 */   
	iowrite32(0x1F0000 , pcie->reg_base + RCCFGNUM);
	val = ioread32(pcie->config_base + 0x90);
	return  ((val & LINK_UP_FIELD) >> 20); 
}

static void npcm_initialize_as_root_complex(struct npcm_pcie *pcie)
{ 
	/* put RC core to reset (write 0 to enter reset and 1 to release) */
	regmap_write_bits(pcie->gcr_regmap, INTCR3, INTCR3_RCCORER_BIT, 0x0);

	regmap_write_bits(pcie->gcr_regmap, MFSEL3,
			  MFSEL3_PCIEPUSE_BIT, MFSEL3_PCIEPUSE_BIT);

	/* put RC to reset (write 1 to enter reset and 0 to enable module) */
	reset_control_assert(pcie->reset);

	/* release RC from reset */
	regmap_write_bits(pcie->gcr_regmap, INTCR3,
			  INTCR3_RCCORER_BIT, INTCR3_RCCORER_BIT);

	/* enable RC */
	reset_control_deassert(pcie->reset);

	/* Only for NPCM8XX set error report to no slave error */
	iowrite32(PCIERC_CFG_NO_SLVERR, pcie->reg_base + PCIERC_AXI_ERROR_REPORT);
}

static int set_translation_window(void __iomem * config_address_base,dma_addr_t *source_addr,
		u32 size,dma_addr_t *dest_addr,u8 win_type,u8 target)
{
	u8 win_size = 11 ;
	u32 val;

	if (size < 4096)
		printk("%s : ERROR : window size should be greater then 4KB\n " , __FUNCTION__);

	size=(size >> (win_size + 2));
	while(size) {
		size= (size >> 1);
		win_size++;
	}

#ifdef __LP64__
	writel(((uint64_t)source_addr & 0xffffffff) +
			(win_size << PCI_RC_ATTR_WIN_SIZE_POS)+(1 << PCI_RC_ATTR_WIN_EN_POS) , config_address_base);
	writel(((uint64_t)source_addr >> 32 ) & 0xffffffff ,config_address_base + 0x4 );
	writel(((uint64_t)dest_addr & 0xffffffff) , config_address_base + 0x8);
	writel(((uint64_t)dest_addr >> 32 ) & 0xffffffff , config_address_base + 0xc );
#else
	writel( ((u32)source_addr  ) +
			(win_size << PCI_RC_ATTR_WIN_SIZE_POS)+(1 << PCI_RC_ATTR_WIN_EN_POS) , config_address_base);
	writel(0 ,config_address_base + 0x4 );
	writel((u32)dest_addr, config_address_base + 0x8);
	writel(0, config_address_base + 0xc );
#endif
	val = (win_type << PCI_RC_ATTR_TRSF_PARAM_POS) + (target << PCI_RC_ATTR_TRSL_ID_POS);
	writel(val, config_address_base + 0x10);
	return 0;
}

static int npcm_msi_init(struct npcm_pcie *pcie)
{
	struct device *dev = pcie->dev;
	struct device_node *node = dev->of_node;
	int ret;

	/* enable MSI interrupt  */
	iowrite32(PCIERC_ISTATUS_LOCAL_MSI_BIT, pcie->reg_base + PCIERC_IMASK_LOCAL_ADDR);

	set_translation_window((void __iomem *)pcie->reg_base + RCPA0SAL, 0, SZ_512M , 0,
			        PLDA_XPRESS_RICH_MEMORY_WINDOW ,PLDA_XPRESS_RICH_TARGET_AXI_MASTER) ;

	pcie->irq = irq_of_parse_and_map(node, 0);
	if (!pcie->irq) {
		dev_err(dev, "failed to map irq\n");
		return -ENODEV;	
	}

	ret = devm_request_irq(dev, pcie->irq, npcm_msi_handler,
			       IRQF_SHARED | IRQF_NO_THREAD, "npcm-pcie-rc",
			       pcie);
	if (ret) {
		dev_err(dev, "unable to request irq %d\n", pcie->irq);
		return ret;
	}
	
	ret = npcm_pcie_init_irq_domain(pcie);
	if (ret) {
		dev_err(dev, "Failed creating IRQ Domain\n");
		return ret;
	}

	return ret;
}

static void npcm_pcie_rc_init_config_window(struct npcm_pcie *pcie)
{	
	struct of_pci_range range;
	struct of_pci_range_parser parser;
	struct device *dev = pcie->dev;
	struct device_node *node = dev->of_node;
	u32 start_win_num = 2;

	if (of_pci_range_parser_init(&parser, node))
		return;

	/* Enable configuration window */
	iowrite32((pcie->res->start & 0xFFFFF000) | (CFG_SIZE_4K << 1) | RCA_WIN_EN, pcie->reg_base + RCAPnSAL(1));
	iowrite32(0, pcie->reg_base + RCAPnSAH(1));
	iowrite32(pcie->res->start, pcie->reg_base + RCAPnTAL(1));
	iowrite32(0, pcie->reg_base + RCAPnTAH(1));
	iowrite32(TRSF_PARAM_CONFIG | TRSL_ID_PCIE_CONFIG, pcie->reg_base + RCAPnTP(1));

	for_each_of_pci_range(&parser, &range) {
		unsigned long size;
		int bit_size;

		if (start_win_num > PCI_RC_MAX_AXI_PCI_WIN)
			continue;

		size = range.size;
		bit_size = find_first_bit(&size, 32);
		switch (range.flags & IORESOURCE_TYPE_BITS) {
		case IORESOURCE_IO:
			iowrite32(range.pci_addr, pcie->reg_base + RCAPnTAL(start_win_num));
			iowrite32(0, pcie->reg_base + RCAPnTAH(start_win_num));
			iowrite32(TRSF_PARAM_IO | TRSL_ID_PCIE_CONFIG, pcie->reg_base + RCAPnTP(start_win_num));
			iowrite32(0, pcie->reg_base + RCAPnSAH(start_win_num));
			iowrite32((range.cpu_addr & 0xFFFFF000) | ((bit_size - 1) << 1) | RCA_WIN_EN, pcie->reg_base + RCAPnSAL(start_win_num));
			break;
		case IORESOURCE_MEM:
			iowrite32(range.pci_addr, pcie->reg_base + RCAPnTAL(start_win_num));
			iowrite32(0, pcie->reg_base + RCAPnTAH(start_win_num));
			iowrite32(TRSF_PARAM_MEMORY | TRSL_ID_PCIE_TX_RX, pcie->reg_base + RCAPnTP(start_win_num));
			iowrite32(0, pcie->reg_base + RCAPnSAH(start_win_num));
			iowrite32((range.cpu_addr & 0xFFFFF000) | ((bit_size - 1) << 1) | RCA_WIN_EN, pcie->reg_base + RCAPnSAL(start_win_num));
			break;
		}

		start_win_num++;
	}

}

static int npcm_config_read(struct pci_bus *bus, 
                                     unsigned int devfn, 
                                     int where, 
                                     int size, 
                                     u32 *value)
{
	struct npcm_pcie *pcie = bus->sysdata;
	if (npcm_pcie_rc_device_connected(pcie) == 0) {
		 pr_info("npcm_pcie_rc_config_read - NO LINK\n");        
		 *value = 0xFFFFFFFF;
		 return PCIBIOS_SUCCESSFUL;
	}

	if ((bus->number == 0) && (devfn == 0) && ((where & 0xFFFFFFFC) == 8)) {
		*value = 0x6040001;
	} else {		
		iowrite32(0x1F0000 | (((unsigned int)(bus->number)) << 8) | (devfn & 0xFF), pcie->reg_base + RCCFGNUM);
		*value = ioread32(pcie->config_base + (where & 0xFFFFFFFC));
	}
	 
	if (size == 1)
		*value = (*value >> (8 * (where & 3))) & 0xff;
	else if (size == 2)
		*value = (*value >> (8 * (where & 3))) & 0xffff;
	//pr_info("npcm_pcie_rc_config_read (b:%d, df:%d, o:0x%x, v:0x%x) RCCFGNUM->0x%x \n",(int)(bus->number), devfn, where, *value, ioread32(pcie->reg_base + RCCFGNUM));        

	return PCIBIOS_SUCCESSFUL;
}

static int npcm_config_write(struct pci_bus *bus, unsigned int devfn, 
					int where, int size, u32 value)
{
	struct npcm_pcie *pcie = bus->sysdata;
	u32 org_val, new_val;    
	int ret = PCIBIOS_SUCCESSFUL;

	if (((bus->number > 0) || ((bus->number == 0) && (devfn > 0))) && (npcm_pcie_rc_device_connected(pcie) == 0)) {
		pr_info("npcm_pcie_rc_config_write - NO LINK\n");        
		return ret;
	}

#ifdef CONFIG_PCI_MSI
	if((bus->number == 0) && (devfn == 0) && (where == 0x10)) {
		pcie->bar0 = value;
		iowrite32(0x1F0000 ,pcie->reg_base + RCCFGNUM);
		org_val = ioread32(pcie->config_base + PCI_CMD_STATUS_REG);
		org_val |= (PCI_COMMAND_IO | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER);
		iowrite32(org_val , pcie->config_base + PCI_CMD_STATUS_REG);
	}
#endif

	iowrite32(0x1F0000 | (((unsigned int)(bus->number)) << 8) |
		     (devfn & 0xFF), pcie->reg_base + RCCFGNUM);

	//pr_info("npcm_pcie_rc_config_write (b:%d, df:%d, o:0x%x, s:%d v:0x%x) RCCFGNUM->0x%x\n", (int)(bus->number), devfn, where, size, value, ioread32(pcie->reg_base + RCCFGNUM));

	if (size == 4) 
		iowrite32(value, pcie->config_base + (where & 0xFFFFFFFC));
	else if (size == 2) {
		org_val = ioread32(pcie->config_base + (where & 0xFFFFFFFC));
		value   = (value & 0x0000FFFF) << ((where & 0x3) * 8);
		new_val = (org_val & ~(0x0000FFFF << ((where & 0x3) * 8))) | value;
		iowrite32(new_val, pcie->config_base + (where & 0xFFFFFFFC));
	}
	else if (size == 1) {
		org_val = ioread32(pcie->config_base + (where & 0xFFFFFFFC));
		value   = (value & 0x000000FF) << ((where & 0x3) * 8);
		new_val = (org_val & ~(0x000000FF << ((where & 0x3) * 8))) | value;
		iowrite32(new_val, pcie->config_base + (where & 0xFFFFFFFC));
	}
	else 
		ret = PCIBIOS_BAD_REGISTER_NUMBER;

	return ret;
}

static struct pci_ops npcm_pcie_ops = {
	.read		= npcm_config_read,
	.write		= npcm_config_write,
	
};

static int npcm_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pci_host_bridge *bridge;
	struct npcm_pcie *pcie;
	int virq;

	bridge = devm_pci_alloc_host_bridge(dev, sizeof(struct npcm_pcie));
	if (!bridge)
		return -ENODEV;

	pcie = pci_host_bridge_priv(bridge);

	pcie->dev = dev;

	pcie->reg_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pcie->reg_base))
		return PTR_ERR(pcie->reg_base);

	pcie->res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	pcie->config_base = ioremap(pcie->res->start, resource_size(pcie->res));
	if (IS_ERR(pcie->config_base))
		return PTR_ERR(pcie->config_base);

	platform_set_drvdata(pdev, pcie);

	virq = platform_get_irq(pdev, 0);
	if (virq < 0)
		return virq;

	pcie->reset = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(pcie->reset))
		return PTR_ERR(pcie->reset);

	pcie->gcr_regmap = syscon_regmap_lookup_by_compatible("nuvoton,npcm845-gcr");
	if (IS_ERR(pcie->gcr_regmap)) {
		dev_err(&pdev->dev, "Failed to find nuvoton,npcm845-gcr\n");
		return PTR_ERR(pcie->gcr_regmap);
	}

	npcm_initialize_as_root_complex(pcie);
	npcm_pcie_rc_init_config_window(pcie);

	spin_lock_init(&pcie->used_msi_lock);

	bridge->sysdata = pcie;
	bridge->ops = &npcm_pcie_ops;

#ifdef CONFIG_PCI_MSI
	npcm_msi_init(pcie);
	npcm_pcie_msi_chip.dev = dev;
	bridge->msi = &npcm_pcie_msi_chip;
#endif
	return pci_host_probe(bridge);

	return 0;
}

static const struct of_device_id npcm_pcie_ids[] = {
	{
		.compatible = "nuvoton,npcm845-pcie", },
	{ }
};

static struct platform_driver npcm_pcie_driver = {
	.probe	= npcm_pcie_probe,
	.driver	= {
		.name = KBUILD_MODNAME,
		.of_match_table = npcm_pcie_ids,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(npcm_pcie_driver);
