// SPDX-License-Identifier: GPL-2.0
/*
 * NPCM8xx I3C master driver
 *
 * Copyright (C) Nuvoton Technology corporation.
 * Based on a work from: Conor Culhane <conor.culhane@silvaco.com>
 *                       Miquel RAYNAL <miquel.raynal@bootlin.com>
 */
#define DEBUG
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/errno.h>
#include <linux/i3c/master.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

/* Master Mode Registers */
#define NPCM_I3C_MCONFIG      0x000
#define   NPCM_I3C_MCONFIG_MASTER_EN BIT(0)
#define   NPCM_I3C_MCONFIG_DISTO(x) FIELD_PREP(BIT(3), (x))
#define   NPCM_I3C_MCONFIG_HKEEP(x) FIELD_PREP(GENMASK(5, 4), (x))
#define   NPCM_I3C_MCONFIG_ODSTOP(x) FIELD_PREP(BIT(6), (x))
#define   NPCM_I3C_MCONFIG_PPBAUD(x) FIELD_PREP(GENMASK(11, 8), (x))
#define   NPCM_I3C_MCONFIG_PPLOW(x) FIELD_PREP(GENMASK(15, 12), (x))
#define   NPCM_I3C_MCONFIG_ODBAUD(x) FIELD_PREP(GENMASK(23, 16), (x))
#define   NPCM_I3C_MCONFIG_ODHPP(x) FIELD_PREP(BIT(24), (x))
#define   NPCM_I3C_MCONFIG_SKEW(x) FIELD_PREP(GENMASK(27, 25), (x))
#define   NPCM_I3C_MCONFIG_I2CBAUD(x) FIELD_PREP(GENMASK(31, 28), (x))

#define NPCM_I3C_MCTRL        0x084
#define   NPCM_I3C_MCTRL_REQUEST_MASK GENMASK(2, 0)
#define   NPCM_I3C_MCTRL_REQUEST_NONE 0
#define   NPCM_I3C_MCTRL_REQUEST_START_ADDR 1
#define   NPCM_I3C_MCTRL_REQUEST_STOP 2
#define   NPCM_I3C_MCTRL_REQUEST_IBI_ACKNACK 3
#define   NPCM_I3C_MCTRL_REQUEST_PROC_DAA 4
#define   NPCM_I3C_MCTRL_REQUEST_AUTO_IBI 7
#define   NPCM_I3C_MCTRL_TYPE_I3C 0
#define   NPCM_I3C_MCTRL_TYPE_I2C BIT(4)
#define   NPCM_I3C_MCTRL_IBIRESP_AUTO 0
#define   NPCM_I3C_MCTRL_IBIRESP_ACK_WITHOUT_BYTE 0
#define   NPCM_I3C_MCTRL_IBIRESP_ACK_WITH_BYTE BIT(7)
#define   NPCM_I3C_MCTRL_IBIRESP_NACK BIT(6)
#define   NPCM_I3C_MCTRL_IBIRESP_MANUAL GENMASK(7, 6)
#define   NPCM_I3C_MCTRL_DIR(x) FIELD_PREP(BIT(8), (x))
#define   NPCM_I3C_MCTRL_DIR_WRITE 0
#define   NPCM_I3C_MCTRL_DIR_READ 1
#define   NPCM_I3C_MCTRL_ADDR(x) FIELD_PREP(GENMASK(15, 9), (x))
#define   NPCM_I3C_MCTRL_RDTERM(x) FIELD_PREP(GENMASK(23, 16), (x))

#define NPCM_I3C_MSTATUS      0x088
#define   NPCM_I3C_MSTATUS_STATE(x) FIELD_GET(GENMASK(2, 0), (x))
#define   NPCM_I3C_MSTATUS_STATE_DAA(x) (NPCM_I3C_MSTATUS_STATE(x) == 5)
#define   NPCM_I3C_MSTATUS_STATE_IDLE(x) (NPCM_I3C_MSTATUS_STATE(x) == 0)
#define   NPCM_I3C_MSTATUS_BETWEEN(x) FIELD_GET(BIT(4), (x))
#define   NPCM_I3C_MSTATUS_NACKED(x) FIELD_GET(BIT(5), (x))
#define   NPCM_I3C_MSTATUS_IBITYPE(x) FIELD_GET(GENMASK(7, 6), (x))
#define   NPCM_I3C_MSTATUS_IBITYPE_IBI 1
#define   NPCM_I3C_MSTATUS_IBITYPE_MASTER_REQUEST 2
#define   NPCM_I3C_MSTATUS_IBITYPE_HOT_JOIN 3
#define   NPCM_I3C_MINT_SLVSTART BIT(8)
#define   NPCM_I3C_MINT_MCTRLDONE BIT(9)
#define   NPCM_I3C_MINT_COMPLETE BIT(10)
#define   NPCM_I3C_MINT_RXPEND BIT(11)
#define   NPCM_I3C_MINT_TXNOTFULL BIT(12)
#define   NPCM_I3C_MINT_IBIWON BIT(13)
#define   NPCM_I3C_MINT_ERRWARN BIT(15)
#define   NPCM_I3C_MSTATUS_SLVSTART(x) FIELD_GET(NPCM_I3C_MINT_SLVSTART, (x))
#define   NPCM_I3C_MSTATUS_MCTRLDONE(x) FIELD_GET(NPCM_I3C_MINT_MCTRLDONE, (x))
#define   NPCM_I3C_MSTATUS_COMPLETE(x) FIELD_GET(NPCM_I3C_MINT_COMPLETE, (x))
#define   NPCM_I3C_MSTATUS_RXPEND(x) FIELD_GET(NPCM_I3C_MINT_RXPEND, (x))
#define   NPCM_I3C_MSTATUS_TXNOTFULL(x) FIELD_GET(NPCM_I3C_MINT_TXNOTFULL, (x))
#define   NPCM_I3C_MSTATUS_IBIWON(x) FIELD_GET(NPCM_I3C_MINT_IBIWON, (x))
#define   NPCM_I3C_MSTATUS_ERRWARN(x) FIELD_GET(NPCM_I3C_MINT_ERRWARN, (x))
#define   NPCM_I3C_MSTATUS_IBIADDR(x) FIELD_GET(GENMASK(30, 24), (x))

#define NPCM_I3C_IBIRULES     0x08C
#define   NPCM_I3C_IBIRULES_ADDR(slot, addr) FIELD_PREP(GENMASK(29, 0), \
						       ((addr) & 0x3F) << ((slot) * 6))
#define   NPCM_I3C_IBIRULES_ADDRS 5
#define   NPCM_I3C_IBIRULES_MSB0 BIT(30)
#define   NPCM_I3C_IBIRULES_NOBYTE BIT(31)
#define   NPCM_I3C_IBIRULES_MANDBYTE 0
#define NPCM_I3C_MINTSET      0x090
#define NPCM_I3C_MINTCLR      0x094
#define NPCM_I3C_MINTMASKED   0x098
#define NPCM_I3C_MERRWARN     0x09C
#define   NPCM_I3C_MERRWARN_INVREQ	BIT(19)
#define   NPCM_I3C_MERRWARN_NACK(x) FIELD_GET(BIT(2), (x))
#define NPCM_I3C_MDMACTRL     0x0A0
#define NPCM_I3C_MDATACTRL    0x0AC
#define   NPCM_I3C_MDATACTRL_FLUSHTB BIT(0)
#define   NPCM_I3C_MDATACTRL_FLUSHRB BIT(1)
#define   NPCM_I3C_MDATACTRL_UNLOCK_TRIG BIT(3)
#define   NPCM_I3C_MDATACTRL_TXTRIG_FIFO_NOT_FULL GENMASK(5, 4)
#define   NPCM_I3C_MDATACTRL_RXTRIG_FIFO_NOT_EMPTY 0
#define   NPCM_I3C_MDATACTRL_RXCOUNT(x) FIELD_GET(GENMASK(28, 24), (x))
#define   NPCM_I3C_MDATACTRL_TXFULL BIT(30)
#define   NPCM_I3C_MDATACTRL_RXEMPTY BIT(31)

#define NPCM_I3C_MWDATAB      0x0B0
#define   NPCM_I3C_MWDATAB_END BIT(8)

#define NPCM_I3C_MWDATABE     0x0B4
#define NPCM_I3C_MWDATAH      0x0B8
#define NPCM_I3C_MWDATAHE     0x0BC
#define NPCM_I3C_MRDATAB      0x0C0
#define NPCM_I3C_MRDATAH      0x0C8
#define NPCM_I3C_MWMSG_SDR    0x0D0
#define NPCM_I3C_MRMSG_SDR    0x0D4
#define NPCM_I3C_MWMSG_DDR    0x0D8
#define NPCM_I3C_MRMSG_DDR    0x0DC

#define NPCM_I3C_MDYNADDR     0x0E4
#define   NPCM_MDYNADDR_VALID BIT(0)
#define   NPCM_MDYNADDR_ADDR(x) FIELD_PREP(GENMASK(7, 1), (x))

#define NPCM_I3C_MAX_DEVS 32

/* This parameter depends on the implementation and may be tuned */
#define NPCM_I3C_FIFO_SIZE 16

struct npcm_i3c_cmd {
	u8 addr;
	bool rnw;
	u8 *in;
	const void *out;
	unsigned int len;
	unsigned int read_len;
	bool continued;
};

struct npcm_i3c_xfer {
	struct list_head node;
	struct completion comp;
	int ret;
	unsigned int type;
	unsigned int ncmds;
	struct npcm_i3c_cmd cmds[];
};

/**
 * struct npcm_i3c_master - Silvaco I3C Master structure
 * @base: I3C master controller
 * @dev: Corresponding device
 * @regs: Memory mapping
 * @free_slots: Bit array of available slots
 * @addrs: Array containing the dynamic addresses of each attached device
 * @descs: Array of descriptors, one per attached device
 * @hj_work: Hot-join work
 * @ibi_work: IBI work
 * @irq: Main interrupt
 * @pclk: System clock
 * @fclk: Fast clock (bus)
 * @sclk: Slow clock (other events)
 * @xferqueue: Transfer queue structure
 * @xferqueue.list: List member
 * @xferqueue.cur: Current ongoing transfer
 * @xferqueue.lock: Queue lock
 * @ibi: IBI structure
 * @ibi.num_slots: Number of slots available in @ibi.slots
 * @ibi.slots: Available IBI slots
 * @ibi.tbq_slot: To be queued IBI slot
 * @ibi.lock: IBI lock
 */
struct npcm_i3c_master {
	struct i3c_master_controller base;
	struct device *dev;
	void __iomem *regs;
	u32 free_slots;
	u8 addrs[NPCM_I3C_MAX_DEVS];
	struct i3c_dev_desc *descs[NPCM_I3C_MAX_DEVS];
	struct work_struct hj_work;
	struct work_struct ibi_work;
	int irq;
	struct clk *pclk;
	struct clk *fclk;
	struct clk *sclk;
	struct {
		struct list_head list;
		struct npcm_i3c_xfer *cur;
		/* Prevent races between transfers */
		spinlock_t lock;
	} xferqueue;
	struct {
		unsigned int num_slots;
		struct i3c_dev_desc **slots;
		struct i3c_ibi_slot *tbq_slot;
		/* Prevent races within IBI handlers */
		spinlock_t lock;
	} ibi;
};

/**
 * struct npcm_i3c_i3c_dev_data - Device specific data
 * @index: Index in the master tables corresponding to this device
 * @ibi: IBI slot index in the master structure
 * @ibi_pool: IBI pool associated to this device
 */
struct npcm_i3c_i2c_dev_data {
	u8 index;
	int ibi;
	struct i3c_generic_ibi_pool *ibi_pool;
};

static bool npcm_i3c_master_error(struct npcm_i3c_master *master)
{
	u32 mstatus, merrwarn;

	mstatus = readl(master->regs + NPCM_I3C_MSTATUS);
	if (NPCM_I3C_MSTATUS_ERRWARN(mstatus)) {
		merrwarn = readl(master->regs + NPCM_I3C_MERRWARN);
		writel(merrwarn, master->regs + NPCM_I3C_MERRWARN);
		dev_err(master->dev,
			"Error condition: MSTATUS 0x%08x, MERRWARN 0x%08x\n",
			mstatus, merrwarn);

		return true;
	}

	return false;
}

static void npcm_i3c_master_enable_interrupts(struct npcm_i3c_master *master, u32 mask)
{
	writel(mask, master->regs + NPCM_I3C_MINTSET);
}

static void npcm_i3c_master_disable_interrupts(struct npcm_i3c_master *master)
{
	u32 mask = readl(master->regs + NPCM_I3C_MINTSET);

	writel(mask, master->regs + NPCM_I3C_MINTCLR);
}

static inline struct npcm_i3c_master *
to_npcm_i3c_master(struct i3c_master_controller *master)
{
	return container_of(master, struct npcm_i3c_master, base);
}

static void npcm_i3c_master_hj_work(struct work_struct *work)
{
	struct npcm_i3c_master *master;

	master = container_of(work, struct npcm_i3c_master, hj_work);
	i3c_master_do_daa(&master->base);
}

static struct i3c_dev_desc *
npcm_i3c_master_dev_from_addr(struct npcm_i3c_master *master,
			     unsigned int ibiaddr)
{
	int i;

	for (i = 0; i < NPCM_I3C_MAX_DEVS; i++)
		if (master->addrs[i] == ibiaddr)
			break;

	if (i == NPCM_I3C_MAX_DEVS)
		return NULL;

	return master->descs[i];
}

static void npcm_i3c_master_emit_stop(struct npcm_i3c_master *master)
{
	writel(NPCM_I3C_MCTRL_REQUEST_STOP, master->regs + NPCM_I3C_MCTRL);

	/*
	 * This delay is necessary after the emission of a stop, otherwise eg.
	 * repeating IBIs do not get detected. There is a note in the manual
	 * about it, stating that the stop condition might not be settled
	 * correctly if a start condition follows too rapidly.
	 */
	udelay(1);
}

static void npcm_i3c_master_clear_merrwarn(struct npcm_i3c_master *master)
{
	writel(readl(master->regs + NPCM_I3C_MERRWARN),
	       master->regs + NPCM_I3C_MERRWARN);
}

static void npcm_i3c_master_flush_fifo(struct npcm_i3c_master *master)
{
	writel(readl(master->regs + NPCM_I3C_MDATACTRL) |
			NPCM_I3C_MDATACTRL_FLUSHTB | NPCM_I3C_MDATACTRL_FLUSHRB,
			master->regs + NPCM_I3C_MDATACTRL);
}

static int npcm_i3c_master_handle_ibi(struct npcm_i3c_master *master,
				     struct i3c_dev_desc *dev)
{
	struct npcm_i3c_i2c_dev_data *data = i3c_dev_get_master_data(dev);
	struct i3c_ibi_slot *slot;
	unsigned int count;
	u32 mdatactrl;
	u8 *buf;

	slot = i3c_generic_ibi_get_free_slot(data->ibi_pool);
	if (!slot)
		return -ENOSPC;

	slot->len = 0;
	buf = slot->data;

	while (NPCM_I3C_MSTATUS_RXPEND(readl(master->regs + NPCM_I3C_MSTATUS))  &&
	       slot->len < NPCM_I3C_FIFO_SIZE) {
		mdatactrl = readl(master->regs + NPCM_I3C_MDATACTRL);
		count = NPCM_I3C_MDATACTRL_RXCOUNT(mdatactrl);
		readsl(master->regs + NPCM_I3C_MRDATAB, buf, count);
		slot->len += count;
		buf += count;
	}

	master->ibi.tbq_slot = slot;

	return 0;
}

static void npcm_i3c_master_ack_ibi(struct npcm_i3c_master *master,
				   bool mandatory_byte)
{
	unsigned int ibi_ack_nack;

	ibi_ack_nack = NPCM_I3C_MCTRL_REQUEST_IBI_ACKNACK;
	if (mandatory_byte)
		ibi_ack_nack |= NPCM_I3C_MCTRL_IBIRESP_ACK_WITH_BYTE;
	else
		ibi_ack_nack |= NPCM_I3C_MCTRL_IBIRESP_ACK_WITHOUT_BYTE;

	writel(ibi_ack_nack, master->regs + NPCM_I3C_MCTRL);
}

static void npcm_i3c_master_nack_ibi(struct npcm_i3c_master *master)
{
	writel(NPCM_I3C_MCTRL_REQUEST_IBI_ACKNACK |
	       NPCM_I3C_MCTRL_IBIRESP_NACK,
	       master->regs + NPCM_I3C_MCTRL);
}

static void npcm_i3c_master_ibi_work(struct work_struct *work)
{
	struct npcm_i3c_master *master = container_of(work, struct npcm_i3c_master, ibi_work);
	struct npcm_i3c_i2c_dev_data *data;
	unsigned int ibitype, ibiaddr;
	struct i3c_dev_desc *dev;
	u32 status, val;
	int ret;

	/* Acknowledge the incoming interrupt with the AUTOIBI mechanism */
	writel(NPCM_I3C_MCTRL_REQUEST_AUTO_IBI |
	       NPCM_I3C_MCTRL_IBIRESP_AUTO,
	       master->regs + NPCM_I3C_MCTRL);

	/* Wait for IBIWON, should take approximately 100us */
	ret = readl_relaxed_poll_timeout(master->regs + NPCM_I3C_MSTATUS, val,
					 NPCM_I3C_MSTATUS_IBIWON(val), 0, 1000);
	if (ret) {
		dev_err(master->dev, "Timeout when polling for IBIWON\n");
		writel(0, master->regs + NPCM_I3C_MCTRL);
		goto reenable_ibis;
	}

	/* Clear the interrupt status */
	writel(NPCM_I3C_MINT_IBIWON, master->regs + NPCM_I3C_MSTATUS);

	status = readl(master->regs + NPCM_I3C_MSTATUS);
	ibitype = NPCM_I3C_MSTATUS_IBITYPE(status);
	ibiaddr = NPCM_I3C_MSTATUS_IBIADDR(status);

	/* Handle the critical responses to IBI's */
	switch (ibitype) {
	case NPCM_I3C_MSTATUS_IBITYPE_IBI:
		dev = npcm_i3c_master_dev_from_addr(master, ibiaddr);
		if (!dev)
			npcm_i3c_master_nack_ibi(master);
		else
			npcm_i3c_master_handle_ibi(master, dev);
		break;
	case NPCM_I3C_MSTATUS_IBITYPE_HOT_JOIN:
		npcm_i3c_master_ack_ibi(master, false);
		break;
	case NPCM_I3C_MSTATUS_IBITYPE_MASTER_REQUEST:
		npcm_i3c_master_nack_ibi(master);
		break;
	default:
		break;
	}

	/*
	 * If an error happened, we probably got interrupted and the exchange
	 * timedout. In this case we just drop everything, emit a stop and wait
	 * for the slave to interrupt again.
	 */
	if (npcm_i3c_master_error(master)) {
		if (master->ibi.tbq_slot) {
			data = i3c_dev_get_master_data(dev);
			i3c_generic_ibi_recycle_slot(data->ibi_pool,
						     master->ibi.tbq_slot);
			master->ibi.tbq_slot = NULL;
		}

		npcm_i3c_master_emit_stop(master);

		goto reenable_ibis;
	}

	/* Handle the non critical tasks */
	switch (ibitype) {
	case NPCM_I3C_MSTATUS_IBITYPE_IBI:
		if (dev) {
			i3c_master_queue_ibi(dev, master->ibi.tbq_slot);
			master->ibi.tbq_slot = NULL;
		}
		npcm_i3c_master_emit_stop(master);
		break;
	case NPCM_I3C_MSTATUS_IBITYPE_HOT_JOIN:
		queue_work(master->base.wq, &master->hj_work);
		break;
	case NPCM_I3C_MSTATUS_IBITYPE_MASTER_REQUEST:
	default:
		break;
	}

reenable_ibis:
	npcm_i3c_master_enable_interrupts(master, NPCM_I3C_MINT_SLVSTART);
}

static irqreturn_t npcm_i3c_master_irq_handler(int irq, void *dev_id)
{
	struct npcm_i3c_master *master = (struct npcm_i3c_master *)dev_id;
	u32 active = readl(master->regs + NPCM_I3C_MINTMASKED);

	if (!NPCM_I3C_MSTATUS_SLVSTART(active))
		return IRQ_NONE;

	dev_info(master->dev, "slvstart interrupt\n");

	/* Clear the interrupt status */
	writel(NPCM_I3C_MINT_SLVSTART, master->regs + NPCM_I3C_MSTATUS);
	return IRQ_HANDLED;

	npcm_i3c_master_disable_interrupts(master);

	/* Handle the interrupt in a non atomic context */
	queue_work(master->base.wq, &master->ibi_work);

	return IRQ_HANDLED;
}

static int npcm_i3c_master_bus_init(struct i3c_master_controller *m)
{
	struct npcm_i3c_master *master = to_npcm_i3c_master(m);
	struct i3c_bus *bus = i3c_master_get_bus(m);
	struct i3c_device_info info = {};
	unsigned long fclk_rate, fclk_period;
	unsigned int pp_high_period, od_low_period, i2c_period;
	u32 ppbaud, pplow, odhpp, odbaud, i2cbaud, reg;
	u32 freq, div;
	int ret;

	/* Timings derivation */
	fclk_rate = clk_get_rate(master->fclk);
	if (!fclk_rate)
		return -EINVAL;
	fclk_period = DIV_ROUND_UP(1000000000, fclk_rate);

	pplow = 0;
	/* ppFreq = FCLK / 2 / (PPBAUD + 1)), 0 <= PPBAUD <= 15 */
	freq = fclk_rate / 2;
	div = DIV_ROUND_UP(freq, bus->scl_rate.i3c);
	div = (div == 0) ? 1 : (div > 16) ? 16 : div;
	ppbaud = div - 1;
	freq /= div;

	odhpp = 1;
	/* odFreq = ppFreq / (ODBAUD + 1), 1 <= ODBAUD <= 255 */
	/* Let odFreq = ppFreq / 3 */
	div = DIV_ROUND_UP(freq, (bus->scl_rate.i3c / 3));
	div = (div < 2) ? 2 : (div > 256) ? 256 : div;
	odbaud = div - 1;

	/* calcuate i2c_baud */
	pp_high_period = (ppbaud + 1) * fclk_period;
	od_low_period = (odbaud + 1) * pp_high_period;
	i2c_period = DIV_ROUND_UP(1000000000, bus->scl_rate.i2c);
	div = DIV_ROUND_UP(i2c_period, od_low_period);
	i2cbaud = div / 2;
	i2cbaud = (i2cbaud % 2) ? (i2cbaud + 1) : i2cbaud;

	reg = NPCM_I3C_MCONFIG_MASTER_EN |
	      NPCM_I3C_MCONFIG_DISTO(0) |
	      NPCM_I3C_MCONFIG_HKEEP(3) |
	      NPCM_I3C_MCONFIG_ODSTOP(0) |
	      NPCM_I3C_MCONFIG_PPBAUD(ppbaud) |
	      NPCM_I3C_MCONFIG_PPLOW(pplow) |
	      NPCM_I3C_MCONFIG_ODBAUD(odbaud) |
	      NPCM_I3C_MCONFIG_ODHPP(odhpp) |
	      NPCM_I3C_MCONFIG_SKEW(0) |
	      NPCM_I3C_MCONFIG_I2CBAUD(i2cbaud);
	writel(reg, master->regs + NPCM_I3C_MCONFIG);

	dev_dbg(master->dev, "fclk=%lu, period_ns=%lu\n", fclk_rate, fclk_period);
	dev_dbg(master->dev, "i3c scl_rate=%lu, i2c scl_rate=%lu\n",
			bus->scl_rate.i3c, bus->scl_rate.i2c);
	dev_dbg(master->dev, "ppbaud=%u, pplow=%u, odbaud=%u, i2cbaud=%u\n",
			ppbaud, pplow, odbaud, i2cbaud);
	dev_dbg(master->dev, "pp_high=%u, pp_low=%lu\n", pp_high_period,
			(ppbaud + 1 + pplow) * fclk_period);
	dev_dbg(master->dev, "od_high=%d, od_low=%d\n", pp_high_period, od_low_period);
	dev_dbg(master->dev, "i2c_low=%d\n", i2cbaud * od_low_period);
	dev_dbg(master->dev, "mconfig=0x%x\n", readl(master->regs + NPCM_I3C_MCONFIG));

	/* Master core's registration */
	ret = i3c_master_get_free_addr(m, 0);
	if (ret < 0)
		return ret;

	info.dyn_addr = ret;

	writel(NPCM_MDYNADDR_VALID | NPCM_MDYNADDR_ADDR(info.dyn_addr),
	       master->regs + NPCM_I3C_MDYNADDR);

	ret = i3c_master_set_info(&master->base, &info);
	if (ret)
		return ret;

	return 0;
}

static void npcm_i3c_master_bus_cleanup(struct i3c_master_controller *m)
{
	struct npcm_i3c_master *master = to_npcm_i3c_master(m);

	npcm_i3c_master_disable_interrupts(master);

	/* Disable master */
	writel(0, master->regs + NPCM_I3C_MCONFIG);
}

static int npcm_i3c_master_reserve_slot(struct npcm_i3c_master *master)
{
	unsigned int slot;

	if (!(master->free_slots & GENMASK(NPCM_I3C_MAX_DEVS - 1, 0)))
		return -ENOSPC;

	slot = ffs(master->free_slots) - 1;

	master->free_slots &= ~BIT(slot);

	return slot;
}

static void npcm_i3c_master_release_slot(struct npcm_i3c_master *master,
					unsigned int slot)
{
	master->free_slots |= BIT(slot);
}

static int npcm_i3c_master_attach_i3c_dev(struct i3c_dev_desc *dev)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct npcm_i3c_master *master = to_npcm_i3c_master(m);
	struct npcm_i3c_i2c_dev_data *data;
	int slot;

	slot = npcm_i3c_master_reserve_slot(master);
	if (slot < 0)
		return slot;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		npcm_i3c_master_release_slot(master, slot);
		return -ENOMEM;
	}

	data->ibi = -1;
	data->index = slot;
	master->addrs[slot] = dev->info.dyn_addr ? dev->info.dyn_addr :
						   dev->info.static_addr;
	master->descs[slot] = dev;

	i3c_dev_set_master_data(dev, data);

	return 0;
}

static int npcm_i3c_master_reattach_i3c_dev(struct i3c_dev_desc *dev,
					   u8 old_dyn_addr)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct npcm_i3c_master *master = to_npcm_i3c_master(m);
	struct npcm_i3c_i2c_dev_data *data = i3c_dev_get_master_data(dev);

	master->addrs[data->index] = dev->info.dyn_addr ? dev->info.dyn_addr :
							  dev->info.static_addr;

	return 0;
}

static void npcm_i3c_master_detach_i3c_dev(struct i3c_dev_desc *dev)
{
	struct npcm_i3c_i2c_dev_data *data = i3c_dev_get_master_data(dev);
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct npcm_i3c_master *master = to_npcm_i3c_master(m);

	master->addrs[data->index] = 0;
	npcm_i3c_master_release_slot(master, data->index);

	kfree(data);
}

static int npcm_i3c_master_attach_i2c_dev(struct i2c_dev_desc *dev)
{
	struct i3c_master_controller *m = i2c_dev_get_master(dev);
	struct npcm_i3c_master *master = to_npcm_i3c_master(m);
	struct npcm_i3c_i2c_dev_data *data;
	int slot;

	slot = npcm_i3c_master_reserve_slot(master);
	if (slot < 0)
		return slot;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		npcm_i3c_master_release_slot(master, slot);
		return -ENOMEM;
	}

	data->index = slot;
	master->addrs[slot] = dev->addr;

	i2c_dev_set_master_data(dev, data);

	return 0;
}

static void npcm_i3c_master_detach_i2c_dev(struct i2c_dev_desc *dev)
{
	struct npcm_i3c_i2c_dev_data *data = i2c_dev_get_master_data(dev);
	struct i3c_master_controller *m = i2c_dev_get_master(dev);
	struct npcm_i3c_master *master = to_npcm_i3c_master(m);

	npcm_i3c_master_release_slot(master, data->index);

	kfree(data);
}

static int npcm_i3c_master_readb(struct npcm_i3c_master *master, u8 *dst,
				unsigned int len)
{
	int ret, i;
	u32 reg;

	for (i = 0; i < len; i++) {
		ret = readl_poll_timeout(master->regs + NPCM_I3C_MSTATUS, reg,
					 NPCM_I3C_MSTATUS_RXPEND(reg), 0, 1000);
		if (ret)
			return ret;

		dst[i] = readl(master->regs + NPCM_I3C_MRDATAB);
	}

	return 0;
}

static int npcm_i3c_master_do_daa_locked(struct npcm_i3c_master *master,
					u8 *addrs, unsigned int *count)
{
	u64 prov_id[NPCM_I3C_MAX_DEVS] = {}, nacking_prov_id = 0;
	unsigned int dev_nb = 0, last_addr = 0;
	u32 reg;
	int ret, i;

	while (true) {
		/* Enter/proceed with DAA */
		writel(NPCM_I3C_MCTRL_REQUEST_PROC_DAA |
		       NPCM_I3C_MCTRL_TYPE_I3C |
		       NPCM_I3C_MCTRL_IBIRESP_NACK |
		       NPCM_I3C_MCTRL_DIR(NPCM_I3C_MCTRL_DIR_WRITE),
		       master->regs + NPCM_I3C_MCTRL);

		/*
		 * Either one slave will send its ID, or the assignment process
		 * is done.
		 */
		ret = readl_poll_timeout_atomic(master->regs + NPCM_I3C_MSTATUS, reg,
					 NPCM_I3C_MSTATUS_RXPEND(reg) |
					 NPCM_I3C_MSTATUS_MCTRLDONE(reg),
					 1, 1000);
		if (ret)
			return ret;

		if (NPCM_I3C_MSTATUS_RXPEND(reg)) {
			u8 data[6];

			/*
			 * We only care about the 48-bit provisional ID yet to
			 * be sure a device does not nack an address twice.
			 * Otherwise, we would just need to flush the RX FIFO.
			 */
			ret = npcm_i3c_master_readb(master, data, 6);
			if (ret)
				return ret;

			for (i = 0; i < 6; i++)
				prov_id[dev_nb] |= (u64)(data[i]) << (8 * (5 - i));

			/* We do not care about the BCR and DCR yet */
			ret = npcm_i3c_master_readb(master, data, 2);
			if (ret)
				return ret;
		} else if (NPCM_I3C_MSTATUS_MCTRLDONE(reg)) {
			if (NPCM_I3C_MSTATUS_STATE_IDLE(reg) &&
			    NPCM_I3C_MSTATUS_COMPLETE(reg)) {
				/*
				 * All devices received and acked they dynamic
				 * address, this is the natural end of the DAA
				 * procedure.
				 */
				break;
			} else if (NPCM_I3C_MSTATUS_NACKED(reg)) {
				/*
				 * A slave device nacked the address, this is
				 * allowed only once, DAA will be stopped and
				 * then resumed. The same device is supposed to
				 * answer again immediately and shall ack the
				 * address this time.
				 */
				if (prov_id[dev_nb] == nacking_prov_id)
					return -EIO;

				dev_nb--;
				nacking_prov_id = prov_id[dev_nb];
				npcm_i3c_master_emit_stop(master);

				continue;
			} else {
				return -EIO;
			}
		}

		/* Wait for the slave to be ready to receive its address */
		ret = readl_poll_timeout(master->regs + NPCM_I3C_MSTATUS, reg,
					 NPCM_I3C_MSTATUS_MCTRLDONE(reg) &&
					 NPCM_I3C_MSTATUS_STATE_DAA(reg) &&
					 NPCM_I3C_MSTATUS_BETWEEN(reg),
					 0, 1000);
		if (ret)
			return ret;

		/* Give the slave device a suitable dynamic address */
		ret = i3c_master_get_free_addr(&master->base, last_addr + 1);
		if (ret < 0)
			return ret;

		addrs[dev_nb] = ret;
		dev_dbg(master->dev, "DAA: device %d assigned to 0x%02x\n",
			dev_nb, addrs[dev_nb]);

		writel(addrs[dev_nb], master->regs + NPCM_I3C_MWDATAB);
		last_addr = addrs[dev_nb++];
	}

	*count = dev_nb;

	return 0;
}

static int npcm_i3c_update_ibirules(struct npcm_i3c_master *master)
{
	struct i3c_dev_desc *dev;
	u32 reg_mbyte = 0, reg_nobyte = NPCM_I3C_IBIRULES_NOBYTE;
	unsigned int mbyte_addr_ok = 0, mbyte_addr_ko = 0, nobyte_addr_ok = 0,
		nobyte_addr_ko = 0;
	bool list_mbyte = false, list_nobyte = false;

	/* Create the IBIRULES register for both cases */
	i3c_bus_for_each_i3cdev(&master->base.bus, dev) {
		if (I3C_BCR_DEVICE_ROLE(dev->info.bcr) == I3C_BCR_I3C_MASTER)
			continue;

		if (dev->info.bcr & I3C_BCR_IBI_PAYLOAD) {
			reg_mbyte |= NPCM_I3C_IBIRULES_ADDR(mbyte_addr_ok,
							   dev->info.dyn_addr);

			/* IBI rules cannot be applied to devices with MSb=1 */
			if (dev->info.dyn_addr & BIT(7))
				mbyte_addr_ko++;
			else
				mbyte_addr_ok++;
		} else {
			reg_nobyte |= NPCM_I3C_IBIRULES_ADDR(nobyte_addr_ok,
							    dev->info.dyn_addr);

			/* IBI rules cannot be applied to devices with MSb=1 */
			if (dev->info.dyn_addr & BIT(7))
				nobyte_addr_ko++;
			else
				nobyte_addr_ok++;
		}
	}

	/* Device list cannot be handled by hardware */
	if (!mbyte_addr_ko && mbyte_addr_ok <= NPCM_I3C_IBIRULES_ADDRS)
		list_mbyte = true;

	if (!nobyte_addr_ko && nobyte_addr_ok <= NPCM_I3C_IBIRULES_ADDRS)
		list_nobyte = true;

	/* No list can be properly handled, return an error */
	if (!list_mbyte && !list_nobyte)
		return -ERANGE;

	/* Pick the first list that can be handled by hardware, randomly */
	if (list_mbyte)
		writel(reg_mbyte, master->regs + NPCM_I3C_IBIRULES);
	else
		writel(reg_nobyte, master->regs + NPCM_I3C_IBIRULES);

	return 0;
}

static int npcm_i3c_master_do_daa(struct i3c_master_controller *m)
{
	struct npcm_i3c_master *master = to_npcm_i3c_master(m);
	u8 addrs[NPCM_I3C_MAX_DEVS];
	unsigned long flags;
	unsigned int dev_nb;
	int ret, i;

	spin_lock_irqsave(&master->xferqueue.lock, flags);
	ret = npcm_i3c_master_do_daa_locked(master, addrs, &dev_nb);
	spin_unlock_irqrestore(&master->xferqueue.lock, flags);
	if (ret)
		goto emit_stop;

	/* Register all devices who participated to the core */
	for (i = 0; i < dev_nb; i++) {
		ret = i3c_master_add_i3c_dev_locked(m, addrs[i]);
		if (ret) {
			dev_err(master->dev, "Cannot add i3c dev 0x%x\n", addrs[i]);
			npcm_i3c_master_clear_merrwarn(master);
			continue;
		}
	}

	/* Configure IBI auto-rules */
	ret = npcm_i3c_update_ibirules(master);
	if (ret) {
		dev_err(master->dev, "Cannot handle such a list of devices");
		return ret;
	}

	return 0;

emit_stop:
	npcm_i3c_master_emit_stop(master);
	npcm_i3c_master_clear_merrwarn(master);
	if (ret == -EIO)
		return 0;
	else
		return ret;
}

static int npcm_i3c_master_read(struct npcm_i3c_master *master,
			       u8 *in, unsigned int *len)
{
	int offset = 0, i, ret;
	u32 mdctrl, reg;

	while (offset < *len) {
		unsigned int count;

		ret = readl_poll_timeout(master->regs + NPCM_I3C_MDATACTRL,
					 mdctrl,
					 !(mdctrl & NPCM_I3C_MDATACTRL_RXEMPTY),
					 0, 1000);
		if (ret) {
			reg = readl(master->regs + NPCM_I3C_MSTATUS);
			if (NPCM_I3C_MSTATUS_COMPLETE(reg)) {
				*len = offset;
				return 0;
			} else
				return ret;
		}

		count = NPCM_I3C_MDATACTRL_RXCOUNT(mdctrl);
		for (i = 0; i < count; i++)
			in[offset + i] = readl(master->regs + NPCM_I3C_MRDATAB);

		offset += count;
	}

	return 0;
}

static int npcm_i3c_master_write(struct npcm_i3c_master *master,
				const u8 *out, unsigned int len)
{
	int offset = 0, ret;
	u32 mdctrl;

	while (offset < len) {
		ret = readl_poll_timeout(master->regs + NPCM_I3C_MDATACTRL,
					 mdctrl,
					 !(mdctrl & NPCM_I3C_MDATACTRL_TXFULL),
					 0, 1000);
		if (ret)
			return ret;

		/*
		 * The last byte to be sent over the bus must either have the
		 * "end" bit set or be written in MWDATABE.
		 */
		if (likely(offset < (len - 1)))
			writel(out[offset++], master->regs + NPCM_I3C_MWDATAB);
		else
			writel(out[offset++], master->regs + NPCM_I3C_MWDATABE);
	}

	return 0;
}

static int npcm_i3c_master_xfer(struct npcm_i3c_master *master,
			       bool rnw, unsigned int xfer_type, u8 addr,
			       u8 *in, const u8 *out, unsigned int xfer_len,
			       unsigned int *read_len, bool continued)
{
	u32 reg;
	int ret;

	npcm_i3c_master_flush_fifo(master);

	/* pre-writing data to tx fifo if this is a write operation */
	if (!rnw && (xfer_len > 0)) {
		ret = readl_poll_timeout(master->regs + NPCM_I3C_MDATACTRL,
					 reg,
					 !(reg & NPCM_I3C_MDATACTRL_TXFULL),
					 0, 1000);
		if (ret)
			return ret;

		if (xfer_len == 1)
			writel(out[0], master->regs + NPCM_I3C_MWDATABE);
		else
			writel(out[0], master->regs + NPCM_I3C_MWDATAB);
		xfer_len--;
		out++;
	}

	writel(NPCM_I3C_MCTRL_REQUEST_START_ADDR |
	       xfer_type |
	       NPCM_I3C_MCTRL_IBIRESP_NACK |
	       NPCM_I3C_MCTRL_DIR(rnw) |
	       NPCM_I3C_MCTRL_ADDR(addr) |
	       NPCM_I3C_MCTRL_RDTERM(*read_len),
	       master->regs + NPCM_I3C_MCTRL);

	ret = readl_poll_timeout(master->regs + NPCM_I3C_MSTATUS, reg,
				 NPCM_I3C_MSTATUS_MCTRLDONE(reg), 0, 1000);
	if (ret) {
		dev_err(master->dev, "mctrl timeout, addr 0x%x\n", addr);
		goto emit_stop;
	}

	reg = readl(master->regs + NPCM_I3C_MSTATUS);
	if (NPCM_I3C_MSTATUS_NACKED(reg)) {
		dev_dbg(master->dev, "addr 0x%x NACKEd\n", addr);
		goto emit_stop;
	}

	if (rnw) {
		if (*read_len > 0)
			ret = npcm_i3c_master_read(master, in, read_len);
		else
			ret = npcm_i3c_master_read(master, in, &xfer_len);
	} else
		ret = npcm_i3c_master_write(master, out, xfer_len);
	if (ret) {
		dev_err(master->dev, "dev addr 0x%x: %s timeout, xfer_len=%d\n",
			addr, rnw?"read":"write", xfer_len);
		goto emit_stop;
	}

	ret = readl_poll_timeout(master->regs + NPCM_I3C_MSTATUS, reg,
				 NPCM_I3C_MSTATUS_COMPLETE(reg), 0, 1000);
	if (ret) {
		dev_err(master->dev, "dev addr 0x%x complete timeout\n", addr);
		goto emit_stop;
	}

	if (!continued)
		npcm_i3c_master_emit_stop(master);

	return 0;

emit_stop:
	npcm_i3c_master_flush_fifo(master);
	reg = readl(master->regs + NPCM_I3C_MERRWARN);
	if (NPCM_I3C_MERRWARN_NACK(reg))
		ret = I3C_ERROR_M2;
	npcm_i3c_master_emit_stop(master);
	npcm_i3c_master_clear_merrwarn(master);

	return ret;
}

static struct npcm_i3c_xfer *
npcm_i3c_master_alloc_xfer(struct npcm_i3c_master *master, unsigned int ncmds)
{
	struct npcm_i3c_xfer *xfer;

	xfer = kzalloc(struct_size(xfer, cmds, ncmds), GFP_KERNEL);
	if (!xfer)
		return NULL;

	INIT_LIST_HEAD(&xfer->node);
	xfer->ncmds = ncmds;
	xfer->ret = -ETIMEDOUT;

	return xfer;
}

static void npcm_i3c_master_free_xfer(struct npcm_i3c_xfer *xfer)
{
	kfree(xfer);
}

static void npcm_i3c_master_dequeue_xfer_locked(struct npcm_i3c_master *master,
					       struct npcm_i3c_xfer *xfer)
{
	if (master->xferqueue.cur == xfer)
		master->xferqueue.cur = NULL;
	else
		list_del_init(&xfer->node);
}

static void npcm_i3c_master_dequeue_xfer(struct npcm_i3c_master *master,
					struct npcm_i3c_xfer *xfer)
{
	unsigned long flags;

	spin_lock_irqsave(&master->xferqueue.lock, flags);
	npcm_i3c_master_dequeue_xfer_locked(master, xfer);
	spin_unlock_irqrestore(&master->xferqueue.lock, flags);
}

static void npcm_i3c_master_start_xfer_locked(struct npcm_i3c_master *master)
{
	struct npcm_i3c_xfer *xfer = master->xferqueue.cur;
	int ret, i;

	if (!xfer)
		return;

	for (i = 0; i < xfer->ncmds; i++) {
		struct npcm_i3c_cmd *cmd = &xfer->cmds[i];

		ret = npcm_i3c_master_xfer(master, cmd->rnw, xfer->type,
					  cmd->addr, cmd->in, cmd->out,
					  cmd->len, &cmd->read_len,
					  cmd->continued);
		if (ret)
			break;
	}

	xfer->ret = ret;
	complete(&xfer->comp);

	if (ret < 0)
		npcm_i3c_master_dequeue_xfer_locked(master, xfer);

	xfer = list_first_entry_or_null(&master->xferqueue.list,
					struct npcm_i3c_xfer,
					node);
	if (xfer)
		list_del_init(&xfer->node);

	master->xferqueue.cur = xfer;
	npcm_i3c_master_start_xfer_locked(master);
}

static void npcm_i3c_master_enqueue_xfer(struct npcm_i3c_master *master,
					struct npcm_i3c_xfer *xfer)
{
	unsigned long flags;

	init_completion(&xfer->comp);
	spin_lock_irqsave(&master->xferqueue.lock, flags);
	if (master->xferqueue.cur) {
		list_add_tail(&xfer->node, &master->xferqueue.list);
	} else {
		master->xferqueue.cur = xfer;
		npcm_i3c_master_start_xfer_locked(master);
	}
	spin_unlock_irqrestore(&master->xferqueue.lock, flags);
}

static bool
npcm_i3c_master_supports_ccc_cmd(struct i3c_master_controller *master,
				const struct i3c_ccc_cmd *cmd)
{
	/* No software support for CCC commands targeting more than one slave */
	return (cmd->ndests == 1);
}

static int npcm_i3c_master_send_bdcast_ccc_cmd(struct npcm_i3c_master *master,
					      struct i3c_ccc_cmd *ccc)
{
	unsigned int xfer_len = ccc->dests[0].payload.len + 1;
	struct npcm_i3c_xfer *xfer;
	struct npcm_i3c_cmd *cmd;
	u8 *buf;
	int ret;

	xfer = npcm_i3c_master_alloc_xfer(master, 1);
	if (!xfer)
		return -ENOMEM;

	buf = kmalloc(xfer_len, GFP_KERNEL);
	if (!buf) {
		npcm_i3c_master_free_xfer(xfer);
		return -ENOMEM;
	}

	buf[0] = ccc->id;
	memcpy(&buf[1], ccc->dests[0].payload.data, ccc->dests[0].payload.len);

	xfer->type = NPCM_I3C_MCTRL_TYPE_I3C;

	cmd = &xfer->cmds[0];
	cmd->addr = ccc->dests[0].addr;
	cmd->rnw = ccc->rnw;
	cmd->in = NULL;
	cmd->out = buf;
	cmd->len = xfer_len;
	cmd->read_len = 0;
	cmd->continued = false;

	npcm_i3c_master_enqueue_xfer(master, xfer);
	if (!wait_for_completion_timeout(&xfer->comp, msecs_to_jiffies(1000))) {
		npcm_i3c_master_dequeue_xfer(master, xfer);
		xfer->ret = -ETIMEDOUT;
	}

	ret = xfer->ret;
	kfree(buf);
	npcm_i3c_master_free_xfer(xfer);

	return ret;
}

static int npcm_i3c_master_send_direct_ccc_cmd(struct npcm_i3c_master *master,
					      struct i3c_ccc_cmd *ccc)
{
	unsigned int xfer_len = ccc->dests[0].payload.len;
	unsigned int read_len = ccc->rnw ? xfer_len : 0;
	struct npcm_i3c_xfer *xfer;
	struct npcm_i3c_cmd *cmd;
	int ret;

	xfer = npcm_i3c_master_alloc_xfer(master, 2);
	if (!xfer)
		return -ENOMEM;

	xfer->type = NPCM_I3C_MCTRL_TYPE_I3C;

	/* Broadcasted message */
	cmd = &xfer->cmds[0];
	cmd->addr = I3C_BROADCAST_ADDR;
	cmd->rnw = 0;
	cmd->in = NULL;
	cmd->out = &ccc->id;
	cmd->len = 1;
	cmd->read_len = 0;
	cmd->continued = true;

	/* Directed message */
	cmd = &xfer->cmds[1];
	cmd->addr = ccc->dests[0].addr;
	cmd->rnw = ccc->rnw;
	cmd->in = ccc->rnw ? ccc->dests[0].payload.data : NULL;
	cmd->out = ccc->rnw ? NULL : ccc->dests[0].payload.data,
	cmd->len = xfer_len;
	cmd->read_len = read_len;
	cmd->continued = false;

	npcm_i3c_master_enqueue_xfer(master, xfer);
	if (!wait_for_completion_timeout(&xfer->comp, msecs_to_jiffies(1000)))
		npcm_i3c_master_dequeue_xfer(master, xfer);

	/* partial data is returned */
	if (read_len && cmd->read_len < read_len)
		ccc->dests[0].payload.len = cmd->read_len;

	ret = xfer->ret;
	npcm_i3c_master_free_xfer(xfer);

	return ret;
}

static int npcm_i3c_master_send_ccc_cmd(struct i3c_master_controller *m,
				       struct i3c_ccc_cmd *cmd)
{
	struct npcm_i3c_master *master = to_npcm_i3c_master(m);
	bool broadcast = cmd->id < 0x80;
	int ret;

	if (broadcast)
		ret = npcm_i3c_master_send_bdcast_ccc_cmd(master, cmd);
	else
		ret = npcm_i3c_master_send_direct_ccc_cmd(master, cmd);

	if (ret != 0)
		dev_dbg(master->dev, "send ccc 0x%02x %s, ret = %d\n",
				cmd->id, broadcast ? "(broadcast)" : "", ret);

	return ret;
}

static int npcm_i3c_master_priv_xfers(struct i3c_dev_desc *dev,
				     struct i3c_priv_xfer *xfers,
				     int nxfers)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct npcm_i3c_master *master = to_npcm_i3c_master(m);
	struct npcm_i3c_i2c_dev_data *data = i3c_dev_get_master_data(dev);
	struct npcm_i3c_xfer *xfer;
	int ret, i;

	xfer = npcm_i3c_master_alloc_xfer(master, nxfers);
	if (!xfer)
		return -ENOMEM;

	xfer->type = NPCM_I3C_MCTRL_TYPE_I3C;

	for (i = 0; i < nxfers; i++) {
		struct npcm_i3c_cmd *cmd = &xfer->cmds[i];

		cmd->addr = master->addrs[data->index];
		cmd->rnw = xfers[i].rnw;
		cmd->in = xfers[i].rnw ? xfers[i].data.in : NULL;
		cmd->out = xfers[i].rnw ? NULL : xfers[i].data.out;
		cmd->len = xfers[i].len;
		cmd->read_len = xfers[i].rnw ? xfers[i].len : 0;
		cmd->continued = (i + 1) < nxfers;
	}

	npcm_i3c_master_enqueue_xfer(master, xfer);
	if (!wait_for_completion_timeout(&xfer->comp, msecs_to_jiffies(1000)))
		npcm_i3c_master_dequeue_xfer(master, xfer);

	ret = xfer->ret;
	npcm_i3c_master_free_xfer(xfer);

	return ret;
}

static int npcm_i3c_master_i2c_xfers(struct i2c_dev_desc *dev,
				    const struct i2c_msg *xfers,
				    int nxfers)
{
	struct i3c_master_controller *m = i2c_dev_get_master(dev);
	struct npcm_i3c_master *master = to_npcm_i3c_master(m);
	struct npcm_i3c_i2c_dev_data *data = i2c_dev_get_master_data(dev);
	struct npcm_i3c_xfer *xfer;
	int ret, i;

	xfer = npcm_i3c_master_alloc_xfer(master, nxfers);
	if (!xfer)
		return -ENOMEM;

	xfer->type = NPCM_I3C_MCTRL_TYPE_I2C;

	for (i = 0; i < nxfers; i++) {
		struct npcm_i3c_cmd *cmd = &xfer->cmds[i];

		cmd->addr = master->addrs[data->index];
		cmd->rnw = xfers[i].flags & I2C_M_RD;
		cmd->in = cmd->rnw ? xfers[i].buf : NULL;
		cmd->out = cmd->rnw ? NULL : xfers[i].buf;
		cmd->len = xfers[i].len;
		cmd->read_len = cmd->rnw ? xfers[i].len : 0;
		cmd->continued = (i + 1 < nxfers);
	}

	npcm_i3c_master_enqueue_xfer(master, xfer);
	if (!wait_for_completion_timeout(&xfer->comp, msecs_to_jiffies(1000)))
		npcm_i3c_master_dequeue_xfer(master, xfer);

	ret = xfer->ret;
	npcm_i3c_master_free_xfer(xfer);

	return ret;
}

static int npcm_i3c_master_request_ibi(struct i3c_dev_desc *dev,
				      const struct i3c_ibi_setup *req)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct npcm_i3c_master *master = to_npcm_i3c_master(m);
	struct npcm_i3c_i2c_dev_data *data = i3c_dev_get_master_data(dev);
	unsigned long flags;
	unsigned int i;

	if (dev->ibi->max_payload_len > NPCM_I3C_FIFO_SIZE) {
		dev_err(master->dev, "IBI max payload %d should be < %d\n",
			dev->ibi->max_payload_len, NPCM_I3C_FIFO_SIZE);
		return -ERANGE;
	}

	data->ibi_pool = i3c_generic_ibi_alloc_pool(dev, req);
	if (IS_ERR(data->ibi_pool))
		return PTR_ERR(data->ibi_pool);

	spin_lock_irqsave(&master->ibi.lock, flags);
	for (i = 0; i < master->ibi.num_slots; i++) {
		if (!master->ibi.slots[i]) {
			data->ibi = i;
			master->ibi.slots[i] = dev;
			break;
		}
	}
	spin_unlock_irqrestore(&master->ibi.lock, flags);

	if (i < master->ibi.num_slots)
		return 0;

	i3c_generic_ibi_free_pool(data->ibi_pool);
	data->ibi_pool = NULL;

	return -ENOSPC;
}

static void npcm_i3c_master_free_ibi(struct i3c_dev_desc *dev)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct npcm_i3c_master *master = to_npcm_i3c_master(m);
	struct npcm_i3c_i2c_dev_data *data = i3c_dev_get_master_data(dev);
	unsigned long flags;

	spin_lock_irqsave(&master->ibi.lock, flags);
	master->ibi.slots[data->ibi] = NULL;
	data->ibi = -1;
	spin_unlock_irqrestore(&master->ibi.lock, flags);

	i3c_generic_ibi_free_pool(data->ibi_pool);
}

static int npcm_i3c_master_enable_ibi(struct i3c_dev_desc *dev)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);

	return i3c_master_enec_locked(m, dev->info.dyn_addr, I3C_CCC_EVENT_SIR);
}

static int npcm_i3c_master_disable_ibi(struct i3c_dev_desc *dev)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);

	return i3c_master_disec_locked(m, dev->info.dyn_addr, I3C_CCC_EVENT_SIR);
}

static void npcm_i3c_master_recycle_ibi_slot(struct i3c_dev_desc *dev,
					    struct i3c_ibi_slot *slot)
{
	struct npcm_i3c_i2c_dev_data *data = i3c_dev_get_master_data(dev);

	i3c_generic_ibi_recycle_slot(data->ibi_pool, slot);
}

static const struct i3c_master_controller_ops npcm_i3c_master_ops = {
	.bus_init = npcm_i3c_master_bus_init,
	.bus_cleanup = npcm_i3c_master_bus_cleanup,
	.attach_i3c_dev = npcm_i3c_master_attach_i3c_dev,
	.detach_i3c_dev = npcm_i3c_master_detach_i3c_dev,
	.reattach_i3c_dev = npcm_i3c_master_reattach_i3c_dev,
	.attach_i2c_dev = npcm_i3c_master_attach_i2c_dev,
	.detach_i2c_dev = npcm_i3c_master_detach_i2c_dev,
	.do_daa = npcm_i3c_master_do_daa,
	.supports_ccc_cmd = npcm_i3c_master_supports_ccc_cmd,
	.send_ccc_cmd = npcm_i3c_master_send_ccc_cmd,
	.priv_xfers = npcm_i3c_master_priv_xfers,
	.i2c_xfers = npcm_i3c_master_i2c_xfers,
	.request_ibi = npcm_i3c_master_request_ibi,
	.free_ibi = npcm_i3c_master_free_ibi,
	.recycle_ibi_slot = npcm_i3c_master_recycle_ibi_slot,
	.enable_ibi = npcm_i3c_master_enable_ibi,
	.disable_ibi = npcm_i3c_master_disable_ibi,
};

static void npcm_i3c_master_reset(struct npcm_i3c_master *master)
{
	u32 reg;

	/* Clear pending warnings */
	writel(readl(master->regs + NPCM_I3C_MERRWARN),
	       master->regs + NPCM_I3C_MERRWARN);

	/* Set RX and TX tigger levels, flush FIFOs */
	reg = NPCM_I3C_MDATACTRL_FLUSHTB |
	      NPCM_I3C_MDATACTRL_FLUSHRB |
	      NPCM_I3C_MDATACTRL_UNLOCK_TRIG |
	      NPCM_I3C_MDATACTRL_TXTRIG_FIFO_NOT_FULL |
	      NPCM_I3C_MDATACTRL_RXTRIG_FIFO_NOT_EMPTY;
	writel(reg, master->regs + NPCM_I3C_MDATACTRL);

	npcm_i3c_master_disable_interrupts(master);
}

static int npcm_i3c_master_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct npcm_i3c_master *master;
	int ret;

	master = devm_kzalloc(dev, sizeof(*master), GFP_KERNEL);
	if (!master)
		return -ENOMEM;

	master->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(master->regs))
		return PTR_ERR(master->regs);

	master->pclk = devm_clk_get(dev, "pclk");
	if (IS_ERR(master->pclk))
		return PTR_ERR(master->pclk);

	master->fclk = devm_clk_get(dev, "fast_clk");
	if (IS_ERR(master->fclk))
		return PTR_ERR(master->fclk);

	master->sclk = devm_clk_get(dev, "slow_clk");
	if (IS_ERR(master->sclk))
		master->sclk = NULL;

	master->irq = platform_get_irq(pdev, 0);
	if (master->irq <= 0)
		return -ENOENT;

	master->dev = dev;

	npcm_i3c_master_reset(master);

	ret = clk_prepare_enable(master->pclk);
	if (ret)
		return ret;

	ret = clk_prepare_enable(master->fclk);
	if (ret)
		goto err_disable_pclk;

	if (master->sclk) {
		ret = clk_prepare_enable(master->sclk);
		if (ret)
			goto err_disable_fclk;
	}

	INIT_WORK(&master->hj_work, npcm_i3c_master_hj_work);
	INIT_WORK(&master->ibi_work, npcm_i3c_master_ibi_work);
	ret = devm_request_irq(dev, master->irq, npcm_i3c_master_irq_handler,
			       IRQF_NO_SUSPEND, "npcm-i3c-irq", master);
	if (ret)
		goto err_disable_sclk;

	master->free_slots = GENMASK(NPCM_I3C_MAX_DEVS - 1, 0);

	spin_lock_init(&master->xferqueue.lock);
	INIT_LIST_HEAD(&master->xferqueue.list);

	spin_lock_init(&master->ibi.lock);
	master->ibi.num_slots = NPCM_I3C_MAX_DEVS;
	master->ibi.slots = devm_kcalloc(&pdev->dev, master->ibi.num_slots,
					 sizeof(*master->ibi.slots),
					 GFP_KERNEL);
	if (!master->ibi.slots) {
		ret = -ENOMEM;
		goto err_disable_sclk;
	}

	platform_set_drvdata(pdev, master);

	/* Register the master */
	ret = i3c_master_register(&master->base, &pdev->dev,
				  &npcm_i3c_master_ops, false);
	if (ret)
		goto err_disable_sclk;

	dev_info(&pdev->dev, "probe OK\n");
	/* TODO: fix unexpected SLVSTART intterupt */
	/* npcm_i3c_master_enable_interrupts(master, NPCM_I3C_MINT_SLVSTART); */
	return 0;

err_disable_sclk:
	if (master->sclk)
		clk_disable_unprepare(master->sclk);

err_disable_fclk:
	clk_disable_unprepare(master->fclk);

err_disable_pclk:
	clk_disable_unprepare(master->pclk);

	return ret;
}

static int npcm_i3c_master_remove(struct platform_device *pdev)
{
	struct npcm_i3c_master *master = platform_get_drvdata(pdev);
	int ret;

	ret = i3c_master_unregister(&master->base);
	if (ret)
		return ret;

	free_irq(master->irq, master);
	clk_disable_unprepare(master->pclk);
	clk_disable_unprepare(master->fclk);
	if (master->sclk)
		clk_disable_unprepare(master->sclk);

	return 0;
}

static const struct of_device_id npcm_i3c_master_of_match_tbl[] = {
	{ .compatible = "nuvoton,npcm845-i3c" },
	{ /* sentinel */ },
};

static struct platform_driver npcm_i3c_master = {
	.probe = npcm_i3c_master_probe,
	.remove = npcm_i3c_master_remove,
	.driver = {
		.name = "nuvoton-i3c",
		.of_match_table = npcm_i3c_master_of_match_tbl,
	},
};
module_platform_driver(npcm_i3c_master);

MODULE_AUTHOR("Conor Culhane <conor.culhane@silvaco.com>");
MODULE_AUTHOR("Miquel Raynal <miquel.raynal@bootlin.com>");
MODULE_AUTHOR("Stanley Chu <yschu@nuvoton.com>");
MODULE_DESCRIPTION("NPCM8xx I3C master driver");
MODULE_LICENSE("GPL v2");
