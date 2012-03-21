/*
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *	Colin Cross <ccross@google.com>
 *
 * Copyright (C) 2010, NVIDIA Corporation
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>

#include <asm/hardware/gic.h>
#include <asm/mach/irq.h>

#include <mach/iomap.h>
#include <mach/suspend.h>

#include "board.h"

#define INT_SYS_NR	(INT_GPIO_BASE - INT_PRI_BASE)
#define INT_SYS_SZ	(INT_SEC_BASE - INT_PRI_BASE)
#define PPI_NR		((INT_SYS_NR+INT_SYS_SZ-1)/INT_SYS_SZ)

#define ICTLR_CPU_IEP_VFIQ	0x08
#define ICTLR_CPU_IEP_FIR	0x14
#define ICTLR_CPU_IEP_FIR_SET	0x18
#define ICTLR_CPU_IEP_FIR_CLR	0x1c

#define ICTLR_CPU_IER		0x20
#define ICTLR_CPU_IER_SET	0x24
#define ICTLR_CPU_IER_CLR	0x28
#define ICTLR_CPU_IEP_CLASS	0x2c
#define ICTLR_COP_IER		0x30
#define ICTLR_COP_IER_SET	0x34
#define ICTLR_COP_IER_CLR	0x38
#define ICTLR_COP_IEP_CLASS	0x3c

#define FIRST_LEGACY_IRQ	32

#define HOST1X_SYNC_OFFSET 0x3000
#define HOST1X_SYNC_SIZE 0x800
enum {
	HOST1X_SYNC_SYNCPT_THRESH_CPU0_INT_STATUS = 0x40,
	HOST1X_SYNC_SYNCPT_THRESH_INT_DISABLE = 0x60
};

#define irq_to_ictlr(irq) (((irq) - 32) >> 5)
static void __iomem *tegra_ictlr_base = IO_ADDRESS(TEGRA_PRIMARY_ICTLR_BASE);
#define ictlr_to_virt(ictlr) (tegra_ictlr_base + (ictlr) * 0x100)

static void tegra_mask(struct irq_data *d)
{
	void __iomem *addr = ictlr_to_virt(irq_to_ictlr(d->irq));
	if (d->irq < FIRST_LEGACY_IRQ)
		return;
	writel(1 << (d->irq & 31), addr+ICTLR_CPU_IER_CLR);
}

static void tegra_unmask(struct irq_data *d)
{
	void __iomem *addr = ictlr_to_virt(irq_to_ictlr(d->irq));
	if (d->irq < FIRST_LEGACY_IRQ)
		return;
	writel(1<<(d->irq&31), addr+ICTLR_CPU_IER_SET);
}

static void tegra_ack(struct irq_data *d)
{
	void __iomem *addr = ictlr_to_virt(irq_to_ictlr(d->irq));
	if (d->irq < FIRST_LEGACY_IRQ)
		return;
	writel(1<<(d->irq&31), addr+ICTLR_CPU_IEP_FIR_CLR);
}

static void tegra_eoi(struct irq_data *d)
{
	void __iomem *addr = ictlr_to_virt(irq_to_ictlr(d->irq));
	if (d->irq < FIRST_LEGACY_IRQ)
		return;
	writel(1<<(d->irq&31), addr+ICTLR_CPU_IEP_FIR_CLR);
}

static int tegra_set_wake(struct irq_data *data, unsigned int on)
{
	return 0; /* always allow wakeup */
}

static void syncpt_thresh_mask(struct irq_data *data)
{
	(void)data;
}

static void syncpt_thresh_unmask(struct irq_data *data)
{
	(void)data;
}

static void syncpt_thresh_cascade(unsigned int irq, struct irq_desc *desc)
{
	void __iomem *sync_regs = irq_desc_get_handler_data(desc);
	unsigned long reg;
	int id;
	struct irq_chip *chip = irq_get_chip(irq);

	chained_irq_enter(chip, desc);

	reg = readl(sync_regs + HOST1X_SYNC_SYNCPT_THRESH_CPU0_INT_STATUS);

	for_each_set_bit(id, &reg, 32)
		generic_handle_irq(id + INT_SYNCPT_THRESH_BASE);

	chained_irq_exit(chip, desc);
}

static struct irq_chip syncpt_thresh_irq = {
	.name		= "syncpt",
	.irq_mask		= syncpt_thresh_mask,
	.irq_unmask		= syncpt_thresh_unmask
};

void __init syncpt_init_irq(void)
{
	void __iomem *sync_regs;
	unsigned int i;
	int irq;

	sync_regs = ioremap(TEGRA_HOST1X_BASE + HOST1X_SYNC_OFFSET,
			HOST1X_SYNC_SIZE);
	BUG_ON(!sync_regs);

	writel(0xffffffffUL,
		sync_regs + HOST1X_SYNC_SYNCPT_THRESH_INT_DISABLE);
	writel(0xffffffffUL,
		sync_regs + HOST1X_SYNC_SYNCPT_THRESH_CPU0_INT_STATUS);

	for (i = 0; i < INT_SYNCPT_THRESH_NR; i++) {
		irq = INT_SYNCPT_THRESH_BASE + i;
		irq_set_chip_and_handler(irq, &syncpt_thresh_irq,
			handle_simple_irq);
		irq_set_chip_data(irq, sync_regs);
		set_irq_flags(irq, IRQF_VALID);
	}
	irq_set_chained_handler(INT_HOST1X_MPCORE_SYNCPT,
		syncpt_thresh_cascade);
	irq_set_handler_data(INT_HOST1X_MPCORE_SYNCPT, sync_regs);
}

void __init tegra_init_irq(void)
{
	struct irq_chip *gic;
	unsigned int i;

	for (i=0; i<PPI_NR; i++) {
		writel(~0, ictlr_to_virt(i) + ICTLR_CPU_IER_CLR);
		writel(0, ictlr_to_virt(i) + ICTLR_CPU_IEP_CLASS);
		writel(~0, ictlr_to_virt(i) + ICTLR_CPU_IEP_FIR_CLR);
	}

	gic_arch_extn.irq_ack = tegra_ack;
	gic_arch_extn.irq_eoi = tegra_eoi;
	gic_arch_extn.irq_mask = tegra_mask;
	gic_arch_extn.irq_unmask = tegra_unmask;
	gic_arch_extn.irq_set_wake = tegra_set_wake;

	gic_init(0, 29, IO_ADDRESS(TEGRA_ARM_INT_DIST_BASE),
		IO_ADDRESS(TEGRA_ARM_PERIF_BASE + 0x100));

	gic = irq_get_chip(29);

	for (i=INT_PRI_BASE; i<INT_SYNCPT_THRESH_BASE; i++) {
		irq_set_chip_and_handler(i, gic, handle_fasteoi_irq);
		set_irq_flags(i, IRQF_VALID);
	}

	syncpt_init_irq();
}

#ifdef CONFIG_PM
static u32 cop_ier[PPI_NR];
static u32 cpu_ier[PPI_NR];

void tegra_irq_suspend(void)
{
	unsigned long flags;
	int i;

	for (i=INT_PRI_BASE; i<INT_GPIO_BASE; i++) {
		struct irq_desc *desc = irq_to_desc(i);
		if (!desc) continue;
		if (irqd_is_wakeup_set(&desc->irq_data)) {
			pr_debug("irq %d is wakeup\n", i);
			continue;
		}
		disable_irq(i);
	}

	local_irq_save(flags);
	for (i=0; i<PPI_NR; i++) {
		void __iomem *ictlr = ictlr_to_virt(i);
		cpu_ier[i] = readl(ictlr + ICTLR_CPU_IER);
		cop_ier[i] = readl(ictlr + ICTLR_COP_IER);
		writel(~0, ictlr + ICTLR_COP_IER_CLR);
	}
	local_irq_restore(flags);
}

void tegra_irq_resume(void)
{
	unsigned long flags;
	int i;

	local_irq_save(flags);
	for (i=0; i<PPI_NR; i++) {
		void __iomem *ictlr = ictlr_to_virt(i);
		writel(0, ictlr + ICTLR_CPU_IEP_CLASS);
		writel(cpu_ier[i], ictlr + ICTLR_CPU_IER_SET);
		writel(0, ictlr + ICTLR_COP_IEP_CLASS);
		writel(cop_ier[i], ictlr + ICTLR_COP_IER_SET);
	}
	local_irq_restore(flags);

	for (i=INT_PRI_BASE; i<INT_GPIO_BASE; i++) {
		struct irq_desc *desc = irq_to_desc(i);
		if (!desc || irqd_is_wakeup_set(&desc->irq_data)) continue;
		enable_irq(i);
	}
}
#endif
