/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/irqchip.h>
#include <linux/of_platform.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <linux/phy.h>
#include <linux/pm_opp.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx7-iomuxc-gpr.h>
#include <linux/memblock.h>
#include <asm/setup.h>

#include "common.h"
#include "cpuidle.h"

static int ar8031_phy_fixup(struct phy_device *dev)
{
	u16 val;

	/* Set RGMII IO voltage to 1.8V */
	phy_write(dev, 0x1d, 0x1f);
	phy_write(dev, 0x1e, 0x8);

	/* disable phy AR8031 SmartEEE function. */
	phy_write(dev, 0xd, 0x3);
	phy_write(dev, 0xe, 0x805d);
	phy_write(dev, 0xd, 0x4003);
	val = phy_read(dev, 0xe);
	val &= ~(0x1 << 8);
	phy_write(dev, 0xe, val);

	/* introduce tx clock delay */
	phy_write(dev, 0x1d, 0x5);
	val = phy_read(dev, 0x1e);
	val |= 0x0100;
	phy_write(dev, 0x1e, val);

	return 0;
}

static int bcm54220_phy_fixup(struct phy_device *dev)
{
	/* enable RXC skew select RGMII copper mode */
	phy_write(dev, 0x1e, 0x21);
	phy_write(dev, 0x1f, 0x7ea8);
	phy_write(dev, 0x1e, 0x2f);
	phy_write(dev, 0x1f, 0x71b7);

	return 0;
}

#define PHY_ID_AR8031   0x004dd074
#define PHY_ID_BCM54220	0x600d8589
#define PHY_ID_BCM5422x	0x600d8599
static void __init imx7d_enet_phy_init(void)
{
	if (IS_BUILTIN(CONFIG_PHYLIB)) {
		phy_register_fixup_for_uid(PHY_ID_AR8031, 0xffffffff,
					   ar8031_phy_fixup);
		phy_register_fixup_for_uid(PHY_ID_BCM54220, 0xffffffff,
					   bcm54220_phy_fixup);
		phy_register_fixup_for_uid(PHY_ID_BCM5422x, 0xffffffff,
					   bcm54220_phy_fixup);
	}
}

static void __init imx7d_enet_clk_sel(void)
{
	struct regmap *gpr;

	gpr = syscon_regmap_lookup_by_compatible("fsl,imx7d-iomuxc-gpr");
	if (!IS_ERR(gpr)) {
		regmap_update_bits(gpr, IOMUXC_GPR1, IMX7D_GPR1_ENET_TX_CLK_SEL_MASK, 0);
		regmap_update_bits(gpr, IOMUXC_GPR1, IMX7D_GPR1_ENET_CLK_DIR_MASK, 0);
	} else {
		pr_err("failed to find fsl,imx7d-iomux-gpr regmap\n");
	}
}

static inline void imx7d_enet_init(void)
{
	imx6_enet_mac_init("fsl,imx7d-fec");
	imx7d_enet_phy_init();
	imx7d_enet_clk_sel();
}

static void __init imx7d_init_machine(void)
{
	struct device *parent;

	mxc_arch_reset_init_dt();

	parent = imx_soc_device_init();
	if (parent == NULL)
		pr_warn("failed to initialize soc device\n");

	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
	imx7d_pm_init();
	imx_anatop_init();
	imx7d_enet_init();
}

static void __init imx7d_init_irq(void)
{
	imx_init_revision_from_anatop();
	imx_src_init();
	imx_gpcv2_init();
	irqchip_init();
}

static void __init imx7d_init_late(void)
{
	if (IS_ENABLED(CONFIG_ARM_IMX7D_CPUFREQ)) {
		platform_device_register_simple("imx7d-cpufreq", -1, NULL, 0);
	}
	imx7d_cpuidle_init();
}

static const char *imx7d_dt_compat[] __initconst = {
	"fsl,imx7d",
	NULL,
};

static void __init imx7d_map_io(void)
{
	debug_ll_io_init();
	imx7_pm_map_io();
	imx_busfreq_map_io();
}

extern unsigned long int ramoops_phys_addr;
extern unsigned long int ramoops_mem_size;
static void imx7d_reserve(void)
{
	phys_addr_t phys;
	phys_addr_t max_phys;
	struct meminfo *mi;
	struct membank *bank;

#ifdef CONFIG_PSTORE_RAM
	mi = &meminfo;
	if (!mi) {
		pr_err("no memory reserve for ramoops.\n");
		return;
	}
	/* use memmory bank 0 for ram console store */
	bank = &mi->bank[0];
	if (!bank) {
		pr_err("no memory reserve for ramoops.\n");
		return;
	}
	max_phys = bank->start + bank->size;
	/* reserve 256M for uboot avoid ram console data is cleaned by uboot */
	phys = memblock_alloc_base(SZ_1M, SZ_4K, max_phys - SZ_256M);
	if (phys) {
		memblock_remove(phys, SZ_1M);
		memblock_reserve(phys, SZ_1M);
		ramoops_phys_addr = phys;
		ramoops_mem_size = SZ_1M;
	} else {
		ramoops_phys_addr = 0;
		ramoops_mem_size = 0;
		pr_err("no memory reserve for ramoops.\n");
	}
#endif
	return;
}

DT_MACHINE_START(IMX7D, "Freescale i.MX7 Dual (Device Tree)")
	.map_io		= imx7d_map_io,
	.smp            = smp_ops(imx_smp_ops),
	.init_irq	= imx7d_init_irq,
	.init_machine	= imx7d_init_machine,
	.init_late	= imx7d_init_late,
	.dt_compat	= imx7d_dt_compat,
	.reserve     = imx7d_reserve,
	.restart	= mxc_restart,
MACHINE_END
