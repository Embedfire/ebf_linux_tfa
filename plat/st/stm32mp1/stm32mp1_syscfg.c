/*
 * Copyright (c) 2019, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <bsec.h>
#include <debug.h>
#include <mmio.h>
#include <platform_def.h>
#include <stm32mp_dt.h>
#include <stm32mp1_private.h>
#include <stpmic1.h>

/*
 * SYSCFG REGISTER OFFSET (base relative)
 */
#define SYSCFG_BOOTR				0x00U
#define SYSCFG_IOCTRLSETR			0x18U
#define SYSCFG_ICNR				0x1CU
#define SYSCFG_CMPCR				0x20U
#define SYSCFG_CMPENSETR			0x24U

/*
 * SYSCFG_BOOTR Register
 */
#define SYSCFG_BOOTR_BOOT_MASK			GENMASK(2, 0)
#define SYSCFG_BOOTR_BOOTPD_SHIFT		4
/*
 * SYSCFG_IOCTRLSETR Register
 */
#define SYSCFG_IOCTRLSETR_HSLVEN_TRACE		BIT(0)
#define SYSCFG_IOCTRLSETR_HSLVEN_QUADSPI	BIT(1)
#define SYSCFG_IOCTRLSETR_HSLVEN_ETH		BIT(2)
#define SYSCFG_IOCTRLSETR_HSLVEN_SDMMC		BIT(3)
#define SYSCFG_IOCTRLSETR_HSLVEN_SPI		BIT(4)

/*
 * SYSCFG_ICNR Register
 */
#define SYSCFG_ICNR_AXI_M9			BIT(9)

/*
 * SYSCFG_CMPCR Register
 */
#define SYSCFG_CMPCR_SW_CTRL			BIT(1)
#define SYSCFG_CMPCR_READY			BIT(8)

/*
 * SYSCFG_CMPENSETR Register
 */
#define SYSCFG_CMPENSETR_MPU_EN			BIT(0)

#define PRODUCT_BELOW_2V5			BIT(13)

void stm32mp1_syscfg_init(void)
{
	uint32_t bootr;
	uint32_t otp = 0;
	uint32_t vdd_voltage;
	uintptr_t syscfg_base = dt_get_syscfg_base();

	/*
	 * Interconnect update : select master using the port 1.
	 * LTDC = AXI_M9.
	 */
	mmio_write_32(syscfg_base + SYSCFG_ICNR, SYSCFG_ICNR_AXI_M9);
	VERBOSE("[0x%x] SYSCFG.icnr = 0x%08x (LTDC)\n",
		(uint32_t)syscfg_base + SYSCFG_ICNR,
		mmio_read_32(syscfg_base + SYSCFG_ICNR));

	/* Disable Pull-Down for boot pin connected to VDD */
	bootr = mmio_read_32(syscfg_base + SYSCFG_BOOTR);
	bootr &= ~(SYSCFG_BOOTR_BOOT_MASK << SYSCFG_BOOTR_BOOTPD_SHIFT);
	bootr |= (bootr & SYSCFG_BOOTR_BOOT_MASK) << SYSCFG_BOOTR_BOOTPD_SHIFT;
	mmio_write_32(syscfg_base + SYSCFG_BOOTR, bootr);
	VERBOSE("[0x%x] SYSCFG.bootr = 0x%08x\n",
		(uint32_t)syscfg_base + SYSCFG_BOOTR,
		mmio_read_32(syscfg_base + SYSCFG_BOOTR));

	/*
	 * High Speed Low Voltage Pad mode Enable for SPI, SDMMC, ETH, QSPI
	 * and TRACE. Needed above ~50MHz and conditioned by AFMUX selection.
	 * The customer will have to disable this for low frequencies
	 * or if AFMUX is selected but the function not used, typically for
	 * TRACE. Otherwise, impact on power consumption.
	 *
	 * WARNING:
	 *   Enabling High Speed mode while VDD > 2.7V
	 *   with the OTP product_below_2v5 (OTP 18, BIT 13)
	 *   erroneously set to 1 can damage the IC!
	 *   => TF-A sets the register only if VDD < 2.7V (in DT)
	 *      but this value needs to be consistent with board design.
	 */
	if (bsec_read_otp(&otp, HW2_OTP) != BSEC_OK) {
		otp = otp & PRODUCT_BELOW_2V5;
	}

	/* Get VDD = pwr-supply */
	vdd_voltage = dt_get_pwr_vdd_voltage();
	VERBOSE("VDD regulator voltage = %d\n", vdd_voltage);

	/* Check if VDD is Low Voltage */
	if (vdd_voltage == 0U) {
		INFO("VDD unknown");
	} else if (vdd_voltage < 2700000U) {
		mmio_write_32(syscfg_base + SYSCFG_IOCTRLSETR,
			      SYSCFG_IOCTRLSETR_HSLVEN_TRACE |
			      SYSCFG_IOCTRLSETR_HSLVEN_QUADSPI |
			      SYSCFG_IOCTRLSETR_HSLVEN_ETH |
			      SYSCFG_IOCTRLSETR_HSLVEN_SDMMC |
			      SYSCFG_IOCTRLSETR_HSLVEN_SPI);

		if (otp == 0U) {
			INFO("Product_below_2v5=0: HSLVEN protected by HW\n");
		}
	} else {
		if (otp != 0U) {
			INFO("Product_below_2v5=1: HSLVEN update is\n");
			INFO("  destructive, no update as VDD>2.7V\n");
		}
	}

	VERBOSE("[0x%x] SYSCFG.IOCTRLSETR = 0x%08x\n",
		(uint32_t)syscfg_base + SYSCFG_IOCTRLSETR,
		mmio_read_32(syscfg_base + SYSCFG_IOCTRLSETR));

	/*
	 * Activate automatic I/O compensation.
	 * Warning: need to ensure CSI enabled and ready in clock driver.
	 */
	mmio_write_32(syscfg_base + SYSCFG_CMPENSETR, SYSCFG_CMPENSETR_MPU_EN);

	while ((mmio_read_32(syscfg_base + SYSCFG_CMPCR) &
		SYSCFG_CMPCR_READY) != 0U) {
		;
	}

	mmio_clrbits_32(syscfg_base + SYSCFG_CMPCR, SYSCFG_CMPCR_SW_CTRL);

	VERBOSE("[0x%x] SYSCFG.cmpcr = 0x%08x\n",
		(uint32_t)syscfg_base + SYSCFG_CMPCR,
		mmio_read_32(syscfg_base + SYSCFG_CMPCR));
}
