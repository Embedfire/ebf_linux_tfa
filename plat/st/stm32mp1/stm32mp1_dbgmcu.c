/*
 * Copyright (c) 2016-2018, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <bsec.h>
#include <debug.h>
#include <dt-bindings/clock/stm32mp1-clks.h>
#include <errno.h>
#include <mmio.h>
#include <platform_def.h>
#include <stm32mp1_dbgmcu.h>
#include <stm32mp1_rcc.h>
#include <stm32mp1_shared_resources.h>
#include <utils_def.h>

#define DBGMCU_IDC		0x0U
#define IDC_DEV_ID_MASK		GENMASK(11, 0)
#define DBGMCU_APB4FZ1		0x2CU
#define DBGMCU_APB4FZ1_IWDG2	BIT(2)

#define TAMP_DBG_BACKUP_REG_ID	20
#define TAMP_DBG_DEBUG		BIT(16)

static uintptr_t get_rcc_base(void)
{
	/* This is called before stm32mp_rcc_base() is available */
	return RCC_BASE;
}

static int stm32mp1_dbgmcu_init(void)
{
	uint32_t dbg_conf;
	uintptr_t rcc_base = get_rcc_base();

	dbg_conf = bsec_read_debug_conf();

	if ((dbg_conf & BSEC_DBGSWGEN) == 0U) {
		uint32_t result = bsec_write_debug_conf(dbg_conf |
							BSEC_DBGSWGEN);

		if (result != BSEC_OK) {
			ERROR("Error enabling DBGSWGEN\n");
			return (int)result;
		}
	}

	if ((mmio_read_32(rcc_base + RCC_DBGCFGR) & RCC_DBGCFGR_DBGCKEN) ==
	    0U) {
		mmio_setbits_32(rcc_base + RCC_DBGCFGR, RCC_DBGCFGR_DBGCKEN);
	}

	return 0;
}

#if STM32MP1_DEBUG_ENABLE
int stm32mp1_dbgmcu_boot_debug_info(void)
{
	uint32_t backup_reg_dbg;

	stm32mp_clk_enable(RTCAPB);

	backup_reg_dbg = (mmio_read_32(tamp_bkpr(TAMP_DBG_BACKUP_REG_ID))
			  & TAMP_DBG_DEBUG);

	stm32mp_clk_disable(RTCAPB);

	if (backup_reg_dbg != 0U) {
		return 1;
	}

	return 0;
}

int stm32mp1_dbgmcu_clear_boot_info(void)
{
	stm32mp_clk_enable(RTCAPB);

	mmio_clrbits_32(tamp_bkpr(TAMP_DBG_BACKUP_REG_ID),
			TAMP_DBG_DEBUG);

	stm32mp_clk_disable(RTCAPB);

	return 0;
}

uint32_t stm32mp1_dbgmcu_is_debug_on(void)
{
	uint32_t dbg_conf;

	dbg_conf = bsec_read_debug_conf();

	return (dbg_conf & (BSEC_SPIDEN | BSEC_SPINDEN));
}

#if LOG_LEVEL >= LOG_LEVEL_VERBOSE
void stm32mp1_dbgmcu_hexdump8(uint8_t *buf, uint32_t len)
{
	uint32_t i;

	VERBOSE("  ");
	for (i = 0; i < len; i++) {
		printf("%02x ", buf[i]);
		if (((i + 1) % 16) == 0) {
			if ((i + 1) < len) {
				printf("\n");
				VERBOSE("  ");
			}
		} else if (((i + 1) % 8) == 0) {
			printf(" ");
		}
	}
	printf("\n");
}
#endif
#endif

uint32_t stm32mp1_dbgmcu_get_chip_version(void)
{
	if (stm32mp1_dbgmcu_init() == 0) {
		return (mmio_read_32(DBGMCU_BASE + DBGMCU_IDC) >> 16);
	}

	return 0;
}

uint32_t stm32mp1_dbgmcu_get_chip_dev_id(void)
{
	if (stm32mp1_dbgmcu_init() == 0) {
		return (mmio_read_32(DBGMCU_BASE + DBGMCU_IDC) &
			IDC_DEV_ID_MASK);
	}

	return 0;
}

int stm32mp1_dbgmcu_freeze_iwdg2(void)
{
	if (stm32mp1_dbgmcu_init() == 0) {
		uint32_t dbg_conf = bsec_read_debug_conf();

		if (((dbg_conf & BSEC_SPIDEN) != 0U) ||
		    ((dbg_conf & BSEC_SPINDEN) != 0U)) {
			mmio_setbits_32(DBGMCU_BASE + DBGMCU_APB4FZ1,
					DBGMCU_APB4FZ1_IWDG2);
		}

		return 0;
	}

	return -EPERM;
}
