/*
 * Copyright (c) 2017-2018, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <arch_helpers.h>
#include <boot_api.h>
#include <context.h>
#include <context_mgmt.h>
#include <dt-bindings/clock/stm32mp1-clks.h>
#include <errno.h>
#include <mmio.h>
#include <platform_def.h>
#include <smccc_helpers.h>
#include <stm32_rtc.h>
#include <stm32mp1_clk.h>
#include <stm32mp1_context.h>
#include <stm32mp1_ddr_regs.h>
#include <stm32mp1_shared_resources.h>
#include <string.h>
#include <utils.h>

#define TAMP_BOOT_ITF_BACKUP_REG_ID	U(20)
#define TAMP_BOOT_ITF_MASK		U(0x0000FF00)
#define TAMP_BOOT_ITF_SHIFT		8

#define TRAINING_AREA_SIZE		64

#ifdef AARCH32_SP_OPTEE
/* OPTEE_MAILBOX_MAGIC relates to struct backup_data_s as defined */
#define OPTEE_MAILBOX_MAGIC_V1		0x01
#define OPTEE_MAILBOX_MAGIC		((OPTEE_MAILBOX_MAGIC_V1 << 16) + \
						TRAINING_AREA_SIZE)
#endif

struct backup_data_s {
#ifdef AARCH32_SP_OPTEE
	uint32_t magic;
	uint32_t core0_resume_hint;
	uint32_t zq0cr0_zdata;
	uint8_t ddr_training_backup[TRAINING_AREA_SIZE];
#else
	smc_ctx_t saved_smc_context[PLATFORM_CORE_COUNT];
	cpu_context_t saved_cpu_context[PLATFORM_CORE_COUNT];
	uint32_t zq0cr0_zdata;
	struct stm32_rtc_calendar rtc;
	uint8_t ddr_training_backup[TRAINING_AREA_SIZE];
#endif
};

#ifdef AARCH32_SP_OPTEE
uint32_t stm32_pm_get_optee_ep(void)
{
	struct backup_data_s *backup_data;
	uint32_t ep;

	stm32mp_clk_enable(BKPSRAM);

	/* Context & Data to be saved at the beginning of Backup SRAM */
	backup_data = (struct backup_data_s *)STM32MP_BACKUP_RAM_BASE;

	if (backup_data->magic != OPTEE_MAILBOX_MAGIC) {
		panic();
	}

	ep = backup_data->core0_resume_hint;

	stm32mp_clk_disable(BKPSRAM);

	return ep;
}
#else /*AARCH32_SP_OPTEE*/
void stm32_clean_context(void)
{
	stm32mp_clk_enable(BKPSRAM);

	zeromem((void *)STM32MP_BACKUP_RAM_BASE, sizeof(struct backup_data_s));

	stm32mp_clk_disable(BKPSRAM);
}

int stm32_save_context(uint32_t zq0cr0_zdata)
{
	void *smc_context;
	void *cpu_context;
	struct backup_data_s *backup_data;

	stm32mp_clk_enable(BKPSRAM);

	/* Context & Data to be saved at the beginning of Backup SRAM */
	backup_data = (struct backup_data_s *)STM32MP_BACKUP_RAM_BASE;

	/* Retrieve smc context struct address */
	smc_context = smc_get_ctx(NON_SECURE);

	/* Retrieve smc context struct address */
	cpu_context = cm_get_context(NON_SECURE);

	/* Save context in Backup SRAM */
	memcpy(&backup_data->saved_smc_context[0], smc_context,
	       sizeof(smc_ctx_t) * PLATFORM_CORE_COUNT);
	memcpy(&backup_data->saved_cpu_context[0], cpu_context,
	       sizeof(cpu_context_t) * PLATFORM_CORE_COUNT);

	backup_data->zq0cr0_zdata = zq0cr0_zdata;

	stm32_rtc_get_calendar(&backup_data->rtc);

	stm32mp_clk_disable(BKPSRAM);

	return 0;
}

int stm32_restore_context(void)
{
	void *smc_context;
	void *cpu_context;
	struct backup_data_s *backup_data;
	struct stm32_rtc_calendar current_calendar;
	unsigned long long stdby_time_in_ms;

	stm32mp_clk_enable(BKPSRAM);

	/* Context & Data to be saved at the beginning of Backup SRAM */
	backup_data = (struct backup_data_s *)STM32MP_BACKUP_RAM_BASE;

	/* Retrieve smc context struct address */
	smc_context = smc_get_ctx(NON_SECURE);

	/* Retrieve smc context struct address */
	cpu_context = cm_get_context(NON_SECURE);

	/* Restore data from Backup SRAM */
	memcpy(smc_context, backup_data->saved_smc_context,
	       sizeof(smc_ctx_t) * PLATFORM_CORE_COUNT);
	memcpy(cpu_context, backup_data->saved_cpu_context,
	       sizeof(cpu_context_t) * PLATFORM_CORE_COUNT);

	/* update STGEN counter with standby mode length */
	stm32_rtc_get_calendar(&current_calendar);
	stdby_time_in_ms = stm32_rtc_diff_calendar(&current_calendar,
						   &backup_data->rtc);
	stm32mp1_stgen_increment(stdby_time_in_ms);

	stm32mp_clk_disable(BKPSRAM);

	return 0;
}
#endif /*AARCH32_SP_OPTEE*/

uint32_t stm32_get_zdata_from_context(void)
{
	struct backup_data_s *backup_data;
	uint32_t zdata;

	stm32mp_clk_enable(BKPSRAM);

	backup_data = (struct backup_data_s *)STM32MP_BACKUP_RAM_BASE;

	zdata = (backup_data->zq0cr0_zdata >> DDRPHYC_ZQ0CRN_ZDATA_SHIFT) &
		DDRPHYC_ZQ0CRN_ZDATA_MASK;

	stm32mp_clk_disable(BKPSRAM);

	return zdata;
}

int stm32_save_boot_interface(uint32_t interface, uint32_t instance)
{
	uint32_t bkpr_itf_idx = tamp_bkpr(TAMP_BOOT_ITF_BACKUP_REG_ID);

	stm32mp_clk_enable(RTCAPB);

	mmio_clrsetbits_32(bkpr_itf_idx,
			   TAMP_BOOT_ITF_MASK,
			   ((interface << 4) | (instance & 0xFU)) <<
			   TAMP_BOOT_ITF_SHIFT);

	stm32mp_clk_disable(RTCAPB);

	return 0;
}

int stm32_get_boot_interface(uint32_t *interface, uint32_t *instance)
{
	uint32_t backup_reg_itf;
	uint32_t bkpr_itf_idx = tamp_bkpr(TAMP_BOOT_ITF_BACKUP_REG_ID);

	stm32mp_clk_enable(RTCAPB);

	backup_reg_itf = (mmio_read_32(bkpr_itf_idx) &
			  TAMP_BOOT_ITF_MASK) >> TAMP_BOOT_ITF_SHIFT;

	stm32mp_clk_disable(RTCAPB);

	*interface = backup_reg_itf >> 4;
	*instance = backup_reg_itf & 0xFU;

	return 0;
}

/*
 * When returning from STANDBY, the 64 first bytes of DDR will be overwritten
 * during DDR DQS training. This area must then be saved before going to
 * standby, and will be restored after
 */
void stm32_save_ddr_training_area(void)
{
	struct backup_data_s *backup_data;

	stm32mp_clk_enable(BKPSRAM);

	backup_data = (struct backup_data_s *)STM32MP_BACKUP_RAM_BASE;

	memcpy(&backup_data->ddr_training_backup,
	       (const uint32_t *)STM32MP_DDR_BASE,
	       TRAINING_AREA_SIZE);
	dsb();

	stm32mp_clk_disable(BKPSRAM);
}

void stm32_restore_ddr_training_area(void)
{
	struct backup_data_s *backup_data;

	stm32mp_clk_enable(BKPSRAM);

	backup_data = (struct backup_data_s *)STM32MP_BACKUP_RAM_BASE;

	memcpy((uint32_t *)STM32MP_DDR_BASE,
	       &backup_data->ddr_training_backup,
	       TRAINING_AREA_SIZE);
	dsb();

	stm32mp_clk_disable(BKPSRAM);
}
