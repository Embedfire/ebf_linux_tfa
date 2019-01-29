/*
 * Copyright (c) 2017-2018, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <smccc_helpers.h>
#include <stm32mp1_ddr_helpers.h>
#include <stm32mp1_power_config.h>
#include <stm32mp1_smc.h>
#include "low_power_svc.h"

uint32_t sr_mode_scv_handler(uint32_t x1, uint32_t x2)
{
	uint32_t ret = STM32_SMC_OK;

	switch (x2) {
	case STM32_SMC_SR_MODE_SSR:
		ddr_sr_mode_ssr();
		break;

	case STM32_SMC_SR_MODE_ASR:
		ddr_sr_mode_asr();
		break;

	case STM32_SMC_SR_MODE_HSR:
		ddr_sr_mode_hsr();
		break;

	default:
		ret = STM32_SMC_INVALID_PARAMS;
		break;
	}

	return ret;
}

uint32_t pm_domain_scv_handler(uint32_t x1, uint32_t x2)
{
	if (stm32mp1_set_pm_domain_state((enum stm32mp1_pm_domain)x1,
					 (bool)x2) < 0) {
		return STM32_SMC_FAILED;
	}

	return STM32_SMC_OK;
}
