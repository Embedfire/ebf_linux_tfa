/*
 * Copyright (c) 2017-2019, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <debug.h>
#include <limits.h>
#include <mmio.h>
#include <platform_def.h>
#include <stm32_gpio.h>
#include <stm32mp_common.h>
#include <stm32mp_shres_helpers.h>
#include <stm32mp1_clk.h>
#include <stm32mp1_rcc.h>
#include <stm32mp1_shared_resources.h>
#include <stm32mp1_smc.h>
#include "rcc_svc.h"

#define STD_REG			0
#define SET_REG			1
#define CLR_REG			2

static void shared_clk_request(uint32_t request,
			       uint32_t offset, uint32_t value)
{
	unsigned long id;
	unsigned int bit;
	uint32_t enable_bits = 0;
	int clr_std_set = STD_REG;

	switch (request) {
	case STM32_SMC_REG_WRITE:
	case STM32_SMC_REG_SET:
	case STM32_SMC_REG_CLEAR:
		break;
	default:
		return;
	}

	switch (offset) {
	case RCC_MP_APB5ENSETR:
		clr_std_set = SET_REG;
		/* Non secure backup registers requires RTCAPB clock */
		enable_bits |= RCC_MP_APB5ENSETR_RTCAPBEN;
		break;
	case RCC_MP_APB5ENCLRR:
		clr_std_set = CLR_REG;
		/* Non secure backup registers requires RTCAPB clock */
		enable_bits |= RCC_MP_APB5ENSETR_RTCAPBEN;
		break;

	case RCC_MP_AHB5ENSETR:
		clr_std_set = SET_REG;
		if (stm32mp_gpio_bank_is_shared(GPIO_BANK_Z)) {
			enable_bits |= RCC_MP_AHB5ENSETR_GPIOZEN;
		}
		break;
	case RCC_MP_AHB5ENCLRR:
		clr_std_set = CLR_REG;
		if (stm32mp_gpio_bank_is_shared(GPIO_BANK_Z)) {
			enable_bits |= RCC_MP_AHB5ENSETR_GPIOZEN;
		}
		break;
	default:
		break;
	}

	if ((clr_std_set != STD_REG) && (request == STM32_SMC_REG_CLEAR)) {
		return;
	}

	/*
	 * Parse bit that relate to a functional clock.
	 * Call stm32mp1_clk_enable/disable_non_secure() for that clock
	 * according to request (write/set/clear) and target register
	 * (write or set/clear).
	 */
	for (bit = 0; bit < __WORD_BIT; bit++) {

		if ((BIT(bit) & enable_bits) == 0U) {
			continue;
		}

		id = stm32mp1_clk_rcc2id(offset, bit);
		if (id == ~0U) {
			panic();
		}

		switch (clr_std_set) {
		case SET_REG:
			if ((BIT(bit) & value) != 0U) {
				stm32mp1_clk_enable_non_secure(id);
			}
			break;
		case CLR_REG:
			if ((BIT(bit) & value) != 0U) {
				stm32mp1_clk_disable_non_secure(id);
			}
			break;
		default:
			/* Standard registers case */
			switch (request) {
			case STM32_SMC_REG_WRITE:
				if ((BIT(bit) & value) != 0U) {
					stm32mp1_clk_enable_non_secure(id);
				} else {
					stm32mp1_clk_disable_non_secure(id);
				}
				break;
			case STM32_SMC_REG_SET:
				if ((BIT(bit) & value) != 0U) {
					stm32mp1_clk_enable_non_secure(id);
				}
				break;
			case STM32_SMC_REG_CLEAR:
				if ((BIT(bit) & value) != 0U) {
					stm32mp1_clk_disable_non_secure(id);
				}
				break;
			default:
				return;
			}
			break;
		}

		enable_bits &= ~BIT(bit);
		if (enable_bits == 0U) {
			break;
		}
	}
}

static void access_allowed_mask(uint32_t request, uint32_t offset,
				uint32_t value, uint32_t allowed_mask)
{
	uint32_t addr = stm32mp_rcc_base() + offset;
	uint32_t masked_value = value & allowed_mask;

	switch (request) {
	case STM32_SMC_REG_WRITE:
		switch (offset) {
		/* CLR registers show the SET state, not the CLR state */
		case RCC_OCENCLRR:
		case RCC_MP_SREQCLRR:
		case RCC_APB5RSTCLRR:
		case RCC_AHB5RSTCLRR:
		case RCC_MP_APB5ENCLRR:
		case RCC_MP_AHB5ENCLRR:
		case RCC_MP_APB5LPENCLRR:
		case RCC_MP_AHB5LPENCLRR:
		case RCC_MP_IWDGFZCLRR:
			mmio_write_32(addr, masked_value);
			break;
		default:
			stm32mp_mmio_clrsetbits_32_shregs(addr, allowed_mask,
							  masked_value);
			break;
		}
		VERBOSE("wrt 0x%x = 0x%x => 0x%x\n", offset, value,
			mmio_read_32(addr));
		break;

	case STM32_SMC_REG_SET:
		switch (offset) {
		/* CLR registers show the SET state, not the CLR state */
		case RCC_OCENCLRR:
		case RCC_MP_SREQCLRR:
		case RCC_APB5RSTCLRR:
		case RCC_AHB5RSTCLRR:
		case RCC_MP_APB5ENCLRR:
		case RCC_MP_AHB5ENCLRR:
		case RCC_MP_APB5LPENCLRR:
		case RCC_MP_AHB5LPENCLRR:
		case RCC_MP_IWDGFZCLRR:
			mmio_write_32(addr, masked_value);
			break;
		default:
			stm32mp_mmio_setbits_32_shregs(addr, masked_value);
			break;
		}
		VERBOSE("set 0x%x = 0x%x => 0x%x\n", offset, value,
			mmio_read_32(addr));
		break;

	case STM32_SMC_REG_CLEAR:
		switch (offset) {
		case RCC_OCENCLRR:
		case RCC_MP_SREQCLRR:
		case RCC_APB5RSTCLRR:
		case RCC_AHB5RSTCLRR:
		case RCC_MP_APB5ENCLRR:
		case RCC_MP_AHB5ENCLRR:
		case RCC_MP_APB5LPENCLRR:
		case RCC_MP_AHB5LPENCLRR:
		case RCC_MP_IWDGFZCLRR:
			/* Nothing to do on CLR registers */
			break;
		default:
			stm32mp_mmio_clrbits_32_shregs(addr, masked_value);
			break;
		}
		VERBOSE("clear 0x%x = 0x%x => 0x%x\n", offset, value,
			mmio_read_32(addr));
		break;

	default:
		break;
	}
}

static void raw_allowed_access_request(uint32_t request,
				       uint32_t offset, uint32_t value)
{
	uint32_t allowed_mask = 0;

	/* Use UINT32_MAX if no secure restriction on register access */
	switch (offset) {
	case RCC_OCENSETR:
	case RCC_OCENCLRR:
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_HSI)) {
			allowed_mask |= RCC_OCENR_HSION | RCC_OCENR_HSIKERON;
		}
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_CSI)) {
			allowed_mask |= RCC_OCENR_CSION | RCC_OCENR_CSIKERON;
		}
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_HSE)) {
			allowed_mask |= RCC_OCENR_HSEON | RCC_OCENR_HSEKERON |
					RCC_OCENR_HSEBYP | RCC_OCENR_HSECSSON |
					RCC_OCENR_DIGBYP;
		}
		break;

	case RCC_HSICFGR:
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_HSI)) {
			allowed_mask = UINT32_MAX;
		}
		break;

	case RCC_CSICFGR:
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_CSI)) {
			allowed_mask = UINT32_MAX;
		}
		break;

	case RCC_MP_CIER:
	case RCC_MP_CIFR:
		/* RCC_MP_CIFR_xxxRDYF matches CIER and CIFR bit mapping */
		allowed_mask |= RCC_MP_CIFR_WKUPF | RCC_MP_CIFR_PLL4DYF;

		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_LSI)) {
			allowed_mask |= RCC_MP_CIFR_LSIRDYF;
		}
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_LSE)) {
			allowed_mask |= RCC_MP_CIFR_LSERDYF;
		}
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_HSI)) {
			allowed_mask |= RCC_MP_CIFR_HSIRDYF;
		}
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_HSE)) {
			allowed_mask |= RCC_MP_CIFR_HSERDYF;
		}
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_CSI)) {
			allowed_mask |= RCC_MP_CIFR_CSIRDYF;
		}
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_PLL1)) {
			allowed_mask |= RCC_MP_CIFR_PLL1DYF;
		}
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_PLL2)) {
			allowed_mask |= RCC_MP_CIFR_PLL2DYF;
		}
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_PLL3)) {
			allowed_mask |= RCC_MP_CIFR_PLL3DYF;
		}
		break;

	case RCC_PLL1CR:
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_PLL1_P)) {
			allowed_mask |= RCC_PLLNCR_DIVPEN;
		}
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_PLL1_Q)) {
			allowed_mask |= RCC_PLLNCR_DIVQEN;
		}
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_PLL1_R)) {
			allowed_mask |= RCC_PLLNCR_DIVREN;
		}
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_PLL1)) {
			allowed_mask |= RCC_PLLNCR_PLLON | RCC_PLLNCR_PLLRDY |
					RCC_PLLNCR_SSCG_CTRL;
		}
		break;

	case RCC_PLL2CR:
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_PLL2_P)) {
			allowed_mask |= RCC_PLLNCR_DIVPEN;
		}
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_PLL2_Q)) {
			allowed_mask |= RCC_PLLNCR_DIVQEN;
		}
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_PLL2_R)) {
			allowed_mask |= RCC_PLLNCR_DIVREN;
		}
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_PLL2)) {
			allowed_mask |= RCC_PLLNCR_PLLON | RCC_PLLNCR_PLLRDY |
					RCC_PLLNCR_SSCG_CTRL;
		}
		break;

	case RCC_PLL3CR:
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_PLL3_P)) {
			allowed_mask |= RCC_PLLNCR_DIVPEN;
		}
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_PLL3_Q)) {
			allowed_mask |= RCC_PLLNCR_DIVQEN;
		}
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_PLL3_R)) {
			allowed_mask |= RCC_PLLNCR_DIVREN;
		}
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_PLL3)) {
			allowed_mask |= RCC_PLLNCR_PLLON | RCC_PLLNCR_PLLRDY |
					RCC_PLLNCR_SSCG_CTRL;
		}
		break;

	case RCC_MP_BOOTCR:		/* Allowed MPU/MCU reboot cfg */
	case RCC_MP_GCR:		/* Allowed MPU/MCU reboot cfg */
	case RCC_MP_GRSTCSETR:		/* Allowed MCU and system reset */
	case RCC_BR_RSTSCLRR:		/* Allowed system reset status */
	case RCC_MC_RSTSCLRR:		/* Allowed system reset status */
	case RCC_MP_RSTSCLRR:		/* Allowed system reset status */
		allowed_mask = UINT32_MAX;
		break;
	case RCC_APB5RSTSETR:
	case RCC_APB5RSTCLRR:
	case RCC_MP_APB5ENSETR:
	case RCC_MP_APB5ENCLRR:
	case RCC_MP_APB5LPENSETR:
	case RCC_MP_APB5LPENCLRR:
		/*
		 * SPI6/I2C4/I2C6/USART1/IWDG1 resources may be non secure.
		 * TZPC/TZC/BSEC/STGEN resources are secure only.
		 * Bit mask RCC_MP_APB5ENSETR_xxxEN fits EN, RST and LPEN.
		 */
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_SPI6)) {
			allowed_mask |= RCC_MP_APB5ENSETR_SPI6EN;
		}
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_I2C4)) {
			allowed_mask |= RCC_MP_APB5ENSETR_I2C4EN;
		}
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_I2C6)) {
			allowed_mask |= RCC_MP_APB5ENSETR_I2C6EN;
		}
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_USART1)) {
			allowed_mask |= RCC_MP_APB5ENSETR_USART1EN;
		}
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_IWDG1)) {
			allowed_mask |= RCC_MP_APB5ENSETR_IWDG1APBEN;
		}
		break;
	case RCC_AHB5RSTSETR:
	case RCC_AHB5RSTCLRR:
	case RCC_MP_AHB5ENSETR:
	case RCC_MP_AHB5ENCLRR:
	case RCC_MP_AHB5LPENSETR:
	case RCC_MP_AHB5LPENCLRR:
		/*
		 * RNG1/HASH1/CRYP1/GPIOZ resources are accessible if
		 * related BKPSRAM resources are reserved to secure services.
		 * Bit mask RCC_MP_AHB5ENSETR_xxxEN fits EN, RST and LPEN.
		 */
		if (stm32mp_gpio_bank_is_non_secure(GPIO_BANK_Z)) {
			allowed_mask |= RCC_MP_AHB5ENSETR_GPIOZEN;
		}
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_CRYP1)) {
			allowed_mask |= RCC_MP_AHB5ENSETR_CRYP1EN;
		}
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_HASH1)) {
			allowed_mask |= RCC_MP_AHB5ENSETR_HASH1EN;
		}
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_RNG1)) {
			allowed_mask |= RCC_MP_AHB5ENSETR_RNG1EN;
		}
		break;
	case RCC_RTCDIVR:
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_RTC)) {
			allowed_mask = UINT32_MAX;
		}
		break;
	case RCC_I2C46CKSELR:
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_I2C4) &&
		    stm32mp1_periph_is_non_secure(STM32MP1_SHRES_I2C6)) {
			allowed_mask = UINT32_MAX;
		}
		break;
	case RCC_SPI6CKSELR:
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_SPI6)) {
			allowed_mask = UINT32_MAX;
		}
		break;
	case RCC_UART1CKSELR:
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_USART1)) {
			allowed_mask = UINT32_MAX;
		}
		break;
	case RCC_RNG1CKSELR:
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_RNG1)) {
			allowed_mask = UINT32_MAX;
		}
		break;
	case RCC_MP_IWDGFZSETR:
	case RCC_MP_IWDGFZCLRR:
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_IWDG1)) {
			allowed_mask |= RCC_MP_IWDGFZSETR_IWDG1;
		}
		allowed_mask |= RCC_MP_IWDGFZSETR_IWDG2;
		break;
	default:
		return;
	}

	if (allowed_mask != 0U) {
		access_allowed_mask(request, offset, value, allowed_mask);
	}
}

uint32_t rcc_scv_handler(uint32_t x1, uint32_t x2, uint32_t x3)
{
	uint32_t request = x1;
	uint32_t offset = x2;
	uint32_t value = x3;

	/*
	 * x2 may be either the RCC register offset or the register
	 * full physical address.
	 */
	if ((offset & ~RCC_OFFSET_MASK) != 0) {
		if ((offset & ~RCC_OFFSET_MASK) != stm32mp_rcc_base()) {
			return STM32_SMC_INVALID_PARAMS;
		}

		offset &= RCC_OFFSET_MASK;
	}

	/* Some clocks may be managed by some secure services */
	shared_clk_request(request, offset, value);

	/* RCC controls for non secure resource may be accessed straight */
	raw_allowed_access_request(request, offset, value);

	return STM32_SMC_OK;
}

uint32_t rcc_cal_scv_handler(uint32_t x1)
{
	uint32_t ret = STM32_SMC_FAILED;

	switch (x1) {
	case CK_CSI:
		if (stm32mp1_calib_start_csi_cal() ==  0) {
			ret = STM32_SMC_OK;
		}
		break;

	case CK_HSI:
		if (stm32mp1_calib_start_hsi_cal() == 0) {
			ret = STM32_SMC_OK;
		}
		break;

	default:
		ret = STM32_SMC_INVALID_PARAMS;
		break;
	}

	return ret;
}

uint32_t rcc_opp_scv_handler(uint32_t x1, uint32_t x2, uint32_t *res)
{
	uint32_t cmd = x1;
	uint32_t opp = x2 / 1000U; /* KHz */

	switch (cmd) {
	case STM32_SMC_RCC_OPP_SET:
		if (stm32mp1_set_opp_khz(opp) != 0) {
			return STM32_SMC_FAILED;
		}
		break;

	case STM32_SMC_RCC_OPP_ROUND:
		if (stm32mp1_round_opp_khz(&opp) != 0) {
			return STM32_SMC_FAILED;
		}

		if (opp > (UINT32_MAX / 1000U)) {
			return STM32_SMC_FAILED;
		}

		*res = opp * 1000U;
		break;

	default:
		return STM32_SMC_INVALID_PARAMS;
	}

	return STM32_SMC_OK;
}
