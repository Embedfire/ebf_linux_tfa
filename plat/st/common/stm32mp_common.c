/*
 * Copyright (c) 2018-2019, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <arch_helpers.h>
#include <assert.h>
#include <bl_common.h>
#include <debug.h>
#include <mmio.h>
#include <platform.h>
#include <platform_def.h>
#include <stm32mp_clkfunc.h>
#include <stm32mp_common.h>
#include <stm32mp_dt.h>

uintptr_t plat_get_ns_image_entrypoint(void)
{
	return BL33_BASE;
}

unsigned int plat_get_syscnt_freq2(void)
{
	return read_cntfrq_el0();
}

#pragma weak stm32mp_plat_reset
void __dead2 stm32mp_plat_reset(int cpu)
{
	panic();
}

/* Functions to save and get boot context address given by ROM code */
static uintptr_t boot_ctx_address;

void stm32mp_save_boot_ctx_address(uintptr_t address)
{
	boot_ctx_address = address;
}

uintptr_t stm32mp_get_boot_ctx_address(void)
{
	return boot_ctx_address;
}

/*
 * This function determines if one single core is presently running. This is
 * done by OTP read.
 * Returns 1 if yes, 0 if more that one core is running, -1 if error.
 */
#pragma weak stm32mp_is_single_core

bool stm32mp_is_single_core(void)
{
	return false;
}

uintptr_t stm32mp_ddrctrl_base(void)
{
	static uintptr_t ddrctrl_base;

	if (ddrctrl_base == 0) {
		ddrctrl_base = dt_get_ddrctrl_base();

		assert(ddrctrl_base == DDRCTRL_BASE);
	}

	return ddrctrl_base;
}

uintptr_t stm32mp_ddrphyc_base(void)
{
	static uintptr_t ddrphyc_base;

	if (ddrphyc_base == 0) {
		ddrphyc_base = dt_get_ddrphyc_base();

		assert(ddrphyc_base == DDRPHYC_BASE);
	}

	return ddrphyc_base;
}

uintptr_t stm32mp_pwr_base(void)
{
	static uintptr_t pwr_base;

	if (pwr_base == 0) {
		pwr_base = dt_get_pwr_base();

		assert(pwr_base == PWR_BASE);
	}

	return pwr_base;
}

uintptr_t stm32mp_rcc_base(void)
{
	static uintptr_t rcc_base;

	if (rcc_base == 0) {
		rcc_base = dt_get_rcc_base();

		assert(rcc_base == RCC_BASE);
	}

	return rcc_base;
}

uintptr_t stm32_get_gpio_bank_base(unsigned int bank)
{
	switch (bank) {
	case GPIO_BANK_A ... GPIO_BANK_K:
		return GPIOA_BASE + (bank * GPIO_BANK_OFFSET);
	case GPIO_BANK_Z:
		return GPIOZ_BASE;
	default:
		panic();
	}
}

/* Return clock ID on success, negative value on error */
int stm32_get_gpio_bank_clock(unsigned int bank)
{
	switch (bank) {
	case GPIO_BANK_A ... GPIO_BANK_K:
		return (int)GPIOA + (bank - GPIO_BANK_A);
	case GPIO_BANK_Z:
		return (int)GPIOZ;
	default:
		panic();
	}
}

uint32_t stm32_get_gpio_bank_offset(unsigned int bank)
{
	if (bank == GPIO_BANK_Z) {
		return 0;
	} else {
		return bank * GPIO_BANK_OFFSET;
	}
}

uint64_t s2tick(uint32_t timeout_s)
{
	return (uint64_t)timeout_s * read_cntfrq_el0();
}

uint64_t ms2tick(uint32_t timeout_ms)
{
	return s2tick(timeout_ms) / 1000U;
}

uint64_t us2tick(uint32_t timeout_us)
{
	return s2tick(timeout_us) / (1000U * 1000U);
}

uint64_t timeout_start(void)
{
	return read_cntpct_el0();
}

bool timeout_elapsed(uint64_t tick_start, uint64_t tick_to)
{
	return (tick_to != 0U) && ((read_cntpct_el0() - tick_start) > tick_to);
}
