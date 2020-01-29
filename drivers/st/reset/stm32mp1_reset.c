/*
 * Copyright (c) 2018-2019, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <bl_common.h>
#include <debug.h>
#include <delay_timer.h>
#include <limits.h>
#include <mmio.h>
#include <platform_def.h>
#include <stm32mp_reset.h>
#include <utils_def.h>

#define RST_CLR_OFFSET			4U
#define RESET_TIMEOUT_1MS_IN_US		1000
#define RESET_TIMEOUT_STEP_US		10

static uint32_t id2reg_offset(unsigned int reset_id)
{
	return ((reset_id & GENMASK(31, 5)) >> 5) * sizeof(uint32_t);
}

static uint8_t id2reg_bit_pos(unsigned int reset_id)
{
	return (uint8_t)(reset_id & GENMASK(4, 0));
}

void stm32mp_reset_assert(uint32_t id)
{
	uint32_t offset = id2reg_offset(id);
	uint32_t bitmsk = BIT(id2reg_bit_pos(id));
	int nb_tries = RESET_TIMEOUT_1MS_IN_US / RESET_TIMEOUT_STEP_US;
	uintptr_t rcc_base = stm32mp_rcc_base();

	mmio_write_32(rcc_base + offset, bitmsk);

	while (((mmio_read_32(rcc_base + offset) & bitmsk) == 0U) &&
	       (nb_tries != 0)) {
		udelay(RESET_TIMEOUT_STEP_US);
		nb_tries--;
	}

	if (nb_tries == 0) {
		ERROR("Reset timeout\n");
		panic();
	}
}

void stm32mp_reset_deassert(uint32_t id)
{
	uint32_t offset = id2reg_offset(id) + RST_CLR_OFFSET;
	uint32_t bitmsk = BIT(id2reg_bit_pos(id));
	int nb_tries = RESET_TIMEOUT_1MS_IN_US / RESET_TIMEOUT_STEP_US;
	uintptr_t rcc_base = stm32mp_rcc_base();

	mmio_write_32(rcc_base + offset, bitmsk);

	while (((mmio_read_32(rcc_base + offset) & bitmsk) != 0U) &&
	       (nb_tries != 0)) {
		udelay(RESET_TIMEOUT_STEP_US);
		nb_tries--;
	}

	if (nb_tries == 0) {
		ERROR("Reset timeout\n");
		panic();
	}
}
