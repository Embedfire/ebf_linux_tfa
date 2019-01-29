/*
 * Copyright (C) 2018, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <arch_helpers.h>
#include <mmio.h>
#include <spinlock.h>
#include <stm32mp_shres_helpers.h>

static struct spinlock shregs_lock;

static int stm32mp_lock_available(void)
{
	/* The spinlocks are used only when MMU is enabled */
#ifdef AARCH32
	return (read_sctlr() & SCTLR_M_BIT) && (read_sctlr() & SCTLR_C_BIT);
#else
	return (read_sctlr_el3() & SCTLR_M_BIT) &&
		(read_sctlr_el3() & SCTLR_C_BIT);
#endif
}

void stm32mp_lock_shregs(void)
{
	if (stm32mp_lock_available() == 0U) {
		return;
	}

	/* Assume interrupts are masked */
	spin_lock(&shregs_lock);
}

void stm32mp_unlock_shregs(void)
{
	if (stm32mp_lock_available() == 0U) {
		return;
	}

	spin_unlock(&shregs_lock);
}

/* Shared register access: upon shared resource lock */
void stm32mp_mmio_clrsetbits_32_shregs(uintptr_t addr, uint32_t clear,
				       uint32_t set)
{
	stm32mp_lock_shregs();

	mmio_clrsetbits_32(addr, clear, set);

	stm32mp_unlock_shregs();
}

void stm32mp_mmio_clrbits_32_shregs(uintptr_t addr, uint32_t clear)
{
	stm32mp_lock_shregs();

	mmio_clrbits_32(addr, clear);

	stm32mp_unlock_shregs();
}

void stm32mp_mmio_setbits_32_shregs(uintptr_t addr, uint32_t set)
{
	stm32mp_lock_shregs();

	mmio_setbits_32(addr, set);

	stm32mp_unlock_shregs();
}
