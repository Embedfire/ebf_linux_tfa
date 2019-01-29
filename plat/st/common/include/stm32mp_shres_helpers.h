/*
 * Copyright (C) 2018, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STM32MP_SHRES_HELPERS_H
#define STM32MP_SHRES_HELPERS_H

#include <debug.h>
#include <stdint.h>

/*
 * Lock/unlock access to shared registers
 *
 * @lock - NULL or pointer to spin lock
 */

void stm32mp_lock_shregs(void);
void stm32mp_unlock_shregs(void);
void stm32mp_mmio_clrsetbits_32_shregs(uintptr_t addr, uint32_t clear,
				       uint32_t set);
void stm32mp_mmio_clrbits_32_shregs(uintptr_t addr, uint32_t clear);
void stm32mp_mmio_setbits_32_shregs(uintptr_t addr, uint32_t set);

/*
 * Shared reference counter: increments by 2 on secure increment
 * request, decrements by 2 on secure decrement request. Bit #0
 * is set to 1 on non-secure increment request and reset to 0 on
 * non-secure decrement request. The counter initializes to
 * either 0, 1 or 2 upon their expect default state.
 * Counters saturates once above UINT_MAX / 2.
 */
#define SHREFCNT_NONSECURE_FLAG		0x1UL
#define SHREFCNT_SECURE_STEP		0x2UL
#define SHREFCNT_MAX			(UINT32_MAX / 2)

/* Return 1 if refcnt increments from 0, else return 0 */
static inline int stm32mp_incr_shrefcnt(unsigned int *refcnt, bool secure)
{
	int rc = !*refcnt;

	if (secure) {
		*refcnt += SHREFCNT_SECURE_STEP;
		if (*refcnt >= SHREFCNT_MAX) {
			panic();
		}
	} else {
		*refcnt |= SHREFCNT_NONSECURE_FLAG;
	}

	return rc;
}

/* Return 1 if refcnt decrements to 0, else return 0 */
static inline int stm32mp_decr_shrefcnt(unsigned int *refcnt, bool secure)
{
	int  rc = 0;

	if (secure) {
		if (*refcnt < SHREFCNT_MAX) {
			if (*refcnt < SHREFCNT_SECURE_STEP) {
				panic();
			}
			*refcnt -= SHREFCNT_SECURE_STEP;
			rc = !*refcnt;
		}
	} else {
		rc = (*refcnt == SHREFCNT_NONSECURE_FLAG);
		*refcnt &= ~SHREFCNT_NONSECURE_FLAG;
	}

	return rc;
}

static inline int stm32mp_incr_refcnt(unsigned int *refcnt)
{
	return stm32mp_incr_shrefcnt(refcnt, true);
}

static inline int stm32mp_decr_refcnt(unsigned int *refcnt)
{
	return stm32mp_decr_shrefcnt(refcnt, true);
}

#endif /* STM32MP_SHRES_HELPERS_H */
