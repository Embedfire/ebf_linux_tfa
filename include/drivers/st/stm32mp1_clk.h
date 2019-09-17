/*
 * Copyright (c) 2018, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STM32MP1_CLK_H
#define STM32MP1_CLK_H

#include <stdbool.h>
#include <stdint.h>

#include <dt-bindings/clock/stm32mp1-clks.h>

int stm32mp1_clk_probe(void);
int stm32mp1_clk_init(void);

bool stm32mp1_rcc_is_secure(void);
bool stm32mp1_rcc_is_mckprot(void);

void __stm32mp1_clk_enable(unsigned long id, bool caller_is_secure);
void __stm32mp1_clk_disable(unsigned long id, bool caller_is_secure);
bool stm32mp1_clk_is_enabled(unsigned long id);

static inline void stm32mp1_clk_enable_non_secure(unsigned long id)
{
	__stm32mp1_clk_enable(id, false);
}

static inline void stm32mp1_clk_enable_secure(unsigned long id)
{
	__stm32mp1_clk_enable(id, true);
}

static inline void stm32mp1_clk_disable_non_secure(unsigned long id)
{
	__stm32mp1_clk_disable(id, false);
}

static inline void stm32mp1_clk_disable_secure(unsigned long id)
{
	__stm32mp1_clk_disable(id, true);
}

unsigned int stm32mp1_clk_get_refcount(unsigned long id);

unsigned long stm32mp_clk_get_rate(unsigned long id);
unsigned long stm32mp_clk_timer_get_rate(unsigned long id);

void stm32mp1_stgen_increment(unsigned long long offset_in_ms);

bool stm32mp1_rtc_get_read_twice(void);

/* SMP protection on RCC registers access */
void stm32mp1_clk_rcc_regs_lock(void);
void stm32mp1_clk_rcc_regs_unlock(void);

unsigned long stm32mp1_clk_rcc2id(unsigned int offset, unsigned int bit);
#if defined(IMAGE_BL32)
void stm32mp1_clk_mpu_suspend(void);
void stm32mp1_clk_mpu_resume(void);
#endif

void stm32mp1_register_clock_parents_secure(unsigned long id);

void stm32mp1_update_earlyboot_clocks_state(void);

#endif /* STM32MP1_CLK_H */
