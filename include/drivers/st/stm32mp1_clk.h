/*
 * Copyright (c) 2018-2019, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STM32MP1_CLK_H
#define STM32MP1_CLK_H

#include <stdbool.h>
#include <stdint.h>

#include <dt-bindings/clock/stm32mp1-clks.h>

#define PLL1_SETTINGS_VALID_ID	U(0x504C4C31) /* "PLL1" */

int stm32mp1_clk_probe(void);
int stm32mp1_clk_init(uint32_t pll1_freq_mhz);

int stm32mp1_clk_compute_all_pll1_settings(uint32_t buck1_voltage);
void stm32mp1_clk_lp_save_opp_pll1_settings(uint8_t *data, size_t size);
void stm32mp1_clk_lp_load_opp_pll1_settings(uint8_t *data, size_t size);

int stm32mp1_clk_get_maxfreq_opp(uint32_t *freq_mhz, uint32_t *voltage_mv);

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

bool stm32mp1_rtc_get_read_twice(void);

/* SMP protection on RCC registers access */
void stm32mp1_clk_rcc_regs_lock(void);
void stm32mp1_clk_rcc_regs_unlock(void);

void stm32mp1_stgen_restore_counter(unsigned long long value,
				    unsigned long long offset_in_ms);
unsigned long long stm32mp1_stgen_get_counter(void);

unsigned long stm32mp1_clk_rcc2id(unsigned int offset, unsigned int bit);

int stm32mp1_round_opp_khz(uint32_t *freq_khz);
int stm32mp1_set_opp_khz(uint32_t freq_khz);

#if defined(IMAGE_BL32)
void stm32mp1_clk_mpu_suspend(void);
void stm32mp1_clk_mpu_resume(void);
#endif

void stm32mp1_register_clock_parents_secure(unsigned long id);

void stm32mp1_update_earlyboot_clocks_state(void);

#endif /* STM32MP1_CLK_H */
