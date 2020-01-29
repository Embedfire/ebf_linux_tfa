/*
 * Copyright (c) 2015-2019, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STM32MP_COMMON_H
#define STM32MP_COMMON_H

#include <cdefs.h>
#include <stdbool.h>

void __dead2 stm32mp_plat_reset(int cpu);

void stm32mp_save_boot_ctx_address(uintptr_t address);
uintptr_t stm32mp_get_boot_ctx_address(void);

bool stm32mp_is_single_core(void);

uintptr_t stm32mp_ddrctrl_base(void);
uintptr_t stm32mp_ddrphyc_base(void);
uintptr_t stm32mp_pwr_base(void);
uintptr_t stm32mp_rcc_base(void);

int stm32_gic_enable_spi(int node, const char *name);

int stm32_get_otp_index(const char *otp_name, uint32_t *otp_idx,
			uint32_t *otp_len);
int stm32_get_otp_value(const char *otp_name, uint32_t *otp_val);
uint32_t stm32_read_otp_status(uint32_t *otp_value, uint32_t word);
uint8_t stm32_iwdg_get_instance(uintptr_t base);
uint32_t stm32_iwdg_get_otp_config(uintptr_t base);

#if defined(IMAGE_BL2)
uint32_t stm32_iwdg_shadow_update(uintptr_t base, uint32_t flags);
#endif

uintptr_t stm32_get_gpio_bank_base(unsigned int bank);
int stm32_get_gpio_bank_clock(unsigned int bank);
uint32_t stm32_get_gpio_bank_offset(unsigned int bank);

bool stm32mp_supports_cpu_opp(uint32_t opp_id);
bool stm32mp_ddr_supports_ssr_asr(void);

void stm32mp_print_cpuinfo(void);
void stm32mp_print_boardinfo(void);

uint64_t s2tick(uint32_t timeout_s);
uint64_t ms2tick(uint32_t timeout_ms);
uint64_t us2tick(uint32_t timeout_us);
uint64_t timeout_start(void);
bool timeout_elapsed(uint64_t tick_start, uint64_t tick_to);

void stm32mp_io_setup(void);

#endif /* STM32MP1_COMMON_H */
