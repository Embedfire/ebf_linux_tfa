/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2017-2018, STMicroelectronics
 */

#ifndef STM32MP1_SHARED_RESOURCES_H
#define STM32MP1_SHARED_RESOURCES_H

#include <debug.h>
#include <etzpc.h>
#include <stdbool.h>

void stm32mp_clk_enable(unsigned long id);
void stm32mp_clk_disable(unsigned long id);

#define STM32MP1_SHRES_GPIOZ(i)		(STM32MP1_SHRES_GPIOZ_0 + i)

enum stm32mp_shres {
	STM32MP1_SHRES_GPIOZ_0 = 0,
	STM32MP1_SHRES_GPIOZ_1,
	STM32MP1_SHRES_GPIOZ_2,
	STM32MP1_SHRES_GPIOZ_3,
	STM32MP1_SHRES_GPIOZ_4,
	STM32MP1_SHRES_GPIOZ_5,
	STM32MP1_SHRES_GPIOZ_6,
	STM32MP1_SHRES_GPIOZ_7,
	STM32MP1_SHRES_IWDG1,
	STM32MP1_SHRES_USART1,
	STM32MP1_SHRES_SPI6,
	STM32MP1_SHRES_I2C4,
	STM32MP1_SHRES_RNG1,
	STM32MP1_SHRES_HASH1,
	STM32MP1_SHRES_CRYP1,
	STM32MP1_SHRES_I2C6,
	STM32MP1_SHRES_RTC,
	STM32MP1_SHRES_MCU,
	STM32MP1_SHRES_HSI,
	STM32MP1_SHRES_LSI,
	STM32MP1_SHRES_HSE,
	STM32MP1_SHRES_LSE,
	STM32MP1_SHRES_CSI,
	STM32MP1_SHRES_PLL1,
	STM32MP1_SHRES_PLL1_P,
	STM32MP1_SHRES_PLL1_Q,
	STM32MP1_SHRES_PLL1_R,
	STM32MP1_SHRES_PLL2,
	STM32MP1_SHRES_PLL2_P,
	STM32MP1_SHRES_PLL2_Q,
	STM32MP1_SHRES_PLL2_R,
	STM32MP1_SHRES_PLL3,
	STM32MP1_SHRES_PLL3_P,
	STM32MP1_SHRES_PLL3_Q,
	STM32MP1_SHRES_PLL3_R,

	STM32MP1_SHRES_COUNT
};

void stm32mp1_register_secure_periph(unsigned int id);
void stm32mp1_register_shared_periph(unsigned int id);
void stm32mp1_register_non_secure_periph(unsigned int id);
void stm32mp_register_secure_periph_iomem(uintptr_t base);
void stm32mp_register_non_secure_periph_iomem(uintptr_t base);
void stm32mp_register_secure_gpio(unsigned int bank, unsigned int pin);
void stm32mp_register_non_secure_gpio(unsigned int bank, unsigned int pin);
void stm32mp1_register_etzpc_decprot(unsigned int id,
				     enum etzpc_decprot_attributes attr);

bool stm32mp1_periph_is_shared(unsigned long id);
bool stm32mp1_periph_is_non_secure(unsigned long id);
bool stm32mp1_periph_is_secure(unsigned long id);
bool stm32mp1_periph_is_unregistered(unsigned long id);

bool stm32mp_gpio_bank_is_shared(unsigned int bank);
bool stm32mp_gpio_bank_is_non_secure(unsigned int bank);
bool stm32mp_gpio_bank_is_secure(unsigned int bank);

bool stm32mp1_clock_is_shareable(unsigned long clock_id);
bool stm32mp1_clock_is_shared(unsigned long clock_id);
bool stm32mp1_clock_is_non_secure(unsigned long clock_id);

void stm32mp1_driver_init_late(void);

#endif /* STM32MP1_SHARED_RESOURCES_H */
