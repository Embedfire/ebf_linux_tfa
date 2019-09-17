/*
 * Copyright (c) 2018, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __STM32_IWDG_H__
#define __STM32_IWDG_H__

#include <stdint.h>

#define IWDG_HW_ENABLED			BIT(0)
#define IWDG_DISABLE_ON_STOP		BIT(1)
#define IWDG_DISABLE_ON_STANDBY		BIT(2)

int stm32_iwdg_init(void);
void stm32_iwdg_refresh(uint32_t instance);
void __dead2 stm32_iwdg_it_handler(int id);

#endif /* __STM32_IWDG_H__ */
