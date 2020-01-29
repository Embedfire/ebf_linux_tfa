/*
 * Copyright (c) 2015-2019, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __PLAT_DBGMCU_H__
#define __PLAT_DBGMCU_H__

#include <stdint.h>

#if LOG_LEVEL >= LOG_LEVEL_VERBOSE
#define VERBOSE_HEXDUMP8(buf, len) stm32mp1_dbgmcu_hexdump8(buf, len)
#else
#define VERBOSE_HEXDUMP8(buf, len)
#endif

int stm32mp1_dbgmcu_get_chip_version(uint32_t *chip_version);
int stm32mp1_dbgmcu_get_chip_dev_id(uint32_t *chip_dev_id);
int stm32mp1_dbgmcu_freeze_iwdg2(void);
int stm32mp1_dbgmcu_boot_debug_info(void);
void stm32mp1_dbgmcu_clear_boot_info(void);
bool stm32mp1_dbgmcu_is_debug_on(void);
void stm32mp1_dbgmcu_hexdump8(uint8_t *buf, uint32_t len);

#endif /* __PLAT_DBGMCU_H__ */
