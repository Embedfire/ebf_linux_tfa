/*
 * Copyright (c) 2017-2018, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STM32MP1_DDR_HELPERS_H
#define STM32MP1_DDR_HELPERS_H

#include <stdbool.h>

void ddr_enable_clock(void);
int ddr_sw_self_refresh_exit(void);
int ddr_standby_sr_entry(uint32_t *zq0cr0_zdata);
void ddr_sr_mode_ssr(void);
void ddr_sr_mode_asr(void);
void ddr_sr_mode_hsr(void);
bool ddr_is_nonsecured_area(uintptr_t address, uint32_t length);

#endif /* STM32MP1_DDR_HELPERS_H */
