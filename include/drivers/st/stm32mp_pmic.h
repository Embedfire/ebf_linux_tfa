/*
 * Copyright (c) 2017-2019, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STM32MP_PMIC_H
#define STM32MP_PMIC_H

#include <platform_def.h>
#include <stdbool.h>
#include <stdint.h>
#include <stm32mp_regulator.h>

int dt_pmic_status(void);
int pmic_configure_boot_on_regulators(void);
int pmic_set_lp_config(const char *node_name);
bool initialize_pmic_i2c(void);
bool is_pmic_regulator(struct stm32mp_regulator *regu);
void bind_pmic_regulator(struct stm32mp_regulator *regu);
void initialize_pmic(void);
void configure_pmic(void);
#if STM32MP1_DEBUG_ENABLE
int pmic_keep_debug_unit(void);
#endif
#if DEBUG
void print_pmic_info_and_debug(void);
#else
static inline void print_pmic_info_and_debug(void)
{
}
#endif
int pmic_ddr_power_init(enum ddr_type ddr_type);

#endif /* STM32MP_PMIC_H */
