/*
 * Copyright (c) 2017-2018, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STM32MP_PMIC_H
#define STM32MP_PMIC_H

#include <platform_def.h>
#include <stdbool.h>
#include <stdint.h>

int dt_pmic_status(void);
int dt_pmic_configure_boot_on_regulators(void);
int dt_pmic_set_lp_config(const char *node_name);
bool initialize_pmic_i2c(void);
void initialize_pmic(void);
#if STM32MP1_DEBUG_ENABLE
int pmic_keep_debug_unit(void);
#endif
int pmic_ddr_power_init(enum ddr_type ddr_type);

#endif /* STM32MP_PMIC_H */
