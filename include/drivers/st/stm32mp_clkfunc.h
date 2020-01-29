/*
 * Copyright (c) 2017-2019, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STM32MP_CLKFUNC_H
#define STM32MP_CLKFUNC_H

#include <libfdt.h>
#include <stdbool.h>

int fdt_get_rcc_node(void);
int fdt_rcc_read_uint32_array(const char *prop_name,
			      uint32_t *array, uint32_t count);
uint32_t fdt_rcc_read_uint32_default(const char *prop_name,
				     uint32_t dflt_value);
int fdt_rcc_subnode_offset(const char *name);
const fdt32_t *fdt_rcc_read_prop(const char *prop_name, int *lenp);
bool fdt_get_rcc_secure_status(void);

uintptr_t fdt_get_stgen_base(void);
int fdt_get_clock_id(int node);
int fdt_get_clock_id_by_name(int node, const char *name);
unsigned long fdt_get_uart_clock_freq(uintptr_t instance);

bool fdt_is_pll1_predefined(void);

#endif /* STM32MP_CLKFUNC_H */
