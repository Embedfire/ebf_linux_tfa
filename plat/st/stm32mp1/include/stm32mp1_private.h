/*
 * Copyright (c) 2015-2019, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STM32MP1_PRIVATE_H
#define STM32MP1_PRIVATE_H

#include <bl_common.h>
#include <boot_api.h>
#include <stdbool.h>

enum boot_device_e {
	BOOT_DEVICE_USB,
	BOOT_DEVICE_BOARD
};

void configure_mmu(void);

void stm32mp_mask_timer(void);
void __dead2 stm32mp_wait_cpu_reset(void);

void stm32mp1_arch_security_setup(void);
void stm32mp1_security_setup(void);
void stm32mp1_sp_min_security_setup(void);

enum boot_device_e get_boot_device(void);

#if STM32MP_UART_PROGRAMMER
uintptr_t get_uart_address(uint32_t instance_nb);
#endif

void stm32mp1_gic_pcpu_init(void);
void stm32mp1_gic_init(void);

enum etzpc_decprot_attributes stm32mp_etzpc_binding2decprot(uint32_t mode);

void stm32mp1_syscfg_init(void);
void stm32mp1_syscfg_enable_io_compensation(void);
void stm32mp1_syscfg_disable_io_compensation(void);

#endif /* STM32MP1_PRIVATE_H */
