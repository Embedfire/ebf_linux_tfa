/*
 * Copyright (c) 2016-2018, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __STM32MP1_SMC_H__
#define __STM32MP1_SMC_H__

#include <platform_def.h>

/*
 * SMC function IDs for STM32 Service queries
 * STM32 SMC services use the space between 0x82000000 and 0x8200FFFF
 * like this is defined in SMC calling Convention by ARM
 * for SiP (silicon Partner)
 * https://developer.arm.com/docs/den0028/latest
 */

/* Secure Service access from Non-secure */
#define STM32_SMC_RCC			0x82001000
#define STM32_SMC_PWR			0x82001001
#define STM32_SMC_RCC_CAL		0x82001002
#define STM32_SMC_BSEC			0x82001003

/* Low Power services */
#define STM32_SMC_SR_MODE		0x82001004
#define STM32_SMC_PD_DOMAIN		0x82001008

/* SMC function IDs for SiP Service queries */
#define STM32_SIP_SVC_CALL_COUNT	0x8200ff00
#define STM32_SIP_SVC_UID		0x8200ff01
/*					0x8200ff02 is reserved */
#define STM32_SIP_SVC_VERSION		0x8200ff03

/* STM32 SiP Service Calls version numbers */
#define STM32_SIP_SVC_VERSION_MAJOR	0x0
#define STM32_SIP_SVC_VERSION_MINOR	0x1

/* Number of STM32 SiP Calls implemented */
#define STM32_COMMON_SIP_NUM_CALLS	10

/* Register access service use for RCC/RTC/PWR */
#define STM32_SMC_REG_READ		0x0
#define STM32_SMC_REG_WRITE		0x1
#define STM32_SMC_REG_SET		0x2
#define STM32_SMC_REG_CLEAR		0x3

/* Service for BSEC */
#define STM32_SMC_READ_SHADOW		0x01
#define STM32_SMC_PROG_OTP		0x02
#define STM32_SMC_WRITE_SHADOW		0x03
#define STM32_SMC_READ_OTP		0x04
#define STM32_SMC_READ_ALL		0x05
#define STM32_SMC_WRITE_ALL		0x06

/* SMC error codes */
#define STM32_SMC_OK			0x00000000U
#define STM32_SMC_NOT_SUPPORTED		0xFFFFFFFFU
#define STM32_SMC_FAILED		0xFFFFFFFEU
#define STM32_SMC_INVALID_PARAMS	0xFFFFFFFDU

/* DDR Self-Refresh modes */
#define STM32_SMC_SR_MODE_SSR		0x0
#define STM32_SMC_SR_MODE_ASR		0x1
#define STM32_SMC_SR_MODE_HSR		0x2

#endif /* __STM32MP1_SMC_H__ */
