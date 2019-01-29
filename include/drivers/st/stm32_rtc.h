/*
 * Copyright (c) 2017-2018, STMicroelectronics - All Rights Reserved
 * Copyright (c) 2017, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __PLAT_RTC_H__
#define __PLAT_RTC_H__

#include <stdbool.h>

#define RTC_TR		0x00U
#define RTC_DR		0x04U
#define RTC_SSR		0x08U
#define RTC_ICSR	0x0CU
#define RTC_PRER	0x10U
#define RTC_WUTR	0x14U
#define RTC_CR		0x18U
#define RTC_SMCR	0x20U
#define RTC_WPR		0x24U
#define RTC_CALR	0x28U
#define RTC_SHIFTR	0x2CU
#define RTC_TSTR	0x30U
#define RTC_TSDR	0x34U
#define RTC_TSSSR	0x38U
#define RTC_ALRMAR	0x40U
#define RTC_ALRMASSR	0x44U
#define RTC_ALRMBR	0x48U
#define RTC_ALRMBSSR	0x4CU
#define RTC_SR		0x50U
#define RTC_SCR		0x5CU
#define RTC_OR		0x60U

struct stm32_rtc_calendar {
	uint32_t ssr;
	uint32_t tr;
	uint32_t dr;
};

enum months {
	JANUARY = 1,
	FEBRUARY,
	MARCH,
	APRIL,
	MAY,
	JUNE,
	JULY,
	AUGUST,
	SEPTEMBER,
	OCTOBER,
	NOVEMBER,
	DECEMBER,
	NB_MONTHS = 12
};

struct stm32_rtc_time {
	uint32_t hour;
	uint32_t min;
	uint32_t sec;
	uint32_t wday;
	uint32_t day;
	enum months month;
	uint32_t year;
};

void stm32_rtc_get_calendar(struct stm32_rtc_calendar *calendar);
unsigned long long stm32_rtc_diff_calendar(struct stm32_rtc_calendar *current,
					   struct stm32_rtc_calendar *ref);
void stm32_rtc_set_tamper_timestamp(void);
bool stm32_rtc_is_timestamp_enable(void);
void stm32_rtc_get_timestamp(struct stm32_rtc_time *tamp_ts);
int stm32_rtc_init(void);

/* SMP protection on RTC registers access */
void stm32_rtc_regs_lock(void);
void stm32_rtc_regs_unlock(void);

#endif /* __PLAT_RTC_H__ */
