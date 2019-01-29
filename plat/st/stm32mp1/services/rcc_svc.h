/*
 * Copyright (c) 2017, STMicroelectronics - All Rights Reserved
 * Copyright (c) 2017, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef RCC_SVC_H
#define RCC_SVC_H

uint32_t rcc_scv_handler(uint32_t x1, uint32_t x2, uint32_t x3);
uint32_t rcc_cal_scv_handler(uint32_t x1);

#endif /* RCC_SVC_H */
