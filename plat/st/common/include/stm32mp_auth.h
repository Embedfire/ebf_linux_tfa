/*
 * Copyright (c) 2015-2018, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STM32MP_AUTH_H
#define STM32MP_AUTH_H

#include <boot_api.h>

int check_header(boot_api_image_header_t *header, uintptr_t buffer);
int check_authentication(boot_api_image_header_t *header, uintptr_t buffer);

#endif /* STM32MP_AUTH_H */
