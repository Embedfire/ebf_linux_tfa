/*
 * Copyright (c) 2015-2018, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STM32MP_AUTH_H
#define STM32MP_AUTH_H

#include <boot_api.h>

struct auth_ops {
	uint32_t (*check_key)(uint8_t *p_pub_key_in,
			      uint8_t *p_pub_key_out);
	uint32_t (*verify_signature)(uint8_t *p_hash_in, uint8_t *p_pub_key_in,
				     uint8_t *p_signature, uint32_t ecc_algo);
};

void stm32mp_init_auth(struct auth_ops *init_ptr);
int check_header(boot_api_image_header_t *header, uintptr_t buffer);
int check_authentication(boot_api_image_header_t *header, uintptr_t buffer);

#endif /* STM32MP_AUTH_H */
