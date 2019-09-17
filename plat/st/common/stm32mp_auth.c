/*
 * Copyright (c) 2016-2018, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <debug.h>
#include <errno.h>
#include <hash_sec.h>
#include <io_storage.h>
#include <platform_def.h>
#include <stm32mp_auth.h>
#include <stm32mp_common.h>

static const struct auth_ops *stm32mp_auth_ops;

void stm32mp_init_auth(struct auth_ops *init_ptr)
{
	if ((init_ptr == NULL) ||
	    (init_ptr->check_key == NULL) ||
	    (init_ptr->verify_signature == NULL)) {
		panic();
	}

	stm32mp_auth_ops = init_ptr;
}

int check_header(boot_api_image_header_t *header, uintptr_t buffer)
{
	uint32_t i;
	uint32_t img_checksum = 0;

	/*
	 * Check header/payload validity:
	 *	- Header magic
	 *	- Header version
	 *	- Payload checksum
	 */
	if (header->magic != BOOT_API_IMAGE_HEADER_MAGIC_NB) {
		ERROR("Header magic is not correct\n");
		return -EINVAL;
	}

	if (header->header_version != BOOT_API_HEADER_VERSION) {
		ERROR("Header version is not correct\n");
		return -EINVAL;
	}

	for (i = 0; i < header->image_length; i++) {
		img_checksum += *(uint8_t *)(buffer + i);
	}

	if (header->payload_checksum != img_checksum) {
		ERROR("Payload checksum is not correct:\n");
		ERROR("  Computed: 0x%x (awaited: 0x%x)\n", img_checksum,
		      header->payload_checksum);
		return -EINVAL;
	}

	return 0;
}

int check_authentication(boot_api_image_header_t *header, uintptr_t buffer)
{
	uint32_t sec_closed, uret;
	HASH_HandleTypeDef hhash;
	uint8_t image_hash[BOOT_API_SHA256_DIGEST_SIZE_IN_BYTES];
	uint32_t header_skip_cksum = sizeof(header->magic) +
		sizeof(header->image_signature) +
		sizeof(header->payload_checksum);

	uret = bsec_read_otp(&sec_closed, BOOT_API_OTP_MODE_WORD_NB);
	if (uret != 0) {
		ERROR("Error reading OTP configuration\n");
		return -EINVAL;
	}

	/* Check Security Status */
	if ((sec_closed & BIT(BOOT_API_OTP_MODE_CLOSED_BIT_POS)) == 0U) {
		if (header->option_flags != 0U) {
			WARN("Skip signature check (header option)\n");
			return 0;
		}
		INFO("Check signature on Non-Full-Secured platform\n");
	}

	/* Check Public Key */
	if (stm32mp_auth_ops->check_key(header->ecc_pubk, NULL) != STD_OK) {
		return -EINVAL;
	}

	/* Compute end of header hash and payload hash */
	uret = HASH_SHA256_Init(&hhash);
	if (uret != STD_OK) {
		ERROR("Hash init failed\n");
		return -EBUSY;
	}

	uret = HASH_SHA256_Accumulate(&hhash,
				      (uint8_t *)&header->header_version,
				      sizeof(boot_api_image_header_t) -
				      header_skip_cksum);
	if (uret != STD_OK) {
		ERROR("Hash of header failed\n");
		return -EINVAL;
	}

	uret = HASH_SHA256_Start(&hhash, (uint8_t *)buffer,
				 header->image_length, image_hash,
				 HASH_TIMEOUT_VALUE);
	if (uret != STD_OK) {
		ERROR("Hash of payload failed\n");
		return -EINVAL;
	}

	uret = HASH_SHA256_Finish(&hhash, image_hash, HASH_TIMEOUT_VALUE);
	if (uret != STD_OK) {
		ERROR("Hash of payload failed\n");
		return -EINVAL;
	}

	/* Verify signature */
	if (stm32mp_auth_ops->verify_signature
	    (image_hash, header->ecc_pubk,
	     header->image_signature,
	     header->ecc_algo_type) != STD_OK) {
		return -EINVAL;
	}

	return 0;
}

