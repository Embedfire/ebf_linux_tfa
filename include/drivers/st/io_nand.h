/*
 * Copyright (c) 2015-2017, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __IO_NAND_H__
#define __IO_NAND_H__

#define STM32_NAND_MAX_BLOCK_SIZE	512

typedef struct nand_device_info {
	uint64_t                device_size;    /* Size of device in bytes */
	uint32_t                block_size;       /* block size in bytes */
} nand_device_info_t;

int register_io_dev_nand(const io_dev_connector_t **dev_con);

#endif /* __IO_NAND_H__ */
