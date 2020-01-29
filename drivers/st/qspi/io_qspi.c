/*
 * Copyright (c) 2015-2019, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <debug.h>
#include <io_driver.h>
#include <io_qspi.h>
#include <io_storage.h>
#include <libfdt.h>
#include <mmio.h>
#include <platform.h>
#include <platform_def.h>
#include <string.h>

#define QSPI_COMPAT		"st,stm32f469-qspi"

#define TIMEOUT_100_US		us2tick(100)

/* QSPI device functions */
static int qspi_dev_open(const uintptr_t init_params,
			 io_dev_info_t **dev_info);
static int qspi_block_open(io_dev_info_t *dev_info, const uintptr_t spec,
			   io_entity_t *entity);
static int qspi_dev_init(io_dev_info_t *dev_info, const uintptr_t init_params);
static int qspi_block_seek(io_entity_t *entity, int mode,
			   signed long long offset);
static int qspi_block_read(io_entity_t *entity, uintptr_t buffer,
			   size_t length, size_t *length_read);
static int qspi_block_close(io_entity_t *entity);
static int qspi_dev_close(io_dev_info_t *dev_info);
static io_type_t device_type_qspi(void);

static QSPI_HandleTypeDef *hsd;
static uint64_t seek_offset;

static struct dt_node_info qspi_node_info;
static uintptr_t qspi_mm_base;
static size_t qspi_mm_size;
static bool qspi_memory_map;

static const io_dev_connector_t qspi_dev_connector = {
	.dev_open = qspi_dev_open
};

static const io_dev_funcs_t qspi_dev_funcs = {
	.type = device_type_qspi,
	.open = qspi_block_open,
	.seek = qspi_block_seek,
	.size = NULL,
	.read = qspi_block_read,
	.write = NULL,
	.close = qspi_block_close,
	.dev_init = qspi_dev_init,
	.dev_close = qspi_dev_close,
};

static const io_dev_info_t qspi_dev_info = {
	.funcs = &qspi_dev_funcs,
	.info = (uintptr_t)0,
};

/* Identify the device type as memmap */
static io_type_t device_type_qspi(void)
{
	return IO_TYPE_QSPI;
}

/* Open a connection to the qspi device */
static int qspi_dev_open(const uintptr_t init_params, io_dev_info_t **dev_info)
{
	int qspi_node;
	int qspi_subnode = 0;
	int ret;
	uint32_t fsize;
	uint32_t flash_max_freq = 0;
	uint32_t div = 8U; /* Default clock divider */
	uint32_t presc;
	void *fdt;
	const fdt32_t *cuint = NULL;

	assert(dev_info);

	if (fdt_get_address(&fdt) == 0) {
		panic();
	}

	hsd = (QSPI_HandleTypeDef *)init_params;

	qspi_node = dt_get_node(&qspi_node_info, -1, QSPI_COMPAT);
	assert(qspi_node_info.base == (uintptr_t)hsd->instance);

	*dev_info = (io_dev_info_t *)&qspi_dev_info;

	ret = fdt_get_reg_props_by_name(qspi_node, "qspi_mm", &qspi_mm_base,
					&qspi_mm_size);
	if (ret != 0) {
		INFO("NOR: Indirect mode\n");
		qspi_memory_map = false;

		/*
		 * Indirect mode is already initialized by bootrom:
		 * no configuration to be done.
		 */
		return 0;
	}

	/* Size of NOR is 2^(fsise + 1) */
	fsize = ((__builtin_ctz(qspi_mm_size) - 1) << QSPI_DCR_FSIZE_SHIFT) &
		QSPI_DCR_FSIZE_MASK;

	/* switch to memory mapped mode */
	INFO("NOR: Memory mapped mode\n");
	qspi_memory_map = true;

	fdt_for_each_subnode(qspi_subnode, fdt, qspi_node) {
		cuint = fdt_getprop(fdt, qspi_subnode, "reg", NULL);

		if ((cuint != NULL) && (fdt32_to_cpu(*cuint) == 0U)) {
			/* Flash node found */
			cuint = fdt_getprop(fdt, qspi_subnode,
					    "spi-max-frequency", NULL);
			if (cuint != NULL) {
				flash_max_freq = fdt32_to_cpu(*cuint);
			}

			break;
		}
	}

	if (flash_max_freq != 0) {
		div = div_round_up(stm32mp_clk_get_rate(qspi_node_info.clock),
				   flash_max_freq);
	}

	presc = div - 1U;

	if (presc > UINT8_MAX) {
		presc = UINT8_MAX;
	}

	presc <<= QSPI_CR_PRESCALER_SHIFT;

	/* Check if QuadSPI was configured in dual flash mode */
	if ((hsd->instance->CR & QSPI_CR_DFM) != 0U) {
		hsd->instance->CR = QSPI_CR_EN | presc | QSPI_CR_DFM;
		if (hsd->is_dual == 0U) {
			WARN("Dual NOR configured in IP\n");
			WARN("  but set as single in boot context\n");
			WARN("  -> Try Dual NOR boot\n");
		}
	} else {
		hsd->instance->CR = QSPI_CR_EN | presc;
		if (hsd->is_dual != 0U) {
			WARN("Single NOR configured in IP\n");
			WARN("  but set as dual in boot context\n");
			WARN("  -> Try Single NOR boot\n");
		}
	}

	hsd->instance->DCR = fsize | QSPI_DCR_CSHT;
	hsd->instance->CCR = QSPI_DFLT_READ_FLAGS | QSPI_CCR_FMODE_MM;

	return 0;
}

static int qspi_dev_init(io_dev_info_t *dev_info, const uintptr_t init_params)
{
	return 0;
}

/* Close a connection to the qspi device */
static int qspi_dev_close(io_dev_info_t *dev_info)
{
	uint64_t start;

	/* Send Abort command to end all transfers */
	hsd->instance->CR |= QSPI_CR_ABORT;

	start = timeout_start();
	while ((hsd->instance->CR & QSPI_CR_ABORT) != 0U) {
		if (timeout_elapsed(start, TIMEOUT_100_US)) {
			return -ETIMEDOUT;
		}
	}

	return 0;
}

/* Open a file on the qspi device */
static int qspi_block_open(io_dev_info_t *dev_info, const uintptr_t spec,
			   io_entity_t *entity)
{
	seek_offset = 0;
	return 0;
}

/* Seek to a particular file offset on the qspi device */
static int qspi_block_seek(io_entity_t *entity, int mode,
			   signed long long offset)
{
	seek_offset = offset;
	return 0;
}

/* Read blocks in indirect mode */
static int qspi_block_read_indr(io_entity_t *entity, uintptr_t buffer,
				size_t length, size_t *length_read)
{
	uint32_t local_length = (uint32_t)length;
	uint8_t *data = (uint8_t *)buffer;
	uint8_t *qspi_dr_u8;
	uint64_t start;
	int ret = 0;

	assert(hsd);

	qspi_dr_u8 = (uint8_t *)&hsd->instance->DR;

	/* Clear all flags */
	hsd->instance->FCR = QSPI_SR_TCF | QSPI_SR_FTF |
		QSPI_FCR_CTOF | QSPI_FCR_CSMF;
	hsd->instance->DLR = local_length - 1;
	hsd->instance->CCR = QSPI_DFLT_READ_FLAGS | QSPI_CCR_FMODE;
	hsd->instance->AR = seek_offset;

	while (local_length != 0U) {
		*data++ = *qspi_dr_u8;
		local_length--;
	}

	*length_read = length;

	start = timeout_start();
	while ((hsd->instance->SR & QSPI_SR_BUSY) != 0U) {
		if (timeout_elapsed(start, TIMEOUT_100_US)) {
			ret = -ETIMEDOUT;
			break;
		}
	}

	return ret;
}

static int qspi_block_read_mm_part(io_entity_t *entity, uintptr_t buffer,
				   size_t length, size_t *length_read)
{
	uint64_t start;
	int ret = 0;

	assert(hsd);

	memcpy((uint8_t *)buffer, (uint8_t *)qspi_mm_base + seek_offset,
	       length);

	*length_read = length;

	start = timeout_start();
	while ((hsd->instance->SR & QSPI_SR_BUSY) != 0U) {
		if (timeout_elapsed(start, TIMEOUT_100_US)) {
			ret = -ETIMEDOUT;
			break;
		}
	}

	/* Send abort to avoid prefetch issues */
	hsd->instance->CR |= QSPI_CR_ABORT;

	start = timeout_start();
	while ((hsd->instance->CR & QSPI_CR_ABORT) != 0U) {
		if (timeout_elapsed(start, TIMEOUT_100_US)) {
			ret = -ETIMEDOUT;
			break;
		}
	}

	return ret;

}

/* Read blocks in memory map mode */
static int qspi_block_read_mm(io_entity_t *entity, uintptr_t buffer,
			      size_t length, size_t *length_read)
{
	if (seek_offset + length + 1 >= qspi_mm_size) {
		if (length > QSPI_NOR_LBA_SIZE) {
			size_t length_read_mm, length_read_indr;

			qspi_block_read_mm_part(entity, buffer,
						length - QSPI_NOR_LBA_SIZE,
						&length_read_mm);

			seek_offset += length - QSPI_NOR_LBA_SIZE;

			qspi_block_read_indr(entity, buffer + length -
					     QSPI_NOR_LBA_SIZE,
					     QSPI_NOR_LBA_SIZE,
					     &length_read_indr);

			*length_read = length_read_mm + length_read_indr;
		} else {
			qspi_block_read_indr(entity, buffer, length,
					     length_read);
		}
	} else {
		qspi_block_read_mm_part(entity, buffer, length, length_read);
	}

	return 0;

}

/* Read data from a file on the qspi device */
static int qspi_block_read(io_entity_t *entity, uintptr_t buffer,
			   size_t length, size_t *length_read)
{
	if (qspi_memory_map) {
		return qspi_block_read_mm(entity, buffer, length, length_read);
	}

	return qspi_block_read_indr(entity, buffer, length, length_read);
}

/* Close a file on the qspi device */
static int qspi_block_close(io_entity_t *entity)
{
	return 0;
}

/* Exported functions */

/* Register the qspi driver with the IO abstraction */
int register_io_dev_qspi(const io_dev_connector_t **dev_con)
{
	int result;

	assert(dev_con);

	result = io_register_device(&qspi_dev_info);
	if (result == 0) {
		*dev_con = &qspi_dev_connector;
	}

	return result;
}
