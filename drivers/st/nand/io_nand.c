/*
 * Copyright (c) 2015-2017, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <debug.h>
#include <io_driver.h>
#include <io_nand.h>
#include <io_storage.h>
#include <nand.h>
#include <string.h>
#include <utils_def.h>

/* NAND device functions */
static int nand_dev_open(const uintptr_t init_params, io_dev_info_t **dev_info);
static int nand_block_open(io_dev_info_t *dev_info, const uintptr_t spec,
			   io_entity_t *entity);
static int nand_dev_init(io_dev_info_t *dev_info, const uintptr_t init_params);
static int nand_block_seek(io_entity_t *entity, int mode,
			   signed long long offset);
static int nand_block_read(io_entity_t *entity, uintptr_t buffer,
			   size_t length, size_t *length_read);
static int nand_block_close(io_entity_t *entity);
static int nand_dev_close(io_dev_info_t *dev_info);
static io_type_t device_type_nand(void);

static NAND_HandleTypeDef *hnand;
static uint64_t seek_offset;

static const io_dev_connector_t nand_dev_connector = {
	.dev_open = nand_dev_open
};

static const io_dev_funcs_t nand_dev_funcs = {
	.type = device_type_nand,
	.open = nand_block_open,
	.seek = nand_block_seek,
	.size = NULL,
	.read = nand_block_read,
	.write = NULL,
	.close = nand_block_close,
	.dev_init = nand_dev_init,
	.dev_close = nand_dev_close,
};

static const io_dev_info_t nand_dev_info = {
	.funcs = &nand_dev_funcs,
	.info = (uintptr_t)0,
};

/* Identify the device type as memmap */
static io_type_t device_type_nand(void)
{
	return IO_TYPE_NAND;
}

/* Open a connection to the nand device */
static int nand_dev_open(const uintptr_t init_params, io_dev_info_t **dev_info)
{
	uint32_t nb_page_per_block, page_size;

	assert(dev_info);
	*dev_info = (io_dev_info_t *)&nand_dev_info;

	hnand = (NAND_HandleTypeDef *)init_params;

	if (nand_initialize(hnand) != STD_OK)
		return -EIO;

	hnand->Info.page_size_shift = 0;

	page_size = hnand->Info.PageSize;
	if (!IS_POWER_OF_TWO(page_size))
		return -EINVAL;

	while (page_size >>= 1)
		hnand->Info.page_size_shift++;

	nb_page_per_block = hnand->Info.BlockSize;
	if (!IS_POWER_OF_TWO(nb_page_per_block))
		return -EINVAL;

	/* Block size is (page_size * number of pages per blocks) */
	hnand->Info.block_size_shift = hnand->Info.page_size_shift;
	while (nb_page_per_block >>= 1)
		hnand->Info.block_size_shift++;

	return 0;
}

static int nand_dev_init(io_dev_info_t *dev_info, const uintptr_t init_params)
{
	return 0;
}

/* Close a connection to the nand device */
static int nand_dev_close(io_dev_info_t *dev_info)
{
	return 0;
}

/* Open a file on the nand device */
static int nand_block_open(io_dev_info_t *dev_info, const  uintptr_t spec,
			   io_entity_t *entity)
{
	seek_offset = 0;
	return 0;
}

/* Seek to a particular file offset on the nand device */
static int nand_block_seek(io_entity_t *entity, int mode,
			   signed long long offset)
{
	seek_offset = offset;
	return 0;
}

/* Read data from a file on the nand device */
static int nand_block_read(io_entity_t *entity, uintptr_t buffer,
			   size_t length, size_t *length_read)
{
	uint32_t block_size_shift = hnand->Info.block_size_shift;
	uint32_t page_size_shift = hnand->Info.page_size_shift;
	uint32_t buffer_index = 0;
	uint32_t num_sectors_read = seek_offset / BCH_PAGE_SECTOR;
	uint64_t number_sectors_to_read = length / BCH_PAGE_SECTOR;
	uint64_t nand_address_block, nand_address_page;
	NAND_AddressTypeDef nand_address;

	*length_read = 0;

	nand_address_block = seek_offset;
	while (block_size_shift) {
		nand_address_block >>= 1;
		if (!nand_address_block)
			break;
		block_size_shift--;
	}

	nand_address.Block = (uint16_t)nand_address_block;

	nand_address_page = seek_offset & ((1 << hnand->Info.block_size_shift)
					   - 1);
	while (page_size_shift) {
		nand_address_page >>= 1;
		if (!nand_address_page)
			break;
		page_size_shift--;
	}

	nand_address.Page = (uint16_t)nand_address_page;

	while (NAND_Check_Bad_Block(hnand, &nand_address) == BAD_BLOCK) {
		nand_address.Block++;
		if (nand_address.Block >= hnand->Info.BlockNb) {
			ERROR("Cannot find valid block\n");
			return -EIO;
		}
	}

	if ((length % BCH_PAGE_SECTOR) != 0) {
		/*
		 * yes, remainder required,
		 * adjust total number of sectors required
		 * one more page needed to read,
		 * data will not fill all page
		 */
		number_sectors_to_read += 1;
	}

	/*
	 * Proceed here with reading/copying of
	 * (number_of_pages_remaining_to_read * BCH_PAGE_SECTOR)
	 * from NAND memory to SYSRAM download area
	 */
	while (number_sectors_to_read != 0) {
		uint32_t bch_sector_nb = num_sectors_read %
			(hnand->Info.PageSize / BCH_PAGE_SECTOR);

		if (NAND_Read_Logical_Page(hnand, &nand_address,
					   (uint8_t *)(buffer +
						       (buffer_index *
							BCH_PAGE_SECTOR)),
					   bch_sector_nb) != STD_OK) {
			VERBOSE("Page read failed or end of NAND reached\n");
			return -EIO;
		}

		/* Increment read sectors number */
		num_sectors_read++;
		buffer_index++;

		/* Decrement sectors to read */
		number_sectors_to_read--;

		/* Increment the NAND address */
		NAND_Address_Inc(hnand, &nand_address, num_sectors_read);
	}

	*length_read = ((num_sectors_read - (seek_offset / BCH_PAGE_SECTOR)) *
			BCH_PAGE_SECTOR);

	return 0;
}

/* Close a file on the nand device */
static int nand_block_close(io_entity_t *entity)
{
	return 0;
}

/* Exported functions */

/* Register the nand driver with the IO abstraction */
int register_io_dev_nand(const io_dev_connector_t **dev_con)
{
	int result;

	assert(dev_con);

	result = io_register_device(&nand_dev_info);
	if (!result)
		*dev_con = &nand_dev_connector;

	return result;
}
