/*
 * Copyright (c) 2015-2019, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <arch_helpers.h>
#include <assert.h>
#include <boot_api.h>
#include <debug.h>
#include <io_block.h>
#include <io_driver.h>
#include <io_dummy.h>
#include <io_mmc.h>
#if STM32MP_FMC_NAND
#include <io_nand.h>
#endif
#include <io_programmer_st_usb.h>
#if STM32MP1_QSPI_NOR
#include <io_qspi.h>
#endif
#include <io_stm32image.h>
#include <io_storage.h>
#if STM32MP_UART_PROGRAMMER
#include <io_uart.h>
#endif
#include <mmc.h>
#include <mmio.h>
#if STM32MP_FMC_NAND
#include <nand.h>
#endif
#include <partition.h>
#include <platform.h>
#include <platform_def.h>
#include <stm32_gpio.h>
#include <stm32_sdmmc2.h>
#include <stm32mp_common.h>
#include <stm32mp_dt.h>
#include <string.h>
#ifdef STM32MP_USB
#include <usb_core.h>
#include <usb_ctx.h>
#include <usb_dwc2.h>
#include <usb_st_dfu.h>
#endif
#include <utils.h>

/* IO devices */
static const io_dev_connector_t *dummy_dev_con;
static uintptr_t dummy_dev_handle;
static uintptr_t dummy_dev_spec;

static uintptr_t image_dev_handle;

#if STM32MP_SDMMC || STM32MP_EMMC
static io_block_spec_t gpt_block_spec = {
	.offset = 0,
	.length = 34 * MMC_BLOCK_SIZE, /* Size of GPT table */
};

static uint32_t block_buffer[MMC_BLOCK_SIZE] __aligned(MMC_BLOCK_SIZE);

static const io_block_dev_spec_t mmc_block_dev_spec = {
	/* It's used as temp buffer in block driver */
	.buffer = {
		.offset = (size_t)&block_buffer,
		.length = MMC_BLOCK_SIZE,
	},
	.ops = {
		.read = mmc_read_blocks,
		.write = NULL,
	},
	.block_size = MMC_BLOCK_SIZE,
};
#endif

static uintptr_t storage_dev_handle;
#if STM32MP_SDMMC || STM32MP_EMMC
static const io_dev_connector_t *mmc_dev_con;
#endif

#if STM32MP1_QSPI_NOR
static const io_dev_connector_t *qspi_dev_con;

static QSPI_HandleTypeDef qspi_dev_spec = {
	.instance = (QUADSPI_TypeDef *)STM32MP1_QSPI1_BASE,
	.is_dual = 0,
};
#endif

#if STM32MP_UART_PROGRAMMER
/* uart*/
static const io_dev_connector_t *uart_dev_con;

static UART_HandleTypeDef uart_programmer = {
	.Init.BaudRate			= STM32MP_UART_BAUDRATE,
	.Init.StopBits			= UART_STOPBITS_1,
	.Init.HwFlowCtl			= UART_HWCONTROL_NONE,
	.Init.Mode			= UART_MODE_TX_RX,
	.Init.OverSampling		= UART_OVERSAMPLING_16,
	.Init.FIFOMode			= UART_FIFOMODE_ENABLE,
	.AdvancedInit.AdvFeatureInit	= UART_ADVFEATURE_AUTOBAUDRATE_INIT,
	.AdvancedInit.AutoBaudRateEnable = UART_ADVFEATURE_AUTOBAUDRATE_DISABLE,
};
#endif /* STM32MP_UART_PROGRAMMER */

#ifdef STM32MP_USB
static usb_handle_t usb_core_handle;
static usb_dfu_handle_t usb_dfu_handle;
static pcd_handle_t pcd_handle;
static const io_dev_connector_t *usb_dev_con;
#endif /*STM32MP_USB*/

#if STM32MP_FMC_NAND
/* nand */
static const io_dev_connector_t *nand_dev_con;
static NAND_HandleTypeDef nand_dev_spec;
#endif

#ifdef AARCH32_SP_OPTEE
static const struct stm32image_part_info optee_header_partition_spec = {
	.name = OPTEE_HEADER_IMAGE_NAME,
	.binary_type = OPTEE_HEADER_BINARY_TYPE,
};

static const struct stm32image_part_info optee_pager_partition_spec = {
	.name = OPTEE_PAGER_IMAGE_NAME,
	.binary_type = OPTEE_PAGER_BINARY_TYPE,
};

static const struct stm32image_part_info optee_paged_partition_spec = {
	.name = OPTEE_PAGED_IMAGE_NAME,
	.binary_type = OPTEE_PAGED_BINARY_TYPE,
};
#else
static const io_block_spec_t bl32_block_spec = {
	.offset = BL32_BASE,
	.length = STM32MP_BL32_SIZE
};
#endif

static const io_block_spec_t bl2_block_spec = {
	.offset = BL2_BASE,
	.length = STM32MP_BL2_SIZE,
};

static const struct stm32image_part_info bl33_partition_spec = {
	.name = BL33_IMAGE_NAME,
	.binary_type = BL33_BINARY_TYPE,
};

enum {
	IMG_IDX_BL33,
#ifdef AARCH32_SP_OPTEE
	IMG_IDX_OPTEE_HEADER,
	IMG_IDX_OPTEE_PAGER,
	IMG_IDX_OPTEE_PAGED,
#endif
	IMG_IDX_NUM
};

static struct stm32image_device_info stm32image_dev_info_spec __unused = {
	.lba_size = MMC_BLOCK_SIZE,
	.part_info[IMG_IDX_BL33] = {
		.name = BL33_IMAGE_NAME,
		.binary_type = BL33_BINARY_TYPE,
	},
#ifdef AARCH32_SP_OPTEE
	.part_info[IMG_IDX_OPTEE_HEADER] = {
		.name = OPTEE_HEADER_IMAGE_NAME,
		.binary_type = OPTEE_HEADER_BINARY_TYPE,
	},
	.part_info[IMG_IDX_OPTEE_PAGER] = {
		.name = OPTEE_PAGER_IMAGE_NAME,
		.binary_type = OPTEE_PAGER_BINARY_TYPE,
	},
	.part_info[IMG_IDX_OPTEE_PAGED] = {
		.name = OPTEE_PAGED_IMAGE_NAME,
		.binary_type = OPTEE_PAGED_BINARY_TYPE,
	},
#endif
};

static io_block_spec_t stm32image_block_spec = {
	.offset = 0,
	.length = 0,
};

static const io_dev_connector_t *stm32image_dev_con __unused;

static int open_dummy(const uintptr_t spec);
static int open_image(const uintptr_t spec);
static int open_storage(const uintptr_t spec);

struct plat_io_policy {
	uintptr_t *dev_handle;
	uintptr_t image_spec;
	int (*check)(const uintptr_t spec);
};

static const struct plat_io_policy policies[] = {
	[BL2_IMAGE_ID] = {
		.dev_handle = &dummy_dev_handle,
		.image_spec = (uintptr_t)&bl2_block_spec,
		.check = open_dummy
	},
#ifdef AARCH32_SP_OPTEE
	[BL32_IMAGE_ID] = {
		.dev_handle = &image_dev_handle,
		.image_spec = (uintptr_t)&optee_header_partition_spec,
		.check = open_image
	},
	[BL32_EXTRA1_IMAGE_ID] = {
		.dev_handle = &image_dev_handle,
		.image_spec = (uintptr_t)&optee_pager_partition_spec,
		.check = open_image
	},
	[BL32_EXTRA2_IMAGE_ID] = {
		.dev_handle = &image_dev_handle,
		.image_spec = (uintptr_t)&optee_paged_partition_spec,
		.check = open_image
	},
#else
	[BL32_IMAGE_ID] = {
		.dev_handle = &dummy_dev_handle,
		.image_spec = (uintptr_t)&bl32_block_spec,
		.check = open_dummy
	},
#endif
	[BL33_IMAGE_ID] = {
		.dev_handle = &image_dev_handle,
		.image_spec = (uintptr_t)&bl33_partition_spec,
		.check = open_image
	},
#if STM32MP_SDMMC || STM32MP_EMMC
	[GPT_IMAGE_ID] = {
		.dev_handle = &storage_dev_handle,
		.image_spec = (uintptr_t)&gpt_block_spec,
		.check = open_storage
	},
#endif
	[STM32_IMAGE_ID] = {
		.dev_handle = &storage_dev_handle,
		.image_spec = (uintptr_t)&stm32image_block_spec,
		.check = open_storage
	}
};

static int open_dummy(const uintptr_t spec)
{
	return io_dev_init(dummy_dev_handle, 0);
}

static int open_image(const uintptr_t spec)
{
	return io_dev_init(image_dev_handle, 0);
}

static int open_storage(const uintptr_t spec)
{
	return io_dev_init(storage_dev_handle, 0);
}

static void print_boot_device(boot_api_context_t *boot_context)
{
	switch (boot_context->boot_interface_selected) {
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_FLASH_SD:
		INFO("Using SDMMC\n");
		break;
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_FLASH_EMMC:
		INFO("Using EMMC\n");
		break;
#if STM32MP_UART_PROGRAMMER
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_SERIAL_UART:
		INFO("Using UART\n");
		break;
#endif
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_FLASH_NOR_QSPI:
		INFO("Using NOR\n");
		if (boot_context->nor_isdual != 0U) {
			INFO("NOR is configured in Dual Flash Mode\n");
		}
		break;
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_FLASH_NAND_FMC:
		INFO("Using NAND\n");
		break;
#ifdef STM32MP_USB
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_SERIAL_USB:
		INFO("Using USB\n");
		break;
#endif
#ifdef STM32MP1_QSPI_NAND
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_FLASH_NAND_QSPI:
		INFO("Using QSPI NAND\n");
		break;
#endif
	default:
		ERROR("Boot interface not found\n");
		panic();
		break;
	}

	if (boot_context->boot_interface_instance != 0U) {
		INFO("  Instance %d\n", boot_context->boot_interface_instance);
	}
}

#if STM32MP_SDMMC
static void print_bootrom_sd_status(boot_api_context_t *boot_context)
{
	if (boot_context->sd_err_internal_timeout_cnt != 0U) {
		WARN("BootROM: %d timeout issues\n",
		     boot_context->sd_err_internal_timeout_cnt);
	}

	if (boot_context->sd_err_dcrc_fail_cnt != 0U) {
		WARN("BootROM: %d DCRCFAIL error\n",
		     boot_context->sd_err_dcrc_fail_cnt);
	}

	if (boot_context->sd_err_dtimeout_cnt != 0U) {
		WARN("BootROM: %d DTIMEOUT error\n",
		     boot_context->sd_err_dtimeout_cnt);
	}

	if (boot_context->sd_err_ctimeout_cnt != 0U) {
		WARN("BootROM: %d CTIMEOUT error\n",
		     boot_context->sd_err_ctimeout_cnt);
	}

	if (boot_context->sd_err_ccrc_fail_cnt != 0U) {
		WARN("BootROM: %d CCRCFAIL error count\n",
		     boot_context->sd_err_ccrc_fail_cnt);
	}

	if (boot_context->sd_overall_retry_cnt != 0U) {
		WARN("BootROM: %d command retries\n",
		     boot_context->sd_overall_retry_cnt);
	}
}
#endif

#if STM32MP_EMMC
static void print_bootrom_emmc_status(boot_api_context_t *boot_context)
{
	INFO("BootROM: %d (0x%x) bytes copied from eMMC\n",
	     boot_context->emmc_nbbytes_rxcopied_tosysram_download_area,
	     boot_context->emmc_nbbytes_rxcopied_tosysram_download_area);

	if (boot_context->emmc_error_status !=
	    BOOT_API_CTX_EMMC_ERROR_STATUS_NONE) {
		WARN("BootROM eMMC error:\n");
		switch (boot_context->emmc_error_status) {
		case BOOT_API_CTX_EMMC_ERROR_STATUS_CMD_TIMEOUT:
			WARN("  CMD timeout\n");
			break;
		case BOOT_API_CTX_EMMC_ERROR_STATUS_ACK_TIMEOUT:
			WARN("  ACK timeout\n");
			break;
		case BOOT_API_CTX_EMMC_ERROR_STATUS_DATA_CRC_FAIL:
			WARN("  DATA CRC failed\n");
			break;
		case BOOT_API_CTX_EMMC_ERROR_STATUS_NOT_ENOUGH_BOOT_DATA_RX:
			WARN("  Not enough data copied\n");
			break;
		case BOOT_API_CTX_EMMC_ERROR_STATUS_HEADER_NOT_FOUND:
			WARN("  Header not found\n");
			break;
		case BOOT_API_CTX_EMMC_ERROR_STATUS_HEADER_SIZE_ZERO:
			WARN("  Header size is zero\n");
			break;
		case BOOT_API_CTX_EMMC_ERROR_STATUS_IMAGE_NOT_COMPLETE:
			WARN("  Image not complete\n");
			break;
		default:
			WARN("  Error not listed\n");
			break;
		}
	}

	switch (boot_context->emmc_xfer_status) {
	case BOOT_API_CTX_EMMC_XFER_STATUS_NOT_STARTED:
		WARN("BootROM: eMMC transfer status:\n");
		WARN("  not started\n");
		break;
	case BOOT_API_CTX_EMMC_XFER_STATUS_DATAEND_DETECTED:
		break;
	case BOOT_API_CTX_EMMC_XFER_STATUS_XFER_OVERALL_TIMEOUT_DETECTED:
		WARN("BootROM: eMMC transfer status:\n");
		WARN("  timeout detected\n");
		break;
	case BOOT_API_CTX_EMMC_XFER_STATUS_XFER_DATA_TIMEOUT:
		WARN("BootROM: eMMC transfer status:\n");
		WARN("  data timeout detected\n");
		break;
	default:
		WARN("BootROM: eMMC transfer status:\n");
		WARN("  status not listed\n");
		break;
	}
}
#endif

#if STM32MP_EMMC || STM32MP_SDMMC
static void boot_mmc(enum mmc_device_type mmc_dev_type,
		     uint16_t boot_interface_instance)
{
	int io_result __unused;
	uint8_t idx;
	struct stm32image_part_info *part;
	struct stm32_sdmmc2_params params;
	struct mmc_device_info device_info;
	const partition_entry_t *entry;

	zeromem(&device_info, sizeof(struct mmc_device_info));
	zeromem(&params, sizeof(struct stm32_sdmmc2_params));

	device_info.mmc_dev_type = mmc_dev_type;

	switch (boot_interface_instance) {
	case 1:
		params.reg_base = STM32MP_SDMMC1_BASE;
		break;
	case 2:
		params.reg_base = STM32MP_SDMMC2_BASE;
		break;
	case 3:
		params.reg_base = STM32MP_SDMMC3_BASE;
		break;
	default:
		WARN("SDMMC instance not found, using default\n");
		if (mmc_dev_type == MMC_IS_SD) {
			params.reg_base = STM32MP_SDMMC1_BASE;
		} else {
			params.reg_base = STM32MP_SDMMC2_BASE;
		}
		break;
	}

	params.device_info = &device_info;
	if (stm32_sdmmc2_mmc_init(&params) != 0) {
		ERROR("SDMMC%u init failed\n", boot_interface_instance);
		panic();
	}

	/* Open MMC as a block device to read GPT table */
	io_result = register_io_dev_block(&mmc_dev_con);
	if (io_result != 0) {
		panic();
	}

	io_result = io_dev_open(mmc_dev_con, (uintptr_t)&mmc_block_dev_spec,
				&storage_dev_handle);
	assert(io_result == 0);

	partition_init(GPT_IMAGE_ID);

	io_result = io_dev_close(storage_dev_handle);
	assert(io_result == 0);

	stm32image_dev_info_spec.device_size =
		stm32_sdmmc2_mmc_get_device_size();

	for (idx = 0U; idx < IMG_IDX_NUM; idx++) {
		part = &stm32image_dev_info_spec.part_info[idx];
		entry = get_partition_entry(part->name);
		if (entry == NULL) {
			ERROR("Partition %s not found\n", part->name);
			panic();
		}

		part->part_offset = entry->start;
		part->bkp_offset = 0U;
	}

	/*
	 * Re-open MMC with io_mmc, for better perfs compared to
	 * io_block.
	 */
	io_result = register_io_dev_mmc(&mmc_dev_con);
	assert(io_result == 0);

	io_result = io_dev_open(mmc_dev_con, 0, &storage_dev_handle);
	assert(io_result == 0);

	io_result = register_io_dev_stm32image(&stm32image_dev_con);
	assert(io_result == 0);

	io_result = io_dev_open(stm32image_dev_con,
				(uintptr_t)&stm32image_dev_info_spec,
				&image_dev_handle);
	assert(io_result == 0);
}
#endif

#if STM32MP1_QSPI_NOR
static void boot_qspi_nor(boot_api_context_t *boot_context)
{
	int io_result __unused;
	uint8_t idx;
	struct stm32image_part_info *part;

	io_result = register_io_dev_qspi(&qspi_dev_con);
	assert(io_result == 0);

	if (boot_context->boot_interface_instance != 1U) {
		WARN("NOR instance not found, using default\n");
	}

	qspi_dev_spec.instance = (QUADSPI_TypeDef *)STM32MP1_QSPI1_BASE;
	qspi_dev_spec.is_dual = boot_context->nor_isdual;

	/* Open connections to devices */
	io_result = io_dev_open(qspi_dev_con,
				(uintptr_t)&qspi_dev_spec,
				&storage_dev_handle);
	assert(io_result == 0);

	stm32image_dev_info_spec.device_size = QSPI_NOR_MAX_SIZE;
	stm32image_dev_info_spec.lba_size = QSPI_NOR_LBA_SIZE;

	idx = IMG_IDX_BL33;
	part = &stm32image_dev_info_spec.part_info[idx];
	part->part_offset = STM32MP_NOR_BL33_OFFSET;
	part->bkp_offset = QSPI_NOR_BLK_SIZE;

#ifdef AARCH32_SP_OPTEE
	idx = IMG_IDX_OPTEE_HEADER;
	part = &stm32image_dev_info_spec.part_info[idx];
	part->part_offset = STM32MP_NOR_TEEH_OFFSET;
	part->bkp_offset = QSPI_NOR_BLK_SIZE;

	idx = IMG_IDX_OPTEE_PAGED;
	part = &stm32image_dev_info_spec.part_info[idx];
	part->part_offset = STM32MP_NOR_TEED_OFFSET;
	part->bkp_offset = QSPI_NOR_BLK_SIZE;

	idx = IMG_IDX_OPTEE_PAGER;
	part = &stm32image_dev_info_spec.part_info[idx];
	part->part_offset = STM32MP_NOR_TEEX_OFFSET;
	part->bkp_offset = QSPI_NOR_BLK_SIZE;
#endif

	io_result = register_io_dev_stm32image(&stm32image_dev_con);
	assert(io_result == 0);

	io_result = io_dev_open(stm32image_dev_con,
				(uintptr_t)&stm32image_dev_info_spec,
				&image_dev_handle);
	assert(io_result == 0);
}
#endif

#if STM32MP_FMC_NAND
static void boot_fmc_nand(boot_api_context_t *boot_context)
{
	int io_result __unused;
	uint8_t idx;
	struct stm32image_part_info *part;

	/* Register the IO devices on this platform */
	io_result = register_io_dev_nand(&nand_dev_con);
	assert(io_result == 0);

	nand_dev_spec.Instance = (FMC_TypeDef *)STM32MP_FMC_BASE;
	nand_dev_spec.Info.PageSize = boot_context->nand_page_size;
	nand_dev_spec.Info.BlockSize = boot_context->nand_block_size;
	nand_dev_spec.Info.BlockNb = boot_context->nand_block_nb;
	nand_dev_spec.Info.BusWidth = boot_context->nand_data_width;
	nand_dev_spec.Info.ECCcorrectability =
		boot_context->nand_ecc_bits;

	/* Open connections to devices */
	io_result = io_dev_open(nand_dev_con, (uintptr_t)&nand_dev_spec,
				&storage_dev_handle);
	assert(io_result == 0);

	stm32image_dev_info_spec.device_size =
		nand_dev_spec.Info.PageSize *
		nand_dev_spec.Info.BlockSize *
		nand_dev_spec.Info.BlockNb;
	stm32image_dev_info_spec.lba_size = BCH_PAGE_SECTOR;

	idx = IMG_IDX_BL33;
	part = &stm32image_dev_info_spec.part_info[idx];
	part->part_offset = STM32MP_NAND_BL33_OFFSET;
	part->bkp_offset = nand_dev_spec.Info.PageSize *
		nand_dev_spec.Info.BlockSize;

#ifdef AARCH32_SP_OPTEE
	idx = IMG_IDX_OPTEE_HEADER;
	part = &stm32image_dev_info_spec.part_info[idx];
	part->part_offset = STM32MP_NAND_TEEH_OFFSET;
	part->bkp_offset = nand_dev_spec.Info.PageSize *
		nand_dev_spec.Info.BlockSize;

	idx = IMG_IDX_OPTEE_PAGED;
	part = &stm32image_dev_info_spec.part_info[idx];
	part->part_offset = STM32MP_NAND_TEED_OFFSET;
	part->bkp_offset = nand_dev_spec.Info.PageSize *
		nand_dev_spec.Info.BlockSize;

	idx = IMG_IDX_OPTEE_PAGER;
	part = &stm32image_dev_info_spec.part_info[idx];
	part->part_offset = STM32MP_NAND_TEEX_OFFSET;
	part->bkp_offset = nand_dev_spec.Info.PageSize *
		nand_dev_spec.Info.BlockSize;
#endif

	io_result = register_io_dev_stm32image(&stm32image_dev_con);
	assert(io_result == 0);

	io_result = io_dev_open(stm32image_dev_con,
				(uintptr_t)&stm32image_dev_info_spec,
				&image_dev_handle);
	assert(io_result == 0);
}
#endif

#if STM32MP_UART_PROGRAMMER
static void flash_uart(uint16_t boot_interface_instance)
{
	int io_result __unused;
	uintptr_t uart_addr;

	/* Register the IO devices on this platform */
	io_result = register_io_dev_uart(&uart_dev_con);
	assert(io_result == 0);

	uart_programmer.Init.WordLength = UART_WORDLENGTH_9B;
	uart_programmer.Init.Parity = UART_PARITY_EVEN;
	uart_addr = get_uart_address(boot_interface_instance);

	if (uart_addr != 0U) {
		uart_programmer.Instance = (USART_TypeDef *)uart_addr;
	} else {
		WARN("UART instance not found, using default\n");
		uart_programmer.Instance = (USART_TypeDef *)USART2_BASE;
	}

	/* Open connections to devices */
	io_result = io_dev_open(uart_dev_con, (uintptr_t)&uart_programmer,
				&image_dev_handle);
	assert(!io_result);
}
#endif

#ifdef STM32MP_USB
static void flash_usb(struct usb_ctx *usb_context)
{
	int io_result __unused;

	pcd_handle.in_ep[0].maxpacket = 0x40;
	pcd_handle.out_ep[0].maxpacket = 0x40;

	pcd_handle.state = HAL_PCD_STATE_READY;

	usb_core_handle.data = &pcd_handle;

	usb_dwc2_init_driver(&usb_core_handle,
			     (uint32_t *)USB_OTG_BASE);

	usb_dfu_register_callback(&usb_core_handle);

	stm32mp_usb_init_desc(&usb_core_handle);

	usb_core_handle.ep_in[0].maxpacket = 0x40;
	usb_core_handle.ep_out[0].maxpacket = 0x40;

	usb_core_handle.ep0_state =
		usb_context->pusbd_device_ctx->ep0_state;
	usb_core_handle.dev_state = USBD_STATE_CONFIGURED;

	usb_core_handle.class_data = &usb_dfu_handle;
	usb_dfu_handle.dev_state = DFU_STATE_IDLE;

	/* Register the IO devices on this platform */
	io_result = register_io_dev_usb(&usb_dev_con);
	assert(io_result == 0);

	/* Open connections to devices */
	io_result = io_dev_open(usb_dev_con,
				(uintptr_t)&usb_core_handle,
				&image_dev_handle);

	assert(io_result == 0);
}
#endif

void stm32mp_io_setup(void)
{
	int io_result __unused;
	boot_api_context_t *boot_context =
		(boot_api_context_t *)stm32mp_get_boot_ctx_address();

	print_boot_device(boot_context);

	if ((boot_context->boot_partition_used_toboot == 1U) ||
	    (boot_context->boot_partition_used_toboot == 2U)) {
		INFO("Boot used partition fsbl%d\n",
		     boot_context->boot_partition_used_toboot);
	}

	io_result = register_io_dev_dummy(&dummy_dev_con);
	assert(io_result == 0);

	io_result = io_dev_open(dummy_dev_con, dummy_dev_spec,
				&dummy_dev_handle);
	assert(io_result == 0);

	switch (boot_context->boot_interface_selected) {
#if STM32MP_SDMMC
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_FLASH_SD:
		print_bootrom_sd_status(boot_context);
		boot_mmc(MMC_IS_SD, boot_context->boot_interface_instance);
		break;
#endif
#if STM32MP_EMMC
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_FLASH_EMMC:
		print_bootrom_emmc_status(boot_context);
		boot_mmc(MMC_IS_EMMC, boot_context->boot_interface_instance);
		break;
#endif
#if STM32MP1_QSPI_NOR
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_FLASH_NOR_QSPI:
		boot_qspi_nor(boot_context);
		break;
#endif
#if STM32MP_FMC_NAND
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_FLASH_NAND_FMC:
		boot_fmc_nand(boot_context);
		break;
#endif
#if STM32MP_UART_PROGRAMMER
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_SERIAL_UART:
		flash_uart(boot_context->boot_interface_instance);
		break;
#endif
#ifdef STM32MP_USB
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_SERIAL_USB:
		flash_usb((struct usb_ctx *)boot_context->usb_context);
		break;
#endif
#ifdef STM32MP1_QSPI_NAND
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_FLASH_NAND_QSPI:
		ERROR("QSPI NAND not supported\n");
		break;
#endif

	default:
		ERROR("Boot interface %d not supported\n",
		      boot_context->boot_interface_selected);
		break;
	}
}

/*
 * Return an IO device handle and specification which can be used to access
 * an image. Use this to enforce platform load policy.
 */
int plat_get_image_source(unsigned int image_id, uintptr_t *dev_handle,
			  uintptr_t *image_spec)
{
	int rc;
	const struct plat_io_policy *policy;

	assert(image_id < ARRAY_SIZE(policies));

	policy = &policies[image_id];
	rc = policy->check(policy->image_spec);
	if (rc == 0) {
		*image_spec = policy->image_spec;
		*dev_handle = *(policy->dev_handle);
	}

	return rc;
}
