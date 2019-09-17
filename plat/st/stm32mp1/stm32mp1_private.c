/*
 * Copyright (c) 2015-2019, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <arch_helpers.h>
#include <assert.h>
#include <bl_common.h>
#include <debug.h>
#include <gicv2.h>
#include <mmio.h>
#include <platform.h>
#include <platform_def.h>
#include <stm32_iwdg.h>
#include <stm32mp_common.h>
#include <stm32mp_dt.h>
#include <stm32mp_reset.h>
#include <xlat_tables_v2.h>

/* Internal layout of the 32bit OTP word board_id */
#define BOARD_ID_BOARD_NB_MASK		GENMASK(31, 16)
#define BOARD_ID_BOARD_NB_SHIFT		16
#define BOARD_ID_VARIANT_MASK		GENMASK(15, 12)
#define BOARD_ID_VARIANT_SHIFT		12
#define BOARD_ID_REVISION_MASK		GENMASK(11, 8)
#define BOARD_ID_REVISION_SHIFT		8
#define BOARD_ID_BOM_MASK		GENMASK(3, 0)

#define BOARD_ID2NB(_id)		(((_id) & BOARD_ID_BOARD_NB_MASK) >> \
					 BOARD_ID_BOARD_NB_SHIFT)
#define BOARD_ID2VAR(_id)		(((_id) & BOARD_ID_VARIANT_MASK) >> \
					 BOARD_ID_VARIANT_SHIFT)
#define BOARD_ID2REV(_id)		(((_id) & BOARD_ID_REVISION_MASK) >> \
					 BOARD_ID_REVISION_SHIFT)
#define BOARD_ID2BOM(_id)		((_id) & BOARD_ID_BOM_MASK)

#define MAP_ROM		MAP_REGION_FLAT(STM32MP_ROM_BASE, \
					STM32MP_ROM_SIZE, \
					MT_MEMORY | \
					MT_RO | \
					MT_SECURE | \
					MT_EXECUTE)

#define MAP_SRAM	MAP_REGION_FLAT(STM32MP_SYSRAM_BASE, \
					STM32MP_SYSRAM_SIZE, \
					MT_MEMORY | \
					MT_RW | \
					MT_SECURE | \
					MT_EXECUTE_NEVER)

#define MAP_SRAM_MCU	MAP_REGION_FLAT(STM32MP_SRAM_MCU_BASE, \
					STM32MP_SRAM_MCU_SIZE, \
					MT_MEMORY | \
					MT_RW | \
					MT_NS | \
					MT_EXECUTE_NEVER)

#define MAP_RETRAM	MAP_REGION_FLAT(STM32MP_RETRAM_BASE, \
					STM32MP_RETRAM_SIZE, \
					MT_MEMORY | \
					MT_RW | \
					MT_NS | \
					MT_EXECUTE_NEVER)

#define MAP_DEVICE1	MAP_REGION_FLAT(STM32MP1_DEVICE1_BASE, \
					STM32MP1_DEVICE1_SIZE, \
					MT_DEVICE | \
					MT_RW | \
					MT_SECURE | \
					MT_EXECUTE_NEVER)

#define MAP_DEVICE2	MAP_REGION_FLAT(STM32MP1_DEVICE2_BASE, \
					STM32MP1_DEVICE2_SIZE, \
					MT_DEVICE | \
					MT_RW | \
					MT_SECURE | \
					MT_EXECUTE_NEVER)

#if defined(IMAGE_BL2)
static const mmap_region_t stm32mp1_mmap[] = {
	MAP_ROM,
	MAP_SRAM,
#if defined(STM32MP_USB)
	MAP_SRAM_MCU,
#endif
	MAP_DEVICE1,
	MAP_DEVICE2,
	{0}
};
#endif
#if defined(IMAGE_BL32)
static const mmap_region_t stm32mp1_mmap[] = {
	MAP_ROM,
	MAP_SRAM,
	MAP_DEVICE1,
	MAP_DEVICE2,
	{0}
};
#endif

void configure_mmu(void)
{
#ifndef MMU_OFF
	unsigned int flags = 0;

	mmap_add(stm32mp1_mmap);
	init_xlat_tables();
#ifdef DCACHE_OFF
	flags |= DISABLE_DCACHE;
#endif
	enable_mmu_svc_mon(flags);
#endif
}

#if STM32MP_UART_PROGRAMMER
/*
 * UART Management
 */
static const uintptr_t stm32mp1_uart_addresses[8] = {
	USART1_BASE,
	USART2_BASE,
	USART3_BASE,
	UART4_BASE,
	UART5_BASE,
	USART6_BASE,
	UART7_BASE,
	UART8_BASE,
};

uintptr_t get_uart_address(uint32_t instance_nb)
{
	if (!instance_nb || instance_nb > ARRAY_SIZE(stm32mp1_uart_addresses))
		return 0;

	return stm32mp1_uart_addresses[instance_nb - 1];
}
#endif

#define ARM_CNTXCTL_IMASK	BIT(1)

void stm32mp_mask_timer(void)
{
	/* Mask timer interrupts */
	write_cntp_ctl(read_cntp_ctl() | ARM_CNTXCTL_IMASK);
	write_cntv_ctl(read_cntv_ctl() | ARM_CNTXCTL_IMASK);
}

void __dead2 stm32mp_wait_cpu_reset(void)
{
	uint32_t id;

	dcsw_op_all(DC_OP_CISW);
	write_sctlr(read_sctlr() & ~SCTLR_C_BIT);
	dcsw_op_all(DC_OP_CISW);
	__asm__("clrex");

	dsb();
	isb();

	for ( ; ; ) {
		do {
			id = plat_ic_get_pending_interrupt_id();

			if (id <= MAX_SPI_ID) {
				gicv2_end_of_interrupt(id);

				plat_ic_disable_interrupt(id);
			}
		} while (id <= MAX_SPI_ID);

		wfi();
	}
}

/*
 * tzc_source_ip contains the TZC transaction source IPs that need to be reset
 * before a C-A7 subsystem is reset (i.e. independent reset):
 * - C-A7 subsystem is reset separately later in the sequence,
 * - C-M4 subsystem is not concerned here,
 * - DAP is excluded for debug purpose,
 * - IPs are stored with their ETZPC IDs (STM32MP1_ETZPC_MAX_ID if not
 *   applicable) because some of them need to be reset only if they are not
 *   configured in MCU isolation mode inside ETZPC device tree.
 */
struct tzc_source_ip {
	uint32_t reset_id;
	uint32_t clock_id;
	uint32_t decprot_id;
};

#define _TZC_FIXED(res, clk)			\
	{						\
		.reset_id = (res),			\
		.clock_id = (clk),			\
		.decprot_id = STM32MP1_ETZPC_MAX_ID,	\
	}

#define _TZC_COND(res, clk, decprot)			\
	{						\
		.reset_id = (res),			\
		.clock_id = (clk),			\
		.decprot_id = (decprot),		\
	}

static const struct tzc_source_ip tzc_source_ip[] = {
	_TZC_FIXED(LTDC_R, LTDC_PX),
	_TZC_FIXED(GPU_R, GPU),
	_TZC_FIXED(USBH_R, USBH),
	_TZC_FIXED(SDMMC1_R, SDMMC1_K),
	_TZC_FIXED(SDMMC2_R, SDMMC2_K),
	_TZC_FIXED(MDMA_R, MDMA),
	_TZC_COND(USBO_R, USBO_K, STM32MP1_ETZPC_OTG_ID),
	_TZC_COND(SDMMC3_R, SDMMC3_K, STM32MP1_ETZPC_SDMMC3_ID),
	_TZC_COND(ETHMAC_R, ETHMAC, STM32MP1_ETZPC_ETH_ID),
	_TZC_COND(DMA1_R, DMA1, STM32MP1_ETZPC_DMA1_ID),
	_TZC_COND(DMA2_R, DMA2, STM32MP1_ETZPC_DMA2_ID),
};

void __dead2 stm32mp_plat_reset(int cpu)
{
	uint32_t reg = RCC_MP_GRSTCSETR_MPUP0RST;
	uint32_t id;

	/* Mask timer interrupts */
	stm32mp_mask_timer();

	for (id = 0U; id < ARRAY_SIZE(tzc_source_ip); id++) {
		if ((!stm32mp1_clk_is_enabled(tzc_source_ip[id].clock_id)) ||
		    ((tzc_source_ip[id].decprot_id != STM32MP1_ETZPC_MAX_ID) &&
		     (etzpc_get_decprot(tzc_source_ip[id].decprot_id) ==
		      TZPC_DECPROT_MCU_ISOLATION))) {
			continue;
		}

		if (tzc_source_ip[id].reset_id != GPU_R) {
			stm32mp_reset_assert(tzc_source_ip[id].reset_id);
			stm32mp_reset_deassert(tzc_source_ip[id].reset_id);
		} else {
			/* GPU reset automatically cleared by hardware */
			mmio_setbits_32(stm32mp_rcc_base() + RCC_AHB6RSTSETR,
					RCC_AHB6RSTSETR_GPURST);
		}
	}

	if (stm32mp_is_single_core() == 0) {
		unsigned int sec_cpu = (cpu == STM32MP_PRIMARY_CPU) ?
			STM32MP_SECONDARY_CPU : STM32MP_PRIMARY_CPU;

		gicv2_raise_sgi(ARM_IRQ_SEC_SGI_1, sec_cpu);
		reg |= RCC_MP_GRSTCSETR_MPUP1RST;
	}

	do {
		id = plat_ic_get_pending_interrupt_id();

		if (id <= MAX_SPI_ID) {
			gicv2_end_of_interrupt(id);

			plat_ic_disable_interrupt(id);
		}
	} while (id <= MAX_SPI_ID);

	mmio_write_32(stm32mp_rcc_base() + RCC_MP_GRSTCSETR, reg);

	stm32mp_wait_cpu_reset();
}

static uint32_t get_part_number(void)
{
	uint32_t part_number = 0;

	if (bsec_shadow_read_otp(&part_number, PART_NUMBER_OTP) != BSEC_OK) {
		ERROR("BSEC: PART_NUMBER_OTP Error\n");
		return -1;
	}

	part_number = (part_number & PART_MASK) >> PART_SHIFT;

	return (part_number | (stm32mp1_dbgmcu_get_chip_dev_id() << 16));
}

static uint32_t get_cpu_package(void)
{
	uint32_t package = 0;

	if (bsec_shadow_read_otp(&package, PACKAGE_OTP) != BSEC_OK) {
		ERROR("BSEC: PART_NUMBER_OTP Error\n");
		return -1;
	}

	return ((package & PKG_MASK) >> PKG_SHIFT);
}

void stm32mp_print_cpuinfo(void)
{
	const char *cpu_s, *cpu_r, *pkg;

	/* MPUs Part Numbers */
	switch (get_part_number()) {
	case STM32MP157C_PART_NB:
		cpu_s = "157C";
		break;
	case STM32MP157A_PART_NB:
		cpu_s = "157A";
		break;
	case STM32MP153C_PART_NB:
		cpu_s = "153C";
		break;
	case STM32MP153A_PART_NB:
		cpu_s = "153A";
		break;
	case STM32MP151C_PART_NB:
		cpu_s = "151C";
		break;
	case STM32MP151A_PART_NB:
		cpu_s = "151A";
		break;
	default:
		cpu_s = "????";
		break;
	}

	/* Package */
	switch (get_cpu_package()) {
	case PKG_AA_LBGA448:
		pkg = "AA";
		break;
	case PKG_AB_LBGA354:
		pkg = "AB";
		break;
	case PKG_AC_TFBGA361:
		pkg = "AC";
		break;
	case PKG_AD_TFBGA257:
		pkg = "AD";
		break;
	default:
		pkg = "??";
		break;
	}

	/* REVISION */
	switch (stm32mp1_dbgmcu_get_chip_version()) {
	case STM32MP1_REV_A:
		cpu_r = "A";
		break;
	case STM32MP1_REV_B:
		cpu_r = "B";
		break;
	default:
		cpu_r = "?";
		break;
	}

	NOTICE("CPU: STM32MP%s%s Rev.%s\n", cpu_s, pkg, cpu_r);
}

void stm32mp_print_boardinfo(void)
{
	uint32_t board_id = 0;
	uint32_t board_otp;
	int bsec_node, bsec_board_id_node;
	void *fdt;
	const fdt32_t *cuint;

	if (fdt_get_address(&fdt) == 0) {
		panic();
	}

	bsec_node = fdt_node_offset_by_compatible(fdt, -1, DT_BSEC_COMPAT);
	if (bsec_node < 0) {
		return;
	}

	bsec_board_id_node = fdt_subnode_offset(fdt, bsec_node, "board_id");
	if (bsec_board_id_node <= 0) {
		return;
	}

	cuint = fdt_getprop(fdt, bsec_board_id_node, "reg", NULL);
	if (cuint == NULL) {
		ERROR("board_id node without reg property\n");
		panic();
	}

	board_otp = fdt32_to_cpu(*cuint) / sizeof(uint32_t);

	if (bsec_shadow_read_otp(&board_id, board_otp) != BSEC_OK) {
		ERROR("BSEC: PART_NUMBER_OTP Error\n");
		return;
	}

	if (board_id != 0U) {
		char rev[2];

		rev[0] = BOARD_ID2REV(board_id) - 1 + 'A';
		rev[1] = '\0';
		NOTICE("Board: MB%04x Var%d Rev.%s-%02d\n",
		       BOARD_ID2NB(board_id),
		       BOARD_ID2VAR(board_id),
		       rev,
		       BOARD_ID2BOM(board_id));
	}
}

/*
 * This function determines if one single core is presently running. This is
 * done by OTP read.
 * Returns 1 if yes, 0 if more that one core is running, -1 if error.
 */
int stm32mp_is_single_core(void)
{
	uint32_t part_number = get_part_number();

	/* STM32MP151x is a single core */
	if ((part_number == STM32MP151A_PART_NB) ||
	    (part_number == STM32MP151C_PART_NB)) {
		return 1;
	}

	return 0;
}

uint8_t stm32_iwdg_get_instance(uintptr_t base)
{
	switch (base) {
	case IWDG1_BASE:
		return IWDG1_INST;
	case IWDG2_BASE:
		return IWDG2_INST;
	default:
		panic();
	}
}

uint32_t stm32_iwdg_get_otp_config(uintptr_t base)
{
	uint8_t idx;
	uint32_t iwdg_cfg = 0;
	uint32_t otp_value;

#if defined(IMAGE_BL2)
	if (bsec_shadow_read_otp(&otp_value, HW2_OTP) != BSEC_OK) {
		panic();
	}
#elif defined(IMAGE_BL32)
	if (bsec_read_otp(&otp_value, HW2_OTP) != BSEC_OK) {
		panic();
	}
#endif

	idx = stm32_iwdg_get_instance(base);

	if ((otp_value & BIT(idx + IWDG_HW_POS)) != 0U) {
		iwdg_cfg |= IWDG_HW_ENABLED;
	}

	if ((otp_value & BIT(idx + IWDG_FZ_STOP_POS)) != 0U) {
		iwdg_cfg |= IWDG_DISABLE_ON_STOP;
	}

	if ((otp_value & BIT(idx + IWDG_FZ_STANDBY_POS)) != 0U) {
		iwdg_cfg |= IWDG_DISABLE_ON_STANDBY;
	}

	return iwdg_cfg;
}

#if defined(IMAGE_BL2)
uint32_t stm32_iwdg_shadow_update(uintptr_t base, uint32_t flags)
{
	uint32_t idx;
	uint32_t otp;
	uint32_t result;

	if (bsec_shadow_read_otp(&otp, HW2_OTP) != BSEC_OK) {
		panic();
	}

	idx = stm32_iwdg_get_instance(base);

	if ((flags & IWDG_DISABLE_ON_STOP) != 0) {
		otp |= BIT(idx + IWDG_FZ_STOP_POS);
	}

	if ((flags & IWDG_DISABLE_ON_STANDBY) != 0) {
		otp |= BIT(idx + IWDG_FZ_STANDBY_POS);
	}

	result = bsec_write_otp(otp, HW2_OTP);
	if (result != BSEC_OK) {
		return result;
	}

	/* Sticky lock OTP_IWDG (read and write) */
	if ((bsec_set_sr_lock(HW2_OTP) != BSEC_OK) ||
	    (bsec_set_sw_lock(HW2_OTP) != BSEC_OK)) {
		return BSEC_LOCK_FAIL;
	}

	return BSEC_OK;
}
#endif

/*
 * This function allows to split bindings between platform and ETZPC
 * HW mapping. If this conversion was done at driver level, the driver
 * should include all supported platform bindings. ETZPC may be used on
 * other platforms.
 */
enum etzpc_decprot_attributes stm32mp_etzpc_binding2decprot(uint32_t mode)
{
	switch (mode) {
	case DECPROT_S_RW:
		return TZPC_DECPROT_S_RW;
	case DECPROT_NS_R_S_W:
		return TZPC_DECPROT_NS_R_S_W;
	case DECPROT_MCU_ISOLATION:
		return TZPC_DECPROT_MCU_ISOLATION;
	case DECPROT_NS_RW:
		return TZPC_DECPROT_NS_RW;
	default:
		panic();
	}
}
