/*
 * Copyright (c) 2015-2019, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <arch_helpers.h>
#include <assert.h>
#include <bl_common.h>
#include <bsec.h>
#include <context.h>
#include <context_mgmt.h>
#include <debug.h>
#include <dt-bindings/clock/stm32mp1-clks.h>
#include <etzpc.h>
#include <generic_delay_timer.h>
#include <gicv2.h>
#include <mmio.h>
#include <platform.h>
#include <platform_def.h>
#include <platform_sp_min.h>
#include <stm32_console.h>
#include <stm32_gpio.h>
#include <stm32_iwdg.h>
#include <stm32_rng.h>
#include <stm32_rtc.h>
#include <stm32_tamp.h>
#include <stm32_timer.h>
#include <stm32mp_dt.h>
#include <stm32mp_pmic.h>
#include <stm32mp1_clk.h>
#include <stm32mp1_context.h>
#include <stm32mp1_power_config.h>
#include <stm32mp1_private.h>
#include <stm32mp1_shared_resources.h>
#include <stpmic1.h>
#include <string.h>
#include <tzc400.h>
#include <xlat_tables_v2.h>

/******************************************************************************
 * Placeholder variables for copying the arguments that have been passed to
 * BL32 from BL2.
 ******************************************************************************/
static entry_point_info_t bl33_image_ep_info;

static struct console_stm32 console;

static struct stm32_tamp_int int_tamp[PLAT_MAX_TAMP_INT] = {
	TAMP_UNUSED,
	TAMP_UNUSED,
	TAMP_UNUSED,
	TAMP_UNUSED,
	TAMP_UNUSED,
};

static struct stm32_tamp_ext ext_tamp[PLAT_MAX_TAMP_EXT] = {
	TAMP_UNUSED,
	TAMP_UNUSED,
	TAMP_UNUSED,
};

static void tzc_it_handler(void)
{
	ERROR("No IT handler in ARM tzc400 driver\n");
}

static void stm32_sgi1_it_handler(void)
{
	uint32_t id;

	stm32mp_mask_timer();

	gicv2_end_of_interrupt(ARM_IRQ_SEC_SGI_1);

	do {
		id = plat_ic_get_pending_interrupt_id();

		if (id <= MAX_SPI_ID) {
			gicv2_end_of_interrupt(id);

			plat_ic_disable_interrupt(id);
		}
	} while (id <= MAX_SPI_ID);

	stm32mp_wait_cpu_reset();
}

static void configure_wakeup_interrupt(void)
{
	int irq_num = fdt_rcc_enable_it("wakeup");

	if (irq_num < 0) {
		ERROR("irq_num = %d\n", irq_num);
		panic();
	}

	plat_ic_set_interrupt_priority(irq_num, STM32MP1_IRQ_RCC_SEC_PRIO);
}

static void initialize_pll1_settings(void)
{
	uint32_t vddcore_voltage = 0U;
	int ret;

	if (stm32_are_pll1_settings_valid_in_context()) {
		return;
	}

	if (dt_pmic_status() > 0) {
		ret = stpmic1_regulator_voltage_get("buck1");
		if (ret < 0) {
			panic();
		}

		vddcore_voltage = (uint32_t)ret;
	}

	if (stm32mp1_clk_compute_all_pll1_settings(vddcore_voltage) != 0) {
		panic();
	}
}

/*******************************************************************************
 * Interrupt handler for FIQ (secure IRQ)
 ******************************************************************************/
void sp_min_plat_fiq_handler(uint32_t id)
{
	uint32_t value = 0;

	switch (id & INT_ID_MASK) {
	case ARM_IRQ_SEC_PHY_TIMER:
	case STM32MP1_IRQ_MCU_SEV:
	case STM32MP1_IRQ_RCC_WAKEUP:
		stm32mp1_calib_it_handler(id);
		break;
	case STM32MP1_IRQ_TZC400:
		tzc400_init(STM32MP1_TZC_BASE);
		tzc400_it_handler();
		panic();
		break;
	case STM32MP1_IRQ_TAMPSERRS:
		stm32_tamp_it_handler();
		break;
	case ARM_IRQ_SEC_SGI_1:
		stm32_sgi1_it_handler();
		break;
	case STM32MP1_IRQ_IWDG1:
	case STM32MP1_IRQ_IWDG2:
		stm32_iwdg_it_handler(id);
		break;
	case STM32MP1_IRQ_AXIERRIRQ:
		ERROR("STM32MP1_IRQ_AXIERRIRQ generated\n");
		tzc400_init(STM32MP1_TZC_BASE);
		__asm__("mrc	p15, 1, %0, c9, c0, 3" : "=r" (value));
		if (value) {
			/* we have a pending IT clear it */
			value = 0;
			__asm__("mcr	p15, 1, %0, c9, c0, 3" :: "r" (value));
		} else {
			ERROR("IRQ_AXIERRIRQ handle call w/o any flag set!!\n");
		}

		/* Check if FIQ has been generated due to TZC400 abort*/
		if (tzc400_is_pending_interrupt()) {
			tzc_it_handler();
		} else {
			ERROR("IRQ_AXIERRIRQ cause can't be detected");
		}

		panic();
		break;
	default:
		ERROR("SECURE IT handler not define for it : %u", id);
		break;
	}
}

/*******************************************************************************
 * Return a pointer to the 'entry_point_info' structure of the next image for
 * the security state specified. BL33 corresponds to the non-secure image type
 * while BL32 corresponds to the secure image type. A NULL pointer is returned
 * if the image does not exist.
 ******************************************************************************/
entry_point_info_t *sp_min_plat_get_bl33_ep_info(void)
{
	entry_point_info_t *next_image_info;
	uint32_t bkpr_core1_addr =
		tamp_bkpr(BOOT_API_CORE1_BRANCH_ADDRESS_TAMP_BCK_REG_IDX);
	uint32_t bkpr_core1_magic =
		tamp_bkpr(BOOT_API_CORE1_MAGIC_NUMBER_TAMP_BCK_REG_IDX);

	next_image_info = &bl33_image_ep_info;

	/*
	 * PC is set to 0 when resetting after STANDBY
	 * The context should be restored, and the image information
	 * should be filled with what what was saved
	 */
	if (next_image_info->pc == 0U) {
		void *cpu_context;
		uint32_t magic_nb, saved_pc;

		stm32mp_clk_enable(RTCAPB);

		magic_nb = mmio_read_32(bkpr_core1_magic);
		saved_pc = mmio_read_32(bkpr_core1_addr);

		stm32mp_clk_disable(RTCAPB);

		if (stm32_restore_context() != 0) {
			panic();
		}

		cpu_context = cm_get_context(NON_SECURE);

		next_image_info->spsr = read_ctx_reg(get_regs_ctx(cpu_context),
						     CTX_SPSR);

		/* PC should be retrieved in backup register if OK, else it can
		 * be retrieved from non-secure context
		 */
		if (magic_nb == BOOT_API_A7_CORE0_MAGIC_NUMBER) {
			/* BL33 return address should be in DDR */
			if ((saved_pc < STM32MP_DDR_BASE) ||
			    (saved_pc > (STM32MP_DDR_BASE +
					 (dt_get_ddr_size() - 1U)))) {
				panic();
			}

			next_image_info->pc = saved_pc;
		} else {
			next_image_info->pc =
				read_ctx_reg(get_regs_ctx(cpu_context), CTX_LR);
		}
	}

	return next_image_info;
}

/*******************************************************************************
 * Perform any BL32 specific platform actions.
 ******************************************************************************/
void sp_min_early_platform_setup2(u_register_t arg0, u_register_t arg1,
				  u_register_t arg2, u_register_t arg3)
{
#if STM32MP_UART_PROGRAMMER
	uint32_t boot_itf, boot_instance;
#endif
	struct dt_node_info dt_dev_info;
	int result;
	bl_params_t *params_from_bl2 = (bl_params_t *)arg0;

	/* Imprecise aborts can be masked in NonSecure */
	write_scr(read_scr() | SCR_AW_BIT);

	assert(params_from_bl2 != NULL);
	assert(params_from_bl2->h.type == PARAM_BL_PARAMS);
	assert(params_from_bl2->h.version >= VERSION_2);

	bl_params_node_t *bl_params = params_from_bl2->head;

	mmap_add_region(BL_CODE_BASE, BL_CODE_BASE,
			BL_CODE_END - BL_CODE_BASE,
			MT_CODE | MT_SECURE);

#if SEPARATE_CODE_AND_RODATA
	mmap_add_region(BL_RO_DATA_BASE, BL_RO_DATA_BASE,
			BL_RO_DATA_LIMIT - BL_RO_DATA_BASE,
			MT_RO_DATA | MT_SECURE);
#endif

	mmap_add_region(STM32MP_DDR_BASE, STM32MP_DDR_BASE,
			dt_get_ddr_size() - STM32MP_DDR_S_SIZE,
			MT_MEMORY | MT_RW | MT_NS);

	configure_mmu();

	/*
	 * Copy BL33 entry point information.
	 * They are stored in Secure RAM, in BL2's address space.
	 */
	while (bl_params != NULL) {
		if (bl_params->image_id == BL33_IMAGE_ID) {
			bl33_image_ep_info = *bl_params->ep_info;
			break;
		}

		bl_params = bl_params->next_params_info;
	}

	if (dt_open_and_check() < 0) {
		panic();
	}

	if (bsec_probe() != 0) {
		panic();
	}

	if (stm32mp1_clk_probe() < 0) {
		panic();
	}

	/* Initialize uart for console except if it is used by programmer */
	result = dt_get_stdout_uart_info(&dt_dev_info);
#if STM32MP_UART_PROGRAMMER
	stm32_get_boot_interface(&boot_itf, &boot_instance);

	if ((result > 0) && dt_dev_info.status &&
	    !((boot_itf == BOOT_API_CTX_BOOT_INTERFACE_SEL_SERIAL_UART) &&
	      (get_uart_address(boot_instance) == dt_dev_info.base))) {
#else
	if ((result > 0) && dt_dev_info.status) {
#endif
		if (console_stm32_register(dt_dev_info.base, 0,
					   STM32MP_UART_BAUDRATE, &console) ==
		    0) {
			panic();
		}
	}

	if (dt_pmic_status() > 0) {
		initialize_pmic();
	}

	initialize_pll1_settings();

	stm32mp1_init_lp_states();
}

/*******************************************************************************
 * Set security setup in sp_min
 ******************************************************************************/
void stm32mp1_sp_min_security_setup(void)
{
	uint32_t filter_conf = 0;
	uint32_t active_conf = 0;
	int ret;

	if (etzpc_init() != 0) {
		ERROR("ETZPC configuration issue\n");
		panic();
	}

	/* Init rtc driver */
	ret = stm32_rtc_init();
	if (ret < 0) {
		WARN("RTC driver init error %i\n", ret);
	}

	/*  Init rng driver */
	ret = stm32_rng_init();
	if (ret < 0) {
		WARN("RNG driver init error %i\n", ret);
	}

	/* Init tamper */
	if (stm32_tamp_init() > 0) {
		stm32_tamp_configure_internal(int_tamp, PLAT_MAX_TAMP_INT);
		stm32_tamp_configure_external(ext_tamp, PLAT_MAX_TAMP_EXT,
					      filter_conf, active_conf);

		/* Enable timestamp for tamper */
		stm32_rtc_set_tamper_timestamp();
	}

	if (stm32_timer_init() == 0) {
		stm32mp1_calib_init();
	}
}

/*******************************************************************************
 * Initialize the MMU, security and the GIC.
 ******************************************************************************/
void sp_min_platform_setup(void)
{
	/* Initialize tzc400 after DDR initialization */
	stm32mp1_security_setup();

	generic_delay_timer_init();

	stm32mp1_gic_init();

	/* Update security settings */
	stm32mp1_sp_min_security_setup();

	if (stm32_iwdg_init() < 0) {
		panic();
	}

	configure_wakeup_interrupt();

	stm32mp1_driver_init_late();
}

void sp_min_plat_arch_setup(void)
{
}

void sp_min_plat_runtime_setup(void)
{
}
