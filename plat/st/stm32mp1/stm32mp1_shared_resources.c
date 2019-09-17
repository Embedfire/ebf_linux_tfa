// SPDX-License-Identifier: BSD-3-Clause
/*
 * Copyright (c) 2017-2019, STMicroelectronics
 */

#include <arch.h>
#include <assert.h>
#include <debug.h>
#include <platform_def.h>
#include <stdint.h>
#include <stdio.h>
#include <stm32_gpio.h>
#include <stm32mp_dt.h>
#include <stm32mp_shres_helpers.h>

static bool registering_locked;
static int8_t gpioz_nbpin = -1;

/*
 * Generic clock enable/disable from secure world.
 * Some drivers may use non secure resources in specific execution context:
 * when the other SMP core(s) are offline and non secure is never reached.
 * In such cases, drivers shall enable/disable the HW clock only if it was not
 * left enabled by the non secure world.
 *
 * During driver initializations, before registering_locked is locked, all
 * driver simply enable/disable the clock as if the peripheral was secure.
 */
void stm32mp_clk_enable(unsigned long id)
{
	if (registering_locked) {
		if (stm32mp1_clock_is_non_secure(id)) {
			assert(stm32mp1_clk_get_refcount(id) == 0U);

			if (stm32mp1_clk_is_enabled(id)) {
				return;
			}
		}
	}

	stm32mp1_clk_enable_secure(id);
}

void stm32mp_clk_disable(unsigned long id)
{
	if (registering_locked) {
		if (stm32mp1_clock_is_non_secure(id)) {
			if (stm32mp1_clk_get_refcount(id) == 0U) {
				return;
			}
		}
	}

	stm32mp1_clk_disable_secure(id);
}

/*
 * Shared peripherals and resources.
 * Defines resource that may be non secure, secure or shared.
 * May be a device, a bus, a clock, a memory.
 *
 * State default to PERIPH_UNREGISTERED resource is not explicitly
 * set here.
 *
 * Resource driver not built, the resource defaults
 * to non secure ownership.
 *
 * Each IO of the GPIOZ IO can be secure or non secure.
 * When the GPIO driver is enabled, the GPIOZ bank is fully non secure
 * only if each IO is non secure and the GPIOZ bank is shared if it
 * includes secure and non secure IOs.
 *
 * BKPSRAM is assumed shared.
 * DDR control (DDRC and DDRPHY) is secure.
 * Inits will define the resource state according the device tree
 * and the driver initialization sequences.
 *
 * The platform initialization uses these information to set the ETZPC
 * configuration. Non secure services (as clocks or regulator accesses)
 * rely on these information to drive the related service execution.
 */
#define SHRES_NON_SECURE		3
#define SHRES_SHARED			2
#define SHRES_SECURE			1
#define SHRES_UNREGISTERED		0

static uint8_t shres_state[STM32MP1_SHRES_COUNT];

static const char *shres2str_id_tbl[STM32MP1_SHRES_COUNT] = {
	[STM32MP1_SHRES_GPIOZ(0)] = "GPIOZ0",
	[STM32MP1_SHRES_GPIOZ(1)] = "GPIOZ1",
	[STM32MP1_SHRES_GPIOZ(2)] = "GPIOZ2",
	[STM32MP1_SHRES_GPIOZ(3)] = "GPIOZ3",
	[STM32MP1_SHRES_GPIOZ(4)] = "GPIOZ4",
	[STM32MP1_SHRES_GPIOZ(5)] = "GPIOZ5",
	[STM32MP1_SHRES_GPIOZ(6)] = "GPIOZ6",
	[STM32MP1_SHRES_GPIOZ(7)] = "GPIOZ7",
	[STM32MP1_SHRES_IWDG1] = "IWDG1",
	[STM32MP1_SHRES_USART1] = "USART1",
	[STM32MP1_SHRES_SPI6] = "SPI6",
	[STM32MP1_SHRES_I2C4] = "I2C4",
	[STM32MP1_SHRES_RNG1] = "RNG1",
	[STM32MP1_SHRES_HASH1] = "HASH1",
	[STM32MP1_SHRES_CRYP1] = "CRYP1",
	[STM32MP1_SHRES_I2C6] = "I2C6",
	[STM32MP1_SHRES_RTC] = "RTC",
	[STM32MP1_SHRES_MCU] = "MCU",
	[STM32MP1_SHRES_HSI] = "HSI",
	[STM32MP1_SHRES_LSI] = "LSI",
	[STM32MP1_SHRES_HSE] = "HSE",
	[STM32MP1_SHRES_LSE] = "LSE",
	[STM32MP1_SHRES_CSI] = "CSI",
	[STM32MP1_SHRES_PLL1] = "PLL1",
	[STM32MP1_SHRES_PLL1_P] = "PLL1_P",
	[STM32MP1_SHRES_PLL1_Q] = "PLL1_Q",
	[STM32MP1_SHRES_PLL1_R] = "PLL1_R",
	[STM32MP1_SHRES_PLL2] = "PLL2",
	[STM32MP1_SHRES_PLL2_P] = "PLL2_P",
	[STM32MP1_SHRES_PLL2_Q] = "PLL2_Q",
	[STM32MP1_SHRES_PLL2_R] = "PLL2_R",
	[STM32MP1_SHRES_PLL3] = "PLL3",
	[STM32MP1_SHRES_PLL3_P] = "PLL3_P",
	[STM32MP1_SHRES_PLL3_Q] = "PLL3_Q",
	[STM32MP1_SHRES_PLL3_R] = "PLL3_R",
};

static const char *shres2str_id(unsigned int id)
{
	return shres2str_id_tbl[id];
}

static const char *shres2str_state_tbl[4] = {
	[SHRES_SHARED] = "shared",
	[SHRES_NON_SECURE] = "non secure",
	[SHRES_SECURE] = "secure",
	[SHRES_UNREGISTERED] = "unregistered",
};

static const char *shres2str_state(unsigned int id)
{
	return shres2str_state_tbl[id];
}

struct shres2decprot {
	unsigned int shres_id;
	unsigned int decprot_id;
	const char *decprot_str;
};

#define SHRES2DECPROT(shres, decprot, str) {	\
		.shres_id = shres,		\
		.decprot_id = decprot,		\
		.decprot_str = str,		\
	}

#define SHRES_INVALID		~0U
static const struct shres2decprot shres2decprot_tbl[] = {
	SHRES2DECPROT(STM32MP1_SHRES_IWDG1, STM32MP1_ETZPC_IWDG1_ID, "IWDG1"),
	SHRES2DECPROT(STM32MP1_SHRES_USART1, STM32MP1_ETZPC_USART1_ID, "UART1"),
	SHRES2DECPROT(STM32MP1_SHRES_SPI6, STM32MP1_ETZPC_SPI6_ID, "SPI6"),
	SHRES2DECPROT(STM32MP1_SHRES_I2C4, STM32MP1_ETZPC_I2C4_ID, "I2C4"),
	SHRES2DECPROT(STM32MP1_SHRES_RNG1, STM32MP1_ETZPC_RNG1_ID, "RNG1"),
	SHRES2DECPROT(STM32MP1_SHRES_HASH1, STM32MP1_ETZPC_HASH1_ID, "HASH1"),
	SHRES2DECPROT(STM32MP1_SHRES_CRYP1, STM32MP1_ETZPC_CRYP1_ID, "CRYP1"),
	SHRES2DECPROT(STM32MP1_SHRES_I2C6, STM32MP1_ETZPC_I2C6_ID, "I2C6"),
	/* Below are specific IDs without a 1-to-1 mapping to SHRES IDs */
	SHRES2DECPROT(SHRES_INVALID, STM32MP1_ETZPC_STGENC_ID, "STGEN"),
	SHRES2DECPROT(SHRES_INVALID, STM32MP1_ETZPC_BKPSRAM_ID, "BKPSRAM"),
	SHRES2DECPROT(SHRES_INVALID, STM32MP1_ETZPC_DDRCTRL_ID, "DDRCTRL"),
	SHRES2DECPROT(SHRES_INVALID, STM32MP1_ETZPC_DDRPHYC_ID, "DDRPHY"),
};

static unsigned int decprot2shres(unsigned int decprot_id)
{
	uint32_t i;

	for (i = 0; i < ARRAY_SIZE(shres2decprot_tbl); i++) {
		if (shres2decprot_tbl[i].decprot_id == decprot_id) {
			return shres2decprot_tbl[i].shres_id;
		}
	}

	VERBOSE("No shared resource %u", decprot_id);
	return SHRES_INVALID;
}

static const char *decprot2str(unsigned int decprot_id)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(shres2decprot_tbl); i++) {
		if (shres2decprot_tbl[i].decprot_id == decprot_id) {
			return shres2decprot_tbl[i].decprot_str;
		}
	}

	ERROR("Invalid ID %u", decprot_id);
	panic();
}

static unsigned int get_gpioz_nbpin(void)
{
	if (gpioz_nbpin < 0) {
		gpioz_nbpin = (int8_t)fdt_get_gpioz_nbpins_from_dt();
		assert((gpioz_nbpin == 0) ||
		       (gpioz_nbpin == STM32MP_GPIOZ_PIN_MAX_COUNT));
	}

	return (unsigned int)gpioz_nbpin;
}

static bool shareable_resource(unsigned int id)
{
	switch (id) {
	default:
		/* Currently no shareable resource */
		return false;
	}
}

static void register_periph(unsigned int id, unsigned int state)
{
	assert(id < STM32MP1_SHRES_COUNT &&
	       state > SHRES_UNREGISTERED &&
	       state <= SHRES_NON_SECURE);

	if (registering_locked) {
		if (shres_state[id] == state) {
			return;
		}

		panic();
	}

	if ((state == SHRES_SHARED && !shareable_resource(id)) ||
	    ((shres_state[id] != SHRES_UNREGISTERED) &&
	     (shres_state[id] != state))) {
		VERBOSE("Cannot change %s from %s to %s\n",
			shres2str_id(id),
			shres2str_state(shres_state[id]),
			shres2str_state(state));
		panic();
	}

	shres_state[id] = (uint8_t)state;

	if (shres_state[id] == SHRES_UNREGISTERED) {
		VERBOSE("Register %s as %s\n",
			shres2str_id(id), shres2str_state(state));
	}

	switch (id) {
	case STM32MP1_SHRES_GPIOZ(0) ... STM32MP1_SHRES_GPIOZ(7):
		if ((id - STM32MP1_SHRES_GPIOZ(0)) >= get_gpioz_nbpin()) {
			ERROR("Invalid GPIO pin %u, %u pin(s) available\n",
			      id - STM32MP1_SHRES_GPIOZ(0),
			      get_gpioz_nbpin());
			panic();
		}
		break;
	default:
		break;
	}

	/* Explore clock tree to lock dependencies */
	if ((state == SHRES_SECURE) || (state == SHRES_SHARED)) {
		switch (id) {
		case STM32MP1_SHRES_GPIOZ(0) ... STM32MP1_SHRES_GPIOZ(7):
			stm32mp1_register_clock_parents_secure(GPIOZ);
			break;
		case STM32MP1_SHRES_IWDG1:
			stm32mp1_register_clock_parents_secure(IWDG1);
			break;
		case STM32MP1_SHRES_USART1:
			stm32mp1_register_clock_parents_secure(USART1_K);
			break;
		case STM32MP1_SHRES_SPI6:
			stm32mp1_register_clock_parents_secure(SPI6_K);
			break;
		case STM32MP1_SHRES_I2C4:
			stm32mp1_register_clock_parents_secure(I2C4_K);
			break;
		case STM32MP1_SHRES_RNG1:
			stm32mp1_register_clock_parents_secure(RNG1_K);
			break;
		case STM32MP1_SHRES_HASH1:
			stm32mp1_register_clock_parents_secure(HASH1);
			break;
		case STM32MP1_SHRES_CRYP1:
			stm32mp1_register_clock_parents_secure(CRYP1);
			break;
		case STM32MP1_SHRES_I2C6:
			stm32mp1_register_clock_parents_secure(I2C6_K);
			break;
		case STM32MP1_SHRES_RTC:
			stm32mp1_register_clock_parents_secure(RTC);
			break;
		case STM32MP1_SHRES_PLL1_P:
		case STM32MP1_SHRES_PLL1_Q:
		case STM32MP1_SHRES_PLL1_R:
			register_periph(STM32MP1_SHRES_PLL1, SHRES_SECURE);
			stm32mp1_register_clock_parents_secure(PLL1);
			break;
		case STM32MP1_SHRES_PLL2_P:
		case STM32MP1_SHRES_PLL2_Q:
		case STM32MP1_SHRES_PLL2_R:
			register_periph(STM32MP1_SHRES_PLL2, SHRES_SECURE);
			stm32mp1_register_clock_parents_secure(PLL2);
			break;
		case STM32MP1_SHRES_PLL3_P:
		case STM32MP1_SHRES_PLL3_Q:
		case STM32MP1_SHRES_PLL3_R:
			register_periph(STM32MP1_SHRES_PLL3, SHRES_SECURE);
			stm32mp1_register_clock_parents_secure(PLL3);
			break;
		default:
			/* No expected resource dependency */
			break;
		}
	}
}

static bool stm32mp1_mckprot_resource(unsigned int id)
{
	switch (id) {
	case STM32MP1_SHRES_MCU:
	case STM32MP1_SHRES_PLL3:
	case STM32MP1_SHRES_PLL3_P:
	case STM32MP1_SHRES_PLL3_Q:
	case STM32MP1_SHRES_PLL3_R:
		return true;
	default:
		return false;
	}
}

/* Register resource by ID */
void stm32mp1_register_secure_periph(unsigned int id)
{
	register_periph(id, SHRES_SECURE);
}

void stm32mp1_register_shared_periph(unsigned int id)
{
	register_periph(id, SHRES_SHARED);
}

void stm32mp1_register_non_secure_periph(unsigned int id)
{
	register_periph(id, SHRES_NON_SECURE);
}

/* Register resource by IO memory base address */
static void register_periph_iomem(uintptr_t base, unsigned int state)
{
	unsigned int id;

	switch (base) {
	case IWDG1_BASE:
		id = STM32MP1_SHRES_IWDG1;
		break;
	case USART1_BASE:
		id = STM32MP1_SHRES_USART1;
		break;
	case SPI6_BASE:
		id = STM32MP1_SHRES_SPI6;
		break;
	case I2C4_BASE:
		id = STM32MP1_SHRES_I2C4;
		break;
	case I2C6_BASE:
		id = STM32MP1_SHRES_I2C6;
		break;
	case RTC_BASE:
		id = STM32MP1_SHRES_RTC;
		break;
	case RNG1_BASE:
		id = STM32MP1_SHRES_RNG1;
		break;
	case CRYP1_BASE:
		id = STM32MP1_SHRES_CRYP1;
		break;
	case HASH1_BASE:
		id = STM32MP1_SHRES_HASH1;
		break;

	case GPIOA_BASE:
	case GPIOB_BASE:
	case GPIOC_BASE:
	case GPIOD_BASE:
	case GPIOE_BASE:
	case GPIOF_BASE:
	case GPIOG_BASE:
	case GPIOH_BASE:
	case GPIOI_BASE:
	case GPIOJ_BASE:
	case GPIOK_BASE:
	case USART2_BASE:
	case USART3_BASE:
	case UART4_BASE:
	case UART5_BASE:
	case USART6_BASE:
	case UART7_BASE:
	case UART8_BASE:
	case IWDG2_BASE:
		/* Allow drivers to register some non secure resources */
		VERBOSE("IO for non secure resource 0x%x\n",
			(unsigned int)base);
		if (state != SHRES_NON_SECURE) {
			panic();
		}

		return;

	default:
		panic();
		break;
	}

	register_periph(id, state);
}

void stm32mp_register_secure_periph_iomem(uintptr_t base)
{
	register_periph_iomem(base, SHRES_SECURE);
}

void stm32mp_register_non_secure_periph_iomem(uintptr_t base)
{
	register_periph_iomem(base, SHRES_NON_SECURE);
}

/* Register GPIO resource */
void stm32mp_register_secure_gpio(unsigned int bank, unsigned int pin)
{
	switch (bank) {
	case GPIO_BANK_Z:
		register_periph(STM32MP1_SHRES_GPIOZ(pin), SHRES_SECURE);
		break;
	default:
		ERROR("GPIO bank %u cannot be secured\n", bank);
		panic();
	}
}

void stm32mp_register_non_secure_gpio(unsigned int bank, unsigned int pin)
{
	switch (bank) {
	case GPIO_BANK_Z:
		register_periph(STM32MP1_SHRES_GPIOZ(pin), SHRES_NON_SECURE);
		break;
	default:
		break;
	}
}

void stm32mp1_register_etzpc_decprot(unsigned int id,
				     enum etzpc_decprot_attributes attr)
{
	unsigned int state = SHRES_SECURE;
	unsigned int id_shres;

	switch (attr) {
	case TZPC_DECPROT_S_RW:
		break;
	case TZPC_DECPROT_NS_R_S_W:
	case TZPC_DECPROT_MCU_ISOLATION:
	case TZPC_DECPROT_NS_RW:
		state = SHRES_NON_SECURE;
		break;
	default:
		panic();
	}

	id_shres = decprot2shres(id);
	if (id_shres == SHRES_INVALID) {
		if (state == SHRES_SECURE) {
			panic();
		}
	} else {
		register_periph(id_shres, state);
	}
}

/* Get resource state: these accesses lock the registering support */
static void lock_registering(void)
{
	registering_locked = true;
}

bool stm32mp1_periph_is_shared(unsigned long id)
{
	lock_registering();

	return shres_state[id] == SHRES_SHARED;
}

bool stm32mp1_periph_is_non_secure(unsigned long id)
{
	lock_registering();

	return shres_state[id] == SHRES_NON_SECURE;
}

bool stm32mp1_periph_is_secure(unsigned long id)
{
	lock_registering();

	return shres_state[id] == SHRES_SECURE;
}

bool stm32mp1_periph_is_unregistered(unsigned long id)
{
	lock_registering();

	return shres_state[id] == SHRES_UNREGISTERED;
}

bool stm32mp_gpio_bank_is_shared(unsigned int bank)
{
	unsigned int non_secure = 0;
	unsigned int i;

	lock_registering();

	if (bank != GPIO_BANK_Z) {
		return false;
	}

	for (i = 0U; i < get_gpioz_nbpin(); i++) {
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_GPIOZ(i)) ||
		    stm32mp1_periph_is_unregistered(STM32MP1_SHRES_GPIOZ(i))) {
			non_secure++;
		}
	}

	return (non_secure != 0) && (non_secure < get_gpioz_nbpin());
}

bool stm32mp_gpio_bank_is_non_secure(unsigned int bank)
{
	unsigned int non_secure = 0;
	unsigned int i;

	lock_registering();

	if (bank != GPIO_BANK_Z) {
		return true;
	}

	for (i = 0U; i < get_gpioz_nbpin(); i++) {
		if (stm32mp1_periph_is_non_secure(STM32MP1_SHRES_GPIOZ(i)) ||
		    stm32mp1_periph_is_unregistered(STM32MP1_SHRES_GPIOZ(i))) {
			non_secure++;
		}
	}

	return non_secure == get_gpioz_nbpin();
}

bool stm32mp_gpio_bank_is_secure(unsigned int bank)
{
	unsigned int secure = 0;
	unsigned int i;

	lock_registering();

	if (bank != GPIO_BANK_Z) {
		return false;
	}

	for (i = 0U; i < get_gpioz_nbpin(); i++) {
		if (stm32mp1_periph_is_secure(STM32MP1_SHRES_GPIOZ(i))) {
			secure++;
		}
	}

	return secure == get_gpioz_nbpin();
}

bool stm32mp1_clock_is_shareable(unsigned long clock_id)
{
	switch (clock_id) {
	case GPIOZ:
		return get_gpioz_nbpin() > 0;
	case RTCAPB:
		return true;
	default:
		return false;
	}
}

bool stm32mp1_clock_is_shared(unsigned long clock_id)
{
	lock_registering();

	switch (clock_id) {
	case GPIOZ:
		if (get_gpioz_nbpin() > 0) {
			return stm32mp_gpio_bank_is_shared(GPIO_BANK_Z);
		} else {
			return false;
		}
	case RTCAPB:
		/* RTCAPB is shared for non secure backup registers */
		return true;
	default:
		return false;
	}
}

bool stm32mp1_clock_is_non_secure(unsigned long clock_id)
{
	unsigned int shres_id;

	lock_registering();

	if (stm32mp1_clock_is_shared(clock_id)) {
		return false;
	}

	switch (clock_id) {
	case BSEC:
	case BKPSRAM:
	case TZPC:
	case TZC1:
	case TZC2:
	case STGEN_K:
	case DDRC1:
	case DDRC1LP:
	case DDRC2:
	case DDRC2LP:
	case DDRPHYC:
	case DDRPHYCLP:
	case DDRCAPB:
	case DDRCAPBLP:
	case AXIDCG:
	case DDRPHYCAPB:
	case DDRPHYCAPBLP:
		return false;
	case IWDG1:
		shres_id = STM32MP1_SHRES_IWDG1;
		break;
	case USART1_K:
		shres_id = STM32MP1_SHRES_USART1;
		break;
	case SPI6_K:
		shres_id = STM32MP1_SHRES_SPI6;
		break;
	case I2C4_K:
		shres_id = STM32MP1_SHRES_I2C4;
		break;
	case RNG1_K:
		shres_id = STM32MP1_SHRES_RNG1;
		break;
	case HASH1:
		shres_id = STM32MP1_SHRES_HASH1;
		break;
	case CRYP1:
		shres_id = STM32MP1_SHRES_CRYP1;
		break;
	case I2C6_K:
		shres_id = STM32MP1_SHRES_I2C6;
		break;
	case RTC:
		shres_id = STM32MP1_SHRES_RTC;
		break;
	default:
		return true;
	}

	return stm32mp1_periph_is_non_secure(shres_id);
}

/* ETZPC configuration at drivers initialization completion */
static enum etzpc_decprot_attributes decprot_periph_attr(unsigned int id)
{
	switch (id) {
	case STM32MP1_SHRES_GPIOZ(0) ... STM32MP1_SHRES_GPIOZ(7):
		assert((id - STM32MP1_SHRES_GPIOZ(0)) < get_gpioz_nbpin());
		return TZPC_DECPROT_NS_RW;
	default:
		if (stm32mp1_periph_is_non_secure(id)) {
			return TZPC_DECPROT_NS_RW;
		}

		return TZPC_DECPROT_S_RW;
	}
}

static bool check_decprot(unsigned int id, enum etzpc_decprot_attributes exp)
{
	enum etzpc_decprot_attributes cur = etzpc_get_decprot(id);

	if (cur == exp) {
		return true;
	}

	switch (exp) {
	case TZPC_DECPROT_NS_RW:
		if (cur == TZPC_DECPROT_S_RW) {
			WARN("ETZPC: %s (%d) could be non secure\n",
			     decprot2str(id), id);
		}
		return true;

	case TZPC_DECPROT_S_RW:
		ERROR("ETZPC: %s (%d) expected secure but DECPROT = %d\n",
		      decprot2str(id), id, cur);
		break;

	case TZPC_DECPROT_NS_R_S_W:
	case TZPC_DECPROT_MCU_ISOLATION:
	default:
		panic();
	}

	return false;
}

static void check_etzpc_secure_configuration(void)
{
	bool error = false;

	assert(registering_locked);

	error |= !check_decprot(STM32MP1_ETZPC_STGENC_ID, TZPC_DECPROT_S_RW);

	error |= !check_decprot(STM32MP1_ETZPC_BKPSRAM_ID, TZPC_DECPROT_S_RW);

	error |= !check_decprot(STM32MP1_ETZPC_USART1_ID,
				decprot_periph_attr(STM32MP1_SHRES_USART1));

	error |= !check_decprot(STM32MP1_ETZPC_SPI6_ID,
				decprot_periph_attr(STM32MP1_SHRES_SPI6));

	error |= !check_decprot(STM32MP1_ETZPC_I2C4_ID,
				decprot_periph_attr(STM32MP1_SHRES_I2C4));

	error |= !check_decprot(STM32MP1_ETZPC_RNG1_ID,
				decprot_periph_attr(STM32MP1_SHRES_RNG1));

	error |= !check_decprot(STM32MP1_ETZPC_HASH1_ID,
				decprot_periph_attr(STM32MP1_SHRES_HASH1));

	error |= !check_decprot(STM32MP1_ETZPC_CRYP1_ID,
				decprot_periph_attr(STM32MP1_SHRES_CRYP1));

	error |= !check_decprot(STM32MP1_ETZPC_DDRCTRL_ID, TZPC_DECPROT_S_RW);

	error |= !check_decprot(STM32MP1_ETZPC_DDRPHYC_ID, TZPC_DECPROT_S_RW);

	error |= !check_decprot(STM32MP1_ETZPC_I2C6_ID,
				decprot_periph_attr(STM32MP1_SHRES_I2C6));

	if (error) {
		panic();
	}
}

static void check_rcc_secure_configuration(void)
{
	uint32_t n;
	uint32_t error = 0;
	bool mckprot = stm32mp1_rcc_is_mckprot();
	bool secure = stm32mp1_rcc_is_secure();

	for (n = 0; n < ARRAY_SIZE(shres_state); n++) {
		if  ((shres_state[n] == SHRES_SECURE) ||
		     (shres_state[n] == SHRES_SHARED)) {
			if ((stm32mp1_mckprot_resource(n) && (!mckprot)) ||
			    !secure) {
				ERROR("RCC %s MCKPROT %s and %s (%u) secure\n",
				      secure ? "secure" : "non secure",
				      mckprot ? "set" : "not set",
				      shres2str_id(n), n);
				error++;
			}
		}
	}

	if (error != 0U) {
		panic();
	}
}

static void check_gpio_secure_configuration(void)
{
	uint32_t pin;

	for (pin = 0U; pin < get_gpioz_nbpin(); pin++) {
		bool secure =
			stm32mp1_periph_is_secure(STM32MP1_SHRES_GPIOZ(pin));

		set_gpio_secure_cfg(GPIO_BANK_Z, pin, secure);
	}
}

void stm32mp1_driver_init_late(void)
{
	uint32_t __unused id;

	registering_locked = true;

#if LOG_LEVEL >= LOG_LEVEL_INFO
	for (id = 0; id < STM32MP1_SHRES_COUNT; id++) {
		uint8_t *state = &shres_state[id];

		/* Display only the secure and shared resources */
		if ((*state == SHRES_NON_SECURE) ||
		    ((*state == SHRES_UNREGISTERED))) {
			continue;
		}

		INFO("stm32mp %s (%u): %s\n",
		     shres2str_id(id), id,
		     *state == SHRES_SECURE ? "Secure only" :
		     *state == SHRES_SHARED ? "Shared" :
		     *state == SHRES_NON_SECURE ? "Non secure" :
		     *state == SHRES_UNREGISTERED ? "Unregistered" :
		     "<Invalid>");
	}
#endif

	stm32mp1_update_earlyboot_clocks_state();

	check_rcc_secure_configuration();
	check_etzpc_secure_configuration();
	check_gpio_secure_configuration();
}

