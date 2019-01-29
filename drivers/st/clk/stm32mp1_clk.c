/*
 * Copyright (C) 2018, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
 */

#include <arch.h>
#include <arch_helpers.h>
#include <assert.h>
#include <debug.h>
#include <delay_timer.h>
#include <dt-bindings/clock/stm32mp1-clksrc.h>
#include <errno.h>
#include <generic_delay_timer.h>
#include <libfdt.h>
#include <mmio.h>
#include <platform.h>
#include <spinlock.h>
#include <stdint.h>
#include <stdio.h>
#include <stm32mp_clkfunc.h>
#include <stm32mp_common.h>
#include <stm32mp_dt.h>
#include <stm32mp_shres_helpers.h>
#include <stm32mp1_clk.h>
#include <stm32mp1_clkfunc.h>
#include <stm32mp1_private.h>
#include <stm32mp1_rcc.h>
#include <stm32mp1_shared_resources.h>
#if defined(IMAGE_BL32)
#include <stm32_timer.h>
#endif
#include <utils_def.h>

#define MAX_HSI_HZ	64000000
#define USB_PHY_48_MHZ	48000000

#define TIMEOUT_200MS	ms2tick(200)
#define TIMEOUT_1S	s2tick(1)

#define PLLRDY_TIMEOUT	TIMEOUT_200MS
#define CLKSRC_TIMEOUT	TIMEOUT_200MS
#define CLKDIV_TIMEOUT	TIMEOUT_200MS
#define HSIDIV_TIMEOUT	TIMEOUT_200MS
#define OSCRDY_TIMEOUT	TIMEOUT_1S

#if defined(IMAGE_BL32)
#define CAL_MAX_RETRY	20U
#endif

enum stm32mp1_parent_id {
/* Oscillators are defined in enum stm32mp_osc_id */

/* Other parent source */
	_HSI_KER = NB_OSC,
	_HSE_KER,
	_HSE_KER_DIV2,
	_CSI_KER,
	_PLL1_P,
	_PLL1_Q,
	_PLL1_R,
	_PLL2_P,
	_PLL2_Q,
	_PLL2_R,
	_PLL3_P,
	_PLL3_Q,
	_PLL3_R,
	_PLL4_P,
	_PLL4_Q,
	_PLL4_R,
	_ACLK,
	_PCLK1,
	_PCLK2,
	_PCLK3,
	_PCLK4,
	_PCLK5,
	_HCLK6,
	_HCLK2,
	_CK_PER,
	_CK_MPU,
	_CK_MCU,
	_USB_PHY_48,
	_PARENT_NB,
	_UNKNOWN_ID = 0xff,
};

/* Lists only the parent clock we are interested in */
enum stm32mp1_parent_sel {
	_I2C12_SEL,
	_I2C35_SEL,
	_STGEN_SEL,
	_I2C46_SEL,
	_SPI6_SEL,
	_USART1_SEL,
	_RNG1_SEL,
	_UART6_SEL,
	_UART24_SEL,
	_UART35_SEL,
	_UART78_SEL,
	_SDMMC12_SEL,
	_SDMMC3_SEL,
	_QSPI_SEL,
	_FMC_SEL,
	_ASS_SEL,
	_MSS_SEL,
	_USBPHY_SEL,
	_USBO_SEL,
	_PARENT_SEL_NB,
	_UNKNOWN_SEL = 0xff,
};

enum stm32mp1_pll_id {
	_PLL1,
	_PLL2,
	_PLL3,
	_PLL4,
	_PLL_NB
};

enum stm32mp1_div_id {
	_DIV_P,
	_DIV_Q,
	_DIV_R,
	_DIV_NB,
};

enum stm32mp1_clksrc_id {
	CLKSRC_MPU,
	CLKSRC_AXI,
	CLKSRC_MCU,
	CLKSRC_PLL12,
	CLKSRC_PLL3,
	CLKSRC_PLL4,
	CLKSRC_RTC,
	CLKSRC_MCO1,
	CLKSRC_MCO2,
	CLKSRC_NB
};

enum stm32mp1_clkdiv_id {
	CLKDIV_MPU,
	CLKDIV_AXI,
	CLKDIV_MCU,
	CLKDIV_APB1,
	CLKDIV_APB2,
	CLKDIV_APB3,
	CLKDIV_APB4,
	CLKDIV_APB5,
	CLKDIV_RTC,
	CLKDIV_MCO1,
	CLKDIV_MCO2,
	CLKDIV_NB
};

enum stm32mp1_pllcfg {
	PLLCFG_M,
	PLLCFG_N,
	PLLCFG_P,
	PLLCFG_Q,
	PLLCFG_R,
	PLLCFG_O,
	PLLCFG_NB
};

enum stm32mp1_pllcsg {
	PLLCSG_MOD_PER,
	PLLCSG_INC_STEP,
	PLLCSG_SSCG_MODE,
	PLLCSG_NB
};

enum stm32mp1_plltype {
	PLL_800,
	PLL_1600,
	PLL_TYPE_NB
};

struct stm32mp1_pll {
	uint8_t refclk_min;
	uint8_t refclk_max;
	uint8_t divn_max;
};

struct stm32mp1_clk_gate {
	uint16_t offset;
	uint8_t bit;
	uint8_t index;
	uint8_t set_clr;
	uint8_t sel; /* Relates to enum stm32mp1_parent_sel */
	uint8_t fixed; /* Relates to enum stm32mp1_parent_id */
};

struct stm32mp1_clk_sel {
	uint16_t offset;
	uint8_t src;
	uint8_t msk;
	uint8_t nb_parent;
	const uint8_t *parent;
};

#define REFCLK_SIZE 4
struct stm32mp1_clk_pll {
	enum stm32mp1_plltype plltype;
	uint16_t rckxselr;
	uint16_t pllxcfgr1;
	uint16_t pllxcfgr2;
	uint16_t pllxfracr;
	uint16_t pllxcr;
	uint16_t pllxcsgr;
	enum stm32mp_osc_id refclk[REFCLK_SIZE];
};

#if defined(IMAGE_BL32)
struct stm32mp1_trim_boundary_t {
	/* Max boundary trim value around forbidden value */
	unsigned int x1;
	/* Min boundary trim value around forbidden value */
	unsigned int x2;
};

struct stm32mp1_clk_cal {
	uint16_t *fbv;
	unsigned int cal_ref;
	int trim_max;
	int trim_min;
	unsigned int boundary_max;
	unsigned long ref_freq;
	unsigned int freq_margin;
	unsigned long (*get_freq)(void);
	void (*set_trim)(unsigned int cal);
	unsigned int (*get_trim)(void);
	struct stm32mp1_trim_boundary_t boundary[16];
};

/* RCC Wakeup status */
static bool rcc_wakeup;
#endif

/* Clocks with selectable source and non set/clr register access */
#define _CLK_SELEC(off, b, idx, s)			\
	{						\
		.offset = (off),			\
		.bit = (b),				\
		.index = (idx),				\
		.set_clr = 0,				\
		.sel = (s),				\
		.fixed = _UNKNOWN_ID,			\
	}

/* Clocks with fixed source and non set/clr register access */
#define _CLK_FIXED(off, b, idx, f)			\
	{						\
		.offset = (off),			\
		.bit = (b),				\
		.index = (idx),				\
		.set_clr = 0,				\
		.sel = _UNKNOWN_SEL,			\
		.fixed = (f),				\
	}

/* Clocks with selectable source and set/clr register access */
#define _CLK_SC_SELEC(off, b, idx, s)			\
	{						\
		.offset = (off),			\
		.bit = (b),				\
		.index = (idx),				\
		.set_clr = 1,				\
		.sel = (s),				\
		.fixed = _UNKNOWN_ID,			\
	}

/* Clocks with fixed source and set/clr register access */
#define _CLK_SC_FIXED(off, b, idx, f)			\
	{						\
		.offset = (off),			\
		.bit = (b),				\
		.index = (idx),				\
		.set_clr = 1,				\
		.sel = _UNKNOWN_SEL,			\
		.fixed = (f),				\
	}

#define _CLK_PARENT(idx, off, s, m, p)			\
	[(idx)] = {					\
		.offset = (off),			\
		.src = (s),				\
		.msk = (m),				\
		.parent = (p),				\
		.nb_parent = ARRAY_SIZE(p)		\
	}

#define _CLK_PLL(idx, type, off1, off2, off3,		\
		 off4, off5, off6,			\
		 p1, p2, p3, p4)			\
	[(idx)] = {					\
		.plltype = (type),			\
		.rckxselr = (off1),			\
		.pllxcfgr1 = (off2),			\
		.pllxcfgr2 = (off3),			\
		.pllxfracr = (off4),			\
		.pllxcr = (off5),			\
		.pllxcsgr = (off6),			\
		.refclk[0] = (p1),			\
		.refclk[1] = (p2),			\
		.refclk[2] = (p3),			\
		.refclk[3] = (p4),			\
	}

static const uint8_t stm32mp1_clks[][2] = {
	{ CK_PER, _CK_PER },
	{ CK_MPU, _CK_MPU },
	{ CK_AXI, _ACLK },
	{ CK_MCU, _CK_MCU },
	{ CK_HSE, _HSE },
	{ CK_CSI, _CSI },
	{ CK_LSI, _LSI },
	{ CK_LSE, _LSE },
	{ CK_HSI, _HSI },
	{ CK_HSE_DIV2, _HSE_KER_DIV2 },
};

#define NB_GATES	ARRAY_SIZE(stm32mp1_clk_gate)

static const struct stm32mp1_clk_gate stm32mp1_clk_gate[] = {
	_CLK_FIXED(RCC_DDRITFCR, 0, DDRC1, _ACLK),
	_CLK_FIXED(RCC_DDRITFCR, 1, DDRC1LP, _ACLK),
	_CLK_FIXED(RCC_DDRITFCR, 2, DDRC2, _ACLK),
	_CLK_FIXED(RCC_DDRITFCR, 3, DDRC2LP, _ACLK),
	_CLK_FIXED(RCC_DDRITFCR, 4, DDRPHYC, _PLL2_R),
	_CLK_FIXED(RCC_DDRITFCR, 5, DDRPHYCLP, _PLL2_R),
	_CLK_FIXED(RCC_DDRITFCR, 6, DDRCAPB, _PCLK4),
	_CLK_FIXED(RCC_DDRITFCR, 7, DDRCAPBLP, _PCLK4),
	_CLK_FIXED(RCC_DDRITFCR, 8, AXIDCG, _ACLK),
	_CLK_FIXED(RCC_DDRITFCR, 9, DDRPHYCAPB, _PCLK4),
	_CLK_FIXED(RCC_DDRITFCR, 10, DDRPHYCAPBLP, _PCLK4),
	_CLK_SC_FIXED(RCC_MP_APB1ENSETR, 6, TIM12_K, _PCLK1),
	_CLK_SC_SELEC(RCC_MP_APB1ENSETR, 14, USART2_K, _UART24_SEL),
	_CLK_SC_SELEC(RCC_MP_APB1ENSETR, 15, USART3_K, _UART35_SEL),
	_CLK_SC_SELEC(RCC_MP_APB1ENSETR, 16, UART4_K, _UART24_SEL),
	_CLK_SC_SELEC(RCC_MP_APB1ENSETR, 17, UART5_K, _UART35_SEL),
	_CLK_SC_SELEC(RCC_MP_APB1ENSETR, 18, UART7_K, _UART78_SEL),
	_CLK_SC_SELEC(RCC_MP_APB1ENSETR, 19, UART8_K, _UART78_SEL),
	_CLK_SC_SELEC(RCC_MP_APB1ENSETR, 21, I2C1_K, _I2C12_SEL),
	_CLK_SC_SELEC(RCC_MP_APB1ENSETR, 22, I2C2_K, _I2C12_SEL),
	_CLK_SC_SELEC(RCC_MP_APB1ENSETR, 23, I2C3_K, _I2C35_SEL),
	_CLK_SC_SELEC(RCC_MP_APB1ENSETR, 24, I2C5_K, _I2C35_SEL),
	_CLK_SC_FIXED(RCC_MP_APB2ENSETR, 2, TIM15_K, _PCLK2),
	_CLK_SC_SELEC(RCC_MP_APB2ENSETR, 13, USART6_K, _UART6_SEL),

	_CLK_SC_SELEC(RCC_MP_APB4ENSETR, 8, DDRPERFM, _UNKNOWN_SEL),
	_CLK_SC_SELEC(RCC_MP_APB4ENSETR, 15, IWDG2, _UNKNOWN_SEL),
	_CLK_SC_SELEC(RCC_MP_APB4ENSETR, 16, USBPHY_K, _USBPHY_SEL),

	_CLK_SC_SELEC(RCC_MP_APB5ENSETR, 0, SPI6_K, _SPI6_SEL),
	_CLK_SC_SELEC(RCC_MP_APB5ENSETR, 2, I2C4_K, _I2C46_SEL),
	_CLK_SC_SELEC(RCC_MP_APB5ENSETR, 3, I2C6_K, _I2C46_SEL),
	_CLK_SC_SELEC(RCC_MP_APB5ENSETR, 4, USART1_K, _USART1_SEL),
	_CLK_SC_FIXED(RCC_MP_APB5ENSETR, 8, RTCAPB, _PCLK5),
	_CLK_SC_FIXED(RCC_MP_APB5ENSETR, 11, TZC1, _PCLK5),
	_CLK_SC_FIXED(RCC_MP_APB5ENSETR, 12, TZC2, _PCLK5),
	_CLK_SC_FIXED(RCC_MP_APB5ENSETR, 13, TZPC, _PCLK5),
	_CLK_SC_FIXED(RCC_MP_APB5ENSETR, 15, IWDG1, _PCLK5),
	_CLK_SC_FIXED(RCC_MP_APB5ENSETR, 16, BSEC, _PCLK5),
	_CLK_SC_SELEC(RCC_MP_APB5ENSETR, 20, STGEN_K, _STGEN_SEL),

	_CLK_SC_SELEC(RCC_MP_AHB2ENSETR, 8, USBO_K, _USBO_SEL),
	_CLK_SC_SELEC(RCC_MP_AHB2ENSETR, 16, SDMMC3_K, _SDMMC3_SEL),

	_CLK_SC_SELEC(RCC_MP_AHB4ENSETR, 0, GPIOA, _UNKNOWN_SEL),
	_CLK_SC_SELEC(RCC_MP_AHB4ENSETR, 1, GPIOB, _UNKNOWN_SEL),
	_CLK_SC_SELEC(RCC_MP_AHB4ENSETR, 2, GPIOC, _UNKNOWN_SEL),
	_CLK_SC_SELEC(RCC_MP_AHB4ENSETR, 3, GPIOD, _UNKNOWN_SEL),
	_CLK_SC_SELEC(RCC_MP_AHB4ENSETR, 4, GPIOE, _UNKNOWN_SEL),
	_CLK_SC_SELEC(RCC_MP_AHB4ENSETR, 5, GPIOF, _UNKNOWN_SEL),
	_CLK_SC_SELEC(RCC_MP_AHB4ENSETR, 6, GPIOG, _UNKNOWN_SEL),
	_CLK_SC_SELEC(RCC_MP_AHB4ENSETR, 7, GPIOH, _UNKNOWN_SEL),
	_CLK_SC_SELEC(RCC_MP_AHB4ENSETR, 8, GPIOI, _UNKNOWN_SEL),
	_CLK_SC_SELEC(RCC_MP_AHB4ENSETR, 9, GPIOJ, _UNKNOWN_SEL),
	_CLK_SC_SELEC(RCC_MP_AHB4ENSETR, 10, GPIOK, _UNKNOWN_SEL),

	_CLK_SC_FIXED(RCC_MP_AHB5ENSETR, 0, GPIOZ, _PCLK5),
	_CLK_SC_FIXED(RCC_MP_AHB5ENSETR, 4, CRYP1, _PCLK5),
	_CLK_SC_FIXED(RCC_MP_AHB5ENSETR, 5, HASH1, _PCLK5),
	_CLK_SC_SELEC(RCC_MP_AHB5ENSETR, 6, RNG1_K, _RNG1_SEL),
	_CLK_SC_FIXED(RCC_MP_AHB5ENSETR, 8, BKPSRAM, _PCLK5),

	_CLK_SC_SELEC(RCC_MP_AHB6ENSETR, 12, FMC_K, _FMC_SEL),
	_CLK_SC_SELEC(RCC_MP_AHB6ENSETR, 14, QSPI_K, _QSPI_SEL),
	_CLK_SC_SELEC(RCC_MP_AHB6ENSETR, 16, SDMMC1_K, _SDMMC12_SEL),
	_CLK_SC_SELEC(RCC_MP_AHB6ENSETR, 17, SDMMC2_K, _SDMMC12_SEL),
	_CLK_SC_SELEC(RCC_MP_AHB6ENSETR, 24, USBH, _UNKNOWN_SEL),

	_CLK_SELEC(RCC_DBGCFGR, 8, CK_DBG, _UNKNOWN_SEL),
};

static const uint8_t i2c12_parents[] = {
	_PCLK1, _PLL4_R, _HSI_KER, _CSI_KER
};

static const uint8_t i2c35_parents[] = {
	_PCLK1, _PLL4_R, _HSI_KER, _CSI_KER
};

static const uint8_t stgen_parents[] = {
	_HSI_KER, _HSE_KER
};

static const uint8_t i2c46_parents[] = {
	_PCLK5, _PLL3_Q, _HSI_KER, _CSI_KER
};

static const uint8_t spi6_parents[] = {
	_PCLK5, _PLL4_Q, _HSI_KER, _CSI_KER, _HSE_KER, _PLL3_Q
};

static const uint8_t usart1_parents[] = {
	_PCLK5, _PLL3_Q, _HSI_KER, _CSI_KER, _PLL4_Q, _HSE_KER
};

static const uint8_t rng1_parents[] = {
	_CSI, _PLL4_R, _LSE, _LSI
};

static const uint8_t uart6_parents[] = {
	_PCLK2, _PLL4_Q, _HSI_KER, _CSI_KER, _HSE_KER
};

static const uint8_t uart234578_parents[] = {
	_PCLK1, _PLL4_Q, _HSI_KER, _CSI_KER, _HSE_KER
};

static const uint8_t sdmmc12_parents[] = {
	_HCLK6, _PLL3_R, _PLL4_P, _HSI_KER
};

static const uint8_t sdmmc3_parents[] = {
	_HCLK2, _PLL3_R, _PLL4_P, _HSI_KER
};

static const uint8_t qspi_parents[] = {
	_ACLK, _PLL3_R, _PLL4_P, _CK_PER
};

static const uint8_t fmc_parents[] = {
	_ACLK, _PLL3_R, _PLL4_P, _CK_PER
};

static const uint8_t ass_parents[] = {
	_HSI, _HSE, _PLL2
};

static const uint8_t mss_parents[] = {
	_HSI, _HSE, _CSI, _PLL3
};

static const uint8_t usbphy_parents[] = {
	_HSE_KER, _PLL4_R, _HSE_KER_DIV2
};

static const uint8_t usbo_parents[] = {
	_PLL4_R, _USB_PHY_48
};

static const struct stm32mp1_clk_sel stm32mp1_clk_sel[_PARENT_SEL_NB] = {
	_CLK_PARENT(_I2C12_SEL, RCC_I2C12CKSELR, 0, 0x7, i2c12_parents),
	_CLK_PARENT(_I2C35_SEL, RCC_I2C35CKSELR, 0, 0x7, i2c35_parents),
	_CLK_PARENT(_STGEN_SEL, RCC_STGENCKSELR, 0, 0x3, stgen_parents),
	_CLK_PARENT(_I2C46_SEL, RCC_I2C46CKSELR, 0, 0x7, i2c46_parents),
	_CLK_PARENT(_SPI6_SEL, RCC_SPI6CKSELR, 0, 0x7, spi6_parents),
	_CLK_PARENT(_USART1_SEL, RCC_UART1CKSELR, 0, 0x7, usart1_parents),
	_CLK_PARENT(_RNG1_SEL, RCC_RNG1CKSELR, 0, 0x3, rng1_parents),
	_CLK_PARENT(_UART6_SEL, RCC_UART6CKSELR, 0, 0x7, uart6_parents),
	_CLK_PARENT(_UART24_SEL, RCC_UART24CKSELR, 0, 0x7, uart234578_parents),
	_CLK_PARENT(_UART35_SEL, RCC_UART35CKSELR, 0, 0x7, uart234578_parents),
	_CLK_PARENT(_UART78_SEL, RCC_UART78CKSELR, 0, 0x7, uart234578_parents),
	_CLK_PARENT(_SDMMC12_SEL, RCC_SDMMC12CKSELR, 0, 0x7, sdmmc12_parents),
	_CLK_PARENT(_SDMMC3_SEL, RCC_SDMMC3CKSELR, 0, 0x7, sdmmc3_parents),
	_CLK_PARENT(_QSPI_SEL, RCC_QSPICKSELR, 0, 0xf, qspi_parents),
	_CLK_PARENT(_FMC_SEL, RCC_FMCCKSELR, 0, 0xf, fmc_parents),
	_CLK_PARENT(_ASS_SEL, RCC_ASSCKSELR, 0, 0x3, ass_parents),
	_CLK_PARENT(_MSS_SEL, RCC_MSSCKSELR, 0, 0x3, mss_parents),
	_CLK_PARENT(_USBPHY_SEL, RCC_USBCKSELR, 0, 0x3, usbphy_parents),
	_CLK_PARENT(_USBO_SEL, RCC_USBCKSELR, 4, 0x1, usbo_parents),
};

/* Define characteristic of PLL according type */
#define DIVN_MIN	24
static const struct stm32mp1_pll stm32mp1_pll[PLL_TYPE_NB] = {
	[PLL_800] = {
		.refclk_min = 4,
		.refclk_max = 16,
		.divn_max = 99,
	},
	[PLL_1600] = {
		.refclk_min = 8,
		.refclk_max = 16,
		.divn_max = 199,
	},
};

/* PLLNCFGR2 register divider by output */
static const uint8_t pllncfgr2[_DIV_NB] = {
	[_DIV_P] = RCC_PLLNCFGR2_DIVP_SHIFT,
	[_DIV_Q] = RCC_PLLNCFGR2_DIVQ_SHIFT,
	[_DIV_R] = RCC_PLLNCFGR2_DIVR_SHIFT,
};

static const struct stm32mp1_clk_pll stm32mp1_clk_pll[_PLL_NB] = {
	_CLK_PLL(_PLL1, PLL_1600,
		 RCC_RCK12SELR, RCC_PLL1CFGR1, RCC_PLL1CFGR2,
		 RCC_PLL1FRACR, RCC_PLL1CR, RCC_PLL1CSGR,
		 _HSI, _HSE, _UNKNOWN_OSC_ID, _UNKNOWN_OSC_ID),
	_CLK_PLL(_PLL2, PLL_1600,
		 RCC_RCK12SELR, RCC_PLL2CFGR1, RCC_PLL2CFGR2,
		 RCC_PLL2FRACR, RCC_PLL2CR, RCC_PLL2CSGR,
		 _HSI, _HSE, _UNKNOWN_OSC_ID, _UNKNOWN_OSC_ID),
	_CLK_PLL(_PLL3, PLL_800,
		 RCC_RCK3SELR, RCC_PLL3CFGR1, RCC_PLL3CFGR2,
		 RCC_PLL3FRACR, RCC_PLL3CR, RCC_PLL3CSGR,
		 _HSI, _HSE, _CSI, _UNKNOWN_OSC_ID),
	_CLK_PLL(_PLL4, PLL_800,
		 RCC_RCK4SELR, RCC_PLL4CFGR1, RCC_PLL4CFGR2,
		 RCC_PLL4FRACR, RCC_PLL4CR, RCC_PLL4CSGR,
		 _HSI, _HSE, _CSI, _I2S_CKIN),
};

/* Prescaler table lookups for clock computation */
/* div = /1 /2 /4 /8 / 16 /64 /128 /512 */
static const uint8_t stm32mp1_mcu_div[16] = {
	0, 1, 2, 3, 4, 6, 7, 8, 9, 9, 9, 9, 9, 9, 9, 9
};

/* div = /1 /2 /4 /8 /16 : same divider for PMU and APBX */
#define stm32mp1_mpu_div stm32mp1_mpu_apbx_div
#define stm32mp1_apbx_div stm32mp1_mpu_apbx_div
static const uint8_t stm32mp1_mpu_apbx_div[8] = {
	0, 1, 2, 3, 4, 4, 4, 4
};

/* div = /1 /2 /3 /4 */
static const uint8_t stm32mp1_axi_div[8] = {
	1, 2, 3, 4, 4, 4, 4, 4
};

#if LOG_LEVEL >= LOG_LEVEL_VERBOSE
static const char * const stm32mp1_clk_parent_name[_PARENT_NB] __unused = {
	[_HSI] = "HSI",
	[_HSE] = "HSE",
	[_CSI] = "CSI",
	[_LSI] = "LSI",
	[_LSE] = "LSE",
	[_I2S_CKIN] = "I2S_CKIN",
	[_HSI_KER] = "HSI_KER",
	[_HSE_KER] = "HSE_KER",
	[_HSE_KER_DIV2] = "HSE_KER_DIV2",
	[_CSI_KER] = "CSI_KER",
	[_PLL1_P] = "PLL1_P",
	[_PLL1_Q] = "PLL1_Q",
	[_PLL1_R] = "PLL1_R",
	[_PLL2_P] = "PLL2_P",
	[_PLL2_Q] = "PLL2_Q",
	[_PLL2_R] = "PLL2_R",
	[_PLL3_P] = "PLL3_P",
	[_PLL3_Q] = "PLL3_Q",
	[_PLL3_R] = "PLL3_R",
	[_PLL4_P] = "PLL4_P",
	[_PLL4_Q] = "PLL4_Q",
	[_PLL4_R] = "PLL4_R",
	[_ACLK] = "ACLK",
	[_PCLK1] = "PCLK1",
	[_PCLK2] = "PCLK2",
	[_PCLK3] = "PCLK3",
	[_PCLK4] = "PCLK4",
	[_PCLK5] = "PCLK5",
	[_HCLK6] = "KCLK6",
	[_HCLK2] = "HCLK2",
	[_CK_PER] = "CK_PER",
	[_CK_MPU] = "CK_MPU",
	[_CK_MCU] = "CK_MCU",
	[_USB_PHY_48] = "USB_PHY_48",
};

static const char *
const stm32mp1_clk_parent_sel_name[_PARENT_SEL_NB] __unused = {
	[_I2C12_SEL] = "I2C12",
	[_I2C35_SEL] = "I2C35",
	[_STGEN_SEL] = "STGEN",
	[_I2C46_SEL] = "I2C46",
	[_SPI6_SEL] = "SPI6",
	[_USART1_SEL] = "USART1",
	[_RNG1_SEL] = "RNG1",
	[_UART6_SEL] = "UART6",
	[_UART24_SEL] = "UART24",
	[_UART35_SEL] = "UART35",
	[_UART78_SEL] = "UART78",
	[_SDMMC12_SEL] = "SDMMC12",
	[_SDMMC3_SEL] = "SDMMC3",
	[_QSPI_SEL] = "QSPI",
	[_FMC_SEL] = "FMC",
	[_ASS_SEL] = "ASS",
	[_MSS_SEL] = "MSS",
	[_USBPHY_SEL] = "USBPHY",
	[_USBO_SEL] = "USBO",
};
#endif

/* RCC clock device driver private */
static unsigned long stm32mp1_osc[NB_OSC];
static struct spinlock reg_lock;
static unsigned int gate_refcounts[NB_GATES];
static struct spinlock refcount_lock;

static const struct stm32mp1_clk_gate *gate_ref(unsigned int idx)
{
	return &stm32mp1_clk_gate[idx];
}

static const struct stm32mp1_clk_sel *clk_sel_ref(unsigned int idx)
{
	return &stm32mp1_clk_sel[idx];
}

static const struct stm32mp1_clk_pll *pll_ref(unsigned int idx)
{
	return &stm32mp1_clk_pll[idx];
}

#if defined(IMAGE_BL32)
/* List of forbiden values for hsi */
static uint16_t fbv_hsi[] = { 512, 480, 448, 416, 384, 352, 320, 288, 256, 224,
	192, 160, 128, 96, 64, 32, 0 };
static uint16_t fbv_csi[] = { 256, 240, 224, 208, 192, 176, 160, 144, 128, 112,
	96, 80, 64, 48, 32, 16, 0 };

static void hsi_set_trim(unsigned int cal);
static unsigned int hsi_get_trimed_cal(void);
static void csi_set_trim(unsigned int cal);
static unsigned int csi_get_trimed_cal(void);

static struct stm32mp1_clk_cal stm32mp1_clk_cal_hsi = {
	.fbv = fbv_hsi,
	.trim_max = 63,
	.trim_min = -64,
	.ref_freq = 0,
	.freq_margin = 1,
	.set_trim = hsi_set_trim,
	.get_trim = hsi_get_trimed_cal,
};

static struct stm32mp1_clk_cal stm32mp1_clk_cal_csi = {
	.fbv = fbv_csi,
	.trim_max = 15,
	.trim_min = -16,
	.ref_freq = 0,
	.freq_margin = 2,
	.set_trim = csi_set_trim,
	.get_trim = csi_get_trimed_cal,
};

static uint32_t timer_val;
#endif

static int stm32mp1_lock_available(void)
{
	/* The spinlocks are used only when MMU is enabled */
	return (read_sctlr() & SCTLR_M_BIT) && (read_sctlr() & SCTLR_C_BIT);
}

static void stm32mp1_clk_lock(struct spinlock *lock)
{
	if (stm32mp1_lock_available() == 0U) {
		return;
	}

	/* Assume interrupts are masked */
	spin_lock(lock);
}

static void stm32mp1_clk_unlock(struct spinlock *lock)
{
	if (stm32mp1_lock_available() == 0U) {
		return;
	}

	spin_unlock(lock);
}

bool stm32mp1_rcc_is_secure(void)
{
	uintptr_t rcc_base = stm32mp_rcc_base();

	return (mmio_read_32(rcc_base + RCC_TZCR) & RCC_TZCR_TZEN) != 0;
}

bool stm32mp1_rcc_is_mckprot(void)
{
	uintptr_t rcc_base = stm32mp_rcc_base();

	return (mmio_read_32(rcc_base + RCC_TZCR) & RCC_TZCR_MCKPROT) != 0;
}

void stm32mp1_clk_rcc_regs_lock(void)
{
	stm32mp1_clk_lock(&reg_lock);
}

void stm32mp1_clk_rcc_regs_unlock(void)
{
	stm32mp1_clk_unlock(&reg_lock);
}

static unsigned int get_id_from_rcc_bit(unsigned int offset, unsigned int bit)
{
	unsigned int idx;

	for (idx = 0U; idx < NB_GATES; idx++) {
		const struct stm32mp1_clk_gate *gate = gate_ref(idx);

		if ((offset == gate->offset) && (bit == gate->bit)) {
			return gate->index;
		}

		if ((gate->set_clr != 0U) &&
		    (offset == (gate->offset + RCC_MP_ENCLRR_OFFSET)) &&
		    (bit == gate->bit)) {
			return gate->index;
		}
	}

	/* Currently only supported gated clocks */
	return ~0U;
}

static unsigned long stm32mp1_clk_get_fixed(enum stm32mp_osc_id idx)
{
	if (idx >= NB_OSC) {
		return 0;
	}

	return stm32mp1_osc[idx];
}

static int stm32mp1_clk_get_gated_id(unsigned long id)
{
	unsigned int i;

	for (i = 0U; i < NB_GATES; i++) {
		if (gate_ref(i)->index == id) {
			return i;
		}
	}

	ERROR("%s: clk id %d not found\n", __func__, (uint32_t)id);

	return -EINVAL;
}

static enum stm32mp1_parent_sel stm32mp1_clk_get_sel(int i)
{
	return (enum stm32mp1_parent_sel)(gate_ref(i)->sel);
}

static enum stm32mp1_parent_id stm32mp1_clk_get_fixed_parent(int i)
{
	return (enum stm32mp1_parent_id)gate_ref(i)->fixed;
}

static int stm32mp1_clk_get_parent(unsigned long id)
{
	const struct stm32mp1_clk_sel *sel;
	uint32_t j, p_sel;
	int i;
	enum stm32mp1_parent_id p;
	enum stm32mp1_parent_sel s;
	uintptr_t rcc_base = stm32mp_rcc_base();

	for (j = 0U; j < ARRAY_SIZE(stm32mp1_clks); j++) {
		if (stm32mp1_clks[j][0] == id) {
			return (int)stm32mp1_clks[j][1];
		}
	}

	i = stm32mp1_clk_get_gated_id(id);
	if (i < 0) {
		panic();
	}

	p = stm32mp1_clk_get_fixed_parent(i);
	if (p < _PARENT_NB) {
		return (int)p;
	}

	s = stm32mp1_clk_get_sel(i);
	if (s == _UNKNOWN_SEL) {
		return -EINVAL;
	}
	if (s >= _PARENT_SEL_NB) {
		panic();
	}

	sel = clk_sel_ref(s);
	p_sel = (mmio_read_32(rcc_base + sel->offset) >> sel->src) & sel->msk;
	if (p_sel < sel->nb_parent) {
#if LOG_LEVEL >= LOG_LEVEL_VERBOSE
		VERBOSE("%s: %s clock is the parent %s of clk id %ld\n",
			__func__,
			stm32mp1_clk_parent_name[sel->parent[p_sel]],
			stm32mp1_clk_parent_sel_name[s], id);
#endif
		return (int)sel->parent[p_sel];
	}

	return -EINVAL;
}

static unsigned long stm32mp1_pll_get_fref(const struct stm32mp1_clk_pll *pll)
{
	uint32_t selr = mmio_read_32(stm32mp_rcc_base() + pll->rckxselr);
	uint32_t src = selr & RCC_SELR_REFCLK_SRC_MASK;

	return stm32mp1_clk_get_fixed(pll->refclk[src]);
}

/*
 * pll_get_fvco() : return the VCO or (VCO / 2) frequency for the requested PLL
 * - PLL1 & PLL2 => return VCO / 2 with Fpll_y_ck = FVCO / 2 * (DIVy + 1)
 * - PLL3 & PLL4 => return VCO     with Fpll_y_ck = FVCO / (DIVy + 1)
 * => in all cases Fpll_y_ck = pll_get_fvco() / (DIVy + 1)
 */
static unsigned long stm32mp1_pll_get_fvco(const struct stm32mp1_clk_pll *pll)
{
	unsigned long refclk, fvco;
	uint32_t cfgr1, fracr, divm, divn;
	uintptr_t rcc_base = stm32mp_rcc_base();

	cfgr1 = mmio_read_32(rcc_base + pll->pllxcfgr1);
	fracr = mmio_read_32(rcc_base + pll->pllxfracr);

	divm = (cfgr1 & (RCC_PLLNCFGR1_DIVM_MASK)) >> RCC_PLLNCFGR1_DIVM_SHIFT;
	divn = cfgr1 & RCC_PLLNCFGR1_DIVN_MASK;

	refclk = stm32mp1_pll_get_fref(pll);

	/*
	 * With FRACV :
	 *   Fvco = Fck_ref * ((DIVN + 1) + FRACV / 2^13) / (DIVM + 1)
	 * Without FRACV
	 *   Fvco = Fck_ref * ((DIVN + 1) / (DIVM + 1)
	 */
	if ((fracr & RCC_PLLNFRACR_FRACLE) != 0U) {
		uint32_t fracv = (fracr & RCC_PLLNFRACR_FRACV_MASK) >>
				 RCC_PLLNFRACR_FRACV_SHIFT;
		unsigned long long numerator, denominator;

		numerator = (((unsigned long long)divn + 1U) << 13) + fracv;
		numerator = refclk * numerator;
		denominator = ((unsigned long long)divm + 1U) << 13;
		fvco = (unsigned long)(numerator / denominator);
	} else {
		fvco = (unsigned long)(refclk * (divn + 1U) / (divm + 1U));
	}

	return fvco;
}

static unsigned long stm32mp1_read_pll_freq(enum stm32mp1_pll_id pll_id,
					    enum stm32mp1_div_id div_id)
{
	const struct stm32mp1_clk_pll *pll = pll_ref(pll_id);
	unsigned long dfout;
	uint32_t cfgr2, divy;

	if (div_id >= _DIV_NB) {
		return 0;
	}

	cfgr2 = mmio_read_32(stm32mp_rcc_base() + pll->pllxcfgr2);
	divy = (cfgr2 >> pllncfgr2[div_id]) & RCC_PLLNCFGR2_DIVX_MASK;

	dfout = stm32mp1_pll_get_fvco(pll) / (divy + 1U);


	return dfout;
}

static unsigned long get_clock_rate(int p)
{
	uint32_t reg, clkdiv;
	unsigned long clock = 0;
	uintptr_t rcc_base = stm32mp_rcc_base();

	switch (p) {
	case _CK_MPU:
	/* MPU sub system */
		reg = mmio_read_32(rcc_base + RCC_MPCKSELR);
		switch (reg & RCC_SELR_SRC_MASK) {
		case RCC_MPCKSELR_HSI:
			clock = stm32mp1_clk_get_fixed(_HSI);
			break;
		case RCC_MPCKSELR_HSE:
			clock = stm32mp1_clk_get_fixed(_HSE);
			break;
		case RCC_MPCKSELR_PLL:
			clock = stm32mp1_read_pll_freq(_PLL1, _DIV_P);
			break;
		case RCC_MPCKSELR_PLL_MPUDIV:
			clock = stm32mp1_read_pll_freq(_PLL1, _DIV_P);

			reg = mmio_read_32(rcc_base + RCC_MPCKDIVR);
			clkdiv = reg & RCC_MPUDIV_MASK;
			if (clkdiv != 0U) {
				clock /= stm32mp1_mpu_div[clkdiv];
			}
			break;
		default:
			break;
		}
		break;
	/* AXI sub system */
	case _ACLK:
	case _HCLK2:
	case _HCLK6:
	case _PCLK4:
	case _PCLK5:
		reg = mmio_read_32(rcc_base + RCC_ASSCKSELR);
		switch (reg & RCC_SELR_SRC_MASK) {
		case RCC_ASSCKSELR_HSI:
			clock = stm32mp1_clk_get_fixed(_HSI);
			break;
		case RCC_ASSCKSELR_HSE:
			clock = stm32mp1_clk_get_fixed(_HSE);
			break;
		case RCC_ASSCKSELR_PLL:
			clock = stm32mp1_read_pll_freq(_PLL2, _DIV_P);
			break;
		default:
			break;
		}

		/* System clock divider */
		reg = mmio_read_32(rcc_base + RCC_AXIDIVR);
		clock /= stm32mp1_axi_div[reg & RCC_AXIDIV_MASK];

		switch (p) {
		case _PCLK4:
			reg = mmio_read_32(rcc_base + RCC_APB4DIVR);
			clock >>= stm32mp1_apbx_div[reg & RCC_APBXDIV_MASK];
			break;
		case _PCLK5:
			reg = mmio_read_32(rcc_base + RCC_APB5DIVR);
			clock >>= stm32mp1_apbx_div[reg & RCC_APBXDIV_MASK];
			break;
		default:
			break;
		}
		break;
	/* MCU sub system */
	case _CK_MCU:
	case _PCLK1:
	case _PCLK2:
	case _PCLK3:
		reg = mmio_read_32(rcc_base + RCC_MSSCKSELR);
		switch (reg & RCC_SELR_SRC_MASK) {
		case RCC_MSSCKSELR_HSI:
			clock = stm32mp1_clk_get_fixed(_HSI);
			break;
		case RCC_MSSCKSELR_HSE:
			clock = stm32mp1_clk_get_fixed(_HSE);
			break;
		case RCC_MSSCKSELR_CSI:
			clock = stm32mp1_clk_get_fixed(_CSI);
			break;
		case RCC_MSSCKSELR_PLL:
			clock = stm32mp1_read_pll_freq(_PLL3, _DIV_P);
			break;
		default:
			break;
		}

		/* MCU clock divider */
		reg = mmio_read_32(rcc_base + RCC_MCUDIVR);
		clock >>= stm32mp1_mcu_div[reg & RCC_MCUDIV_MASK];

		switch (p) {
		case _PCLK1:
			reg = mmio_read_32(rcc_base + RCC_APB1DIVR);
			clock >>= stm32mp1_apbx_div[reg & RCC_APBXDIV_MASK];
			break;
		case _PCLK2:
			reg = mmio_read_32(rcc_base + RCC_APB2DIVR);
			clock >>= stm32mp1_apbx_div[reg & RCC_APBXDIV_MASK];
			break;
		case _PCLK3:
			reg = mmio_read_32(rcc_base + RCC_APB3DIVR);
			clock >>= stm32mp1_apbx_div[reg & RCC_APBXDIV_MASK];
			break;
		case _CK_MCU:
		default:
			break;
		}
		break;
	case _CK_PER:
		reg = mmio_read_32(rcc_base + RCC_CPERCKSELR);
		switch (reg & RCC_SELR_SRC_MASK) {
		case RCC_CPERCKSELR_HSI:
			clock = stm32mp1_clk_get_fixed(_HSI);
			break;
		case RCC_CPERCKSELR_HSE:
			clock = stm32mp1_clk_get_fixed(_HSE);
			break;
		case RCC_CPERCKSELR_CSI:
			clock = stm32mp1_clk_get_fixed(_CSI);
			break;
		default:
			break;
		}
		break;
	case _HSI:
	case _HSI_KER:
		clock = stm32mp1_clk_get_fixed(_HSI);
		break;
	case _CSI:
	case _CSI_KER:
		clock = stm32mp1_clk_get_fixed(_CSI);
		break;
	case _HSE:
	case _HSE_KER:
		clock = stm32mp1_clk_get_fixed(_HSE);
		break;
	case _HSE_KER_DIV2:
		clock = stm32mp1_clk_get_fixed(_HSE) >> 1;
		break;
	case _LSI:
		clock = stm32mp1_clk_get_fixed(_LSI);
		break;
	case _LSE:
		clock = stm32mp1_clk_get_fixed(_LSE);
		break;
	/* PLL */
	case _PLL1_P:
		clock = stm32mp1_read_pll_freq(_PLL1, _DIV_P);
		break;
	case _PLL1_Q:
		clock = stm32mp1_read_pll_freq(_PLL1, _DIV_Q);
		break;
	case _PLL1_R:
		clock = stm32mp1_read_pll_freq(_PLL1, _DIV_R);
		break;
	case _PLL2_P:
		clock = stm32mp1_read_pll_freq(_PLL2, _DIV_P);
		break;
	case _PLL2_Q:
		clock = stm32mp1_read_pll_freq(_PLL2, _DIV_Q);
		break;
	case _PLL2_R:
		clock = stm32mp1_read_pll_freq(_PLL2, _DIV_R);
		break;
	case _PLL3_P:
		clock = stm32mp1_read_pll_freq(_PLL3, _DIV_P);
		break;
	case _PLL3_Q:
		clock = stm32mp1_read_pll_freq(_PLL3, _DIV_Q);
		break;
	case _PLL3_R:
		clock = stm32mp1_read_pll_freq(_PLL3, _DIV_R);
		break;
	case _PLL4_P:
		clock = stm32mp1_read_pll_freq(_PLL4, _DIV_P);
		break;
	case _PLL4_Q:
		clock = stm32mp1_read_pll_freq(_PLL4, _DIV_Q);
		break;
	case _PLL4_R:
		clock = stm32mp1_read_pll_freq(_PLL4, _DIV_R);
		break;
	/* Other */
	case _USB_PHY_48:
		clock = USB_PHY_48_MHZ;
		break;
	default:
		break;
	}

	return clock;
}

static void __clk_enable(struct stm32mp1_clk_gate const *gate)
{
	uintptr_t rcc_base = stm32mp_rcc_base();

	if (gate->set_clr != 0U) {
		mmio_write_32(rcc_base + gate->offset, BIT(gate->bit));
	} else {
		stm32mp_mmio_setbits_32_shregs(rcc_base + gate->offset,
					       BIT(gate->bit));
	}

	VERBOSE("Clock %d has been enabled", gate->index);
}

static void __clk_disable(struct stm32mp1_clk_gate const *gate)
{
	uintptr_t rcc_base = stm32mp_rcc_base();

	if (gate->set_clr != 0U) {
		mmio_write_32(rcc_base + gate->offset + RCC_MP_ENCLRR_OFFSET,
			      BIT(gate->bit));
	} else {
		stm32mp_mmio_clrbits_32_shregs(rcc_base + gate->offset,
					       BIT(gate->bit));
	}

	VERBOSE("Clock %d has been disabled", gate->index);
}

static bool __clk_is_enabled(struct stm32mp1_clk_gate const *gate)
{
	uintptr_t rcc_base = stm32mp_rcc_base();

	return mmio_read_32(rcc_base + gate->offset) & BIT(gate->bit);
}

unsigned int stm32mp1_clk_get_refcount(unsigned long id)
{
	int i = stm32mp1_clk_get_gated_id(id);

	if (i < 0) {
		panic();
	}

	return gate_refcounts[i];
}

void __stm32mp1_clk_enable(unsigned long id, bool secure)
{
	const struct stm32mp1_clk_gate *gate;
	int i = stm32mp1_clk_get_gated_id(id);
	unsigned int *refcnt;

	if (i < 0) {
		ERROR("Clock %d can't be enabled\n", (uint32_t)id);
		panic();
	}

	gate = gate_ref(i);
	refcnt = &gate_refcounts[i];

	stm32mp1_clk_lock(&refcount_lock);

	if (stm32mp_incr_shrefcnt(refcnt, secure) != 0) {
		__clk_enable(gate);
	}

	stm32mp1_clk_unlock(&refcount_lock);
}

void __stm32mp1_clk_disable(unsigned long id, bool secure)
{
	const struct stm32mp1_clk_gate *gate;
	int i = stm32mp1_clk_get_gated_id(id);
	unsigned int *refcnt;

	if (i < 0) {
		ERROR("Clock %d can't be disabled\n", (uint32_t)id);
		panic();
	}

	gate = gate_ref(i);
	refcnt = &gate_refcounts[i];

	stm32mp1_clk_lock(&refcount_lock);

	if (stm32mp_decr_shrefcnt(refcnt, secure) != 0) {
		__clk_disable(gate);
	}

	stm32mp1_clk_unlock(&refcount_lock);
}

bool stm32mp1_clk_is_enabled(unsigned long id)
{
	int i = stm32mp1_clk_get_gated_id(id);

	if (i < 0) {
		panic();
	}

	return __clk_is_enabled(gate_ref(i));
}

unsigned long stm32mp_clk_get_rate(unsigned long id)
{
	int p = stm32mp1_clk_get_parent(id);

	if (p < 0) {
		return 0;
	}

	return get_clock_rate(p);
}

static void stm32mp1_ls_osc_set(bool enable, uint32_t offset, uint32_t mask_on)
{
	uint32_t address = stm32mp_rcc_base() + offset;

	if (enable) {
		mmio_setbits_32(address, mask_on);
	} else {
		mmio_clrbits_32(address, mask_on);
	}
}

static void stm32mp1_hs_ocs_set(bool enable, uint32_t mask_on)
{
	uint32_t offset = enable ? RCC_OCENSETR : RCC_OCENCLRR;
	uint32_t address = stm32mp_rcc_base() + offset;

	mmio_write_32(address, mask_on);
}

static int stm32mp1_osc_wait(bool enable, uint32_t offset, uint32_t mask_rdy)
{
	uint64_t start;
	uint32_t mask_test;
	uint32_t address = stm32mp_rcc_base() + offset;

	if (enable) {
		mask_test = mask_rdy;
	} else {
		mask_test = 0;
	}

	start = timeout_start();
	while ((mmio_read_32(address) & mask_rdy) != mask_test) {
		if (timeout_elapsed(start, OSCRDY_TIMEOUT)) {
			ERROR("OSC %x @ %x timeout for enable=%d : 0x%x\n",
			      mask_rdy, address, enable, mmio_read_32(address));
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static void stm32mp1_lse_enable(bool bypass, bool digbyp, uint32_t lsedrv)
{
	uint32_t value;
	uintptr_t rcc_base = stm32mp_rcc_base();

	if (digbyp) {
		mmio_setbits_32(rcc_base + RCC_BDCR, RCC_BDCR_DIGBYP);
	}

	if (bypass || digbyp) {
		mmio_setbits_32(rcc_base + RCC_BDCR, RCC_BDCR_LSEBYP);
	}

	/*
	 * Warning: not recommended to switch directly from "high drive"
	 * to "medium low drive", and vice-versa.
	 */
	value = (mmio_read_32(rcc_base + RCC_BDCR) & RCC_BDCR_LSEDRV_MASK) >>
		RCC_BDCR_LSEDRV_SHIFT;

	while (value != lsedrv) {
		if (value > lsedrv) {
			value--;
		} else {
			value++;
		}

		mmio_clrsetbits_32(rcc_base + RCC_BDCR,
				   RCC_BDCR_LSEDRV_MASK,
				   value << RCC_BDCR_LSEDRV_SHIFT);
	}

	stm32mp1_ls_osc_set(true, RCC_BDCR, RCC_BDCR_LSEON);
}

static void stm32mp1_lse_wait(void)
{
	if (stm32mp1_osc_wait(true, RCC_BDCR, RCC_BDCR_LSERDY) != 0) {
		VERBOSE("%s: failed\n", __func__);
	}
}

static void stm32mp1_lsi_set(bool enable)
{
	stm32mp1_ls_osc_set(enable, RCC_RDLSICR, RCC_RDLSICR_LSION);

	if (stm32mp1_osc_wait(enable, RCC_RDLSICR, RCC_RDLSICR_LSIRDY) !=
	    0) {
		VERBOSE("%s: failed\n", __func__);
	}
}

static void stm32mp1_hse_enable(bool bypass, bool digbyp, bool css)
{
	uintptr_t rcc_base = stm32mp_rcc_base();

	if (digbyp) {
		mmio_write_32(rcc_base + RCC_OCENSETR, RCC_OCENR_DIGBYP);
	}

	if (bypass || digbyp) {
		mmio_write_32(rcc_base + RCC_OCENSETR, RCC_OCENR_HSEBYP);
	}

	stm32mp1_hs_ocs_set(true, RCC_OCENR_HSEON);
	if (stm32mp1_osc_wait(true, RCC_OCRDYR, RCC_OCRDYR_HSERDY) !=
	    0) {
		VERBOSE("%s: failed\n", __func__);
	}

	if (css) {
		mmio_write_32(rcc_base + RCC_OCENSETR, RCC_OCENR_HSECSSON);
	}
}

static void stm32mp1_csi_set(bool enable)
{
	stm32mp1_hs_ocs_set(enable, RCC_OCENR_CSION);
	if (stm32mp1_osc_wait(enable, RCC_OCRDYR, RCC_OCRDYR_CSIRDY) !=
	    0) {
		VERBOSE("%s: failed\n", __func__);
	}
}

static void stm32mp1_hsi_set(bool enable)
{
	stm32mp1_hs_ocs_set(enable, RCC_OCENR_HSION);
	if (stm32mp1_osc_wait(enable, RCC_OCRDYR, RCC_OCRDYR_HSIRDY) !=
	    0) {
		VERBOSE("%s: failed\n", __func__);
	}
}

static int stm32mp1_set_hsidiv(uint8_t hsidiv)
{
	uint64_t start;
	uintptr_t rcc_base = stm32mp_rcc_base();
	uint32_t address = rcc_base + RCC_OCRDYR;

	mmio_clrsetbits_32(rcc_base + RCC_HSICFGR,
			   RCC_HSICFGR_HSIDIV_MASK,
			   RCC_HSICFGR_HSIDIV_MASK & (uint32_t)hsidiv);

	start = timeout_start();
	while ((mmio_read_32(address) & RCC_OCRDYR_HSIDIVRDY) == 0U) {
		if (timeout_elapsed(start, HSIDIV_TIMEOUT)) {
			ERROR("HSIDIV failed @ 0x%x: 0x%x\n",
			      address, mmio_read_32(address));
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static int stm32mp1_hsidiv(unsigned long hsifreq)
{
	uint8_t hsidiv;
	uint32_t hsidivfreq = MAX_HSI_HZ;

	for (hsidiv = 0; hsidiv < 4U; hsidiv++) {
		if (hsidivfreq == hsifreq) {
			break;
		}

		hsidivfreq /= 2U;
	}

	if (hsidiv == 4U) {
		ERROR("Invalid clk-hsi frequency\n");
		return -1;
	}

	if (hsidiv != 0U) {
		return stm32mp1_set_hsidiv(hsidiv);
	}

	return 0;
}

static bool stm32mp1_check_pll_conf(enum stm32mp1_pll_id pll_id,
				    unsigned int clksrc,
				    uint32_t *pllcfg, int plloff)
{
	const struct stm32mp1_clk_pll *pll = pll_ref(pll_id);
	uintptr_t rcc_base = stm32mp_rcc_base();
	uint32_t pllxcr = rcc_base + pll->pllxcr;
	enum stm32mp1_plltype type = pll->plltype;
	uint32_t address = rcc_base + (clksrc >> 4);
	unsigned long refclk;
	uint32_t ifrge = 0U;
	uint32_t src, value, fracv;

	/* Check PLL output */
	if (mmio_read_32(pllxcr) != RCC_PLLNCR_PLLON) {
		return false;
	}

	/* Check current clksrc */
	src = mmio_read_32(address) & RCC_SELR_SRC_MASK;
	if (src != (clksrc & RCC_SELR_SRC_MASK)) {
		return false;
	}

	/* Check Div */
	src = mmio_read_32(rcc_base + pll->rckxselr) &
		RCC_SELR_REFCLK_SRC_MASK;

	refclk = stm32mp1_clk_get_fixed(pll->refclk[src]) /
		 (pllcfg[PLLCFG_M] + 1U);

	if ((refclk < (stm32mp1_pll[type].refclk_min * 1000000U)) ||
	    (refclk > (stm32mp1_pll[type].refclk_max * 1000000U))) {
		return false;
	}

	if ((type == PLL_800) && (refclk >= 8000000U)) {
		ifrge = 1U;
	}

	value = (pllcfg[PLLCFG_N] << RCC_PLLNCFGR1_DIVN_SHIFT) &
		RCC_PLLNCFGR1_DIVN_MASK;
	value |= (pllcfg[PLLCFG_M] << RCC_PLLNCFGR1_DIVM_SHIFT) &
		 RCC_PLLNCFGR1_DIVM_MASK;
	value |= (ifrge << RCC_PLLNCFGR1_IFRGE_SHIFT) &
		 RCC_PLLNCFGR1_IFRGE_MASK;
	if (mmio_read_32(rcc_base + pll->pllxcfgr1) != value) {
		return false;
	}

	/* Fractional configuration */
	fracv = fdt_read_uint32_default(plloff, "frac", 0);

	value = fracv << RCC_PLLNFRACR_FRACV_SHIFT;
	value |= RCC_PLLNFRACR_FRACLE;
	if (mmio_read_32(rcc_base + pll->pllxfracr) != value) {
		return false;
	}

	/* Output config */
	value = (pllcfg[PLLCFG_P] << RCC_PLLNCFGR2_DIVP_SHIFT) &
		RCC_PLLNCFGR2_DIVP_MASK;
	value |= (pllcfg[PLLCFG_Q] << RCC_PLLNCFGR2_DIVQ_SHIFT) &
		 RCC_PLLNCFGR2_DIVQ_MASK;
	value |= (pllcfg[PLLCFG_R] << RCC_PLLNCFGR2_DIVR_SHIFT) &
		 RCC_PLLNCFGR2_DIVR_MASK;
	if (mmio_read_32(rcc_base + pll->pllxcfgr2) != value) {
		return false;
	}

	return true;
}

static void stm32mp1_pll_start(enum stm32mp1_pll_id pll_id)
{
	const struct stm32mp1_clk_pll *pll = pll_ref(pll_id);
	uint32_t pllxcr = stm32mp_rcc_base() + pll->pllxcr;

	mmio_write_32(pllxcr, RCC_PLLNCR_PLLON);
}

static int stm32mp1_pll_output(enum stm32mp1_pll_id pll_id, uint32_t output)
{
	const struct stm32mp1_clk_pll *pll = pll_ref(pll_id);
	uint32_t pllxcr = stm32mp_rcc_base() + pll->pllxcr;
	uint64_t start;

	start = timeout_start();
	/* Wait PLL lock */
	while ((mmio_read_32(pllxcr) & RCC_PLLNCR_PLLRDY) == 0U) {
		if (timeout_elapsed(start, PLLRDY_TIMEOUT)) {
			ERROR("PLL%d start failed @ 0x%x: 0x%x\n",
			      pll_id, pllxcr, mmio_read_32(pllxcr));
			return -ETIMEDOUT;
		}
	}

	/* Start the requested output */
	mmio_setbits_32(pllxcr, output << RCC_PLLNCR_DIVEN_SHIFT);

	return 0;
}

static int stm32mp1_pll_stop(enum stm32mp1_pll_id pll_id)
{
	const struct stm32mp1_clk_pll *pll = pll_ref(pll_id);
	uint32_t pllxcr = stm32mp_rcc_base() + pll->pllxcr;
	uint64_t start;

	/* Stop all output */
	mmio_clrbits_32(pllxcr, RCC_PLLNCR_DIVPEN | RCC_PLLNCR_DIVQEN |
			RCC_PLLNCR_DIVREN);

	/* Stop PLL */
	mmio_clrbits_32(pllxcr, RCC_PLLNCR_PLLON);

	start = timeout_start();
	/* Wait PLL stopped */
	while ((mmio_read_32(pllxcr) & RCC_PLLNCR_PLLRDY) != 0U) {
		if (timeout_elapsed(start, PLLRDY_TIMEOUT)) {
			ERROR("PLL%d stop failed @ 0x%x: 0x%x\n",
			      pll_id, pllxcr, mmio_read_32(pllxcr));
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static void stm32mp1_pll_config_output(enum stm32mp1_pll_id pll_id,
				       uint32_t *pllcfg)
{
	const struct stm32mp1_clk_pll *pll = pll_ref(pll_id);
	uintptr_t rcc_base = stm32mp_rcc_base();
	uint32_t value;

	value = (pllcfg[PLLCFG_P] << RCC_PLLNCFGR2_DIVP_SHIFT) &
		RCC_PLLNCFGR2_DIVP_MASK;
	value |= (pllcfg[PLLCFG_Q] << RCC_PLLNCFGR2_DIVQ_SHIFT) &
		 RCC_PLLNCFGR2_DIVQ_MASK;
	value |= (pllcfg[PLLCFG_R] << RCC_PLLNCFGR2_DIVR_SHIFT) &
		 RCC_PLLNCFGR2_DIVR_MASK;
	mmio_write_32(rcc_base + pll->pllxcfgr2, value);
}

static int stm32mp1_pll_config(enum stm32mp1_pll_id pll_id,
			       uint32_t *pllcfg, uint32_t fracv)
{
	const struct stm32mp1_clk_pll *pll = pll_ref(pll_id);
	uint32_t rcc_base = stm32mp_rcc_base();
	enum stm32mp1_plltype type = pll->plltype;
	unsigned long refclk;
	uint32_t ifrge = 0;
	uint32_t src, value;

	src = mmio_read_32(rcc_base + pll->rckxselr) &
		RCC_SELR_REFCLK_SRC_MASK;

	refclk = stm32mp1_clk_get_fixed(pll->refclk[src]) /
		 (pllcfg[PLLCFG_M] + 1U);

	if ((refclk < (stm32mp1_pll[type].refclk_min * 1000000U)) ||
	    (refclk > (stm32mp1_pll[type].refclk_max * 1000000U))) {
		return -EINVAL;
	}

	if ((type == PLL_800) && (refclk >= 8000000U)) {
		ifrge = 1U;
	}

	value = (pllcfg[PLLCFG_N] << RCC_PLLNCFGR1_DIVN_SHIFT) &
		RCC_PLLNCFGR1_DIVN_MASK;
	value |= (pllcfg[PLLCFG_M] << RCC_PLLNCFGR1_DIVM_SHIFT) &
		 RCC_PLLNCFGR1_DIVM_MASK;
	value |= (ifrge << RCC_PLLNCFGR1_IFRGE_SHIFT) &
		 RCC_PLLNCFGR1_IFRGE_MASK;
	mmio_write_32(rcc_base + pll->pllxcfgr1, value);

	/* Fractional configuration */
	value = 0;
	mmio_write_32(rcc_base + pll->pllxfracr, value);

	value = fracv << RCC_PLLNFRACR_FRACV_SHIFT;
	mmio_write_32(rcc_base + pll->pllxfracr, value);

	value |= RCC_PLLNFRACR_FRACLE;
	mmio_write_32(rcc_base + pll->pllxfracr, value);

	stm32mp1_pll_config_output(pll_id, pllcfg);

	return 0;
}

static void stm32mp1_pll_csg(enum stm32mp1_pll_id pll_id, uint32_t *csg)
{
	const struct stm32mp1_clk_pll *pll = pll_ref(pll_id);
	uint32_t pllxcsg = 0;

	pllxcsg |= (csg[PLLCSG_MOD_PER] << RCC_PLLNCSGR_MOD_PER_SHIFT) &
		    RCC_PLLNCSGR_MOD_PER_MASK;

	pllxcsg |= (csg[PLLCSG_INC_STEP] << RCC_PLLNCSGR_INC_STEP_SHIFT) &
		    RCC_PLLNCSGR_INC_STEP_MASK;

	pllxcsg |= (csg[PLLCSG_SSCG_MODE] << RCC_PLLNCSGR_SSCG_MODE_SHIFT) &
		    RCC_PLLNCSGR_SSCG_MODE_MASK;

	mmio_write_32(stm32mp_rcc_base() + pll->pllxcsgr, pllxcsg);
}

static int stm32mp1_set_clksrc(unsigned int clksrc)
{
	uint32_t address = stm32mp_rcc_base() + (clksrc >> 4);
	uint64_t start;

	mmio_clrsetbits_32(address, RCC_SELR_SRC_MASK,
			   clksrc & RCC_SELR_SRC_MASK);

	start = timeout_start();
	while ((mmio_read_32(address) & RCC_SELR_SRCRDY) == 0U) {
		if (timeout_elapsed(start, CLKSRC_TIMEOUT)) {
			ERROR("CLKSRC %x start failed @ 0x%x: 0x%x\n",
			      clksrc, address, mmio_read_32(address));
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static int stm32mp1_set_clkdiv(unsigned int clkdiv, uint32_t address)
{
	uint64_t start;

	mmio_clrsetbits_32(address, RCC_DIVR_DIV_MASK,
			   clkdiv & RCC_DIVR_DIV_MASK);

	start = timeout_start();
	while ((mmio_read_32(address) & RCC_DIVR_DIVRDY) == 0U) {
		if (timeout_elapsed(start, CLKDIV_TIMEOUT)) {
			ERROR("CLKDIV %x start failed @ 0x%x: 0x%x\n",
			      clkdiv, address, mmio_read_32(address));
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static void stm32mp1_mco_csg(uint32_t clksrc, uint32_t clkdiv)
{
	uint32_t address = stm32mp_rcc_base() + (clksrc >> 4);

	/*
	 * Binding clksrc :
	 *      bit15-4 offset
	 *      bit3:   disable
	 *      bit2-0: MCOSEL[2:0]
	 */
	if ((clksrc & 0x8U) != 0U) {
		mmio_clrbits_32(address, RCC_MCOCFG_MCOON);
	} else {
		mmio_clrsetbits_32(address,
				   RCC_MCOCFG_MCOSRC_MASK,
				   clksrc & RCC_MCOCFG_MCOSRC_MASK);
		mmio_clrsetbits_32(address,
				   RCC_MCOCFG_MCODIV_MASK,
				   clkdiv << RCC_MCOCFG_MCODIV_SHIFT);
		mmio_setbits_32(address, RCC_MCOCFG_MCOON);
	}
}

static void stm32mp1_set_rtcsrc(unsigned int clksrc, bool lse_css)
{
	uint32_t address = stm32mp_rcc_base() + RCC_BDCR;

	if (((mmio_read_32(address) & RCC_BDCR_RTCCKEN) == 0U) ||
	    (clksrc != (uint32_t)CLK_RTC_DISABLED)) {
		mmio_clrsetbits_32(address,
				   RCC_BDCR_RTCSRC_MASK,
				   clksrc << RCC_BDCR_RTCSRC_SHIFT);

		mmio_setbits_32(address, RCC_BDCR_RTCCKEN);
	}

	if (lse_css) {
		mmio_setbits_32(address, RCC_BDCR_LSECSSON);
	}
}

static void stm32mp1_stgen_config(void)
{
	uintptr_t stgen;
	uint32_t cntfid0;
	unsigned long rate;
	unsigned long long counter;

	stgen = fdt_get_stgen_base();
	cntfid0 = mmio_read_32(stgen + CNTFID_OFF);
	rate = get_clock_rate(stm32mp1_clk_get_parent(STGEN_K));

	if (cntfid0 == rate) {
		return;
	}

	mmio_clrbits_32(stgen + CNTCR_OFF, CNTCR_EN);
	counter = (unsigned long long)mmio_read_32(stgen + CNTCVL_OFF);
	counter |= ((unsigned long long)mmio_read_32(stgen + CNTCVU_OFF)) << 32;
	counter = (counter * rate / cntfid0);

	mmio_write_32(stgen + CNTCVL_OFF, (uint32_t)counter);
	mmio_write_32(stgen + CNTCVU_OFF, (uint32_t)(counter >> 32));
	mmio_write_32(stgen + CNTFID_OFF, rate);
	mmio_setbits_32(stgen + CNTCR_OFF, CNTCR_EN);

	write_cntfrq((u_register_t)rate);

	/* Need to update timer with new frequency */
	generic_delay_timer_init();
}

unsigned long stm32mp_clk_timer_get_rate(unsigned long id)
{
	unsigned long parent_rate;
	uint32_t prescaler, timpre;
	uintptr_t rcc_base = stm32mp_rcc_base();

	parent_rate = stm32mp_clk_get_rate(id);

	if (id < TIM1_K) {
		prescaler = mmio_read_32(rcc_base + RCC_APB1DIVR) &
			    RCC_APBXDIV_MASK;
		timpre = mmio_read_32(rcc_base + RCC_TIMG1PRER) &
			 RCC_TIMGXPRER_TIMGXPRE;
	} else {
		prescaler = mmio_read_32(rcc_base + RCC_APB2DIVR) &
			    RCC_APBXDIV_MASK;
		timpre = mmio_read_32(rcc_base + RCC_TIMG2PRER) &
			 RCC_TIMGXPRER_TIMGXPRE;
	}

	if (!prescaler) {
		return parent_rate;
	}

	return parent_rate * (timpre + 1) * 2;
}

void stm32mp1_stgen_increment(unsigned long long offset_in_ms)
{
	uintptr_t stgen;
	unsigned long long cnt;

	stgen = fdt_get_stgen_base();

	cnt = ((unsigned long long)mmio_read_32(stgen + CNTCVU_OFF) << 32) |
		mmio_read_32(stgen + CNTCVL_OFF);

	cnt += (offset_in_ms * mmio_read_32(stgen + CNTFID_OFF)) / 1000U;

	mmio_clrbits_32(stgen + CNTCR_OFF, CNTCR_EN);
	mmio_write_32(stgen + CNTCVL_OFF, (uint32_t)cnt);
	mmio_write_32(stgen + CNTCVU_OFF, (uint32_t)(cnt >> 32));
	mmio_setbits_32(stgen + CNTCR_OFF, CNTCR_EN);
}

/*******************************************************************************
 * This function determines the number of needed RTC calendar read operations
 * to get consistent values (1 or 2 depending on clock frequencies).
 * If APB1 frequency is lower than 7 times the RTC one, the software has to
 * read the calendar time and date registers twice.
 * Returns true if read twice is needed, false else.
 ******************************************************************************/
bool stm32mp1_rtc_get_read_twice(void)
{
	unsigned long apb1_freq;
	uint32_t rtc_freq;
	uint32_t apb1_div;
	uintptr_t rcc_base = stm32mp_rcc_base();

	switch ((mmio_read_32(rcc_base + RCC_BDCR) &
		 RCC_BDCR_RTCSRC_MASK) >> RCC_BDCR_RTCSRC_SHIFT) {
	case 1:
		rtc_freq = stm32mp_clk_get_rate(CK_LSE);
		break;
	case 2:
		rtc_freq = stm32mp_clk_get_rate(CK_LSI);
		break;
	case 3:
		rtc_freq = stm32mp_clk_get_rate(CK_HSE);
		rtc_freq /= (mmio_read_32(rcc_base + RCC_RTCDIVR) &
			     RCC_DIVR_DIV_MASK) + 1U;
		break;
	default:
		panic();
	}

	apb1_div = mmio_read_32(rcc_base + RCC_APB1DIVR) & RCC_APBXDIV_MASK;
	apb1_freq = stm32mp_clk_get_rate(CK_MCU) >> apb1_div;

	return apb1_freq < (rtc_freq * 7U);
}

static void stm32mp1_pkcs_config(uint32_t pkcs)
{
	uint32_t address = stm32mp_rcc_base() + ((pkcs >> 4) & 0xFFFU);
	uint32_t value = pkcs & 0xFU;
	uint32_t mask = 0xFU;

	if ((pkcs & BIT(31)) != 0U) {
		mask <<= 4;
		value <<= 4;
	}

	mmio_clrsetbits_32(address, mask, value);
}

int stm32mp1_clk_init(void)
{
	uint32_t rcc_base = stm32mp_rcc_base();
	unsigned int clksrc[CLKSRC_NB];
	unsigned int clkdiv[CLKDIV_NB];
	unsigned int pllcfg[_PLL_NB][PLLCFG_NB];
	int plloff[_PLL_NB];
	int ret, len;
	enum stm32mp1_pll_id i;
	bool lse_css = false;
	bool pll3_preserve = false;
	bool pll4_preserve = false;
	bool pll4_bootrom = false;
	const fdt32_t *pkcs_cell;
	int stgen_p = stm32mp1_clk_get_parent((int)STGEN_K);
	int usbphy_p = stm32mp1_clk_get_parent((int)USBPHY_K);

	/* Check status field to disable security */
	if (!fdt_get_rcc_secure_status()) {
		mmio_write_32(rcc_base + RCC_TZCR, 0);
	}

	ret = fdt_rcc_read_uint32_array("st,clksrc", clksrc,
					(uint32_t)CLKSRC_NB);
	if (ret < 0) {
		return -FDT_ERR_NOTFOUND;
	}

	ret = fdt_rcc_read_uint32_array("st,clkdiv", clkdiv,
					(uint32_t)CLKDIV_NB);
	if (ret < 0) {
		return -FDT_ERR_NOTFOUND;
	}

	for (i = (enum stm32mp1_pll_id)0; i < _PLL_NB; i++) {
		char name[12];

		snprintf(name, sizeof(name), "st,pll@%d", i);
		plloff[i] = fdt_rcc_subnode_offset(name);

		if (!fdt_check_node(plloff[i])) {
			continue;
		}

		ret = fdt_read_uint32_array(plloff[i], "cfg",
					    pllcfg[i], (int)PLLCFG_NB);
		if (ret < 0) {
			return -FDT_ERR_NOTFOUND;
		}
	}

	stm32mp1_mco_csg(clksrc[CLKSRC_MCO1], clkdiv[CLKDIV_MCO1]);
	stm32mp1_mco_csg(clksrc[CLKSRC_MCO2], clkdiv[CLKDIV_MCO2]);

	/*
	 * Switch ON oscillator found in device-tree.
	 * Note: HSI already ON after BootROM stage.
	 */
	if (stm32mp1_osc[_LSI] != 0U) {
		stm32mp1_lsi_set(true);
	}
	if (stm32mp1_osc[_LSE] != 0U) {
		bool bypass, digbyp;
		uint32_t lsedrv;

		bypass = fdt_osc_read_bool(_LSE, "st,bypass");
		digbyp = fdt_osc_read_bool(_LSE, "st,digbypass");
		lse_css = fdt_osc_read_bool(_LSE, "st,css");
		lsedrv = fdt_osc_read_uint32_default(_LSE, "st,drive",
						     LSEDRV_MEDIUM_HIGH);
		stm32mp1_lse_enable(bypass, digbyp, lsedrv);
	}
	if (stm32mp1_osc[_HSE] != 0U) {
		bool bypass, digbyp, css;

		bypass = fdt_osc_read_bool(_HSE, "st,bypass");
		digbyp = fdt_osc_read_bool(_HSE, "st,digbypass");
		css = fdt_osc_read_bool(_HSE, "st,css");
		stm32mp1_hse_enable(bypass, digbyp, css);
	}
	/*
	 * CSI is mandatory for automatic I/O compensation (SYSCFG_CMPCR)
	 * => switch on CSI even if node is not present in device tree
	 */
	stm32mp1_csi_set(true);

	/* Come back to HSI */
	ret = stm32mp1_set_clksrc(CLK_MPU_HSI);
	if (ret != 0) {
		return ret;
	}
	ret = stm32mp1_set_clksrc(CLK_AXI_HSI);
	if (ret != 0) {
		return ret;
	}
	ret = stm32mp1_set_clksrc(CLK_MCU_HSI);
	if (ret != 0) {
		return ret;
	}

	if ((mmio_read_32(rcc_base + RCC_MP_RSTSCLRR) &
	     RCC_MP_RSTSCLRR_MPUP0RSTF) != 0) {
		pll3_preserve = stm32mp1_check_pll_conf(_PLL3,
							clksrc[CLKSRC_PLL3],
							pllcfg[_PLL3],
							plloff[_PLL3]);
		pll4_preserve = stm32mp1_check_pll_conf(_PLL4,
							clksrc[CLKSRC_PLL4],
							pllcfg[_PLL4],
							plloff[_PLL4]);
	}
	/* Don't initialize PLL4, when used by BOOTROM */
	if ((get_boot_device() == BOOT_DEVICE_USB) &&
	    ((stgen_p == (int)_PLL4_R) || (usbphy_p == (int)_PLL4_R))) {
		pll4_bootrom = true;
		pll4_preserve = true;
	}

	for (i = (enum stm32mp1_pll_id)0; i < _PLL_NB; i++) {
		if (((i == _PLL3) && pll3_preserve) ||
		    ((i == _PLL4) && pll4_preserve)) {
			continue;
		}

		ret = stm32mp1_pll_stop(i);
		if (ret != 0) {
			return ret;
		}
	}

	/* Configure HSIDIV */
	if (stm32mp1_osc[_HSI] != 0U) {
		ret = stm32mp1_hsidiv(stm32mp1_osc[_HSI]);
		if (ret != 0) {
			return ret;
		}
		stm32mp1_stgen_config();
	}

	/* Select DIV */
	/* No ready bit when MPUSRC != CLK_MPU_PLL1P_DIV, MPUDIV is disabled */
	mmio_write_32(rcc_base + RCC_MPCKDIVR,
		      clkdiv[CLKDIV_MPU] & RCC_DIVR_DIV_MASK);
	ret = stm32mp1_set_clkdiv(clkdiv[CLKDIV_AXI], rcc_base + RCC_AXIDIVR);
	if (ret != 0) {
		return ret;
	}
	ret = stm32mp1_set_clkdiv(clkdiv[CLKDIV_APB4], rcc_base + RCC_APB4DIVR);
	if (ret != 0) {
		return ret;
	}
	ret = stm32mp1_set_clkdiv(clkdiv[CLKDIV_APB5], rcc_base + RCC_APB5DIVR);
	if (ret != 0) {
		return ret;
	}
	ret = stm32mp1_set_clkdiv(clkdiv[CLKDIV_MCU], rcc_base + RCC_MCUDIVR);
	if (ret != 0) {
		return ret;
	}
	ret = stm32mp1_set_clkdiv(clkdiv[CLKDIV_APB1], rcc_base + RCC_APB1DIVR);
	if (ret != 0) {
		return ret;
	}
	ret = stm32mp1_set_clkdiv(clkdiv[CLKDIV_APB2], rcc_base + RCC_APB2DIVR);
	if (ret != 0) {
		return ret;
	}
	ret = stm32mp1_set_clkdiv(clkdiv[CLKDIV_APB3], rcc_base + RCC_APB3DIVR);
	if (ret != 0) {
		return ret;
	}

	/* No ready bit for RTC */
	mmio_write_32(rcc_base + RCC_RTCDIVR,
		      clkdiv[CLKDIV_RTC] & RCC_DIVR_DIV_MASK);

	/* Configure PLLs source */
	ret = stm32mp1_set_clksrc(clksrc[CLKSRC_PLL12]);
	if (ret != 0) {
		return ret;
	}

	if (!pll3_preserve) {
		ret = stm32mp1_set_clksrc(clksrc[CLKSRC_PLL3]);
		if (ret != 0) {
			return ret;
		}
	}

	if (!pll4_preserve) {
		ret = stm32mp1_set_clksrc(clksrc[CLKSRC_PLL4]);
		if (ret != 0) {
			return ret;
		}
	}

	/* Configure and start PLLs */
	for (i = (enum stm32mp1_pll_id)0; i < _PLL_NB; i++) {
		uint32_t fracv;
		uint32_t csg[PLLCSG_NB];

		if (((i == _PLL3) && pll3_preserve) ||
		    ((i == _PLL4) && pll4_preserve && !pll4_bootrom)) {
			continue;
		}

		if (!fdt_check_node(plloff[i])) {
			continue;
		}

		if ((i == _PLL4) && pll4_bootrom) {
			/* Set output divider if not done by the Bootrom */
			stm32mp1_pll_config_output(i, pllcfg[i]);
			continue;
		}

		fracv = fdt_read_uint32_default(plloff[i], "frac", 0);

		ret = stm32mp1_pll_config(i, pllcfg[i], fracv);
		if (ret != 0) {
			return ret;
		}
		ret = fdt_read_uint32_array(plloff[i], "csg", csg,
					    (uint32_t)PLLCSG_NB);
		if (ret == 0) {
			stm32mp1_pll_csg(i, csg);
		} else if (ret != -FDT_ERR_NOTFOUND) {
			return ret;
		}

		stm32mp1_pll_start(i);
	}
	/* Wait and start PLLs ouptut when ready */
	for (i = (enum stm32mp1_pll_id)0; i < _PLL_NB; i++) {
		if (!fdt_check_node(plloff[i])) {
			continue;
		}

		ret = stm32mp1_pll_output(i, pllcfg[i][PLLCFG_O]);
		if (ret != 0) {
			return ret;
		}
	}
	/* Wait LSE ready before to use it */
	if (stm32mp1_osc[_LSE] != 0U) {
		stm32mp1_lse_wait();
	}

	/* Configure with expected clock source */
	ret = stm32mp1_set_clksrc(clksrc[CLKSRC_MPU]);
	if (ret != 0) {
		return ret;
	}
	ret = stm32mp1_set_clksrc(clksrc[CLKSRC_AXI]);
	if (ret != 0) {
		return ret;
	}
	ret = stm32mp1_set_clksrc(clksrc[CLKSRC_MCU]);
	if (ret != 0) {
		return ret;
	}
	stm32mp1_set_rtcsrc(clksrc[CLKSRC_RTC], lse_css);

	/* Configure PKCK */
	pkcs_cell = fdt_rcc_read_prop("st,pkcs", &len);
	if (pkcs_cell != NULL) {
		bool ckper_disabled = false;
		uint32_t j;
		uint32_t usbreg_bootrom = 0U;

		if (pll4_bootrom) {
			usbreg_bootrom = mmio_read_32(rcc_base + RCC_USBCKSELR);
		}

		for (j = 0; j < ((uint32_t)len / sizeof(uint32_t)); j++) {
			uint32_t pkcs = fdt32_to_cpu(pkcs_cell[j]);

			if (pkcs == (uint32_t)CLK_CKPER_DISABLED) {
				ckper_disabled = true;
				continue;
			}
			stm32mp1_pkcs_config(pkcs);
		}

		/*
		 * CKPER is source for some peripheral clocks
		 * (FMC-NAND / QPSI-NOR) and switching source is allowed
		 * only if previous clock is still ON
		 * => deactivated CKPER only after switching clock
		 */
		if (ckper_disabled) {
			stm32mp1_pkcs_config(CLK_CKPER_DISABLED);
		}

		if (pll4_bootrom) {
			uint32_t usbreg_value, usbreg_mask;
			const struct stm32mp1_clk_sel *sel;

			sel = clk_sel_ref(_USBPHY_SEL);
			usbreg_mask = (uint32_t)sel->msk << sel->src;
			sel = clk_sel_ref(_USBO_SEL);
			usbreg_mask |= (uint32_t)sel->msk << sel->src;

			usbreg_value = mmio_read_32(rcc_base + RCC_USBCKSELR) &
				       usbreg_mask;
			usbreg_bootrom &= usbreg_mask;
			if (usbreg_bootrom != usbreg_value) {
				VERBOSE("forbidden new USB clk path\n");
				VERBOSE("vs bootrom on USB boot\n");
				return -FDT_ERR_BADVALUE;
			}
		}
	}

	/* Switch OFF HSI if not found in device-tree */
	if (stm32mp1_osc[_HSI] == 0U) {
		stm32mp1_hsi_set(false);
	}
	stm32mp1_stgen_config();

	/* Software Self-Refresh mode (SSR) during DDR initilialization */
	mmio_clrsetbits_32(rcc_base + RCC_DDRITFCR,
			   RCC_DDRITFCR_DDRCKMOD_MASK,
			   RCC_DDRITFCR_DDRCKMOD_SSR <<
			   RCC_DDRITFCR_DDRCKMOD_SHIFT);

	return 0;
}

static void stm32mp1_osc_clk_init(const char *name,
				  enum stm32mp_osc_id index)
{
	uint32_t frequency;

	if (fdt_osc_read_freq(name, &frequency) == 0) {
		stm32mp1_osc[index] = frequency;
	}
}

#if defined(IMAGE_BL32)
/*
 * HSI Calibration part
 */
static void hsi_set_trim(unsigned int cal)
{
	int clk_trim = (int)cal - (int)stm32mp1_clk_cal_hsi.cal_ref;
	uint32_t trim = ((uint32_t)clk_trim << RCC_HSICFGR_HSITRIM_SHIFT) &
			RCC_HSICFGR_HSITRIM_MASK;

	mmio_clrsetbits_32(stm32mp_rcc_base() + RCC_HSICFGR,
			   RCC_HSICFGR_HSITRIM_MASK, trim);
}

static unsigned int hsi_get_trimed_cal(void)
{
	uint32_t utrim = (mmio_read_32(stm32mp_rcc_base() + RCC_HSICFGR) &
			  RCC_HSICFGR_HSITRIM_MASK) >>
			 RCC_HSICFGR_HSITRIM_SHIFT;
	int trim = (int)utrim - stm32mp1_clk_cal_hsi.trim_max;

	if (trim + (int)stm32mp1_clk_cal_hsi.cal_ref < 0) {
		return 0;
	}

	return stm32mp1_clk_cal_hsi.cal_ref + trim;
}

static void csi_set_trim(unsigned int cal)
{
	int clk_trim = (int)cal - (int)stm32mp1_clk_cal_csi.cal_ref +
		       stm32mp1_clk_cal_csi.trim_max + 1;
	uint32_t trim = ((uint32_t)clk_trim << RCC_CSICFGR_CSITRIM_SHIFT) &
			RCC_CSICFGR_CSITRIM_MASK;

	mmio_clrsetbits_32(stm32mp_rcc_base() + RCC_CSICFGR,
			   RCC_CSICFGR_CSITRIM_MASK, trim);
}

static unsigned int csi_get_trimed_cal(void)
{
	uint32_t trim = (mmio_read_32(stm32mp_rcc_base() + RCC_CSICFGR) &
			 RCC_CSICFGR_CSITRIM_MASK) >>
			RCC_CSICFGR_CSITRIM_SHIFT;

	return (int)trim - stm32mp1_clk_cal_csi.trim_max +
		(int)stm32mp1_clk_cal_csi.cal_ref - 1;
}

static unsigned int trim_increase(struct stm32mp1_clk_cal *clk_cal,
				  unsigned int cal)
{
	struct stm32mp1_trim_boundary_t *boundary;
	unsigned int new_cal;
	int i;

	/* By default: last calibration value */
	new_cal = cal;

	/* Start from Lowest cal value */
	for (i = (int)clk_cal->boundary_max - 1; i >= 0; i--) {
		boundary = &clk_cal->boundary[i];

		if (cal < boundary->x2) {
			new_cal = boundary->x2;
			break;
		}

		if ((cal >= boundary->x2) && (cal < boundary->x1)) {
			new_cal = cal + 1;
			break;
		}
	}

	return new_cal;
}

static unsigned int trim_decrease(struct stm32mp1_clk_cal *clk_cal,
				  unsigned int cal)
{
	struct stm32mp1_trim_boundary_t *boundary;
	unsigned int new_cal;
	unsigned int i;

	/* By default: last calibration value */
	new_cal = cal;

	/* Start from Highest cal value */
	for (i = 0; i < clk_cal->boundary_max; i++) {
		boundary = &clk_cal->boundary[i];

		if (cal > boundary->x1) {
			new_cal = boundary->x1;
			break;
		}

		if ((cal > boundary->x2) && (cal <= boundary->x1)) {
			new_cal = cal - 1;
			break;
		}
	}

	return new_cal;
}

static void stm32mp1_rcc_calibration(struct stm32mp1_clk_cal *clk_cal)
{
	unsigned long freq = clk_cal->get_freq();
	unsigned long min = clk_cal->ref_freq -
		((clk_cal->ref_freq * clk_cal->freq_margin) / 100);
	unsigned long max = clk_cal->ref_freq +
		((clk_cal->ref_freq * clk_cal->freq_margin) / 100);
	unsigned int nb_retries = CAL_MAX_RETRY;
	int cal = clk_cal->get_trim();

	VERBOSE("Freq is %lu min %lu max %lu\n", freq, min, max);

	while (((freq < min) || (freq > max)) && (nb_retries != 0U)) {

		if (freq < min) {
			cal = trim_increase(clk_cal, cal);
		} else {
			cal = trim_decrease(clk_cal, cal);
		}

		clk_cal->set_trim(cal);

		freq = clk_cal->get_freq();

		nb_retries--;
	}

	if (nb_retries == 0U) {
		ERROR("Calibration Failed\n");
		panic();
	}
}

static void save_trim(struct stm32mp1_clk_cal *clk_cal,
		      unsigned int i, unsigned int x1, unsigned int x2)
{
	clk_cal->boundary[i].x1 = x1;
	clk_cal->boundary[i].x2 = x2;
}

static int trim_find_prev_boundary(struct stm32mp1_clk_cal *clk_cal,
				   unsigned int x1)
{
	unsigned int x = x1;
	unsigned long freq;

	clk_cal->set_trim(x1 + 1);
	freq = clk_cal->get_freq();

	while (x >= (clk_cal->cal_ref + clk_cal->trim_min)) {
		x--;
		clk_cal->set_trim(x);

		if (clk_cal->get_freq() <= freq) {
			break;
		}
	};

	return x;
}

static void trim_table_init(struct stm32mp1_clk_cal *clk_cal)
{
	uint16_t *trim_fbv = clk_cal->fbv;
	unsigned int min;
	unsigned int max;
	int boundary = 0;
	int i = 0;

	max = clk_cal->cal_ref + clk_cal->trim_max;
	min = clk_cal->cal_ref + clk_cal->trim_min;

	while (trim_fbv[i]) {
		unsigned int x;
		unsigned int x1 = trim_fbv[i];
		unsigned int x2 = trim_fbv[i + 1];

		if ((max <= x2) || (min >= x1)) {
			i++;
			if (boundary != 0) {
				goto out;
			}
			continue;
		}

		/* Take forbiden value + 1 */
		x2 = x2 + 1;
		if (x2 < min) {
			x2 = min;
		}

		if (boundary == 0) {
			/* Save first boundary */
			save_trim(clk_cal, boundary, max, x2);
			boundary++;
			i++;
			continue;
		}

		x = trim_find_prev_boundary(clk_cal, x1);
		/* Save boundary values */
		save_trim(clk_cal, boundary, x - 1, x2);
		boundary++;
		i++;
	};
out:
	clk_cal->boundary_max = boundary;
}

bool stm32mp1_rcc_get_wakeup(void)
{
	return rcc_wakeup;
}

void stm32mp1_rcc_set_wakeup(bool state)
{
	rcc_wakeup = state;
}

void stm32mp1_rcc_it_handler(uint32_t id)
{
	uintptr_t rcc_base = stm32mp_rcc_base();

	switch (id) {
	case STM32MP1_IRQ_RCC_WAKEUP:
		plat_ic_set_priority_mask(GIC_HIGHEST_NS_PRIORITY);
		mmio_setbits_32(rcc_base + RCC_MP_CIFR, RCC_MP_CIFR_WKUPF);
		stm32mp1_rcc_set_wakeup(true);
		return;

	case STM32MP1_IRQ_MCU_SEV:
		stm32mp1_rcc_set_wakeup(false);
		if ((mmio_read_32(EXTI_BASE + EXTI_RPR3) &
		     EXTI_RPR3_RPIF65) != 0U) {
			mmio_setbits_32(EXTI_BASE + EXTI_RPR3,
					EXTI_RPR3_RPIF65);
		}

		if ((mmio_read_32(EXTI_BASE + EXTI_FPR3) &
		     EXTI_FPR3_FPIF65) != 0U) {
			mmio_setbits_32(EXTI_BASE + EXTI_FPR3,
					EXTI_FPR3_FPIF65);
		}

		break;

	case ARM_IRQ_SEC_PHY_TIMER:
	default:
		break;
	}

	if (stm32mp1_clk_cal_hsi.ref_freq != 0U) {
		stm32mp1_rcc_calibration(&stm32mp1_clk_cal_hsi);
	}

	if (stm32mp1_clk_cal_csi.ref_freq != 0U) {
		stm32mp1_rcc_calibration(&stm32mp1_clk_cal_csi);
	}

	if (timer_val != 0U) {
		write_cntptval(timer_val);
	}
}

int stm32mp1_rcc_start_hsi_cal(void)
{
	if (stm32mp1_clk_cal_hsi.ref_freq == 0U) {
		return -ENOENT;
	}

	stm32mp1_rcc_calibration(&stm32mp1_clk_cal_hsi);
	return 0;
}

int stm32mp1_rcc_start_csi_cal(void)
{
	if (stm32mp1_clk_cal_csi.ref_freq == 0U) {
		return -ENOENT;
	}

	stm32mp1_rcc_calibration(&stm32mp1_clk_cal_csi);
	return 0;
}

static void init_hsi_cal(void)
{
	int len;

	if (fdt_rcc_read_prop("st,hsi-cal", &len) == NULL) {
		return;
	}

	stm32_timer_freq_func(&stm32mp1_clk_cal_hsi.get_freq, HSI_CAL);
	assert(stm32mp1_clk_cal_hsi.get_freq);

	stm32mp1_clk_cal_hsi.ref_freq = stm32mp_clk_get_rate(CK_HSI);

	/* Read initial value */
	stm32mp1_clk_cal_hsi.cal_ref =
		((mmio_read_32(stm32mp_rcc_base() + RCC_HSICFGR)
		  & RCC_HSICFGR_HSICAL_MASK) >> RCC_HSICFGR_HSICAL_SHIFT);

	trim_table_init(&stm32mp1_clk_cal_hsi);

	stm32mp1_clk_cal_hsi.set_trim(stm32mp1_clk_cal_hsi.cal_ref);

	stm32mp1_rcc_calibration(&stm32mp1_clk_cal_hsi);
}

static void init_csi_cal(void)
{
	int len;

	if (fdt_rcc_read_prop("st,csi-cal", &len) == NULL) {
		return;
	}

	stm32_timer_freq_func(&stm32mp1_clk_cal_csi.get_freq, CSI_CAL);
	assert(stm32mp1_clk_cal_csi.get_freq);

	stm32mp1_clk_cal_csi.ref_freq = stm32mp_clk_get_rate(CK_CSI);

	/* Read initial value */
	stm32mp1_clk_cal_csi.cal_ref =
		((mmio_read_32(stm32mp_rcc_base() + RCC_CSICFGR) &
		  RCC_CSICFGR_CSICAL_MASK) >> RCC_CSICFGR_CSICAL_SHIFT);

	trim_table_init(&stm32mp1_clk_cal_csi);

	stm32mp1_clk_cal_csi.set_trim(stm32mp1_clk_cal_csi.cal_ref);

	stm32mp1_rcc_calibration(&stm32mp1_clk_cal_csi);
}

void stm32mp1_cal_init(void)
{
	init_hsi_cal();
	init_csi_cal();

	timer_val = fdt_rcc_read_uint32_default("st,cal-sec", 0) *
		plat_get_syscnt_freq2();

	if (timer_val != 0U) {
		/* Load & enable timer */
		write_cntptval(timer_val);
		write_cntpctl(BIT(0));
	};

	if (fdt_rcc_enable_it("mcu_sev") < 0) {
		VERBOSE("No MCU calibration\n");
	}
}
#endif

static void stm32mp1_osc_init(void)
{
	enum stm32mp_osc_id i;

	for (i = (enum stm32mp_osc_id)0 ; i < NB_OSC; i++) {
		stm32mp1_osc_clk_init(stm32mp_osc_node_label[i], i);
	}
}

/*
 * Lookup platform clock from enable bit location in RCC registers.
 * Return a valid clock ID on success, return ~0 on error.
 */
unsigned long stm32mp1_clk_rcc2id(unsigned int offset, unsigned int bit)
{
	return get_id_from_rcc_bit(offset, bit);
}

/*
 * Get the parent ID of the target parent clock, for tagging as secure
 * shared clock dependencies.
 */
static int get_parent_id_parent(unsigned int parent_id)
{
	enum stm32mp1_parent_sel s = _UNKNOWN_SEL;
	enum stm32mp1_pll_id pll_id;
	uint32_t p_sel;

	switch (parent_id) {
	case _ACLK:
	case _PCLK4:
	case _PCLK5:
		s = _ASS_SEL;
		break;
	case _PLL1_P:
	case _PLL1_Q:
	case _PLL1_R:
		pll_id = _PLL1;
		break;
	case _PLL2_P:
	case _PLL2_Q:
	case _PLL2_R:
		pll_id = _PLL2;
		break;
	case _PLL3_P:
	case _PLL3_Q:
	case _PLL3_R:
		pll_id = _PLL3;
		break;
	case _PLL4_P:
	case _PLL4_Q:
	case _PLL4_R:
		pll_id = _PLL4;
		break;
	case _PCLK1:
	case _PCLK2:
	case _HCLK2:
	case _HCLK6:
	case _CK_PER:
	case _CK_MPU:
	case _CK_MCU:
	case _USB_PHY_48:
		/* We do not expected to access these */
		panic();
		break;
	default:
		/* Other parents have no parent */
		return -1;
	}

	if (s != _UNKNOWN_SEL) {
		const struct stm32mp1_clk_sel *sel = clk_sel_ref(s);
		uintptr_t rcc_base = stm32mp_rcc_base();

		p_sel = (mmio_read_32(rcc_base + sel->offset) >> sel->src) &
			sel->msk;

		if (p_sel < sel->nb_parent) {
			return (int)sel->parent[p_sel];
		}
	} else {
		const struct stm32mp1_clk_pll *pll = pll_ref(pll_id);
		uintptr_t rcc_base = stm32mp_rcc_base();

		p_sel = mmio_read_32(rcc_base + pll->rckxselr) &
			RCC_SELR_REFCLK_SRC_MASK;

		if (pll->refclk[p_sel] != _UNKNOWN_OSC_ID) {
			return (int)pll->refclk[p_sel];
		}
	}

#if LOG_LEVEL >= LOG_LEVEL_VERBOSE
	VERBOSE("No parent selected for %s",
		stm32mp1_clk_parent_name[parent_id]);
#endif

	return -1;
}

static void secure_parent_clocks(unsigned long parent_id)
{
	int grandparent_id;

	switch (parent_id) {
	/* Secure only the parents for these clocks */
	case _ACLK:
	case _HCLK2:
	case _HCLK6:
	case _PCLK4:
	case _PCLK5:
		break;
	/* PLLs */
	case _PLL1_P:
		stm32mp1_register_secure_periph(STM32MP1_SHRES_PLL1_P);
		break;
	case _PLL1_Q:
		stm32mp1_register_secure_periph(STM32MP1_SHRES_PLL1_Q);
		break;
	case _PLL1_R:
		stm32mp1_register_secure_periph(STM32MP1_SHRES_PLL1_R);
		break;

	case _PLL2_P:
		stm32mp1_register_secure_periph(STM32MP1_SHRES_PLL2_P);
		break;
	case _PLL2_Q:
		stm32mp1_register_secure_periph(STM32MP1_SHRES_PLL2_Q);
		break;
	case _PLL2_R:
		stm32mp1_register_secure_periph(STM32MP1_SHRES_PLL2_R);
		break;

	case _PLL3_P:
		stm32mp1_register_secure_periph(STM32MP1_SHRES_PLL3_P);
		break;
	case _PLL3_Q:
		stm32mp1_register_secure_periph(STM32MP1_SHRES_PLL3_Q);
		break;
	case _PLL3_R:
		stm32mp1_register_secure_periph(STM32MP1_SHRES_PLL3_R);
		break;

	/* Source clocks */
	case _HSI:
	case _HSI_KER:
		stm32mp1_register_secure_periph(STM32MP1_SHRES_HSI);
		break;
	case _LSI:
		stm32mp1_register_secure_periph(STM32MP1_SHRES_LSI);
		break;
	case _CSI:
	case _CSI_KER:
		stm32mp1_register_secure_periph(STM32MP1_SHRES_CSI);
		break;
	case _HSE:
	case _HSE_KER:
	case _HSE_KER_DIV2:
		stm32mp1_register_secure_periph(STM32MP1_SHRES_HSE);
		break;
	case _LSE:
		stm32mp1_register_secure_periph(STM32MP1_SHRES_LSE);
		break;

	default:
		panic();
	}

	grandparent_id = get_parent_id_parent(parent_id);
	if (grandparent_id >= 0) {
		secure_parent_clocks(grandparent_id);
	}
}

void stm32mp1_register_clock_parents_secure(unsigned long clock_id)
{
	int parent_id;

	if (!stm32mp1_rcc_is_secure()) {
		return;
	}

	switch (clock_id) {
	case PLL1:
		parent_id = get_parent_id_parent(_PLL1_P);
		break;
	case PLL2:
		parent_id = get_parent_id_parent(_PLL2_P);
		break;
	case PLL3:
		parent_id = get_parent_id_parent(_PLL3_P);
		break;
	case PLL4:
		ERROR("PLL4 cannot be secured\n");
		panic();
		break;
	default:
		/* Others are expected gateable clock */
		parent_id = stm32mp1_clk_get_parent(clock_id);
		break;
	}

	if (parent_id < 0) {
		ERROR("No parent for clock %lu", clock_id);
		panic();
	}

	secure_parent_clocks(parent_id);
}

/* Sync secure clock refcount after all drivers probe/inits,  */
void stm32mp1_update_earlyboot_clocks_state(void)
{
	unsigned int idx;

	for (idx = 0U; idx < NB_GATES; idx++) {
		const struct stm32mp1_clk_gate *gate = gate_ref(idx);
		unsigned long clock_id = gate_ref(idx)->index;

		/* Drop non secure refcnt on non shared shareable clocks */
		if (__clk_is_enabled(gate) &&
		    stm32mp1_clock_is_shareable(clock_id) &&
		    !stm32mp1_clock_is_shared(clock_id)) {
			stm32mp1_clk_disable_non_secure(clock_id);
		}
	}

#if LOG_LEVEL >= LOG_LEVEL_VERBOSE
	/* Dump clocks state */
	for (idx = 0U; idx < NB_GATES; idx++) {
		const struct stm32mp1_clk_gate *gate = gate_ref(idx);
		unsigned long __unused clock_id = gate->index;
		unsigned int __unused refcnt = gate_refcounts[idx];
		int __unused p = stm32mp1_clk_get_parent(clock_id);

		VERBOSE("stm32mp1 clk %lu %sabled (refcnt %d) (parent %d %s)\n",
			clock_id, __clk_is_enabled(gate) ? "en" : "dis",
			refcnt, p,
			p < 0 ? "n.a" : stm32mp1_clk_parent_sel_name[p]);
	}
#endif
}

static void sync_earlyboot_clocks_state(void)
{
	unsigned int idx;
	int res;

	for (idx = 0U; idx < NB_GATES; idx++) {
		assert(gate_refcounts[idx] == 0);
	}

	/* Set a non secure refcnt for shareable clocks enabled from boot */
	for (idx = 0U; idx < NB_GATES; idx++) {
		struct stm32mp1_clk_gate const *gate = gate_ref(idx);

		if (__clk_is_enabled(gate) &&
		    stm32mp1_clock_is_shareable(gate->index)) {
			gate_refcounts[idx] = SHREFCNT_NONSECURE_FLAG;
		}
	}

	/*
	 * Register secure clock parents and init a refcount for
	 * secure only resources that are not registered from a driver probe.
	 * - DDR controller and phy clocks.
	 * - TZC400, ETZPC and STGEN clocks.
	 * - RTCAPB clocks on multi-core
	 */
	stm32mp1_register_clock_parents_secure(DDRC1);
	stm32mp1_clk_enable_secure(DDRC1);
	stm32mp1_register_clock_parents_secure(DDRC1LP);
	stm32mp1_clk_enable_secure(DDRC1LP);
	stm32mp1_register_clock_parents_secure(DDRC2);
	stm32mp1_clk_enable_secure(DDRC2);
	stm32mp1_register_clock_parents_secure(DDRC2LP);
	stm32mp1_clk_enable_secure(DDRC2LP);
	stm32mp1_register_clock_parents_secure(DDRPHYC);
	stm32mp1_clk_enable_secure(DDRPHYC);
	stm32mp1_register_clock_parents_secure(DDRPHYCLP);
	stm32mp1_clk_enable_secure(DDRPHYCLP);
	stm32mp1_register_clock_parents_secure(DDRCAPB);
	stm32mp1_clk_enable_secure(DDRCAPB);
	stm32mp1_register_clock_parents_secure(AXIDCG);
	stm32mp1_clk_enable_secure(AXIDCG);
	stm32mp1_register_clock_parents_secure(DDRPHYCAPB);
	stm32mp1_clk_enable_secure(DDRPHYCAPB);
	stm32mp1_register_clock_parents_secure(DDRPHYCAPBLP);
	stm32mp1_clk_enable_secure(DDRPHYCAPBLP);

	stm32mp1_register_clock_parents_secure(TZPC);
	stm32mp1_clk_enable_secure(TZPC);
	stm32mp1_register_clock_parents_secure(TZC1);
	stm32mp1_clk_enable_secure(TZC1);
	stm32mp1_register_clock_parents_secure(TZC2);
	stm32mp1_clk_enable_secure(TZC2);
	stm32mp1_register_clock_parents_secure(STGEN_K);
	stm32mp1_clk_enable_secure(STGEN_K);

	stm32mp1_register_clock_parents_secure(BSEC);
	stm32mp1_register_clock_parents_secure(BKPSRAM);

	stm32mp1_register_clock_parents_secure(RTCAPB);

	res = stm32mp_is_single_core();
	if (res < 0) {
		panic();
	}

	if (res == 0) {
		stm32mp1_clk_enable_secure(RTCAPB);
	}
}

void stm32mp1_rcc_init_late(void)
{
#if defined(IMAGE_BL32)
	int irq_num;

	if (!stm32mp1_rcc_is_secure()) {
		return;
	}

	irq_num = fdt_rcc_enable_it("wakeup");
	if (irq_num < 0) {
		panic();
	}

	plat_ic_set_interrupt_priority(irq_num, STM32MP1_IRQ_RCC_SEC_PRIO);
#endif
}

int stm32mp1_clk_probe(void)
{
	stm32mp1_osc_init();

	sync_earlyboot_clocks_state();

	return 0;
}
