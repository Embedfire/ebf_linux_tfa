/*
 * Copyright (c) 2019, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*****************************************************************************
 * This file is only needed for current Soc revision which has a limitation on
 * debug reset halt. This can be removed when using the Soc revision that
 * fixes the limitation. Anyway, this source code identifies the Soc revision
 * and is only executed if it corresponds, so it can be kept on other
 * revisions without any consequence.
 * The revisions that need the workaround have ID values:
 * - 0x2000X500
 * - 0x2001X500
 ****************************************************************************/

/*****************************************************************************
 * This file has been intentionally transformed in order to use only ASM
 * standard macros and instructions. It could then be easily back-ported to
 * other pieces of software.
 ****************************************************************************/

#include <asm_macros.S>

#define BIT_(nr)			((1) << (nr))

#define BSEC_BASE			0x5C005000
#define BSEC_OTP_DATA_OFF		(BSEC_BASE + 0x200)
#define BSEC_OTP_DATA0			BSEC_OTP_DATA_OFF
#define BSEC_OTP_DATA0_CLOSED		BIT_(6)

#define DBGMCU_BASE			0x50081000
#define DBGMCU_IDC			0x00
#define DBGMCU_IDC_REV_ID_DEV_ID_MSK	0xFFFE0FFF
#define DBGMCU_IDC_REV_ID_DEV_ID_VALUE	0x20000500

#define RCC_BASE			0x50000000
#define RCC_MP_APB5ENSETR		0x208
#define RCC_MP_APB5ENSETR_RTCAPBEN	BIT_(8)
#define RCC_DBGCFGR			0x80C
#define RCC_DBGCFGR_DBGCKEN		BIT_(8)

#define PWR_BASE			0x50001000
#define PWR_CR1				0x00
#define PWR_CR1_DBP			BIT_(8)

#define TAMP_BASE			0x5C00A000
#define TAMP_BKP_REGISTER_BASE		TAMP_BASE + 0x100
#define TAMP_BKP_REGISTER_20		TAMP_BKP_REGISTER_BASE + (20 << 2)

#define CA7_DBG0_BASE			0x500D0000
#define DBG_DSCR			0x88
#define DBG_DSCR_HDBGEN			BIT_(14)

#define FSBL_ENTRYPOINT			bl2_entrypoint

	.globl	plat_dbg_attach_loop


func plat_dbg_attach_loop
	/*
	 * This function is the first call of FSBL_ENTRYPOINT.
	 * Boot rom parameters are stored in r0..r3, so we mustn't use them
	 * here. And because they are saved in r9..r12 just after the
	 * execution of this function, we should only use these registers.
	 * By this way, debug will be done in conditions closed to the initial
	 * context.
	 */

	/*
	 * Check Sec Close bit in OTP (word 0 bit 6). If enabled, do not allow
	 * debug session and exit function.
	 */
	ldr	r12, =BSEC_OTP_DATA0
	ldr	r12, [r12]
	ands	r11, r12, #BSEC_OTP_DATA0_CLOSED
	bne	func_exit

	/*
	 * Enable DBGMCU clock (only if not done).
	 * Initial register content will be saved in r10.
	 * So r10 mustn't be used before restore part.
	 */
	ldr	r12, =(RCC_BASE + RCC_DBGCFGR)
	ldr	r10, [r12]
	tst	r10, #RCC_DBGCFGR_DBGCKEN
	bne	dbgmcu_clk_enabled
	orr	r11, r10, #RCC_DBGCFGR_DBGCKEN
	str	r11, [r12]

	/* Get SoC revision ID and device ID in r11 */
dbgmcu_clk_enabled:
	ldr	r12, =(DBGMCU_BASE + DBGMCU_IDC)
	ldr	r12, [r12]
	and	r11, r12, #DBGMCU_IDC_REV_ID_DEV_ID_MSK

	/* Restore initial RCC_DBGCFGR content saved in r10 */
	ldr	r12, =(RCC_BASE + RCC_DBGCFGR)
	str	r10, [r12]

	/*
	 * Check SoC revision ID and device ID in r11,
	 * exit if different from REV_ID_DEV_ID_VALUE
	 */
	ldr	r12, =(DBGMCU_IDC_REV_ID_DEV_ID_VALUE)
	teq	r11, r12
	bne	func_exit

	/*
	 * Enable RTC clock before reading tamper (only if not done).
	 * Initial register content will be saved in r10.
	 * So r10 mustn't be used before restore part.
	 */
	ldr	r12, =(RCC_BASE + RCC_MP_APB5ENSETR)
	ldr	r10, [r12]
	tst	r10, #RCC_MP_APB5ENSETR_RTCAPBEN
	bne	rtc_clk_enabled
	orr	r11, r10, #RCC_MP_APB5ENSETR_RTCAPBEN
	str	r11, [r12]

rtc_clk_enabled:
	/*
	 * Disable the backup domain write protection (only if not done).
	 * Initial register content will be saved in r9.
	 * So r9 mustn't be used before restore part.
	 */
	ldr	r12, =(PWR_BASE + PWR_CR1)
	ldr	r9, [r12]
	tst	r9, #PWR_CR1_DBP
	bne	poll_dbp
	orr	r11, r9, #PWR_CR1_DBP
	str	r11, [r12]
poll_dbp:
	/* poll loop to ensure write is effective */
	ldr	r11, [r12]
	tst	r11, #PWR_CR1_DBP
	beq	poll_dbp

	/*
	 * Clear bit 16 of TAMPER backup register 20 (only if set).
	 * Firstly read the register value.
	 * - If bit = 0, r11 = 0 after 'ands' operation. Next step is to
	 *   restore RCC_MP_APB5ENSETR and PWR_CR1 contents and exit.
	 * - If bit = 1, r11 != 0 after 'ands' operation, but could be
	 *   equal to 0 after 'bic' operation. Here, after clearing the bit,
	 *   r11 has to be set to a non-zero value. Next step is to restore
	 *   register contents and continue.
	 *
	 * So r11 mustn't be used in restore part, to keep this information
	 * for the next step, i.e. decide to continue execution (r11 = 1) or
	 * exit function (r11 = 0).
	 */
	ldr	r12, =(TAMP_BKP_REGISTER_20)
	ldr	r11, [r12]
	ands	r11, r11, #(BIT_(16))
	beq	restore_reg
	ldr	r11, [r12]
	bic	r11, #(BIT_(16))
	str	r11, [r12]
	ldr	r11, =(1)

restore_reg:
	/* Restore initial RCC_MP_APB5ENSETR content saved in r10 */
	ldr	r12, =(RCC_BASE + RCC_MP_APB5ENSETR)
	str	r10, [r12]

	/* Restore initial PWR_CR1 content saved in r9 */
	ldr	r12, =(PWR_BASE + PWR_CR1)
	str	r9, [r12]
poll_cr1:
	/* poll loop to ensure write is effective */
	ldr	r12, =(PWR_BASE + PWR_CR1)
	ldr	r12, [r12]
	teq	r12, r9
	bne	poll_cr1

	/*
	 * Exit if bit 16 of TAMPER backup register 20 has been cleared.
	 * Information is saved in r11 register.
	 */
	teq	r11, #0
	beq	func_exit

	/* Get time counter frequency */
	mrc	15, 0, r12, cr14, cr0, 0
	/* Get current time counter value, save it in r11 as start value */
	mrrc	15, 0, r11, r10, cr14
	/*
	 * Compute (current time + 1 second) counter value, save it in r12 as
	 * end value.
	 */
	add	r12, r12, r11

loop:
	/*
	 * Read CA7_DBG0 DBG_DSCR HDBGEN bit value.
	 * If set, put a software breakpoint and branch to FSBL_ENTRYPOINT.
	 * If cleared, continue and check loop time expiry
	 */
	ldr	r10, =(CA7_DBG0_BASE + DBG_DSCR)
	ldr	r10, [r10]
	tst	r10, #DBG_DSCR_HDBGEN
	beq	loop_continue
	/* Set a software breakpoint (ID 5) */
	bkpt	5
	/* Jump to entrypoint */
	b	FSBL_ENTRYPOINT
loop_continue:
	/*
	 * Check 1 second expiry by using r11 and r12 saved values.
	 * Get current time counter value, save it in r10 as current value.
	 */
	mrrc	15, 0, r10, r9, cr14
	/* Check if MSB 64-bit increment done between start and end values */
	cmp	r12, r11
	bmi	msb_incr
	/* No increment, simply check if current < end, exit if yes */
	cmp	r12, r10
	bmi	func_exit
	b	loop
msb_incr:
	/*
	 * Increment happened between start and end, here we need to check if
	 * (current > end) && (current < start). Exit if yes.
	 */
	cmp	r12, r10
	bpl	loop
	cmp	r11, r10
	bmi	loop
func_exit:
	bx	lr
endfunc plat_dbg_attach_loop
