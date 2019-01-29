/*
 * Copyright (c) 2015-2017, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __IO_QSPI_H__
#define __IO_QSPI_H__

#define QSPI_FMODE_MASK			0x0C000000
#define QSPI_FMODE_MEMORY_MAPPED	0x0C000000

#define QSPI_NOR_MAX_SIZE		0x10000000 /* 256 MB*/
#define QSPI_NOR_LBA_SIZE		0x00000200 /* 512 B*/
#define QSPI_NOR_BLK_SIZE		0x00040000 /* 256 KB*/

#define QSPI_CR_EN			0x00000001
#define QSPI_CR_ABORT			0x00000002
#define QSPI_CR_TCEN			0x00000008
#define QSPI_CR_SSHIFT			0x00000010
#define QSPI_CR_DFM			0x00000040
#define QSPI_CR_PRESCALER_SHIFT		24

#define QSPI_DCR_CSHT			0x00000100
#define QSPI_DCR_FSIZE_MASK		0x001F0000
#define QSPI_DCR_FSIZE_SHIFT		16

#define QSPI_CCR_INST			0x0000000B
#define QSPI_CCR_IMODE			0x00000100
#define QSPI_CCR_ADMODE			0x00000400
#define QSPI_CCR_ADSIZE			0x00002000
#define QSPI_CCR_ABMODE			0x00000000
#define QSPI_CCR_ABSIZE			0x00000000
#define QSPI_CCR_DCYC			0x00200000
#define QSPI_CCR_DMODE			0x01000000
#define QSPI_CCR_FMODE_MM		0x0C000000
#define QSPI_CCR_FMODE			0x04000000
#define QSPI_CCR_SIOO			0x00000000
#define QSPI_CCR_DDRM			0x00000000
#define QSPI_CCR_DHHC			0x00000000

#define QSPI_SR_FLEVEL			0x1F00
#define QSPI_SR_BUSY			0x0020
#define QSPI_SR_TOF			0x0010
#define QSPI_SR_SMF			0x0008
#define QSPI_SR_FTF			0x0004
#define QSPI_SR_TCF			0x0002
#define QSPI_SR_TEF			0x0001

#define QSPI_FCR_CTOF			0x0008
#define QSPI_FCR_CSMF			0x0004
#define QSPI_FCR_CTCF			0x0002
#define QSPI_FCR_CTEF			0x0001

#define QSPI_DFLT_READ_FLAGS		(QSPI_CCR_INST | QSPI_CCR_IMODE | \
					 QSPI_CCR_ADMODE | QSPI_CCR_ADSIZE | \
					 QSPI_CCR_ABMODE | QSPI_CCR_ABSIZE | \
					 QSPI_CCR_DCYC | QSPI_CCR_DMODE | \
					 QSPI_CCR_SIOO | QSPI_CCR_DDRM | \
					 QSPI_CCR_DHHC)

/*
 * QUAD Serial Peripheral Interface
 */

typedef struct {
	volatile uint32_t CR;       /* Control register */
	volatile uint32_t DCR;      /* Device Configuration register */
	volatile uint32_t SR;       /* Status register */
	volatile uint32_t FCR;      /* Flag Clear register */
	volatile uint32_t DLR;      /* Data Length register */
	volatile uint32_t CCR;      /* Communication Configuration register */
	volatile uint32_t AR;       /* Address register */
	volatile uint32_t ABR;      /* Alternate Bytes register */
	volatile uint32_t DR;       /* Data register */
	volatile uint32_t PSMKR;    /* Polling Status Mask register */
	volatile uint32_t PSMAR;    /* Polling Status Match register */
	volatile uint32_t PIR;      /* Polling Interval register */
	volatile uint32_t LPTR;     /* Low Power Timeout register */
} QUADSPI_TypeDef;

/*
 * QSPI Handle Structure definition
 */
typedef struct {
	QUADSPI_TypeDef	*instance;	/* QSPI registers base address */
	uint32_t	is_dual;
} QSPI_HandleTypeDef;

int register_io_dev_qspi(const io_dev_connector_t **dev_con);

#endif /* __IO_QSPI_H__ */
