/*
 ******************************************************************************
 * @file    nand.c
 * @author  mgentilini - MCD IntroPack team - MPU AP v1 bootROM project
 * @version V0.1
 * @date    28-April-2016
 * @brief   Nand FMC driver module for STM32MP1.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) STMicroelectronics</center></h2>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ******************************************************************************
 */

#include <assert.h>
#include <debug.h>
#include <delay_timer.h>
#include <limits.h>
#include <nand.h>
#include <platform.h>
#include <platform_def.h>
#include <stdint.h>
#include <utils_def.h>

/* Other internal NAND driver definitions */
#define CMD_SECTION             ((uint32_t)(1 << 16))  /* A16 high */
#define ADDR_SECTION            ((uint32_t)(1 << 17))  /* A17 high */
#define DATA_SECTION            (0)                    /* A16 and A17 low */

#define EIGHT_BIT_ACCESS    0x0
#define SIXTEEN_BIT_ACCESS  0x1

#define ONFI_SIG_VALUE      0x49464E4F /* "ONFI" */
#define EXT_PAGE_SIG_VALUE  0x53505045 /* "EPPS" */

/* NAND instance number */
#define NAND_INSTANCE_NB    1
/* Wait Time following NAND_RESET command sent (30us) */
#define NAND_RST_TIMEOUT   30
#define NAND_READY_WAIT_TIMEOUT_VAL_250MS_AT_64MHz  16000000

/* NAND ECC Calculation wait timeout */
#define NAND_ECC_CALCULATION_TIMEOUT_VAL_250MS	ms2tick(250)

#define NAND_ECC_PAGE_SECTOR     512
#define NAND_ECC_HAMMING1              1
#define NAND_ECC_BCH4                  4
#define NAND_ECC_BCH8                  8
#define NAND_ECC_HAMMING1_BYTES_NB_8b  3
#define NAND_ECC_BCH4_BYTES_NB_8b      7
#define NAND_ECC_BCH8_BYTES_NB_8b     13
#define NAND_ECC_HAMMING1_BYTES_NB_16b 4
#define NAND_ECC_BCH4_BYTES_NB_16b     8
#define NAND_ECC_BCH8_BYTES_NB_16b    14
#define PARAM_PAGE_SIZE          256

/* NAND memory status */
#define NAND_BUSY                  ((uint32_t)0x00000000U)
#define NAND_ERROR                 ((uint32_t)0x00000001U)
#define NAND_READY                 ((uint32_t)0x00000040U)

/* NAND ONFI commands */
#define NAND_CMD_RESET                   ((uint8_t)0xFFU)
#define NAND_CMD_READID                  ((uint8_t)0x90U)
#define NAND_CMD_READID_SIG_ADDR         ((uint8_t)0x20U)
#define NAND_CMD_READ_PARAM_PAGE         ((uint8_t)0xECU)
#define NAND_CMD_READ_1ST                ((uint8_t)0x00U)
#define NAND_CMD_READ_2ND                ((uint8_t)0x30U)
#define NAND_CMD_STATUS                  ((uint8_t)0x70U)
#define NAND_CMD_CHANGE_1ST              ((uint8_t)0x05U)
#define NAND_CMD_CHANGE_2ND              ((uint8_t)0xE0U)

/* CRC calculation */
#define CRC_POLYNOM      0x8005
#define CRC_INIT_VALUE   0x4F4E

static void nand_calc_timing(NAND_HandleTypeDef *hNand)
{
	nand_timings *tims = &hNand->Info.timings;
	unsigned long hclk = stm32mp_clk_get_rate(FMC_K);
	unsigned long hclkp = FMC_NSEC_PER_SEC / (hclk / 1000);
	unsigned long timing, tar, tclr, thiz, twait;
	unsigned long tset_mem, tset_att, thold_mem, thold_att;

	tar = MAX(hclkp, FMC_TAR_MIN);
	timing = div_round_up(tar, hclkp) - 1;
	tims->tar = MIN(timing, FMC_PCR_TIMING_MASK);

	tclr = MAX(hclkp, FMC_TCLR_MIN);
	timing = div_round_up(tclr, hclkp) - 1;
	tims->tclr = MIN(timing, FMC_PCR_TIMING_MASK);

	tims->thiz = FMC_THIZ;
	thiz = (tims->thiz + 1) * hclkp;

	/*
	 * tWAIT > tRP
	 * tWAIT > tWP
	 * tWAIT > tREA + tIO
	 */
	twait = MAX(hclkp, FMC_TRP_MIN);
	twait = MAX(twait, FMC_TWP_MIN);
	twait = MAX(twait, FMC_TREA_MAX + FMC_TIO);
	timing = div_round_up(twait, hclkp);
	tims->twait = MIN(MAX(timing, 1UL), FMC_PMEM_PATT_TIMING_MASK);

	/*
	 * tSETUP_MEM > tCS - tWAIT
	 * tSETUP_MEM > tALS - tWAIT
	 * tSETUP_MEM > tDS - (tWAIT - tHIZ)
	 */
	tset_mem = hclkp;
	if (twait < FMC_TCS_MIN && (tset_mem < FMC_TCS_MIN - twait)) {
		tset_mem = FMC_TCS_MIN - twait;
	}
	if (twait < FMC_TALS_MIN && (tset_mem < FMC_TALS_MIN - twait)) {
		tset_mem = FMC_TALS_MIN - twait;
	}
	if (twait > thiz && (twait - thiz < FMC_TDS_MIN) &&
	    (tset_mem < FMC_TDS_MIN - (twait - thiz))) {
		tset_mem = FMC_TDS_MIN - (twait - thiz);
	}
	timing = div_round_up(tset_mem, hclkp);
	tims->tset_mem = MIN(MAX(timing, 1UL), FMC_PMEM_PATT_TIMING_MASK);

	/*
	 * tHOLD_MEM > tCH
	 * tHOLD_MEM > tREH - tSETUP_MEM
	 * tHOLD_MEM > max(tRC, tWC) - (tSETUP_MEM + tWAIT)
	 */
	thold_mem = MAX(hclkp, FMC_TCH_MIN);
	if (tset_mem < FMC_TREH_MIN &&
	    (thold_mem < FMC_TREH_MIN - tset_mem)) {
		thold_mem = FMC_TREH_MIN - tset_mem;
	}
	if ((tset_mem + twait < FMC_TRC_MIN) &&
	    (thold_mem < FMC_TRC_MIN - (tset_mem + twait))) {
		thold_mem = FMC_TRC_MIN  - (tset_mem + twait);
	}
	if ((tset_mem + twait < FMC_TWC_MIN) &&
	    (thold_mem < FMC_TWC_MIN - (tset_mem + twait))) {
		thold_mem = FMC_TWC_MIN - (tset_mem + twait);
	}
	timing = div_round_up(thold_mem, hclkp);
	tims->thold_mem = MIN(MAX(timing, 1UL), FMC_PMEM_PATT_TIMING_MASK);

	/*
	 * tSETUP_ATT > tCS - tWAIT
	 * tSETUP_ATT > tCLS - tWAIT
	 * tSETUP_ATT > tALS - tWAIT
	 * tSETUP_ATT > tRHW - tHOLD_MEM
	 * tSETUP_ATT > tDS - (tWAIT - tHIZ)
	 */
	tset_att = hclkp;
	if (twait < FMC_TCS_MIN && (tset_att < FMC_TCS_MIN - twait)) {
		tset_att = FMC_TCS_MIN - twait;
	}
	if (twait < FMC_TCLS_MIN && (tset_att < FMC_TCLS_MIN - twait)) {
		tset_att = FMC_TCLS_MIN - twait;
	}
	if (twait < FMC_TALS_MIN && (tset_att < FMC_TALS_MIN - twait)) {
		tset_att = FMC_TALS_MIN - twait;
	}
	if (thold_mem < FMC_TRHW_MIN &&
	    (tset_att < FMC_TRHW_MIN - thold_mem)) {
		tset_att = FMC_TRHW_MIN - thold_mem;
	}
	if (twait > thiz && (twait - thiz < FMC_TDS_MIN) &&
	    (tset_att < FMC_TDS_MIN - (twait - thiz))) {
		tset_att = FMC_TDS_MIN - (twait - thiz);
	}
	timing = div_round_up(tset_att, hclkp);
	tims->tset_att = MIN(MAX(timing, 1UL), FMC_PMEM_PATT_TIMING_MASK);

	/*
	 * tHOLD_ATT > tALH
	 * tHOLD_ATT > tCH
	 * tHOLD_ATT > tCLH
	 * tHOLD_ATT > tCOH
	 * tHOLD_ATT > tDH
	 * tHOLD_ATT > tWB + tIO + tSYNC - tSETUP_MEM
	 * tHOLD_ATT > tADL - tSETUP_MEM
	 * tHOLD_ATT > tWH - tSETUP_MEM
	 * tHOLD_ATT > tWHR - tSETUP_MEM
	 * tHOLD_ATT > tRC - (tSETUP_ATT + tWAIT)
	 * tHOLD_ATT > tWC - (tSETUP_ATT + tWAIT)
	 */
	thold_att = MAX(hclkp, FMC_TALH_MIN);
	thold_att = MAX(thold_att, FMC_TCH_MIN);
	thold_att = MAX(thold_att, FMC_TCLH_MIN);
	thold_att = MAX(thold_att, FMC_TCOH_MIN);
	thold_att = MAX(thold_att, FMC_TDH_MIN);
	if ((FMC_TWB_MAX + FMC_TIO + FMC_TSYNC > tset_mem) &&
	    (thold_att < FMC_TWB_MAX + FMC_TIO + FMC_TSYNC - tset_mem)) {
		thold_att = FMC_TWB_MAX + FMC_TIO + FMC_TSYNC - tset_mem;
	}
	if (tset_mem < FMC_TADL_MIN &&
	    (thold_att < FMC_TADL_MIN - tset_mem)) {
		thold_att = FMC_TADL_MIN - tset_mem;
	}
	if (tset_mem < FMC_TWH_MIN &&
	    (thold_att < FMC_TWH_MIN - tset_mem)) {
		thold_att = FMC_TWH_MIN - tset_mem;
	}
	if (tset_mem < FMC_TWHR_MIN &&
	   (thold_att < FMC_TWHR_MIN - tset_mem)) {
		thold_att = FMC_TWHR_MIN - tset_mem;
	}
	if (tset_att + twait < FMC_TRC_MIN &&
	    (thold_att < FMC_TRC_MIN - (tset_att + twait))) {
		thold_att = FMC_TRC_MIN - (tset_att + twait);
	}
	if (tset_att + twait < FMC_TWC_MIN &&
	    (thold_att < FMC_TWC_MIN - (tset_att + twait))) {
		thold_att = FMC_TWC_MIN - (tset_att + twait);
	}
	timing = div_round_up(thold_att, hclkp);
	tims->thold_att = MIN(MAX(timing, 1UL), FMC_PMEM_PATT_TIMING_MASK);

	VERBOSE("Nand timings: %u - %u - %u - %u - %u - %u - %u - %u\n",
		tims->tclr, tims->tar, tims->thiz, tims->twait,
		tims->thold_mem, tims->tset_mem,
		tims->thold_att, tims->tset_att);
}

/*****************************************************************************
 *
 * Function:            Nand_Init
 *
 * Description:         This function initializes the Nand FMC driver.
 *
 * Input parameters:    NAND_HandleTypeDef * hNand
 *                      uint32_t             bus_width
 *
 * Output parameters:   none
 *
 * Return:              None
 *
 *****************************************************************************/
static void Nand_Init(NAND_HandleTypeDef *hNand, uint32_t bus_width,
		      uint32_t bch_algo)
{
	assert(hNand);

	/*
	 * Initialize NAND control Interface
	 * Wait feature disabled, memory bank disabled, ECC logic disabled,
	 * BCH is selected, TCLR and TAR to max values, ECC sector size to 512B,
	 * BCH 8-bit is selected, enabled read access
	 *
	 * Be careful to not set ECCEN bit before Read Parameter Page command
	 * ECC logic must be enabled just before Read Logical Page
	 * Note: with PWAITEN feature, NAND controller temporizes next accesses,
	 * so no need to wait for ready/not busy after each command
	 */

	/*
	 * Parameters should be given by boot_context
	 * Try to retrieve them with Nand_DetectAndInit
	 */
	if (!hNand->Info.PageSize) {
		hNand->Instance->PCReg |= FMC_PCR_ECCALG |  /* Select BCH */
			FMC_PCR_ECCSS_0 |   /* ECCSS: Sector Size=512 bytes */
			FMC_PCR_BCHECC |
			FMC_PCR_PWAITEN;

		hNand->Instance->PCReg &= ~FMC_PCR_PBKEN &
			~FMC_PCR_WE &
			~FMC_PCR_ECCSS_1 &  /* ECCSS: Sector Size=512 bytes */
			~FMC_PCR_ECCSS_2;   /* ECCSS: Sector Size=512 bytes */

		if (bus_width == SIXTEEN_BIT_ACCESS) {
			hNand->Instance->PCReg |= FMC_PCR_PWID_0;
			hNand->Instance->PCReg &= ~FMC_PCR_PWID_1;
		} else {
			hNand->Instance->PCReg &= ~FMC_PCR_PWID_0;
			hNand->Instance->PCReg &= ~FMC_PCR_PWID_1;
		}

		if (bch_algo == NAND_ECC_BCH4) {
			/* BCH selected */
			hNand->Instance->PCReg |= FMC_PCR_ECCALG;
			 /* BCH4 selected */
			hNand->Instance->PCReg &= ~FMC_PCR_BCHECC;
		} else if (bch_algo == NAND_ECC_BCH8) {
			 /* BCH selected */
			hNand->Instance->PCReg |= FMC_PCR_ECCALG;
			/* BCH8 selected */
			hNand->Instance->PCReg |= FMC_PCR_BCHECC;
		} else {
			/* Hamming code selected */
			hNand->Instance->PCReg &= ~FMC_PCR_ECCALG;
		}
	}

	nand_calc_timing(hNand);

	/* Set tclr/tar timings */
	hNand->Instance->PCReg &=
		~FMC_PCR_TCLR_0 &  /* tCLR: forced to value 0 */
		~FMC_PCR_TCLR_1 &  /* tCLR: forced to value 0 */
		~FMC_PCR_TCLR_2 &  /* tCLR: forced to value 0 */
		~FMC_PCR_TCLR_3 &  /* tCLR: forced to value 0 */
		~FMC_PCR_TAR_0 &   /* tAR: forced to value 0 */
		~FMC_PCR_TAR_1 &   /* tAR: forced to value 0 */
		~FMC_PCR_TAR_2 &   /* tAR: forced to value 0 */
		~FMC_PCR_TAR_3;	   /* tAR: forced to value 0 */

	hNand->Instance->PCReg |= FMC_PCR_TCLR(hNand->Info.timings.tclr);
	hNand->Instance->PCReg |= FMC_PCR_TAR(hNand->Info.timings.tar);

	/* Set tset/twait/thold/thiz timings in common bank */
	hNand->Instance->PMEM = FMC_PMEM_MEMSET(hNand->Info.timings.tset_mem);
	hNand->Instance->PMEM |= FMC_PMEM_MEMWAIT(hNand->Info.timings.twait);
	hNand->Instance->PMEM |=
		FMC_PMEM_MEMHOLD(hNand->Info.timings.thold_mem);
	hNand->Instance->PMEM |= FMC_PMEM_MEMHIZ(hNand->Info.timings.thiz);

	/* Set tset/twait/thold/thiz timings in attribut bank */
	hNand->Instance->PATT = FMC_PATT_ATTSET(hNand->Info.timings.tset_att);
	hNand->Instance->PATT |= FMC_PATT_ATTWAIT(hNand->Info.timings.twait);
	hNand->Instance->PATT |=
		FMC_PATT_ATTHOLD(hNand->Info.timings.thold_att);
	hNand->Instance->PATT |= FMC_PATT_ATTHIZ(hNand->Info.timings.thiz);

	/* Enable the NAND device */
	hNand->Instance->PCReg |= FMC_PCR_PBKEN;

	/* Enable FMC Controller IP */
	hNand->Instance->BCR1 |= FMC_BCR1_FMCEN;
}

/*
 * @brief  NAND memory read status
 * @param  hNand: pointer to a NAND_HandleTypeDef structure that contains
 *                the configuration information for NAND module.
 * @retval NAND status
 */
static uint32_t NAND_ReadStatus(NAND_HandleTypeDef *hNand)
{
	uint32_t data;
	uintptr_t deviceComMemAddr;
	uintptr_t deviceAttrMemAddr;

	assert(hNand);

	/* Identify the device address */
	deviceComMemAddr = FLASH_COMMON_MEM_BASE;
	deviceAttrMemAddr = FLASH_ATTRIB_MEM_BASE;

	/* Send Read status operation command */
	*(__IO uint8_t *)(deviceAttrMemAddr | CMD_SECTION) =
		NAND_CMD_STATUS;

	/* Read status register data */
	data = *(__IO uint8_t *)deviceComMemAddr;

	/* Return the status */
	if ((data & NAND_ERROR) == NAND_ERROR)
		return NAND_ERROR;
	else if ((data & NAND_READY) == NAND_READY)
		return NAND_READY;

	return NAND_BUSY;
}

/*****************************************************************************
 *
 * Function:            NAND_WaitReadyWithTimeout
 *
 * Description:         This function wait with timeout of 250 ms until
 *			NAND Ready bit is set.
 *                      Nand detection and initialization.
 *
 * Input parameters:    NAND_HandleTypeDef * hNand
 *
 * Return:              value 0 is no timeout occurred, value 1 if 250 ms
 *			wait timeout elapsed while waiting for NAND Ready
 *			flag to raise.
 *
 *****************************************************************************/
static uint32_t NAND_WaitReadyWithTimeout(NAND_HandleTypeDef *hNand)
{
	uint32_t timerValInit = 0;
	uint32_t timeoutDetected = 0;

	/* Is Nand handle NULL ? */
	assert(hNand);

	/* Get timer current value at start of loop */
	timerValInit = (uint32_t)read_cntpct_el0();

	/* Wait NAND Ready by a read status command until response or */
	/* NAND Ready timeout elapses                                 */
	/* timeout = 250ms @ 64 MHz of gentimer                       */
	while ((NAND_ReadStatus(hNand) != NAND_READY) &&
	       (timeoutDetected == 0)) {
		/* Check if timeout occurred */
		if (((uint32_t)read_cntpct_el0() - timerValInit) >=
		   NAND_READY_WAIT_TIMEOUT_VAL_250MS_AT_64MHz)
			timeoutDetected = 1;
	}

	return(timeoutDetected);
}

/*****************************************************************************
 *
 * Function:            Nand_Reset
 *
 * Description:         Resets NAND Flash memory.
 *
 * Input parameters:    NAND_HandleTypeDef * hNand
 *
 * Output parameters:   none
 *
 * Return:              Std_ReturnType
 *
 *****************************************************************************/
static Std_ReturnType Nand_Reset(NAND_HandleTypeDef *hNand)
{
	Std_ReturnType retVal = STD_NOT_OK;
	uintptr_t deviceAddress;
	uint32_t timeoutDetected = 0;

	/* Is Nand handle NULL ? */
	assert(hNand);

	/* Identify the device address */
	deviceAddress = FLASH_COMMON_MEM_BASE;

	/* Send NAND reset command */
	/* Writes the command value in command section */
	*(uint8_t *)(deviceAddress | CMD_SECTION) = NAND_CMD_RESET;

	/* Wait the fixed 30 us time : after the NAND Reset command sent */
	/* 30us @ 64 MHz of gentimer */
	udelay(NAND_RST_TIMEOUT);

	/* Wait NANDReady with timeout 250 ms */
	timeoutDetected = NAND_WaitReadyWithTimeout(hNand);

	/* If no timeout occurred : the NAND Reset phase was successful */
	if (timeoutDetected == 0)
	       /* Set good status on exit */
		retVal = STD_OK;

	return retVal;
}

/*****************************************************************************
 *
 * Function:            Nand_ReadIDCode
 *
 * Description:         This function reads the NAND ID code
 *
 * Input parameters:    NAND_HandleTypeDef * hNand: NAND handle
 *                      NAND_IDTypeDef * pNAND_ID: NAND ID structure
 *
 * Output parameters:   none
 *
 * Return:              None
 *
 *****************************************************************************/
static void Nand_ReadIDCode(NAND_HandleTypeDef *hNand, NAND_IDTypeDef *pNAND_ID)
{
	uint32_t data;
	uintptr_t deviceComMemAddr;
	uintptr_t deviceAttrMemAddr;

	assert(hNand);

	/* Identify the device addresses */
	deviceComMemAddr  = FLASH_COMMON_MEM_BASE;
	deviceAttrMemAddr = FLASH_ATTRIB_MEM_BASE;

	/* Send Read ID command sequence */
	*(__IO uint8_t *)(deviceComMemAddr | CMD_SECTION) = NAND_CMD_READID;
	*(__IO uint8_t *)(deviceAttrMemAddr | ADDR_SECTION) = 0x00;

	udelay(NAND_RST_TIMEOUT);

	/* Read the electronic signature from NAND flash */
	data = *(__IO uint16_t *)deviceComMemAddr;

	/* Return the data read */
	pNAND_ID->Maker_Id   = (uint8_t)(data);
	pNAND_ID->Device_Id  = (uint8_t)(data >> 8);
}

/**
 * @brief  NAND check CRC16
 * @param  crc: initial CRC value
 * @param  data_in: pointer to buffer
 * @param  data_len: length of data
 * @retval CRC 16 bits
 */
static uint16_t NAND_CheckCrc16(uint16_t crc, uint8_t *data_in,
				uint32_t data_len)
{
	/* Algorithm from ONFI standard */

	uint32_t i, j, bit;

	for (i = 0; i < data_len; i++) {
		uint8_t curParam = *data_in++;

		for (j = 0x80; j != 0; j >>= 1) {
			bit = crc & 0x8000;
			crc <<= 1;

			if (curParam & j)
				bit ^= 0x8000;

			if (bit)
				crc ^= CRC_POLYNOM;
		}
		crc &= 0xFFFF;
	}

	return crc;
}

/*****************************************************************************
 *
 * Function:          Nand_ReadParameterPage
 *
 * Description:       This function reads the NAND parameter page
 *                    (command 0xEC)
 *                    The Read Parameter Page command retrieves
 *                    the data structure that describes
 *                    the targets organization, features, timings
 *                    and other behavioral parameters.
 *                    There may also be additional information
 *                    provided in an extended parameter page.
 *
 * Input parameters:  NAND_HandleTypeDef * hNand
 *
 * Output parameters: none
 *
 * Return:            Std_ReturnType
 *
 *****************************************************************************/
static Std_ReturnType Nand_ReadParameterPage(NAND_HandleTypeDef *hNand)
{
	uint32_t index;
	uint8_t  buffer[PARAM_PAGE_SIZE];
	uintptr_t deviceComMemAddr;
	uintptr_t deviceAttrMemAddr;
	int      i;
	uint16_t crc16;

	assert(hNand);

	/* Identify the device address */
	deviceComMemAddr = FLASH_COMMON_MEM_BASE;
	deviceAttrMemAddr = FLASH_ATTRIB_MEM_BASE;

	/* Send ONFI Parameter Page Read sequence */
	*(uint8_t *)(deviceComMemAddr | CMD_SECTION) = NAND_CMD_READ_PARAM_PAGE;
	*(uint8_t *)(deviceAttrMemAddr | ADDR_SECTION) = 0x00;

	udelay(NAND_RST_TIMEOUT);

	/*
	 * Read Parameter Page
	 * Note: There are three consecutive redondant copies
	 * of parameter page
	 */
	for (i = 0; i < 3; i++) {
		/* Read parameter page from NAND flash */
		/* Get data */
		for (index = 0; index < PARAM_PAGE_SIZE; index++)
			buffer[index] = *(uint8_t *)deviceComMemAddr;

		/*
		 * Bytes 0-4: Parameter page signature
		 * This field contains the parameter page signature.
		 * When two or more bytes of the signature are valid,
		 * then it denotes that a valid copy
		 * of the parameter page is present
		 */
		hNand->Info.Signature = (buffer[3] << 24) | (buffer[2] << 16) |
			(buffer[1] << 8) | buffer[0];
		crc16 =  (buffer[254] | buffer[255] << 8);

		if ((hNand->Info.Signature == ONFI_SIG_VALUE) &&
		    (NAND_CheckCrc16(CRC_INIT_VALUE, buffer,
				     PARAM_PAGE_SIZE - 2) == crc16))
			break;
	}

	if (i == 3) {
		/* Could not find ONFI parameter page */
		INFO("%s: No Onfi Parameter Page\n", __func__);
		return STD_NOT_OK;
	}

	/* Byte 6, bit 0: data bus width (8 or 16) */
	hNand->Info.BusWidth = buffer[6] & 0x1;

	/* Bytes 80-83: Page size */
	hNand->Info.PageSize = (buffer[83] << 24) | (buffer[82] << 16) |
		(buffer[81] << 8) | buffer[80];

	/* Bytes 92-95 : Block size in number of pages */
	hNand->Info.BlockSize = (buffer[95] << 24) | (buffer[94] << 16) |
		(buffer[93] << 8) | buffer[92];

	/* Byte 96-99: Number of blocks per logical unit */
	hNand->Info.BlockNb = (buffer[99] << 24) | (buffer[98] << 16) |
		(buffer[97] << 8) | buffer[96];

	/*
	 * Byte 112: Number of bits ECC correctability
	 * This field indicates the number of bits that the host
	 * should be able to correct per 512 bytes of data
	 */
	if (buffer[112] != 0xFF) {
		hNand->Info.ECCcorrectability = buffer[112];
	} else if ((buffer[4] != 1) && (buffer[4] != 2) &&
		   buffer[6] & (1 << 7)) {
		/* NAND ONFI, version > 2.1, with extended parameter page */
		uint32_t ext_page_size = ((buffer[13] << 8) | buffer[12]);
		uint32_t ext_page_offset = buffer[14] * PARAM_PAGE_SIZE;
		uint32_t ecc_block_offset = 32;

		/* Send ONFI Parameter Page Read sequence */
		*(uint8_t *)(deviceComMemAddr | CMD_SECTION) =
			NAND_CMD_READ_PARAM_PAGE;
		*(uint8_t *)(deviceAttrMemAddr | ADDR_SECTION) = 0x00;

		/* Skip ONFI parameter pages to read Extended parameter page */
		*(uint8_t *)(deviceComMemAddr | CMD_SECTION) =
			NAND_CMD_CHANGE_1ST;

		*(uint8_t *)(deviceComMemAddr | ADDR_SECTION) =
			ADDR_1ST_CYCLE(ext_page_offset);

		*(uint8_t *)(deviceAttrMemAddr | ADDR_SECTION) =
			ADDR_2ND_CYCLE(ext_page_offset);

		*(uint8_t *)(deviceAttrMemAddr | CMD_SECTION) =
			NAND_CMD_CHANGE_2ND;

		udelay(NAND_RST_TIMEOUT);

		/* Copy Extended parameter page */
		for (index = 0; index < PARAM_PAGE_SIZE; index++)
			buffer[index] = *(uint8_t *)deviceComMemAddr;

		if (((buffer[5] << 24) | (buffer[4] << 16) |
		     (buffer[3] << 8) | buffer[2]) != EXT_PAGE_SIG_VALUE) {
			WARN("Extended parameter page signature is wrong\n");
			return STD_NOT_OK;
		}

		/*
		 * Extended parameter page can fit in buffer
		 * we can try to calculate its CRC
		 */
		if (ext_page_size <= PARAM_PAGE_SIZE) {
			crc16 = (buffer[1] << 8) | buffer[0];

			if (NAND_CheckCrc16(CRC_INIT_VALUE, buffer,
					    ext_page_size) != crc16) {
				WARN("Extended page CRC failed\n");
				return STD_NOT_OK;
			}
		}

		/*
		 * Check the 8 sections of extended parameter page
		 * for an extended ECC information block (type = 2)
		 */
		for (i = 0; i < 8; i++) {
			if (buffer[16 + 2 * i] == 2)
				break;

			ecc_block_offset += buffer[17 + 2 * i];
		}

		if (i == 8) {
			WARN("ECC extended block could not be found\n");
			return STD_NOT_OK;
		}

		if (ext_page_size <= PARAM_PAGE_SIZE) {
			/*
			 * ECC correctcatbility is the first byte
			 * of the ECC block
			 */
			hNand->Info.ECCcorrectability =
				buffer[ecc_block_offset];
		} else {
			/* Send ONFI Parameter Page Read sequence */
			*(uint8_t *)(deviceComMemAddr | CMD_SECTION) =
				NAND_CMD_READ_PARAM_PAGE;
			*(uint8_t *)(deviceAttrMemAddr | ADDR_SECTION) = 0x00;

			/*
			 * Skip ONFI parameter pages to read
			 * Extended parameter page
			 */
			*(uint8_t *)(deviceComMemAddr | CMD_SECTION) =
				NAND_CMD_CHANGE_1ST;
			*(uint8_t *)(deviceComMemAddr | ADDR_SECTION) =
				ADDR_1ST_CYCLE(ext_page_offset +
					       ecc_block_offset);

			*(uint8_t *)(deviceAttrMemAddr | ADDR_SECTION) =
				ADDR_2ND_CYCLE(ext_page_offset +
					       ecc_block_offset);

			*(uint8_t *)(deviceAttrMemAddr | CMD_SECTION) =
				NAND_CMD_CHANGE_2ND;

			udelay(NAND_RST_TIMEOUT);

			hNand->Info.ECCcorrectability =
				*(uint8_t *)deviceComMemAddr;
		}
	} else {
		WARN("ECC correctability could not be found\n");
		return STD_NOT_OK;
	}

	return STD_OK;
}

/*****************************************************************************
 *
 * Function:            Nand_DetectAndInit
 *
 * Description:         This function initializes the Nand FMC driver.
 *                      Nand detection and initialization.
 *
 * Input parameters:    NAND_HandleTypeDef * hNand
 *
 * Output parameters:   none
 *
 * Return:              Std_ReturnType
 *
 *****************************************************************************/
static Std_ReturnType Nand_DetectAndInit(NAND_HandleTypeDef *hNand)
{
	NAND_IDTypeDef pNAND_ID;
	uint32_t nand_param_in_otp;

	assert(hNand);

	/* Remark : ReadID and ReadParameterPage are commands on 8-bit */

	/* ReadID code */
	Nand_ReadIDCode(hNand, &pNAND_ID);

	/* Check if NAND parameters are stored in OTP */
	if (stm32_get_otp_value(NAND_OTP, &nand_param_in_otp) != 0) {
		return STD_NOT_OK;
	}

	if (nand_param_in_otp & NAND_PARAM_STORED_IN_OTP) {
		/*
		 * NAND parameter shall be read from OTP
		 */
		hNand->Info.BusWidth = (nand_param_in_otp & NAND_WIDTH_MASK) >>
			NAND_WIDTH_OFFSET;

		switch ((nand_param_in_otp & NAND_PAGE_SIZE_MASK) >>
			NAND_PAGE_SIZE_OFFSET) {
		case NAND_PAGE_SIZE_2K:
			hNand->Info.PageSize = 2048;
			break;

		case NAND_PAGE_SIZE_4K:
			hNand->Info.PageSize = 4096;
			break;

		case NAND_PAGE_SIZE_8K:
			hNand->Info.PageSize = 8192;
			break;

		default:
			hNand->Info.PageSize = 0;
			ERROR("Cannot read NAND page size\n");
			return STD_NOT_OK;
		}

		switch ((nand_param_in_otp & NAND_BLOCK_SIZE_MASK) >>
			NAND_BLOCK_SIZE_OFFSET) {
		case NAND_BLOCK_SIZE_64_PAGES:
			hNand->Info.BlockSize = 64;
			break;

		case NAND_BLOCK_SIZE_128_PAGES:
			hNand->Info.BlockSize = 128;
			break;

		case NAND_BLOCK_SIZE_256_PAGES:
			hNand->Info.BlockSize = 256;
			break;

		default:
			hNand->Info.BlockSize = 0;
			ERROR("Cannot read NAND block size\n");
			return STD_NOT_OK;
		}

		hNand->Info.BlockNb = ((nand_param_in_otp &
					NAND_BLOCK_NB_MASK) >>
				       NAND_BLOCK_NB_OFFSET) *
			NAND_BLOCK_NB_UNIT;

		switch ((nand_param_in_otp & NAND_ECC_BIT_NB_MASK) >>
			NAND_ECC_BIT_NB_OFFSET) {
		case NAND_ECC_BIT_NB_UNSET:
			hNand->Info.ECCcorrectability = 0;
			break;

		case NAND_ECC_BIT_NB_1_BITS:
			hNand->Info.ECCcorrectability = 1;
			break;

		case NAND_ECC_BIT_NB_4_BITS:
			hNand->Info.ECCcorrectability = 4;
			break;

		case NAND_ECC_BIT_NB_8_BITS:
			hNand->Info.ECCcorrectability = 8;
			break;

		default:
			hNand->Info.ECCcorrectability = 0;
			ERROR("Cannot read ECCbit number\n");
			return STD_NOT_OK;
		}
	} else {
		/*
		 * ONFI or 'ONFI compliant' NAND
		 * 'ONFI compliant' means that parameters used by bootrom are
		 * available in parameter table at same offsets as in a
		 * ONFI parameter table.
		 * NAND parameter shall be read from Parameter table.
		 */

		uint32_t onfi_ecc;

		/* Read Nand parameter page */
		if (Nand_ReadParameterPage(hNand) != STD_OK) {
			ERROR("%s: NAND not initialized\n", __func__);
			return STD_NOT_OK;
		}

		onfi_ecc = hNand->Info.ECCcorrectability;

		/*
		 * For ONFI nand, ECC number of bits may be overridden by a
		 * value from OTP configuration.
		 */
		switch ((nand_param_in_otp & NAND_ECC_BIT_NB_MASK) >>
			NAND_ECC_BIT_NB_OFFSET) {
		case NAND_ECC_BIT_NB_UNSET:
			hNand->Info.ECCcorrectability = 0;
			break;

		case NAND_ECC_BIT_NB_1_BITS:
			hNand->Info.ECCcorrectability = 1;
			break;

		case NAND_ECC_BIT_NB_4_BITS:
			hNand->Info.ECCcorrectability = 4;
			break;

		case NAND_ECC_BIT_NB_8_BITS:
			hNand->Info.ECCcorrectability = 8;
			break;

		default:
			hNand->Info.ECCcorrectability = 0;
			break;
		}

		/*
		 * if OTP info is wrong, fall back to parameter read in ONFI
		 * parameter page
		 */
		if (hNand->Info.ECCcorrectability == 0)
			hNand->Info.ECCcorrectability = onfi_ecc;
	}

	Nand_Init(hNand, hNand->Info.BusWidth, hNand->Info.ECCcorrectability);

	/* Now NAND Flash is detected and initialized */
	return STD_OK;
}

/**
 * @brief  NAND read page command sequence
 * @param  hNand: pointer to a NAND_HandleTypeDef structure that contains
 *                the configuration information for NAND module.
 * @param colAddr: column address
 * @param rowAddr: row address
 * @retval Std_ReturnType
 */
static void NAND_Read_Page_Cmd(NAND_HandleTypeDef *hNand, uint32_t colAddr,
			       uint32_t rowAddr)
{
	uintptr_t deviceComMemAddr;
	uintptr_t deviceAttrMemAddr;

	assert(hNand);

	/* Identify the device address */
	deviceComMemAddr = FLASH_COMMON_MEM_BASE;
	deviceAttrMemAddr = FLASH_ATTRIB_MEM_BASE;

	/* If NAND 8bit */
	if (hNand->Info.BusWidth == EIGHT_BIT_ACCESS) {
		*(__IO uint8_t *)(deviceComMemAddr | CMD_SECTION) =
			NAND_CMD_READ_1ST;

		/* C1 */
		*(__IO uint8_t *)(deviceComMemAddr | ADDR_SECTION) =
			ADDR_1ST_CYCLE(colAddr);
		/* C2 */
		*(__IO uint8_t *)(deviceComMemAddr | ADDR_SECTION) =
			ADDR_2ND_CYCLE(colAddr);
		/* R1 */
		*(__IO uint8_t *)(deviceComMemAddr | ADDR_SECTION) =
			ADDR_1ST_CYCLE(rowAddr);
		/* R2 */
		*(__IO uint8_t *)(deviceComMemAddr | ADDR_SECTION) =
			ADDR_2ND_CYCLE(rowAddr);

		/* R3 */
		*(__IO uint8_t *)(deviceAttrMemAddr | ADDR_SECTION) =
			ADDR_3RD_CYCLE(rowAddr);

		*(__IO uint8_t *)(deviceAttrMemAddr | CMD_SECTION) =
			NAND_CMD_READ_2ND;
	} else {
		/* NAND 16bit */
		*(__IO uint16_t *)(deviceComMemAddr | CMD_SECTION) =
			NAND_CMD_READ_1ST;

		/* C1 */
		*(__IO uint16_t *)(deviceComMemAddr | ADDR_SECTION) =
			ADDR_1ST_CYCLE(colAddr);
		/* C2 */
		*(__IO uint16_t *)(deviceComMemAddr | ADDR_SECTION) =
			ADDR_2ND_CYCLE(colAddr);
		/* R1 */
		*(__IO uint16_t *)(deviceComMemAddr | ADDR_SECTION) =
			ADDR_1ST_CYCLE(rowAddr);
		/* R2 */
		*(__IO uint16_t *)(deviceComMemAddr | ADDR_SECTION) =
			ADDR_2ND_CYCLE(rowAddr);

		/* R3 */
		*(__IO uint16_t *)(deviceAttrMemAddr | ADDR_SECTION) =
			ADDR_3RD_CYCLE(rowAddr);

		*(__IO uint16_t *)(deviceAttrMemAddr | CMD_SECTION) =
			NAND_CMD_READ_2ND;
	}
}

/*****************************************************************************
 *
 * Function:            _bit_count
 *
 * Description:         Return the number of bits set to one in a 32bit word
 *
 * Input parameters:    v    32 bit word
 *
 * Return:              number of bits set to one in v.
 *
 *****************************************************************************/
static uint32_t _bit_count(uint32_t v)
{
	uint32_t c = 0;

	for (uint32_t i = 0; i < CHAR_BIT * sizeof(typeof(v)); i++)
		if (v & (BIT(i)))
			c++;

	return c;
}

/*****************************************************************************
 *
 * Function:            NAND_Hamming_Correction
 *
 * Description:         Consider Hamming correction code, and apply correction
 *                      if needed and possible.
 *
 * Input parameters:    Buffer : pointer to buffer containing data read.
 *                      EccBuffer : pointer to buffer containing ecc code read
 *                                  from out of band area.
 *                      EccCalculated : ECC calculated by FMC during reading of
 *					data.
 *
 * Return:              0 no error detected
 *                      1 one error detected and correted
 *                      > 1 unrecoverable error
 *                      -1 ecc error
 *
 *****************************************************************************/
static int NAND_Hamming_Correction(uint8_t *Buffer, uint8_t *EccBuffer,
				   uint32_t EccCalculated)
{
	uint8_t xor_ecc_ones;
	uint16_t xor_ecc_1b, xor_ecc_2b, xor_ecc_3b;
	union {
		uint32_t val;
		uint8_t  bytes[4];
	} xor_ecc;

	/* Page size--------ECC_Code Size
	 * 256---------------22 bits LSB  (ECC_CODE & 0x003FFFFF)
	 * 512---------------24 bits      (ECC_CODE & 0x00FFFFFF)
	 * 1024--------------26 bits      (ECC_CODE & 0x03FFFFFF)
	 * 2048--------------28 bits      (ECC_CODE & 0x0FFFFFFF)
	 * 4096--------------30 bits      (ECC_CODE & 0x3FFFFFFF)
	 * 8192--------------32 bits      (ECC_CODE & 0xFFFFFFFF)
	 */

	/* For Page size 512, ECC_Code size 24 bits */
	xor_ecc_1b = (EccCalculated & 0x000000FF) ^ EccBuffer[0];
	xor_ecc_2b = ((EccCalculated & 0x0000FF00) >> 8) ^ EccBuffer[1];
	xor_ecc_3b = ((EccCalculated & 0x00FF0000) >> 16) ^ EccBuffer[2];

	xor_ecc.val = 0L;
	xor_ecc.bytes[2] = xor_ecc_3b;
	xor_ecc.bytes[1] = xor_ecc_2b;
	xor_ecc.bytes[0] = xor_ecc_1b;

	if (xor_ecc.val == 0)
		return 0; /* No Error */

	xor_ecc_ones = _bit_count(xor_ecc.val);
	if (xor_ecc_ones < 23) {
		if (xor_ecc_ones == 12) {
			uint16_t bit_address, byte_address;

			/* Correctable ERROR */
			bit_address = ((xor_ecc_1b >> 1) & 0x01) |
				      ((xor_ecc_1b >> 2) & 0x02) |
				      ((xor_ecc_1b >> 3) & 0x04);

			byte_address = ((xor_ecc_1b >> 7) & 0x01) |
				       ((xor_ecc_2b)      & 0x02) |
				       ((xor_ecc_2b >> 1) & 0x04) |
				       ((xor_ecc_2b >> 2) & 0x08) |
				       ((xor_ecc_2b >> 3) & 0x10) |
				       ((xor_ecc_3b << 4) & 0x20) |
				       ((xor_ecc_3b << 3) & 0x40) |
				       ((xor_ecc_3b << 2) & 0x80) |
				       ((xor_ecc_3b << 1) & 0x100);

			/* Correct bit error in the data */
			Buffer[byte_address] = (Buffer[byte_address]) ^
				((uint8_t)(1 << bit_address));
			INFO("Hamming: 1 ECC error corrected\n");
			return 1;
		}

		/* Non Correctable ERROR */
		ERROR("%s: Uncorrectable ECC Errors\n", __func__);
		return 2;
	}

	/* ECC ERROR */
	ERROR("%s: Hamming correction error\n", __func__);
	return -1;
}

/*****************************************************************************
 *
 * Function:            NAND_Read_Logical_Page
 *
 * Description:         Read 512B sector from NAND memory page
 *
 * Input parameters:    hNand: pointer to a NAND_HandleTypeDef structure
 *                      that contains the configuration information
 *                      for NAND module.
 *                      Address : pointer to NAND address structure
 *                      Buffer : pointer to destination read buffer
 *                      bch_sector_nb : 512B sector number in the page
 *
 *
 * Return:              Std_ReturnType
 *
 *****************************************************************************/
Std_ReturnType NAND_Read_Logical_Page(NAND_HandleTypeDef *hNand,
				      NAND_AddressTypeDef *Address,
				      uint8_t *Buffer, uint32_t bch_sector_nb)
{
	uintptr_t deviceComMemAddr;
	uintptr_t deviceAttrMemAddr;
	uint32_t size = 0;
	uint32_t index, bloc_nb, nb_pages_per_block, offset;
	uint32_t ecc_size = 0;
	uint32_t rowAddr = 0, colAddr = 0;
	uint8_t EccBuffer[NAND_ECC_BCH8_BYTES_NB_16b];
	uint32_t heccr = 0;

	assert(hNand);

	/* Identify the device address */
	deviceComMemAddr = FLASH_COMMON_MEM_BASE;
	deviceAttrMemAddr = FLASH_ATTRIB_MEM_BASE;

	bloc_nb = Address->Block;
	nb_pages_per_block = hNand->Info.BlockSize;

	/* Page and block to be read */
	rowAddr = (bloc_nb * nb_pages_per_block) + Address->Page;

	/* If NAND 8bit */
	if (hNand->Info.BusWidth == EIGHT_BIT_ACCESS) {
		/* Byte or word in page to be read */
		colAddr = bch_sector_nb * NAND_ECC_PAGE_SECTOR;

		size = NAND_ECC_PAGE_SECTOR;
	} else {
		/* If NAND 16bit */
		/* Byte or word in page to be read */
		colAddr = (bch_sector_nb * NAND_ECC_PAGE_SECTOR) / 2;

		size = NAND_ECC_PAGE_SECTOR / 2;
	}

	/* Reset ECC enabling */
	hNand->Instance->PCReg &= ~FMC_PCR_ECCEN;
	hNand->Instance->PCReg &= ~FMC_PCR_WE;
	hNand->Instance->PCReg |= FMC_PCR_ECCEN;

	/* Clear status */
	hNand->Instance->BCHICR = 0x1F;

	/* Send read page command sequence */
	NAND_Read_Page_Cmd(hNand, colAddr, rowAddr);

	/* Get data into destination buffer */
	/* If NAND 8bit */
	if (hNand->Info.BusWidth == EIGHT_BIT_ACCESS) {
		uint8_t *Buffer8b = (uint8_t *)Buffer;

		for (index = 0; index < size; index++)
			*Buffer8b++ = *(uint8_t *)deviceComMemAddr;
	} else {
		/* If NAND 16bit */
		uint16_t *Buffer16b = (uint16_t *)Buffer;

		for (index = 0; index < size; index++)
			*Buffer16b++ = *(uint16_t *)deviceComMemAddr;
	}

	if (hNand->Info.ECCcorrectability == NAND_ECC_HAMMING1) {
		/* During read of data , syndrome is calculated */
		/* Wait until decoding error is ready */
		uint32_t timerValInit = read_cntpct_el0();

		while ((hNand->Instance->SR & FMC_SR_NWRF) != FMC_SR_NWRF) {
			if ((read_cntpct_el0() - timerValInit) >
			    NAND_ECC_CALCULATION_TIMEOUT_VAL_250MS) {
				ERROR("%s: syndrome calculation timeout\n",
				      __func__);
				return STD_NOT_OK;
			}
		}

		heccr = hNand->Instance->HECCR;
	}

	/* Read now corresponding ECC bytes (7 or 13) in spare area */
	/* Page and block to be read */

	if (hNand->Info.ECCcorrectability == NAND_ECC_BCH4) {
		if (hNand->Info.BusWidth == EIGHT_BIT_ACCESS)
			ecc_size = NAND_ECC_BCH4_BYTES_NB_8b;
		else
			ecc_size = NAND_ECC_BCH4_BYTES_NB_16b;
	} else if (hNand->Info.ECCcorrectability == NAND_ECC_BCH8) {
		if (hNand->Info.BusWidth == EIGHT_BIT_ACCESS)
			ecc_size = NAND_ECC_BCH8_BYTES_NB_8b;
		else
			ecc_size = NAND_ECC_BCH8_BYTES_NB_16b;
	} else {
		if (hNand->Info.BusWidth == EIGHT_BIT_ACCESS)
			ecc_size = NAND_ECC_HAMMING1_BYTES_NB_8b;
		else
			ecc_size = NAND_ECC_HAMMING1_BYTES_NB_16b;
	}

	offset = 2;

	/* If NAND 8bit */
	if (hNand->Info.BusWidth == EIGHT_BIT_ACCESS) {
		/* Byte or word in page to be read */
		/* See SRS mapping */
		colAddr = hNand->Info.PageSize + offset +
			(bch_sector_nb * ecc_size);
	} else {
		/* If NAND 16bit */
		/* Byte or word in page to be read */
		/* See SRS mapping */
		colAddr = (hNand->Info.PageSize + offset +
			   (bch_sector_nb * ecc_size)) / 2;
	}

	/* Send change read column command */
	if (hNand->Info.BusWidth == EIGHT_BIT_ACCESS) {
		*(__IO uint8_t *)(deviceComMemAddr | CMD_SECTION) =
			NAND_CMD_CHANGE_1ST;

		/* C1 */
		*(__IO uint8_t *)(deviceComMemAddr | ADDR_SECTION) =
			ADDR_1ST_CYCLE(colAddr);
		/* C2 */
		*(__IO uint8_t *)(deviceAttrMemAddr | ADDR_SECTION) =
			ADDR_2ND_CYCLE(colAddr);

		*(__IO uint8_t *)(deviceAttrMemAddr | CMD_SECTION) =
			NAND_CMD_CHANGE_2ND;
	} else {
		*(__IO uint16_t *)(deviceComMemAddr | CMD_SECTION) =
			NAND_CMD_CHANGE_1ST;

		/* C1 */
		*(__IO uint16_t *)(deviceComMemAddr | ADDR_SECTION) =
			ADDR_1ST_CYCLE(colAddr);
		/* C2 */
		*(__IO uint16_t *)(deviceAttrMemAddr | ADDR_SECTION) =
			ADDR_2ND_CYCLE(colAddr);

		*(__IO uint16_t *)(deviceAttrMemAddr | CMD_SECTION) =
			NAND_CMD_CHANGE_2ND;
	}

	/* Get data into destination EccBuffer */

	/* If NAND 8bit */
	if (hNand->Info.BusWidth == EIGHT_BIT_ACCESS) {
		uint8_t  *pEccBuffer = EccBuffer;

		for (index = 0; index < ecc_size; index++)
			*(uint8_t *)pEccBuffer++ = *(uint8_t *)deviceComMemAddr;
	} else {
		/* If NAND 16bit */
		uint16_t *pEccBuffer16b = (uint16_t *)(EccBuffer);

		for (index = 0; index < ecc_size / 2; index++) {
			*(uint16_t *)pEccBuffer16b++ =
				*(uint16_t *)deviceComMemAddr;
		}
	}

	if (hNand->Info.ECCcorrectability == NAND_ECC_HAMMING1) {
		int nb_errors;

		nb_errors = NAND_Hamming_Correction(Buffer, EccBuffer,
						    heccr);
		if ((nb_errors != 0) && (nb_errors != 1))
			return STD_NOT_OK;

	} else /* BCH error correction */ {
		uint32_t ecc_errors_nb;
		uint32_t i;
		uint32_t errorPosition[8];

		/*
		 * During read of data and parity bits, syndrome is calculated,
		 * then error location is launched
		 * Wait until decoding error is ready
		 */
		uint32_t timerValInit = (uint32_t)read_cntpct_el0();

		while ((hNand->Instance->BCHISR & FMC_BCHISR_DERF) !=
		       FMC_BCHISR_DERF) {
			if (((uint32_t)read_cntpct_el0() - timerValInit) >
			    NAND_ECC_CALCULATION_TIMEOUT_VAL_250MS)
				return STD_NOT_OK;
		}

		/* Read decoding results */
		/* Check if there were uncorrectable errors */
		if (hNand->Instance->BCHISR & FMC_BCHISR_DUEF) {
			VERBOSE("%s: Uncorrectable ECC Error\n", __func__);
			return STD_NOT_OK;
		}

		/* Check if there were errors */
		if (!(hNand->Instance->BCHISR & FMC_BCHISR_DEFF))
			return STD_OK;

		/* Read number of errors */
		ecc_errors_nb = hNand->Instance->BCHSR & FMC_BCHSR_DEN;

		VERBOSE("%s: ECC Errors detected: %d\n", __func__,
			ecc_errors_nb);

		/* Retrieve the error position corresponding to the error
		 * number
		 */
		/* Error 1 position : FMC_BCHDSR1.EBP1 */
		errorPosition[0] = hNand->Instance->BCHDSR1 & FMC_BCHDSR1_EBP1;
		/* Error 2 position : FMC_BCHDSR1.EBP2 */
		errorPosition[1] = (hNand->Instance->BCHDSR1 & FMC_BCHDSR1_EBP2)
			>> FMC_EBP2_MASK;
		/* Error 3 position : FMC_BCHDSR2.EBP1 */
		errorPosition[2] = hNand->Instance->BCHDSR2 & FMC_BCHDSR2_EBP1;
		/* Error 4 position : FMC_BCHDSR2.EBP2 */
		errorPosition[3] = (hNand->Instance->BCHDSR2 & FMC_BCHDSR2_EBP2)
			>> FMC_EBP2_MASK;
		/* Error 5 position : FMC_BCHDSR3.EBP1 */
		errorPosition[4] = hNand->Instance->BCHDSR3 & FMC_BCHDSR3_EBP1;
		/* Error 6 position : FMC_BCHDSR3.EBP2 */
		errorPosition[5] = (hNand->Instance->BCHDSR3 & FMC_BCHDSR3_EBP2)
			>> FMC_EBP2_MASK;
		/* Error 7 position : FMC_BCHDSR4.EBP1 */
		errorPosition[6] = hNand->Instance->BCHDSR4 & FMC_BCHDSR4_EBP1;
		/* Error 8 position : FMC_BCHDSR4.EBP2 */
		errorPosition[7] = (hNand->Instance->BCHDSR4 & FMC_BCHDSR4_EBP2)
			>> FMC_EBP2_MASK;

		/* Error position indicates error bit number in binary */
		/* Retrieve the mask of the wrong bit to correct */
		for (i = 0; i < ecc_errors_nb; i++) {
			VERBOSE("%s: ECC Error position: %d\n", __func__,
				errorPosition[i]);
			/* Correct only if error is in data area */
			if (errorPosition[i] < 0x1000) {
				/*
				 * Retrieve the mask of the wrong bit
				 * to correct
				 */
				uint32_t bitMask = BIT(errorPosition[i] & 0x07);

				/*
				 * Remove bit postion in the error position
				 * (to have byte to correct)
				 */
				errorPosition[i] >>= 0x03;
				VERBOSE("%s: ECC Error in data area\n",
					__func__);

				/* Fix the error : invert the wrong bit */
				*(Buffer + errorPosition[i]) ^= bitMask;
			} else {
				VERBOSE("%s: ECC Error not in data area\n",
					__func__);
			}
		}
	}

	return STD_OK;
}

/**
 * @brief  NAND check bad blck
 * @param  hNand: pointer to a NAND_HandleTypeDef structure that contains
 *                the configuration information for NAND module.
 *         Address: pointer to NAND address structure
 * @retval BAD_BLOCK or GOOD_BLOCK
 */
uint32_t NAND_Check_Bad_Block(NAND_HandleTypeDef *hNand,
			      NAND_AddressTypeDef *Address)
{
	uintptr_t deviceComMemAddr = 0;
	uint32_t bloc_nb = 0;
	uint32_t nb_pages_per_block = 0;
	uint32_t page_nb;
	uint32_t block_status = BAD_BLOCK;
	uint32_t rowAddr = 0, colAddr = 0;
	uint8_t  bbm_marker, bbm_marker_2;
	uint16_t  bbm_marker16b;

	assert(hNand);

	/* Identify the device address */
	deviceComMemAddr = FLASH_COMMON_MEM_BASE;

	/*
	 * Bad block indication is is the 2 first bytes of the 1st
	 * or 2nd page of the block (depends on manufacturer)
	 */

	/*
	 * Read the 2 first bytes of the spare area
	 * of the 1st page of the block
	 */
	bloc_nb = Address->Block;
	nb_pages_per_block = hNand->Info.BlockSize;

	/* Page and block to be read */
	page_nb = 0;
	rowAddr = (bloc_nb * nb_pages_per_block) + page_nb;

	/* If NAND 8bit */
	if (hNand->Info.BusWidth == EIGHT_BIT_ACCESS) {
		/* Byte or word in page to be read */
		colAddr = hNand->Info.PageSize;
	} else {
		/* If NAND 16bit */
		/* Byte or word in page to be read */
		colAddr = hNand->Info.PageSize / 2;
	}

	/* Send read page command sequence */
	NAND_Read_Page_Cmd(hNand, colAddr, rowAddr);

	udelay(NAND_RST_TIMEOUT);

	/* Get data into destination buffer */
	/* If NAND 8bit */
	if (hNand->Info.BusWidth == EIGHT_BIT_ACCESS) {
		bbm_marker = *(uint8_t *)deviceComMemAddr;
		bbm_marker_2 = *(uint8_t *)deviceComMemAddr;

		if ((bbm_marker != 0xFF) || (bbm_marker_2 != 0xFF))
			block_status = BAD_BLOCK;
		else
			block_status = GOOD_BLOCK;

	} else {
		/* If NAND 16bit */
		bbm_marker16b = *(uint16_t *)deviceComMemAddr;

		if (bbm_marker16b != 0xFFFF)
			block_status = BAD_BLOCK;
		else
			block_status = GOOD_BLOCK;
	}

	if (block_status == BAD_BLOCK)
		return block_status;

	/*
	 * Read the 2 first bytes of the spare area of the 2nd page
	 * of the block
	 * Page and block to be read
	 */
	page_nb = 1;
	rowAddr = (bloc_nb * nb_pages_per_block) + page_nb;

	/* Send read page command sequence */
	NAND_Read_Page_Cmd(hNand, colAddr, rowAddr);

	udelay(NAND_RST_TIMEOUT);

	/* Get data into destination buffer */
	/* If NAND 8bit */
	if (hNand->Info.BusWidth == EIGHT_BIT_ACCESS) {
		bbm_marker = *(uint8_t *)deviceComMemAddr;
		bbm_marker_2 = *(uint8_t *)deviceComMemAddr;

		if ((bbm_marker == 0xFF) && (bbm_marker_2 == 0xFF))
			block_status = GOOD_BLOCK;
		else
			block_status = BAD_BLOCK;
	} else {
		/* If NAND 16bit */
		bbm_marker16b = *(uint16_t *)deviceComMemAddr;

		if (bbm_marker16b == 0xFFFF)
			block_status = GOOD_BLOCK;
		else
			block_status = BAD_BLOCK;
	}

	return block_status;
}

/**
 * @brief  Increment the NAND memory address
 * @param  hNand: pointer to a NAND_HandleTypeDef structure that contains
 *                the configuration information for NAND module.
 * @param Address: pointer to NAND address structure
 * @retval Std_ReturnType
 */
Std_ReturnType NAND_Address_Inc(NAND_HandleTypeDef *hNand,
				NAND_AddressTypeDef *Address,
				uint32_t numPagesRead)
{
	assert(hNand);

	/* Increment page address */
	if ((numPagesRead % (hNand->Info.PageSize / BCH_PAGE_SECTOR)) == 0)
		Address->Page++;

	/* Check NAND address is valid */
	if (Address->Page == hNand->Info.BlockSize) {
		Address->Page = 0;
		/* Search for next valid block */
		Address->Block++;
		while (Address->Block < hNand->Info.BlockNb &&
		       NAND_Check_Bad_Block(hNand, Address) == BAD_BLOCK)
			Address->Block++;

		if (Address->Block == hNand->Info.BlockNb)
			return STD_NOT_OK;
	}

	return STD_OK;
}

/**
 * @brief  Initialize driver only if needed:
 *	   bootrom must have initialize NAND and bootcontext
 *	   must contains correct value.
 * @param  hNand: pointer to a NAND_HandleTypeDef structure that contains
 *                the configuration information for NAND module.
 * @retval Std_ReturnType
 */
Std_ReturnType nand_initialize(NAND_HandleTypeDef *hnand)
{
	if (!hnand->Info.PageSize) {
		INFO("Reconfigure NAND\n");
		/* Not properly initialized by bootrom */
		Nand_Init(hnand, EIGHT_BIT_ACCESS, NAND_ECC_BCH8);

		if (Nand_Reset(hnand) != STD_OK) {
			ERROR("nand: Reset error (IP base::0x%lx)\n",
			      (uintptr_t)hnand->Instance);
			return STD_NOT_OK;
		}

		if (Nand_DetectAndInit(hnand) != STD_OK) {
			ERROR("nand: DetectAndInit error (IP base:0x%lx)\n",
			      (uintptr_t)hnand->Instance);
			return STD_NOT_OK;
		}
	} else {
		/* Initialization done, just need to set correct timing */
		Nand_Init(hnand, hnand->Info.BusWidth,
			  hnand->Info.ECCcorrectability);
	}
	return STD_OK;
}

