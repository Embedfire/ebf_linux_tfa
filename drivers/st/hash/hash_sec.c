/*
 * Copyright (c) 2015-2018, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <arch_helpers.h>
#include <assert.h>
#include <debug.h>
#include <hash_sec.h>
#include <platform_def.h>
#include <stdint.h>

#define RESET			0

#define timer_GetTimeSec()	(uint32_t)read_cntpct_el0()

static void HASH_WriteData(HASH_HandleTypeDef *hHash, const uint8_t *pInBuffer,
			   uint32_t SizeInBytes);
static void HASH_GetDigest(HASH_HandleTypeDef *hHash, uint8_t *pMsgDigest);
static uint32_t HASH_WaitBusyClearWithTimeout(HASH_HandleTypeDef *hHash,
					      uint32_t timeout);
static uint32_t HASH_WaitDcisClearWithTimeout(HASH_HandleTypeDef *hHash,
					      uint32_t timeout);

/*****************************************************************************
 *
 * Function:    HASH_SHA256_Init
 *
 * Description: Initializes the HASH peripheral
 *
 * Parameters:	hHash : pointer to a HASH_HandleTypeDef structure that contains
 *		the configuration information for HASH module
 *
 * Return:      none
 *
 *****************************************************************************/
Std_ReturnType HASH_SHA256_Init(HASH_HandleTypeDef *hHash)
{
	/* Check the hash handle allocation */
	if (!hHash) {
		return STD_NOT_OK;
	}

	hHash->Instance = (HASH_TypeDef *)HASH_BASE;

	/* Enable HASH1 clock */
	stm32mp_clk_enable(HASH1);

	hHash->Error = HASH_INIT_NOT_DONE;

	hHash->pHashInBuffPtr = NULL;

	hHash->pHashOutBuffPtr = NULL;

	/* Set the data type to 8 bits */
	hHash->Instance->CR |= HASH_DATATYPE_8B;

	/* Select the SHA256 mode and reset the HASH processor core,
	 * so that the HASH will be ready to compute the message digest
	 * of a new message
	 */
	hHash->Instance->CR |= HASH_AlgoSelection_SHA256 | HASH_CR_INIT;

	/* Return function status */
	hHash->Error = HASH_INIT_DONE;

	return STD_OK;
}

/*****************************************************************************
 *
 * Function:    HASH_SHA256_Start
 *
 * Description: Initializes the HASH peripheral in SHA256 mode
 *		then processes pInBuffer.
 *		The digest is available in pOutBuffer.
 *
 * Parameters:	hHash : pointer to a HASH_HandleTypeDef structure that contains
 *		the configuration information for HASH module
 *
 *		pInBuffer: Pointer to the input buffer (buffer to be hashed).
 *
 *		sizeInBytes: Length of the input buffer in bytes.
 *		If the Size is not multiple of 64 bytes,
 *		the padding is managed by hardware.
 *
 *		pOutBuffer: Pointer to the computed digest.
 *		Its size must be 32 bytes.
 *
 *		Timeout: Specify Timeout value
 *
 * Return:      none
 *
 *****************************************************************************/
Std_ReturnType HASH_SHA256_Start(HASH_HandleTypeDef *hHash,
				 const uint8_t *pInBuffer, uint32_t sizeInBytes,
				 uint8_t *pOutBuffer, uint32_t Timeout)
{
	uint32_t timeoutDetected = 0;

	/* Check the hash handle allocation */
	if (!hHash) {
		return STD_NOT_OK;
	}

	hHash->pHashInBuffPtr = pInBuffer;
	hHash->pHashOutBuffPtr = pOutBuffer;

	/* Configure the number of valid bits in last word 32 bits
	 * of the bit stream
	 */
	__HAL_HASH_SET_NBVALIDBITSINLAST32BITSWORD(sizeInBytes);

	/* Fill in entire input buffer to be hashed in HASH DIN register
	 * Note : intermediate digest is computed each time 64 bytes are written
	 * in HASH_DIN register
	 */
	HASH_WriteData(hHash, pInBuffer, sizeInBytes);

	/* Start the Final Digest calculation */
	hHash->Instance->STR |= HASH_STR_DCAL;

	/* Check for timeout */
	timeoutDetected = HASH_WaitBusyClearWithTimeout(hHash, Timeout);
	if (timeoutDetected != 0) {
		hHash->Error = HASH_TIMEOUT;
		return STD_NOT_OK;
	}

	/* Read the message digest in output buffer */
	HASH_GetDigest(hHash, pOutBuffer);

	hHash->Error = HASH_DIGEST_DONE;

	return STD_OK;
}

/*****************************************************************************
 *
 * Function:    HASH_SHA256_Accumulate
 *
 * Description: Initializes the HASH peripheral in SHA256 mode then processes
 *		pInBuffer.
 *
 * Parameters:	hHash: pointer to a HASH_HandleTypeDef structure that contains
 *              the configuration information for HASH module
 *
 *		pInBuffer: Pointer to the input buffer (buffer to be hashed).
 *
 *		sizeInBytes: Length of the input buffer in bytes.
 *		If the Size is not multiple of 64 bytes, the padding is managed
 *		by hardware.
 *
 * Return:	none
 *
 *****************************************************************************/
Std_ReturnType HASH_SHA256_Accumulate(HASH_HandleTypeDef *hHash,
				      const uint8_t *pInBuffer,
				      uint32_t sizeInBytes)
{
	/* Check the hash handle allocation */
	if (!hHash) {
		return STD_NOT_OK;
	}

	hHash->pHashInBuffPtr = pInBuffer;

	/* Configure the number of valid bits in last 32 bits word
	 * of the message
	 */
	__HAL_HASH_SET_NBVALIDBITSINLAST32BITSWORD(sizeInBytes);

	/* Write input buffer in data register */
	HASH_WriteData(hHash, pInBuffer, sizeInBytes);

	/* Return function status */
	hHash->Error = HASH_ACCU_DONE;

	return STD_OK;
}

/*****************************************************************************
 *
 * Function:	HASH_SHA256_Finish
 *
 * Description: Returns the computed digest in SHA256.
 *
 * Parameters:	hHash: pointer to a HASH_HandleTypeDef structure that contains
 *		the configuration information for HASH module
 *
 *		pOutBuffer: Pointer to the computed digest.
 *		Its size must be 32 bytes.
 *
 *		Timeout: Timeout value
 *
 * Return:	none
 *
 *****************************************************************************/
Std_ReturnType HASH_SHA256_Finish(HASH_HandleTypeDef *hHash,
				  uint8_t *pOutBuffer, uint32_t Timeout)
{
	uint32_t timeoutDetected = 0;

	/* Check the hash handle allocation */
	if (!hHash) {
		return STD_NOT_OK;
	}

	hHash->pHashOutBuffPtr = pOutBuffer;

	/* Check for the Timeout */
	timeoutDetected = HASH_WaitDcisClearWithTimeout(hHash, Timeout);
	if (timeoutDetected != 0) {
		hHash->Error = HASH_TIMEOUT;
		return STD_NOT_OK;
	}

	/* Read the message digest */
	HASH_GetDigest(hHash, pOutBuffer);

	/* Return function status */
	hHash->Error = HASH_FINISHED;

	return STD_OK;
}

/* Static functions ----------------------------------------------------------*/

/**
 * @brief  Writes the input buffer in HASH_DIN register.
 * @param  pInBuffer: Pointer to input buffer
 * @param  SizeInBytes : The size of input buffer (in bytes)
 * @retval None
 */
static void HASH_WriteData(HASH_HandleTypeDef *hHash, const uint8_t *pInBuffer,
			   uint32_t SizeInBytes)
{
	uint32_t *pInWord32 = (uint32_t *)pInBuffer;
	uint32_t nbBlocks = (SizeInBytes / HASH_BLOCK_SIZE_NB_BYTES);
	uint32_t remainingBytesInLastBlock = (SizeInBytes %
					      HASH_BLOCK_SIZE_NB_BYTES);
	uint32_t remainingBytesInLastWord = (SizeInBytes % 4);
	uint32_t idxBlock = 0;
	uint32_t idxWord32;

	/* For all blocks */
	for (idxBlock = 0; idxBlock < nbBlocks; idxBlock++) {
		for (idxWord32 = 0; idxWord32 < (HASH_BLOCK_SIZE_NB_BYTES / 4);
		     idxWord32++) {
			hHash->Instance->DIN = *pInWord32;
			pInWord32++;
		}

		/* Wait until end of computation of intermediate digest */
		while ((hHash->Instance->SR & HASH_SR_DINIS) == HASH_SR_DINIS) {
			;
		}
	}

	if (remainingBytesInLastBlock != 0) {
		uint32_t nbWords32Remaining = (remainingBytesInLastBlock / 4);

		for (idxWord32 = 0; idxWord32 < nbWords32Remaining;
		     idxWord32++) {
			hHash->Instance->DIN = *pInWord32;
			pInWord32++;
		}
	}

	if (remainingBytesInLastWord != 0) {
		hHash->Instance->DIN = *pInWord32;
	}
}

#ifndef __CC_ARM
/* __rev is a builtin command in ARM compiler
 * it needs to be remapped on __builtin_bswap32 for GCC
 */
#define __rev __builtin_bswap32
#endif

/**
 * @brief  Provides the message digest result.
 * @param  hHash: Hash handle
 * @param  pMsgDigest: Pointer to the message digest
 * @retval None
 */
static void HASH_GetDigest(HASH_HandleTypeDef *hHash, uint8_t *pMsgDigest)
{
	uintptr_t msgdigest = (uintptr_t)pMsgDigest;

	/* Read the message digest */
	*(uintptr_t *)(msgdigest) = __rev(hHash->Instance->HR[0]);
	msgdigest += 4;
	*(uintptr_t *)(msgdigest) = __rev(hHash->Instance->HR[1]);
	msgdigest += 4;
	*(uintptr_t *)(msgdigest) = __rev(hHash->Instance->HR[2]);
	msgdigest += 4;
	*(uintptr_t *)(msgdigest) = __rev(hHash->Instance->HR[3]);
	msgdigest += 4;
	*(uintptr_t *)(msgdigest) = __rev(hHash->Instance->HR[4]);
	msgdigest += 4;
	*(uintptr_t *)(msgdigest) = __rev(HASH_DIGEST->HR[5]);
	msgdigest += 4;
	*(uintptr_t *)(msgdigest) = __rev(HASH_DIGEST->HR[6]);
	msgdigest += 4;
	*(uintptr_t *)(msgdigest) = __rev(HASH_DIGEST->HR[7]);
}

/**
 * @brief   This function waits until bit HASH_SR_BUSY is 1b0' before a timeout.
 * @param   [in]     hHash           : hash handle
 * @param   [in]     timeout         : timeout value
 *
 * @retval  timeoutDetected : value 0x1 if timeout was detected while
 *          waiting for bit BUSY to be cleared.
 ******************************************************************************
 */

static uint32_t HASH_WaitBusyClearWithTimeout(HASH_HandleTypeDef *hHash,
					      uint32_t timeout)
{
	uint32_t timerValInit = 0;
	uint32_t timeoutDetected = 0;

	/* Get timer current value at start of loop */
	timerValInit = timer_GetTimeSec();

	while (((hHash->Instance->SR & HASH_SR_BUSY) == HASH_SR_BUSY) &&
	       (timeoutDetected == 0)) {
		/* Sense timeout occurrence */
		if ((timer_GetTimeSec() - timerValInit) >= timeout) {
			timeoutDetected = 1;
		}
	}

	return timeoutDetected;
}

/**
 * @brief   This function waits until bit HASH_SR_DCIS is 1b0' before a timeout.
 * @param   [in]     hHash           : hash handle
 * @param   [in]     timeout         : timeout value
 *
 * @retval  timeoutDetected : value 0x1 if timeout was detected while
 *          waiting for bit DCIS to be cleared.
 ******************************************************************************
 */

static uint32_t HASH_WaitDcisClearWithTimeout(HASH_HandleTypeDef *hHash,
					      uint32_t timeout)
{
	uint32_t timerValInit = 0;
	uint32_t timeoutDetected = 0;

	/* Get timer current value at start of loop */
	timerValInit = timer_GetTimeSec();

	while ((HAL_IS_BIT_CLR(hHash->Instance->SR, HASH_SR_DCIS)) &&
	       (timeoutDetected == 0)) {
		/* Sense timeout occurrence */
		if ((timer_GetTimeSec() - timerValInit) >= timeout) {
			timeoutDetected = 1;
		}
	}

	return timeoutDetected;
}
