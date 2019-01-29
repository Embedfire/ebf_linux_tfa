/**
 ******************************************************************************
 * @file    hash_sec.h
 * @author  mgentilini - MCD IntroPack team - MPU AP v1 bootROM project
 * @version V0.1
 * @date    22-Sept-2015
 * @brief   Header file for STM32MP1 HASH driver module.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HASH_SEC_H
#define __HASH_SEC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32mp1xx_hal.h>
#include <stm32_hal_hash_reg.h>

/* Definitions----------------------------------------------------------------*/
#define HASH ((HASH_TypeDef *)HASH_BASE)
#define HASH_DIGEST ((HASH_DIGEST_TypeDef *)(HASH_BASE + 0x310))

#define HASH_TIMEOUT_VALUE         0x500000

/*
 * HASH IP uses 512 bits : ie 64 bytes block in order to start computation of
 * Hash intermediate Digest
 * The digest intermediate computation starts only after 64 bytes are entered
 */
#define HASH_BLOCK_SIZE_NB_BYTES   64

/**
 * @brief Set the number of valid bits in last word 32 bits written in Data
 *	  register
 * @param  SIZE: size in byte of last data written in Data register.
 * @retval None
 */

#define __HAL_HASH_SET_NBVALIDBITSINLAST32BITSWORD(SIZE) \
	do { \
		HASH->STR &= ~(HASH_STR_NBW); \
		HASH->STR |= 8 * ((SIZE) % 4); \
	} while (0)

/**
 * @brief  HASH Handle Structure definition
 */
typedef struct {
	HASH_TypeDef    *Instance;         /*!< HASH Registers base address */
	uint32_t        State;
	uint32_t        Error;
	const uint8_t   *pHashInBuffPtr;   /*!< Pointer to input buffer */
	const uint8_t   *pHashOutBuffPtr;  /*!< Pointer to output buffer */
} HASH_HandleTypeDef;

#define HASH_DIGEST_DONE    0x0
#define HASH_TIMEOUT        0x1
#define HASH_INIT_DONE      0x2
#define HASH_ACCU_DONE      0x3
#define HASH_FINISHED       0x4
#define HASH_INIT_NOT_DONE  0x5

/** @defgroup HASH_Algo_Selection
 * @{
 */
#define HASH_AlgoSelection_SHA256 HASH_CR_ALGO /*!< HASH function is SHA256 */

/**
 * @}
 */

/** @defgroup HASH_Data_Type
 * @{
 */
#define HASH_DATATYPE_8B HASH_CR_DATATYPE_1 /*!< 8-bit data. All bytes are
					     * swapped
					     */

/* Exported functions --------------------------------------------------------*/

/* HASH processing using polling  *********************************************/
Std_ReturnType HASH_SHA256_Init(HASH_HandleTypeDef *hHash);

Std_ReturnType HASH_SHA256_Start(HASH_HandleTypeDef *hHash,
				 const uint8_t *pInBuffer,
				 uint32_t sizeInBytes,
				 uint8_t *pOutBuffer, uint32_t Timeout);

Std_ReturnType HASH_SHA256_Accumulate(HASH_HandleTypeDef *hHash,
				      const uint8_t *pInBuffer,
				      uint32_t sizeInBytes);

Std_ReturnType HASH_SHA256_Finish(HASH_HandleTypeDef *hHash,
				  uint8_t *pOutBuffer, uint32_t Timeout);

#ifdef __cplusplus
}
#endif

#endif /* __HASH_SEC_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

