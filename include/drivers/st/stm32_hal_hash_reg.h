/**
  ******************************************************************************
  * @file    stm32_hal_hash_reg.h
  * @author  MCD Intropack Team - MPU AP v1 bootROM project
  * @date    22 September 2015
  * @brief   STM32  Device Peripheral Access Layer Header File.
  *
  *          This file contains:
  *           - Data structures and the address mapping for all peripherals
  *           - Peripheral's registers declarations and bits definition
  *           - Macros to access peripherals registers hardware
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * SPDX-License-Identifier: BSD-3-Clause
  *
  ******************************************************************************
  */

#ifndef _STM32_HAL_HASH_REG_H_
#define _STM32_HAL_HASH_REG_H_

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */


/** @addtogroup Peripheral_registers_structures
  * @{
  */

/**
  * @brief Secure digital input/output Interface
  */
#ifndef __ASM_INCLUDE__
/** 
  * @brief HASH
  */
  
typedef struct 
{
  __IO uint32_t CR;               /*!< HASH control register,          Address offset: 0x00        */
  __IO uint32_t DIN;              /*!< HASH data input register,       Address offset: 0x04        */
  __IO uint32_t STR;              /*!< HASH start register,            Address offset: 0x08        */
  __IO uint32_t HR[5];            /*!< HASH digest registers,          Address offset: 0x0C-0x1C   */
  __IO uint32_t IMR;              /*!< HASH interrupt enable register, Address offset: 0x20        */
  __IO uint32_t SR;               /*!< HASH status register,           Address offset: 0x24        */
	uint32_t RESERVED[52];        /*!< Reserved, 0x28-0xF4                                         */
  __IO uint32_t CSR[54];          /*!< HASH context swap registers,    Address offset: 0x0F8-0x1CC */
} HASH_TypeDef;
#endif /* __ASM_INCLUDE__ */

#ifndef __ASM_INCLUDE__
/** 
  * @brief HASH_DIGEST
  */
  
typedef struct 
{
  __IO uint32_t HR[8];     /*!< HASH digest registers,          Address offset: 0x310-0x32C */ 
} HASH_DIGEST_TypeDef;
#endif /* __ASM_INCLUDE__ */





/** @addtogroup Exported_constants
  * @{
  */

  /** @addtogroup Peripheral_Registers_Bits_Definition
  * @{
  */
  
/******************  Bits definition for HASH_CR register  ********************/
#define HASH_CR_INIT                         ((uint32_t)0x00000004)
#define HASH_CR_DMAE                         ((uint32_t)0x00000008)
#define HASH_CR_DATATYPE                     ((uint32_t)0x00000030)
#define HASH_CR_DATATYPE_0                   ((uint32_t)0x00000010)
#define HASH_CR_DATATYPE_1                   ((uint32_t)0x00000020)
#define HASH_CR_MODE                         ((uint32_t)0x00000040)
#define HASH_CR_ALGO                         ((uint32_t)0x00040080)
#define HASH_CR_ALGO_0                       ((uint32_t)0x00000080)
#define HASH_CR_ALGO_1                       ((uint32_t)0x00040000)
#define HASH_CR_NBW                          ((uint32_t)0x00000F00)
#define HASH_CR_NBW_0                        ((uint32_t)0x00000100)
#define HASH_CR_NBW_1                        ((uint32_t)0x00000200)
#define HASH_CR_NBW_2                        ((uint32_t)0x00000400)
#define HASH_CR_NBW_3                        ((uint32_t)0x00000800)
#define HASH_CR_DINNE                        ((uint32_t)0x00001000)
#define HASH_CR_MDMAT                        ((uint32_t)0x00002000)
#define HASH_CR_LKEY                         ((uint32_t)0x00010000)

/******************  Bits definition for HASH_STR register  *******************/
#define HASH_STR_NBW                         ((uint32_t)0x0000001F)
#define HASH_STR_NBW_0                       ((uint32_t)0x00000001)
#define HASH_STR_NBW_1                       ((uint32_t)0x00000002)
#define HASH_STR_NBW_2                       ((uint32_t)0x00000004)
#define HASH_STR_NBW_3                       ((uint32_t)0x00000008)
#define HASH_STR_NBW_4                       ((uint32_t)0x00000010)
#define HASH_STR_DCAL                        ((uint32_t)0x00000100)

/******************  Bits definition for HASH_IMR register  *******************/
#define HASH_IMR_DINIM                       ((uint32_t)0x00000001)
#define HASH_IMR_DCIM                        ((uint32_t)0x00000002)

/******************  Bits definition for HASH_SR register  ********************/
#define HASH_SR_DINIS                        ((uint32_t)0x00000001)
#define HASH_SR_DCIS                         ((uint32_t)0x00000002)
#define HASH_SR_DMAS                         ((uint32_t)0x00000004)
#define HASH_SR_BUSY                         ((uint32_t)0x00000008)

/**
  * @}
  */
 
/**
  * @}
  */


/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _STM32_HAL_HASH_REG_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
