/*
 ******************************************************************************
 * @file    nand.h
 * @author  mgentilini - MCD IntroPack team - MPU AP v1 bootROM project
 * @version V0.1
 * @date    28-April-2016
 * @brief   Header file for STM32MP1 Nand driver module.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) STMicroelectronics</center></h2>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ******************************************************************************
 */

#ifndef __NAND_H
#define __NAND_H

#include <stm32mp1xx_hal.h>

/* FMC NAND Flash 'common memory' data area */
#define FLASH_COMMON_MEM_BASE 0x80000000
/* FMC NAND Flash 'attribute memory' data area */
#define FLASH_ATTRIB_MEM_BASE 0x88000000

#define BCH_PAGE_SECTOR     512

/* BBM */
#define GOOD_BLOCK 0
#define BAD_BLOCK  1

/****************  Bit definition for FMC_PCR register  *******************/
/* Wait feature enable bit */
#define  FMC_PCR_PWAITEN	((uint32_t)0x00000002)
/* PC Card/NAND Flash memory bank enable bit */
#define  FMC_PCR_PBKEN		((uint32_t)0x00000004)

/* PWID[1:0] bits (NAND Flash databus width) */
#define  FMC_PCR_PWID		((uint32_t)0x00000030)
#define  FMC_PCR_PWID_0		((uint32_t)0x00000010)	/* Bit 0 */
#define  FMC_PCR_PWID_1		((uint32_t)0x00000020)	/* Bit 1 */

/* ECC computation logic enable bit */
#define  FMC_PCR_ECCEN		((uint32_t)0x00000040)

/* ECC algorithm */
#define  FMC_PCR_ECCALG		((uint32_t)0x00000100)

/* TCLR[3:0] bits (CLE to RE delay) */
#define  FMC_PCR_TCLR_0		((uint32_t)0x00000200)	/* Bit 0 */
#define  FMC_PCR_TCLR_1		((uint32_t)0x00000400)	/* Bit 1 */
#define  FMC_PCR_TCLR_2		((uint32_t)0x00000800)	/* Bit 2 */
#define  FMC_PCR_TCLR_3		((uint32_t)0x00001000)	/* Bit 3 */

/* TAR[3:0] bits (ALE to RE delay) */
#define  FMC_PCR_TAR_0		((uint32_t)0x00002000)	/* Bit 0 */
#define  FMC_PCR_TAR_1		((uint32_t)0x00004000)	/* Bit 1 */
#define  FMC_PCR_TAR_2		((uint32_t)0x00008000)	/* Bit 2 */
#define  FMC_PCR_TAR_3		((uint32_t)0x00010000)	/* Bit 3 */

/* ECCSS[1:0] bits (ECC sector size) */
#define  FMC_PCR_ECCSS		((uint32_t)0x000E0000)
#define  FMC_PCR_ECCSS_0	((uint32_t)0x00020000)	/* Bit 0 */
#define  FMC_PCR_ECCSS_1	((uint32_t)0x00040000)	/* Bit 1 */
#define  FMC_PCR_ECCSS_2	((uint32_t)0x00080000)	/* Bit 2 */

/* BCH Error correction capability */
#define  FMC_PCR_BCHECC		((uint32_t)0x01000000)

/* Write enable */
#define  FMC_PCR_WE		((uint32_t)0x02000000)

/****************  Bit definition for FMC_BCR1 register  *******************/
/* FMC controller enable */
#define  FMC_BCR1_FMCEN		((uint32_t)0x80000000)

/****************  Bit definition for FMC_BCHISR register  *******************/
/* Decoder Error Ready Flag */
#define FMC_BCHISR_DERF		((uint32_t)0x00000002)
/* Decoder Uncorrectable Error Flag */
#define FMC_BCHISR_DUEF		((uint32_t)0x00000001)
/* Decoder Error Found Flag */
#define FMC_BCHISR_DEFF		((uint32_t)0x00000004)

/****************  Bit definition for FMC_BCHSR register  *******************/
/* Decoder Error Number */
#define FMC_BCHSR_DEN		((uint32_t)0x0000000F)

/****************  Bit definition for FMC_BCHDSR0 register  *******************/
/* Decoder Uncorrectable Error */
#define FMC_BCHDSR0_DUE		((uint32_t)0x00000001)
/* Decoder Error Number */
#define FMC_BCHDSR0_DEN		((uint32_t)0x000000F0)

/****************  Bit definition for FMC_BCHDSR1 register  *******************/
#define FMC_BCHDSR1_EBP1	((uint32_t)0x00001FFF)
#define FMC_BCHDSR1_EBP2	((uint32_t)0x1FFF0000)

/****************  Bit definition for FMC_BCHDSR2 register  *******************/
#define FMC_BCHDSR2_EBP1      ((uint32_t)0x00001FFF)
#define FMC_BCHDSR2_EBP2	((uint32_t)0x1FFF0000)

/****************  Bit definition for FMC_BCHDSR3 register  *******************/
#define FMC_BCHDSR3_EBP1	((uint32_t)0x00001FFF)
#define FMC_BCHDSR3_EBP2	((uint32_t)0x1FFF0000)

/****************  Bit definition for FMC_BCHDSR4 register  *******************/
#define FMC_BCHDSR4_EBP1	((uint32_t)0x00001FFF)
#define FMC_BCHDSR4_EBP2	((uint32_t)0x1FFF0000)

/****************  Bit definition for FMC_SR register  **********************/
#define FMC_SR_NWRF		((uint32_t)0x00000040)
#define FMC_SR_PF		((uint32_t)0x00000010)

#define FMC_NSEC_PER_SEC		1000000000L

/* Timings */
#define FMC_THIZ			1
#define FMC_TIO				8000
#define FMC_TSYNC			3000
#define FMC_PCR_TIMING_MASK		0xFUL
#define FMC_PMEM_PATT_TIMING_MASK	0xFFUL

/* Register: FMC_PCR */
#define FMC_PCR_TCLR(x)			(((x) & 0xf) << 9)
#define FMC_PCR_TAR(x)			(((x) & 0xf) << 13)

/* Register: FMC_PMEM */
#define FMC_PMEM_MEMSET(x)		(((x) & 0xff) << 0)
#define FMC_PMEM_MEMWAIT(x)		(((x) & 0xff) << 8)
#define FMC_PMEM_MEMHOLD(x)		(((x) & 0xff) << 16)
#define FMC_PMEM_MEMHIZ(x)		(((x) & 0xff) << 24)

/* Register: FMC_PATT */
#define FMC_PATT_ATTSET(x)		(((x) & 0xff) << 0)
#define FMC_PATT_ATTWAIT(x)		(((x) & 0xff) << 8)
#define FMC_PATT_ATTHOLD(x)		(((x) & 0xff) << 16)
#define FMC_PATT_ATTHIZ(x)		(((x) & 0xff) << 24)

/* NAND ONFI Default Value Mode 0 */
#define FMC_TADL_MIN	400000UL
#define FMC_TALH_MIN	20000UL
#define FMC_TALS_MIN	50000UL
#define FMC_TAR_MIN	25000UL
#define FMC_TCH_MIN	20000UL
#define FMC_TCLH_MIN	20000UL
#define FMC_TCLR_MIN	20000UL
#define FMC_TCLS_MIN	50000UL
#define FMC_TCOH_MIN	0UL
#define FMC_TCS_MIN	70000UL
#define FMC_TDH_MIN	20000UL
#define FMC_TDS_MIN	40000UL
#define FMC_TRC_MIN	100000UL
#define FMC_TREA_MAX	40000UL
#define FMC_TREH_MIN	30000UL
#define FMC_TRHW_MIN	200000UL
#define FMC_TRP_MIN	50000UL
#define FMC_TWB_MAX	200000UL
#define FMC_TWC_MIN	100000UL
#define FMC_TWH_MIN	30000UL
#define FMC_TWHR_MIN	120000UL
#define FMC_TWP_MIN	50000UL


#define FMC_EBP2_MASK   16

/* 1st addressing cycle */
#define ADDR_1ST_CYCLE(__ADDRESS__)       (uint8_t)(__ADDRESS__)
/* 2nd addressing cycle */
#define ADDR_2ND_CYCLE(__ADDRESS__)       (uint8_t)((__ADDRESS__) >> 8)
/* 3rd addressing cycle */
#define ADDR_3RD_CYCLE(__ADDRESS__)       (uint8_t)((__ADDRESS__) >> 16)
/* 4th addressing cycle */
#define ADDR_4TH_CYCLE(__ADDRESS__)       (uint8_t)((__ADDRESS__) >> 24)

typedef struct {
	uint8_t tclr;
	uint8_t tar;
	uint8_t thiz;
	uint8_t twait;
	uint8_t thold_mem;
	uint8_t tset_mem;
	uint8_t thold_att;
	uint8_t tset_att;
} nand_timings;

typedef struct {
	uint32_t Signature;        /* NAND Parameter page signature */

	uint32_t PageSize;         /* NAND memory page (without spare area) size
				    * measured in B
				    */

	uint32_t BlockSize;        /* NAND memory block size in number
				    * of pages
				    */

	uint32_t BlockNb;          /* NAND memory number of blocks */

	uint32_t ZoneSize;         /* NAND memory zone size measured in number
				    * of blocks
				    */

	uint32_t BusWidth;         /* NAND memory bus width in bytes */

	uint32_t ECCcorrectability; /* NAND number of bits ECC correctability */

	uint32_t page_size_shift;
	uint32_t block_size_shift;

	nand_timings timings;

} NAND_InfoTypeDef;

/* NAND Memory address Structure definition */
typedef struct {
	uint16_t Page;   /* NAND memory Page address  */

	uint16_t Zone;   /* NAND memory Zone address  */

	uint16_t Block;  /* NAND memory Block address */
} NAND_AddressTypeDef;

/* NAND Memory electronic signature Structure definition */
typedef struct {
	uint8_t Maker_Id;

	uint8_t Device_Id;
} NAND_IDTypeDef;

/* NAND handle Structure definition */
typedef struct {
	FMC_TypeDef *Instance;  /* Register base address */

	NAND_InfoTypeDef Info;  /* NAND characteristic information structure */
} NAND_HandleTypeDef;

/* Exported functions */
Std_ReturnType nand_initialize(NAND_HandleTypeDef *hnand);
Std_ReturnType NAND_Read_Logical_Page(NAND_HandleTypeDef *hNand,
				      NAND_AddressTypeDef *Address,
				      uint8_t *Buffer, uint32_t bch_sector_nb);
uint32_t NAND_Check_Bad_Block(NAND_HandleTypeDef *hNand,
			      NAND_AddressTypeDef *Address);
Std_ReturnType NAND_Address_Inc(NAND_HandleTypeDef *hNand,
				NAND_AddressTypeDef *Address,
				uint32_t numPagesRead);

#endif /* __NAND_H */

