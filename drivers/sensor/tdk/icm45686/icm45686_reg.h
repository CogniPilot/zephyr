/*
 * Copyright (c) 2022 Intel Corporation
 * Copyright (c) 2025 Croxel Inc.
 * Copyright (c) 2025 CogniPilot Foundation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ICM45686_REG_H_
#define ZEPHYR_DRIVERS_SENSOR_ICM45686_REG_H_

#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>

/* Address value has a read bit */
#define REG_READ_BIT BIT(7)

/* IREG BASE ADDR OFFSETS */
#define IMEM_SRAM_BASE 		0x0000
#define IPREG_BAR_BASE 		0XA000
#define IPREG_SYS1_BASE 	0xA400
#define IPREG_SYS2_BASE 	0xA500
#define IPREG_TOP1_BASE 	0xA200

/* Registers */
/* Register Bank 0 */
#define REG_ACCEL_DATA_X1_UI		0x00
#define REG_ACCEL_DATA_X0_UI		0x01
#define REG_ACCEL_DATA_Y1_UI		0x02
#define REG_ACCEL_DATA_Y0_UI		0x03
#define REG_ACCEL_DATA_Z1_UI		0x04
#define REG_ACCEL_DATA_Z0_UI		0x05
#define REG_GYRO_DATA_X1_UI			0x06
#define REG_GYRO_DATA_X0_UI			0x07
#define REG_GYRO_DATA_Y1_UI			0x08
#define REG_GYRO_DATA_Y0_UI			0x09
#define REG_GYRO_DATA_Z1_UI			0x0A
#define REG_GYRO_DATA_Z0_UI			0x0B
#define REG_TEMP_DATA1_UI			0x0C
#define REG_TEMP_DATA0_UI			0x0D
#define REG_PWR_MGMT0				0x10
#define REG_FIFO_COUNT_0			0x12
#define REG_FIFO_COUNT_1			0x13
#define REG_FIFO_DATA				0x14
#define REG_INT1_CONFIG0			0x16
#define REG_INT1_CONFIG1			0x17
#define REG_INT1_CONFIG2			0x18
#define REG_INT1_STATUS0			0x19
#define REG_INT1_STATUS1			0x1A
#define REG_ACCEL_CONFIG0			0x1B
#define REG_GYRO_CONFIG0			0x1C
#define REG_FIFO_CONFIG0			0x1D
#define REG_FIFO_CONFIG1_0			0x1E
#define REG_FIFO_CONFIG1_1			0x1F
#define REG_FIFO_CONFIG2			0x20
#define REG_FIFO_CONFIG3			0x21
#define REG_FIFO_CONFIG4			0x22
#define REG_EDMP_APEX_EN0 			0x29
#define REG_EDMP_APEX_EN1 			0x2A
#define REG_APEX_BUFFER_MGMT		0x2B
#define REG_INTF_CONFIG1_OVRD		0x2D
#define REG_DRIVE_CONFIG0			0x32
#define REG_DRIVE_CONFIG1			0x33
#define REG_INT_APEX_CONFIG0 		0x39
#define REG_INT_APEX_CONFIG1 		0x3A
#define REG_INT_APEX_STATUS0 		0x3B
#define REG_INT_APEX_STATUS1 		0x3C
#define REG_WHO_AM_I				0x72
#define REG_HOST_MSG 				0x73
#define REG_IREG_ADDR_15_8			0x7C
#define REG_IREG_ADDR_7_0			0x7D
#define REG_IREG_DATA				0x7E
#define REG_MISC2					0x7F

/* Register Bank IMEM_SRAM */
#define REG_IMEM_SRAM_REG_56 		(0x38 + IMEM_SRAM_BASE)	//Config self-test
#define REG_IMEM_SRAM_REG_57		(0X39 + IMEM_SRAM_BASE)	//Config self-test tolerance limits
#define REG_IMEM_SRAM_REG_64		(0X40 + IMEM_SRAM_BASE)	//Self-test debug enable register
#define REG_IMEM_SRAM_REG_68		(0X44 + IMEM_SRAM_BASE)	//Self-test status register

/* stc_patch_en
 *
 * Mechanism for enabling patches execution in SRAM for self-test operations
 * bit0: If set, enable SRAM patching for accel self-test phase 1
 * bit1: If set, enable SRAM patching for accel self-test phase 2
 * bit2: If set, enable SRAM patching for gyro self-test phase 1
 * bit3: If set, enable SRAM patching for gyro self-test phase 2
 */
#define REG_IMEM_SRAM_STC_PATCH_EN      (0x3c + IMEM_SRAM_BASE)

/* Self-test register bits */
//EDMP_STC_RESULTS
#define STC_RESULTS_ACCEL_X_MASK   0x0001
#define STC_RESULTS_ACCEL_Y_MASK   0x0002
#define STC_RESULTS_ACCEL_Z_MASK   0x0004
#define STC_RESULTS_GYRO_X_MASK    0x0008
#define STC_RESULTS_GYRO_Y_MASK    0x0010
#define STC_RESULTS_GYRO_Z_MASK    0x0020
#define STC_RESULTS_ST_STATUS_MASK 0x00C0
#define STC_RESULTS_ACCEL_SC_MASK  0x0300
#define STC_RESULTS_GYRO_SC_MASK   0x0C00

//EDMP_STC_CONFIGPARAMS
#define SELFTESTCAL_INIT_EN_MASK   0x0001
#define SELFTESTCAL_INIT_EN        0x0001
#define SELFTEST_ACCEL_EN_MASK     0x0002
#define SELFTEST_ACCEL_EN          0x0002
#define SELFTEST_GYRO_EN_MASK      0x0004
#define SELFTEST_GYRO_EN           0x0004
#define SELFTEST_AVERAGE_TIME_MASK 0x0380
#define SELFTEST_ACCEL_THRESH_MASK 0x1C00
#define SELFTEST_GYRO_THRESH_MASK  0xE000

/* Register Bank IPREG_TOP1 */
#define REG_EDMP_PRGRM_IRQ0_0			(0x4F + IPREG_TOP1_BASE)
#define REG_EDMP_PRGRM_IRQ0_1			(0x50 + IPREG_TOP1_BASE)
#define REG_EDMP_PRGRM_IRQ1_0			(0x51 + IPREG_TOP1_BASE)
#define REG_EDMP_PRGRM_IRQ1_1			(0x52 + IPREG_TOP1_BASE)
#define REG_EDMP_PRGRM_IRQ2_0			(0x53 + IPREG_TOP1_BASE)
#define REG_EDMP_PRGRM_IRQ2_1			(0x54 + IPREG_TOP1_BASE)
#define REG_EDMP_SP_START_ADDR 			(0X55 + IPREG_TOP1_BASE)
#define REG_FIFO_SRAM_SLEEP				(0xA7 + IPREG_TOP1_BASE)
#define REG_ISR_0_7						(0x6e + IPREG_TOP1_BASE)
#define REG_ISR_8_15					(0x6f + IPREG_TOP1_BASE)
#define REG_ISR_16_23					(0x70 + IPREG_TOP1_BASE)
#define REG_STATUS_MASK_PIN_0_7			(0x71 + IPREG_TOP1_BASE)
#define REG_STATUS_MASK_PIN_8_15		(0x72 + IPREG_TOP1_BASE)
#define REG_STATUS_MASK_PIN_16_23		(0x73 + IPREG_TOP1_BASE)
#define REG_SELFTEST 					(0x90 + IPREG_TOP1_BASE)
#define REG_IPREG_MISC					(0x97 + IPREG_TOP1_BASE)

/* EDMP defines */
#define EDMP_RAM_BASE	0x0
#define EDMP_ROM_BASE	0x4000
#define EDMP_ROM_START_ADDR_IRQ0 EDMP_ROM_BASE
#define EDMP_ROM_START_ADDR_IRQ1 (EDMP_ROM_BASE + 0x04)
#define EDMP_ROM_START_ADDR_IRQ2 (EDMP_ROM_BASE + 0x08)
#define EDMP_ROM_DATA_SIZE	0x4C0
#define APEX_FEATURE_STACK_END	0x500
#define EDMP_RAM_FEATURE_PRGM_RAM_BASE	0x500
#define EDMP_HOST_INT_TAP_DET_POS	0x0
#define EDMP_HOST_INT_HIGHG_DET_POS	0x1
#define EDMP_HOST_INT_LOWG_DET_POS	0x2
#define EDMP_HOST_INT_TILT_DET_POS	0x3
#define EDMP_HOST_INT_STEP_CNT_OVFL_DET_POS	0x4
#define EDMP_HOST_INT_STEP_DET_POS	0x5
#define EDMP_HOST_INT_FF_DET_POS	0x6
#define EDMP_HOST_INT_R2W_WAKE_DET_POS	0x7
#define EDMP_HOST_INT_B2S_DET_POS	0x7
#define EDMP_HOST_INT_REVB2S_DET_POS	0x0
#define EDMP_HOST_INT_R2W_SLEEP_DET_POS	0x0
#define EDMP_HOST_INT_SMD_DET_POS	0x1
#define EDMP_HOST_INT_SELF_TEST_DONE_POS	0x2
#define EDMP_HOST_INT_SA_DONE_POS	0x4
#define EDMP_HOST_INT_BASIC_SMD_DET_POS	0x5
#define RAM_PATCHES_PRGM_RAM_BASE	0x500

/* User Bank IPREG_SYS1 - Gyro-related config settings */
#define REG_IPREG_SYS1_OFFSET			0xA400
#define REG_IPREG_SYS1_REG_172			0xAC

/* User Bank IPREG_SYS2 - Accel-related config settings */
#define REG_IPREG_SYS2_OFFSET			0xA500
#define REG_IPREG_SYS2_REG_131			0x83

/* Helper Macros for register manipulation */
#define REG_PWR_MGMT0_ACCEL_MODE(val)			((val) & BIT_MASK(2))
#define REG_PWR_MGMT0_GYRO_MODE(val)			(((val) & BIT_MASK(2)) << 2)

#define REG_ACCEL_CONFIG0_ODR(val)			((val) & BIT_MASK(4))
#define REG_ACCEL_CONFIG0_FS(val)			(((val) & BIT_MASK(3)) << 4)

#define REG_GYRO_CONFIG0_ODR(val)			((val) & BIT_MASK(4))
#define REG_GYRO_CONFIG0_FS(val)			(((val) & BIT_MASK(4)) << 4)

#define REG_DRIVE_CONFIG0_SPI_SLEW(val)			(((val) & BIT_MASK(2)) << 1)
#define REG_DRIVE_CONFIG1_I3C_SLEW(val)			(((val) & BIT_MASK(3)) |		   \
							 (((val) & BIT_MASK(3)) << 3))

#define REG_MISC2_SOFT_RST(val)				((val << 1) & BIT(1))

#define REG_IPREG_SYS1_REG_172_GYRO_LPFBW_SEL(val)	(val & BIT_MASK(3))

#define REG_IPREG_SYS2_REG_131_ACCEL_LPFBW_SEL(val)	(val & BIT_MASK(3))

#define REG_INT1_CONFIG0_STATUS_EN_DRDY(val)		(((val) & BIT_MASK(1)) << 2)
#define REG_INT1_CONFIG0_STATUS_EN_FIFO_THS(val)	(((val) & BIT_MASK(1)) << 1)
#define REG_INT1_CONFIG0_STATUS_EN_FIFO_FULL(val)	((val) & BIT_MASK(1))

#define REG_INT1_CONFIG2_EN_OPEN_DRAIN(val)		(((val) & BIT_MASK(1)) << 2)
#define REG_INT1_CONFIG2_EN_LATCH_MODE(val)		(((val) & BIT_MASK(1)) << 1)
#define REG_INT1_CONFIG2_EN_ACTIVE_HIGH(val)		((val) & BIT_MASK(1))

#define REG_INT1_STATUS0_DRDY(val)			(((val) & BIT_MASK(1)) << 2)
#define REG_INT1_STATUS0_FIFO_THS(val)			(((val) & BIT_MASK(1)) << 1)
#define REG_INT1_STATUS0_FIFO_FULL(val)			((val) & BIT_MASK(1))

#define REG_FIFO_CONFIG0_FIFO_MODE_BYPASS		0
#define REG_FIFO_CONFIG0_FIFO_MODE_STREAM		1
#define REG_FIFO_CONFIG0_FIFO_MODE_STOP_ON_FULL		2

#define REG_FIFO_CONFIG0_FIFO_DEPTH_2K			0x07
#define REG_FIFO_CONFIG0_FIFO_DEPTH_8K			0x1F

#define REG_FIFO_CONFIG0_FIFO_MODE(val)			(((val) & BIT_MASK(2)) << 6)
#define REG_FIFO_CONFIG0_FIFO_DEPTH(val)		((val) & BIT_MASK(6))

#define REG_FIFO_CONFIG1_0_FIFO_WM_THS(val)		((val) & BIT_MASK(8))
#define REG_FIFO_CONFIG1_1_FIFO_WM_THS(val)		(((val) >> 8) & BIT_MASK(8))

#define REG_FIFO_CONFIG2_FIFO_FLUSH(val)		(((val) & BIT_MASK(1)) << 7)
#define REG_FIFO_CONFIG2_FIFO_WM_GT_THS(val)		(((val) & BIT_MASK(1)) << 3)

#define REG_FIFO_CONFIG3_FIFO_HIRES_EN(val)		(((val) & BIT_MASK(1)) << 3)
#define REG_FIFO_CONFIG3_FIFO_GYRO_EN(val)		(((val) & BIT_MASK(1)) << 2)
#define REG_FIFO_CONFIG3_FIFO_ACCEL_EN(val)		(((val) & BIT_MASK(1)) << 1)
#define REG_FIFO_CONFIG3_FIFO_EN(val)			((val) & BIT_MASK(1))

/* Misc. Defines */
#define WHO_AM_I_ICM45686 0xE9

#define REG_IREG_PREPARE_WRITE_ARRAY(base, reg, val)	{((base) >> 8) & 0xFF, reg, val}

#define FIFO_HEADER_EXT_HEADER_EN(val)			(((val) & BIT_MASK(1)) << 7)
#define FIFO_HEADER_ACCEL_EN(val)			(((val) & BIT_MASK(1)) << 6)
#define FIFO_HEADER_GYRO_EN(val)			(((val) & BIT_MASK(1)) << 5)
#define FIFO_HEADER_HIRES_EN(val)			(((val) & BIT_MASK(1)) << 4)

#define FIFO_NO_DATA					0x8000
#define FIFO_COUNT_MAX_HIGH_RES				104

#endif /* ZEPHYR_DRIVERS_SENSOR_ICM45686_REG_H_ */
