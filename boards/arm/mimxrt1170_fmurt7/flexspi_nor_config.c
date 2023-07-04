/*
 * Copyright (c) 2019, MADMACHINE LIMITED
 *
 * refer to hal_nxp board file
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <flexspi_nor_config.h>

#ifdef CONFIG_NXP_IMX_RT_BOOT_HEADER
#if defined(__CC_ARM) || defined(__ARMCC_VERSION) || defined(__GNUC__)
__attribute__((section(".boot_hdr.conf")))
#elif defined(__ICCARM__)
#pragma location = ".boot_hdr.conf"
#endif

const struct flexspi_nor_config_t Qspiflash_config = {
	.memConfig = {
		.tag = FLEXSPI_CFG_BLK_TAG,
		.version = FLEXSPI_CFG_BLK_VERSION,
		.readSampleClkSrc =
			kFlexSPIReadSampleClk_LoopbackInternally,
		.csHoldTime = 1u,
		.csSetupTime = 1u,
		.sflashPadType = kSerialFlash_1Pad,
		.serialClkFreq = kFlexSpiSerialClk_80MHz,
		.sflashA1Size = 64u * 1024u * 1024u,
		.lookupTable = {
			FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD,
					0x03, RADDR_SDR,
					FLEXSPI_1PAD, 0x18),
			FLEXSPI_LUT_SEQ(READ_SDR, FLEXSPI_1PAD,
			        0x04, STOP,
					FLEXSPI_1PAD, 0),
		},
	},
	.pageSize = 256u,
	.sectorSize = 4u * 1024u,
	.blockSize = 64u * 1024u,
	.isUniformBlockSize = false,
};
#endif /* CONFIG_NXP_IMX_RT_BOOT_HEADER */
