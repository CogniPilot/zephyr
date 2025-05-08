/*
 * Copyright (c) 2025 Croxel Inc.
 * Copyright (c) 2025 CogniPilot Foundation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_MODEM_UBX_CHECKSUM_
#define ZEPHYR_MODEM_UBX_CHECKSUM_

/** Macrobatics to compute UBX checksum at compile time */

#define UBX_CSUM_A(...) UBX_CSUM_A_(__VA_ARGS__)

#define UBX_CSUM_A_(...) UBX_CSUM_A_I(__VA_ARGS__, 0, 0, 0, 0, 0, 0, 0, 0, 0)

#define UBX_CSUM_A_I(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, ...)				   \
	((a1) + (a2) + (a3) + (a4) + (a5) + (a6) + (a7) + (a8) + (a9) + (a10)) & 0xFF

#define UBX_CSUM_B(...) UBX_CSUM_B_(__VA_ARGS__)

#define UBX_CSUM_B_(...) UBX_CSUM_B_I(NUM_VA_ARGS(__VA_ARGS__), __VA_ARGS__,			   \
				      0, 0, 0, 0, 0, 0, 0, 0, 0)

#define UBX_CSUM_B_I(len, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, ...)				   \
	(((len) * a1) + ((len - 1) * a2) + ((len - 2) * a3) + ((len - 3) * a4) +		   \
	 ((len - 4) * a5) + ((len - 5) * a6) + ((len - 6) * a7) + ((len - 7) * a8) +		   \
	 ((len - 8) * a9) + ((len - 9) * a10)) & 0xFF

#define UBX_CSUM(...) UBX_CSUM_A(__VA_ARGS__), UBX_CSUM_B(__VA_ARGS__)

#endif /* ZEPHYR_MODEM_UBX_CHECKSUM_ */
