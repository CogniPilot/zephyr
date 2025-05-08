/*
 * Copyright (c) 2025 Croxel Inc.
 * Copyright (c) 2025 CogniPilot Foundation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_MODEM_UBX_PROTOCOL_
#define ZEPHYR_MODEM_UBX_PROTOCOL_

#include <stdint.h>
#include <zephyr/modem/ubx/checksum.h>

#define UBX_FRM_HEADER_SZ			6
#define UBX_FRM_FOOTER_SZ			2
#define UBX_FRM_SZ_WITHOUT_PAYLOAD		(UBX_FRM_HEADER_SZ + UBX_FRM_FOOTER_SZ)
#define UBX_FRM_SZ(payload_size)		(payload_size + UBX_FRM_SZ_WITHOUT_PAYLOAD)

#define UBX_PREAMBLE_SYNC_CHAR_1		0xB5
#define UBX_PREAMBLE_SYNC_CHAR_2		0x62

#define UBX_FRM_PREAMBLE_SYNC_CHAR_1_IDX	0
#define UBX_FRM_PREAMBLE_SYNC_CHAR_2_IDX	1
#define UBX_FRM_MSG_CLASS_IDX			2

#define UBX_PAYLOAD_SZ_MAX			256
#define UBX_FRM_SZ_MAX				UBX_FRM_SZ(UBX_PAYLOAD_SZ_MAX)

struct ubx_frame {
	uint8_t preamble_sync_char_1;
	uint8_t preamble_sync_char_2;
	uint8_t class;
	uint8_t id;
	uint16_t payload_size;
	uint8_t payload_and_checksum[];
};

struct ubx_frame_match {
	uint8_t class;
	uint8_t id;
	struct {
		uint8_t *buf;
		uint16_t len;
	} payload;
};

enum ubx_class_id {
	UBX_CLASS_ID_NAV = 0x01,  /* Navigation Results Messages */
	UBX_CLASS_ID_RXM = 0x02,  /* Receiver Manager Messages */
	UBX_CLASS_ID_INF = 0x04,  /* Information Messages */
	UBX_CLASS_ID_ACK = 0x05,  /* Ack/Nak Messages */
	UBX_CLASS_ID_CFG = 0x06,  /* Configuration Input Messages */
	UBX_CLASS_ID_UPD = 0x09,  /* Firmware Update Messages */
	UBX_CLASS_ID_MON = 0x0A,  /* Monitoring Messages */
	UBX_CLASS_ID_TIM = 0x0D,  /* Timing Messages */
	UBX_CLASS_ID_MGA = 0x13,  /* Multiple GNSS Assistance Messages */
	UBX_CLASS_ID_LOG = 0x21,  /* Logging Messages */
	UBX_CLASS_ID_SEC = 0x27,  /* Security Feature Messages */
};

enum ubx_msg_id_ack {
	UBX_MSG_ID_ACK = 0x01,
	UBX_MSG_ID_NAK = 0x00
};

enum ubx_msg_id_mon {
	UBX_MSG_ID_MON_VER = 0x04,
};

struct ubx_ack {
	uint8_t class_id;
	uint8_t msg_id;
};

#define UBX_FRAME_ACK_INITIALIZER(_class_id, _msg_id)						   \
	UBX_FRAME_INITIALIZER_PAYLOAD(UBX_CLASS_ID_ACK, UBX_MSG_ID_ACK, _class_id, _msg_id)

#define UBX_FRAME_NAK_INITIALIZER(_class_id, _msg_id)						   \
	UBX_FRAME_INITIALIZER_PAYLOAD(UBX_CLASS_ID_ACK, UBX_MSG_ID_NAK, _class_id, _msg_id)

#define UBX_FRAME_INITIALIZER_PAYLOAD(_class_id, _msg_id, ...)					   \
	_UBX_FRAME_INITIALIZER_PAYLOAD(_class_id, _msg_id, __VA_ARGS__)

#define _UBX_FRAME_INITIALIZER_PAYLOAD(_class_id, _msg_id, ...)					   \
	{											   \
		.preamble_sync_char_1 = UBX_PREAMBLE_SYNC_CHAR_1,				   \
		.preamble_sync_char_2 = UBX_PREAMBLE_SYNC_CHAR_2,				   \
		.class = _class_id,								   \
		.id = _msg_id,									   \
		.payload_size = (NUM_VA_ARGS(__VA_ARGS__)) & 0xFFFF,				   \
		.payload_and_checksum = {							   \
			__VA_ARGS__,								   \
			UBX_CSUM(_class_id, _msg_id,						   \
				 ((NUM_VA_ARGS(__VA_ARGS__)) & 0xFF),				   \
				 (((NUM_VA_ARGS(__VA_ARGS__)) >> 8) & 0xFF),			   \
				 __VA_ARGS__),							   \
		},										   \
	}

#define UBX_FRAME_GET_INITIALIZER(_class_id, _msg_id)						   \
	{											   \
		.preamble_sync_char_1 = UBX_PREAMBLE_SYNC_CHAR_1,				   \
		.preamble_sync_char_2 = UBX_PREAMBLE_SYNC_CHAR_2,				   \
		.class = _class_id,								   \
		.id = _msg_id,									   \
		.payload_size = 0,								   \
		.payload_and_checksum = {							   \
			UBX_CSUM(_class_id, _msg_id, 0, 0),					   \
		},										   \
	}

#endif /* ZEPHYR_MODEM_UBX_PROTOCOL_ */
