/*
 * Copyright (c) 2025 CogniPilot Foundation
 * Copyright (c) 2025 NXP Semiconductors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_INPUT_INPUT_CRSF_H_
#define ZEPHYR_INCLUDE_DRIVERS_INPUT_INPUT_CRSF_H_

#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* CRSF Frame Constants */
#define CRSF_SYNC_BYTE       0xC8
#define CRSF_FRAME_MAX_LEN   64
#define CRSF_MAX_PAYLOAD_LEN (CRSF_FRAME_MAX_LEN - 4) /* Sync(1)+Len(1)+Type(1)+CRC(1) */

/* CRSF Packet Types */
enum crsf_packet_type {
	CRSF_TYPE_GPS = 0x02,
	CRSF_TYPE_BATTERY = 0x08,
	CRSF_TYPE_LINK_TX_ID = 0x13, /* ELRS specific */
	CRSF_TYPE_LINK_STATS = 0x14,
	CRSF_TYPE_RC_CHANNELS = 0x16,
	CRSF_TYPE_ATTITUDE = 0x1E,
	CRSF_TYPE_FLIGHT_MODE = 0x21,
	CRSF_TYPE_PING_DEVICES = 0x28,
	CRSF_TYPE_DEVICE_INFO = 0x29,
	CRSF_TYPE_PARAMETER_SETTINGS = 0x2B,
	CRSF_TYPE_COMMAND = 0x32,
	CRSF_TYPE_RADIO_ID = 0x3A,
};

/* * Telemetry Payloads
 * Note: CRSF protocol uses Big Endian for multi-byte values.
 * Users must use sys_put_be* or sys_cpu_to_be* helpers when populating these.
 */

/* Type 0x02 - GPS */
struct crsf_payload_gps {
	int32_t lat;       /* Degree * 10,000,000 (Big Endian) */
	int32_t lon;       /* Degree * 10,000,000 (Big Endian) */
	uint16_t speed;    /* km/h * 10 (Big Endian) */
	uint16_t heading;  /* Degree * 100 (Big Endian) */
	uint16_t altitude; /* Meters + 1000m offset (Big Endian) */
	uint8_t satellites;
} __packed;

/* Type 0x08 - Battery */
struct crsf_payload_battery {
	uint16_t voltage_dV;     /* Volts * 10 (Big Endian) */
	uint16_t current_dA;     /* Amps * 10  (Big Endian) */
	uint8_t capacity_mah[3]; /* mAh (Big Endian, 24-bit) */
	uint8_t remaining_pct;   /* % */
} __packed;

/* Type 0x14 - Link stats */
struct crsf_link_stats {
    uint8_t uplink_rssi_1;       /* Uplink RSSI Ant. 1 (dBm * -1) */
    uint8_t uplink_rssi_2;       /* Uplink RSSI Ant. 2 (dBm * -1) */
    uint8_t uplink_link_quality; /* Uplink LQ (0-100 %) */
    int8_t  uplink_snr;          /* Uplink SNR (dB) */
    uint8_t active_antenna;      /* Active Antenna (0 or 1) */
    uint8_t rf_mode;             /* RF Mode (0=4Hz, 1=50Hz, 2=150Hz, etc.) */
    uint8_t tx_power;            /* TX Power (Enum value, not mW) */
    uint8_t downlink_rssi;       /* Downlink RSSI (dBm * -1) */
    uint8_t downlink_link_quality; /* Downlink LQ (0-100 %) */
    int8_t  downlink_snr;        /* Downlink SNR (dB) */
} __packed;

/* Type 0x16 - RC Channels */
#define CRSF_TICKS_TO_US(x)  ((x - 992) * 5 / 8 + 1500)
#define CRSF_US_TO_TICKS(x)  ((x - 1500) * 8 / 5 + 992)

/* Type 0x1E - Attitude */
struct crsf_payload_attitude {
	int16_t pitch_rad; /* Pitch (rad) * 10000 (Big Endian) */
	int16_t roll_rad;  /* Roll  (rad) * 10000 (Big Endian) */
	int16_t yaw_rad;   /* Yaw   (rad) * 10000 (Big Endian) */
} __packed;

/* Type 0x21 - Flightmore */
/* Null-terminated C string */

/**
 * @brief Send a generic CRSF telemetry frame.
 * * @param dev Pointer to the CRSF input device.
 * @param type Packet type (from enum crsf_packet_type).
 * @param payload Pointer to the raw payload data.
 * @param payload_len Length of the payload (must be <= CRSF_MAX_PAYLOAD_LEN).
 * @return 0 on success, negative errno on failure.
 */
int input_crsf_send_telemetry(const struct device *dev, uint8_t type, const uint8_t *payload,
			      size_t payload_len);

/**
 * @brief Retrieve CRSF link statistics for a given device.
 * * @param dev Pointer to the CRSF input device.
 * @return A struct crsf_link_stats containing the link statistics for the specified device.
 */
struct crsf_link_stats input_crsf_get_link_stats(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_INPUT_INPUT_CRSF_H_ */
