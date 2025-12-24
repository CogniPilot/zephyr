/*
 * Copyright (c) 2025 CogniPilot Foundation
 * Copyright (c) 2025 NXP Semiconductors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT tbs_crsf

#include <zephyr/cache.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/drivers/input/input_crsf.h>

LOG_MODULE_REGISTER(tbs_crsf, CONFIG_INPUT_LOG_LEVEL);

#if DT_NODE_HAS_STATUS_OKAY(DT_CHOSEN(zephyr_dtcm)) && CONFIG_INPUT_CRSF_USE_DTCM_FOR_DMA_BUFFER
#define _dma_buffer_section __dtcm_noinit_section
#elif defined(CONFIG_NOCACHE_MEMORY)
#define _dma_buffer_section __nocache
#else
#define _dma_buffer_section
#define CRSF_INVALIDATE_CACHE
#endif

struct crsf_input_channel {
	uint32_t crsf_channel;
	uint32_t type;
	uint32_t zephyr_code;
};

const struct uart_config uart_cfg_crsf = {.baudrate = 420000,
					  .parity = UART_CFG_PARITY_NONE,
					  .stop_bits = UART_CFG_STOP_BITS_1,
					  .data_bits = UART_CFG_DATA_BITS_8,
					  .flow_ctrl = UART_CFG_FLOW_CTRL_NONE};

struct input_crsf_config {
	uint8_t num_channels;
	const struct crsf_input_channel *channel_info;
	const struct device *uart_dev;
};

/* CRSF Protocol Constants */
#define CRSF_MAX_FRAME_LEN         64
#define CRSF_CHANNEL_COUNT         16
#define CRSF_CONNECTION_TIMEOUT_MS 100
#define CRSF_TX_BUF_SIZE           64
#define CRSF_RX_BUF_SIZE           128  /* Async RX DMA buffer size */
#define CRSF_RX_TIMEOUT_US         1000 /* Flush timeout for async RX */

#define REPORT_FILTER      CONFIG_INPUT_CRSF_REPORT_FILTER
#define CHANNEL_VALUE_ZERO CONFIG_INPUT_CRSF_CHANNEL_VALUE_ZERO
#define CHANNEL_VALUE_ONE  CONFIG_INPUT_CRSF_CHANNEL_VALUE_ONE

/* RX State Machine */
enum crsf_rx_state {
	RX_STATE_SYNC, /* Waiting for 0xC8 */
	RX_STATE_DATA, /* Reading Payload + CRC */
};

struct input_crsf_data {
	struct k_thread thread;
	struct k_sem report_lock;

	struct crsf_link_stats link_stats;

	/* RX State */
	enum crsf_rx_state rx_state;
	uint8_t payload_len;                     /* Value of the 'Len' byte */
	uint16_t xfer_bytes;                     /* Bytes read so far into rd_data */
	uint8_t rd_data[CRSF_MAX_FRAME_LEN + 4]; /* Reassembly buffer */

	/* Async RX Buffers (Double buffering) */
	uint8_t *rx_buf_a;
	uint8_t *rx_buf_b;

	/* Double buffer for the thread */
	uint8_t crsf_frame[CRSF_MAX_FRAME_LEN];
	bool in_sync; /* Logical link state (valid RC frames being received) */

	/* TX State */
	uint8_t *tx_scratch_buf;
	atomic_t tx_busy; /* Flag to prevent concurrent uart_tx calls */

	uint16_t last_reported_value[CRSF_CHANNEL_COUNT];
	int8_t channel_mapping[CRSF_CHANNEL_COUNT];
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_INPUT_CRSF_THREAD_STACK_SIZE);
};

// atomic send has some problems for now
#define POLL_TX

/* * Telemetry Sending (Push to Ring Buffer)
 */
int input_crsf_send_telemetry(const struct device *dev, uint8_t type, const uint8_t *payload,
			      size_t payload_len)
{
	const struct input_crsf_config *const config = dev->config;
	struct input_crsf_data *data = dev->data;
	uint8_t frame[CRSF_MAX_FRAME_LEN];
	uint8_t offset = 0;
	uint8_t crc;

	if (payload_len > CRSF_MAX_PAYLOAD_LEN) {
		LOG_ERR("CRSF payload too large");
		return -EINVAL;
	}

	/* Construct Frame */
	frame[offset++] = CRSF_SYNC_BYTE;
	frame[offset++] = (uint8_t)(payload_len + 2);
	frame[offset++] = type;

	if (payload_len > 0 && payload != NULL) {
		memcpy(&frame[offset], payload, payload_len);
		offset += payload_len;
	}

	crc = crc8(&frame[2], offset - 2, 0xD5, 0x00, false);
	frame[offset++] = crc;

	if (atomic_cas(&data->tx_busy, 0, 1)) {
#ifdef POLL_TX
		for (int i = 0; i < offset; i++) {
			uart_poll_out(config->uart_dev, frame[i]);
		}
		atomic_set(&data->tx_busy, 0);
#else
		memcpy(data->tx_scratch_buf, frame, offset);
		arch_dcache_flush_range(data->tx_scratch_buf, offset);

		int err = uart_tx(config->uart_dev, data->tx_scratch_buf, offset, SYS_FOREVER_US);

		if (err) {
			LOG_ERR("Failed to start UART TX: %d", err);
			atomic_set(&data->tx_busy, 0);
		}

#endif
	}

	return 0;
}

struct crsf_link_stats input_crsf_get_link_stats(const struct device *dev)
{
	struct input_crsf_data *data = dev->data;

	return data->link_stats;
}

/* * Input Reporting Helper (Unchanged logic)
 */
static void input_crsf_report(const struct device *dev, unsigned int crsf_channel,
			      unsigned int value)
{
	const struct input_crsf_config *const config = dev->config;
	struct input_crsf_data *const data = dev->data;
	int channel = data->channel_mapping[crsf_channel];

	if (channel == -1) {
		return;
	}

	if (data->last_reported_value[channel] != value &&
	    (value >= (data->last_reported_value[channel] + REPORT_FILTER) ||
	     value <= (data->last_reported_value[channel] - REPORT_FILTER))) {
		switch (config->channel_info[channel].type) {
		case INPUT_EV_ABS:
		case INPUT_EV_MSC:
			input_report(dev, config->channel_info[channel].type,
				     config->channel_info[channel].zephyr_code, value, false,
				     K_FOREVER);
			break;
		default:
			if (value > CHANNEL_VALUE_ONE) {
				input_report_key(dev, config->channel_info[channel].zephyr_code, 1,
						 false, K_FOREVER);
			} else if (value < CHANNEL_VALUE_ZERO) {
				input_report_key(dev, config->channel_info[channel].zephyr_code, 0,
						 false, K_FOREVER);
			}
		}
		data->last_reported_value[channel] = value;
	}
}

static void input_crsf_input_report_thread(const struct device *dev, void *dummy2, void *dummy3)
{
	struct input_crsf_data *const data = dev->data;
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);
	uint8_t i, channel;
	uint8_t *crsf_channel_data = &data->crsf_frame[3];
	uint32_t value;
	int bits_read;
	unsigned int key;
	int ret;
	bool connected_reported = false;

	while (true) {
		if (!data->in_sync) {
			k_sem_take(&data->report_lock, K_FOREVER);
			if (data->in_sync) {
				LOG_DBG("CRSF RC link active");
			} else {
				continue;
			}
		} else {
			ret = k_sem_take(&data->report_lock, K_MSEC(CRSF_CONNECTION_TIMEOUT_MS));
			if (ret == -EAGAIN) {
				key = irq_lock();
				data->in_sync = false;
				data->rx_state = RX_STATE_SYNC;
				irq_unlock(key);
				connected_reported = false;
				LOG_DBG("CRSF RC link lost");
				continue;
			}
		}

		if (!connected_reported) {
			LOG_INF("CRSF RC Connected");
			connected_reported = true;
		}

		/* Parse the data */
		channel = 0;
		value = 0;
		bits_read = 0;

		for (i = 0; i < 22; i++) {
			/* Read the next byte */
			unsigned char byte = crsf_channel_data[i];

			/* Extract bits and construct the 11-bit value */
			value |= byte << bits_read;
			bits_read += 8;

			/* Check if we've read enough bits to form a full 11-bit value */
			while (bits_read >= 11) {
				input_crsf_report(dev, channel,
						  CRSF_TICKS_TO_US((int)(value & 0x7FF)));

				/* Shift right to prepare for the next 11 bits */
				value >>= 11;
				bits_read -= 11;
				channel++;
			}
		}

#ifdef CONFIG_INPUT_CRSF_SEND_SYNC
		input_report(dev, 0, 0, 0, true, K_FOREVER);
#endif
	}
}

/*
 * Byte Processor: Implements State Machine
 * Called by the Async Callback for every received byte
 */
static void crsf_process_byte(const struct device *dev, uint8_t byte)
{
	struct input_crsf_data *const data = dev->data;

	switch (data->rx_state) {
	case RX_STATE_SYNC:
		/* logic: waiting for [SYNC, LEN, TYPE] sequence or just SYNC validation */
		data->rd_data[data->xfer_bytes] = byte;
		data->xfer_bytes++;

		if (data->rd_data[0] != CRSF_SYNC_BYTE) {
			/* Reset if first byte isn't SYNC */
			data->xfer_bytes = 0;
		}
		/* Once we have 2 bytes (SYNC, LEN), we know the payload length */
		else if (data->xfer_bytes == 2) {
			data->payload_len = byte;
			/* Sanity check length to prevent overflow */
			if (data->payload_len > CRSF_MAX_PAYLOAD_LEN || data->payload_len < 2) {
				LOG_WRN("Invalid CRSF len: %d", data->payload_len);
				data->xfer_bytes = 0;
			} else {
				data->rx_state = RX_STATE_DATA;
			}
		}
		break;

	case RX_STATE_DATA:
		data->rd_data[data->xfer_bytes] = byte;
		data->xfer_bytes++;

		/* Target: Sync(1) + Len(1) + Payload(Len) */
		if (data->xfer_bytes == (2 + data->payload_len)) {
			uint8_t calc_crc =
				crc8(&data->rd_data[2], data->rd_data[1] - 1, 0xD5, 0x00, false);

			if (calc_crc != data->rd_data[2 + data->rd_data[1] - 1]) {
				LOG_WRN("CRSF CRC mismatch");
			} else if (data->rd_data[2] == CRSF_TYPE_RC_CHANNELS) {
				memcpy(data->crsf_frame, data->rd_data, data->xfer_bytes);
				data->in_sync = true;
				k_sem_give(&data->report_lock);
			} else if (data->rd_data[2] == CRSF_TYPE_LINK_STATS &&
				   data->xfer_bytes - 4 == sizeof(struct crsf_link_stats)) {
				memcpy(&data->link_stats, &data->rd_data[3],
				       sizeof(struct crsf_link_stats));
			}

			/* Reset for next frame */
			data->rx_state = RX_STATE_SYNC;
			data->xfer_bytes = 0;
		}
		break;
	}
}

/*
 * Async UART Callback
 */
static void crsf_uart_callback(const struct device *uart_dev, struct uart_event *evt,
			       void *user_data)
{
	const struct device *dev = user_data;
	struct input_crsf_data *data = dev->data;
	int i;

	switch (evt->type) {
	case UART_TX_DONE:
		/* TX finished, release busy flag */
		LOG_DBG("CRSF TX Done");
		atomic_set(&data->tx_busy, 0);
		break;

	case UART_TX_ABORTED:
		atomic_set(&data->tx_busy, 0);
		LOG_ERR("CRSF TX Aborted");
		break;

	case UART_RX_RDY:
#ifdef CRSF_INVALIDATE_CACHE
		arch_dcache_invd_range(&evt->data.rx.buf[evt->data.rx.offset], evt->data.rx.len);
#endif
		/* Process received data chunk */
		for (i = 0; i < evt->data.rx.len; i++) {
			crsf_process_byte(dev, evt->data.rx.buf[evt->data.rx.offset + i]);
		}
		break;

	case UART_RX_BUF_REQUEST:
		/* Provide the next buffer to keep reception continuous */
		{
			uint8_t *next_buf = (evt->data.rx_buf.buf == data->rx_buf_a)
						    ? data->rx_buf_b
						    : data->rx_buf_a;
			uart_rx_buf_rsp(uart_dev, next_buf, CRSF_RX_BUF_SIZE);
		}
		break;

	case UART_RX_BUF_RELEASED:
		/* Buffer released, nothing specific to do here for static buffers */
		break;

	case UART_RX_DISABLED:
		/* Restart RX if disabled (error recovery) */
		uart_rx_enable(uart_dev, data->rx_buf_a, CRSF_RX_BUF_SIZE, CRSF_RX_TIMEOUT_US);
		break;

	default:
		break;
	}
}

static int input_crsf_init(const struct device *dev)
{
	const struct input_crsf_config *const config = dev->config;
	struct input_crsf_data *const data = dev->data;
	int i, ret;

	/* Init TX Ring Buffer */
	atomic_set(&data->tx_busy, 0);

	LOG_INF("Initializing CRSF driver");

	for (i = 0; i < CRSF_CHANNEL_COUNT; i++) {
		data->last_reported_value[i] = 0;
		data->channel_mapping[i] = -1;
	}

	data->xfer_bytes = 0;
	data->rx_state = RX_STATE_SYNC;
	data->in_sync = false;

	for (i = 0; i < config->num_channels; i++) {
		data->channel_mapping[config->channel_info[i].crsf_channel - 1] = i;
	}

	ret = uart_configure(config->uart_dev, &uart_cfg_crsf);
	if (ret < 0) {
		LOG_ERR("Unable to configure UART port: %d", ret);
		return ret;
	}

	/* Set Async Callback */
	ret = uart_callback_set(config->uart_dev, crsf_uart_callback, (void *)dev);
	if (ret < 0) {
		LOG_ERR("Failed to set UART callback: %d", ret);
		return ret;
	}

	k_sem_init(&data->report_lock, 0, 1);

	/* Start Async RX */
	ret = uart_rx_enable(config->uart_dev, data->rx_buf_a, CRSF_RX_BUF_SIZE,
			     CRSF_RX_TIMEOUT_US);
	if (ret < 0) {
		LOG_ERR("Failed to enable UART RX: %d", ret);
		return ret;
	}

	k_thread_create(&data->thread, data->thread_stack,
			K_KERNEL_STACK_SIZEOF(data->thread_stack),
			(k_thread_entry_t)input_crsf_input_report_thread, (void *)dev, NULL, NULL,
			CONFIG_INPUT_CRSF_THREAD_PRIORITY, 0, K_NO_WAIT);

	k_thread_name_set(&data->thread, dev->name);

	return 0;
}

#define INPUT_CHANNEL_CHECK(input_channel_id)                                                      \
	BUILD_ASSERT(IN_RANGE(DT_PROP(input_channel_id, channel), 1, 16),                          \
		     "invalid channel number");                                                    \
	BUILD_ASSERT(DT_PROP(input_channel_id, type) == INPUT_EV_ABS ||                            \
			     DT_PROP(input_channel_id, type) == INPUT_EV_KEY ||                    \
			     DT_PROP(input_channel_id, type) == INPUT_EV_MSC,                      \
		     "invalid channel type");

#define CRSF_INPUT_CHANNEL_INITIALIZER(input_channel_id)                                           \
	{                                                                                          \
		.crsf_channel = DT_PROP(input_channel_id, channel),                                \
		.type = DT_PROP(input_channel_id, type),                                           \
		.zephyr_code = DT_PROP(input_channel_id, zephyr_code),                             \
	},

#define INPUT_CRSF_INIT(n)                                                                         \
                                                                                                   \
	static const struct crsf_input_channel input_##n[] = {                                     \
		DT_INST_FOREACH_CHILD(n, CRSF_INPUT_CHANNEL_INITIALIZER)};                         \
                                                                                                   \
	DT_INST_FOREACH_CHILD(n, INPUT_CHANNEL_CHECK)                                              \
                                                                                                   \
	static uint8_t __aligned(4) _dma_buffer_section crsf_##n##_rx_buf_a[CRSF_RX_BUF_SIZE];     \
	static uint8_t __aligned(4) _dma_buffer_section crsf_##n##_rx_buf_b[CRSF_RX_BUF_SIZE];     \
	static uint8_t __aligned(4) _dma_buffer_section crsf_##n##_tx_buf[CRSF_TX_BUF_SIZE];       \
	static struct input_crsf_data crsf_data_##n = {                                            \
		.rx_buf_a = crsf_##n##_rx_buf_a,                                                   \
		.rx_buf_b = crsf_##n##_rx_buf_b,                                                   \
		.tx_scratch_buf = crsf_##n##_tx_buf,                                               \
	};                                                                                         \
                                                                                                   \
	static const struct input_crsf_config crsf_cfg_##n = {                                     \
		.channel_info = input_##n,                                                         \
		.uart_dev = DEVICE_DT_GET(DT_INST_BUS(n)),                                         \
		.num_channels = ARRAY_SIZE(input_##n),                                             \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, input_crsf_init, NULL, &crsf_data_##n, &crsf_cfg_##n,             \
			      POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(INPUT_CRSF_INIT)
