/*
 * Copyright (c) 2024 CogniPilot Foundation
 * Copyright (c) 2024 NXP Semiconductors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT futaba_sbus

#include <zephyr/device.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/input/input.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/time_units.h>
#include <zephyr/sys_clock.h>
#include <zephyr/drivers/uart.h>

LOG_MODULE_REGISTER(futaba_sbus, CONFIG_INPUT_LOG_LEVEL);

/* Driver config */
struct input_channel_config {
	uint32_t channel;
	uint32_t type;
	uint32_t zephyr_code;
};

const struct uart_config uart_cfg_sbus = {.baudrate = 100000,
					  .parity = UART_CFG_PARITY_EVEN,
					  .stop_bits = UART_CFG_STOP_BITS_2,
					  .data_bits = UART_CFG_DATA_BITS_8,
					  .flow_ctrl = UART_CFG_FLOW_CTRL_NONE};

struct input_sbus_config {
	uint8_t num_channels;
	const struct input_channel_config *channel_info;
	const struct device *uart_dev;
	uart_irq_callback_user_data_t cb;
};

#define SBUS_FRAME_LEN 25
#define SBUS_HEADER    0x0F
#define SBUS_FOOTER    0x00

#define SBUS_TRANSMISSION_TIME  4
#define SBUS_INTERFRAME_SPACING 10
#define SBUS_CHANNEL_COUNT      16

#define REPORT_FILTER      CONFIG_INPUT_SBUS_REPORT_FILTER
#define CHANNEL_VALUE_ZERO 1200 /**< maximum width for a binary zero */
#define CHANNEL_VALUE_ONE  1800 /**< minimum width for a binary one */

struct input_sbus_data {
	struct k_thread thread;
	struct k_sem report_lock;

	uint16_t xfer_bytes;
	uint8_t rd_data[SBUS_FRAME_LEN];
	uint8_t sbus_frame[SBUS_FRAME_LEN];
	bool partial_sync;
	bool in_sync;
	uint32_t last_rx_time;

	uint16_t last_reported_value[SBUS_CHANNEL_COUNT];
	int8_t channel_mapping[SBUS_CHANNEL_COUNT];

	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_INPUT_SBUS_THREAD_STACK_SIZE);
};

static void input_sbus_report(const struct device *dev, unsigned int sbus_channel,
			      unsigned int value)
{
	const struct input_sbus_config *const config = dev->config;
	struct input_sbus_data *const data = dev->data;

	int channel = data->channel_mapping[sbus_channel];

	// Not Mapped
	if (channel == -1) {
		return;
	}

	if (value > (data->last_reported_value[channel] + REPORT_FILTER) ||
	    value < (data->last_reported_value[channel] - REPORT_FILTER)) {
		switch (config->channel_info[channel].type) {
		case INPUT_EV_ABS:
		case INPUT_EV_MSC:
			input_report(dev, config->channel_info[channel].type,
				     config->channel_info[channel].zephyr_code, value, false,
				     K_FOREVER);
			break;

		case INPUT_EV_KEY:
			if (value > CHANNEL_VALUE_ONE) {
				input_report_key(dev, config->channel_info[channel].zephyr_code, 1,
						 false, K_FOREVER);
			} else if (value < CHANNEL_VALUE_ZERO) {
				input_report_key(dev, config->channel_info[channel].zephyr_code, 0,
						 false, K_FOREVER);
			}
			break;
		}
		data->last_reported_value[channel] = value;
	}
}

static void input_sbus_input_report_thread(const struct device *dev, void *dummy2, void *dummy3)
{
	struct input_sbus_data *const data = dev->data;

	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);

	int i;

	while (true) {
		k_sem_take(&data->report_lock, K_FOREVER);

		// Parse the data
		uint8_t *sbus_channel_data = &data->sbus_frame[1];
		unsigned int channel = 0;
		unsigned int value = 0;
		int bitsRead = 0;

		for (i = 0; i < 22; i++) {
			// Read the next byte
			unsigned char byte = sbus_channel_data[i];

			// Extract bits and construct the 11-bit value
			value |= (byte << bitsRead);
			bitsRead += 8;

			// Check if we've read enough bits to form a full 11-bit value
			while (bitsRead >= 11) {
				input_sbus_report(dev, channel, value & 0x7FF);

				// Shift right to prepare for the next 11 bits
				value >>= 11;
				bitsRead -= 11;
				channel++;
			}
		}
	}
}

static void sbus_uart_isr(const struct device *uart_dev, void *user_data)
{
	const struct device *dev = user_data;
	struct input_sbus_data *const data = dev->data;

	if (uart_dev == NULL) {
		LOG_DBG("UART device is NULL");
		return;
	}

	if (!uart_irq_update(uart_dev)) {
		LOG_DBG("Unable to start processing interrupts");
		return;
	}

	while (uart_irq_rx_ready(uart_dev) && data->xfer_bytes <= SBUS_FRAME_LEN) {
		if (data->in_sync) {
			data->xfer_bytes +=
				uart_fifo_read(uart_dev, &data->rd_data[data->xfer_bytes],
					       SBUS_FRAME_LEN - data->xfer_bytes);
		} else if (data->partial_sync) {
			data->xfer_bytes +=
				uart_fifo_read(uart_dev, &data->rd_data[data->xfer_bytes],
					       SBUS_FRAME_LEN - data->xfer_bytes);
			if (data->xfer_bytes == SBUS_FRAME_LEN) {
				// Transfer took longer then 4ms probably faulty
				if (k_uptime_get_32() - data->last_rx_time > 4) {
					data->xfer_bytes = 0;
					data->partial_sync = false;
				} else if (data->rd_data[0] == SBUS_HEADER &&
					   data->rd_data[SBUS_FRAME_LEN - 1] == SBUS_FOOTER) {
					data->in_sync = true;
				} else {
					// Dummy read to clear fifo
					uart_fifo_read(uart_dev, &data->rd_data[0], 1);
					data->xfer_bytes = 0;
					data->partial_sync = false;
				}
			}
		} else {
			if (uart_fifo_read(uart_dev, &data->rd_data[0], 1) == 1) {
				if (data->rd_data[0] == SBUS_HEADER) {
					data->partial_sync = true;
					data->xfer_bytes = 1;
					data->last_rx_time = k_uptime_get_32();
				}
			}
		}
	}

	if (data->in_sync && data->xfer_bytes == SBUS_FRAME_LEN) {
		data->xfer_bytes = 0;
		uint32_t now = k_uptime_get_32();

		if (now - data->last_rx_time < SBUS_INTERFRAME_SPACING &&
		    data->rd_data[0] == SBUS_HEADER &&
		    data->rd_data[SBUS_FRAME_LEN - 1] == SBUS_FOOTER) {
			memcpy(data->sbus_frame, data->rd_data, SBUS_FRAME_LEN);
			k_sem_give(&data->report_lock);
		} else {
			data->partial_sync = false;
			data->in_sync = false;
		}
	}
}

/*
 * @brief Initialize sbus driver
 */
int input_sbus_init(const struct device *dev)
{
	const struct input_sbus_config *const config = dev->config;
	struct input_sbus_data *const data = dev->data;
	int i;
	int ret = 0;

	uart_irq_rx_disable(config->uart_dev);
	uart_irq_tx_disable(config->uart_dev);

	LOG_DBG("Initializing SBUS driver");

	for (i = 0; i < SBUS_CHANNEL_COUNT; i++) {
		data->last_reported_value[i] = 0;
		data->channel_mapping[i] = -1;
	}

	data->xfer_bytes = 0;
	data->in_sync = false;
	data->partial_sync = false;
	data->last_rx_time = 0;

	for (i = 0; i < config->num_channels; i++) {
		if (config->channel_info[i].channel == 0 ||
		    config->channel_info[i].channel > SBUS_CHANNEL_COUNT) {
			LOG_ERR("%s: invalid channel number %d (must be greater then 0 and "
				"smaller then %i)",
				dev->name, config->channel_info[i].channel, SBUS_CHANNEL_COUNT + 1);
			return -EINVAL;
		}
		data->channel_mapping[i] = config->channel_info[i].channel - 1;
	}

	ret = uart_configure(config->uart_dev, &uart_cfg_sbus);
	if (ret == -ENOSYS) {
		LOG_ERR("Unable to configure UART port");
		return -ENOSYS;
	}

	ret = uart_irq_callback_user_data_set(config->uart_dev, config->cb, (void *)dev);

	if (ret < 0) {
		if (ret == -ENOTSUP) {
			LOG_ERR("Interrupt-driven UART API support not enabled");
		} else if (ret == -ENOSYS) {
			LOG_ERR("UART device does not support interrupt-driven API");
		} else {
			LOG_ERR("Error setting UART callback: %d", ret);
		}
		return ret;
	}

	uart_irq_rx_enable(config->uart_dev);

	/* Initialize semaphore used by thread to report input */
	k_sem_init(&data->report_lock, 0, 1);

	k_thread_create(&data->thread, data->thread_stack, CONFIG_INPUT_SBUS_THREAD_STACK_SIZE,
			(k_thread_entry_t)input_sbus_input_report_thread, (void *)dev, NULL, NULL,
			K_PRIO_COOP(4), 0, K_NO_WAIT);

	k_thread_name_set(&data->thread, "sbus");

	return ret;
}

#define INPUT_INFO(input_channel_id)                                                               \
	{                                                                                          \
		.channel = DT_PROP(input_channel_id, channel),                                     \
		.type = DT_PROP(input_channel_id, type),                                           \
		.zephyr_code = DT_PROP(input_channel_id, zephyr_code),                             \
	},

#define INPUT_SBUS_INIT(n)                                                                         \
                                                                                                   \
	static const struct input_channel_config input_##id[] = {                                  \
		DT_INST_FOREACH_CHILD(n, INPUT_INFO)};                                             \
                                                                                                   \
	static struct input_sbus_data sbus_data_##n;                                               \
                                                                                                   \
	static const struct input_sbus_config sbus_cfg_##n = {                                     \
		.channel_info = input_##id,                                                        \
		.uart_dev = DEVICE_DT_GET(DT_INST_BUS(n)),                                         \
		.num_channels = ARRAY_SIZE(input_##id),                                            \
		.cb = sbus_uart_isr,                                                               \
	};                                                                                         \
                                                                                                   \
	static int sbus_##n##_init(const struct device *dev);                                      \
	DEVICE_DT_INST_DEFINE(n, sbus_##n##_init, NULL, &sbus_data_##n, &sbus_cfg_##n,             \
			      POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);                      \
                                                                                                   \
	static int sbus_##n##_init(const struct device *dev)                                       \
	{                                                                                          \
		return input_sbus_init(dev);                                                       \
	}

DT_INST_FOREACH_STATUS_OKAY(INPUT_SBUS_INIT)
