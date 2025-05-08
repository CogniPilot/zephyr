/*
 * Copyright (c) 2024 NXP
 * Copyright (c) 2025 Croxel Inc.
 * Copyright (c) 2025 CogniPilot Foundation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT u_blox_m8

#include <zephyr/kernel.h>
#include <zephyr/drivers/gnss.h>

#include <zephyr/modem/ubx.h>
#include <zephyr/modem/backend/uart.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ubx_m8, CONFIG_GNSS_LOG_LEVEL);

struct ubx_m8_config {
	const struct device *bus;
};

struct ubx_m8_data {
	struct {
		struct modem_pipe *pipe;
		struct modem_backend_uart uart_backend;
		uint8_t receive_buf[128];
		uint8_t transmit_buf[128];
	} backend;
	struct {
		struct modem_ubx inst;
		uint8_t receive_buf[128];
	} ubx;
	struct {
		struct modem_ubx_script inst;
		uint8_t response_buf[128];
		struct k_sem lock;
	} script;
};

static int ubx_m8_init(const struct device *dev)
{
	int err = 0;

	{
		const struct ubx_m8_config *cfg = dev->config;
		struct ubx_m8_data *data = dev->data;

		const struct modem_ubx_config ubx_config = {
			.user_data = data,
			.receive_buf = data->ubx.receive_buf,
			.receive_buf_size = sizeof(data->ubx.receive_buf),
		};

		(void)modem_ubx_init(&data->ubx.inst, &ubx_config);

		const struct modem_backend_uart_config uart_backend_config = {
			.uart = cfg->bus,
			.receive_buf = data->backend.receive_buf,
			.receive_buf_size = sizeof(data->backend.receive_buf),
			.transmit_buf = data->backend.transmit_buf,
			.transmit_buf_size = sizeof(data->backend.transmit_buf),
		};

		data->backend.pipe = modem_backend_uart_init(&data->backend.uart_backend,
							     &uart_backend_config);
		err = modem_pipe_open(data->backend.pipe, K_MSEC(100));
		if (err != 0) {
			LOG_ERR("Failed to open Modem pipe: %d", err);
			return err;
		}

		err = modem_ubx_attach(&data->ubx.inst, data->backend.pipe);
		if (err != 0) {
			LOG_ERR("Failed to attach UBX inst to modem pipe: %d", err);
			return err;
		}

		(void)k_sem_init(&data->script.lock, 1, 1);

		data->script.inst.retry_count = 1;
		data->script.inst.timeout = K_SECONDS(1);
		data->script.inst.response.buf = data->script.response_buf;
		data->script.inst.response.buf_len = sizeof(data->script.response_buf);
	}
	{
		struct ubx_m8_data *data = dev->data;
		const static struct ubx_frame version_get = UBX_FRAME_GET_INITIALIZER(
									UBX_CLASS_ID_MON,
									UBX_MSG_ID_MON_VER);

		data->script.inst.request.buf = &version_get;
		data->script.inst.request.len = UBX_FRM_SZ(version_get.payload_size);
		data->script.inst.match.filter.class = 0;
		data->script.inst.match.filter.id = 0;

		err = modem_ubx_run_script(&data->ubx.inst, &data->script.inst);
		if (err != 0) {
			LOG_ERR("Failed to send ubx command: %d", err);
			return err;
		}
	}

	return 0;
}

static DEVICE_API(gnss, gnss_api) = {};

#define UBX_M8(inst)										   \
												   \
	static const struct ubx_m8_config ubx_m8_cfg_##inst = {					   \
		.bus = DEVICE_DT_GET(DT_INST_BUS(inst)),					   \
	};											   \
												   \
	static struct ubx_m8_data ubx_m8_data_##inst = {					   \
	};											   \
												   \
	DEVICE_DT_INST_DEFINE(inst,								   \
			      ubx_m8_init,							   \
			      NULL,								   \
			      &ubx_m8_data_##inst,						   \
			      &ubx_m8_cfg_##inst,						   \
			      POST_KERNEL,							   \
			      CONFIG_GNSS_INIT_PRIORITY,					   \
			      &gnss_api);

DT_INST_FOREACH_STATUS_OKAY(UBX_M8)
