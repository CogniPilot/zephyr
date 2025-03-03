/*
 * Copyright (c) 2025 Croxel Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT pixart_paa3905

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/rtio/rtio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(PAA3905, CONFIG_SENSOR_LOG_LEVEL);

struct paa3905_data {
	struct {
		struct rtio_iodev *iodev;
		struct rtio *ctx;
	} rtio;
};

struct paa3905_config {
	uint32_t unused;
};

static void paa3905_submit(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
}

static DEVICE_API(sensor, paa3905_driver_api) = {
};

static int paa3905_init(const struct device *dev)
{
	return -1;
}

#define PAA3905_INIT(inst)									   \
												   \
	RTIO_DEFINE(paa3905_rtio_ctx_##inst, 8, 8);						   \
	SPI_DT_IODEV_DEFINE(paa3905_bus_##inst,							   \
			    DT_DRV_INST(inst),							   \
			    SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB,		   \
			    0U);								   \
												   \
	static const struct paa3905_config paa3905_cfg_##inst = {				   \
		/* No config's set from device-tree so far */					   \
	};											   \
												   \
	static struct paa3905_data paa3905_data_##inst = {					   \
		.rtio = {									   \
			.iodev = &paa3905_bus_##inst,						   \
			.ctx = &paa3905_rtio_ctx_##inst,					   \
		},										   \
	};											   \
												   \
	SENSOR_DEVICE_DT_INST_DEFINE(inst,							   \
				     paa3905_init,						   \
				     NULL,							   \
				     &paa3905_data_##inst,					   \
				     &paa3905_cfg_##inst,					   \
				     POST_KERNEL,						   \
				     CONFIG_SENSOR_INIT_PRIORITY,				   \
				     &paa3905_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PAA3905_INIT)
