/*
 * Copyright 2024-2025 NXP
 * Copyright 2025 Croxel, Inc.
 * Copyright 2025 CogniPilot Foundation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_lpspi

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(spi_lpspi, CONFIG_SPI_LOG_LEVEL);

#include "spi_nxp_lpspi_priv.h"

struct lpspi_driver_data {
	struct spi_rtio *rtio_ctx;
};

static void spi_nxp_iodev_start(const struct device *dev)
{
}

static void spi_nxp_iodev_prepare_start(const struct device *dev)
{
}

static void spi_mcux_iodev_complete(const struct device *dev, int status)
{
}

static void lpspi_isr(const struct device *dev)
{
}

static void lpspi_submit(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	struct lpspi_data *data = (struct lpspi_data *)dev->data;
	struct lpspi_driver_data *drv_data = (struct lpspi_driver_data *)data->driver_data;
	struct spi_rtio *rtio_ctx = drv_data->rtio_ctx;

	if (spi_rtio_submit(rtio_ctx, iodev_sqe)) {
		spi_nxp_iodev_prepare_start(dev);
		spi_nxp_iodev_start(dev);
	}
}

static int lpspi_transceive_sync(const struct device *dev, const struct spi_config *spi_cfg,
				 const struct spi_buf_set *tx_bufs,
				 const struct spi_buf_set *rx_bufs)
{
	return -1;
}

static int lpspi_init(const struct device *dev)
{
	return -1;
}

static DEVICE_API(spi, lpspi_driver_api) = {
	.transceive = lpspi_transceive_sync,
	.iodev_submit = lpspi_submit,
	.release = spi_lpspi_release,
};


#define LPSPI_RTIO_INIT(n)									   \
	SPI_NXP_LPSPI_COMMON_INIT(n)								   \
	SPI_LPSPI_CONFIG_INIT(n)								   \
												   \
	SPI_RTIO_DEFINE(spi_nxp_rtio_##n, CONFIG_SPI_NXP_RTIO_SQ_SIZE,				   \
			CONFIG_SPI_NXP_RTIO_SQ_SIZE);						   \
												   \
												   \
	static struct lpspi_driver_data lpspi_##n##_driver_data = {				   \
		.rtio_ctx = &spi_nxp_rtio_##n,							   \
	};											   \
												   \
	static struct lpspi_data lpspi_data_##n = {						   \
		SPI_NXP_LPSPI_COMMON_DATA_INIT(n)						   \
		.driver_data = &lpspi_##n##_driver_data,					   \
	};											   \
												   \
	SPI_DEVICE_DT_INST_DEFINE(n, lpspi_init, NULL, &lpspi_data_##n,				   \
				  &lpspi_config_##n, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,	   \
				  &lpspi_driver_api);

DT_INST_FOREACH_STATUS_OKAY(LPSPI_RTIO_INIT)
