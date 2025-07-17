/*
 * Copyright (c) 2025 Croxel Inc.
 * Copyright (c) 2025 CogniPilot Foundation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/check.h>
#include "bmm350.h"
#include "bmm350_stream.h"
#include "bmm350_decoder.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(BMM350_STREAM, CONFIG_SENSOR_LOG_LEVEL);

enum bmm350_stream_state {
	BMM350_STREAM_OFF = 0,
	BMM350_STREAM_ON = 1,
	BMM350_STREAM_BUSY = 2,
};

static void bmm350_stream_event_complete(struct rtio *ctx, const struct rtio_sqe *sqe, void *arg0)
{
	struct rtio_iodev_sqe *iodev_sqe = (struct rtio_iodev_sqe *)arg0;
	struct sensor_read_config *cfg = (struct sensor_read_config *)iodev_sqe->sqe.iodev->data;
	const struct device *dev = (const struct device *)sqe->userdata;
	struct bmm350_data *data = dev->data;
	uint8_t *buf;
	uint32_t buf_len;
	struct rtio_cqe *cqe;
	int err = 0;

	do {
		cqe = rtio_cqe_consume(ctx);
		if (cqe == NULL) {
			continue;
		}

		/** Keep looping through results until we get the first error.
		 * Usually this causes the remaining CQEs to result in -ECANCELED.
		 */
		if (err == 0) {
			err = cqe->result;
		}
		rtio_cqe_release(ctx, cqe);
	} while (cqe != NULL);

	if (err != 0) {
		goto bmm350_stream_evt_finish;
	}

	/* We've allocated the data already, just grab the pointer to fill comp-data
	 * now that the bus transfer is complete.
	 */
	err = rtio_sqe_rx_buf(iodev_sqe, 0, 0, &buf, &buf_len);

	CHECKIF(err != 0 || !buf || buf_len < sizeof(struct bmm350_encoded_data)) {
		LOG_ERR("Couldn't get encoded buffer on completion");
		err = -EIO;
		goto bmm350_stream_evt_finish;
	}

	err = bmm350_encode(dev, cfg, true, buf);
	if (err != 0) {
		LOG_ERR("Failed to encode frame: %d", err);
		goto bmm350_stream_evt_finish;
	}

bmm350_stream_evt_finish:
	atomic_set(&data->stream.state, BMM350_STREAM_ON);

	if (err < 0) {
		rtio_iodev_sqe_err(iodev_sqe, err);
	} else {
		rtio_iodev_sqe_ok(iodev_sqe, 0);
	}

}

/** TODO: Not working for IBI interrupts!!! */
static int bmm350_enable_events(const struct device *dev)
{
	const struct bmm350_config *cfg = dev->config;
	int err;

	if (cfg->drdy_int.port) {
		err = bmm350_prep_reg_write_async(dev, BMM350_REG_INT_CTRL, cfg->int_flags, NULL);
		if (err < 0) {
			return err;
		}
		err = gpio_pin_interrupt_configure_dt(&cfg->drdy_int, GPIO_INT_EDGE_TO_ACTIVE);
		if (err < 0) {
			return err;
		}
	} else if (cfg->bus.rtio.type == BMM350_BUS_TYPE_I3C) {
		struct rtio_sqe *out_sqe;

		err = bmm350_prep_reg_write_async(dev, BMM350_REG_INT_CTRL_IBI, 0x11, &out_sqe);
		if (err < 0) {
			return err;
		}
		out_sqe->flags |= RTIO_SQE_CHAINED;

		err = bmm350_prep_reg_write_async(dev, BMM350_REG_INT_CTRL, 0x80, NULL);
		if (err < 0) {
			return err;
		}
	} else {
		return -EIO;
	}

	rtio_submit(cfg->bus.rtio.ctx, 0);

	return 0;
}

static int bmm350_disable_events(const struct device *dev)
{
	const struct bmm350_config *cfg = dev->config;
	struct rtio_sqe *out_sqe;
	int err;

	if (cfg->drdy_int.port) {
		(void)gpio_pin_interrupt_configure_dt(&cfg->drdy_int, GPIO_INT_DISABLE);
	}

	err = bmm350_prep_reg_write_async(dev, BMM350_REG_INT_CTRL, 0, &out_sqe);
	if (err < 0) {
		return err;
	}
	out_sqe->flags |= RTIO_SQE_CHAINED;

	err = bmm350_prep_reg_write_async(dev, BMM350_REG_INT_CTRL_IBI, 0, NULL);
	if (err < 0) {
		return err;
	}

	rtio_submit(cfg->bus.rtio.ctx, 0);

	return  0;
}

static void bmm350_event_handler(const struct device *dev)
{
	struct bmm350_data *data = dev->data;
	const struct bmm350_config *cfg = dev->config;
	struct rtio_iodev_sqe *iodev_sqe = data->stream.iodev_sqe;
	uint8_t *buf = NULL;
	uint32_t buf_len = 0;
	int err;

	CHECKIF(!data->stream.iodev_sqe ||
		FIELD_GET(RTIO_SQE_CANCELED, iodev_sqe->sqe.flags)) {

		LOG_WRN("Callback triggered with no streaming submission - Disabling interrupts");

		(void)bmm350_disable_events(dev);

		(void)atomic_set(&data->stream.state, BMM350_STREAM_OFF);

		return;
	}

	CHECKIF(atomic_cas(&data->stream.state, BMM350_STREAM_ON, BMM350_STREAM_BUSY) == false) {
		LOG_WRN("Callback triggered while stream is busy. Ignoring request");
		return;
	}

	err = rtio_sqe_rx_buf(iodev_sqe,
			      sizeof(struct bmm350_encoded_data),
			      sizeof(struct bmm350_encoded_data),
			      &buf, &buf_len);
	CHECKIF(err != 0 || buf_len < sizeof(struct bmm350_encoded_data)) {
		LOG_ERR("Failed to allocate BMM350 encoded buffer: %d", err);
		rtio_iodev_sqe_err(iodev_sqe, -ENOMEM);
		return;
	}

	struct bmm350_encoded_data *edata = (struct bmm350_encoded_data *)buf;
	struct rtio_sqe *read_sqe = NULL;
	struct rtio_sqe *cb_sqe;

	err = bmm350_prep_reg_read_async(dev, BMM350_REG_MAG_X_XLSB,
					 edata->payload.buf, sizeof(edata->payload.buf),
					 &read_sqe);
	CHECKIF(err < 0 || !read_sqe) {
		rtio_iodev_sqe_err(iodev_sqe, err);
		return;
	}
	read_sqe->flags |= RTIO_SQE_CHAINED;

	cb_sqe = rtio_sqe_acquire(cfg->bus.rtio.ctx);

	rtio_sqe_prep_callback_no_cqe(cb_sqe, bmm350_stream_event_complete,
				      iodev_sqe, (void *)dev);

	rtio_submit(cfg->bus.rtio.ctx, 0);
}

static void bmm350_gpio_callback(const struct device *port, struct gpio_callback *cb, uint32_t pin)
{
	struct bmm350_stream *stream = CONTAINER_OF(cb, struct bmm350_stream, cb);
	const struct device *dev = stream->dev;

	bmm350_event_handler(dev);
}

#if DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(DT_DRV_COMPAT, i3c)
static int bmm350_ibi_cb(struct i3c_device_desc *target, struct i3c_ibi_payload *payload)
{
	bmm350_event_handler(target->dev);

	return 0;
}
#endif

void bmm350_stream_submit(const struct device *dev,
			  struct rtio_iodev_sqe *iodev_sqe)
{
	const struct sensor_read_config *read_config = iodev_sqe->sqe.iodev->data;
	struct bmm350_data *data = dev->data;
	int err;

	if ((read_config->count != 1) ||
	    (read_config->triggers[0].trigger != SENSOR_TRIG_DATA_READY)) {
		LOG_ERR("Only SENSOR_TRIG_DATA_READY is supported");
		rtio_iodev_sqe_err(iodev_sqe, -ENOTSUP);
		return;
	}

	data->stream.iodev_sqe = iodev_sqe;

	if (atomic_cas(&data->stream.state, BMM350_STREAM_OFF, BMM350_STREAM_ON)) {
		err = bmm350_enable_events(dev);
		if (err < 0) {
			rtio_iodev_sqe_err(iodev_sqe, err);
			return;
		}
	}
}

int bmm350_stream_init(const struct device *dev)
{
	struct bmm350_data *data = dev->data;
	const struct bmm350_config *cfg = dev->config;
	int err;

	/** Needed to get back the device handle from the callback context */
	data->stream.dev = dev;

	(void)atomic_set(&data->stream.state, BMM350_STREAM_OFF);

	if (cfg->drdy_int.port) {
		if (!device_is_ready(cfg->drdy_int.port)) {
			LOG_ERR("INT device is not ready");
			return -ENODEV;
		}

		err = gpio_pin_configure_dt(&cfg->drdy_int, GPIO_INPUT);
		if (err < 0) {
			return err;
		}

		gpio_init_callback(&data->stream.cb, bmm350_gpio_callback, BIT(cfg->drdy_int.pin));

		err = gpio_add_callback(cfg->drdy_int.port, &data->stream.cb);
		if (err < 0) {
			return err;
		}

#if DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(DT_DRV_COMPAT, i3c)
	} else if (cfg->bus.rtio.type == BMM350_BUS_TYPE_I3C) {
		const struct i3c_iodev_data *iodev_data = cfg->bus.rtio.iodev->data;

		struct i3c_device_desc *desc = i3c_device_find(iodev_data->bus, &cfg->bus.rtio.id);

		if (desc == NULL) {
			LOG_ERR("Failed to find I3C device");
			return -ENODEV;
		}
		desc->ibi_cb = bmm350_ibi_cb;

		err = i3c_ibi_enable(desc);
		if (err) {
			LOG_ERR("Failed to enable IBI: %d", err);
			return err;
		}
#endif
	} else {
		LOG_ERR("Unexpected stream cfg - No INT nor I3C enabled");
		return -EIO;
	}


	return bmm350_disable_events(dev);
}
