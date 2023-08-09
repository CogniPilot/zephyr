/*
 * Copyright (c) 2022, Prevas A/S
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_mcux_qdec

#include <errno.h>
#include <stdint.h>

#include <fsl_enc.h>
#include <fsl_xbara.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/qdec_mcux.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(qdec_mcux, CONFIG_SENSOR_LOG_LEVEL);

struct qdec_mcux_config {
	ENC_Type *base;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	const struct pinctrl_dev_config *pincfg;
	uint16_t filterCount;
	uint16_t filterSamplePeriod;
	int32_t single_phase_mode;
	enc_prescaler_t ipbus_prescaler;
	int32_t speed_measurement;
	XBARA_Type *xbar;
	size_t xbar_maps_len;
	int xbar_maps[];
};

struct qdec_mcux_data {
	enc_config_t qdec_config;
	int32_t position;
	uint16_t counts_per_revolution;
#if (defined(FSL_FEATURE_ENC_HAS_POSDPER) && FSL_FEATURE_ENC_HAS_POSDPER)
	double clock_period;
	double speed;
	int8_t last_speed_sign;
#endif
};

static enc_decoder_work_mode_t int_to_work_mode(int32_t val)
{
	return val == 0 ? kENC_DecoderWorkAsNormalMode : kENC_DecoderWorkAsSignalPhaseCountMode;
}

static int qdec_calc_clock_period(const struct device *dev)
{
	const struct qdec_mcux_config *config = dev->config;
	struct qdec_mcux_data *data = dev->data;

	if (config->speed_measurement) {
		uint32_t clock_freq;
		if (clock_control_get_rate(config->clock_dev, config->clock_subsys, &clock_freq)) {
			return -EINVAL;
		}
		data->clock_period =
			1.0f / ((double)clock_freq / (2 << (data->qdec_config.prescalerValue - 1)));
	}
	return 0;
}

static int qdec_mcux_attr_set(const struct device *dev, enum sensor_channel ch,
			      enum sensor_attribute attr, const struct sensor_value *val)
{
	const struct qdec_mcux_config *config = dev->config;
	struct qdec_mcux_data *data = dev->data;

	if (ch != SENSOR_CHAN_ROTATION) {
		return -ENOTSUP;
	}

	switch ((enum sensor_attribute_qdec_mcux)attr) {
	case SENSOR_ATTR_QDEC_MOD_VAL:
		if (!IN_RANGE(val->val1, 1, UINT16_MAX)) {
			LOG_ERR("SENSOR_ATTR_QDEC_MOD_VAL value invalid");
			return -EINVAL;
		}
		data->counts_per_revolution = val->val1;
		return 0;
	case SENSOR_ATTR_QDEC_ENABLE_SINGLE_PHASE:
		data->qdec_config.decoderWorkMode = int_to_work_mode(val->val1);
		/* Reini */
		ENC_Init(config->base, &data->qdec_config);
		return 0;
	case SENSOR_ATTR_QDEC_IPBUS_PRESCALER:
		if (val->val1 % 2 == 0) {
			data->qdec_config.prescalerValue = ilog2(val->val1);
			ENC_Init(config->base, &data->qdec_config);
			return qdec_calc_clock_period(dev);
		} else {
			return -EINVAL;
		}
		return 0;
	default:
		return -ENOTSUP;
	}
}

static int qdec_mcux_attr_get(const struct device *dev, enum sensor_channel ch,
			      enum sensor_attribute attr, struct sensor_value *val)
{
	struct qdec_mcux_data *data = dev->data;

	if (ch != SENSOR_CHAN_ROTATION) {
		return -ENOTSUP;
	}

	switch ((enum sensor_attribute_qdec_mcux)attr) {
	case SENSOR_ATTR_QDEC_MOD_VAL:
		val->val1 = data->counts_per_revolution;
		return 0;
	case SENSOR_ATTR_QDEC_ENABLE_SINGLE_PHASE:
		val->val1 =
			data->qdec_config.decoderWorkMode == kENC_DecoderWorkAsNormalMode ? 0 : 1;
		return 0;
	case SENSOR_ATTR_QDEC_IPBUS_PRESCALER:
		val->val1 = 2 << (data->qdec_config.prescalerValue - 1);
		return 0;
	default:
		return -ENOTSUP;
	}
}

static int qdec_mcux_fetch(const struct device *dev, enum sensor_channel ch)
{
	const struct qdec_mcux_config *config = dev->config;
	struct qdec_mcux_data *data = dev->data;

	if (ch != SENSOR_CHAN_ALL) {
		return -ENOTSUP;
	}

	/* Read position */
	data->position = ENC_GetPositionValue(config->base);

#if (defined(FSL_FEATURE_ENC_HAS_POSDPER) && FSL_FEATURE_ENC_HAS_POSDPER)
	if (config->speed_measurement) {
		uint16_t dummy = ENC_GetPositionDifferenceValue(config->base);
		(void)dummy;

		int16_t pos_diff = ENC_GetHoldPositionDifferenceValue(config->base);
		uint16_t period = ENC_GetHoldPositionDifferencePeriodValue(config->base);
		int8_t speed_sign;

		/* Shaft has moved during speed measurement interval */
		if (pos_diff != 0) {
			if (period == UINT16_MAX) {
				LOG_WRN("%s, period equals UINT16_MAX can't calculate speed. "
					"Please increase the sampling rate or increase the "
					"prescaler to avoid period overflowing",
					dev->name);
				return -EOVERFLOW;
			}

			if (pos_diff > 0) {
				speed_sign = 1;
			} else {
				speed_sign = -1;
			}

			if (speed_sign == data->last_speed_sign) {
				/* Calculate speed in rad/s */
				data->speed =
					(pos_diff / (period * data->clock_period) * 2.0 * M_PI) /
					data->counts_per_revolution;
			} else {
				data->speed = 0;
			}
			data->last_speed_sign = speed_sign;
		} else {
			data->speed = 0;
		}

		LOG_DBG("%lf rad/s period %i", data->speed, period);
	}
#endif

	LOG_DBG("pos %d", data->position);

	return 0;
}

static int qdec_mcux_ch_get(const struct device *dev, enum sensor_channel ch,
			    struct sensor_value *val)
{
	const struct qdec_mcux_config *config = dev->config;
	struct qdec_mcux_data *data = dev->data;

	switch (ch) {
	case SENSOR_CHAN_ROTATION:
		sensor_value_from_float(val, (data->position * 360.0f)
					/ data->counts_per_revolution);
		break;
#if (defined(FSL_FEATURE_ENC_HAS_POSDPER) && FSL_FEATURE_ENC_HAS_POSDPER)
	case SENSOR_CHAN_RPM:
		if (config->speed_measurement) {
			sensor_value_from_double(val, data->speed);
		} else {
			return -ENOTSUP;
		}
		break;
#endif
	default:
		return -ENOTSUP;
	}

	return 0;
}

static const struct sensor_driver_api qdec_mcux_api = {
	.attr_set = &qdec_mcux_attr_set,
	.attr_get = &qdec_mcux_attr_get,
	.sample_fetch = &qdec_mcux_fetch,
	.channel_get = &qdec_mcux_ch_get,
};

static int qdec_mcux_init(const struct device *dev)
{
	int i;
	const struct qdec_mcux_config *config = dev->config;
	struct qdec_mcux_data *data = dev->data;

	LOG_DBG("Initializing %s", dev->name);

	i = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	assert(i == 0);

	/* Quadrature Encoder inputs are only accessible via crossbar */
	XBARA_Init(config->xbar);
	for (i = 0; i < config->xbar_maps_len; i += 2) {
		XBARA_SetSignalsConnection(config->xbar, config->xbar_maps[i],
					   config->xbar_maps[i + 1]);
	}

	ENC_GetDefaultConfig(&data->qdec_config);
	data->qdec_config.decoderWorkMode = int_to_work_mode(config->single_phase_mode);
	data->qdec_config.filterCount = config->filterCount;
	data->qdec_config.filterSamplePeriod = config->filterSamplePeriod;
	data->qdec_config.prescalerValue = config->ipbus_prescaler;
	LOG_DBG("Latency is %u filter clock cycles + 2 IPBus clock periods",
		data->qdec_config.filterSamplePeriod * (data->qdec_config.filterCount + 3));

#if (defined(FSL_FEATURE_ENC_HAS_POSDPER) && FSL_FEATURE_ENC_HAS_POSDPER)
	if (qdec_calc_clock_period(dev) < 0) {
		return -EINVAL;
	}
#endif

	ENC_Init(config->base, &data->qdec_config);

	/* Update the position counter with initial value. */
	ENC_DoSoftwareLoadInitialPositionValue(config->base);

	return 0;
}

#define XBAR_PHANDLE(n) DT_INST_PHANDLE(n, xbar)

#define QDEC_CHECK_COND(n, p, min, max)                                                            \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, p),                                                   \
		    (BUILD_ASSERT(IN_RANGE(DT_INST_PROP(n, p), min, max),                          \
				  STRINGIFY(p) " value is out of range")),                         \
		     ())

#define QDEC_MCUX_INIT(n)                                                                          \
                                                                                                   \
	BUILD_ASSERT((DT_PROP_LEN(XBAR_PHANDLE(n), xbar_maps) % 2) == 0,                           \
		     "xbar_maps length must be an even number");                                   \
	QDEC_CHECK_COND(n, counts_per_revolution, 1, UINT16_MAX);                                  \
	QDEC_CHECK_COND(n, filter_sample_period, 0, UINT8_MAX);                                    \
                                                                                                   \
	static struct qdec_mcux_data qdec_mcux_##n##_data = {                                      \
		.counts_per_revolution = DT_INST_PROP(n, counts_per_revolution)};                  \
                                                                                                   \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
                                                                                                   \
	static const struct qdec_mcux_config qdec_mcux_##n##_config = {                            \
		.base = (ENC_Type *)DT_INST_REG_ADDR(n),                                           \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, name),              \
		.xbar = (XBARA_Type *)DT_REG_ADDR(XBAR_PHANDLE(n)),                                \
		.xbar_maps_len = DT_PROP_LEN(XBAR_PHANDLE(n), xbar_maps),                          \
		.xbar_maps = DT_PROP(XBAR_PHANDLE(n), xbar_maps),                                  \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                       \
		.filterCount = DT_INST_PROP_OR(n, filterCount, 0),                                 \
		.filterSamplePeriod = DT_INST_PROP_OR(n, filterSamplePeriod, 0),                   \
		.single_phase_mode = DT_INST_PROP(n, single_phase_mode),                           \
		.speed_measurement = DT_INST_PROP(n, speed_measurement),                           \
		.ipbus_prescaler = DT_INST_ENUM_IDX(n, ipbus_prescaler),                           \
	};                                                                                         \
                                                                                                   \
	SENSOR_DEVICE_DT_INST_DEFINE(n, qdec_mcux_init, NULL, &qdec_mcux_##n##_data,               \
				     &qdec_mcux_##n##_config, POST_KERNEL,                         \
				     CONFIG_SENSOR_INIT_PRIORITY, &qdec_mcux_api);

DT_INST_FOREACH_STATUS_OKAY(QDEC_MCUX_INIT)
