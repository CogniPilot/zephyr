/*
 * Copyright (c) 2022, Prevas A/S
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_mcux_qdec

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <errno.h>
#include <stdint.h>

#include <fsl_enc.h>
#include <fsl_xbara.h>

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/qdec_mcux.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(qdec_mcux, CONFIG_SENSOR_LOG_LEVEL);

struct qdec_mcux_config {
	ENC_Type *base;
	const struct pinctrl_dev_config *pincfg;
	XBARA_Type *xbar;
	size_t xbar_maps_len;
	int xbar_maps[];
};

struct qdec_mcux_data {
	enc_config_t qdec_config;
	int32_t position;
	int32_t speed;
	uint16_t last_time_duration;
	int8_t last_speed_sign;
	uint16_t counts_per_revolution;
};

static enc_decoder_work_mode_t int_to_work_mode(int32_t val)
{
	return val == 0 ? kENC_DecoderWorkAsNormalMode :
			  kENC_DecoderWorkAsSignalPhaseCountMode;
}

static int qdec_mcux_attr_set(const struct device *dev, enum sensor_channel ch,
	enum sensor_attribute attr, const struct sensor_value *val)
{
	struct qdec_mcux_data *data = dev->data;

	if (ch != SENSOR_CHAN_ROTATION) {
		return -ENOTSUP;
	}

	switch ((enum sensor_attribute_qdec_mcux) attr) {
	case SENSOR_ATTR_QDEC_MOD_VAL:
		if (!IN_RANGE(val->val1, 1, UINT16_MAX)) {
			LOG_ERR("SENSOR_ATTR_QDEC_MOD_VAL value invalid");
			return -EINVAL;
		}
		data->counts_per_revolution = val->val1;
		return 0;
	case SENSOR_ATTR_QDEC_ENABLE_SINGLE_PHASE:
		data->qdec_config.decoderWorkMode =
			int_to_work_mode(val->val1);
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

	switch ((enum sensor_attribute_qdec_mcux) attr) {
	case SENSOR_ATTR_QDEC_MOD_VAL:
		val->val1 = data->counts_per_revolution;
		return 0;
	case SENSOR_ATTR_QDEC_ENABLE_SINGLE_PHASE:
		val->val1 = data->qdec_config.decoderWorkMode ==
			    kENC_DecoderWorkAsNormalMode ? 0 : 1;
		return 0;
	default:
		return -ENOTSUP;
	}
}


#define QDC_TIMER_FREQUENCY 240000000.0f //TODO dynamics in DTS
#define PRESCALER 2048 //TODO dynamic in DTS

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

	/* Read POSDH, POSDPERH and LASTEDGEH */
	uint16_t dummy = ENC_GetPositionDifferenceValue(config->base);
	int16_t POSDH = ENC_GetHoldPositionDifferenceValue(config->base);
	uint16_t POSDPERH = ENC_GetHoldPositionDifferencePeriodValue(config->base);
	uint16_t LASTEDGEH = ENC_GetHoldLastEdgeTimeValue(config->base);
	uint16_t period;
	int8_t speed_sign;
	float speed;
	int64_t i64Numerator;
	float speedCalConst = 123;

	/* POSDH == 0? */
	if(POSDH != 0)
	{
		/* Shaft is moving during speed measurement interval */
		POSDH = POSDH;
		period = POSDPERH;
		data->last_time_duration = period;

		if(POSDH > 0)
		{
			speed_sign = 1;
		}
		else
		{
			speed_sign = -1;
		}

		if(speed_sign == data->last_speed_sign)
		{
			/* Calculate speed */
			speed = POSDH / (POSDPERH * (1.0f / (QDC_TIMER_FREQUENCY / 2048))) /
			        data->counts_per_revolution * 60;
		}
		else
		{
			speed = 0;
		}
		data->last_speed_sign = speed_sign;
	}
	else
	{
		speed = 0;
	}

	/*
	this->sSpeed.f16SpeedFilt = GDFLIB_FilterIIR1_F16(MLIB_Conv_F16l(speed), &this->sSpeed.sQDCSpeedFilter);
	this->sSpeed.fltSpeed = MLIB_ConvSc_FLTsf(this->sSpeed.f16SpeedFilt, this->sSpeed.fltSpeedFrac16ToAngularCoeff);*/


#endif

	//LOG_DBG("pos %d", data->position);
	LOG_DBG("pos %d RPM %f diff %d period @240Mhz/2048 %d cpr %d", data->position, speed, POSDH, POSDPERH, data->counts_per_revolution);

	return 0;
}

static int qdec_mcux_ch_get(const struct device *dev, enum sensor_channel ch,
			 struct sensor_value *val)
{
	struct qdec_mcux_data *data = dev->data;

	double rotation = (data->position * 2.0 * M_PI) / data->counts_per_revolution;

	switch (ch) {
	case SENSOR_CHAN_ROTATION:
		sensor_value_from_double(val, rotation);
		break;
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

static void init_inputs(const struct device *dev)
{
	int i;
	const struct qdec_mcux_config *config = dev->config;

	i = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	assert(i == 0);

	/* Quadrature Encoder inputs are only accessible via crossbar */
	XBARA_Init(config->xbar);
	for (i = 0; i < config->xbar_maps_len; i += 2) {
		XBARA_SetSignalsConnection(config->xbar, config->xbar_maps[i],
					   config->xbar_maps[i + 1]);
	}
}

#define XBAR_PHANDLE(n)	DT_INST_PHANDLE(n, xbar)

#define QDEC_CHECK_COND(n, p, min, max)						\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, p), (				\
		    BUILD_ASSERT(IN_RANGE(DT_INST_PROP(n, p), min, max),	\
				 STRINGIFY(p) " value is out of range")), ())

#define QDEC_SET_COND(n, v, p)							\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, p), (v = DT_INST_PROP(n, p)), ())

#define QDEC_MCUX_INIT(n)							\
										\
	BUILD_ASSERT((DT_PROP_LEN(XBAR_PHANDLE(n), xbar_maps) % 2) == 0,	\
			"xbar_maps length must be an even number");		\
	QDEC_CHECK_COND(n, counts_per_revolution, 1, UINT16_MAX);		\
	QDEC_CHECK_COND(n, filter_sample_period, 0, UINT8_MAX);			\
										\
	static struct qdec_mcux_data qdec_mcux_##n##_data = {			\
		.counts_per_revolution = DT_INST_PROP(n, counts_per_revolution) \
	};									\
										\
	PINCTRL_DT_INST_DEFINE(n);						\
										\
	static const struct qdec_mcux_config qdec_mcux_##n##_config = {		\
		.base = (ENC_Type *)DT_INST_REG_ADDR(n),			\
		.xbar = (XBARA_Type *)DT_REG_ADDR(XBAR_PHANDLE(n)),		\
		.xbar_maps_len = DT_PROP_LEN(XBAR_PHANDLE(n), xbar_maps),	\
		.xbar_maps = DT_PROP(XBAR_PHANDLE(n), xbar_maps),		\
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),			\
	};									\
										\
	static int qdec_mcux_##n##_init(const struct device *dev)		\
	{									\
		const struct qdec_mcux_config *config = dev->config;		\
		struct qdec_mcux_data *data = dev->data;			\
										\
		LOG_DBG("Initializing %s", dev->name);				\
										\
		init_inputs(dev);						\
										\
		ENC_GetDefaultConfig(&data->qdec_config);			\
		data->qdec_config.decoderWorkMode = int_to_work_mode(		\
			DT_INST_PROP(n, single_phase_mode));			\
		QDEC_SET_COND(n, data->qdec_config.filterCount, filter_count);	\
		QDEC_SET_COND(n, data->qdec_config.filterSamplePeriod,		\
			  filter_sample_period);				\
		LOG_DBG("Latency is %u filter clock cycles + 2 IPBus clock "	\
			"periods", data->qdec_config.filterSamplePeriod *	\
			(data->qdec_config.filterCount + 3));			\
		ENC_Init(config->base, &data->qdec_config);			\
										\
		/* Update the position counter with initial value. */		\
		ENC_DoSoftwareLoadInitialPositionValue(config->base);		\
										\
		return 0;							\
	}									\
										\
										\
	SENSOR_DEVICE_DT_INST_DEFINE(n, qdec_mcux_##n##_init, NULL,		\
			      &qdec_mcux_##n##_data, &qdec_mcux_##n##_config,	\
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,		\
			      &qdec_mcux_api);					\
										\

DT_INST_FOREACH_STATUS_OKAY(QDEC_MCUX_INIT)
