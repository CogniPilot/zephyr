/*
 * Copyright 2025 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_tpm_qdec

#include <errno.h>
#include <soc.h>
#include <fsl_tpm.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/qdec_mcux.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(qdec_tpm, CONFIG_PWM_LOG_LEVEL);

struct qdec_tpm_config {
	TPM_Type *base;
	tpm_clock_prescale_t prescale;
	const struct pinctrl_dev_config *pincfg;
	const tpm_phase_params_t phaseParams;
};

struct qdec_tpm_data {
	int32_t position;
	uint16_t counts_per_revolution;
};

static int qdec_tpm_attr_set(const struct device *dev, enum sensor_channel ch,
			     enum sensor_attribute attr, const struct sensor_value *val)
{
	struct qdec_tpm_data *data = dev->data;

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
	default:
		return -ENOTSUP;
	}
}

static int qdec_tpm_attr_get(const struct device *dev, enum sensor_channel ch,
			     enum sensor_attribute attr, struct sensor_value *val)
{
	struct qdec_tpm_data *data = dev->data;

	if (ch != SENSOR_CHAN_ROTATION) {
		return -ENOTSUP;
	}

	switch ((enum sensor_attribute_qdec_mcux)attr) {
	case SENSOR_ATTR_QDEC_MOD_VAL:
		val->val1 = data->counts_per_revolution;
		return 0;
	default:
		return -ENOTSUP;
	}
}

static int qdec_tpm_fetch(const struct device *dev, enum sensor_channel ch)
{
	const struct qdec_tpm_config *config = dev->config;
	struct qdec_tpm_data *data = dev->data;

	if (ch != SENSOR_CHAN_ALL) {
		return -ENOTSUP;
	}

	/* Read position */
	data->position = TPM_GetCurrentTimerCount(config->base);

	LOG_DBG("pos %d", data->position);

	return 0;
}

static int qdec_tpm_ch_get(const struct device *dev, enum sensor_channel ch,
			   struct sensor_value *val)
{
	struct qdec_tpm_data *data = dev->data;

	switch (ch) {
	case SENSOR_CHAN_ROTATION:
		sensor_value_from_float(val,
					(data->position * 360.0f) / data->counts_per_revolution);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int qdec_tpm_init(const struct device *dev)
{
	const struct qdec_tpm_config *config = dev->config;
	tpm_config_t tpm_config;
	int err;

	err = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (err) {
		return err;
	}

	TPM_GetDefaultConfig(&tpm_config);
	tpm_config.prescale = config->prescale;

	TPM_Init(config->base, &tpm_config);

	TPM_SetTimerPeriod(config->base, 0xFFFFFFFF);

	TPM_SetupQuadDecode(config->base, &config->phaseParams, &config->phaseParams,
			    kTPM_QuadPhaseEncode);

	TPM_StartTimer(config->base, kTPM_SystemClock);

	return 0;
}

static DEVICE_API(sensor, qdec_tpm_api) = {
	.attr_set = &qdec_tpm_attr_set,
	.attr_get = &qdec_tpm_attr_get,
	.sample_fetch = &qdec_tpm_fetch,
	.channel_get = &qdec_tpm_ch_get,
};

#define TO_TPM_PRESCALE_DIVIDE(val) _DO_CONCAT(kTPM_Prescale_Divide_, val)

#define QDEC_MCUX_INIT(n)                                                                          \
                                                                                                   \
	static struct qdec_tpm_data qdec_mcux_##n##_data = {                                       \
		.counts_per_revolution = DT_INST_PROP(n, counts_per_revolution)};                  \
                                                                                                   \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
                                                                                                   \
	static const struct qdec_tpm_config qdec_mcux_##n##_config = {                             \
		.base = (TPM_Type *)DT_INST_REG_ADDR(n),                                           \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                       \
		.prescale = TO_TPM_PRESCALE_DIVIDE(DT_INST_PROP(n, prescaler)),                    \
		.phaseParams = {0, kTPM_QuadPhaseNormal}};                                         \
                                                                                                   \
	SENSOR_DEVICE_DT_INST_DEFINE(n, qdec_tpm_init, NULL, &qdec_mcux_##n##_data,                \
				     &qdec_mcux_##n##_config, POST_KERNEL,                         \
				     CONFIG_SENSOR_INIT_PRIORITY, &qdec_tpm_api);

DT_INST_FOREACH_STATUS_OKAY(QDEC_MCUX_INIT)