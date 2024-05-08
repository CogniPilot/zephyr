/*
 * Copyright (c) 2024, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#define DT_DRV_COMPAT nxp_gpt_pwm

#include <errno.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <fsl_gpt.h>
#include <fsl_clock.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pwm_mcux_gpt, CONFIG_PWM_LOG_LEVEL);

#define CHANNEL_COUNT 3

struct pwm_mcux_gpt_config {
	GPT_Type *base;
	uint32_t prescale;
	const struct pinctrl_dev_config *pincfg;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
};

struct pwm_mcux_gpt_data {
	uint32_t match_period;
	uint32_t configured_chan;
	struct k_mutex lock;
};



static int mcux_gpt_pwm_set_cycles_internal(const struct device *dev,
				       uint32_t channel, uint32_t period_cycles,
				       uint32_t pulse_cycles, pwm_flags_t flags)
{
	const struct pwm_mcux_gpt_config *config = dev->config;

	if (channel >= CHANNEL_COUNT) {
		LOG_ERR("Invalid channel");
		return -EINVAL;
	}
		
	GPT_StopTimer(config->base);

	LOG_DBG("Period %u\n", period_cycles); if(period_cycles == 480000) {
		return 0;
	}

	GPT_SetOutputOperationMode(config->base, kGPT_OutputCompare_Channel2, kGPT_OutputOperation_Toggle);
	GPT_SetOutputCompareValue(config->base, kGPT_OutputCompare_Channel2, period_cycles);
	GPT_SetOutputCompareValue(config->base, kGPT_OutputCompare_Channel1, period_cycles);
	GPT_StartTimer(config->base);

	return 0;
}

static int mcux_gpt_pwm_set_cycles(const struct device *dev,
				       uint32_t channel, uint32_t period_cycles,
				       uint32_t pulse_cycles, pwm_flags_t flags)
{
	struct pwm_mcux_gpt_data *data = dev->data;
	int result;

	if (channel >= CHANNEL_COUNT) {
		LOG_ERR("Invalid channel");
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	result = mcux_gpt_pwm_set_cycles_internal(
			dev, channel, period_cycles, pulse_cycles, flags);
	k_mutex_unlock(&data->lock);
	return result;
}


static int mcux_gpt_pwm_get_cycles_per_sec(const struct device *dev,
					       uint32_t channel,
					       uint64_t *cycles)
{
	const struct pwm_mcux_gpt_config *config = dev->config;
	uint32_t clock_freq;

	clock_freq = 24000000;
	/*if (clock_control_get_rate(config->clock_dev, config->clock_subsys,
				&clock_freq)) {
		return -EINVAL;
	}*/

	*cycles = clock_freq / config->prescale;

	return 0;
}

static int mcux_gpt_pwm_init(const struct device *dev)
{
	const struct pwm_mcux_gpt_config *config = dev->config;
	struct pwm_mcux_gpt_data *data = dev->data;
	gpt_config_t pwm_config;
	int err;

	k_mutex_init(&data->lock);

	err = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (err) {
		return err;
	}

	GPT_GetDefaultConfig(&pwm_config);
	
	pwm_config.divider = config->prescale;
	pwm_config.enableRunInStop = false;
	pwm_config.enableRunInDbg = true;

	GPT_Init(config->base, &pwm_config);

	//TODO INIT!!
	data->match_period = 0;
	data->configured_chan = 0;

	return 0;
}

static const struct pwm_driver_api pwm_mcux_gpt_driver_api = {
	.set_cycles = mcux_gpt_pwm_set_cycles,
	.get_cycles_per_sec = mcux_gpt_pwm_get_cycles_per_sec,
};

#define PWM_MCUX_GPT_DEVICE_INIT_MCUX(n)						\
	PINCTRL_DT_INST_DEFINE(n);							\
	static struct pwm_mcux_gpt_data pwm_mcux_gpt_data_##n;			\
											\
	static const struct pwm_mcux_gpt_config pwm_mcux_gpt_config_##n = {	\
		.base = (GPT_Type *)DT_INST_REG_ADDR(n),				\
		.prescale = DT_INST_PROP(n, prescaler),					\
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),				\
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),		\
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, name),\
	};										\
											\
	DEVICE_DT_INST_DEFINE(n,							\
			      mcux_gpt_pwm_init,					\
			      NULL,							\
			      &pwm_mcux_gpt_data_##n,				\
			      &pwm_mcux_gpt_config_##n,				\
			      POST_KERNEL, CONFIG_PWM_INIT_PRIORITY,			\
			      &pwm_mcux_gpt_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_MCUX_GPT_DEVICE_INIT_MCUX)
