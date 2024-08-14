/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#define LOG_MODULE_NAME nxp_flexio_dshot
#include <zephyr/logging/log.h>
#include <fsl_flexio.h>
#include <fsl_clock.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME, CONFIG_NXP_FLEXIO_DSHOT_LOG_LEVEL);

#include <zephyr/drivers/misc/nxp_flexio/nxp_flexio.h>

#define DT_DRV_COMPAT	nxp_flexio_dshot

struct nxp_flexio_dshot_channel {
	uint8_t dshot_channel_count;
	struct nxp_flexio_dshot_channel_config *dshot_info;
};


struct nxp_flexio_dshot_config {
	const struct device *flexio_dev;
	FLEXIO_Type *flexio_base;
	const struct pinctrl_dev_config *pincfg;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	const struct nxp_flexio_dshot_channel *channel;
	const struct nxp_flexio_child *child;
};

struct nxp_flexio_dshot_data {
	uint32_t flexio_clk;
};

struct nxp_flexio_dshot_channel_config {
	/** Flexio used pin index */
	uint8_t pin_id;
};

static int nxp_flexio_dshot_init(const struct device *dev)
{
	const struct nxp_flexio_dshot_config *config = dev->config;



	return 0;
}


#define _FLEXIO_DSHOT_GEN_CONFIG(n)								\
	{											\
		.pin_id = DT_PROP(n, pin_id),							\
	},

#define FLEXIO_DSHOT_GEN_CONFIG(n)								\
	static struct nxp_flexio_dshot_channel_config flexio_pwm_##n##_init[] = {			\
		DT_INST_FOREACH_CHILD_STATUS_OKAY(n, _FLEXIO_DSHOT_GEN_CONFIG)		\
	};											\
	static const struct nxp_flexio_dshot_channel flexio_pwm_##n##_info = {			\
		.dshot_channel_count = ARRAY_SIZE(flexio_pwm_##n##_init),			\
		.dshot_info = flexio_pwm_##n##_init,						\
	};

#define FLEXIO_DSHOT_TIMER_INDEX_INIT(n)								\
	static uint8_t flexio_pwm_##n##_timer_index[ARRAY_SIZE(flexio_pwm_##n##_init)];

#define FLEXIO_DSHOT_CHILD_CONFIG(n)								\
	static const struct nxp_flexio_child mcux_flexio_pwm_child_##n = {			\
		.isr = NULL,									\
		.user_data = NULL,								\
		.res = {									\
			.shifter_index = NULL,							\
			.shifter_count = 0,							\
			.timer_index = (uint8_t *)flexio_pwm_##n##_timer_index,			\
			.timer_count = ARRAY_SIZE(flexio_pwm_##n##_init)			\
		}										\
	};

#define FLEXIO_DSHOT_GEN_GET_CONFIG(n)							\
	.channel = &flexio_pwm_##n##_info,


#define NXP_FLEXIO_DSHOT_INIT(n)								\
	PINCTRL_DT_INST_DEFINE(n);								\
	FLEXIO_DSHOT_GEN_CONFIG(n)								\
	FLEXIO_DSHOT_TIMER_INDEX_INIT(n)								\
	FLEXIO_DSHOT_CHILD_CONFIG(n)								\
	static const struct nxp_flexio_dshot_config nxp_flexio_dshot_config_##n = {			\
		.flexio_dev = DEVICE_DT_GET(DT_INST_PARENT(n)),					\
		.flexio_base = (FLEXIO_Type *)DT_REG_ADDR(DT_INST_PARENT(n)),			\
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),					\
		.clock_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(DT_INST_PARENT(n))),			\
		.clock_subsys = (clock_control_subsys_t)DT_CLOCKS_CELL(DT_INST_PARENT(n), name),\
		.child = &mcux_flexio_pwm_child_##n,						\
		FLEXIO_DSHOT_GEN_GET_CONFIG(n)						\
	};											\
												\
	static struct nxp_flexio_dshot_data nxp_flexio_dshot_data_##n;				\
	DEVICE_DT_INST_DEFINE(n,								\
			      &nxp_flexio_dshot_init,						\
			      NULL,								\
			      &nxp_flexio_dshot_data_##n,						\
			      &nxp_flexio_dshot_config_##n,					\
			      POST_KERNEL, CONFIG_NXP_FLEXIO_DSHOT_INIT_PRIORITY,				\
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(NXP_FLEXIO_DSHOT_INIT)