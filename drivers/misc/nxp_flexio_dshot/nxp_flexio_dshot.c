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

#define DT_DRV_COMPAT nxp_flexio_dshot

#define DSHOT_TIMERS             FLEXIO_SHIFTBUFNIS_COUNT
#define DSHOT_THROTTLE_POSITION  5u
#define DSHOT_TELEMETRY_POSITION 4u
#define NIBBLES_SIZE             4u
#define DSHOT_NUMBER_OF_NIBBLES  3u

static const uint32_t gcr_decode[32] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x9, 0xA,
					0xB, 0x0, 0xD, 0xE, 0xF, 0x0, 0x0, 0x2, 0x3, 0x0, 0x5,
					0x6, 0x7, 0x0, 0x0, 0x8, 0x1, 0x0, 0x4, 0xC, 0x0};

typedef enum {
	DSHOT_START = 0,
	DSHOT_12BIT_FIFO,
	DSHOT_12BIT_TRANSFERRED,
	DSHOT_TRANSMIT_COMPLETE,
	BDSHOT_RECEIVE,
	BDSHOT_RECEIVE_COMPLETE,
} dshot_state;

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
	uint32_t dshot_tcmp;
	uint32_t bdshot_tcmp;
	uint32_t dshot_mask;
	uint32_t bdshot_recv_mask;
	uint32_t bdshot_parsed_recv_mask;
};

struct nxp_flexio_dshot_channel_config {
	/** Flexio used pin index */
	uint8_t pin_id;
	bool init;
	uint32_t data_seg1;
	uint32_t irq_data;
	dshot_state state;
	bool bdshot;
	uint32_t raw_response;
	uint16_t erpm;
	uint32_t crc_error_cnt;
	uint32_t frame_error_cnt;
	uint32_t no_response_cnt;
	uint32_t last_no_response_cnt;
};

static void nxp_flexio_dshot_output(const struct device *dev, uint32_t channel)
{
	const struct nxp_flexio_dshot_config *config = dev->config;
	struct nxp_flexio_dshot_data *data = dev->data;
	FLEXIO_Type *flexio_base = (FLEXIO_Type *)(config->flexio_base);
	struct nxp_flexio_child *child = (struct nxp_flexio_child *)(config->child);
	struct nxp_flexio_dshot_channel_config *dshot_info = &config->channel->dshot_info[channel];

	flexio_timer_config_t timerConfig;
	flexio_shifter_config_t shifterConfig;

	/* Disable Shifter */
	(void)memset(&shifterConfig, 0, sizeof(shifterConfig));
	FLEXIO_SetShifterConfig(flexio_base, child->res.timer_index[channel], &shifterConfig);

	/* No start bit, stop bit low */
	shifterConfig.inputSource = kFLEXIO_ShifterInputFromPin;
	shifterConfig.shifterStop = kFLEXIO_ShifterStopBitLow;
	shifterConfig.shifterStart = kFLEXIO_ShifterStartBitDisabledLoadDataOnEnable;

	/* Transmit mode, output to FXIO pin, inverted output for bdshot */
	shifterConfig.timerSelect = channel;
	shifterConfig.timerPolarity = kFLEXIO_ShifterTimerPolarityOnPositive;
	shifterConfig.pinConfig = kFLEXIO_PinConfigOutput;
	shifterConfig.pinSelect = dshot_info->pin_id;
	shifterConfig.pinPolarity = dshot_info->bdshot;
	shifterConfig.shifterMode = kFLEXIO_ShifterModeTransmit;

	FLEXIO_SetShifterConfig(flexio_base, child->res.timer_index[channel], &shifterConfig);

	(void)memset(&timerConfig, 0, sizeof(timerConfig));

	/* Start transmitting on trigger, disable on compare */
	timerConfig.timerOutput = kFLEXIO_TimerOutputOneNotAffectedByReset;
	timerConfig.timerDecrement = kFLEXIO_TimerDecSrcOnFlexIOClockShiftTimerOutput;
	timerConfig.timerReset = kFLEXIO_TimerResetNever;
	timerConfig.timerDisable = kFLEXIO_TimerDisableOnTimerCompare;
	timerConfig.timerEnable = kFLEXIO_TimerEnableOnTriggerHigh;
	timerConfig.timerStop = kFLEXIO_TimerStopBitDisabled;
	timerConfig.timerStart = kFLEXIO_TimerStartBitDisabled;

	timerConfig.timerCompare = data->dshot_tcmp;

	/* Baud mode, Trigger on shifter write */
	timerConfig.triggerSelect = (4 * channel) + 1;
	timerConfig.triggerPolarity = kFLEXIO_TimerTriggerPolarityActiveLow;
	timerConfig.triggerSource = kFLEXIO_TimerTriggerSourceInternal;
	timerConfig.pinConfig = kFLEXIO_PinConfigOutputDisabled;
	timerConfig.pinSelect = 0;
	timerConfig.pinPolarity = kFLEXIO_PinActiveLow;
	timerConfig.timerMode = kFLEXIO_TimerModeDual8BitBaudBit;

	FLEXIO_SetTimerConfig(flexio_base, child->res.timer_index[channel], &timerConfig);
}

static void nxp_flexio_bdshot_input(const struct device *dev, uint32_t channel)
{
	const struct nxp_flexio_dshot_config *config = dev->config;
	struct nxp_flexio_dshot_data *data = dev->data;
	FLEXIO_Type *flexio_base = (FLEXIO_Type *)(config->flexio_base);
	struct nxp_flexio_child *child = (struct nxp_flexio_child *)(config->child);
	struct nxp_flexio_dshot_channel_config *dshot_info = &config->channel->dshot_info[channel];

	flexio_timer_config_t timerConfig;
	flexio_shifter_config_t shifterConfig;

	(void)memset(&timerConfig, 0, sizeof(timerConfig));
	(void)memset(&shifterConfig, 0, sizeof(shifterConfig));

	/* Transmit done, disable timer and reconfigure to receive*/
	FLEXIO_SetTimerConfig(flexio_base, child->res.timer_index[channel], &timerConfig);

	/* Input data from pin, no start/stop bit*/
	shifterConfig.inputSource = kFLEXIO_ShifterInputFromPin;
	shifterConfig.shifterStop = kFLEXIO_ShifterStopBitDisable;
	shifterConfig.shifterStart = kFLEXIO_ShifterStartBitDisabledLoadDataOnShift;

	/* Shifter receive mode, on FXIO pin input */
	shifterConfig.timerSelect = channel;
	shifterConfig.timerPolarity = kFLEXIO_ShifterTimerPolarityOnPositive;
	shifterConfig.pinConfig = kFLEXIO_PinConfigOutputDisabled;
	shifterConfig.pinSelect = dshot_info->pin_id;
	shifterConfig.pinPolarity = kFLEXIO_PinActiveLow;
	shifterConfig.shifterMode = kFLEXIO_ShifterModeReceive;

	FLEXIO_SetShifterConfig(flexio_base, child->res.timer_index[channel], &shifterConfig);

	/* Make sure there no shifter flags high from transmission */
	FLEXIO_ClearShifterStatusFlags(flexio_base, 1 << channel);

	/* Enable on pin transition, resychronize through reset on rising
	 * edge */
	timerConfig.timerOutput = kFLEXIO_TimerOutputOneAffectedByReset;
	timerConfig.timerDecrement = kFLEXIO_TimerDecSrcOnFlexIOClockShiftTimerOutput;
	timerConfig.timerReset = kFLEXIO_TimerResetOnTimerPinRisingEdge;
	timerConfig.timerDisable = kFLEXIO_TimerDisableOnTimerCompare;
	timerConfig.timerEnable = kFLEXIO_TimerEnableOnTriggerBothEdge;
	timerConfig.timerStop = kFLEXIO_TimerStopBitDisabled;
	timerConfig.timerStart = kFLEXIO_TimerStartBitEnabled;

	timerConfig.timerCompare = data->bdshot_tcmp;

	/* Baud mode, Trigger on shifter write */
	timerConfig.triggerSelect = 2 * dshot_info->pin_id;
	timerConfig.triggerPolarity = kFLEXIO_TimerTriggerPolarityActiveHigh;
	timerConfig.triggerSource = kFLEXIO_TimerTriggerSourceInternal;
	timerConfig.pinConfig = kFLEXIO_PinConfigOutputDisabled;
	timerConfig.pinSelect = 0;
	timerConfig.pinPolarity = kFLEXIO_PinActiveLow;
	timerConfig.timerMode = kFLEXIO_TimerModeDual8BitBaudBit;

	FLEXIO_SetTimerConfig(flexio_base, child->res.timer_index[channel], &timerConfig);
}

static int nxp_flexio_dshot_init(const struct device *dev)
{
	const struct nxp_flexio_dshot_config *config = dev->config;
	struct nxp_flexio_dshot_data *data = dev->data;
	uint8_t channel = 0;
	int err;
	struct nxp_flexio_child *child = (struct nxp_flexio_child *)(config->child);

	if (!device_is_ready(config->clock_dev)) {
		return -ENODEV;
	}

	if (clock_control_get_rate(config->clock_dev, config->clock_subsys, &data->flexio_clk)) {
		return -EINVAL;
	}

	const int dshot_pwm_freq = 600000; // FIXME

	/* Calculate dshot timings based on dshot_pwm_freq */
	data->dshot_tcmp = 0x2F00 | (((data->flexio_clk / (dshot_pwm_freq * 3) / 2) - 1) & 0xFF);
	data->bdshot_tcmp =
		0x2900 | (((data->flexio_clk / (dshot_pwm_freq * 5 / 4) / 2) - 3) & 0xFF);

	err = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (err) {
		return err;
	}

	// FIXME use timer and shifter from child_attach allocation

	err = nxp_flexio_child_attach(config->flexio_dev, child);
	if (err < 0) {
		return err;
	}

	for (channel = 0; channel < config->channel->dshot_channel_count; channel++) {
		nxp_flexio_dshot_output(dev, channel);
	}

	return 0;
}

void nxp_flexio_dshot_trigger(const struct device *dev)
{
	const struct nxp_flexio_dshot_config *config = dev->config;
	struct nxp_flexio_dshot_data *data = dev->data;
	FLEXIO_Type *flexio_base = (FLEXIO_Type *)(config->flexio_base);
	struct nxp_flexio_dshot_channel_config *dshot_info;

	// Calc data now since we're not event driven
	if (data->bdshot_recv_mask != 0x0) {
		// up_bdshot_erpm(); //FIXME
	}

	FLEXIO_ClearTimerStatusFlags(flexio_base, 0xFF);

	for (uint8_t channel = 0; (channel < config->channel->dshot_channel_count); channel++) {
		dshot_info = &config->channel->dshot_info[channel];

		if (dshot_info->bdshot && (data->bdshot_recv_mask & (1 << channel)) == 0) {
			dshot_info->no_response_cnt++;
		}

		if (dshot_info->init && dshot_info->data_seg1 != 0) {
			flexio_base->SHIFTBUF[channel] = dshot_info->data_seg1;
		}
	}

	data->bdshot_recv_mask = 0x0;

	FLEXIO_ClearTimerStatusFlags(flexio_base, 0xFF);
	FLEXIO_EnableShifterStatusInterrupts(flexio_base, 0xFF);
	FLEXIO_EnableTimerStatusInterrupts(flexio_base, 0xFF);
}

/* Expand packet from 16 bits 48 to get T0H and T1H timing */
uint64_t nxp_flexio_dshot_expand_data(uint16_t packet)
{
	unsigned int mask;
	unsigned int index = 0;
	uint64_t expanded = 0x0;

	for (mask = 0x8000; mask != 0; mask >>= 1) {
		if (packet & mask) {
			expanded = expanded | ((uint64_t)0x3 << index);

		} else {
			expanded = expanded | ((uint64_t)0x1 << index);
		}

		index = index + 3;
	}

	return expanded;
}

/**
 * bits 	1-11	- throttle value (0-47 are reserved, 48-2047 give 2000 steps of throttle
 *resolution) bit 	12		- dshot telemetry enable/disable bits 	13-16	- XOR
 *checksum
 **/
void nxp_flexio_dshot_data_set(const struct device *dev, unsigned channel, uint16_t throttle,
			       bool telemetry)
{
	const struct nxp_flexio_dshot_config *config = dev->config;
	struct nxp_flexio_dshot_channel_config *dshot_info = &config->channel->dshot_info[channel];
	FLEXIO_Type *flexio_base = (FLEXIO_Type *)(config->flexio_base);

	if (channel < DSHOT_TIMERS && dshot_info->init) {
		uint16_t csum_data;
		uint16_t packet = 0;
		uint16_t checksum = 0;

		packet |= throttle << DSHOT_THROTTLE_POSITION;
		packet |= ((uint16_t)telemetry & 0x01) << DSHOT_TELEMETRY_POSITION;

		if (dshot_info->bdshot) {
			csum_data = ~packet;

		} else {
			csum_data = packet;
		}

		/* XOR checksum calculation */
		csum_data >>= NIBBLES_SIZE;

		for (unsigned i = 0; i < DSHOT_NUMBER_OF_NIBBLES; i++) {
			checksum ^= (csum_data & 0x0F); // XOR data by nibbles
			csum_data >>= NIBBLES_SIZE;
		}

		packet |= (checksum & 0x0F);

		uint64_t dshot_expanded = nxp_flexio_dshot_expand_data(packet);

		dshot_info->data_seg1 = (uint32_t)(dshot_expanded & 0xFFFFFF);
		dshot_info->irq_data = (uint32_t)(dshot_expanded >> 24);
		dshot_info->state = DSHOT_START;

		if (dshot_info->bdshot) {
			flexio_base->TIMCTL[channel] = 0;
			FLEXIO_DisableShifterStatusInterrupts(flexio_base, 0xFF);

			nxp_flexio_dshot_output(dev, channel);

			FLEXIO_ClearTimerStatusFlags(flexio_base, 0xFF);
		}
	}
}

static int nxp_flexio_dshot_isr(void *user_data)
{
	const struct device *dev = (const struct device *)user_data;
	const struct nxp_flexio_dshot_config *config = dev->config;
	struct nxp_flexio_dshot_data *data = dev->data;
	FLEXIO_Type *flexio_base = (FLEXIO_Type *)(config->flexio_base);
	uint32_t flags = FLEXIO_GetShifterStatusFlags(flexio_base);
	uint32_t channel;

	for (channel = 0; flags && channel < DSHOT_TIMERS; channel++) {
		if (flags & (1 << channel)) {
			FLEXIO_DisableShifterStatusInterrupts(flexio_base, 1 << channel);

			if (config->channel->dshot_info[channel].state == DSHOT_START) {
				config->channel->dshot_info[channel].state = DSHOT_12BIT_FIFO;
				flexio_base->SHIFTBUF[channel] =
					config->channel->dshot_info[channel].irq_data;
			} else if (config->channel->dshot_info[channel].state == BDSHOT_RECEIVE) {
				config->channel->dshot_info[channel].state =
					BDSHOT_RECEIVE_COMPLETE;
				config->channel->dshot_info[channel].raw_response =
					flexio_base->SHIFTBUFBIS[channel];

				data->bdshot_recv_mask |= (1 << channel);

				if (data->bdshot_recv_mask == data->dshot_mask) {
					// Received telemetry on all channels
					// Schedule workqueue?
				}
			}
		}
	}

	flags = FLEXIO_GetTimerStatusFlags(flexio_base);

	for (channel = 0; flags; (channel = (channel + 1) % DSHOT_TIMERS)) {
		flags = FLEXIO_GetTimerStatusFlags(flexio_base);

		if (flags & (1 << channel)) {
			FLEXIO_ClearTimerStatusFlags(flexio_base, 1 << channel);

			if (config->channel->dshot_info[channel].state == DSHOT_12BIT_FIFO) {
				config->channel->dshot_info[channel].state =
					DSHOT_12BIT_TRANSFERRED;

			} else if (!config->channel->dshot_info[channel].bdshot &&
				   config->channel->dshot_info[channel].state ==
					   DSHOT_12BIT_TRANSFERRED) {
				config->channel->dshot_info[channel].state =
					DSHOT_TRANSMIT_COMPLETE;

			} else if (config->channel->dshot_info[channel].bdshot &&
				   config->channel->dshot_info[channel].state ==
					   DSHOT_12BIT_TRANSFERRED) {
				FLEXIO_DisableShifterStatusInterrupts(flexio_base, 1 << channel);
				config->channel->dshot_info[channel].state = BDSHOT_RECEIVE;

				/* Configure shifter and timer to receive data */
				nxp_flexio_bdshot_input(dev, channel);

				/* Enable shifter interrupt for receiving data */
				FLEXIO_EnableShifterStatusInterrupts(flexio_base, 1 << channel);
			}
		}
	}
	return 0;
}

#define _FLEXIO_DSHOT_GEN_CONFIG(n)                                                                \
	{                                                                                          \
		.pin_id = DT_PROP(n, pin_id),                                                      \
		.bdshot = false,                                                                   \
	},

#define FLEXIO_DSHOT_GEN_CONFIG(n)                                                                 \
	static struct nxp_flexio_dshot_channel_config flexio_dshot_##n##_init[] = {                \
		DT_INST_FOREACH_CHILD_STATUS_OKAY(n, _FLEXIO_DSHOT_GEN_CONFIG)};                   \
	static const struct nxp_flexio_dshot_channel flexio_dshot_##n##_info = {                   \
		.dshot_channel_count = ARRAY_SIZE(flexio_dshot_##n##_init),                        \
		.dshot_info = flexio_dshot_##n##_init,                                             \
	};

#define FLEXIO_DSHOT_FLEXIO_INDEX_INIT(n)                                                          \
	static uint8_t flexio_dshot_##n##_timer_index[ARRAY_SIZE(flexio_dshot_##n##_init)];        \
	static uint8_t flexio_dshot_##n##_shifter_index[ARRAY_SIZE(flexio_dshot_##n##_init)];

#define FLEXIO_DSHOT_CHILD_CONFIG(n)                                                               \
	static const struct nxp_flexio_child mcux_flexio_dshot_child_##n = {                       \
		.isr = nxp_flexio_dshot_isr,                                                       \
		.user_data = (void *)DEVICE_DT_INST_GET(n),                                        \
		.res = {.shifter_index = (uint8_t *)flexio_dshot_##n##_shifter_index,              \
			.shifter_count = ARRAY_SIZE(flexio_dshot_##n##_init),                      \
			.timer_index = (uint8_t *)flexio_dshot_##n##_timer_index,                  \
			.timer_count = ARRAY_SIZE(flexio_dshot_##n##_init)}};

#define FLEXIO_DSHOT_GEN_GET_CONFIG(n) .channel = &flexio_dshot_##n##_info,

#define NXP_FLEXIO_DSHOT_INIT(n)                                                                   \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	FLEXIO_DSHOT_GEN_CONFIG(n)                                                                 \
	FLEXIO_DSHOT_FLEXIO_INDEX_INIT(n)                                                          \
	FLEXIO_DSHOT_CHILD_CONFIG(n)                                                               \
	static const struct nxp_flexio_dshot_config nxp_flexio_dshot_config_##n = {                \
		.flexio_dev = DEVICE_DT_GET(DT_INST_PARENT(n)),                                    \
		.flexio_base = (FLEXIO_Type *)DT_REG_ADDR(DT_INST_PARENT(n)),                      \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                       \
		.clock_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(DT_INST_PARENT(n))),                     \
		.clock_subsys = (clock_control_subsys_t)DT_CLOCKS_CELL(DT_INST_PARENT(n), name),   \
		.child = &mcux_flexio_dshot_child_##n,                                             \
		FLEXIO_DSHOT_GEN_GET_CONFIG(n)};                                                   \
                                                                                                   \
	static struct nxp_flexio_dshot_data nxp_flexio_dshot_data_##n;                             \
	DEVICE_DT_INST_DEFINE(n, &nxp_flexio_dshot_init, NULL, &nxp_flexio_dshot_data_##n,         \
			      &nxp_flexio_dshot_config_##n, POST_KERNEL,                           \
			      CONFIG_NXP_FLEXIO_DSHOT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(NXP_FLEXIO_DSHOT_INIT)