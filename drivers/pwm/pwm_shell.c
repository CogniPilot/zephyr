/*
 * Copyright (c) 2019 Vestas Wind Systems A/S
 * Copyright (c) 2023 Rudis Laboratories
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief PWM shell commands.
 */

#include <zephyr/shell/shell.h>
#include <zephyr/drivers/pwm.h>
#include <stdlib.h>
#include <stdio.h>

struct iter_entry {
    char* name;
    struct pwm_dt_spec spec;
};

#define PWM_SHELL_NODE DT_NODE_EXISTS(DT_NODELABEL(pwm_shell))

#if PWM_SHELL_NODE
#define PWM_ENTRY(node_id) { .name = DT_NODE_FULL_NAME(node_id), .spec = PWM_DT_SPEC_GET(node_id)},
#endif

struct iter_entry pwm_table[] = {
#if PWM_SHELL_NODE
    DT_FOREACH_CHILD(DT_NODELABEL(pwm_shell), PWM_ENTRY)
#endif
};

struct args_index {
	uint8_t device;
	uint8_t channel;
	uint8_t period;
	uint8_t pulse;
	uint8_t flags;
};

static const struct args_index args_indx = {
	.device = 1,
	.channel = 2,
	.period = 3,
	.pulse = 4,
	.flags = 5,
};
#if PWM_SHELL_NODE
struct args_alias_index {
	uint8_t alias_name;
	uint8_t pulse;
};

static const struct args_alias_index args_alias_indx = {
	.alias_name = 1,
	.pulse = 2,
};
#endif

static int cmd_cycles(const struct shell *sh, size_t argc, char **argv)
{
	pwm_flags_t flags = 0;
	const struct device *dev;
	uint32_t period;
	uint32_t pulse;
	uint32_t channel;
	int err;

	dev = device_get_binding(argv[args_indx.device]);
	if (!dev) {
		shell_error(sh, "PWM device not found");
		return -EINVAL;
	}

	channel = strtoul(argv[args_indx.channel], NULL, 0);
	period = strtoul(argv[args_indx.period], NULL, 0);
	pulse = strtoul(argv[args_indx.pulse], NULL, 0);

	if (argc == (args_indx.flags + 1)) {
		flags = strtoul(argv[args_indx.flags], NULL, 0);
	}

	err = pwm_set_cycles(dev, channel, period, pulse, flags);
	if (err) {
		shell_error(sh, "failed to setup PWM (err %d)",
			    err);
		return err;
	}

	return 0;
}

static int cmd_usec(const struct shell *sh, size_t argc, char **argv)
{
	pwm_flags_t flags = 0;
	const struct device *dev;
	uint32_t period;
	uint32_t pulse;
	uint32_t channel;
	int err;

	dev = device_get_binding(argv[args_indx.device]);
	if (!dev) {
		shell_error(sh, "PWM device not found");
		return -EINVAL;
	}

	channel = strtoul(argv[args_indx.channel], NULL, 0);
	period = strtoul(argv[args_indx.period], NULL, 0);
	pulse = strtoul(argv[args_indx.pulse], NULL, 0);

	if (argc == (args_indx.flags + 1)) {
		flags = strtoul(argv[args_indx.flags], NULL, 0);
	}

	err = pwm_set(dev, channel, PWM_USEC(period), PWM_USEC(pulse), flags);
	if (err) {
		shell_error(sh, "failed to setup PWM (err %d)", err);
		return err;
	}

	return 0;
}

static int cmd_nsec(const struct shell *sh, size_t argc, char **argv)
{
	pwm_flags_t flags = 0;
	const struct device *dev;
	uint32_t period;
	uint32_t pulse;
	uint32_t channel;
	int err;

	dev = device_get_binding(argv[args_indx.device]);
	if (!dev) {
		shell_error(sh, "PWM device not found");
		return -EINVAL;
	}

	channel = strtoul(argv[args_indx.channel], NULL, 0);
	period = strtoul(argv[args_indx.period], NULL, 0);
	pulse = strtoul(argv[args_indx.pulse], NULL, 0);

	if (argc == (args_indx.flags + 1)) {
		flags = strtoul(argv[args_indx.flags], NULL, 0);
	}

	err = pwm_set(dev, channel, period, pulse, flags);
	if (err) {
		shell_error(sh, "failed to setup PWM (err %d)", err);
		return err;
	}

	return 0;
}


static int cmd_list_aliases(const struct shell *sh)
{	
#if PWM_SHELL_NODE
	if (sizeof(pwm_table) > 0) {
		shell_print(sh, "******\033[31mPWM ALIASES\033[0m******");
		for(int i = 0; i < sizeof(pwm_table)/sizeof(pwm_table[0]) ; i++)
		{	
			shell_print(sh,
				"\033[31m%s\033[0m: device->\033[33m%s\033[0m, "
				"channel->\033[33m%d\033[0m, period->\033[33m%.3f\033[0mus",
				pwm_table[i].name, pwm_table[i].spec.dev->name,
				pwm_table[i].spec.channel, (double)(pwm_table[i].spec.period/1e3));
		}
		return 0;
	}
#endif
	shell_error(sh, "pwm_shell node has no PWM aliases listed in the dts.");

	return 0;
}

static int cmd_usec_alias(const struct shell *sh, size_t argc, char **argv )
{
#if PWM_SHELL_NODE
	if (sizeof(pwm_table) > 0) {
		char* alias;
		uint32_t pulse;
		int err;
		alias = argv[args_alias_indx.alias_name];
		pulse = strtoul(argv[args_alias_indx.pulse], NULL, 0);
		for(int i = 0; i < sizeof(pwm_table)/sizeof(pwm_table[0]) ; i++)
		{	
			if (!strcmp(alias, pwm_table[i].name)) {
				err = pwm_set_pulse_dt(&pwm_table[i].spec, PWM_USEC(pulse));
				if (err < 0) {
					shell_error(sh, "Error %d: %s failed to set pulse width %d",
						err, alias, pulse);
					return 0;
				}
				return 0;
			}
			
		}
		shell_error(sh, "Error: no alias matched %s, unable to set pulse %d",
			alias, pulse);
		return 0;
	}
#endif
	shell_error(sh, "pwm_shell node has no PWM aliases listed in the dts.");

	return 0;
}

static int cmd_nsec_alias(const struct shell *sh, size_t argc, char **argv )
{
#if PWM_SHELL_NODE
	if (sizeof(pwm_table) > 0) {
		char* alias;
		uint32_t pulse;
		int err;
		alias = argv[args_alias_indx.alias_name];
		pulse = strtoul(argv[args_alias_indx.pulse], NULL, 0);
		for(int i = 0; i < sizeof(pwm_table)/sizeof(pwm_table[0]) ; i++)
		{	
			if (!strcmp(alias, pwm_table[i].name)) {
				err = pwm_set_pulse_dt(&pwm_table[i].spec, pulse);
				if (err < 0) {
					shell_error(sh, "Error %d: %s failed to set pulse width %d", err, alias, pulse);
					return 0;
				}
				return 0;
			}
			
		}
		shell_error(sh, "Error: no alias matched %s, unable to set pulse %d", alias, pulse);
		return 0;
	} 
#endif
	shell_error(sh, "pwm_shell node has no PWM aliases listed in the dts.");

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(pwm_cmds,
	SHELL_CMD_ARG(cycles, NULL, "<device> <channel> <period in cycles> "
		      "<pulse width in cycles> [flags]", cmd_cycles, 5, 1),
	SHELL_CMD_ARG(usec, NULL, "<device> <channel> <period in usec> "
		      "<pulse width in usec> [flags]", cmd_usec, 5, 1),
	SHELL_CMD_ARG(nsec, NULL, "<device> <channel> <period in nsec> "
		      "<pulse width in nsec> [flags]", cmd_nsec, 5, 1),
	SHELL_CMD_ARG(list_aliases, NULL, "list_aliases", cmd_list_aliases, 1, 0),
	SHELL_CMD_ARG(usec_alias, NULL, "<device> <pulse width in usec> ", 
		      cmd_usec_alias, 3, 0),
	SHELL_CMD_ARG(nsec_alias, NULL, "<device> <pulse width in nsec> ", 
		      cmd_nsec_alias, 3, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(pwm, &pwm_cmds, "PWM shell commands", NULL);
