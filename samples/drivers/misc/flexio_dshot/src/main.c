/*
 * Copyright (c) 2024 Peter van der Perk <peter.vanderperk@nxp.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

/**
 * @file Output dshot
 */

int main(void)
{
	const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(flexio0_dshot));
	int erpm;

	nxp_flexio_dshot_data_set(dev, 0, 0x0, false);

	while (1) {
		nxp_flexio_dshot_trigger(dev);
		k_sleep(K_MSEC(500));
		if (up_bdshot_get_erpm(dev, 0, &erpm)) {
			printf("ERPM %i\n", erpm);
		}
	}

	return 0;
}
