/*
 * Copyright (c) 2025 CogniPilot Foundation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_GNSS_U_BLOX_F9P_H_
#define ZEPHYR_INCLUDE_DRIVERS_GNSS_U_BLOX_F9P_H_

#include <zephyr/device.h>
#include <zephyr/modem/ubx/protocol.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Get the latest UBX-TIM-TP time pulse information.
 *
 * TIM-TP describes the GNSS time of the NEXT TIMEPULSE edge, including
 * the picosecond quantization error of that edge. Pair it with a
 * hardware timestamp of the pulse to discipline a local clock.
 *
 * @param[in] dev		F9P GNSS device instance.
 * @param[out] tim_tp		Latest TIM-TP payload.
 * @param[out] uptime_ticks	Kernel uptime tick count at message reception.
 *				May be NULL.
 * @param[out] seq		Reception sequence counter (increments per
 *				TIM-TP message). May be NULL.
 *
 * @return 0 on success, -EAGAIN if no TIM-TP message has been received yet.
 */
int u_blox_f9p_timepulse_get(const struct device *dev, struct ubx_tim_tp *tim_tp,
			     int64_t *uptime_ticks, uint32_t *seq);

/**
 * @brief Get the latest UBX-TIM-TM2 time mark information.
 *
 * TIM-TM2 reports the GNSS time of the last edge observed on an EXTINT
 * pin. It is emitted at the measurement rate, and only for epochs in
 * which an edge was detected, so consecutive reads may return the same
 * time mark. Compare the sequence counter to detect a fresh report.
 *
 * @param[in] dev		F9P GNSS device instance.
 * @param[out] tm2		Latest TIM-TM2 payload.
 * @param[out] uptime_ticks	Kernel uptime tick count at message reception.
 *				May be NULL.
 * @param[out] seq		Reception sequence counter (increments per
 *				TIM-TM2 message). May be NULL.
 *
 * @return 0 on success, -EAGAIN if no TIM-TM2 message has been received yet.
 */
int u_blox_f9p_timemark_get(const struct device *dev, struct ubx_tim_tm2 *tm2,
			    int64_t *uptime_ticks, uint32_t *seq);

/**
 * @brief Get the latest UBX-NAV-TIMELS leap second information.
 *
 * @param[in] dev	F9P GNSS device instance.
 * @param[out] timels	Latest NAV-TIMELS payload.
 *
 * @return 0 on success, -EAGAIN if no NAV-TIMELS message has been received yet.
 */
int u_blox_f9p_leap_get(const struct device *dev, struct ubx_nav_timels *timels);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_GNSS_U_BLOX_F9P_H_ */
