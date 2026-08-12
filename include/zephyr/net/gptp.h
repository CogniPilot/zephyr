/*
 * Copyright (c) 2017 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public functions for the Precision Time Protocol Stack.
 *
 */

#ifndef ZEPHYR_INCLUDE_NET_GPTP_H_
#define ZEPHYR_INCLUDE_NET_GPTP_H_

/**
 * @brief generic Precision Time Protocol (gPTP) support
 * @defgroup gptp gPTP support
 * @since 1.13
 * @version 0.1.0
 * @ingroup networking
 * @{
 */

#include <zephyr/net/net_core.h>
#include <zephyr/net/ptp_time.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @cond INTERNAL_HIDDEN */

#define GPTP_OFFSET_SCALED_LOG_VAR_UNKNOWN 0x436A

#define GPTP_PRIORITY1_NON_GM_CAPABLE      255
#define GPTP_PRIORITY1_GM_CAPABLE          248

#if defined(CONFIG_NET_GPTP_BMCA_PRIORITY2)
#define GPTP_PRIORITY2_DEFAULT             CONFIG_NET_GPTP_BMCA_PRIORITY2
#else
#define GPTP_PRIORITY2_DEFAULT             248
#endif

/** @endcond */

/**
 * @brief Scaled Nanoseconds.
 */
struct gptp_scaled_ns {
	/** High half. */
	int32_t high;

	/** Low half. */
	int64_t low;
} __packed;

/**
 * @brief UScaled Nanoseconds.
 */
struct gptp_uscaled_ns {
	/** High half. */
	uint32_t high;

	/** Low half. */
	uint64_t low;
} __packed;

/** @cond INTERNAL_HIDDEN */

#if defined(CONFIG_NEWLIB_LIBC)
#include <math.h>

#define GPTP_POW2(exp) pow(2, exp)
#else

static inline double gptp_pow2(int exp)
{
	double res;

	if (exp >= 0) {
		res = 1 << exp;
	} else {
		res = 1.0;

		while (exp++) {
			res /= 2;
		}
	}

	return res;
}

#define GPTP_POW2(exp) gptp_pow2(exp)
#endif

/* Pre-calculated constants */
/* 2^16 */
#define GPTP_POW2_16	65536.0
/* 2^41 */
#define GPTP_POW2_41	2199023255552.0

/* Message types. Event messages have BIT(3) set to 0, and general messages
 * have that bit set to 1. IEEE 802.1AS chapter 10.5.2.2.2
 */
#define GPTP_SYNC_MESSAGE                0x00
#define GPTP_DELAY_REQ_MESSAGE           0x01
#define GPTP_PATH_DELAY_REQ_MESSAGE      0x02
#define GPTP_PATH_DELAY_RESP_MESSAGE     0x03
#define GPTP_FOLLOWUP_MESSAGE            0x08
#define GPTP_DELAY_RESP_MESSAGE          0x09
#define GPTP_PATH_DELAY_FOLLOWUP_MESSAGE 0x0a
#define GPTP_ANNOUNCE_MESSAGE            0x0b
#define GPTP_SIGNALING_MESSAGE           0x0c
#define GPTP_MANAGEMENT_MESSAGE          0x0d

#define GPTP_IS_EVENT_MSG(msg_type)      (!((msg_type) & BIT(3)))

#define GPTP_CLOCK_ID_LEN                8

/** @endcond */

/**
 * @brief Port Identity.
 */
struct gptp_port_identity {
	/** Clock identity of the port. */
	uint8_t clk_id[GPTP_CLOCK_ID_LEN];

	/** Number of the port. */
	uint16_t port_number;
} __packed;

/** gPTP message flags */
struct gptp_flags {
	union {
		/** Byte access. */
		uint8_t octets[2];

		/** Whole field access. */
		uint16_t all;
	};
} __packed;

/** gPTP message header */
struct gptp_hdr {
	/** Type of the message. */
	uint8_t message_type:4;

	/** Transport specific, always 1. */
	uint8_t transport_specific:4;

	/** Version of the PTP, always 2. */
	uint8_t ptp_version:4;

	/** Reserved field. */
	uint8_t reserved0:4;

	/** Total length of the message from the header to the last TLV. */
	uint16_t message_length;

	/** Domain number, always 0. */
	uint8_t domain_number;

	/** Reserved field. */
	uint8_t reserved1;

	/** Message flags. */
	struct gptp_flags flags;

	/** Correction Field. The content depends of the message type. */
	int64_t correction_field;

	/** Reserved field. */
	uint32_t reserved2;

	/** Port Identity of the sender. */
	struct gptp_port_identity port_id;

	/** Sequence Id. */
	uint16_t sequence_id;

	/** Control value. Sync: 0, Follow-up: 2, Others: 5. */
	uint8_t control;

	/** Message Interval in Log2 for Sync and Announce messages. */
	int8_t log_msg_interval;
} __packed;

/** @cond INTERNAL_HIDDEN */

#define GPTP_GET_CURRENT_TIME_USCALED_NS(port, uscaled_ns_ptr)		\
	do {								\
		(uscaled_ns_ptr)->low =					\
			gptp_get_current_time_nanosecond(port) << 16;	\
		(uscaled_ns_ptr)->high = 0;				\
	} while (false)

/** @endcond */

/**
 * @typedef gptp_phase_dis_callback_t
 * @brief Define callback that is called after a phase discontinuity has been
 *        sent by the grandmaster.
 * @param gm_identity A pointer to first element of a ClockIdentity array.
 *        The size of the array is GPTP_CLOCK_ID_LEN.
 * @param time_base A pointer to the value of timeBaseIndicator of the current
 *        grandmaster.
 * @param last_gm_ph_change A pointer to the value of lastGmPhaseChange received
 *        from grandmaster.
 * @param last_gm_freq_change A pointer to the value of lastGmFreqChange
 *        received from the grandmaster.
 */
typedef void (*gptp_phase_dis_callback_t)(
	uint8_t *gm_identity,
	uint16_t *time_base,
	struct gptp_scaled_ns *last_gm_ph_change,
	double *last_gm_freq_change);

/**
 * @brief Phase discontinuity callback structure.
 *
 * Stores the phase discontinuity callback information. Caller must make sure
 * that the variable pointed by this is valid during the lifetime of
 * registration. Typically this means that the variable cannot be
 * allocated from stack.
 */
struct gptp_phase_dis_cb {
	/** Node information for the slist. */
	sys_snode_t node;

	/** Phase discontinuity callback. */
	gptp_phase_dis_callback_t cb;
};

/**
 * @brief ClockSourceTime.invoke function parameters
 *
 * Parameters passed by ClockSourceTime.invoke function.
 */
struct gptp_clk_src_time_invoke_params {
	/** Frequency change on the last Time Base Indicator Change. */
	double last_gm_freq_change;

	/** The time this function is invoked. */
	struct net_ptp_extended_time src_time;

	/** Phase change on the last Time Base Indicator Change. */
	struct gptp_scaled_ns last_gm_phase_change;

	/** Time Base - changed only if Phase or Frequency changes. */
	uint16_t time_base_indicator;
};

/** @cond INTERNAL_HIDDEN */

#if defined(CONFIG_NET_GPTP)
void net_gptp_init(void);
#else
static inline void net_gptp_init(void)
{
}
#endif /* CONFIG_NET_GPTP */

/** @endcond */

/**
 * @brief Register a phase discontinuity callback.
 *
 * @param phase_dis Caller specified handler for the callback.
 * @param cb Callback to register.
 */
void gptp_register_phase_dis_cb(struct gptp_phase_dis_cb *phase_dis,
				gptp_phase_dis_callback_t cb);

/**
 * @brief Unregister a phase discontinuity callback.
 *
 * @param phase_dis Caller specified handler for the callback.
 */
void gptp_unregister_phase_dis_cb(struct gptp_phase_dis_cb *phase_dis);

/**
 * @brief Call a phase discontinuity callback function.
 */
void gptp_call_phase_dis_cb(void);

/**
 * @brief Get gPTP time.
 *
 * @details With a grandmaster elsewhere in the domain, the time is
 * captured from the PTP hardware clock of the port in the time-receiver
 * role. On the acting grandmaster, where no port has that role, the
 * node's own PTP hardware clock is captured once a clockClass other than
 * the free-running default has been announced for it (a traceable source
 * configured through gptp_update_gm_quality()); a grandmaster still on the
 * default clockClass returns -EAGAIN.
 *
 * @param slave_time A pointer to structure where timestamp will be saved.
 * @param gm_present A pointer to a boolean where status of the
 *        presence of a grand master will be saved.
 *
 * @return Error code. 0 if no error.
 */
int gptp_event_capture(struct net_ptp_time *slave_time, bool *gm_present);

/**
 * @brief Utility function to print clock id to a user supplied buffer.
 *
 * @param clk_id Clock id
 * @param output Output buffer
 * @param output_len Output buffer len
 *
 * @return Pointer to output buffer
 */
char *gptp_sprint_clock_id(const uint8_t *clk_id, char *output,
			   size_t output_len);

/**
 * @typedef gptp_port_cb_t
 * @brief Callback used while iterating over gPTP ports
 *
 * @param port Port number
 * @param iface Pointer to network interface
 * @param user_data A valid pointer to user data or NULL
 */
typedef void (*gptp_port_cb_t)(int port, struct net_if *iface,
			       void *user_data);

/**
 * @brief Go through all the gPTP ports and call callback for each of them.
 *
 * @param cb User-supplied callback function to call
 * @param user_data User specified data
 */
void gptp_foreach_port(gptp_port_cb_t cb, void *user_data);

/**
 * @brief Get the gptp port number of the network interface.
 *
 * @param iface Network Interface acting as a gptp port.
 *
 * @return gptp port number if found, ENODEV otherwise.
 */
int gptp_get_port_number(struct net_if *iface);

/**
 * @brief Set the gptp port number to the network interface.
 *
 * @param iface Network Interface acting as a gptp port.
 * @param port gptp port number.
 *
 * @return 0 if set is successful, <0 otherwise.
 */
int gptp_set_port_number(struct net_if *iface, uint16_t port);

/**
 * @brief Get gPTP domain.
 * @details This contains all the configuration / status of the gPTP domain.
 *
 * @return Pointer to domain or NULL if not found.
 */
struct gptp_domain *gptp_get_domain(void);

/**
 * @brief This interface is used by the ClockSource entity to provide time to
 *        the ClockMaster entity of a time-aware system.
 *
 * @param arg Current state and parameters of the ClockSource entity.
 */
void gptp_clk_src_time_invoke(struct gptp_clk_src_time_invoke_params *arg);

/**
 * @brief Quality of the local clock as announced to the network.
 *
 * Describes how the local clock is disciplined. These values are carried
 * by the Announce messages sent by this time-aware system and are used by
 * the Best Master Clock selection Algorithm to elect the grandmaster.
 */
struct gptp_gm_quality {
	/** Class of the local clock, see IEEE 802.1AS chapter 8.6.2.2. */
	uint8_t clock_class;

	/** Accuracy of the local clock, see IEEE 802.1AS chapter 8.6.2.3. */
	uint8_t clock_accuracy;

	/** Source of time of the local clock, see IEEE 802.1AS chapter 8.6.2.7. */
	uint8_t time_source;
};

/**
 * @brief Time properties of the local clock as announced to the network.
 *
 * Carried by the Announce messages sent while this time-aware system is
 * the grandmaster, see IEEE 802.1AS chapter 10.3.8. An application
 * getting these values from an external time source uses them to keep
 * the announced properties truthful.
 */
struct gptp_time_properties {
	/** Offset between TAI and UTC, in seconds. */
	int16_t cur_utc_offset;

	/** The UTC offset is known to be correct. */
	bool cur_utc_offset_valid;

	/** The last minute of the current UTC day has 61 seconds. */
	bool leap61;

	/** The last minute of the current UTC day has 59 seconds. */
	bool leap59;

	/** The time is traceable to a primary reference. */
	bool time_traceable;

	/** The frequency is traceable to a primary reference. */
	bool freq_traceable;
};

/**
 * @brief Update the announced quality of the local clock.
 * @details This is used by an application disciplining the local clock from
 *          an external time source, so that the announced quality follows
 *          the state of that source. A typical use is to claim a primary
 *          reference clock class while the source is locked, and to fall
 *          back to a holdover class once it is lost.
 *
 *          The Best Master Clock selection Algorithm is run again with the
 *          new values, which are then carried by the next Announce message
 *          sent by this time-aware system.
 *
 * @param quality New quality of the local clock.
 */
void gptp_update_gm_quality(const struct gptp_gm_quality *quality);

/**
 * @brief Update the announced time properties of the local clock.
 * @details This is used by an application getting the UTC offset and the
 *          leap second indication from an external time source. The values
 *          are announced while this time-aware system is the grandmaster.
 *
 * @param prop New time properties of the local clock.
 */
void gptp_update_time_properties(const struct gptp_time_properties *prop);

/**
 * @brief Return pointer to gPTP packet header in network packet.
 *
 * @param pkt Network packet (received or sent)
 *
 * @return Pointer to gPTP header.
 */
struct gptp_hdr *gptp_get_hdr(struct net_pkt *pkt);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_NET_GPTP_H_ */
