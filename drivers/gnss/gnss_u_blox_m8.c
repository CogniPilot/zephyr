/*
 * Copyright (c) 2024 NXP
 * Copyright (c) 2025 Croxel Inc.
 * Copyright (c) 2025 CogniPilot Foundation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT u_blox_m8

#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/drivers/gnss/gnss_publish.h>

#include <zephyr/modem/ubx.h>
#include <zephyr/modem/backend/uart.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ubx_m8, CONFIG_GNSS_LOG_LEVEL);

struct ubx_m8_config {
	const struct device *bus;
	uint16_t fix_rate_ms;
	uint32_t initial_baudrate;
	const struct ubx_frame *ubx_cfg_rate_frame;
	const struct ubx_frame *ubx_cfg_port_frame;
};

struct ubx_m8_data {
	const struct device *dev;
	struct {
		struct modem_pipe *pipe;
		struct modem_backend_uart uart_backend;
		uint8_t receive_buf[1024];
		uint8_t transmit_buf[256];
	} backend;
	struct {
		struct modem_ubx inst;
		uint8_t receive_buf[1024];
	} ubx;
	struct {
		struct modem_ubx_script inst;
		uint8_t response_buf[512];
		struct k_sem lock;
	} script;
#if CONFIG_GNSS_SATELLITES
	struct gnss_satellite satellites[CONFIG_GNSS_U_BLOX_M8_SATELLITES_COUNT];
#endif
};

static void on_pvt_data(struct modem_ubx *ubx, const struct ubx_frame *frame, size_t len,
			void *user_data)
{
	if (len >= UBX_FRM_SZ(sizeof(struct ubx_nav_pvt))) {
		struct ubx_m8_data *data = CONTAINER_OF(ubx, struct ubx_m8_data, ubx.inst);
		const struct device *dev = data->dev;
		const struct ubx_nav_pvt *nav_pvt =
			(const struct ubx_nav_pvt *)frame->payload_and_checksum;

		if (nav_pvt->time.valid.date &&
		    nav_pvt->time.valid.time &&
		    nav_pvt->time.valid.time_fully_resolved) {

			enum gnss_fix_quality fix_quality = GNSS_FIX_QUALITY_INVALID;
			enum gnss_fix_status fix_status = GNSS_FIX_STATUS_NO_FIX;

			if (nav_pvt->flags.gnss_fix_ok &&
			    !nav_pvt->nav.flags3.invalid_lon_height_hmsl) {

				switch (nav_pvt->fix_type) {
				case UBX_NAV_FIX_TYPE_DR:
				case UBX_NAV_FIX_TYPE_GNSS_DR_COMBINED:
					fix_quality = GNSS_FIX_QUALITY_ESTIMATED;
					fix_status = GNSS_FIX_STATUS_ESTIMATED_FIX;
					break;
				case UBX_NAV_FIX_TYPE_2D:
				case UBX_NAV_FIX_TYPE_3D:
					fix_quality = GNSS_FIX_QUALITY_GNSS_SPS;
					fix_status = GNSS_FIX_STATUS_GNSS_FIX;
					break;
				default:
					break;
				}
			}

			struct gnss_data gnss_data = {
				.info = {
					.satellites_cnt = nav_pvt->nav.num_sv,
					.hdop = nav_pvt->nav.pdop * 10,
					.geoid_separation = (nav_pvt->nav.height -
							     nav_pvt->nav.hmsl),
					.fix_status = fix_status,
					.fix_quality = fix_quality,
				},
				.nav_data = {
					.latitude = (int64_t)nav_pvt->nav.latitude * 100,
					.longitude = (int64_t)nav_pvt->nav.longitude * 100,
					.bearing = (((nav_pvt->nav.head_motion < 0) ?
						     (nav_pvt->nav.head_motion + (360 * 100000)) :
						     (nav_pvt->nav.head_motion)) / 100),
					.speed = nav_pvt->nav.ground_speed,
					.altitude = nav_pvt->nav.hmsl,
				},
				.utc = {
					.hour = nav_pvt->time.hour,
					.minute = nav_pvt->time.minute,
					.millisecond = (nav_pvt->time.second * 1000) +
						       (nav_pvt->time.nano / 1000000),
					.month_day = nav_pvt->time.day,
					.month = nav_pvt->time.month,
					.century_year = (nav_pvt->time.year % 100),
				},
			};

			gnss_publish_data(dev, &gnss_data);
		}
	}
}

#if CONFIG_GNSS_SATELLITES
static void on_sat_data(struct modem_ubx *ubx, const struct ubx_frame *frame, size_t len,
			void *user_data)
{
	const struct ubx_nav_sat *ubx_sat = (const struct ubx_nav_sat *)frame->payload_and_checksum;
	int num_satellites = (len - UBX_FRM_SZ_WITHOUT_PAYLOAD - sizeof(struct ubx_nav_sat)) /
			     sizeof(struct ubx_nav_sat_info);

	if (num_satellites > 0 && (num_satellites == ubx_sat->num_sv)) {
		struct ubx_m8_data *data = CONTAINER_OF(ubx, struct ubx_m8_data, ubx.inst);
		const struct device *dev = data->dev;

		num_satellites = MIN(num_satellites, CONFIG_GNSS_U_BLOX_M8_SATELLITES_COUNT);

		for (size_t i = 0 ; i < num_satellites ; i++) {
			enum gnss_system gnss_system = 0;

			switch (ubx_sat->sat[i].gnss_id) {
			case UBX_GNSS_ID_GPS:
				gnss_system = GNSS_SYSTEM_GPS;
				break;
			case UBX_GNSS_ID_SBAS:
				gnss_system = GNSS_SYSTEM_SBAS;
				break;
			case UBX_GNSS_ID_GALILEO:
				gnss_system = GNSS_SYSTEM_GALILEO;
				break;
			case UBX_GNSS_ID_BEIDOU:
				gnss_system = GNSS_SYSTEM_BEIDOU;
				break;
			case UBX_GNSS_ID_QZSS:
				gnss_system = GNSS_SYSTEM_QZSS;
				break;
			case UBX_GNSS_ID_GLONASS:
				gnss_system = GNSS_SYSTEM_GLONASS;
				break;
			default:
				break;
			}

			struct gnss_satellite sat = {
				/** TODO: Determine how to determine PRN from UBX sat info.
				 * For now passing SV_ID.
				 */
				.prn = ubx_sat->sat[i].sv_id,
				.snr = ubx_sat->sat[i].cno,
				.elevation = ubx_sat->sat[i].elevation,
				.azimuth = ubx_sat->sat[i].azimuth,
				.system = gnss_system,
				.is_tracked = ubx_sat->sat[i].flags.sv_used,
			};

			data->satellites[i] = sat;
		}

		gnss_publish_satellites(dev, data->satellites, num_satellites);
	}
}
#endif

MODEM_UBX_MATCH_ARRAY_DEFINE(u_blox_m8_unsol_messages,
	MODEM_UBX_MATCH_DEFINE(UBX_CLASS_ID_NAV, UBX_MSG_ID_NAV_PVT, on_pvt_data),
#if CONFIG_GNSS_SATELLITES
	MODEM_UBX_MATCH_DEFINE(UBX_CLASS_ID_NAV, UBX_MSG_ID_NAV_SAT, on_sat_data),
#endif
);

static int ubx_m8_msg_get(const struct device *dev, const struct ubx_frame *req,
			  size_t len, void *rsp, size_t min_rsp_size)
{
	struct ubx_m8_data *data = dev->data;
	struct ubx_frame *rsp_frame = (struct ubx_frame *)data->script.inst.response.buf;
	int err;

	data->script.inst.timeout = K_SECONDS(3);
	data->script.inst.retry_count = 2;
	data->script.inst.match.filter.class = req->class;
	data->script.inst.match.filter.id = req->id;
	data->script.inst.request.buf = req;
	data->script.inst.request.len = len;

	err = modem_ubx_run_script(&data->ubx.inst, &data->script.inst);
	if (err != 0 || (data->script.inst.response.buf_len < UBX_FRM_SZ(min_rsp_size))) {
		return -EIO;
	}

	memcpy(rsp, rsp_frame->payload_and_checksum, min_rsp_size);

	return 0;
}

static int ubx_m8_msg_set(const struct device *dev, const struct ubx_frame *req,
			  size_t len, bool wait_for_ack)
{
	struct ubx_m8_data *data = dev->data;

	data->script.inst.timeout = K_SECONDS(3);
	data->script.inst.retry_count = wait_for_ack ? 2 : 0;
	data->script.inst.match.filter.class = wait_for_ack ? UBX_CLASS_ID_ACK : 0;
	data->script.inst.match.filter.id = UBX_MSG_ID_ACK;
	data->script.inst.request.buf = req;
	data->script.inst.request.len = len;

	return modem_ubx_run_script(&data->ubx.inst, &data->script.inst);
}

static int ubx_m8_init(const struct device *dev)
{
	int err = 0;
	struct ubx_m8_data *data = dev->data;
	const struct ubx_m8_config *cfg = dev->config;

	/** Need to store pointer to retrieve it from data, so it can be
	 * published alongside gnss data.
	 */
	data->dev = dev;

	{
		const struct modem_ubx_config ubx_config = {
			.user_data = data,
			.receive_buf = data->ubx.receive_buf,
			.receive_buf_size = sizeof(data->ubx.receive_buf),
			.unsol_matches = {
				.array = u_blox_m8_unsol_messages,
				.size = ARRAY_SIZE(u_blox_m8_unsol_messages),
			},
		};

		(void)modem_ubx_init(&data->ubx.inst, &ubx_config);

		const struct modem_backend_uart_config uart_backend_config = {
			.uart = cfg->bus,
			.receive_buf = data->backend.receive_buf,
			.receive_buf_size = sizeof(data->backend.receive_buf),
			.transmit_buf = data->backend.transmit_buf,
			.transmit_buf_size = sizeof(data->backend.transmit_buf),
		};

		data->backend.pipe = modem_backend_uart_init(&data->backend.uart_backend,
							     &uart_backend_config);
		err = modem_pipe_open(data->backend.pipe, K_SECONDS(1));
		if (err != 0) {
			LOG_ERR("Failed to open Modem pipe: %d", err);
			return err;
		}

		err = modem_ubx_attach(&data->ubx.inst, data->backend.pipe);
		if (err != 0) {
			LOG_ERR("Failed to attach UBX inst to modem pipe: %d", err);
			return err;
		}

		/** TODO: Lock around get/set messages to guard data integrity */
		(void)k_sem_init(&data->script.lock, 1, 1);

		data->script.inst.response.buf = data->script.response_buf;
		data->script.inst.response.buf_len = sizeof(data->script.response_buf);
	}

	/** TODO: Implement baud-rate detection/setting. */
	{
		struct uart_config uart_cfg;

		err = uart_config_get(cfg->bus, &uart_cfg);
		if (err < 0) {
			LOG_ERR("Failed to get UART config: %d", err);
			return err;
		}

		uint32_t desired_baudrate = uart_cfg.baudrate;
		uint32_t initial_baudrate = cfg->initial_baudrate;

		uart_cfg.baudrate = initial_baudrate;
		err = uart_configure(cfg->bus, &uart_cfg);
		if (err < 0) {
			LOG_ERR("Failed to configure UART: %d", err);
		}

		/** One per instance, hence why it's instantiated from device inst macro */
		const struct ubx_frame *prt_cfg_frame = cfg->ubx_cfg_port_frame;
		
		LOG_INF("PRT CFG FRAME - Size: %d, Payload size: %d, Desired speed: %d",
			UBX_FRM_SZ(prt_cfg_frame->payload_size),
			prt_cfg_frame->payload_size,
			desired_baudrate);

		(void)ubx_m8_msg_set(dev, prt_cfg_frame,
				     UBX_FRM_SZ(prt_cfg_frame->payload_size), false);

		uart_cfg.baudrate = desired_baudrate;

		err = uart_configure(cfg->bus, &uart_cfg);
		if (err < 0) {
			LOG_ERR("Failed to configure UART: %d", err);
		}

		const static struct ubx_frame version_get = UBX_FRAME_GET_INITIALIZER(
							UBX_CLASS_ID_MON,
							UBX_MSG_ID_MON_VER);
		struct ubx_mon_ver ver;

		err = ubx_m8_msg_get(dev, &version_get,
					UBX_FRM_SZ(version_get.payload_size),
					(void *)&ver, sizeof(ver));
		if (err != 0) {
			LOG_ERR("Failted to get Modem Version info: %d", err);
			return err;
		}
	}

	const static struct ubx_frame version_get = UBX_FRAME_GET_INITIALIZER(
						UBX_CLASS_ID_MON,
						UBX_MSG_ID_MON_VER);
	struct ubx_mon_ver ver;

	err = ubx_m8_msg_get(dev, &version_get,
				UBX_FRM_SZ(version_get.payload_size),
				(void *)&ver, sizeof(ver));
	if (err != 0) {
		LOG_ERR("Failed to get Modem Version info: %d", err);
		return err;
	}
	LOG_INF("SW Version: %s, HW Version: %s", ver.sw_ver, ver.hw_ver);

	const static struct ubx_frame stop_gnss = UBX_FRAME_CFG_RST_INITIALIZER(
							UBX_CFG_RST_HOT_START,
							UBX_CFG_RST_MODE_GNSS_STOP);

	err = ubx_m8_msg_set(dev, &stop_gnss, UBX_FRM_SZ(stop_gnss.payload_size), false);
	if (err != 0) {
		LOG_ERR("Failed to stop GNSS module: %d", err);
		return err;
	}
	k_sleep(K_MSEC(1000));

	const struct ubx_frame *fix_rate = cfg->ubx_cfg_rate_frame;

	err = ubx_m8_msg_set(dev, fix_rate, UBX_FRM_SZ(fix_rate->payload_size), true);
	if (err != 0) {
		LOG_ERR("Failed to set fix-rate: %d", err);
		return err;
	}

	/** Disabling  */
	const static struct ubx_frame disable_gga = UBX_FRAME_CFG_MSG_RATE_INITIALIZER(
							UBX_CLASS_ID_NMEA_STD,
							UBX_MSG_ID_NMEA_STD_GGA,
							0);

	err = ubx_m8_msg_set(dev, &disable_gga, UBX_FRM_SZ(disable_gga.payload_size), true);
	if (err != 0) {
		LOG_ERR("Failed to disable NMEA GGA message: %d", err);
		return err;
	}

	const static struct ubx_frame disable_rmc = UBX_FRAME_CFG_MSG_RATE_INITIALIZER(
							UBX_CLASS_ID_NMEA_STD,
							UBX_MSG_ID_NMEA_STD_RMC,
							0);
	err = ubx_m8_msg_set(dev, &disable_rmc, UBX_FRM_SZ(disable_rmc.payload_size), true);
	if (err != 0) {
		LOG_ERR("Failed to disable NMEA RMC message: %d", err);
		return err;
	}

	const static struct ubx_frame disable_gsv = UBX_FRAME_CFG_MSG_RATE_INITIALIZER(
							UBX_CLASS_ID_NMEA_STD,
							UBX_MSG_ID_NMEA_STD_GSV,
							0);
	err = ubx_m8_msg_set(dev, &disable_gsv, UBX_FRM_SZ(disable_gsv.payload_size), true);
	if (err != 0) {
		LOG_ERR("Failed to disable NMEA GSV message: %d", err);
		return err;
	}

	const static struct ubx_frame disable_dtm = UBX_FRAME_CFG_MSG_RATE_INITIALIZER(
							UBX_CLASS_ID_NMEA_STD,
							UBX_MSG_ID_NMEA_STD_DTM,
							0);
	err = ubx_m8_msg_set(dev, &disable_dtm, UBX_FRM_SZ(disable_dtm.payload_size), true);
	if (err != 0) {
		LOG_ERR("Failed to disable NMEA DTM message: %d", err);
		return err;
	}

	const static struct ubx_frame disable_gbs = UBX_FRAME_CFG_MSG_RATE_INITIALIZER(
							UBX_CLASS_ID_NMEA_STD,
							UBX_MSG_ID_NMEA_STD_GBS,
							0);
	err = ubx_m8_msg_set(dev, &disable_gbs, UBX_FRM_SZ(disable_gbs.payload_size), true);
	if (err != 0) {
		LOG_ERR("Failed to disable NMEA GBS message: %d", err);
		return err;
	}

	/** Disable the following messages: DTM GBS GLL GNS GRS gSA GST VLW VTG ZDA */
	const static struct ubx_frame disable_gll = UBX_FRAME_CFG_MSG_RATE_INITIALIZER(
							UBX_CLASS_ID_NMEA_STD,
							UBX_MSG_ID_NMEA_STD_GLL,
							0);
	err = ubx_m8_msg_set(dev, &disable_gll, UBX_FRM_SZ(disable_gll.payload_size), true);
	if (err != 0) {
		LOG_ERR("Failed to disable NMEA GLL message: %d", err);
		return err;
	}

	const static struct ubx_frame disable_gns = UBX_FRAME_CFG_MSG_RATE_INITIALIZER(
							UBX_CLASS_ID_NMEA_STD,
							UBX_MSG_ID_NMEA_STD_GNS,
							0);
	err = ubx_m8_msg_set(dev, &disable_gns, UBX_FRM_SZ(disable_gns.payload_size), true);
	if (err != 0) {
		LOG_ERR("Failed to disable NMEA GNS message: %d", err);
		return err;
	}

	const static struct ubx_frame disable_grs = UBX_FRAME_CFG_MSG_RATE_INITIALIZER(
							UBX_CLASS_ID_NMEA_STD,
							UBX_MSG_ID_NMEA_STD_GRS,
							0);
	err = ubx_m8_msg_set(dev, &disable_grs, UBX_FRM_SZ(disable_grs.payload_size), true);
	if (err != 0) {
		LOG_ERR("Failed to disable NMEA GRS message: %d", err);
		return err;
	}

	const static struct ubx_frame disable_gsa = UBX_FRAME_CFG_MSG_RATE_INITIALIZER(
							UBX_CLASS_ID_NMEA_STD,
							UBX_MSG_ID_NMEA_STD_GSA,
							0);
	err = ubx_m8_msg_set(dev, &disable_gsa, UBX_FRM_SZ(disable_gsa.payload_size), true);
	if (err != 0) {
		LOG_ERR("Failed to disable NMEA GSA message: %d", err);
		return err;
	}

	const static struct ubx_frame disable_gst = UBX_FRAME_CFG_MSG_RATE_INITIALIZER(
							UBX_CLASS_ID_NMEA_STD,
							UBX_MSG_ID_NMEA_STD_GST,
							0);
	err = ubx_m8_msg_set(dev, &disable_gst, UBX_FRM_SZ(disable_gst.payload_size), true);
	if (err != 0) {
		LOG_ERR("Failed to disable NMEA GST message: %d", err);
		return err;
	}

	const static struct ubx_frame disable_vlw = UBX_FRAME_CFG_MSG_RATE_INITIALIZER(
							UBX_CLASS_ID_NMEA_STD,
							UBX_MSG_ID_NMEA_STD_VLW,
							0);
	err = ubx_m8_msg_set(dev, &disable_vlw, UBX_FRM_SZ(disable_vlw.payload_size), true);
	if (err != 0) {
		LOG_ERR("Failed to disable NMEA VLW message: %d", err);
		return err;
	}

	const static struct ubx_frame disable_vtg = UBX_FRAME_CFG_MSG_RATE_INITIALIZER(
							UBX_CLASS_ID_NMEA_STD,
							UBX_MSG_ID_NMEA_STD_VTG,
							0);
	err = ubx_m8_msg_set(dev, &disable_vtg, UBX_FRM_SZ(disable_vtg.payload_size), true);
	if (err != 0) {
		LOG_ERR("Failed to disable NMEA VTG message: %d", err);
		return err;
	}

	const static struct ubx_frame disable_zda = UBX_FRAME_CFG_MSG_RATE_INITIALIZER(
							UBX_CLASS_ID_NMEA_STD,
							UBX_MSG_ID_NMEA_STD_ZDA,
							0);
	err = ubx_m8_msg_set(dev, &disable_zda, UBX_FRM_SZ(disable_zda.payload_size), true);
	if (err != 0) {
		LOG_ERR("Failed to disable NMEA ZDA message: %d", err);
		return err;
	}

	const static struct ubx_frame enable_nav = UBX_FRAME_CFG_MSG_RATE_INITIALIZER(
							UBX_CLASS_ID_NAV,
							UBX_MSG_ID_NAV_PVT,
							1);
	err = ubx_m8_msg_set(dev, &enable_nav, UBX_FRM_SZ(enable_nav.payload_size), true);
	if (err != 0) {
		LOG_ERR("Failed to enable UBX NAV message: %d", err);
		return err;
	}

#if CONFIG_GNSS_SATELLITES
	const static struct ubx_frame enable_sat = UBX_FRAME_CFG_MSG_RATE_INITIALIZER(
							UBX_CLASS_ID_NAV,
							UBX_MSG_ID_NAV_SAT,
							1);
	err = ubx_m8_msg_set(dev, &enable_sat, UBX_FRM_SZ(enable_sat.payload_size), true);
	if (err != 0) {
		LOG_ERR("Failed to enable UBX SAT message: %d", err);
		return err;
	}
#endif

	const static struct ubx_frame start_gnss = UBX_FRAME_CFG_RST_INITIALIZER(
							UBX_CFG_RST_HOT_START,
							UBX_CFG_RST_MODE_GNSS_START);

	err = ubx_m8_msg_set(dev, &start_gnss, UBX_FRM_SZ(start_gnss.payload_size), false);
	if (err != 0) {
		LOG_ERR("Failed to start GNSS module: %d", err);
		return err;
	}

	/** TODO: Add unsolicited messages to get GNSS navigation messages */

	return 0;
}

static DEVICE_API(gnss, gnss_api) = {};

#define UBX_M8(inst)										   \
												   \
	BUILD_ASSERT(										   \
		(DT_PROP(DT_INST_BUS(inst), current_speed) == 9600) ||				   \
		(DT_PROP(DT_INST_BUS(inst), current_speed) == 19200) ||				   \
		(DT_PROP(DT_INST_BUS(inst), current_speed) == 38400) ||				   \
		(DT_PROP(DT_INST_BUS(inst), current_speed) == 57600) ||				   \
		(DT_PROP(DT_INST_BUS(inst), current_speed) == 115200) ||			   \
		(DT_PROP(DT_INST_BUS(inst), current_speed) == 230400) ||			   \
		(DT_PROP(DT_INST_BUS(inst), current_speed) == 460800),				   \
		"Invalid current-speed. Please set the UART current-speed to a baudrate "	   \
		"compatible with the modem.");							   \
												   \
	BUILD_ASSERT((DT_INST_PROP(inst, fix_rate) >= 50) &&					   \
		     (DT_INST_PROP(inst, fix_rate) < 65536),					   \
		     "Invalid fix-rate. Please set it higher than 50-ms"			   \
		     " and must fit in 16-bits.");						   \
												   \
	static struct ubx_frame ubx_m8_cfg_rate_##inst = UBX_FRAME_CFG_RATE_INITIALIZER(	   \
								DT_INST_PROP(inst, fix_rate), 1,   \
								UBX_CFG_RATE_TIME_REF_GPS);	   \
												   \
	static struct ubx_frame ubx_m8_prt_cfg_##inst = UBX_FRAME_CFG_PRT_INITIALIZER(		   \
								UBX_CFG_PORT_ID_UART,		   \
								DT_PROP(DT_INST_BUS(inst),	   \
									current_speed),		   \
								UBX_CFG_PRT_PORT_MODE_CHAR_LEN_8,  \
								UBX_CFG_PRT_PORT_MODE_PARITY_NONE, \
								UBX_CFG_PRT_PORT_MODE_STOP_BITS_1);\
												   \
	static const struct ubx_m8_config ubx_m8_cfg_##inst = {					   \
		.bus = DEVICE_DT_GET(DT_INST_BUS(inst)),					   \
		.initial_baudrate = DT_INST_PROP(inst, initial_baudrate),			   \
		.fix_rate_ms = DT_INST_PROP(inst, fix_rate),					   \
		.ubx_cfg_port_frame = &ubx_m8_prt_cfg_##inst,					   \
		.ubx_cfg_rate_frame = &ubx_m8_cfg_rate_##inst,					   \
	};											   \
												   \
	static struct ubx_m8_data ubx_m8_data_##inst = {					   \
	};											   \
												   \
	DEVICE_DT_INST_DEFINE(inst,								   \
			      ubx_m8_init,							   \
			      NULL,								   \
			      &ubx_m8_data_##inst,						   \
			      &ubx_m8_cfg_##inst,						   \
			      POST_KERNEL,							   \
			      CONFIG_GNSS_INIT_PRIORITY,					   \
			      &gnss_api);

DT_INST_FOREACH_STATUS_OKAY(UBX_M8)
