/*
 * Copyright (c) 2024 NXP
 * Copyright (c) 2025 Croxel Inc.
 * Copyright (c) 2025 CogniPilot Foundation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/modem/ubx.h>
#include <zephyr/sys/check.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(modem_ubx, CONFIG_MODEM_MODULES_LOG_LEVEL);

int modem_ubx_run_script(struct modem_ubx *ubx, struct modem_ubx_script *script)
{
	int ret;
	bool wait_for_rsp = script->match.filter.class != 0;

	ret = k_sem_take(&ubx->script_running_sem, script->timeout);
	if (ret != 0) {
		return -EBUSY;
	}

	ubx->script = script;
	k_sem_reset(&ubx->script_stopped_sem);

	int tries = ubx->script->retry_count + 1;
	int32_t ms_per_attempt = (uint64_t)k_ticks_to_ms_floor64(script->timeout.ticks) / tries;

	do {
		ret = modem_pipe_transmit(ubx->pipe,
					  (const uint8_t *)ubx->script->request.buf,
					  ubx->script->request.len);

		if (wait_for_rsp) {
			ret = k_sem_take(&ubx->script_stopped_sem, K_MSEC(ms_per_attempt));
		}
	} while ((--tries > 0) && (ret == -EAGAIN));

	k_sem_give(&ubx->script_running_sem);

	return (ret > 0) ? 0 : ret;
}

static inline uint16_t calc_checksum(const struct ubx_frame *frame, size_t len)
{
	uint8_t ck_a = 0;
	uint8_t ck_b = 0;
	const uint8_t *data = (const uint8_t *)frame;

	for (int i = UBX_FRM_MSG_CLASS_IDX ; i < (UBX_FRM_SZ(frame->payload_size) - 2) ; i++) {
		ck_a = ck_a + data[i];
		ck_b = ck_b + ck_a;
	}

	return ((ck_a & 0xFF) | ((ck_b & 0xFF) << 8));
}

static inline bool process_incoming_data(const uint8_t *data,
					 size_t len,
					 const struct ubx_frame **frame_start,
					 size_t *frame_len,
					 size_t *iterator)
{
	for(int i  = (*iterator) ; i < ((int)len - UBX_FRM_SZ_WITHOUT_PAYLOAD) ; i++) {
		if ((data[i] == UBX_PREAMBLE_SYNC_CHAR_1) &&
		    (data[i +1] == UBX_PREAMBLE_SYNC_CHAR_2)) {

			const struct ubx_frame *frame = (const struct ubx_frame *)&data[i];
			size_t frame_max_len = len - i;

			/* Valid length filtering */
			if (UBX_FRM_SZ(frame->payload_size) > frame_max_len) {
				continue;
			}

			/* Valid checksum filtering */
			uint16_t valid_checksum = calc_checksum(frame,
								UBX_FRM_SZ(frame->payload_size));
			uint16_t ck_a = frame->payload_and_checksum[frame->payload_size];
			uint16_t ck_b = frame->payload_and_checksum[frame->payload_size + 1];
			uint16_t actual_checksum = ck_a | (ck_b << 8);
			if (valid_checksum != actual_checksum) {
				continue;
			}

			*frame_start = frame;
			*frame_len = UBX_FRM_SZ(frame->payload_size);

			*iterator = i + 1;
			return true;
		}
	}

	return false;
}

static inline bool matches_filter(const struct ubx_frame *frame,
				  const struct ubx_frame_match *filter)
{
	if ((frame->class == filter->class) &&
	    (frame->id == filter->id) &&
	    ((filter->payload.len == 0) ||
	     ((frame->payload_size == filter->payload.len) &&
	      (0 == memcmp(frame->payload_and_checksum,
			   filter->payload.buf,
			   filter->payload.len))))) {
		return true;
	} else {
		return false;
	}
}

static void modem_ubx_process_handler(struct k_work *item)
{
	struct modem_ubx *ubx = CONTAINER_OF(item, struct modem_ubx, process_work);
	int ret;

	ret = modem_pipe_receive(ubx->pipe, ubx->receive_buf, ubx->receive_buf_size);

	const uint8_t *received_data = ubx->receive_buf;
	size_t length = ret > 0 ? ret : 0;
	const struct ubx_frame *frame = NULL;
	size_t frame_len = 0;
	size_t iterator = 0;
	bool more_frames_to_process;

	do {
		more_frames_to_process = process_incoming_data(received_data, length,
							       &frame, &frame_len,
							       &iterator);
		if (frame_len > 0) {
			/** Serve script first */
			if (matches_filter(frame, &ubx->script->match.filter)) {
				memcpy(ubx->script->response.buf, frame, frame_len);
				ubx->script->response.received_len = frame_len;

				k_sem_give(&ubx->script_stopped_sem);
			}
			/** Check for unsolicited matches */
			for (size_t i = 0 ; i < ubx->unsol_matches.size ; i++) {
				if (ubx->unsol_matches.array[i].handler &&
				    matches_filter(frame, &ubx->unsol_matches.array[i].filter)) {
					ubx->unsol_matches.array[i].handler(ubx, frame,
									    ubx->user_data);
				}
			}
		}
	} while (more_frames_to_process);
}

static void modem_ubx_pipe_callback(struct modem_pipe *pipe,
				    enum modem_pipe_event event,
				    void *user_data)
{
	struct modem_ubx *ubx = (struct modem_ubx *)user_data;

	if (event == MODEM_PIPE_EVENT_RECEIVE_READY) {
		k_work_submit(&ubx->process_work);
	}
}

int modem_ubx_attach(struct modem_ubx *ubx, struct modem_pipe *pipe)
{
	if (atomic_test_and_set_bit(&ubx->attached, 0) == true) {
		return 0;
	}

	ubx->pipe = pipe;
	modem_pipe_attach(ubx->pipe, modem_ubx_pipe_callback, ubx);
	k_sem_give(&ubx->script_running_sem);

	return 0;
}

void modem_ubx_release(struct modem_ubx *ubx)
{
	struct k_work_sync sync;

	if (atomic_test_and_clear_bit(&ubx->attached, 0) == false) {
		return;
	}

	modem_pipe_release(ubx->pipe);
	k_work_cancel_sync(&ubx->process_work, &sync);
	k_sem_reset(&ubx->script_stopped_sem);
	k_sem_reset(&ubx->script_running_sem);
	ubx->pipe = NULL;
}

int modem_ubx_init(struct modem_ubx *ubx, const struct modem_ubx_config *config)
{
	__ASSERT_NO_MSG(ubx != NULL);
	__ASSERT_NO_MSG(config != NULL);
	__ASSERT_NO_MSG(config->receive_buf != NULL);
	__ASSERT_NO_MSG(config->receive_buf_size > 0);

	memset(ubx, 0x00, sizeof(*ubx));
	ubx->user_data = config->user_data;

	ubx->receive_buf = config->receive_buf;
	ubx->receive_buf_size = config->receive_buf_size;

	ubx->pipe = NULL;

	ubx->unsol_matches.array = config->unsol_matches.array;
	ubx->unsol_matches.size = config->unsol_matches.size;

	k_work_init(&ubx->process_work, modem_ubx_process_handler);
	k_sem_init(&ubx->script_stopped_sem, 0, 1);
	k_sem_init(&ubx->script_running_sem, 1, 1);

	return 0;
}
