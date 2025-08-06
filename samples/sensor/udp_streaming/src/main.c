#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/net/socket.h>

SENSOR_DT_STREAM_IODEV(sensor_iodev_0, DT_NODELABEL(sensor_0),
		       {SENSOR_TRIG_FIFO_WATERMARK, SENSOR_STREAM_DATA_INCLUDE});
SENSOR_DT_STREAM_IODEV(sensor_iodev_1, DT_NODELABEL(sensor_1),
		       {SENSOR_TRIG_FIFO_WATERMARK, SENSOR_STREAM_DATA_INCLUDE});
RTIO_DEFINE_WITH_MEMPOOL(sensor_ctx_1, 2, 64, 128, 128, sizeof(void *));
RTIO_DEFINE_WITH_MEMPOOL(sensor_ctx_0, 2, 64, 128, 128, sizeof(void *));

struct recover_dev {
	bool recover;
	struct rtio_iodev *iodev;
} recover_devs[] = {
	{.iodev = &sensor_iodev_0},
	{.iodev = &sensor_iodev_1},
};

static inline bool *get_recovery_bool(struct rtio_iodev *iodev)
{
	for (size_t i = 0 ; i < ARRAY_SIZE(recover_devs) ; i++) {
		if (iodev == recover_devs[i].iodev) {
			return &recover_devs[i].recover;
		}
	}
	return NULL;
}

#define PEER_PORT 33105

struct sample_data {
	const char *proto;

	struct {
		int sock;
		uint32_t expecting;
		uint32_t counter;
		uint32_t mtu;
		struct udp_control *ctrl;
	} udp;

	struct {
		int sock;
		uint32_t expecting;
		uint32_t received;
		uint32_t counter;
	} tcp;
};

struct configs {
	struct sample_data ipv4;
	struct sample_data ipv6;
} conf;

static int start_udp_proto(struct sample_data *data, sa_family_t family,
			   struct sockaddr *addr, socklen_t addrlen)
{
	data->udp.sock = socket(family, SOCK_DGRAM, IPPROTO_UDP);
	if (data->udp.sock < 0) {
		return -EINVAL;
	}

	return connect(data->udp.sock, addr, addrlen);
}

int start_udp(void)
{
	struct sockaddr_in addr4;

	addr4.sin_family = AF_INET;
	addr4.sin_port = htons(PEER_PORT);
	inet_pton(AF_INET, CONFIG_NET_CONFIG_PEER_IPV4_ADDR, &addr4.sin_addr);

	return start_udp_proto(&conf.ipv4, AF_INET, (struct sockaddr *)&addr4, sizeof(addr4));
}

static int setup_stream(struct rtio *ctx, struct rtio_iodev *iodev)
{
	uint64_t period_ticks = (uint64_t)CONFIG_SYS_CLOCK_TICKS_PER_SEC * 625 / 1000 / 1000;
	struct sensor_value ticks_per_event = {
		.val1 = period_ticks,
	};
	struct sensor_read_config *read_config =
		(struct sensor_read_config *)iodev->data;
	int err;

	printk("setting up stream...\n");

	err = sensor_attr_set(read_config->sensor, SENSOR_CHAN_ALL,
				SENSOR_ATTR_BATCH_DURATION, &ticks_per_event);
	if (err != 0) {
		printk("Failed to set batch-duration attribute: %d\n", err);
		return err;
	}

	err = sensor_stream(iodev, ctx, (void *)iodev, NULL);
	if (err != 0) {
		printk("Failed to start sensor stream: %d\n", err);
		return err;
	}

	return 0;
}

static inline void avg_3_axis_q31(const struct sensor_three_axis_data *data, size_t count,
				  struct sensor_three_axis_data *out)
{
	int64_t avg[3] = {0};

	for (size_t i = 0 ; i < count ; i++) {
		avg[0] += data->readings[i].x;
		avg[1] += data->readings[i].y;
		avg[2] += data->readings[i].z;
	}
	avg[0] /= count;
	avg[1] /= count;
	avg[2] /= count;

	out->header.base_timestamp_ns = data->header.base_timestamp_ns;
	out->header.reading_count = 1;
	out->shift = data->shift;
	out->readings[0].x = avg[0];
	out->readings[0].y = avg[1];
	out->readings[0].z = avg[2];
}

static void process_events(int result, uint8_t *buf, uint32_t len, void *userdata)
{
	struct rtio_iodev *iodev = (struct rtio_iodev *)userdata;
	struct sensor_read_config *read_config = (struct sensor_read_config *)iodev->data;
	const struct device *sensor = read_config->sensor;
	const struct sensor_decoder_api *decoder;
	uint32_t dec_buf_size = sizeof(struct sensor_three_axis_data) +
				(256 - 1) * sizeof(struct sensor_three_axis_sample_data);
	uint8_t decoded_buffer[dec_buf_size];
	uint16_t frame_count;
	struct sensor_three_axis_data average_data[2];
	static atomic_t seq_ct = 0;
	int ret;

	uint32_t current_count = atomic_inc(&seq_ct);

	if (result < 0) {
		printk("recover: %d\n", result);

		bool *recovery = get_recovery_bool(iodev);
		*recovery = true;
		return;
	}

	ret = sensor_get_decoder(sensor, &decoder);
	__ASSERT_NO_MSG(ret == 0);

	{
		struct sensor_three_axis_data *dec_data =
			(struct sensor_three_axis_data *)decoded_buffer;
		uint32_t fit = 0;

		ret = decoder->get_frame_count(
			buf, (struct sensor_chan_spec) {SENSOR_CHAN_ACCEL_XYZ, 0}, &frame_count);
		__ASSERT_NO_MSG(ret >= 0);

		ret = decoder->decode(buf, (struct sensor_chan_spec) {SENSOR_CHAN_ACCEL_XYZ, 0},
				      &fit, 256, decoded_buffer);
		__ASSERT_NO_MSG(ret >= 0);

		avg_3_axis_q31(dec_data, ret, &average_data[0]);
	}
	{
		struct sensor_three_axis_data *dec_data =
			(struct sensor_three_axis_data *)decoded_buffer;
		uint32_t fit = 0;

		ret = decoder->get_frame_count(
			buf, (struct sensor_chan_spec) {SENSOR_CHAN_GYRO_XYZ, 0}, &frame_count);
		__ASSERT_NO_MSG(ret >= 0);
		ret = decoder->decode(buf, (struct sensor_chan_spec) {SENSOR_CHAN_GYRO_XYZ, 0},
				      &fit, 256, decoded_buffer);
		__ASSERT_NO_MSG(ret >= 0);

		avg_3_axis_q31(dec_data, ret, &average_data[1]);
	}

	char net_buffer[1024] = {0};

	int net_len =  snprintk(net_buffer, sizeof(net_buffer),
				"%d, %d, %s,"
				" %" PRIq(6) ", %" PRIq(6) ", %" PRIq(6) " ,"
				" %" PRIq(6) ", %" PRIq(6) ", %" PRIq(6) " , "
				"%lluns\n", current_count, ret, sensor->name,
				PRIq_arg(average_data[0].readings[0].x, 6, average_data[0].shift),
				PRIq_arg(average_data[0].readings[0].y, 6, average_data[0].shift),
				PRIq_arg(average_data[0].readings[0].z, 6, average_data[0].shift),
				PRIq_arg(average_data[1].readings[0].x, 6, average_data[1].shift),
				PRIq_arg(average_data[1].readings[0].y, 6, average_data[1].shift),
				PRIq_arg(average_data[1].readings[0].z, 6, average_data[1].shift),
				average_data[0].header.base_timestamp_ns);

	ssize_t err = send(conf.ipv4.udp.sock, net_buffer, net_len, 0);
	__ASSERT_NO_MSG(err >= 0);
}

static void sensor_processing_thread(void *arg1, void *arg2)
{
	struct rtio *ctx = (struct rtio *)arg1;
	struct rtio_iodev *iodev = (struct rtio_iodev *)arg2;
	int err;
	bool *recovery = get_recovery_bool(iodev);

	err = setup_stream(ctx, iodev);
	__ASSERT_NO_MSG(err == 0);

	printk("Starting main-loop: %p, %p\n", ctx, iodev);
	while (true) {
		sensor_processing_with_callback(ctx, process_events);
		if (*recovery) {
			*recovery = false;
			printk("Error during stream. Attempting recovery...\n");
			rtio_sqe_drop_all(ctx);
			k_sleep(K_MSEC(1));
			err = setup_stream(ctx, iodev);
			__ASSERT_NO_MSG(err == 0);
		}
	}
}

K_THREAD_DEFINE(sensor_stream_0_id, 8192, sensor_processing_thread,
		&sensor_ctx_0, &sensor_iodev_0, NULL, 8, 0, -1);
K_THREAD_DEFINE(sensor_stream_1_id, 8192, sensor_processing_thread,
		&sensor_ctx_1, &sensor_iodev_1, NULL, 8, 0, -1);

int main(void)
{
	int err;

	err = start_udp();
	__ASSERT_NO_MSG(err == 0);

	printk("Starting main-loop\n");

	k_thread_start(sensor_stream_0_id);
	k_thread_start(sensor_stream_1_id);

	return 0;
}
