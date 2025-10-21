/*
 * Copyright (c) 2022 Intel Corporation
 * Copyright (c) 2025 Croxel Inc.
 * Copyright (c) 2025 CogniPilot Foundation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT invensense_icm45686

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i3c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/rtio/rtio.h>

#if defined(CONFIG_SENSOR_ASYNC_API)
#include <zephyr/rtio/work.h>
#endif /* CONFIG_SENSOR_ASYNC_API */

#include "icm45686.h"
#include "icm45686_reg.h"
#include "icm45686_bus.h"
#include "icm45686_decoder.h"
#include "icm45686_trigger.h"
#include "icm45686_stream.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ICM45686, CONFIG_SENSOR_LOG_LEVEL);

static int set_reg8(const struct device *dev, uint8_t reg, uint8_t val) __attribute__((unused));
static int clear_reg8(const struct device *dev, uint8_t reg, uint8_t val) __attribute__((unused));
static int write_mreg(const struct device *dev, uint32_t reg, const uint8_t *buf, uint32_t len) __attribute__((unused));
static int read_mreg(const struct device *dev, uint32_t reg, uint8_t *buf, uint32_t len) __attribute__((unused));
static int icm45686_perform_selftest(const struct device *dev) __attribute__((unused));

static inline int reg_write(const struct device *dev, uint8_t reg, uint8_t val)
{
	printf("write reg 0x%x: 0x%x\n", reg, val);
	return icm45686_bus_write(dev, reg, &val, 1);
}

static inline int reg_read(const struct device *dev, uint8_t reg, uint8_t *val)
{
	int err = 0;

	err |= icm45686_bus_read(dev, reg, val, 1);
	// printf("read reg 0x%x: 0x%x\n", reg, *val);

	return err;
}

static int set_reg8(const struct device *dev, uint8_t reg, uint8_t val)
{
	int err = 0;
	int8_t buf;

	err |= icm45686_bus_read(dev, reg, &buf, 1);
	buf |= val;
	err |= icm45686_bus_write(dev, reg, &buf, 1);

	return err;
}

static int clear_reg8(const struct device *dev, uint8_t reg, uint8_t val)
{
	int err = 0;
	int8_t buf;

	err |= icm45686_bus_read(dev, reg, &buf, 1);
	buf &= ~val;
	err |= icm45686_bus_write(dev, reg, &buf, 1);

	return err;
}

static int write_mreg(const struct device *dev, uint32_t reg, const uint8_t *buf, uint32_t len)
{
	uint8_t data[3];
	int err = 0;

	// uint8_t read_val;

	/* First two bytes are the address where we want to write */
	data[0] = (reg & 0xFF00) >> 8;
	data[1] = reg & 0xFF;
	/* 3rd byte is the first data to write*/
	data[2] = buf[0];

	// printf("Writing reg: %x%x\n", data[0], data[1]);

	/* Burst write address and first byte */
	k_usleep(4);	//Sleep 4us
	err |= icm45686_bus_write(dev, REG_IREG_ADDR_15_8, data, 3);
	// err |= icm45686_bus_read(dev, REG_MISC2, &read_val, 1);
	// printf("REG_MISC2: %x\n", read_val);
	k_usleep(4);	//Sleep 4us
	// printf("REG_MISC2: %x\n", read_val);
	// printf("Wrote to reg: %x\n", data[2]);
	printf("write reg 0x%x%x: 0x%x\n", data[0], data[1], data[2]);

	for (int i = 1; i < len; i++)
	{
		err |= icm45686_bus_write(dev, REG_IREG_DATA, &buf[i], 1);
		// printf("Wrote to reg: %x\n", buf[i]);
		printf("write reg 0x%x%x+%d: 0x%x\n", data[0], data[1], i, buf[i]);
		k_usleep(4);
	}
	
	return err;
}

static int read_mreg(const struct device *dev, uint32_t reg, uint8_t *buf, uint32_t len)
{
	uint8_t data[2];
	int err = 0;

	/* Write address first */
	data[0] = (reg & 0xFF00) >> 8;
	data[1] = reg & 0xFF;
	// printf("reading reg: %x%x\n", data[0], data[1]);
	k_usleep(4);
	err |= icm45686_bus_write(dev, REG_IREG_ADDR_15_8, data, 2);
	k_usleep(10);

	/* Read all bytes one by one */
	for (int i = 0; i < len; i++) {
		err |= icm45686_bus_read(dev, REG_IREG_DATA, &buf[i], 1);
		// printf("read value from 0x%x%x: 0x%x\n", data[0], data[1], buf[i]);
		k_usleep(10);
	}

	return err;
}

static int icm45686_sample_fetch(const struct device *dev,
				 enum sensor_channel chan)
{
	int err;
	struct icm45686_data *data = dev->data;
	struct icm45686_encoded_data *edata = &data->edata;

	if (chan != SENSOR_CHAN_ALL) {
		return -ENOTSUP;
	}

	err = icm45686_bus_read(dev,
				REG_ACCEL_DATA_X1_UI,
				edata->payload.buf,
				sizeof(edata->payload.buf));

	LOG_HEXDUMP_DBG(edata->payload.buf,
			sizeof(edata->payload.buf),
			"ICM45686 data");

	return err;
}

static int icm45686_channel_get(const struct device *dev,
				enum sensor_channel chan,
				struct sensor_value *val)
{
	struct icm45686_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		icm45686_accel_ms(data->edata.header.accel_fs, data->edata.payload.accel.x, false,
				  &val->val1, &val->val2);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		icm45686_accel_ms(data->edata.header.accel_fs, data->edata.payload.accel.y, false,
				  &val->val1, &val->val2);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		icm45686_accel_ms(data->edata.header.accel_fs, data->edata.payload.accel.z, false,
				  &val->val1, &val->val2);
		break;
	case SENSOR_CHAN_GYRO_X:
		icm45686_gyro_rads(data->edata.header.gyro_fs, data->edata.payload.gyro.x, false,
				   &val->val1, &val->val2);
		break;
	case SENSOR_CHAN_GYRO_Y:
		icm45686_gyro_rads(data->edata.header.gyro_fs, data->edata.payload.gyro.y, false,
				   &val->val1, &val->val2);
		break;
	case SENSOR_CHAN_GYRO_Z:
		icm45686_gyro_rads(data->edata.header.gyro_fs, data->edata.payload.gyro.z, false,
				   &val->val1, &val->val2);
		break;
	case SENSOR_CHAN_DIE_TEMP:
		icm45686_temp_c(data->edata.payload.temp, &val->val1, &val->val2);
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		icm45686_accel_ms(data->edata.header.accel_fs, data->edata.payload.accel.x, false,
				  &val[0].val1, &val[0].val2);
		icm45686_accel_ms(data->edata.header.accel_fs, data->edata.payload.accel.y, false,
				  &val[1].val1, &val[1].val2);
		icm45686_accel_ms(data->edata.header.accel_fs, data->edata.payload.accel.z, false,
				  &val[2].val1, &val[2].val2);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		icm45686_gyro_rads(data->edata.header.gyro_fs, data->edata.payload.gyro.x, false,
				   &val->val1, &val->val2);
		icm45686_gyro_rads(data->edata.header.gyro_fs, data->edata.payload.gyro.y, false,
				   &val[1].val1, &val[1].val2);
		icm45686_gyro_rads(data->edata.header.gyro_fs, data->edata.payload.gyro.z, false,
				   &val[2].val1, &val[2].val2);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

#if defined(CONFIG_SENSOR_ASYNC_API)

static void icm45686_complete_result(struct rtio *ctx,
				     const struct rtio_sqe *sqe,
				     void *arg)
{
	struct rtio_iodev_sqe *iodev_sqe = (struct rtio_iodev_sqe *)sqe->userdata;
	struct rtio_cqe *cqe;
	int err = 0;

	do {
		cqe = rtio_cqe_consume(ctx);
		if (cqe != NULL) {
			err = cqe->result;
			rtio_cqe_release(ctx, cqe);
		}
	} while (cqe != NULL);

	if (err) {
		rtio_iodev_sqe_err(iodev_sqe, err);
	} else {
		rtio_iodev_sqe_ok(iodev_sqe, 0);
	}

	LOG_DBG("One-shot fetch completed");
}

static inline void icm45686_submit_one_shot(const struct device *dev,
					    struct rtio_iodev_sqe *iodev_sqe)
{
	const struct sensor_read_config *cfg = iodev_sqe->sqe.iodev->data;
	const struct sensor_chan_spec *const channels = cfg->channels;
	const size_t num_channels = cfg->count;
	uint32_t min_buf_len = sizeof(struct icm45686_encoded_data);
	int err;
	uint8_t *buf;
	uint32_t buf_len;
	struct icm45686_encoded_data *edata;
	struct icm45686_data *data = dev->data;

	err = rtio_sqe_rx_buf(iodev_sqe, min_buf_len, min_buf_len, &buf, &buf_len);
	if (err != 0) {
		LOG_ERR("Failed to get a read buffer of size %u bytes", min_buf_len);
		rtio_iodev_sqe_err(iodev_sqe, err);
		return;
	}

	edata = (struct icm45686_encoded_data *)buf;

	err = icm45686_encode(dev, channels, num_channels, buf);
	if (err != 0) {
		LOG_ERR("Failed to encode sensor data");
		rtio_iodev_sqe_err(iodev_sqe, err);
		return;
	}

	struct rtio_sqe *write_sqe = rtio_sqe_acquire(data->rtio.ctx);
	struct rtio_sqe *read_sqe = rtio_sqe_acquire(data->rtio.ctx);
	struct rtio_sqe *complete_sqe = rtio_sqe_acquire(data->rtio.ctx);

	if (!write_sqe || !read_sqe || !complete_sqe) {
		LOG_ERR("Failed to acquire RTIO SQEs");
		rtio_iodev_sqe_err(iodev_sqe, -ENOMEM);
		return;
	}

	uint8_t val = REG_ACCEL_DATA_X1_UI | REG_READ_BIT;

	rtio_sqe_prep_tiny_write(write_sqe,
				 data->rtio.iodev,
				 RTIO_PRIO_HIGH,
				 &val,
				 1,
				NULL);
	write_sqe->flags |= RTIO_SQE_TRANSACTION;

	rtio_sqe_prep_read(read_sqe,
			   data->rtio.iodev,
			   RTIO_PRIO_HIGH,
			   edata->payload.buf,
			   sizeof(edata->payload.buf),
			   NULL);
	if (data->rtio.type == ICM45686_BUS_I2C) {
		read_sqe->iodev_flags |= RTIO_IODEV_I2C_STOP | RTIO_IODEV_I2C_RESTART;
	} else if (data->rtio.type == ICM45686_BUS_I3C) {
		read_sqe->iodev_flags |= RTIO_IODEV_I3C_STOP | RTIO_IODEV_I3C_RESTART;
	}
	read_sqe->flags |= RTIO_SQE_CHAINED;

	rtio_sqe_prep_callback_no_cqe(complete_sqe,
				      icm45686_complete_result,
				      (void *)dev,
				      iodev_sqe);

	rtio_submit(data->rtio.ctx, 0);
}

static void icm45686_submit(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	const struct sensor_read_config *cfg = iodev_sqe->sqe.iodev->data;

	if (!cfg->is_streaming) {
		icm45686_submit_one_shot(dev, iodev_sqe);
	} else if (IS_ENABLED(CONFIG_ICM45686_STREAM)) {
		icm45686_stream_submit(dev, iodev_sqe);
	} else {
		LOG_ERR("Streaming not supported");
		rtio_iodev_sqe_err(iodev_sqe, -ENOTSUP);
	}
}

#endif /* CONFIG_SENSOR_ASYNC_API */

static DEVICE_API(sensor, icm45686_driver_api) = {
	.sample_fetch = icm45686_sample_fetch,
	.channel_get = icm45686_channel_get,
#if defined(CONFIG_ICM45686_TRIGGER)
	.trigger_set = icm45686_trigger_set,
#endif /* CONFIG_ICM45686_TRIGGER */
#if defined(CONFIG_SENSOR_ASYNC_API)
	.get_decoder = icm45686_get_decoder,
	.submit = icm45686_submit,
#endif /* CONFIG_SENSOR_ASYNC_API */
};

static int icm45686_perform_selftest(const struct device *dev)
{
	inv_imu_selftest_parameters_t st_params;
	uint32_t tmp_stc_params = 0;
	uint8_t read_val;
	uint32_t timeout_us = 3000000; /* 3 seconds */
	int err = 0;

	int8_t intf_config1_ovrd;
	int8_t drive_config0;

	uint8_t stc_status_reg, stc_status, gyro_accel_status;

	/* Do soft-reset */
	//Save INTF_CONFIG1_OVRD register
	reg_read(dev, REG_INTF_CONFIG1_OVRD, &intf_config1_ovrd);

	//Save DRIVE_CONFIG0 register
	reg_read(dev, REG_DRIVE_CONFIG0, &drive_config0);

	//Trigger soft-reset and do not change IREG_DONE bit
	// reg_write(dev, REG_MISC2, 0x3);
	set_reg8(dev, REG_MISC2, 0x2);

	//Wait 1ms for soft reset to be effective
	k_usleep(1000); 

	//Restore INTF_CONFIG1_OVRD register
	reg_write(dev, REG_INTF_CONFIG1_OVRD, intf_config1_ovrd);

	//Restore DRIVE_CONFIG0 register
	reg_write(dev, REG_DRIVE_CONFIG0, drive_config0);

	err |= reg_read(dev, REG_INT1_STATUS0, &read_val);
	//Check if RESET_DONE is set
	if (!((read_val & 0x80) >> 7))
	{
		//Return error if RESET_DONE is not set
		return -1;
	}
	
	//Sleep 10ms
	k_usleep(10000);

	/* Configure start-address for EDMP */
	//Write start address of 3 EDMP IRQ handlers in EDMP_PRGRM_IRQ0_0
	//EDMP_ROM_START_ADDR_IRQ0, EDMP_ROM_START_ADDR_IRQ1, EDMP_ROM_START_ADDR_IRQ2
	uint8_t start_addr[] = {EDMP_ROM_START_ADDR_IRQ0 & 0xff,
				(EDMP_ROM_START_ADDR_IRQ0 & 0xff00) >> 8,
				EDMP_ROM_START_ADDR_IRQ1 & 0xff,
				(EDMP_ROM_START_ADDR_IRQ1 & 0xff00) >> 8,
				EDMP_ROM_START_ADDR_IRQ2 & 0xff,
				(EDMP_ROM_START_ADDR_IRQ2 & 0xff00) >> 8};

	err |= write_mreg(dev, REG_EDMP_PRGRM_IRQ0_0, &start_addr[0], sizeof(start_addr));
	err |= read_mreg(dev, REG_EDMP_PRGRM_IRQ0_0, &read_val, 1);
	printf("REG_EDMP_PRGRM_IRQ0_0: %x\n", read_val);
	err |= read_mreg(dev, REG_EDMP_PRGRM_IRQ0_1, &read_val, 1);
	printf("REG_EDMP_PRGRM_IRQ0_1: %x\n", read_val);

	// reg_write(dev, REG_EDMP_PRGRM_IRQ0_0, (EDMP_ROM_START_ADDR_IRQ0 & 0xff));
	// reg_write(dev, REG_EDMP_PRGRM_IRQ0_1, ((EDMP_ROM_START_ADDR_IRQ0 & 0xff00) >> 8));
	// reg_write(dev, REG_EDMP_PRGRM_IRQ1_0, (EDMP_ROM_START_ADDR_IRQ1 & 0xff));
	// reg_write(dev, REG_EDMP_PRGRM_IRQ1_1, ((EDMP_ROM_START_ADDR_IRQ1 & 0xff00) >> 8));
	// reg_write(dev, REG_EDMP_PRGRM_IRQ2_0, (EDMP_ROM_START_ADDR_IRQ2 & 0xff));
	// reg_write(dev, REG_EDMP_PRGRM_IRQ2_1, ((EDMP_ROM_START_ADDR_IRQ2 & 0xff00) >> 8));

	//Write stack pointer start addres to EDMP_SP_START_ADDR
	// /* Only 8 MSB of SP address is written to register */
	//uint8_t stack_addr = (uint8_t)(APEX_FEATURE_STACK_END >> 8);
	err |= write_mreg(dev, REG_EDMP_SP_START_ADDR, (uint8_t *)(APEX_FEATURE_STACK_END >> 8), 1);
	err |= read_mreg(dev, REG_EDMP_SP_START_ADDR, &read_val, 1);
	printf("REG_EDMP_SP_START_ADDR: %x\n", read_val);


	//Initialize buffer pointers
	err |= reg_read(dev, REG_APEX_BUFFER_MGMT, &read_val);
	printf("REG_APEX_BUFFER_MGMT: %x\n", read_val);
	read_val |= 0x8;	//Set host reads buffer 0
	read_val &= ~0x3;	//Set eDMP write to buffer 0
	err |= reg_write(dev, REG_APEX_BUFFER_MGMT, read_val);
	err |= reg_read(dev, REG_APEX_BUFFER_MGMT, &read_val);
	printf("REG_APEX_BUFFER_MGMT: %x\n", read_val);

	err |= read_mreg(dev, REG_FIFO_SRAM_SLEEP, &read_val, 1);
	printf("REG_FIFO_SRAM_SLEEP: %x\n", read_val);

	/* Set self-test parameters */
	//Power up SRAMd
	err |= read_mreg(dev, REG_FIFO_SRAM_SLEEP, &read_val, 1);
	read_val |= 0x3;
	err |= write_mreg(dev, REG_FIFO_SRAM_SLEEP, &read_val, 1);
	err |= read_mreg(dev, REG_FIFO_SRAM_SLEEP, &read_val, 1);
	printf("REG_FIFO_SRAM_SLEEP: %x\n", read_val);

	//Set self-test parameters
	st_params.stc_init_en 		= SELFTESTCAL_INIT_EN;
	st_params.accel_en 			= SELFTEST_ACCEL_EN;
	st_params.gyro_en 			= SELFTEST_GYRO_EN;
	st_params.avg_time 			= SELFTEST_AVG_TIME_320_MS;
	st_params.accel_limit 		= SELFTEST_ACCEL_THRESHOLD_50_PERCENT;
	st_params.gyro_limit 		= SELFTEST_GYRO_THRESHOLD_50_PERCENT;
	st_params.patch_settings 	= 0;

	// init_en = (st_params->accel_en || st_params->gyro_en);
	// tmp_stc_params |= (init_en ? SELFTESTCAL_INIT_EN : SELFTESTCAL_INIT_DIS);
	// tmp_stc_params |= (st_params->accel_en ? SELFTEST_ACCEL_EN : SELFTEST_ACCEL_DIS);
	// tmp_stc_params |= (st_params->gyro_en ? SELFTEST_GYRO_EN : SELFTEST_GYRO_DIS);
	// tmp_stc_params |= (uint32_t)(st_params->accel_limit & SELFTEST_ACCEL_THRESH_MASK);
	// tmp_stc_params |= (uint32_t)(st_params->gyro_limit & SELFTEST_GYRO_THRESH_MASK);
	// tmp_stc_params |= (uint32_t)(st_params->avg_time & SELFTEST_AVERAGE_TIME_MASK);

	//Set self-test parameters
	tmp_stc_params = st_params.stc_init_en | st_params.accel_en | st_params.gyro_en | st_params.avg_time | st_params.accel_limit | st_params.gyro_limit;
	
	err |= read_mreg(dev, REG_IMEM_SRAM_REG_56, &read_val, 1);
	printf("first read REG_IMEM_SRAM_REG_56: %x\n", read_val);

	//Write self-test parameters to registers
	err |= write_mreg(dev, REG_IMEM_SRAM_REG_56, (uint8_t *)&tmp_stc_params, 2);

	err |= read_mreg(dev, REG_IMEM_SRAM_REG_56, &read_val, 1);
	printf("REG_IMEM_SRAM_REG_56: %x\n", read_val);

	err |= read_mreg(dev, REG_IMEM_SRAM_REG_57, &read_val, 1);
	printf("REG_IMEM_SRAM_REG_57: %x\n", read_val);

	//Disable debug at self-test
	uint8_t debug_en = 0;
	err |= write_mreg(dev, REG_IMEM_SRAM_REG_64, (uint8_t *)&debug_en, 1);

	//Set patch settings
	err |= write_mreg(dev, REG_IMEM_SRAM_STC_PATCH_EN, (uint8_t *)&(st_params.patch_settings), 1);

	err |= read_mreg(dev, REG_IMEM_SRAM_STC_PATCH_EN, &read_val, 1);
	printf("REG_IMEM_SRAM_STC_PATCH_EN: %x\n", read_val);

	/* Run internal self-test */
	//Read Host_MSG and only set the testopenable bit
	err |= reg_read(dev, REG_HOST_MSG, &read_val);
	read_val |= 0x1; 	//Enable test operation
	err |= reg_write(dev, REG_HOST_MSG, read_val);
	err |= reg_read(dev, REG_HOST_MSG, &read_val);
	printf("REG_HOST_MSG: %x\n", read_val);

	//Enable all self-tests
	// err |= read_mreg(dev, REG_SELFTEST, &read_val, 1);
	// read_val |= 0x3f;	//Enable all gyro and accel self-tests
	// err |= write_mreg(dev, REG_SELFTEST, &read_val, 1);
	// err |= read_mreg(dev, REG_SELFTEST, &read_val, 1);
	// printf("REG_SELFTEST: %x\n", read_val);

	//Enable self-test interrupt generation in APEX_CONFIG1 register
	err |= reg_read(dev, REG_INT_APEX_CONFIG1, &read_val);
	read_val &= ~0x4; 	//Enable interrupt generation when self-test is done
	read_val |= 0x13;	//Disable all other interrupts
	err |= reg_write(dev, REG_INT_APEX_CONFIG1, read_val);
	err |= reg_read(dev, REG_INT_APEX_CONFIG1, &read_val);
	printf("REG_INT_APEX_CONFIG1: %x\n", read_val);

	// err |= reg_read(dev, REG_INT_APEX_CONFIG0, &read_val);
	// read_val |= 0xff;	//Disable all other interrupts
	// err |= reg_write(dev, REG_INT_APEX_CONFIG0, read_val);
	// err |= reg_read(dev, REG_INT_APEX_CONFIG0, &read_val);
	// printf("REG_INT_APEX_CONFIG0: %x\n", read_val);

	//Disable the eDMP to be run once when IRQ2 is triggered by setting the EDMP_ON_DEMAND_EN bit.
	err |= read_mreg(dev, REG_STATUS_MASK_PIN_16_23, &read_val, 1);
	read_val &= ~0x20;	//Clear INT_ON_DEMAND_PIN_2_DIS
	err |= write_mreg(dev, REG_STATUS_MASK_PIN_16_23, &read_val, 1);
	err |= read_mreg(dev, REG_STATUS_MASK_PIN_16_23, &read_val, 1);
	printf("REG_STATUS_MASK_PIN_16_23: %x\n", read_val);

	// err |= read_mreg(dev, REG_STATUS_MASK_PIN_0_7, &read_val, 1);
	// // read_val |= 0x20;	//Set INT_ON_DEMAND_PIN_0_DIS
	// read_val &= ~(0x4 | 0x20);	//Clear bits other than reserved
	// err |= write_mreg(dev, REG_STATUS_MASK_PIN_0_7, &read_val, 1);
	err |= read_mreg(dev, REG_STATUS_MASK_PIN_0_7, &read_val, 1);
	printf("REG_STATUS_MASK_PIN_0_7: %x\n", read_val);

	// err |= read_mreg(dev, REG_STATUS_MASK_PIN_8_15, &read_val, 1);
	// read_val &= ~(0x1 | 0x8 | 0x20);	//Clear bits other than reserved
	// err |= write_mreg(dev, REG_STATUS_MASK_PIN_8_15, &read_val, 1);
	err |= read_mreg(dev, REG_STATUS_MASK_PIN_8_15, &read_val, 1);
	printf("REG_STATUS_MASK_PIN_8_15: %x\n", read_val);

	//Enable EDMP in EDMP_APEX_EN1 register
	err |= reg_read(dev, REG_EDMP_APEX_EN1, &read_val);
	read_val |= 0x40;	//Enable EDMP
	err |= reg_write(dev, REG_EDMP_APEX_EN1, read_val);
	err |= reg_read(dev, REG_EDMP_APEX_EN1, &read_val);
	printf("REG_EDMP_APEX_EN1: %x\n", read_val);

	//Run EDMP on demand by setting it in Host_MSG register
	err |= reg_read(dev, REG_HOST_MSG, &read_val);
	read_val |= 0x20; 	//Enable EDMP on demand
	err |= reg_write(dev, REG_HOST_MSG, read_val);
	err |= reg_read(dev, REG_HOST_MSG, &read_val);
	printf("REG_HOST_MSG: %x\n", read_val);

	err |= read_mreg(dev, REG_IPREG_MISC, &read_val, 1);
	if ((read_val & 0x2) >> 1)
	{
		printf("eDMP is idle\n");
	}
	else
	{
		printf("eDMP is busy\n");
	}	

	// read_val = 0;
	// write_mreg(dev, REG_IMEM_SRAM_REG_68, &read_val, 1);
	// read_mreg(dev, REG_IMEM_SRAM_REG_68, &stc_status_reg, 1);
	// printf("stc_status_reg: %x\n", stc_status_reg);

	k_usleep(200);	//Wait 200us

	//Run a while loop to wait for the self-test done interrupt in apex_status1 register
	//Every loop wait 100us. Return error after timeout of 100 loops (10ms)
	while (1) {
		uint8_t apex_status1_reg = 0;
		err |= reg_read(dev, REG_INT_APEX_STATUS1, &apex_status1_reg);

		// if (apex_status1_reg > 0)
		// {
		// 	printf("REG_INT_APEX_STATUS1: %x\t", apex_status1_reg);
		// }

		// err |= read_mreg(dev, REG_IPREG_MISC, &read_val, 1);
		// if (!((read_val & 0x2) >> 1))
		// {
		// 	printf("eDMP is busy\t");
		// }

		//If 
		if ((apex_status1_reg & 0x4) >> 2)
		{
			printf("got interrupt!\n");
			break;
		}

		k_usleep(100);
		timeout_us -= 100;

		if (timeout_us <= 0)
		{
			printf("REG_INT_APEX_STATUS1: %x\n", apex_status1_reg);

			// read_mreg(dev, REG_STATUS_MASK_PIN_0_7, &read_val, 1);
			// printf("REG_STATUS_MASK_PIN_0_7: %x\n", read_val);

			// read_mreg(dev, REG_STATUS_MASK_PIN_8_15, &read_val, 1);
			// printf("REG_STATUS_MASK_PIN_8_15: %x\n", read_val);

			// read_mreg(dev, REG_STATUS_MASK_PIN_16_23, &read_val, 1);
			// printf("REG_STATUS_MASK_PIN_16_23: %x\n", read_val);

			// read_mreg(dev, REG_ISR_0_7, &read_val, 1);
			// printf("REG_ISR_0_7: %x\n", read_val);

			// read_mreg(dev, REG_ISR_8_15, &read_val, 1);
			// printf("REG_ISR_8_15: %x\n", read_val);

			// read_mreg(dev, REG_ISR_16_23, &read_val, 1);
			// printf("REG_ISR_16_23: %x\n", read_val);

			read_mreg(dev, REG_IMEM_SRAM_REG_68, &stc_status_reg, 1);
			printf("stc_status_reg: %x\n", stc_status_reg);
			// return -1;
			break;
		}
	}

	//Disable self-test interrupt generation in APEX_CONFIG1 register
	reg_read(dev, REG_INT_APEX_CONFIG1, &read_val);
	read_val |= 0x4;	//Disable interrupt generation when self-test is done
	reg_write(dev, REG_INT_APEX_CONFIG1, read_val);

	/* Get self-test output */
	//Read ST_status from IMEM_SRAM_REG_68
	read_mreg(dev, REG_IMEM_SRAM_REG_68, &stc_status_reg, 1);

	printf("stc_status_reg: %x\n", stc_status_reg);

	stc_status 			= stc_status_reg >> 6;
	gyro_accel_status 	= stc_status_reg & 0x3f;

	//Check if ST_STATUS == 0 -> Self-test done
	if (stc_status == 0)
	{
		//Check if one of the tests failed
		if (gyro_accel_status != 0x3f)
		{
			//Failed self-test
			err |= -EIO;
		}
		
		//Check if gyro and accel axis passed test
		if (gyro_accel_status & 0x1)	//Accel X passed
		{
			printk("Accel X passed\n");
		}
		if (gyro_accel_status & 0x2)	//Accel Y passed
		{
			printk("Accel Y passed\n");
		}
		if (gyro_accel_status & 0x4)	//Accel Z passed
		{
			printk("Accel Z passed\n");
		}
		if (gyro_accel_status & 0x8)	//Gyro X passed
		{
			printk("Gyro X passed\n");
		}
		if (gyro_accel_status & 0x10)	//Gyro Y passed
		{
			printk("Gyro Y passed\n");
		}
		if (gyro_accel_status & 0x20)	//Gyro Z passed
		{
			printk("Gyro Z passed\n");
		}
		
	}
	else if (stc_status == 1)	// self-test in progress
	{
		//Failed test
		err |= -EIO;
	}
	else	// self-test error or reserved
	{
		//Failed test
		err |= -EIO;
	}
	
	err = -EIO;

	//Return pass or fail
	return err;
}

static int icm45686_init(const struct device *dev)
{
	struct icm45686_data *data = dev->data;
	const struct icm45686_config *cfg = dev->config;
	uint8_t read_val = 0;
	uint8_t val;
	int err;

#if CONFIG_SPI_RTIO
	if ((data->rtio.type == ICM45686_BUS_SPI) && !spi_is_ready_iodev(data->rtio.iodev)) {
		LOG_ERR("Bus is not ready");
		return -ENODEV;
	}
#endif
#if CONFIG_I2C_RTIO
	if ((data->rtio.type == ICM45686_BUS_I2C) && !i2c_is_ready_iodev(data->rtio.iodev)) {
		LOG_ERR("Bus is not ready");
		return -ENODEV;
	}
#endif

	/** Soft-reset sensor to restore config to defaults,
	 * unless it's already handled by I3C initialization.
	 */
	if (data->rtio.type != ICM45686_BUS_I3C) {
		err = reg_write(dev, REG_MISC2, REG_MISC2_SOFT_RST(1));
		if (err) {
			LOG_ERR("Failed to write soft-reset: %d", err);
			return err;
		}
		/* Wait for soft-reset to take effect */
		k_sleep(K_MSEC(1));

		/* A complete soft-reset clears the bit */
		err = reg_read(dev, REG_MISC2, &read_val);
		if (err) {
			LOG_ERR("Failed to read soft-reset: %d", err);
			return err;
		}
		if ((read_val & REG_MISC2_SOFT_RST(1)) != 0) {
			LOG_ERR("Soft-reset command failed");
			return -EIO;
		}
	}

	/* Set Slew-rate to 10-ns typical, to allow proper SPI readouts */

	err = reg_write(dev, REG_DRIVE_CONFIG0, REG_DRIVE_CONFIG0_SPI_SLEW(2));
	if (err) {
		LOG_ERR("Failed to write slew-rate: %d", err);
		return err;
	}
	err = reg_write(dev, REG_DRIVE_CONFIG1, REG_DRIVE_CONFIG1_I3C_SLEW(3));
	if (err) {
		LOG_ERR("Failed to write slew-rate: %d", err);
		return err;
	}
	/* Wait for register to take effect */
	k_sleep(K_USEC(2));

	/* Confirm ID Value matches */
	err = reg_read(dev, REG_WHO_AM_I, &read_val);
	if (err) {
		LOG_ERR("Failed to read WHO_AM_I: %d", err);
		return err;
	}
	if (read_val != WHO_AM_I_ICM45686) {
		LOG_ERR("Unexpected WHO_AM_I value - expected: 0x%02x, actual: 0x%02x",
			WHO_AM_I_ICM45686, read_val);
		return -EIO;
	}

#if CONFIG_ICM45686_SELFTEST_EN
	if (icm45686_perform_selftest(dev) != 0)
	{
		LOG_ERR("Failed self-test");
		return -EIO;
	}
	else
	{
		printk("Self-test was succesfull!\n");
	}
#endif

	/* Sensor Configuration */

	val = REG_PWR_MGMT0_ACCEL_MODE(cfg->settings.accel.pwr_mode) |
	      REG_PWR_MGMT0_GYRO_MODE(cfg->settings.gyro.pwr_mode);
	err = reg_write(dev, REG_PWR_MGMT0, val);
	if (err) {
		LOG_ERR("Failed to write Power settings: %d", err);
		return err;
	}

	val = REG_ACCEL_CONFIG0_ODR(cfg->settings.accel.odr) |
	      REG_ACCEL_CONFIG0_FS(cfg->settings.accel.fs);
	err = reg_write(dev, REG_ACCEL_CONFIG0, val);
	if (err) {
		LOG_ERR("Failed to write Accel settings: %d", err);
		return err;
	}

	val = REG_GYRO_CONFIG0_ODR(cfg->settings.gyro.odr) |
	      REG_GYRO_CONFIG0_FS(cfg->settings.gyro.fs);
	err = reg_write(dev, REG_GYRO_CONFIG0, val);
	if (err) {
		LOG_ERR("Failed to write Gyro settings: %d", err);
		return err;
	}

	/** Write Low-pass filter settings through indirect register access */
	uint8_t gyro_lpf_write_array[] = REG_IREG_PREPARE_WRITE_ARRAY(
						REG_IPREG_SYS1_OFFSET,
						REG_IPREG_SYS1_REG_172,
						REG_IPREG_SYS1_REG_172_GYRO_LPFBW_SEL(
							cfg->settings.gyro.lpf));

	err = icm45686_bus_write(dev, REG_IREG_ADDR_15_8, gyro_lpf_write_array,
				 sizeof(gyro_lpf_write_array));
	if (err) {
		LOG_ERR("Failed to set Gyro BW settings: %d", err);
		return err;
	}

	/** Wait before indirect register write is made effective
	 * before proceeding with next one.
	 */
	k_sleep(K_MSEC(1));

	uint8_t accel_lpf_write_array[] = REG_IREG_PREPARE_WRITE_ARRAY(
						REG_IPREG_SYS2_OFFSET,
						REG_IPREG_SYS2_REG_131,
						REG_IPREG_SYS2_REG_131_ACCEL_LPFBW_SEL(
							cfg->settings.accel.lpf));

	err = icm45686_bus_write(dev, REG_IREG_ADDR_15_8, accel_lpf_write_array,
				 sizeof(accel_lpf_write_array));
	if (err) {
		LOG_ERR("Failed to set Accel BW settings: %d", err);
		return err;
	}

	if (IS_ENABLED(CONFIG_ICM45686_TRIGGER)) {
		err = icm45686_trigger_init(dev);
		if (err) {
			LOG_ERR("Failed to initialize triggers: %d", err);
			return err;
		}
	} else if (IS_ENABLED(CONFIG_ICM45686_STREAM)) {
		err = icm45686_stream_init(dev);
		if (err) {
			LOG_ERR("Failed to initialize streaming: %d", err);
			return err;
		}
	}

	LOG_DBG("Init OK");

	return 0;
}

#define ICM45686_VALID_ACCEL_ODR(pwr_mode, odr)							   \
	((pwr_mode == ICM45686_DT_ACCEL_LP && odr >= ICM45686_DT_ACCEL_ODR_400) ||		   \
	 (pwr_mode == ICM45686_DT_ACCEL_LN && odr <= ICM45686_DT_ACCEL_ODR_12_5) ||		   \
	 (pwr_mode == ICM45686_DT_ACCEL_OFF))

#define ICM45686_VALID_GYRO_ODR(pwr_mode, odr)							   \
	((pwr_mode == ICM45686_DT_GYRO_LP && odr >= ICM45686_DT_GYRO_ODR_400) ||		   \
	 (pwr_mode == ICM45686_DT_GYRO_LN && odr <= ICM45686_DT_GYRO_ODR_12_5) ||		   \
	 (pwr_mode == ICM45686_DT_GYRO_OFF))

#define ICM45686_INIT(inst)									   \
												   \
	RTIO_DEFINE(icm45686_rtio_ctx_##inst, 8, 8);						   \
												   \
	COND_CODE_1(DT_INST_ON_BUS(inst, i3c),							   \
		    (I3C_DT_IODEV_DEFINE(icm45686_bus_##inst,					   \
					 DT_DRV_INST(inst))),					   \
	(COND_CODE_1(DT_INST_ON_BUS(inst, i2c),							   \
		    (I2C_DT_IODEV_DEFINE(icm45686_bus_##inst,					   \
					 DT_DRV_INST(inst))),					   \
		    ())));									   \
	COND_CODE_1(DT_INST_ON_BUS(inst, spi),							   \
		    (SPI_DT_IODEV_DEFINE(icm45686_bus_##inst,					   \
					 DT_DRV_INST(inst),					   \
					 SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB,  \
					 0U)),							   \
		    ());									   \
												   \
												   \
	static const struct icm45686_config icm45686_cfg_##inst = {				   \
		.settings = {									   \
			.accel = {								   \
				.pwr_mode = DT_INST_PROP(inst, accel_pwr_mode),			   \
				.fs = DT_INST_PROP(inst, accel_fs),				   \
				.odr = DT_INST_PROP(inst, accel_odr),				   \
				.lpf = DT_INST_PROP_OR(inst, accel_lpf, 0),			   \
			},									   \
			.gyro = {								   \
				.pwr_mode = DT_INST_PROP(inst, gyro_pwr_mode),			   \
				.fs = DT_INST_PROP(inst, gyro_fs),				   \
				.odr = DT_INST_PROP(inst, gyro_odr),				   \
				.lpf = DT_INST_PROP_OR(inst, gyro_lpf, 0),			   \
			},									   \
			.fifo_watermark = DT_INST_PROP_OR(inst, fifo_watermark, 0),		   \
		},										   \
		.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}),			   \
	};											   \
	static struct icm45686_data icm45686_data_##inst = {					   \
		.edata.header = {								   \
			.accel_fs = DT_INST_PROP(inst, accel_fs),				   \
			.gyro_fs = DT_INST_PROP(inst, gyro_fs),					   \
		},										   \
		.rtio = {									   \
			.iodev = &icm45686_bus_##inst,						   \
			.ctx = &icm45686_rtio_ctx_##inst,					   \
			COND_CODE_1(DT_INST_ON_BUS(inst, i3c),					   \
				(.type = ICM45686_BUS_I3C,					   \
				 .i3c.id = I3C_DEVICE_ID_DT_INST(inst),),			   \
			(COND_CODE_1(DT_INST_ON_BUS(inst, i2c),					   \
				(.type = ICM45686_BUS_I2C), ())))				   \
			COND_CODE_1(DT_INST_ON_BUS(inst, spi),					   \
				(.type = ICM45686_BUS_SPI), ())					   \
		},										   \
	};											   \
												   \
	/* Build-time settings verification: Inform the user of invalid settings at build time */  \
	BUILD_ASSERT(ICM45686_VALID_ACCEL_ODR(DT_INST_PROP(inst, accel_pwr_mode),		   \
					      DT_INST_PROP(inst, accel_odr)),			   \
		     "Invalid accel ODR setting. Please check supported ODRs for LP and LN");	   \
	BUILD_ASSERT(ICM45686_VALID_GYRO_ODR(DT_INST_PROP(inst, gyro_pwr_mode),			   \
					     DT_INST_PROP(inst, gyro_odr)),			   \
		     "Invalid gyro ODR setting. Please check supported ODRs for LP and LN");	   \
												   \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, icm45686_init,					   \
				     NULL,							   \
				     &icm45686_data_##inst,					   \
				     &icm45686_cfg_##inst,					   \
				     POST_KERNEL,						   \
				     CONFIG_SENSOR_INIT_PRIORITY,				   \
				     &icm45686_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ICM45686_INIT)
