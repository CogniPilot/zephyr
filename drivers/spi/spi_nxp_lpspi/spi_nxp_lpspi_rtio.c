/*
 * Copyright 2024-2025 NXP
 * Copyright 2025 Croxel, Inc.
 * Copyright 2025 CogniPilot Foundation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_lpspi

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(spi_lpspi, CONFIG_SPI_LOG_LEVEL);

#include "spi_nxp_lpspi_priv.h"

struct lpspi_driver_data {
	struct spi_rtio *rtio_ctx;
	uint8_t word_size_bytes;
	struct {
		size_t words_to_clock;
		size_t words_clocked_tx;
		size_t words_clocked_rx;
	} total;
	struct {
		struct rtio_sqe *sqe;
		size_t words_clocked;
	} tx_curr;
	struct {
		struct rtio_sqe *sqe;
		size_t words_clocked;
	} rx_curr;
	uint8_t lpspi_op_mode;
};

static inline size_t get_sqe_clock_cycles(struct rtio_sqe *sqe)
{
	switch (sqe->op) {
	case RTIO_OP_RX:
		return sqe->rx.buf_len;
	case RTIO_OP_TX:
		return sqe->tx.buf_len;
	case RTIO_OP_TINY_TX:
		return sqe->tiny_tx.buf_len;
	case RTIO_OP_TXRX:
		return sqe->txrx.buf_len;
	default:
		return 0;
	}
}

static inline struct rtio_sqe *get_next_sqe(struct rtio_sqe *sqe)
{
	struct rtio_iodev_sqe *curr_iodev_sqe = CONTAINER_OF(sqe, struct rtio_iodev_sqe, sqe);
	struct rtio_iodev_sqe *next_iodev_sqe = rtio_txn_next(curr_iodev_sqe);

	return &next_iodev_sqe->sqe;
}

static inline size_t get_total_sqe_clock_cycles(struct rtio_sqe *head)
{
	size_t total_size = 0;
	struct rtio_iodev_sqe *curr_iodev_sqe = CONTAINER_OF(head, struct rtio_iodev_sqe, sqe);

	while (curr_iodev_sqe != NULL) {
		total_size += get_sqe_clock_cycles(&curr_iodev_sqe->sqe);
		curr_iodev_sqe = rtio_txn_next(curr_iodev_sqe);
	}

	return total_size;
}

static inline const uint8_t *get_sqe_tx_buf(struct rtio_sqe *sqe)
{
	switch (sqe->op) {
	case RTIO_OP_TX:
		return sqe->tx.buf;
	case RTIO_OP_TINY_TX:
		return sqe->tiny_tx.buf;
	case RTIO_OP_TXRX:
		return sqe->txrx.tx_buf;
	default:
		return NULL;
	}
}

static inline uint8_t *get_sqe_rx_buf(struct rtio_sqe *sqe)
{
	switch (sqe->op) {
	case RTIO_OP_RX:
		return sqe->rx.buf;
	case RTIO_OP_TXRX:
		return sqe->txrx.rx_buf;
	default:
		return NULL;
	}
}

static void lpspi_iodev_complete(const struct device *dev, int status);

static inline uint8_t rx_fifo_cur_len(LPSPI_Type *base)
{
	return (base->FSR & LPSPI_FSR_RXCOUNT_MASK) >> LPSPI_FSR_RXCOUNT_SHIFT;
}

static inline uint8_t tx_fifo_cur_len(LPSPI_Type *base)
{
	return (base->FSR & LPSPI_FSR_TXCOUNT_MASK) >> LPSPI_FSR_TXCOUNT_SHIFT;
}

static inline void lpspi_fetch_rx_fifo(const struct device *dev, uint8_t *buf, size_t offset,
				      size_t fetch_len)
{
	LPSPI_Type *base = (LPSPI_Type *)DEVICE_MMIO_NAMED_GET(dev, reg_base);

	for (size_t i = 0 ; i < fetch_len ; i++) {
		buf[offset + i] = (uint8_t)base->RDR;
	}
}

static inline void lpspi_fill_rx_fifo_nop(const struct device *dev, size_t fill_len)
{
	LPSPI_Type *base = (LPSPI_Type *)DEVICE_MMIO_NAMED_GET(dev, reg_base);
	uint32_t unused_word;

	for (size_t i = 0; i < fill_len; i++) {
		unused_word = base->RDR;
	}
}

static bool lpspi_next_rx_fill(const struct device *dev)
{
	LPSPI_Type *base = (LPSPI_Type *)DEVICE_MMIO_NAMED_GET(dev, reg_base);
	struct lpspi_data *data = dev->data;
	struct lpspi_driver_data *lpspi_data = (struct lpspi_driver_data *)data->driver_data;
	int fetch_len = MIN(lpspi_data->total.words_to_clock - lpspi_data->total.words_clocked_rx,
			    rx_fifo_cur_len(base));

	if (fetch_len <= 0) {
		return false;
	}

	int bytes_left = fetch_len;

	do {
		struct rtio_sqe *sqe = lpspi_data->rx_curr.sqe;
		int curr_len = MIN(get_sqe_clock_cycles(sqe) - lpspi_data->rx_curr.words_clocked,
				   rx_fifo_cur_len(base));
		uint8_t *buf = get_sqe_rx_buf(sqe);

		if (buf != NULL) {
			lpspi_fetch_rx_fifo(dev, buf, lpspi_data->rx_curr.words_clocked, curr_len);
		} else {
			lpspi_fill_rx_fifo_nop(dev, curr_len);
		}
		bytes_left -= curr_len;
		lpspi_data->rx_curr.words_clocked += curr_len;

		if (lpspi_data->rx_curr.words_clocked == get_sqe_clock_cycles(sqe)) {
			lpspi_data->rx_curr.sqe = get_next_sqe(sqe);
			lpspi_data->rx_curr.words_clocked = 0;
		}

	} while (bytes_left > 0);

	lpspi_data->total.words_clocked_rx += fetch_len;

	return lpspi_data->total.words_clocked_rx < lpspi_data->total.words_to_clock;
}

static inline void lpspi_fill_tx_fifo(const struct device *dev, const uint8_t *buf, size_t offset,
				      size_t fill_len)
{
	LPSPI_Type *base = (LPSPI_Type *)DEVICE_MMIO_NAMED_GET(dev, reg_base);

	for (size_t i = 0 ; i < fill_len ; i++) {
		base->TDR = (uint32_t)buf[offset + i];
	}
}

static inline void lpspi_fill_tx_fifo_nop(const struct device *dev, size_t fill_len)
{
	LPSPI_Type *base = (LPSPI_Type *)DEVICE_MMIO_NAMED_GET(dev, reg_base);

	for (size_t i = 0; i < fill_len; i++) {
		base->TDR = 0;
	}
}

/* handles refilling the TX fifo from empty */
static bool lpspi_next_tx_fill(const struct device *dev)
{
	const struct lpspi_config *config = dev->config;
	LPSPI_Type *base = (LPSPI_Type *)DEVICE_MMIO_NAMED_GET(dev, reg_base);
	struct lpspi_data *data = dev->data;
	struct lpspi_driver_data *lpspi_data = (struct lpspi_driver_data *)data->driver_data;
	int fill_len = MIN(lpspi_data->total.words_to_clock - lpspi_data->total.words_clocked_tx,
			   config->tx_fifo_size - tx_fifo_cur_len(base));

	if (fill_len <= 0) {
		return false;
	}

	int bytes_left = fill_len;

	do {
		struct rtio_sqe *sqe = lpspi_data->tx_curr.sqe;
		int curr_len = MIN(get_sqe_clock_cycles(sqe) - lpspi_data->tx_curr.words_clocked,
					config->tx_fifo_size - tx_fifo_cur_len(base));
		const uint8_t *buf = get_sqe_tx_buf(sqe);

		if (buf != NULL) {
			lpspi_fill_tx_fifo(dev, buf, lpspi_data->tx_curr.words_clocked, curr_len);
		} else {
			lpspi_fill_tx_fifo_nop(dev, curr_len);
		}
		bytes_left -= curr_len;
		lpspi_data->tx_curr.words_clocked += curr_len;

		if (lpspi_data->tx_curr.words_clocked == get_sqe_clock_cycles(sqe)) {
			lpspi_data->tx_curr.sqe = get_next_sqe(sqe);
			lpspi_data->tx_curr.words_clocked = 0;
		}
	} while (bytes_left > 0);

	lpspi_data->total.words_clocked_tx += fill_len;

	return true;
}

static inline void lpspi_end_xfer(const struct device *dev)
{
	LPSPI_Type *base = (LPSPI_Type *)DEVICE_MMIO_NAMED_GET(dev, reg_base);
	struct lpspi_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;

	if (!(ctx->config->operation & SPI_HOLD_ON_CS)) {
		spi_context_cs_control(&data->ctx, false);
		base->TCR &= ~(LPSPI_TCR_CONT_MASK | LPSPI_TCR_CONTC_MASK);
		/* don't need to wait for TCR since we are at end of xfer + in IRQ context */
	}
}

static void lpspi_isr(const struct device *dev)
{
	LPSPI_Type *base = (LPSPI_Type *)DEVICE_MMIO_NAMED_GET(dev, reg_base);
	uint32_t status_flags = base->SR;

	if (status_flags & LPSPI_SR_RDF_MASK && base->IER & LPSPI_IER_RDIE_MASK) {
		/* Write-to-clear the RX-FIFO WM bit */
		base->SR = LPSPI_SR_RDF_MASK;
		if (!lpspi_next_rx_fill(dev)) {
			/** No more RX events */
			base->IER &= ~LPSPI_IER_RDIE_MASK;
			/* Flush rx fifo */
			base->CR |= LPSPI_CR_RRF_MASK;
		}
	}
	if (status_flags & LPSPI_SR_TDF_MASK && base->IER & LPSPI_IER_TDIE_MASK) {
		/* Write-to-clear the TX-FIFO WM bit */
		base->SR = LPSPI_SR_TDF_MASK;
		if (!lpspi_next_tx_fill(dev)) {
			/** No more TX events */
			base->IER &= ~LPSPI_IER_TDIE_MASK;

			/** We may be waiting on receiving the last chunk of RX data, hence changing
			 * the RX FIFO watermark to trigger on every byte received from now on and
			 * hence prevent leaving data unread.
			 */
			base->FCR = LPSPI_FCR_TXWATER(0) | LPSPI_FCR_RXWATER(0);

			/** Due to stalling behavior on older LPSPI, if we know we already wrote
			 * all the words into the fifo, then we need to end xfer manually by
			 * writing TCR in order to get last bit clocked out on bus. So all we need
			 * to do is touch the TCR by writing to fifo through TCR register and wait
			 * for final RX interrupt.
			 */
			base->TCR = base->TCR;
		}
	}

	if (base->IER == 0) {
		/** We're done both TX and RX as they each clear their Interrupt
		 * enable bit once fully received. The transfer has completed.
		 */
		lpspi_iodev_complete(dev, 0);
	}
}

static void lpspi_master_setup_native_cs(const struct device *dev, const struct spi_config *spi_cfg)
{
	LPSPI_Type *base = (LPSPI_Type *)DEVICE_MMIO_NAMED_GET(dev, reg_base);

	/* keep the chip select asserted until the end of the zephyr xfer by using
	 * continunous transfer mode. If SPI_HOLD_ON_CS is requested, we need
	 * to also set CONTC in order to continue the previous command to keep CS
	 * asserted.
	 */
	if (spi_cfg->operation & SPI_HOLD_ON_CS || base->TCR & LPSPI_TCR_CONTC_MASK) {
		base->TCR |= LPSPI_TCR_CONTC_MASK | LPSPI_TCR_CONT_MASK;
	} else {
		base->TCR |= LPSPI_TCR_CONT_MASK;
	}

	/* tcr is written to tx fifo */
	lpspi_wait_tx_fifo_empty(dev);
}

static void lpspi_iodev_start(const struct device *dev)
{
	LPSPI_Type *base = (LPSPI_Type *)DEVICE_MMIO_NAMED_GET(dev, reg_base);
	const struct lpspi_config *config = dev->config;
	struct lpspi_data *data = dev->data;
	struct lpspi_driver_data *lpspi_data = (struct lpspi_driver_data *)data->driver_data;
	struct spi_rtio *rtio_ctx = lpspi_data->rtio_ctx;
	struct rtio_sqe *sqe = &rtio_ctx->txn_curr->sqe;
	struct spi_dt_spec *spi_dt_spec = sqe->iodev->data;
	struct spi_config *spi_cfg = &spi_dt_spec->config;
	struct spi_context *ctx = &data->ctx;
	uint8_t op_mode = SPI_OP_MODE_GET(spi_cfg->operation);
	int ret = 0;

	lpspi_data->word_size_bytes =
		DIV_ROUND_UP(SPI_WORD_SIZE_GET(spi_cfg->operation), BITS_PER_BYTE);
	if (lpspi_data->word_size_bytes != 1) {
		LOG_ERR("Driver only works with word size = 1 byte");
		ret = -EINVAL;
		goto lpspi_iodev_start_on_error;
	}

	if (op_mode != SPI_OP_MODE_MASTER) {
		LOG_ERR("Target mode not supported for LPSPI RTIO");
		ret = -ENOTSUP;
		goto lpspi_iodev_start_on_error;
	}

	if (data->major_version < 2 && spi_cfg->operation & SPI_HOLD_ON_CS &&
	    !spi_cs_is_gpio(spi_cfg)) {
		/* on this version of LPSPI, due to errata in design
		 * CS must be deasserted in order to clock all words,
		 * so HOLD_ON_CS flag cannot be supported.
		 */
		ret = -EINVAL;
		goto lpspi_iodev_start_on_error;
	}

	lpspi_data->lpspi_op_mode = op_mode;

	ret = lpspi_configure(dev, spi_cfg);
	if (ret) {
		goto lpspi_iodev_start_on_error;
	}

	base->CR |= LPSPI_CR_RRF_MASK;
	base->IER = 0;
	base->SR |= LPSPI_INTERRUPT_BITS;

	size_t max_side_clocks = get_total_sqe_clock_cycles(sqe);

	if (max_side_clocks == 0) {
		ret = -EINVAL;
		goto lpspi_iodev_start_on_error;
	}

	lpspi_data->total.words_to_clock =
				DIV_ROUND_UP(max_side_clocks, lpspi_data->word_size_bytes);
	lpspi_data->total.words_clocked_rx = 0;
	lpspi_data->total.words_clocked_tx = 0;

	lpspi_data->tx_curr.sqe = sqe;
	lpspi_data->tx_curr.words_clocked = 0;

	lpspi_data->rx_curr.sqe = sqe;
	lpspi_data->rx_curr.words_clocked = 0;

	LOG_DBG("Starting LPSPI transfer");
	spi_context_cs_control(ctx, true);
	lpspi_master_setup_native_cs(dev, spi_cfg);

	base->FCR = LPSPI_FCR_TXWATER(0) | LPSPI_FCR_RXWATER(config->rx_fifo_size / 2);
	base->CR |= LPSPI_CR_MEN_MASK;

	/* start the transfer sequence which are handled by irqs */
	(void)lpspi_next_tx_fill(dev);
	base->IER |= LPSPI_IER_TDIE_MASK | LPSPI_IER_RDIE_MASK;
	return;

lpspi_iodev_start_on_error:
	lpspi_iodev_complete(dev, ret);
}

static void lpspi_iodev_complete(const struct device *dev, int status)
{
	const struct lpspi_config *config = dev->config;
	struct lpspi_data *data = dev->data;
	struct lpspi_driver_data *lpspi_data = (struct lpspi_driver_data *)data->driver_data;
	struct spi_rtio *rtio_ctx = lpspi_data->rtio_ctx;

	NVIC_ClearPendingIRQ(config->irqn);
	lpspi_end_xfer(dev);

	if (spi_rtio_complete(rtio_ctx, status)) {
		lpspi_iodev_start(dev);
	}
}

static void lpspi_submit(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	struct lpspi_data *data = (struct lpspi_data *)dev->data;
	struct lpspi_driver_data *drv_data = (struct lpspi_driver_data *)data->driver_data;
	struct spi_rtio *rtio_ctx = drv_data->rtio_ctx;

	if (spi_rtio_submit(rtio_ctx, iodev_sqe)) {
		lpspi_iodev_start(dev);
	}
}

static int transceive_rtio(const struct device *dev, const struct spi_config *spi_cfg,
			   const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs)
{
	struct lpspi_data *data = (struct lpspi_data *)dev->data;
	struct lpspi_driver_data *drv_data = (struct lpspi_driver_data *)data->driver_data;
	struct spi_rtio *rtio_ctx = drv_data->rtio_ctx;
	int ret;

	spi_context_lock(&data->ctx, false, NULL, NULL, spi_cfg);
	ret = spi_rtio_transceive(rtio_ctx, spi_cfg, tx_bufs, rx_bufs);
	spi_context_release(&data->ctx, ret);

	return ret;
}

static int lpspi_init(const struct device *dev)
{
	LPSPI_Type *base = (LPSPI_Type *)DEVICE_MMIO_NAMED_GET(dev, reg_base);
	struct lpspi_data *data = dev->data;
	struct lpspi_driver_data *drv_data = (struct lpspi_driver_data *)data->driver_data;
	struct spi_rtio *rtio_ctx = drv_data->rtio_ctx;
	int err = 0;

	err = spi_nxp_init_common(dev);
	if (err) {
		return err;
	}

	spi_rtio_init(rtio_ctx, dev);

	/* Starting config should be master with active low CS, to make sure
	 * the CS lines are configured properly at init for the most common use
	 * cases. This can be changed later on transceive call if user specifies
	 * different spi configuration.
	 */
	base->CFGR1 |= LPSPI_CFGR1_MASTER_MASK;
	base->CFGR1 &= ~LPSPI_CFGR1_PCSPOL_MASK;

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

#ifdef CONFIG_SPI_ASYNC
static int transceive_rtio_async(const struct device *dev, const struct spi_config *spi_cfg,
				 const struct spi_buf_set *tx_bufs,
				 const struct spi_buf_set *rx_bufs, spi_callback_t cb,
				 void *userdata)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(spi_cfg);
	ARG_UNUSED(tx_bufs);
	ARG_UNUSED(rx_bufs);
	ARG_UNUSED(cb);
	ARG_UNUSED(userdata);

	return -ENOTSUP;
}
#endif

static DEVICE_API(spi, lpspi_driver_api) = {
	.transceive = transceive_rtio,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = transceive_rtio_async,
#endif
	.iodev_submit = lpspi_submit,
	.release = spi_lpspi_release,
};


#define LPSPI_RTIO_INIT(n)									   \
	SPI_NXP_LPSPI_COMMON_INIT(n)								   \
	SPI_LPSPI_CONFIG_INIT(n)								   \
												   \
	BUILD_ASSERT(DT_INST_PROP(n, tx_fifo_size) == DT_INST_PROP(n, rx_fifo_size),		   \
		     "tx-fifo-size and rx-fifo-size must match for the RTIO SPI driver "	   \
		     "to work. Please make them equal.");					   \
												   \
	SPI_RTIO_DEFINE(spi_nxp_rtio_##n, CONFIG_SPI_NXP_RTIO_SQ_SIZE,				   \
			CONFIG_SPI_NXP_RTIO_SQ_SIZE);						   \
												   \
												   \
	static struct lpspi_driver_data lpspi_##n##_driver_data = {				   \
		.rtio_ctx = &spi_nxp_rtio_##n,							   \
	};											   \
												   \
	static struct lpspi_data lpspi_data_##n = {						   \
		SPI_NXP_LPSPI_COMMON_DATA_INIT(n)						   \
		.driver_data = &lpspi_##n##_driver_data,					   \
	};											   \
												   \
	SPI_DEVICE_DT_INST_DEFINE(n, lpspi_init, NULL, &lpspi_data_##n,				   \
				  &lpspi_config_##n, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,	   \
				  &lpspi_driver_api);

DT_INST_FOREACH_STATUS_OKAY(LPSPI_RTIO_INIT)
