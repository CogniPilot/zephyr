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
	struct {
		struct rtio_sqe *sqe;
		size_t words_clocked;
	} tx_curr;
	struct {
		size_t words_to_clock;
		size_t words_clocked_tx;
		size_t words_to_clock_rx;
	} total;
	struct {
		struct rtio_sqe *sqe;
		size_t words_clocked;
	} rx_curr;
	uint32_t word_size_bytes;
};

static inline size_t get_sqe_words_len(struct rtio_sqe *sqe)
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
	case RTIO_OP_NANO_TXRX:
		return sqe->nano_txrx.buf_len;
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

static inline void set_sqe_words_to_clock(struct rtio_sqe *head, struct lpspi_driver_data *data)
{
	struct rtio_iodev_sqe *curr_iodev_sqe = CONTAINER_OF(head, struct rtio_iodev_sqe, sqe);
	data->total.words_to_clock = 0;
	data->total.words_to_clock_rx = 0;
	data->total.words_clocked_tx = 0;

	while (curr_iodev_sqe != NULL) {
		switch (curr_iodev_sqe->sqe.op) {
			case RTIO_OP_RX:
				data->total.words_to_clock += curr_iodev_sqe->sqe.rx.buf_len;
				data->total.words_to_clock_rx += curr_iodev_sqe->sqe.rx.buf_len;
				break;
			case RTIO_OP_TX:
				data->total.words_to_clock += curr_iodev_sqe->sqe.tx.buf_len;
				break;
			case RTIO_OP_TINY_TX:
				data->total.words_to_clock += curr_iodev_sqe->sqe.tiny_tx.buf_len;
				break;
			case RTIO_OP_TXRX:
				data->total.words_to_clock += curr_iodev_sqe->sqe.txrx.buf_len;
				data->total.words_to_clock_rx += curr_iodev_sqe->sqe.txrx.buf_len;
				break;
			case RTIO_OP_NANO_TXRX:
				data->total.words_to_clock += curr_iodev_sqe->sqe.nano_txrx.buf_len;
				data->total.words_to_clock_rx += curr_iodev_sqe->sqe.nano_txrx.buf_len;
				break;
			default:
				break;
			}
		curr_iodev_sqe = rtio_txn_next(curr_iodev_sqe);
	}

	data->total.words_to_clock =
				DIV_ROUND_UP(data->total.words_to_clock, data->word_size_bytes);
	data->total.words_to_clock_rx =
				DIV_ROUND_UP(data->total.words_to_clock_rx, data->word_size_bytes);
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
	case RTIO_OP_NANO_TXRX:
		return sqe->nano_txrx.tx;
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
	case RTIO_OP_NANO_TXRX:
		return sqe->nano_txrx.rx_buf;
	default:
		return NULL;
	}
}

static void lpspi_rtio_iodev_complete(const struct device *dev, int status);

static inline void lpspi_rtio_fetch_rx_fifo(LPSPI_Type *base, uint8_t *buf, size_t offset,
					    size_t fetch_len)
{
	uint8_t *p = buf + offset;

	for (size_t i = 0 ; i < fetch_len ; i++) {
		*p++ = (uint8_t)base->RDR;
	}
}

static inline void lpspi_rtio_empty_rx_fifo_nop(LPSPI_Type *base, size_t fill_len)
{
	for (size_t i = 0; i < fill_len; i++) {
		(void)base->RDR; // read-and-discard
	}
}

static inline bool lpspi_rtio_next_rx_fetch(const struct device *dev)
{
	LPSPI_Type *base = (LPSPI_Type *)DEVICE_MMIO_NAMED_GET(dev, reg_base);
	struct lpspi_data *data = dev->data;
	struct lpspi_driver_data *lpspi_data = (struct lpspi_driver_data *)data->driver_data;
	uint32_t fetch_len = rx_fifo_cur_len(base);

	if (fetch_len == 0) {
		return true;
	}

	int bytes_left = fetch_len;

	while (bytes_left > 0 && lpspi_data->rx_curr.sqe) {
		struct rtio_sqe *sqe = lpspi_data->rx_curr.sqe;
		size_t rx_sqe_len = get_sqe_words_len(sqe);
		int curr_len = MIN(rx_sqe_len - lpspi_data->rx_curr.words_clocked,
				   bytes_left);
		uint8_t *buf = get_sqe_rx_buf(sqe);

		if (buf != NULL) {
			lpspi_rtio_fetch_rx_fifo(base, buf, lpspi_data->rx_curr.words_clocked,
						 curr_len);
			lpspi_data->total.words_to_clock_rx -= curr_len;
		} else {
			lpspi_rtio_empty_rx_fifo_nop(base, curr_len);
		}
		bytes_left -= curr_len;
		lpspi_data->rx_curr.words_clocked += curr_len;

		if (lpspi_data->rx_curr.words_clocked >= rx_sqe_len) {
			lpspi_data->rx_curr.sqe = get_next_sqe(sqe);
			lpspi_data->rx_curr.words_clocked = 0;
		}
	}
	if (bytes_left > 0) {
		LOG_WRN("rx returned with bytes_left: %d - fetch_len: %d", bytes_left, fetch_len);
	}

	return lpspi_data->total.words_to_clock_rx != 0;
}

static inline void lpspi_rtio_fill_tx_fifo(LPSPI_Type *base, const uint8_t *buf,
					   size_t offset, size_t fill_len)
{
	const uint8_t *p = buf + offset;

	for (size_t i = 0; i < fill_len; i++) {
		base->TDR = (uint32_t)*p++;
	}
}

static inline void lpspi_rtio_fill_tx_fifo_nop(LPSPI_Type *base, size_t fill_len)
{
	for (size_t i = 0; i < fill_len; i++) {
		base->TDR = 0;
	}
}

/* handles refilling the TX fifo from empty */
static inline bool lpspi_rtio_next_tx_fill(const struct device *dev)
{
	const struct lpspi_config *config = dev->config;
	LPSPI_Type *base = (LPSPI_Type *)DEVICE_MMIO_NAMED_GET(dev, reg_base);
	struct lpspi_data *data = dev->data;
	struct lpspi_driver_data *lpspi_data = (struct lpspi_driver_data *)data->driver_data;
	int fifo_remaining_len = config->tx_fifo_size - tx_fifo_cur_len(base);
	int fill_len = MIN(lpspi_data->total.words_to_clock - lpspi_data->total.words_clocked_tx, fifo_remaining_len);

	if (fill_len <= 0) {
		return false;
	}

	volatile int bytes_left = fill_len;

	while (bytes_left > 0 && lpspi_data->tx_curr.sqe) {
		struct rtio_sqe *sqe = lpspi_data->tx_curr.sqe;
		int curr_len = MIN(get_sqe_words_len(sqe) - lpspi_data->tx_curr.words_clocked,
				   bytes_left);
		const uint8_t *buf = get_sqe_tx_buf(sqe);

		if (buf != NULL) {
			lpspi_rtio_fill_tx_fifo(base, buf, lpspi_data->tx_curr.words_clocked,
						curr_len);
		} else {
			lpspi_rtio_fill_tx_fifo_nop(base, curr_len);
		}
		bytes_left -= curr_len;
		lpspi_data->tx_curr.words_clocked += curr_len;

		if (lpspi_data->tx_curr.words_clocked >= get_sqe_words_len(sqe)) {
			lpspi_data->tx_curr.sqe = get_next_sqe(sqe);
			lpspi_data->tx_curr.words_clocked = 0;
		}
	}
	lpspi_data->total.words_clocked_tx += fill_len - bytes_left;

	if (bytes_left > 0) {
		LOG_WRN("tx returned with bytes_left: %d - fifo_remaining: %d Curr Words clocked %d %d/%d", bytes_left,
			fifo_remaining_len, lpspi_data->tx_curr.words_clocked, lpspi_data->total.words_clocked_tx, lpspi_data->total.words_to_clock);
	}


	if(lpspi_data->total.words_to_clock == lpspi_data->total.words_clocked_tx) {
		base->TCR &= ~(LPSPI_TCR_CONT_MASK | LPSPI_TCR_CONTC_MASK);
	}

	return true;
}

// EMC_B1_02 GPIO1 02
// 0x4012C000
#include <fsl_gpio.h>

static void lpspi_isr(const struct device *dev)
{
	GPIO_PinWrite((GPIO_Type *)0x4012C000, 2, 1);
	LPSPI_Type *base = (LPSPI_Type *)DEVICE_MMIO_NAMED_GET(dev, reg_base);
	const struct lpspi_config *config = dev->config;
	struct lpspi_data *data = dev->data;
	struct lpspi_driver_data *lpspi_data = (struct lpspi_driver_data *)data->driver_data;
	uint32_t status_flags = base->SR;
	uint32_t irq_enable = base->IER;

	irq_enable &= ~(LPSPI_IER_RDIE_MASK | LPSPI_IER_TDIE_MASK);
	
	if (lpspi_data->total.words_to_clock_rx > 0) {
		if(lpspi_rtio_next_rx_fetch(dev)) {
			irq_enable |= LPSPI_IER_RDIE_MASK;
			if(lpspi_data->total.words_to_clock_rx < config->rx_fifo_size) {
				base->FCR = LPSPI_FCR_TXWATER(0) | LPSPI_FCR_RXWATER(lpspi_data->total.words_to_clock_rx - 1);
			}
		}
	}

	if (lpspi_data->total.words_to_clock != lpspi_data->total.words_clocked_tx) {
		if(lpspi_rtio_next_tx_fill(dev)) {
			irq_enable |= LPSPI_IER_TDIE_MASK;
		}
	}

	if (status_flags & LPSPI_SR_TCF_MASK) {
		irq_enable &= ~LPSPI_IER_TCIE_MASK;
	}

	if (status_flags & LPSPI_SR_REF_MASK) {
	 	base->IER = 0;
	 	lpspi_rtio_iodev_complete(dev, -EIO);
	} else {
		base->IER = irq_enable;

		if (irq_enable == 0) {
			/** Due to stalling behavior on older LPSPI, if we know we already wrote
			 * all the words into the fifo, then we need to end xfer manually by
			 * writing TCR in order to get last bit clocked out on bus. So all we need
			 * to do is touch the TCR by writing to fifo through TCR register and wait
			 * for final RX interrupt.
			 */
			base->TCR = base->TCR;

			/** We're done both TX and RX as they each clear their Interrupt
			 * enable bit once fully received. The transfer has completed.
			 */
			lpspi_rtio_iodev_complete(dev, 0);
		}
	}
	GPIO_PinWrite((GPIO_Type *)0x4012C000, 2, 0);
}

static void lpspi_rtio_iodev_start(const struct device *dev)
{
	LPSPI_Type *base = (LPSPI_Type *)DEVICE_MMIO_NAMED_GET(dev, reg_base);
	const struct lpspi_config *config = dev->config;
	struct lpspi_data *data = dev->data;
	struct lpspi_driver_data *lpspi_data = (struct lpspi_driver_data *)data->driver_data;
	struct spi_rtio *rtio_ctx = lpspi_data->rtio_ctx;
	struct rtio_sqe *sqe = &rtio_ctx->txn_head->sqe;
	struct spi_dt_spec *spi_dt_spec = sqe->iodev->data;
	struct spi_config *spi_cfg = &spi_dt_spec->config;
	int ret = 0;

	lpspi_data->word_size_bytes =
		DIV_ROUND_UP(SPI_WORD_SIZE_GET(spi_cfg->operation), BITS_PER_BYTE);
	if (lpspi_data->word_size_bytes != 1) {
		LOG_ERR("Driver only works with word size = 1 byte");
		ret = -EINVAL;
		goto lpspi_rtio_iodev_start_on_error;
	}

	if (SPI_OP_MODE_GET(spi_cfg->operation) != SPI_OP_MODE_MASTER) {
		LOG_WRN("Target mode not supported for LPSPI RTIO");
		ret = -ENOTSUP;
		goto lpspi_rtio_iodev_start_on_error;
	}

	if (spi_cfg->operation & SPI_HOLD_ON_CS && !spi_cs_is_gpio(spi_cfg)) {
		ret = -ENOTSUP;
		goto lpspi_rtio_iodev_start_on_error;
	}

	ret = lpspi_configure(dev, spi_cfg);
	if (ret) {
		goto lpspi_rtio_iodev_start_on_error;
	}

	base->CR |= LPSPI_CR_RRF_MASK | LPSPI_CR_RTF_MASK;
	base->SR = LPSPI_INTERRUPT_BITS;

	set_sqe_words_to_clock(sqe, lpspi_data);

	if (lpspi_data->total.words_to_clock == 0) {
		ret = -EINVAL;
		goto lpspi_rtio_iodev_start_on_error;
	}

	lpspi_data->tx_curr.sqe = sqe;
	lpspi_data->tx_curr.words_clocked = 0;

	lpspi_data->rx_curr.sqe = sqe;
	lpspi_data->rx_curr.words_clocked = 0;

	base->TCR = (base->TCR & ~(LPSPI_TCR_PCS_MASK | LPSPI_TCR_RXMSK_MASK)) |
		    LPSPI_TCR_PCS(spi_cfg->slave) |
		    LPSPI_TCR_CONT_MASK;
	spi_context_cs_control(&data->ctx, true);

	/* tcr is written to tx fifo */
	//lpspi_wait_tx_fifo_empty(dev);

	if(lpspi_data->total.words_to_clock < config->rx_fifo_size) {
		base->FCR = LPSPI_FCR_TXWATER(0) | LPSPI_FCR_RXWATER(lpspi_data->total.words_to_clock - 1);
	} else {
		base->FCR = LPSPI_FCR_TXWATER(0) | LPSPI_FCR_RXWATER(config->rx_fifo_size / 2);
	}
	//LOG_ERR("Payload size %d", lpspi_data->total.words_to_clock);

	base->CR = LPSPI_CR_MEN_MASK;

	/* start the transfer sequence which are handled by irqs */
	if (lpspi_rtio_next_tx_fill(dev) == false) {
		ret = -EINVAL;
		LOG_ERR("LPSPI FILL ERROR");
		goto lpspi_rtio_iodev_start_on_error;
	}

	if(sqe->op == RTIO_OP_NANO_TXRX) {
		while(!(base->SR & LPSPI_SR_RDF_MASK));
		if(sqe->nano_txrx.rx_skip > 0) {
			lpspi_rtio_empty_rx_fifo_nop(base, sqe->nano_txrx.rx_skip);
		}
		lpspi_rtio_fetch_rx_fifo(base, get_sqe_rx_buf(sqe), 0, sqe->nano_txrx.buf_len - sqe->nano_txrx.rx_skip);
		spi_rtio_complete(lpspi_data->rtio_ctx, 0);
	} else {
		if(lpspi_data->total.words_to_clock_rx > 0) {
			base->IER = LPSPI_IER_RDIE_MASK | LPSPI_IER_TCIE_MASK;
		} else {
			base->TCR |= LPSPI_TCR_RXMSK_MASK;
			base->IER = LPSPI_IER_TDIE_MASK | LPSPI_IER_TCIE_MASK;
		}
	}
	return;

lpspi_rtio_iodev_start_on_error:
	lpspi_rtio_iodev_complete(dev, ret);
}

static void lpspi_rtio_iodev_complete(const struct device *dev, int status)
{
	const struct lpspi_config *config = dev->config;
	struct lpspi_data *data = dev->data;
	struct lpspi_driver_data *lpspi_data = (struct lpspi_driver_data *)data->driver_data;
	struct spi_rtio *rtio_ctx = lpspi_data->rtio_ctx;
	struct spi_context *ctx = &data->ctx;

	NVIC_ClearPendingIRQ(config->irqn);

	if (!(ctx->config->operation & SPI_HOLD_ON_CS)) {
		spi_context_cs_control(&data->ctx, false);
	}

	/* don't need to wait for TCR since we are at end of xfer + in IRQ context */

	if (spi_rtio_complete(rtio_ctx, status)) {
		lpspi_rtio_iodev_start(dev);
	}
}

static void lpspi_rtio_submit(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	struct lpspi_data *data = (struct lpspi_data *)dev->data;
	struct lpspi_driver_data *drv_data = (struct lpspi_driver_data *)data->driver_data;
	struct spi_rtio *rtio_ctx = drv_data->rtio_ctx;

	if (spi_rtio_submit(rtio_ctx, iodev_sqe)) {
		lpspi_rtio_iodev_start(dev);
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

static int lpspi_rtio_init(const struct device *dev)
{
	struct lpspi_data *data = dev->data;
	struct lpspi_driver_data *drv_data = (struct lpspi_driver_data *)data->driver_data;
	struct spi_rtio *rtio_ctx = drv_data->rtio_ctx;
	int err = 0;

	((GPIO_Type *)0x4012C000)->GDIR |= (1 << 2);
	((GPIO_Type *)0x4012C000)->GDIR |= (1 << 0);

	err = spi_nxp_init_common(dev);
	if (err) {
		return err;
	}

	spi_rtio_init(rtio_ctx, dev);

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
	.iodev_submit = lpspi_rtio_submit,
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
	SPI_DEVICE_DT_INST_DEFINE(n, lpspi_rtio_init, NULL, &lpspi_data_##n,			   \
				  &lpspi_config_##n, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,	   \
				  &lpspi_driver_api);

DT_INST_FOREACH_STATUS_OKAY(LPSPI_RTIO_INIT)
