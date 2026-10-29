/*
 * Copyright 2026 NDR Solution (Thailand) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Continuous SPI slave RX streaming using LPSPI + eDMA circular buffer.
 *
 * Design summary
 * --------------
 * spi_read_stream_async_dt() configures:
 *   1. LPSPI in slave mode with TCR.TXMSK=1 (RX-only, no TX output).
 *   2. FCR.RXWATER=0 (DMA request per word).
 *   3. IER.FCIE=1  (interrupt on Frame Complete Flag = CS deassertion).
 *   4. eDMA channel (dma_config.cyclic=1) from LPSPI_RDR into ring_buf.
 *      DLAST = -(int32_t)ring_buf_size wraps destination automatically.
 *   5. DER.RDDE=1  (RX DMA request enable).
 *
 * On each CS deassertion lpspi_isr() fires FCIE -> calls
 * lpspi_stream_isr_fcf_handler() which:
 *   - Records the frame start/length into a pool descriptor (no alloc).
 *   - Posts descriptor to cfg->frame_fifo (k_fifo_put, ISR-safe).
 *   - Advances write_pos by frame_size (software ring pointer).
 *
 * The application thread drains cfg->frame_fifo with k_fifo_get() and
 * reads directly from the ring buffer — zero CPU copy.
 *
 * Restrictions
 * ------------
 *   - Peripheral mode only.
 *   - Fixed frame_size per streaming session.
 *   - ring_buf_size must be a multiple of frame_size, >= (2 * frame_size).
 *   - LPSPI DTS node must have "dmas" property with an "rx" entry.
 *   - CONFIG_SPI_NXP_LPSPI_DMA must be enabled (DMA driver wired).
 *   - Concurrent spi_transceive() on the same device is rejected with EBUSY.
 */

#define DT_DRV_COMPAT nxp_lpspi

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(spi_lpspi, CONFIG_SPI_LOG_LEVEL);

#include <zephyr/sys/slist.h>
#include "spi_nxp_lpspi_priv.h"

/*
 * S32K358_DMA_TCD.h (included transitively via S32K358.h) defines:
 *   IP_TCD_BASE = 0x40210000  — base of the eDMA TCD register region
 * Each TCD channel occupies 0x4000 bytes; TCD_DADDR resides at offset +0x30.
 * @see 15.6.2.13 TCD Destination Address (TCD0_DADDR - TCD31_DADDR)
 */
#define LPSPI_STREAM_TCD_STRIDE     (0x4000U)   /* bytes between adjacent TCD channels  */
#define LPSPI_STREAM_TCD_DADDR_OFF  (0x030U)    /* offset of DADDR within one TCD channel */

static inline LPSPI_Type* lpspi_stream_base(const struct device* dev) {
    LPSPI_Type* lpspi = (LPSPI_Type*)DEVICE_MMIO_NAMED_GET(dev, reg_base);

    return (lpspi);
}

static inline volatile uint32_t* lpspi_stream_get_tcd_daddr_reg(uint32_t dma_channel) {
    return (volatile uint32_t*)(IP_TCD_BASE +
                                ((dma_channel * LPSPI_STREAM_TCD_STRIDE) + LPSPI_STREAM_TCD_DADDR_OFF));
}

/* Resolves the stream data block for a dt-spec, without exposing dev/lpspi_data to
 * callers that only need read-only access to it (the counter accessors below). */
static inline struct spi_nxp_stream_data const* lpspi_stream_data_dt(struct spi_dt_spec const* spec) {
    struct device const* dev = spec->bus;
    struct lpspi_data const* data = dev->data;

    return (data->stream);
}

static int lpspi_stream_validate_args(struct spi_nxp_stream_data const* stream,
                                      struct spi_dt_spec const* spec,
                                      const struct spi_stream_config* cfg) {
    if (stream == NULL) {
        LOG_ERR("stream: CONFIG_SPI_NXP_LPSPI_STREAM not enabled or stream_data NULL");
        return (-ENODEV);
    }

    if ((cfg == NULL) ||
        (cfg->ring_buf == 0U)     || (cfg->frame_pool  == NULL) ||
        (cfg->frame_fifo == NULL) || (cfg->ring_buf_size == 0U) ||
        (cfg->frame_size == 0U)   || (cfg->frame_pool_count == 0U)) {
        return (-EINVAL);
    }

    if ((cfg->ring_buf_size % cfg->frame_size) != 0U) {
        LOG_ERR("stream: ring_buf_size must be a multiple of frame_size");
        return (-EINVAL);
    }

    if (cfg->ring_buf_size < (2U * cfg->frame_size)) {
        LOG_ERR("stream: ring_buf_size must be >= 2 × frame_size");
        return (-EINVAL);
    }

    if (cfg->frame_pool_count < (cfg->ring_buf_size / cfg->frame_size)) {
        LOG_ERR("stream: frame_pool_count must be >= ring_buf_size / frame_size");
        return (-EINVAL);
    }

    if ((spec->config.operation & SPI_OP_MODE_PERIPHERAL) == 0U) {
        LOG_ERR("stream: peripheral mode (SPI_OP_MODE_PERIPHERAL) required");
        return (-EINVAL);
    }

    return (0);
}

/**
 * No-op DMA callback for the streaming RX channel.
 *
 * dma_mcux_edma.c invokes dma_callback after each major loop. A NULL callback
 * causes a null pointer dereference. The callback absorbs periodic major-loop
 * interrupts since frame notification is handled by FCF.
 */
static void lpspi_stream_dma_callback(const struct device* dma_dev,
                                      void* user_data,
                                      uint32_t channel, int status) {
    ARG_UNUSED(dma_dev);
    ARG_UNUSED(user_data);
    ARG_UNUSED(channel);
    ARG_UNUSED(status);

    /* Intentionally empty: FCF ISR owns frame notification */
}

/**
 * Configure the RX DMA channel for cyclic (circular) operation.
 *
 * Source : fixed at &lpspi->RDR (peripheral, no address increment).
 * Dest   : ring_buf base with automatic wrap (DLAST = -ring_buf_size).
 * Trigger: LPSPI RDDE DMA request (one minor loop per received word).
 *
 * Circular wrap is implemented by EDMA_PrepareTransfer() for
 * kEDMA_PeripheralToMemory: it sets DLAST_SGA = -block_size and SLAST_SDA = 0,
 * so the destination address wraps to ring_buf[0] after each major loop while
 * the source (RDR) remains fixed.  DREQ is left 0 (don't auto-stop) for
 * peripheral transfers, so the channel restarts immediately.
 *
 * INTMAJOR is set by EDMA_PrepareTransfer (kEDMA_MajorInterruptEnable).
 * lpspi_stream_dma_callback() is registered to absorb those interrupts safely
 * without performing any work — FCF/FCIE is the actual frame notification path.
 *
 * source_data_size and source_burst_length are derived from the SPI word size
 * (SPI_WORD_SIZE_GET) rather than inherited from the normal RX DMA config.
 * This ensures the TCD is valid even before any spi_transceive() has run,
 * and guarantees NBYTES == SSIZE (eDMA requirement).
 */
static int lpspi_stream_dma_configure(struct spi_dt_spec const* spec) {
    struct device const* dev = spec->bus;
    LPSPI_Type* lpspi = lpspi_stream_base(dev);
    struct lpspi_data* data = dev->data;
    struct spi_nxp_stream_data* stream = data->stream;
    struct spi_stream_config const* cfg = stream->cfg;
    struct spi_nxp_dma_data* dma_data = (struct spi_nxp_dma_data*)data->driver_data;
    int rc;

    /* ---- Block config ---- */
    struct spi_dma_stream* dma_rx = &dma_data->dma_rx;
    struct dma_block_config* blk_cfg = &dma_rx->dma_blk_cfg;

    (void) memset(blk_cfg, 0, sizeof(struct dma_block_config));
    blk_cfg->source_address   = (uintptr_t)&lpspi->RDR;
    blk_cfg->dest_address     = cfg->ring_buf;
    blk_cfg->block_size       = cfg->ring_buf_size;
    blk_cfg->source_addr_adj  = DMA_ADDR_ADJ_NO_CHANGE;
    blk_cfg->dest_addr_adj    = DMA_ADDR_ADJ_INCREMENT;

    /*
     * Circular wrap needs DLAST_SGA = -ring_buf_size.
     * EDMA_PrepareTransfer() leaves it 0;
     * dma_mcux_edma_patch_basic_cyclic_tcd() installs it (= -block_size for an INCREMENT dest)
     * ONLY when dest_reload_en != 0.
     * So this MUST be 1 — cleared, the buffer never wraps and RX corrupts past the boundary.
     */
    blk_cfg->dest_reload_en   = 1U;
    blk_cfg->source_reload_en = 0U;

    /* ---- Channel config: streaming-specific overrides ---- */
    struct dma_config* dma_cfg = &dma_rx->dma_cfg;
    dma_cfg->channel_direction    = PERIPHERAL_TO_MEMORY;
    dma_cfg->cyclic               = 1U;
    dma_cfg->complete_callback_en = 0U;
    dma_cfg->head_block           = blk_cfg;

    /* Must be non-NULL: nxp_edma_callback calls dma_callback unconditionally. */
    dma_cfg->dma_callback = lpspi_stream_dma_callback;
    dma_cfg->user_data    = (void*)dev;

    rc = dma_config(dma_rx->dma_dev, dma_rx->channel, dma_cfg);
    if (rc == 0) {
        /*
         * Cache the eDMA TCD_DADDR register address for ISR-context spurious-FCF detection.
         *
         * S32K358 eDMA TCD layout (from S32K358_DMA_TCD.h, IP_TCD_BASE = 0x40210000):
         *   channel N:  base_addr = IP_TCD_BASE + N * LPSPI_STREAM_TCD_STRIDE
         *   TCD_DADDR:  base_addr + LPSPI_STREAM_TCD_DADDR_OFF
         *
         * This assumes channel < 12 (no channel-gap adjustment required on S32K358).
         * Channels used for LPSPI streaming (ch4-7) satisfy this constraint.
         * Reading DADDR in the ISR is safe — MMIO register, no locking required.
         */
        stream->dma_daddr_reg = lpspi_stream_get_tcd_daddr_reg(dma_rx->channel);
    }

    return (rc);
}

/* -------------------------------------------------------------------------
 * ISR-context FCF handler (called from lpspi_isr in spi_nxp_lpspi_dma.c)
 * ------------------------------------------------------------------------- */
/**
 * lpspi_stream_isr_fcf_handler() - Handle Frame Complete interrupt.
 *
 * Runs at interrupt priority: must not block, allocate, or call a sleeping
 * Zephyr API. Execution time is bounded by the number of frames coalesced
 * into this interrupt (normally one).
 *
 * FCF is a single status bit, so two CS deassertions landing before (or
 * while) this ISR runs raise it once — one interrupt can cover several
 * received frames. Publishing a fixed one frame per interrupt would leave
 * write_pos permanently one frame behind DADDR: no data is lost (neither
 * guard below trips) but every later FCF then publishes the previous frame,
 * withholding the newest frame of each burst indefinitely. Measuring the
 * DMA/write_pos distance instead keeps write_pos locked to the hardware, so
 * a coalesced interrupt is absorbed rather than latching a permanent lag.
 *
 * Overrun guard: desc->in_fifo is set by this ISR at publish and cleared by
 * the consumer via spi_stream_frame_release(). (A prior version guarded with
 * sys_slist_peek_next(&desc->node) != NULL, which is NULL for the tail/sole
 * fifo entry regardless of link state — blind to the common one-pending-slot
 * case, and would corrupt the fifo by re-queuing an already-linked node.)
 *
 * write_pos and desc_pool_head advance only in the publish branch; on
 * overrun both are left unchanged so the same ring region is re-offered on
 * the next FCF (the DMA will have overwritten it by then, but that is
 * unavoidable without a copy).
 */
void lpspi_stream_isr_fcf_handler(const struct device* dev) {
    struct lpspi_data* data = dev->data;
    struct spi_nxp_stream_data* stream = data->stream;
    const struct spi_stream_config* cfg = stream->cfg;
    uint32_t dma_pos;
    uint32_t pending;
    uint32_t n_frames;

    /*
     * DADDR reflects where DMA writes the NEXT byte; normalised to a ring-buffer
     * offset it equals how far DMA has written.  Both operands below are already
     * in [0, ring_buf_size), so the wrap case is a plain two-term sum.
     */
    dma_pos = (*stream->dma_daddr_reg - (uint32_t)cfg->ring_buf) %
              (uint32_t)cfg->ring_buf_size;

    if (dma_pos >= stream->write_pos) {
        pending = dma_pos - stream->write_pos;
    }
    else {
        pending = ((uint32_t)cfg->ring_buf_size - stream->write_pos) + dma_pos;
    }

    /*
     * Whole frames only.  A partial tail means the controller has already started
     * clocking the next frame; its own FCF is still to come, and publishing it now
     * would hand the consumer a frame that is still being written.  Rounding down
     * is also the safe direction: write_pos stays put on anything not published, so
     * the next interrupt picks it up.  Under-counting self-corrects, over-counting
     * does not.
     */
    n_frames = pending / (uint32_t)cfg->frame_size;

    /* Spurious-FCF guard: CS deasserted without completing a frame */
    if (n_frames == 0U) {
        stream->spurious_count++;
        return;
    }

    if (n_frames > 1U) {
        stream->coalesced_count += (n_frames - 1U);
    }

    for (uint32_t frame = 0U; frame < n_frames; frame++) {
        /* Snapshot frame_start and peek pool slot before any mutation */
        uint32_t frame_start = stream->write_pos;
        uint32_t pool_idx    = stream->desc_pool_head % (uint32_t)cfg->frame_pool_count;
        struct spi_stream_frame* desc = &cfg->frame_pool[pool_idx];
        atomic_val_t in_fifo;

        /* Overrun guard: slot still held by consumer */
        in_fifo = atomic_get(&desc->in_fifo);
        if (in_fifo != 0) {
            stream->overrun_count++;
            break;
        }

        /* Safe to publish - advance pointers and post descriptor */
        stream->write_pos = (frame_start + (uint32_t)cfg->frame_size) %
                            (uint32_t)cfg->ring_buf_size;
        stream->desc_pool_head++;

        desc->data = cfg->ring_buf + frame_start;
        desc->len  = cfg->frame_size;

        (void) atomic_set(&desc->in_fifo, 1);
        k_fifo_put(cfg->frame_fifo, desc);
    }
}

int spi_read_stream_async_dt(struct spi_dt_spec const* spec, const struct spi_stream_config* cfg) {
    struct device const* dev = spec->bus;
    LPSPI_Type* lpspi = lpspi_stream_base(dev);
    struct lpspi_data* data = dev->data;
    struct spi_nxp_stream_data* stream = data->stream;
    struct spi_nxp_dma_data const* dma_data = (struct spi_nxp_dma_data const*)data->driver_data;
    int ret;

    ret = lpspi_stream_validate_args(stream, spec, cfg);
    if (ret != 0) {
        return (ret);
    }

    /* Initialise stream state */
    stream->cfg             = cfg;
    stream->write_pos       = 0U;
    stream->desc_pool_head  = 0U;
    stream->overrun_count   = 0U;
    stream->spurious_count  = 0U;
    stream->coalesced_count = 0U;
    /* stream->dma_daddr_reg is set by lpspi_stream_dma_configure() below */

    /* Reset in_fifo for all pool slots - required on restart after stop,
     * where slots may still be marked in-use from the previous session. */
    for (size_t i = 0U; i < cfg->frame_pool_count; i++) {
        (void) atomic_set(&cfg->frame_pool[i].in_fifo, 0);
    }

    /* Configure LPSPI hardware for peripheral RX-only streaming */
    ret = lpspi_configure(dev, &spec->config);
    if (ret == 0) {
        /* 3-state MISO — slave does not drive output in RX-only stream mode.
        * TXMSK auto-clear only applies in Controller mode; in Peripheral mode
        * the bit stays set until software clears it (see RM TXMSK description).
        */
        lpspi->TCR |= LPSPI_TCR_TXMSK_MASK;

        /* RXWATER=0: DMA request fires per received word (maximum granularity) */
        lpspi->FCR = LPSPI_FCR_RXWATER(0U);

        ret = lpspi_stream_dma_configure(spec);
        if (ret == 0) {
            /* Start DMA — runs continuously; never stopped between frames */
            ret = dma_start(dma_data->dma_rx.dma_dev, dma_data->dma_rx.channel);
            if (ret == 0) {
                /* Enable LPSPI module */
                lpspi->CR |= LPSPI_CR_MEN_MASK;

                /* Enable RX DMA request and Frame Complete interrupt (order matters) */
                lpspi->DER |= LPSPI_DER_RDDE_MASK;
                lpspi->SR   = LPSPI_SR_FCF_MASK;        /* Clear any stale FCF before enabling IRQ */
                lpspi->IER |= LPSPI_IER_FCIE_MASK;

                LOG_INF("stream: started on %s — ring=%u B, frame=%u B, pool=%u entries",
                        dev->name, (unsigned)cfg->ring_buf_size,
                        (unsigned)cfg->frame_size, (unsigned)cfg->frame_pool_count);
            }
        }
    }

    if (ret != 0) {
        LOG_ERR("stream: init failed (%d)", ret);
    }

    return (ret);
}

int spi_stream_stop_dt(struct spi_dt_spec const* spec) {
    struct device const* dev = spec->bus;
    LPSPI_Type* lpspi;
    struct lpspi_data* data = dev->data;
    struct spi_nxp_stream_data* stream = data->stream;
    struct spi_nxp_dma_data const* dma_data;
    int ret;

    if (stream == NULL) {
        return (-ENODEV);
    }

    lpspi = lpspi_stream_base(dev);

    /* Disable interrupt and DMA request first to avoid stray callbacks */
    lpspi->IER &= ~LPSPI_IER_FCIE_MASK;
    lpspi->DER &= ~LPSPI_DER_RDDE_MASK;

    /* Clear TXMSK — does not auto-clear in Peripheral mode */
    lpspi->TCR &= ~LPSPI_TCR_TXMSK_MASK;

    dma_data = (struct spi_nxp_dma_data const*)data->driver_data;
    if (dma_data != NULL) {
        ret = dma_stop(dma_data->dma_rx.dma_dev, dma_data->dma_rx.channel);
        if (ret != 0) {
            LOG_WRN("stream: dma_stop returned %d", ret);
        }
    }

    stream->cfg = NULL;

    LOG_INF("stream: stopped on %s (overruns=%u)",
            dev->name, stream->overrun_count);

    return (0);
}

uint32_t spi_stream_overrun_count_dt(struct spi_dt_spec const* spec) {
    struct spi_nxp_stream_data const* stream = lpspi_stream_data_dt(spec);
    uint32_t count = 0U;

    if (stream != NULL) {
        count = stream->overrun_count;
    }

    return (count);
}

uint32_t spi_stream_spurious_count_dt(struct spi_dt_spec const* spec) {
    struct spi_nxp_stream_data const* stream = lpspi_stream_data_dt(spec);
    uint32_t count = 0U;

    if (stream != NULL) {
        count = stream->spurious_count;
    }

    return (count);
}

uint32_t spi_stream_coalesced_count_dt(struct spi_dt_spec const* spec) {
    struct spi_nxp_stream_data const* stream = lpspi_stream_data_dt(spec);
    uint32_t count = 0U;

    if (stream != NULL) {
        count = stream->coalesced_count;
    }

    return (count);
}

/* END OF FILE */
