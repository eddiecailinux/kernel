/*
 * Register interface file for Samsung Camera Interface (CAMERIC) driver
 *
 * Copyright (C) 2010 - 2013 Samsung Electronics Co., Ltd.
 * Sylwester Nawrocki <s.nawrocki@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include "cameric-reg.h"

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/regmap.h>

#include <media/drv-intf/cameric.h>
#include "media-dev.h"

#include "cameric-core.h"

void cameric_hw_reset(struct cameric_dev *dev)
{
	u32 cfg;

	cfg = readl(dev->regs + CAMERIC_REG_CISRCFMT);
	cfg |= CAMERIC_REG_CISRCFMT_ITU601_8BIT;
	writel(cfg, dev->regs + CAMERIC_REG_CISRCFMT);

	/* Software reset. */
	cfg = readl(dev->regs + CAMERIC_REG_CIGCTRL);
	cfg |= (CAMERIC_REG_CIGCTRL_SWRST | CAMERIC_REG_CIGCTRL_IRQ_LEVEL);
	writel(cfg, dev->regs + CAMERIC_REG_CIGCTRL);
	udelay(10);

	cfg = readl(dev->regs + CAMERIC_REG_CIGCTRL);
	cfg &= ~CAMERIC_REG_CIGCTRL_SWRST;
	writel(cfg, dev->regs + CAMERIC_REG_CIGCTRL);

	if (dev->drv_data->out_buf_count > 4)
		cameric_hw_set_dma_seq(dev, 0xF);
}

static u32 cameric_hw_get_in_flip(struct cameric_ctx *ctx)
{
	u32 flip = CAMERIC_REG_MSCTRL_FLIP_NORMAL;

	if (ctx->hflip)
		flip = CAMERIC_REG_MSCTRL_FLIP_Y_MIRROR;
	if (ctx->vflip)
		flip = CAMERIC_REG_MSCTRL_FLIP_X_MIRROR;

	if (ctx->rotation <= 90)
		return flip;

	return (flip ^ CAMERIC_REG_MSCTRL_FLIP_180) & CAMERIC_REG_MSCTRL_FLIP_180;
}

static u32 cameric_hw_get_target_flip(struct cameric_ctx *ctx)
{
	u32 flip = CAMERIC_REG_CITRGFMT_FLIP_NORMAL;

	if (ctx->hflip)
		flip |= CAMERIC_REG_CITRGFMT_FLIP_Y_MIRROR;
	if (ctx->vflip)
		flip |= CAMERIC_REG_CITRGFMT_FLIP_X_MIRROR;

	if (ctx->rotation <= 90)
		return flip;

	return (flip ^ CAMERIC_REG_CITRGFMT_FLIP_180) & CAMERIC_REG_CITRGFMT_FLIP_180;
}

void cameric_hw_set_rotation(struct cameric_ctx *ctx)
{
	u32 cfg, flip;
	struct cameric_dev *dev = ctx->cameric_dev;

	cfg = readl(dev->regs + CAMERIC_REG_CITRGFMT);
	cfg &= ~(CAMERIC_REG_CITRGFMT_INROT90 | CAMERIC_REG_CITRGFMT_OUTROT90 |
		 CAMERIC_REG_CITRGFMT_FLIP_180);

	/*
	 * The input and output rotator cannot work simultaneously.
	 * Use the output rotator in output DMA mode or the input rotator
	 * in direct fifo output mode.
	 */
	if (ctx->rotation == 90 || ctx->rotation == 270) {
		if (ctx->out_path == CAMERIC_IO_LCDFIFO)
			cfg |= CAMERIC_REG_CITRGFMT_INROT90;
		else
			cfg |= CAMERIC_REG_CITRGFMT_OUTROT90;
	}

	if (ctx->out_path == CAMERIC_IO_DMA) {
		cfg |= cameric_hw_get_target_flip(ctx);
		writel(cfg, dev->regs + CAMERIC_REG_CITRGFMT);
	} else {
		/* LCD FIFO path */
		flip = readl(dev->regs + CAMERIC_REG_MSCTRL);
		flip &= ~CAMERIC_REG_MSCTRL_FLIP_MASK;
		flip |= cameric_hw_get_in_flip(ctx);
		writel(flip, dev->regs + CAMERIC_REG_MSCTRL);
	}
}

void cameric_hw_set_target_format(struct cameric_ctx *ctx)
{
	u32 cfg;
	struct cameric_dev *dev = ctx->cameric_dev;
	struct cameric_frame *frame = &ctx->d_frame;

	dbg("w= %d, h= %d color: %d", frame->width,
	    frame->height, frame->fmt->color);

	cfg = readl(dev->regs + CAMERIC_REG_CITRGFMT);
	cfg &= ~(CAMERIC_REG_CITRGFMT_FMT_MASK | CAMERIC_REG_CITRGFMT_HSIZE_MASK |
		 CAMERIC_REG_CITRGFMT_VSIZE_MASK);

	switch (frame->fmt->color) {
	case CAMERIC_FMT_RGB444...CAMERIC_FMT_RGB888:
		cfg |= CAMERIC_REG_CITRGFMT_RGB;
		break;
	case CAMERIC_FMT_YCBCR420:
		cfg |= CAMERIC_REG_CITRGFMT_YCBCR420;
		break;
	case CAMERIC_FMT_YCBYCR422...CAMERIC_FMT_CRYCBY422:
		if (frame->fmt->colplanes == 1)
			cfg |= CAMERIC_REG_CITRGFMT_YCBCR422_1P;
		else
			cfg |= CAMERIC_REG_CITRGFMT_YCBCR422;
		break;
	default:
		break;
	}

	if (ctx->rotation == 90 || ctx->rotation == 270)
		cfg |= (frame->height << 16) | frame->width;
	else
		cfg |= (frame->width << 16) | frame->height;

	writel(cfg, dev->regs + CAMERIC_REG_CITRGFMT);

	cfg = readl(dev->regs + CAMERIC_REG_CITAREA);
	cfg &= ~CAMERIC_REG_CITAREA_MASK;
	cfg |= (frame->width * frame->height);
	writel(cfg, dev->regs + CAMERIC_REG_CITAREA);
}

static void cameric_hw_set_out_dma_size(struct cameric_ctx *ctx)
{
	struct cameric_dev *dev = ctx->cameric_dev;
	struct cameric_frame *frame = &ctx->d_frame;
	u32 cfg;

	cfg = (frame->f_height << 16) | frame->f_width;
	writel(cfg, dev->regs + CAMERIC_REG_ORGOSIZE);

	/* Select color space conversion equation (HD/SD size).*/
	cfg = readl(dev->regs + CAMERIC_REG_CIGCTRL);
	if (frame->f_width >= 1280) /* HD */
		cfg |= CAMERIC_REG_CIGCTRL_CSC_ITU601_709;
	else	/* SD */
		cfg &= ~CAMERIC_REG_CIGCTRL_CSC_ITU601_709;
	writel(cfg, dev->regs + CAMERIC_REG_CIGCTRL);

}

void cameric_hw_set_out_dma(struct cameric_ctx *ctx)
{
	struct cameric_dev *dev = ctx->cameric_dev;
	struct cameric_frame *frame = &ctx->d_frame;
	struct cameric_dma_offset *offset = &frame->dma_offset;
	struct cameric_fmt *fmt = frame->fmt;
	u32 cfg;

	/* Set the input dma offsets. */
	cfg = (offset->y_v << 16) | offset->y_h;
	writel(cfg, dev->regs + CAMERIC_REG_CIOYOFF);

	cfg = (offset->cb_v << 16) | offset->cb_h;
	writel(cfg, dev->regs + CAMERIC_REG_CIOCBOFF);

	cfg = (offset->cr_v << 16) | offset->cr_h;
	writel(cfg, dev->regs + CAMERIC_REG_CIOCROFF);

	cameric_hw_set_out_dma_size(ctx);

	/* Configure chroma components order. */
	cfg = readl(dev->regs + CAMERIC_REG_CIOCTRL);

	cfg &= ~(CAMERIC_REG_CIOCTRL_ORDER2P_MASK |
		 CAMERIC_REG_CIOCTRL_ORDER422_MASK |
		 CAMERIC_REG_CIOCTRL_YCBCR_PLANE_MASK |
		 CAMERIC_REG_CIOCTRL_RGB16FMT_MASK);

	if (fmt->colplanes == 1)
		cfg |= ctx->out_order_1p;
	else if (fmt->colplanes == 2)
		cfg |= ctx->out_order_2p | CAMERIC_REG_CIOCTRL_YCBCR_2PLANE;
	else if (fmt->colplanes == 3)
		cfg |= CAMERIC_REG_CIOCTRL_YCBCR_3PLANE;

	if (fmt->color == CAMERIC_FMT_RGB565)
		cfg |= CAMERIC_REG_CIOCTRL_RGB565;
	else if (fmt->color == CAMERIC_FMT_RGB555)
		cfg |= CAMERIC_REG_CIOCTRL_ARGB1555;
	else if (fmt->color == CAMERIC_FMT_RGB444)
		cfg |= CAMERIC_REG_CIOCTRL_ARGB4444;

	writel(cfg, dev->regs + CAMERIC_REG_CIOCTRL);
}

static void cameric_hw_en_autoload(struct cameric_dev *dev, int enable)
{
	u32 cfg = readl(dev->regs + CAMERIC_REG_ORGISIZE);
	if (enable)
		cfg |= CAMERIC_REG_CIREAL_ISIZE_AUTOLOAD_EN;
	else
		cfg &= ~CAMERIC_REG_CIREAL_ISIZE_AUTOLOAD_EN;
	writel(cfg, dev->regs + CAMERIC_REG_ORGISIZE);
}

void cameric_hw_en_lastirq(struct cameric_dev *dev, int enable)
{
	u32 cfg = readl(dev->regs + CAMERIC_REG_CIOCTRL);
	if (enable)
		cfg |= CAMERIC_REG_CIOCTRL_LASTIRQ_ENABLE;
	else
		cfg &= ~CAMERIC_REG_CIOCTRL_LASTIRQ_ENABLE;
	writel(cfg, dev->regs + CAMERIC_REG_CIOCTRL);
}

void cameric_hw_set_prescaler(struct cameric_ctx *ctx)
{
	struct cameric_dev *dev =  ctx->cameric_dev;
	struct cameric_scaler *sc = &ctx->scaler;
	u32 cfg, shfactor;

	shfactor = 10 - (sc->hfactor + sc->vfactor);
	cfg = shfactor << 28;

	cfg |= (sc->pre_hratio << 16) | sc->pre_vratio;
	writel(cfg, dev->regs + CAMERIC_REG_CISCPRERATIO);

	cfg = (sc->pre_dst_width << 16) | sc->pre_dst_height;
	writel(cfg, dev->regs + CAMERIC_REG_CISCPREDST);
}

static void cameric_hw_set_scaler(struct cameric_ctx *ctx)
{
	struct cameric_dev *dev = ctx->cameric_dev;
	struct cameric_scaler *sc = &ctx->scaler;
	struct cameric_frame *src_frame = &ctx->s_frame;
	struct cameric_frame *dst_frame = &ctx->d_frame;

	u32 cfg = readl(dev->regs + CAMERIC_REG_CISCCTRL);

	cfg &= ~(CAMERIC_REG_CISCCTRL_CSCR2Y_WIDE | CAMERIC_REG_CISCCTRL_CSCY2R_WIDE |
		 CAMERIC_REG_CISCCTRL_SCALEUP_H | CAMERIC_REG_CISCCTRL_SCALEUP_V |
		 CAMERIC_REG_CISCCTRL_SCALERBYPASS | CAMERIC_REG_CISCCTRL_ONE2ONE |
		 CAMERIC_REG_CISCCTRL_INRGB_FMT_MASK | CAMERIC_REG_CISCCTRL_OUTRGB_FMT_MASK |
		 CAMERIC_REG_CISCCTRL_INTERLACE | CAMERIC_REG_CISCCTRL_RGB_EXT);

	if (!(ctx->flags & CAMERIC_COLOR_RANGE_NARROW))
		cfg |= (CAMERIC_REG_CISCCTRL_CSCR2Y_WIDE |
			CAMERIC_REG_CISCCTRL_CSCY2R_WIDE);

	if (!sc->enabled)
		cfg |= CAMERIC_REG_CISCCTRL_SCALERBYPASS;

	if (sc->scaleup_h)
		cfg |= CAMERIC_REG_CISCCTRL_SCALEUP_H;

	if (sc->scaleup_v)
		cfg |= CAMERIC_REG_CISCCTRL_SCALEUP_V;

	if (sc->copy_mode)
		cfg |= CAMERIC_REG_CISCCTRL_ONE2ONE;

	if (ctx->in_path == CAMERIC_IO_DMA) {
		switch (src_frame->fmt->color) {
		case CAMERIC_FMT_RGB565:
			cfg |= CAMERIC_REG_CISCCTRL_INRGB_FMT_RGB565;
			break;
		case CAMERIC_FMT_RGB666:
			cfg |= CAMERIC_REG_CISCCTRL_INRGB_FMT_RGB666;
			break;
		case CAMERIC_FMT_RGB888:
			cfg |= CAMERIC_REG_CISCCTRL_INRGB_FMT_RGB888;
			break;
		}
	}

	if (ctx->out_path == CAMERIC_IO_DMA) {
		u32 color = dst_frame->fmt->color;

		if (color >= CAMERIC_FMT_RGB444 && color <= CAMERIC_FMT_RGB565)
			cfg |= CAMERIC_REG_CISCCTRL_OUTRGB_FMT_RGB565;
		else if (color == CAMERIC_FMT_RGB666)
			cfg |= CAMERIC_REG_CISCCTRL_OUTRGB_FMT_RGB666;
		else if (color == CAMERIC_FMT_RGB888)
			cfg |= CAMERIC_REG_CISCCTRL_OUTRGB_FMT_RGB888;
	} else {
		cfg |= CAMERIC_REG_CISCCTRL_OUTRGB_FMT_RGB888;

		if (ctx->flags & CAMERIC_SCAN_MODE_INTERLACED)
			cfg |= CAMERIC_REG_CISCCTRL_INTERLACE;
	}

	writel(cfg, dev->regs + CAMERIC_REG_CISCCTRL);
}

void cameric_hw_set_mainscaler(struct cameric_ctx *ctx)
{
	struct cameric_dev *dev = ctx->cameric_dev;
	const struct cameric_variant *variant = dev->variant;
	struct cameric_scaler *sc = &ctx->scaler;
	u32 cfg;

	dbg("main_hratio= 0x%X  main_vratio= 0x%X",
	    sc->main_hratio, sc->main_vratio);

	cameric_hw_set_scaler(ctx);

	cfg = readl(dev->regs + CAMERIC_REG_CISCCTRL);
	cfg &= ~(CAMERIC_REG_CISCCTRL_MHRATIO_MASK |
		 CAMERIC_REG_CISCCTRL_MVRATIO_MASK);

	if (variant->has_mainscaler_ext) {
		cfg |= CAMERIC_REG_CISCCTRL_MHRATIO_EXT(sc->main_hratio);
		cfg |= CAMERIC_REG_CISCCTRL_MVRATIO_EXT(sc->main_vratio);
		writel(cfg, dev->regs + CAMERIC_REG_CISCCTRL);

		cfg = readl(dev->regs + CAMERIC_REG_CIEXTEN);

		cfg &= ~(CAMERIC_REG_CIEXTEN_MVRATIO_EXT_MASK |
			 CAMERIC_REG_CIEXTEN_MHRATIO_EXT_MASK);
		cfg |= CAMERIC_REG_CIEXTEN_MHRATIO_EXT(sc->main_hratio);
		cfg |= CAMERIC_REG_CIEXTEN_MVRATIO_EXT(sc->main_vratio);
		writel(cfg, dev->regs + CAMERIC_REG_CIEXTEN);
	} else {
		cfg |= CAMERIC_REG_CISCCTRL_MHRATIO(sc->main_hratio);
		cfg |= CAMERIC_REG_CISCCTRL_MVRATIO(sc->main_vratio);
		writel(cfg, dev->regs + CAMERIC_REG_CISCCTRL);
	}
}

void cameric_hw_enable_capture(struct cameric_ctx *ctx)
{
	struct cameric_dev *dev = ctx->cameric_dev;
	u32 cfg;

	cfg = readl(dev->regs + CAMERIC_REG_CIIMGCPT);
	cfg |= CAMERIC_REG_CIIMGCPT_CPT_FREN_ENABLE;

	if (ctx->scaler.enabled)
		cfg |= CAMERIC_REG_CIIMGCPT_IMGCPTEN_SC;
	else
		cfg &= CAMERIC_REG_CIIMGCPT_IMGCPTEN_SC;

	cfg |= CAMERIC_REG_CIIMGCPT_IMGCPTEN;
	writel(cfg, dev->regs + CAMERIC_REG_CIIMGCPT);
}

void cameric_hw_disable_capture(struct cameric_dev *dev)
{
	u32 cfg = readl(dev->regs + CAMERIC_REG_CIIMGCPT);
	cfg &= ~(CAMERIC_REG_CIIMGCPT_IMGCPTEN |
		 CAMERIC_REG_CIIMGCPT_IMGCPTEN_SC);
	writel(cfg, dev->regs + CAMERIC_REG_CIIMGCPT);
}

void cameric_hw_set_effect(struct cameric_ctx *ctx)
{
	struct cameric_dev *dev = ctx->cameric_dev;
	struct cameric_effect *effect = &ctx->effect;
	u32 cfg = 0;

	if (effect->type != CAMERIC_REG_CIIMGEFF_FIN_BYPASS) {
		cfg |= CAMERIC_REG_CIIMGEFF_IE_SC_AFTER |
			CAMERIC_REG_CIIMGEFF_IE_ENABLE;
		cfg |= effect->type;
		if (effect->type == CAMERIC_REG_CIIMGEFF_FIN_ARBITRARY)
			cfg |= (effect->pat_cb << 13) | effect->pat_cr;
	}

	writel(cfg, dev->regs + CAMERIC_REG_CIIMGEFF);
}

void cameric_hw_set_rgb_alpha(struct cameric_ctx *ctx)
{
	struct cameric_dev *dev = ctx->cameric_dev;
	struct cameric_frame *frame = &ctx->d_frame;
	u32 cfg;

	if (!(frame->fmt->flags & FMT_HAS_ALPHA))
		return;

	cfg = readl(dev->regs + CAMERIC_REG_CIOCTRL);
	cfg &= ~CAMERIC_REG_CIOCTRL_ALPHA_OUT_MASK;
	cfg |= (frame->alpha << 4);
	writel(cfg, dev->regs + CAMERIC_REG_CIOCTRL);
}

static void cameric_hw_set_in_dma_size(struct cameric_ctx *ctx)
{
	struct cameric_dev *dev = ctx->cameric_dev;
	struct cameric_frame *frame = &ctx->s_frame;
	u32 cfg_o = 0;
	u32 cfg_r = 0;

	if (CAMERIC_IO_LCDFIFO == ctx->out_path)
		cfg_r |= CAMERIC_REG_CIREAL_ISIZE_AUTOLOAD_EN;

	cfg_o |= (frame->f_height << 16) | frame->f_width;
	cfg_r |= (frame->height << 16) | frame->width;

	writel(cfg_o, dev->regs + CAMERIC_REG_ORGISIZE);
	writel(cfg_r, dev->regs + CAMERIC_REG_CIREAL_ISIZE);
}

void cameric_hw_set_in_dma(struct cameric_ctx *ctx)
{
	struct cameric_dev *dev = ctx->cameric_dev;
	struct cameric_frame *frame = &ctx->s_frame;
	struct cameric_dma_offset *offset = &frame->dma_offset;
	u32 cfg;

	/* Set the pixel offsets. */
	cfg = (offset->y_v << 16) | offset->y_h;
	writel(cfg, dev->regs + CAMERIC_REG_CIIYOFF);

	cfg = (offset->cb_v << 16) | offset->cb_h;
	writel(cfg, dev->regs + CAMERIC_REG_CIICBOFF);

	cfg = (offset->cr_v << 16) | offset->cr_h;
	writel(cfg, dev->regs + CAMERIC_REG_CIICROFF);

	/* Input original and real size. */
	cameric_hw_set_in_dma_size(ctx);

	/* Use DMA autoload only in FIFO mode. */
	cameric_hw_en_autoload(dev, ctx->out_path == CAMERIC_IO_LCDFIFO);

	/* Set the input DMA to process single frame only. */
	cfg = readl(dev->regs + CAMERIC_REG_MSCTRL);
	cfg &= ~(CAMERIC_REG_MSCTRL_INFORMAT_MASK
		 | CAMERIC_REG_MSCTRL_IN_BURST_COUNT_MASK
		 | CAMERIC_REG_MSCTRL_INPUT_MASK
		 | CAMERIC_REG_MSCTRL_C_INT_IN_MASK
		 | CAMERIC_REG_MSCTRL_2P_IN_ORDER_MASK
		 | CAMERIC_REG_MSCTRL_ORDER422_MASK);

	cfg |= (CAMERIC_REG_MSCTRL_IN_BURST_COUNT(4)
		| CAMERIC_REG_MSCTRL_INPUT_MEMORY
		| CAMERIC_REG_MSCTRL_FIFO_CTRL_FULL);

	switch (frame->fmt->color) {
	case CAMERIC_FMT_RGB565...CAMERIC_FMT_RGB888:
		cfg |= CAMERIC_REG_MSCTRL_INFORMAT_RGB;
		break;
	case CAMERIC_FMT_YCBCR420:
		cfg |= CAMERIC_REG_MSCTRL_INFORMAT_YCBCR420;

		if (frame->fmt->colplanes == 2)
			cfg |= ctx->in_order_2p | CAMERIC_REG_MSCTRL_C_INT_IN_2PLANE;
		else
			cfg |= CAMERIC_REG_MSCTRL_C_INT_IN_3PLANE;

		break;
	case CAMERIC_FMT_YCBYCR422...CAMERIC_FMT_CRYCBY422:
		if (frame->fmt->colplanes == 1) {
			cfg |= ctx->in_order_1p
				| CAMERIC_REG_MSCTRL_INFORMAT_YCBCR422_1P;
		} else {
			cfg |= CAMERIC_REG_MSCTRL_INFORMAT_YCBCR422;

			if (frame->fmt->colplanes == 2)
				cfg |= ctx->in_order_2p
					| CAMERIC_REG_MSCTRL_C_INT_IN_2PLANE;
			else
				cfg |= CAMERIC_REG_MSCTRL_C_INT_IN_3PLANE;
		}
		break;
	default:
		break;
	}

	writel(cfg, dev->regs + CAMERIC_REG_MSCTRL);

	/* Input/output DMA linear/tiled mode. */
	cfg = readl(dev->regs + CAMERIC_REG_CIDMAPARAM);
	cfg &= ~CAMERIC_REG_CIDMAPARAM_TILE_MASK;

	if (tiled_fmt(ctx->s_frame.fmt))
		cfg |= CAMERIC_REG_CIDMAPARAM_R_64X32;

	if (tiled_fmt(ctx->d_frame.fmt))
		cfg |= CAMERIC_REG_CIDMAPARAM_W_64X32;

	writel(cfg, dev->regs + CAMERIC_REG_CIDMAPARAM);
}


void cameric_hw_set_input_path(struct cameric_ctx *ctx)
{
	struct cameric_dev *dev = ctx->cameric_dev;

	u32 cfg = readl(dev->regs + CAMERIC_REG_MSCTRL);
	cfg &= ~CAMERIC_REG_MSCTRL_INPUT_MASK;

	if (ctx->in_path == CAMERIC_IO_DMA)
		cfg |= CAMERIC_REG_MSCTRL_INPUT_MEMORY;
	else
		cfg |= CAMERIC_REG_MSCTRL_INPUT_EXTCAM;

	writel(cfg, dev->regs + CAMERIC_REG_MSCTRL);
}

void cameric_hw_set_output_path(struct cameric_ctx *ctx)
{
	struct cameric_dev *dev = ctx->cameric_dev;

	u32 cfg = readl(dev->regs + CAMERIC_REG_CISCCTRL);
	cfg &= ~CAMERIC_REG_CISCCTRL_LCDPATHEN_FIFO;
	if (ctx->out_path == CAMERIC_IO_LCDFIFO)
		cfg |= CAMERIC_REG_CISCCTRL_LCDPATHEN_FIFO;
	writel(cfg, dev->regs + CAMERIC_REG_CISCCTRL);
}

void cameric_hw_set_input_addr(struct cameric_dev *dev, struct cameric_addr *paddr)
{
	u32 cfg = readl(dev->regs + CAMERIC_REG_CIREAL_ISIZE);
	cfg |= CAMERIC_REG_CIREAL_ISIZE_ADDR_CH_DIS;
	writel(cfg, dev->regs + CAMERIC_REG_CIREAL_ISIZE);

	writel(paddr->y, dev->regs + CAMERIC_REG_CIIYSA(0));
	writel(paddr->cb, dev->regs + CAMERIC_REG_CIICBSA(0));
	writel(paddr->cr, dev->regs + CAMERIC_REG_CIICRSA(0));

	cfg &= ~CAMERIC_REG_CIREAL_ISIZE_ADDR_CH_DIS;
	writel(cfg, dev->regs + CAMERIC_REG_CIREAL_ISIZE);
}

void cameric_hw_set_output_addr(struct cameric_dev *dev,
			     struct cameric_addr *paddr, int index)
{
	int i = (index == -1) ? 0 : index;
	do {
		writel(paddr->y, dev->regs + CAMERIC_REG_CIOYSA(i));
		writel(paddr->cb, dev->regs + CAMERIC_REG_CIOCBSA(i));
		writel(paddr->cr, dev->regs + CAMERIC_REG_CIOCRSA(i));
		dbg("dst_buf[%d]: 0x%X, cb: 0x%X, cr: 0x%X",
		    i, paddr->y, paddr->cb, paddr->cr);
	} while (index == -1 && ++i < CAMERIC_MAX_OUT_BUFS);
}

int cameric_hw_set_camera_polarity(struct cameric_dev *cameric,
				struct cameric_source_info *cam)
{
	u32 cfg = readl(cameric->regs + CAMERIC_REG_CIGCTRL);

	cfg &= ~(CAMERIC_REG_CIGCTRL_INVPOLPCLK | CAMERIC_REG_CIGCTRL_INVPOLVSYNC |
		 CAMERIC_REG_CIGCTRL_INVPOLHREF | CAMERIC_REG_CIGCTRL_INVPOLHSYNC |
		 CAMERIC_REG_CIGCTRL_INVPOLFIELD);

	if (cam->flags & V4L2_MBUS_PCLK_SAMPLE_FALLING)
		cfg |= CAMERIC_REG_CIGCTRL_INVPOLPCLK;

	if (cam->flags & V4L2_MBUS_VSYNC_ACTIVE_LOW)
		cfg |= CAMERIC_REG_CIGCTRL_INVPOLVSYNC;

	if (cam->flags & V4L2_MBUS_HSYNC_ACTIVE_LOW)
		cfg |= CAMERIC_REG_CIGCTRL_INVPOLHREF;

	if (cam->flags & V4L2_MBUS_HSYNC_ACTIVE_LOW)
		cfg |= CAMERIC_REG_CIGCTRL_INVPOLHSYNC;

	if (cam->flags & V4L2_MBUS_FIELD_EVEN_LOW)
		cfg |= CAMERIC_REG_CIGCTRL_INVPOLFIELD;

	writel(cfg, cameric->regs + CAMERIC_REG_CIGCTRL);

	return 0;
}

struct mbus_pixfmt_desc {
	u32 pixelcode;
	u32 cisrcfmt;
	u16 bus_width;
};

static const struct mbus_pixfmt_desc pix_desc[] = {
	{ MEDIA_BUS_FMT_YUYV8_2X8, CAMERIC_REG_CISRCFMT_ORDER422_YCBYCR, 8 },
	{ MEDIA_BUS_FMT_YVYU8_2X8, CAMERIC_REG_CISRCFMT_ORDER422_YCRYCB, 8 },
	{ MEDIA_BUS_FMT_VYUY8_2X8, CAMERIC_REG_CISRCFMT_ORDER422_CRYCBY, 8 },
	{ MEDIA_BUS_FMT_UYVY8_2X8, CAMERIC_REG_CISRCFMT_ORDER422_CBYCRY, 8 },
};

int cameric_hw_set_camera_source(struct cameric_dev *cameric,
			      struct cameric_source_info *source)
{
	struct cameric_vid_cap *vc = &cameric->vid_cap;
	struct cameric_frame *f = &vc->ctx->s_frame;
	u32 bus_width, cfg = 0;
	int i;

	switch (source->cameric_bus_type) {
	case CAMERIC_BUS_TYPE_ITU_601:
	case CAMERIC_BUS_TYPE_ITU_656:
		for (i = 0; i < ARRAY_SIZE(pix_desc); i++) {
			if (vc->ci_fmt.code == pix_desc[i].pixelcode) {
				cfg = pix_desc[i].cisrcfmt;
				bus_width = pix_desc[i].bus_width;
				break;
			}
		}

		if (i == ARRAY_SIZE(pix_desc)) {
			v4l2_err(&vc->ve.vdev,
				 "Camera color format not supported: %d\n",
				 vc->ci_fmt.code);
			return -EINVAL;
		}

		if (source->cameric_bus_type == CAMERIC_BUS_TYPE_ITU_601) {
			if (bus_width == 8)
				cfg |= CAMERIC_REG_CISRCFMT_ITU601_8BIT;
			else if (bus_width == 16)
				cfg |= CAMERIC_REG_CISRCFMT_ITU601_16BIT;
		} /* else defaults to ITU-R BT.656 8-bit */
		break;
	case CAMERIC_BUS_TYPE_MIPI_CSI2:
		if (cameric_fmt_is_user_defined(f->fmt->color))
			cfg |= CAMERIC_REG_CISRCFMT_ITU601_8BIT;
		break;
	default:
	case CAMERIC_BUS_TYPE_ISP_WRITEBACK:
		/* Anything to do here ? */
		break;
	}

	cfg |= (f->o_width << 16) | f->o_height;
	writel(cfg, cameric->regs + CAMERIC_REG_CISRCFMT);
	return 0;
}

void cameric_hw_set_camera_offset(struct cameric_dev *cameric, struct cameric_frame *f)
{
	u32 hoff2, voff2;

	u32 cfg = readl(cameric->regs + CAMERIC_REG_CIWDOFST);

	cfg &= ~(CAMERIC_REG_CIWDOFST_HOROFF_MASK | CAMERIC_REG_CIWDOFST_VEROFF_MASK);
	cfg |=  CAMERIC_REG_CIWDOFST_OFF_EN |
		(f->offs_h << 16) | f->offs_v;

	writel(cfg, cameric->regs + CAMERIC_REG_CIWDOFST);

	/* See CIWDOFSTn register description in the datasheet for details. */
	hoff2 = f->o_width - f->width - f->offs_h;
	voff2 = f->o_height - f->height - f->offs_v;
	cfg = (hoff2 << 16) | voff2;
	writel(cfg, cameric->regs + CAMERIC_REG_CIWDOFST2);
}

int cameric_hw_set_camera_type(struct cameric_dev *cameric,
			    struct cameric_source_info *source)
{
	struct cameric_vid_cap *vid_cap = &cameric->vid_cap;
	u32 csis_data_alignment = 32;
	u32 cfg, tmp;

	cfg = readl(cameric->regs + CAMERIC_REG_CIGCTRL);

	/* Select ITU B interface, disable Writeback path and test pattern. */
	cfg &= ~(CAMERIC_REG_CIGCTRL_TESTPAT_MASK | CAMERIC_REG_CIGCTRL_SELCAM_ITU_A |
		CAMERIC_REG_CIGCTRL_SELCAM_MIPI | CAMERIC_REG_CIGCTRL_CAMIF_SELWB |
		CAMERIC_REG_CIGCTRL_SELCAM_MIPI_A | CAMERIC_REG_CIGCTRL_CAM_JPEG |
		CAMERIC_REG_CIGCTRL_SELWB_A);

	switch (source->cameric_bus_type) {
	case CAMERIC_BUS_TYPE_MIPI_CSI2:
		cfg |= CAMERIC_REG_CIGCTRL_SELCAM_MIPI;

		if (source->mux_id == 0)
			cfg |= CAMERIC_REG_CIGCTRL_SELCAM_MIPI_A;

		/* TODO: add remaining supported formats. */
		switch (vid_cap->ci_fmt.code) {
		case MEDIA_BUS_FMT_VYUY8_2X8:
			tmp = CAMERIC_REG_CSIIMGFMT_YCBCR422_8BIT;
			break;
		case MEDIA_BUS_FMT_JPEG_1X8:
		case MEDIA_BUS_FMT_S5C_UYVY_JPEG_1X8:
			tmp = CAMERIC_REG_CSIIMGFMT_USER(1);
			cfg |= CAMERIC_REG_CIGCTRL_CAM_JPEG;
			break;
		default:
			v4l2_err(&vid_cap->ve.vdev,
				 "Not supported camera pixel format: %#x\n",
				 vid_cap->ci_fmt.code);
			return -EINVAL;
		}
		tmp |= (csis_data_alignment == 32) << 8;

		writel(tmp, cameric->regs + CAMERIC_REG_CSIIMGFMT);
		break;
	case CAMERIC_BUS_TYPE_ITU_601...CAMERIC_BUS_TYPE_ITU_656:
		if (source->mux_id == 0) /* ITU-A, ITU-B: 0, 1 */
			cfg |= CAMERIC_REG_CIGCTRL_SELCAM_ITU_A;
		break;
	case CAMERIC_BUS_TYPE_LCD_WRITEBACK_A:
		cfg |= CAMERIC_REG_CIGCTRL_CAMIF_SELWB;
		/* fall through */
	case CAMERIC_BUS_TYPE_ISP_WRITEBACK:
		if (cameric->variant->has_isp_wb)
			cfg |= CAMERIC_REG_CIGCTRL_CAMIF_SELWB;
		else
			WARN_ONCE(1, "ISP Writeback input is not supported\n");
		break;
	default:
		v4l2_err(&vid_cap->ve.vdev,
			 "Invalid CAMERIC bus type selected: %d\n",
			 source->cameric_bus_type);
		return -EINVAL;
	}
	writel(cfg, cameric->regs + CAMERIC_REG_CIGCTRL);

	return 0;
}

void cameric_hw_clear_irq(struct cameric_dev *dev)
{
	u32 cfg = readl(dev->regs + CAMERIC_REG_CIGCTRL);
	cfg |= CAMERIC_REG_CIGCTRL_IRQ_CLR;
	writel(cfg, dev->regs + CAMERIC_REG_CIGCTRL);
}

void cameric_hw_enable_scaler(struct cameric_dev *dev, bool on)
{
	u32 cfg = readl(dev->regs + CAMERIC_REG_CISCCTRL);
	if (on)
		cfg |= CAMERIC_REG_CISCCTRL_SCALERSTART;
	else
		cfg &= ~CAMERIC_REG_CISCCTRL_SCALERSTART;
	writel(cfg, dev->regs + CAMERIC_REG_CISCCTRL);
}

void cameric_hw_activate_input_dma(struct cameric_dev *dev, bool on)
{
	u32 cfg = readl(dev->regs + CAMERIC_REG_MSCTRL);
	if (on)
		cfg |= CAMERIC_REG_MSCTRL_ENVID;
	else
		cfg &= ~CAMERIC_REG_MSCTRL_ENVID;
	writel(cfg, dev->regs + CAMERIC_REG_MSCTRL);
}

/* Return an index to the buffer actually being written. */
s32 cameric_hw_get_frame_index(struct cameric_dev *dev)
{
	s32 reg;

	if (dev->drv_data->cistatus2) {
		reg = readl(dev->regs + CAMERIC_REG_CISTATUS2) & 0x3f;
		return reg - 1;
	}

	reg = readl(dev->regs + CAMERIC_REG_CISTATUS);

	return (reg & CAMERIC_REG_CISTATUS_FRAMECNT_MASK) >>
		CAMERIC_REG_CISTATUS_FRAMECNT_SHIFT;
}

/* Return an index to the buffer being written previously. */
s32 cameric_hw_get_prev_frame_index(struct cameric_dev *dev)
{
	s32 reg;

	if (!dev->drv_data->cistatus2)
		return -1;

	//reg = readl(dev->regs + CAMERIC_REG_CISTATUS2);
	return ((reg >> 7) & 0x3f) - 1;
}

/* Locking: the caller holds cameric->slock */
void cameric_activate_capture(struct cameric_ctx *ctx)
{
	cameric_hw_enable_scaler(ctx->cameric_dev, ctx->scaler.enabled);
	cameric_hw_enable_capture(ctx);
}

void cameric_deactivate_capture(struct cameric_dev *cameric)
{
	cameric_hw_en_lastirq(cameric, true);
	cameric_hw_disable_capture(cameric);
	cameric_hw_enable_scaler(cameric, false);
	cameric_hw_en_lastirq(cameric, false);
}

int cameric_hw_camblk_cfg_writeback(struct cameric_dev *cameric)
{
	struct regmap *map = cameric->sysreg;
	unsigned int mask, val, camblk_cfg;
	int ret;
#if 0
	if (map == NULL)
		return 0;

	ret = regmap_read(map, SYSREG_CAMBLK, &camblk_cfg);
	if (ret < 0 || ((camblk_cfg & 0x00700000) >> 20 != 0x3))
		return ret;

	if (!WARN(cameric->id >= 3, "not supported id: %d\n", cameric->id))
		val = 0x1 << (cameric->id + 20);
	else
		val = 0;

	mask = SYSREG_CAMBLK_FIFORST_ISP | SYSREG_CAMBLK_ISPWB_FULL_EN;
	ret = regmap_update_bits(map, SYSREG_CAMBLK, mask, val);
	if (ret < 0)
		return ret;

	usleep_range(1000, 2000);

	val |= SYSREG_CAMBLK_FIFORST_ISP;
	ret = regmap_update_bits(map, SYSREG_CAMBLK, mask, val);
	if (ret < 0)
		return ret;

	mask = SYSREG_ISPBLK_FIFORST_CAM_BLK;
	ret = regmap_update_bits(map, SYSREG_ISPBLK, mask, ~mask);
	if (ret < 0)
		return ret;

	usleep_range(1000, 2000);
#endif
	return regmap_update_bits(map, SYSREG_ISPBLK, mask, mask);
}
