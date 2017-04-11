/*
 * cameric (CAMIF) driver
 *
 * Copyright (C) 2017
 * Eddie Cai <eddie.cai@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, either version 2 of the License,
 * or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/bug.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/list.h>
#include <linux/mfd/syscon.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>
#include <linux/reset.h>
#include "cameric-core.h"
#include "cameric-reg.h"
#include "media-dev.h"

static char *cameric_clocks[MAX_CAMERIC_CLOCKS] = {
	"sclk_cameric", "cameric"
};

static struct cameric_fmt cameric_formats[] = {
	{
		.name		= "RGB565",
		.fourcc		= V4L2_PIX_FMT_RGB565,
		.depth		= { 16 },
		.color		= CAMERIC_FMT_RGB565,
		.memplanes	= 1,
		.colplanes	= 1,
		.flags		= FMT_FLAGS_M2M,
	}, {
		.name		= "BGR666",
		.fourcc		= V4L2_PIX_FMT_BGR666,
		.depth		= { 32 },
		.color		= CAMERIC_FMT_RGB666,
		.memplanes	= 1,
		.colplanes	= 1,
		.flags		= FMT_FLAGS_M2M,
	}, {
		.name		= "BGRA8888, 32 bpp",
		.fourcc		= V4L2_PIX_FMT_BGR32,
		.depth		= { 32 },
		.color		= CAMERIC_FMT_RGB888,
		.memplanes	= 1,
		.colplanes	= 1,
		.flags		= FMT_FLAGS_M2M | FMT_HAS_ALPHA,
	}, {
		.name		= "ARGB1555",
		.fourcc		= V4L2_PIX_FMT_RGB555,
		.depth		= { 16 },
		.color		= CAMERIC_FMT_RGB555,
		.memplanes	= 1,
		.colplanes	= 1,
		.flags		= FMT_FLAGS_M2M_OUT | FMT_HAS_ALPHA,
	}, {
		.name		= "ARGB4444",
		.fourcc		= V4L2_PIX_FMT_RGB444,
		.depth		= { 16 },
		.color		= CAMERIC_FMT_RGB444,
		.memplanes	= 1,
		.colplanes	= 1,
		.flags		= FMT_FLAGS_M2M_OUT | FMT_HAS_ALPHA,
	}, {
		.name		= "YUV 4:4:4",
		.mbus_code	= MEDIA_BUS_FMT_YUV10_1X30,
		.flags		= FMT_FLAGS_WRITEBACK,
	}, {
		.name		= "YUV 4:2:2 packed, YCbYCr",
		.fourcc		= V4L2_PIX_FMT_YUYV,
		.depth		= { 16 },
		.color		= CAMERIC_FMT_YCBYCR422,
		.memplanes	= 1,
		.colplanes	= 1,
		.mbus_code	= MEDIA_BUS_FMT_YUYV8_2X8,
		.flags		= FMT_FLAGS_M2M | FMT_FLAGS_CAM,
	}, {
		.name		= "YUV 4:2:2 packed, CbYCrY",
		.fourcc		= V4L2_PIX_FMT_UYVY,
		.depth		= { 16 },
		.color		= CAMERIC_FMT_CBYCRY422,
		.memplanes	= 1,
		.colplanes	= 1,
		.mbus_code	= MEDIA_BUS_FMT_UYVY8_2X8,
		.flags		= FMT_FLAGS_M2M | FMT_FLAGS_CAM,
	}, {
		.name		= "YUV 4:2:2 packed, CrYCbY",
		.fourcc		= V4L2_PIX_FMT_VYUY,
		.depth		= { 16 },
		.color		= CAMERIC_FMT_CRYCBY422,
		.memplanes	= 1,
		.colplanes	= 1,
		.mbus_code	= MEDIA_BUS_FMT_VYUY8_2X8,
		.flags		= FMT_FLAGS_M2M | FMT_FLAGS_CAM,
	}, {
		.name		= "YUV 4:2:2 packed, YCrYCb",
		.fourcc		= V4L2_PIX_FMT_YVYU,
		.depth		= { 16 },
		.color		= CAMERIC_FMT_YCRYCB422,
		.memplanes	= 1,
		.colplanes	= 1,
		.mbus_code	= MEDIA_BUS_FMT_YVYU8_2X8,
		.flags		= FMT_FLAGS_M2M | FMT_FLAGS_CAM,
	}, {
		.name		= "YUV 4:2:2 planar, Y/Cb/Cr",
		.fourcc		= V4L2_PIX_FMT_YUV422P,
		.depth		= { 16 },
		.color		= CAMERIC_FMT_YCBYCR422,
		.memplanes	= 1,
		.colplanes	= 3,
		.flags		= FMT_FLAGS_M2M,
	}, {
		.name		= "YUV 4:2:2 planar, Y/CbCr",
		.fourcc		= V4L2_PIX_FMT_NV16,
		.depth		= { 16 },
		.color		= CAMERIC_FMT_YCBYCR422,
		.memplanes	= 1,
		.colplanes	= 2,
		.flags		= FMT_FLAGS_M2M,
	}, {
		.name		= "YUV 4:2:2 planar, Y/CrCb",
		.fourcc		= V4L2_PIX_FMT_NV61,
		.depth		= { 16 },
		.color		= CAMERIC_FMT_YCRYCB422,
		.memplanes	= 1,
		.colplanes	= 2,
		.flags		= FMT_FLAGS_M2M,
	}, {
		.name		= "YUV 4:2:0 planar, YCbCr",
		.fourcc		= V4L2_PIX_FMT_YUV420,
		.depth		= { 12 },
		.color		= CAMERIC_FMT_YCBCR420,
		.memplanes	= 1,
		.colplanes	= 3,
		.flags		= FMT_FLAGS_M2M,
	}, {
		.name		= "YUV 4:2:0 planar, Y/CbCr",
		.fourcc		= V4L2_PIX_FMT_NV12,
		.depth		= { 12 },
		.color		= CAMERIC_FMT_YCBCR420,
		.memplanes	= 1,
		.colplanes	= 2,
		.flags		= FMT_FLAGS_M2M,
	}, {
		.name		= "YUV 4:2:0 non-contig. 2p, Y/CbCr",
		.fourcc		= V4L2_PIX_FMT_NV12M,
		.color		= CAMERIC_FMT_YCBCR420,
		.depth		= { 8, 4 },
		.memplanes	= 2,
		.colplanes	= 2,
		.flags		= FMT_FLAGS_M2M,
	}, {
		.name		= "YUV 4:2:0 non-contig. 3p, Y/Cb/Cr",
		.fourcc		= V4L2_PIX_FMT_YUV420M,
		.color		= CAMERIC_FMT_YCBCR420,
		.depth		= { 8, 2, 2 },
		.memplanes	= 3,
		.colplanes	= 3,
		.flags		= FMT_FLAGS_M2M,
	}, {
		.name		= "YUV 4:2:0 non-contig. 2p, tiled",
		.fourcc		= V4L2_PIX_FMT_NV12MT,
		.color		= CAMERIC_FMT_YCBCR420,
		.depth		= { 8, 4 },
		.memplanes	= 2,
		.colplanes	= 2,
		.flags		= FMT_FLAGS_M2M,
	}, {
		.name		= "JPEG encoded data",
		.fourcc		= V4L2_PIX_FMT_JPEG,
		.color		= CAMERIC_FMT_JPEG,
		.depth		= { 8 },
		.memplanes	= 1,
		.colplanes	= 1,
		.mbus_code	= MEDIA_BUS_FMT_JPEG_1X8,
		.flags		= FMT_FLAGS_CAM | FMT_FLAGS_COMPRESSED,
	}, {
		.name		= "S5C73MX interleaved UYVY/JPEG",
		.fourcc		= V4L2_PIX_FMT_S5C_UYVY_JPG,
		.color		= CAMERIC_FMT_YUYV_JPEG,
		.depth		= { 8 },
		.memplanes	= 2,
		.colplanes	= 1,
		.mdataplanes	= 0x2, /* plane 1 holds frame meta data */
		.mbus_code	= MEDIA_BUS_FMT_S5C_UYVY_JPEG_1X8,
		.flags		= FMT_FLAGS_CAM | FMT_FLAGS_COMPRESSED,
	},
};

struct cameric_fmt *cameric_get_format(unsigned int index)
{printk(KERN_INFO "%s\n", __func__);
	if (index >= ARRAY_SIZE(cameric_formats))
		return NULL;

	return &cameric_formats[index];
}

int cameric_check_scaler_ratio(struct cameric_ctx *ctx, int sw, int sh,
			    int dw, int dh, int rotation)
{
	if (rotation == 90 || rotation == 270)
		swap(dw, dh);
	printk(KERN_INFO "%s\n", __func__);
	if (!ctx->scaler.enabled)
		return (sw == dw && sh == dh) ? 0 : -EINVAL;

	if ((sw >= SCALER_MAX_HRATIO * dw) || (sh >= SCALER_MAX_VRATIO * dh))
		return -EINVAL;

	return 0;
}

static int cameric_get_scaler_factor(u32 src, u32 tar, u32 *ratio, u32 *shift)
{
	u32 sh = 6;
	printk(KERN_INFO "%s\n", __func__);
	if (src >= 64 * tar)
		return -EINVAL;

	while (sh--) {
		u32 tmp = 1 << sh;
		if (src >= tar * tmp) {
			*shift = sh, *ratio = tmp;
			return 0;
		}
	}
	*shift = 0, *ratio = 1;
	return 0;
}

int cameric_set_scaler_info(struct cameric_ctx *ctx)
{
	const struct cameric_variant *variant = ctx->cameric_dev->variant;
	struct device *dev = &ctx->cameric_dev->pdev->dev;
	struct cameric_scaler *sc = &ctx->scaler;
	struct cameric_frame *s_frame = &ctx->s_frame;
	struct cameric_frame *d_frame = &ctx->d_frame;
	int tx, ty, sx, sy;
	int ret;
	printk(KERN_INFO "%s\n", __func__);
	if (ctx->rotation == 90 || ctx->rotation == 270) {
		ty = d_frame->width;
		tx = d_frame->height;
	} else {
		tx = d_frame->width;
		ty = d_frame->height;
	}
	if (tx <= 0 || ty <= 0) {
		dev_err(dev, "Invalid target size: %dx%d\n", tx, ty);
		return -EINVAL;
	}

	sx = s_frame->width;
	sy = s_frame->height;
	if (sx <= 0 || sy <= 0) {
		dev_err(dev, "Invalid source size: %dx%d\n", sx, sy);
		return -EINVAL;
	}
	sc->real_width = sx;
	sc->real_height = sy;

	ret = cameric_get_scaler_factor(sx, tx, &sc->pre_hratio, &sc->hfactor);
	if (ret)
		return ret;

	ret = cameric_get_scaler_factor(sy, ty,  &sc->pre_vratio, &sc->vfactor);
	if (ret)
		return ret;

	sc->pre_dst_width = sx / sc->pre_hratio;
	sc->pre_dst_height = sy / sc->pre_vratio;

	if (variant->has_mainscaler_ext) {
		sc->main_hratio = (sx << 14) / (tx << sc->hfactor);
		sc->main_vratio = (sy << 14) / (ty << sc->vfactor);
	} else {
		sc->main_hratio = (sx << 8) / (tx << sc->hfactor);
		sc->main_vratio = (sy << 8) / (ty << sc->vfactor);

	}

	sc->scaleup_h = (tx >= sx) ? 1 : 0;
	sc->scaleup_v = (ty >= sy) ? 1 : 0;

	/* check to see if input and output size/format differ */
	if (s_frame->fmt->color == d_frame->fmt->color
		&& s_frame->width == d_frame->width
		&& s_frame->height == d_frame->height)
		sc->copy_mode = 1;
	else
		sc->copy_mode = 0;

	return 0;
}

static irqreturn_t cameric_irq_handler(int irq, void *priv)
{
	struct cameric_dev *cameric = priv;
	struct cameric_ctx *ctx;
	printk(KERN_INFO "%s\n", __func__);
	cameric_hw_clear_irq(cameric);

	spin_lock(&cameric->slock);

	if (test_and_clear_bit(ST_M2M_PEND, &cameric->state)) {
		if (test_and_clear_bit(ST_M2M_SUSPENDING, &cameric->state)) {
			set_bit(ST_M2M_SUSPENDED, &cameric->state);
			wake_up(&cameric->irq_queue);
			goto out;
		}
		//ctx = v4l2_m2m_get_curr_priv(cameric->m2m.m2m_dev);
		if (ctx != NULL) {
			spin_unlock(&cameric->slock);
			cameric_m2m_job_finish(ctx, VB2_BUF_STATE_DONE);

			if (ctx->state & CAMERIC_CTX_SHUT) {
				ctx->state &= ~CAMERIC_CTX_SHUT;
				wake_up(&cameric->irq_queue);
			}
			return IRQ_HANDLED;
		}
	} else if (test_bit(ST_CAPT_PEND, &cameric->state)) {
		int last_buf = test_bit(ST_CAPT_JPEG, &cameric->state) &&
				cameric->vid_cap.reqbufs_count == 1;
		cameric_capture_irq_handler(cameric, !last_buf);
	}
out:
	spin_unlock(&cameric->slock);
	return IRQ_HANDLED;
}

/* The color format (colplanes, memplanes) must be already configured. */
int cameric_prepare_addr(struct cameric_ctx *ctx, struct vb2_buffer *vb,
		      struct cameric_frame *frame, struct cameric_addr *paddr)
{
	int ret = 0;
	u32 pix_size;
	printk(KERN_INFO "%s\n", __func__);
	if (vb == NULL || frame == NULL)
		return -EINVAL;

	pix_size = frame->width * frame->height;

	dbg("memplanes= %d, colplanes= %d, pix_size= %d",
		frame->fmt->memplanes, frame->fmt->colplanes, pix_size);

	paddr->y = vb2_dma_contig_plane_dma_addr(vb, 0);

	if (frame->fmt->memplanes == 1) {
		switch (frame->fmt->colplanes) {
		case 1:
			paddr->cb = 0;
			paddr->cr = 0;
			break;
		case 2:
			/* decompose Y into Y/Cb */
			paddr->cb = (u32)(paddr->y + pix_size);
			paddr->cr = 0;
			break;
		case 3:
			paddr->cb = (u32)(paddr->y + pix_size);
			/* decompose Y into Y/Cb/Cr */
			if (CAMERIC_FMT_YCBCR420 == frame->fmt->color)
				paddr->cr = (u32)(paddr->cb
						+ (pix_size >> 2));
			else /* 422 */
				paddr->cr = (u32)(paddr->cb
						+ (pix_size >> 1));
			break;
		default:
			return -EINVAL;
		}
	} else if (!frame->fmt->mdataplanes) {
		if (frame->fmt->memplanes >= 2)
			paddr->cb = vb2_dma_contig_plane_dma_addr(vb, 1);

		if (frame->fmt->memplanes == 3)
			paddr->cr = vb2_dma_contig_plane_dma_addr(vb, 2);
	}

	dbg("PHYS_ADDR: y= 0x%X  cb= 0x%X cr= 0x%X ret= %d",
	    paddr->y, paddr->cb, paddr->cr, ret);

	return ret;
}

/* Set order for 1 and 2 plane YCBCR 4:2:2 formats. */
void cameric_set_yuv_order(struct cameric_ctx *ctx)
{
	/* The one only mode supported in SoC. */
	ctx->in_order_2p = CAMERIC_REG_CIOCTRL_ORDER422_2P_LSB_CRCB;
	ctx->out_order_2p = CAMERIC_REG_CIOCTRL_ORDER422_2P_LSB_CRCB;
	printk(KERN_INFO "%s\n", __func__);
	/* Set order for 1 plane input formats. */
	switch (ctx->s_frame.fmt->color) {
	case CAMERIC_FMT_YCRYCB422:
		ctx->in_order_1p = CAMERIC_REG_MSCTRL_ORDER422_YCRYCB;
		break;
	case CAMERIC_FMT_CBYCRY422:
		ctx->in_order_1p = CAMERIC_REG_MSCTRL_ORDER422_CBYCRY;
		break;
	case CAMERIC_FMT_CRYCBY422:
		ctx->in_order_1p = CAMERIC_REG_MSCTRL_ORDER422_CRYCBY;
		break;
	case CAMERIC_FMT_YCBYCR422:
	default:
		ctx->in_order_1p = CAMERIC_REG_MSCTRL_ORDER422_YCBYCR;
		break;
	}
	dbg("ctx->in_order_1p= %d", ctx->in_order_1p);

	switch (ctx->d_frame.fmt->color) {
	case CAMERIC_FMT_YCRYCB422:
		ctx->out_order_1p = CAMERIC_REG_CIOCTRL_ORDER422_YCRYCB;
		break;
	case CAMERIC_FMT_CBYCRY422:
		ctx->out_order_1p = CAMERIC_REG_CIOCTRL_ORDER422_CBYCRY;
		break;
	case CAMERIC_FMT_CRYCBY422:
		ctx->out_order_1p = CAMERIC_REG_CIOCTRL_ORDER422_CRYCBY;
		break;
	case CAMERIC_FMT_YCBYCR422:
	default:
		ctx->out_order_1p = CAMERIC_REG_CIOCTRL_ORDER422_YCBYCR;
		break;
	}
	dbg("ctx->out_order_1p= %d", ctx->out_order_1p);
}

void cameric_prepare_dma_offset(struct cameric_ctx *ctx, struct cameric_frame *f)
{
	bool pix_hoff = ctx->cameric_dev->drv_data->dma_pix_hoff;
	u32 i, depth = 0;
	printk(KERN_INFO "%s\n", __func__);
	for (i = 0; i < f->fmt->memplanes; i++)
		depth += f->fmt->depth[i];

	f->dma_offset.y_h = f->offs_h;
	if (!pix_hoff)
		f->dma_offset.y_h *= (depth >> 3);

	f->dma_offset.y_v = f->offs_v;

	f->dma_offset.cb_h = f->offs_h;
	f->dma_offset.cb_v = f->offs_v;

	f->dma_offset.cr_h = f->offs_h;
	f->dma_offset.cr_v = f->offs_v;

	if (!pix_hoff) {
		if (f->fmt->colplanes == 3) {
			f->dma_offset.cb_h >>= 1;
			f->dma_offset.cr_h >>= 1;
		}
		if (f->fmt->color == CAMERIC_FMT_YCBCR420) {
			f->dma_offset.cb_v >>= 1;
			f->dma_offset.cr_v >>= 1;
		}
	}

	dbg("in_offset: color= %d, y_h= %d, y_v= %d",
	    f->fmt->color, f->dma_offset.y_h, f->dma_offset.y_v);
}

static int cameric_set_color_effect(struct cameric_ctx *ctx, enum v4l2_colorfx colorfx)
{
	struct cameric_effect *effect = &ctx->effect;
	printk(KERN_INFO "%s\n", __func__);
	switch (colorfx) {
	case V4L2_COLORFX_NONE:
		effect->type = CAMERIC_REG_CIIMGEFF_FIN_BYPASS;
		break;
	case V4L2_COLORFX_BW:
		effect->type = CAMERIC_REG_CIIMGEFF_FIN_ARBITRARY;
		effect->pat_cb = 128;
		effect->pat_cr = 128;
		break;
	case V4L2_COLORFX_SEPIA:
		effect->type = CAMERIC_REG_CIIMGEFF_FIN_ARBITRARY;
		effect->pat_cb = 115;
		effect->pat_cr = 145;
		break;
	case V4L2_COLORFX_NEGATIVE:
		effect->type = CAMERIC_REG_CIIMGEFF_FIN_NEGATIVE;
		break;
	case V4L2_COLORFX_EMBOSS:
		effect->type = CAMERIC_REG_CIIMGEFF_FIN_EMBOSSING;
		break;
	case V4L2_COLORFX_ART_FREEZE:
		effect->type = CAMERIC_REG_CIIMGEFF_FIN_ARTFREEZE;
		break;
	case V4L2_COLORFX_SILHOUETTE:
		effect->type = CAMERIC_REG_CIIMGEFF_FIN_SILHOUETTE;
		break;
	case V4L2_COLORFX_SET_CBCR:
		effect->type = CAMERIC_REG_CIIMGEFF_FIN_ARBITRARY;
		effect->pat_cb = ctx->ctrls.colorfx_cbcr->val >> 8;
		effect->pat_cr = ctx->ctrls.colorfx_cbcr->val & 0xff;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/*
 * V4L2 controls handling
 */
#define ctrl_to_ctx(__ctrl) \
	container_of((__ctrl)->handler, struct cameric_ctx, ctrls.handler)

static int __cameric_s_ctrl(struct cameric_ctx *ctx, struct v4l2_ctrl *ctrl)
{
	struct cameric_dev *cameric = ctx->cameric_dev;
	const struct cameric_variant *variant = cameric->variant;
	int ret = 0;
	printk(KERN_INFO "%s\n", __func__);
	if (ctrl->flags & V4L2_CTRL_FLAG_INACTIVE)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_HFLIP:
		ctx->hflip = ctrl->val;
		break;

	case V4L2_CID_VFLIP:
		ctx->vflip = ctrl->val;
		break;

	case V4L2_CID_ROTATE:
		if (cameric_capture_pending(cameric)) {
			ret = cameric_check_scaler_ratio(ctx, ctx->s_frame.width,
					ctx->s_frame.height, ctx->d_frame.width,
					ctx->d_frame.height, ctrl->val);
			if (ret)
				return -EINVAL;
		}
		if ((ctrl->val == 90 || ctrl->val == 270) &&
		    !variant->has_out_rot)
			return -EINVAL;

		ctx->rotation = ctrl->val;
		break;

	case V4L2_CID_ALPHA_COMPONENT:
		ctx->d_frame.alpha = ctrl->val;
		break;

	case V4L2_CID_COLORFX:
		ret = cameric_set_color_effect(ctx, ctrl->val);
		if (ret)
			return ret;
		break;
	}

	ctx->state |= CAMERIC_PARAMS;
	set_bit(ST_CAPT_APPLY_CFG, &cameric->state);
	return 0;
}

static int cameric_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct cameric_ctx *ctx = ctrl_to_ctx(ctrl);
	unsigned long flags;
	int ret;
	printk(KERN_INFO "%s\n", __func__);
	spin_lock_irqsave(&ctx->cameric_dev->slock, flags);
	ret = __cameric_s_ctrl(ctx, ctrl);
	spin_unlock_irqrestore(&ctx->cameric_dev->slock, flags);

	return ret;
}

static const struct v4l2_ctrl_ops cameric_ctrl_ops = {
	.s_ctrl = cameric_s_ctrl,
};

int cameric_ctrls_create(struct cameric_ctx *ctx)
{
	printk(KERN_INFO "%s 1, ctx=%x\n", __func__, ctx);
	unsigned int max_alpha = cameric_get_alpha_mask(ctx->d_frame.fmt);
	printk(KERN_INFO "%s 2\n", __func__);
	struct cameric_ctrls *ctrls = &ctx->ctrls;
	struct v4l2_ctrl_handler *handler = &ctrls->handler;
	printk(KERN_INFO "%s 3\n", __func__);
	if (ctx->ctrls.ready)
		return 0;
	printk(KERN_INFO "%s 4\n", __func__);
	v4l2_ctrl_handler_init(handler, 6);
	printk(KERN_INFO "%s 5\n", __func__);
	ctrls->rotate = v4l2_ctrl_new_std(handler, &cameric_ctrl_ops,
					V4L2_CID_ROTATE, 0, 270, 90, 0);
	ctrls->hflip = v4l2_ctrl_new_std(handler, &cameric_ctrl_ops,
					V4L2_CID_HFLIP, 0, 1, 1, 0);
	ctrls->vflip = v4l2_ctrl_new_std(handler, &cameric_ctrl_ops,
					V4L2_CID_VFLIP, 0, 1, 1, 0);
	printk(KERN_INFO "%s 6, cameric_dev:%x, \n", __func__, ctx->cameric_dev);
	printk(KERN_INFO "%s 6, drv_data:%x, \n", __func__, ctx->cameric_dev->drv_data);
	if (ctx->cameric_dev->drv_data->alpha_color){
		printk(KERN_INFO "%s 51\n", __func__);
		ctrls->alpha = v4l2_ctrl_new_std(handler, &cameric_ctrl_ops,
					V4L2_CID_ALPHA_COMPONENT,
					0, max_alpha, 1, 0);
	}

	else
		ctrls->alpha = NULL;
	printk(KERN_INFO "%s 7\n", __func__);
	ctrls->colorfx = v4l2_ctrl_new_std_menu(handler, &cameric_ctrl_ops,
				V4L2_CID_COLORFX, V4L2_COLORFX_SET_CBCR,
				~0x983f, V4L2_COLORFX_NONE);
	printk(KERN_INFO "%s 8\n", __func__);
	ctrls->colorfx_cbcr = v4l2_ctrl_new_std(handler, &cameric_ctrl_ops,
				V4L2_CID_COLORFX_CBCR, 0, 0xffff, 1, 0);
	printk(KERN_INFO "%s 9\n", __func__);
	ctx->effect.type = CAMERIC_REG_CIIMGEFF_FIN_BYPASS;
	printk(KERN_INFO "%s 10\n", __func__);
	if (!handler->error) {
		v4l2_ctrl_cluster(2, &ctrls->colorfx);
		ctrls->ready = true;
	}
	printk(KERN_INFO "%s 11\n", __func__);
	return handler->error;
}

void cameric_ctrls_delete(struct cameric_ctx *ctx)
{
	struct cameric_ctrls *ctrls = &ctx->ctrls;
	printk(KERN_INFO "%s\n", __func__);
	if (ctrls->ready) {
		v4l2_ctrl_handler_free(&ctrls->handler);
		ctrls->ready = false;
		ctrls->alpha = NULL;
	}
}

void cameric_ctrls_activate(struct cameric_ctx *ctx, bool active)
{
	unsigned int has_alpha = ctx->d_frame.fmt->flags & FMT_HAS_ALPHA;
	struct cameric_ctrls *ctrls = &ctx->ctrls;
	printk(KERN_INFO "%s\n", __func__);
	if (!ctrls->ready)
		return;

	mutex_lock(ctrls->handler.lock);
	v4l2_ctrl_activate(ctrls->rotate, active);
	v4l2_ctrl_activate(ctrls->hflip, active);
	v4l2_ctrl_activate(ctrls->vflip, active);
	v4l2_ctrl_activate(ctrls->colorfx, active);
	if (ctrls->alpha)
		v4l2_ctrl_activate(ctrls->alpha, active && has_alpha);

	if (active) {
		cameric_set_color_effect(ctx, ctrls->colorfx->cur.val);
		ctx->rotation = ctrls->rotate->val;
		ctx->hflip    = ctrls->hflip->val;
		ctx->vflip    = ctrls->vflip->val;
	} else {
		ctx->effect.type = CAMERIC_REG_CIIMGEFF_FIN_BYPASS;
		ctx->rotation = 0;
		ctx->hflip    = 0;
		ctx->vflip    = 0;
	}
	mutex_unlock(ctrls->handler.lock);
}

/* Update maximum value of the alpha color control */
void cameric_alpha_ctrl_update(struct cameric_ctx *ctx)
{
	struct cameric_dev *cameric = ctx->cameric_dev;
	struct v4l2_ctrl *ctrl = ctx->ctrls.alpha;
	printk(KERN_INFO "%s\n", __func__);
	if (ctrl == NULL || !cameric->drv_data->alpha_color)
		return;

	v4l2_ctrl_lock(ctrl);
	ctrl->maximum = cameric_get_alpha_mask(ctx->d_frame.fmt);

	if (ctrl->cur.val > ctrl->maximum)
		ctrl->cur.val = ctrl->maximum;

	v4l2_ctrl_unlock(ctrl);
}

void __cameric_get_format(struct cameric_frame *frame, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pixm = &f->fmt.pix_mp;
	int i;
	printk(KERN_INFO "%s\n", __func__);
	pixm->width = frame->o_width;
	pixm->height = frame->o_height;
	pixm->field = V4L2_FIELD_NONE;
	pixm->pixelformat = frame->fmt->fourcc;
	pixm->colorspace = V4L2_COLORSPACE_JPEG;
	pixm->num_planes = frame->fmt->memplanes;

	for (i = 0; i < pixm->num_planes; ++i) {
		pixm->plane_fmt[i].bytesperline = frame->bytesperline[i];
		pixm->plane_fmt[i].sizeimage = frame->payload[i];
	}
}

/**
 * cameric_adjust_mplane_format - adjust bytesperline/sizeimage for each plane
 * @fmt: cameric pixel format description (input)
 * @width: requested pixel width
 * @height: requested pixel height
 * @pix: multi-plane format to adjust
 */
void cameric_adjust_mplane_format(struct cameric_fmt *fmt, u32 width, u32 height,
			       struct v4l2_pix_format_mplane *pix)
{
	u32 bytesperline = 0;
	int i;
	printk(KERN_INFO "%s\n", __func__);
	pix->colorspace	= V4L2_COLORSPACE_JPEG;
	pix->field = V4L2_FIELD_NONE;
	pix->num_planes = fmt->memplanes;
	pix->pixelformat = fmt->fourcc;
	pix->height = height;
	pix->width = width;

	for (i = 0; i < pix->num_planes; ++i) {
		struct v4l2_plane_pix_format *plane_fmt = &pix->plane_fmt[i];
		u32 bpl = plane_fmt->bytesperline;
		u32 sizeimage;

		if (fmt->colplanes > 1 && (bpl == 0 || bpl < pix->width))
			bpl = pix->width; /* Planar */

		if (fmt->colplanes == 1 && /* Packed */
		    (bpl == 0 || ((bpl * 8) / fmt->depth[i]) < pix->width))
			bpl = (pix->width * fmt->depth[0]) / 8;
		/*
		 * Currently bytesperline for each plane is same, except
		 * V4L2_PIX_FMT_YUV420M format. This calculation may need
		 * to be changed when other multi-planar formats are added
		 * to the cameric_formats[] array.
		 */
		if (i == 0)
			bytesperline = bpl;
		else if (i == 1 && fmt->memplanes == 3)
			bytesperline /= 2;

		plane_fmt->bytesperline = bytesperline;
		sizeimage = pix->width * pix->height * fmt->depth[i] / 8;

		/* Ensure full last row for tiled formats */
		if (tiled_fmt(fmt)) {
			/* 64 * 32 * plane_fmt->bytesperline / 64 */
			u32 row_size = plane_fmt->bytesperline * 32;

			sizeimage = roundup(sizeimage, row_size);
		}

		plane_fmt->sizeimage = max(sizeimage, plane_fmt->sizeimage);
	}
}

/**
 * cameric_find_format - lookup cameric color format by fourcc or media bus format
 * @pixelformat: fourcc to match, ignored if null
 * @mbus_code: media bus code to match, ignored if null
 * @mask: the color flags to match
 * @index: offset in the cameric_formats array, ignored if negative
 */
struct cameric_fmt *cameric_find_format(const u32 *pixelformat, const u32 *mbus_code,
				  unsigned int mask, int index)
{
	struct cameric_fmt *fmt, *def_fmt = NULL;
	unsigned int i;
	int id = 0;
	printk(KERN_INFO "%s\n", __func__);
	if (index >= (int)ARRAY_SIZE(cameric_formats))
		return NULL;

	for (i = 0; i < ARRAY_SIZE(cameric_formats); ++i) {
		fmt = &cameric_formats[i];
		if (!(fmt->flags & mask))
			continue;
		if (pixelformat && fmt->fourcc == *pixelformat)
			return fmt;
		if (mbus_code && fmt->mbus_code == *mbus_code)
			return fmt;
		if (index == id)
			def_fmt = fmt;
		id++;
	}
	return def_fmt;
}

#ifdef CONFIG_PM
static int cameric_m2m_suspend(struct cameric_dev *cameric)
{
	unsigned long flags;
	int timeout;
	printk(KERN_INFO "%s\n", __func__);
	spin_lock_irqsave(&cameric->slock, flags);
	if (!cameric_m2m_pending(cameric)) {
		spin_unlock_irqrestore(&cameric->slock, flags);
		return 0;
	}
	clear_bit(ST_M2M_SUSPENDED, &cameric->state);
	set_bit(ST_M2M_SUSPENDING, &cameric->state);
	spin_unlock_irqrestore(&cameric->slock, flags);

	timeout = wait_event_timeout(cameric->irq_queue,
			     test_bit(ST_M2M_SUSPENDED, &cameric->state),
				 CAMERIC_SHUTDOWN_TIMEOUT);

	clear_bit(ST_M2M_SUSPENDING, &cameric->state);
	return timeout == 0 ? -EAGAIN : 0;
}

static int cameric_m2m_resume(struct cameric_dev *cameric)
{
	struct cameric_ctx *ctx;
	unsigned long flags;
	printk(KERN_INFO "%s\n", __func__);
	spin_lock_irqsave(&cameric->slock, flags);
	/* Clear for full H/W setup in first run after resume */
	ctx = cameric->m2m.ctx;
	cameric->m2m.ctx = NULL;
	spin_unlock_irqrestore(&cameric->slock, flags);

	if (test_and_clear_bit(ST_M2M_SUSPENDED, &cameric->state))
		cameric_m2m_job_finish(ctx, VB2_BUF_STATE_ERROR);

	return 0;
}
#endif /* CONFIG_PM */




struct cameric_clk_rk3288 {
	struct clk	*aclk_isp;
	struct clk	*hclk_isp;
	struct clk	*sclk_isp;
	struct clk	*sclk_isp_jpe;
	struct clk *sclk_mipidsi_24m;
	struct clk *pclk_mipi_csi;
	struct clk *pclk_isp_in;
	struct reset_control *isp_rst;
};

static int rk3288_clk_enable(struct cameric_clk_rk3288 *clk_rst)
{
	printk(KERN_INFO "%s\n", __func__);
	clk_prepare_enable(clk_rst->hclk_isp);
	clk_prepare_enable(clk_rst->aclk_isp);
	clk_prepare_enable(clk_rst->sclk_isp);
	clk_prepare_enable(clk_rst->sclk_isp_jpe);
	clk_prepare_enable(clk_rst->sclk_mipidsi_24m);
	clk_prepare_enable(clk_rst->pclk_isp_in);
	clk_prepare_enable(clk_rst->pclk_mipi_csi);
	return 0;
}

static int rk3288_clk_disable(struct cameric_clk_rk3288 *clk_rst)
{
	printk(KERN_INFO "%s\n", __func__);
	clk_disable_unprepare(clk_rst->hclk_isp);
	clk_disable_unprepare(clk_rst->aclk_isp);
	clk_disable_unprepare(clk_rst->sclk_isp);
	clk_disable_unprepare(clk_rst->sclk_isp_jpe);
	clk_disable_unprepare(clk_rst->sclk_mipidsi_24m);
	clk_disable_unprepare(clk_rst->pclk_isp_in);
	clk_disable_unprepare(clk_rst->pclk_mipi_csi);
	return 0;
}

static struct cameric_clk_rk3288 *cameric_clk_get_rk3288(struct device *dev)
{
	struct cameric_clk_rk3288 *clk_rst;

	printk(KERN_INFO "%s\n", __func__);
	clk_rst = (struct cameric_clk_rk3288 *)devm_kzalloc(dev,
			sizeof(struct cameric_clk_rk3288), GFP_KERNEL);
	if (!clk_rst) {
		dev_err(dev, "Can't allocate cameric_rk3288\n");
		return -ENOMEM;
	}

	clk_rst->aclk_isp = devm_clk_get(dev, "aclk_isp");
	clk_rst->hclk_isp = devm_clk_get(dev, "hclk_isp");
	clk_rst->sclk_isp = devm_clk_get(dev, "sclk_isp");
	clk_rst->sclk_isp_jpe = devm_clk_get(dev, "sclk_isp_jpe");
	clk_rst->sclk_mipidsi_24m = devm_clk_get(dev, "sclk_mipidsi_24m");
	clk_rst->pclk_mipi_csi = devm_clk_get(dev, "pclk_mipi_csi");
	clk_rst->isp_rst = devm_reset_control_get(dev, "rst_isp");
	clk_rst->pclk_isp_in = devm_clk_get(dev, "pclk_isp_in");

	if (IS_ERR_OR_NULL(clk_rst->aclk_isp) ||
		IS_ERR_OR_NULL(clk_rst->hclk_isp) ||
		IS_ERR_OR_NULL(clk_rst->sclk_isp) ||
		IS_ERR_OR_NULL(clk_rst->sclk_isp_jpe) ||
		IS_ERR_OR_NULL(clk_rst->pclk_mipi_csi) ||
		IS_ERR_OR_NULL(clk_rst->isp_rst) ||
		IS_ERR_OR_NULL(clk_rst->pclk_isp_in) ||
		IS_ERR_OR_NULL(clk_rst->sclk_mipidsi_24m)) {
		dev_err(dev, "Get rk3288 clock resouce failed !\n");

		if (!IS_ERR_OR_NULL(clk_rst->aclk_isp))
			devm_clk_put(dev, clk_rst->aclk_isp);
		if (!IS_ERR_OR_NULL(clk_rst->hclk_isp))
			devm_clk_put(dev, clk_rst->hclk_isp);
		if (!IS_ERR_OR_NULL(clk_rst->sclk_isp))
			devm_clk_put(dev, clk_rst->sclk_isp);
		if (!IS_ERR_OR_NULL(clk_rst->sclk_isp_jpe))
			devm_clk_put(dev, clk_rst->sclk_isp_jpe);
		if (!IS_ERR_OR_NULL(clk_rst->pclk_mipi_csi))
			devm_clk_put(dev, clk_rst->pclk_mipi_csi);
		if (!IS_ERR_OR_NULL(clk_rst->pclk_isp_in))
			devm_clk_put(dev, clk_rst->pclk_isp_in);
		if (!IS_ERR_OR_NULL(clk_rst->sclk_mipidsi_24m))
			devm_clk_put(dev, clk_rst->sclk_mipidsi_24m);
		if (!IS_ERR_OR_NULL(clk_rst->isp_rst))
			reset_control_put(clk_rst->isp_rst);

		return -EINVAL;
	}

	clk_set_rate(clk_rst->sclk_isp, 400000000);
	clk_set_rate(clk_rst->sclk_isp_jpe, 400000000);
	reset_control_deassert(clk_rst->isp_rst);
	return clk_rst;
}

static int cameric_clk_put_rk3288(struct device *dev,
									struct cameric_clk_rk3288 *clk_rst)
{
	printk(KERN_INFO "%s\n", __func__);
	if (!IS_ERR_OR_NULL(clk_rst->aclk_isp))
		devm_clk_put(dev, clk_rst->aclk_isp);
	if (!IS_ERR_OR_NULL(clk_rst->hclk_isp))
		devm_clk_put(dev, clk_rst->hclk_isp);
	if (!IS_ERR_OR_NULL(clk_rst->sclk_isp))
		devm_clk_put(dev, clk_rst->sclk_isp);
	if (!IS_ERR_OR_NULL(clk_rst->sclk_isp_jpe))
		devm_clk_put(dev, clk_rst->sclk_isp_jpe);
	if (!IS_ERR_OR_NULL(clk_rst->pclk_mipi_csi))
		devm_clk_put(dev, clk_rst->pclk_mipi_csi);
	if (!IS_ERR_OR_NULL(clk_rst->pclk_isp_in))
		devm_clk_put(dev, clk_rst->pclk_isp_in);
	if (!IS_ERR_OR_NULL(clk_rst->sclk_mipidsi_24m))
		devm_clk_put(dev, clk_rst->sclk_mipidsi_24m);
	if (!IS_ERR_OR_NULL(clk_rst->isp_rst))
		reset_control_put(clk_rst->isp_rst);
	return 0;
}

static void cameric_clk_put(struct cameric_dev *cameric)
{
	struct platform_device *pdev = cameric->pdev;
	struct device *dev = &pdev->dev;

	printk(KERN_INFO "%s\n", __func__);
	cameric_clk_put_rk3288(dev, cameric->clk_rst);
	rk3288_clk_disable(cameric->clk_rst);
}

static int cameric_clk_get(struct cameric_dev *cameric)
{
	struct platform_device *pdev = cameric->pdev;
	struct device *dev = &pdev->dev;
	printk(KERN_INFO "%s\n", __func__);

	cameric->clk_rst = cameric_clk_get_rk3288(dev);
	if(IS_ERR_OR_NULL(cameric->clk_rst)){
		cameric_clk_put(cameric);
		return -ENXIO;
	}

	rk3288_clk_enable(cameric->clk_rst);
	return 0;
}




static const struct of_device_id cameric_of_match[];

static int cameric_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node, *node;
	u32 lclk_freq = 0;
	struct cameric_dev *cameric;
	struct resource *res;
	const struct of_device_id *of_id;
	int ret = 0;
	printk(KERN_INFO "%s 1\n", __func__);
	cameric = devm_kzalloc(dev, sizeof(*cameric), GFP_KERNEL);
	if (!cameric)
		return -ENOMEM;

	cameric->pdev = pdev;




	printk(KERN_INFO "%s 2, np->name=%s\n", __func__, np->name);
	node = of_parse_phandle(np, "rockchip,grf", 0);
	if(!node)
		return -ENODEV;
	printk(KERN_INFO "%s 3, node name:%s\n", __func__, node->name);
	cameric->regmap_grf = syscon_node_to_regmap(node);
	if (IS_ERR_OR_NULL(cameric->regmap_grf))
		return PTR_ERR(cameric->regmap_grf);


	printk(KERN_INFO "%s 4\n", __func__);
	res = platform_get_resource_byname(pdev,
				IORESOURCE_MEM, "csihost-register");
	if (!res)
		return -EINVAL;
	printk(KERN_INFO "%s 5\n", __func__);
	cameric->csihost_base = devm_ioremap_resource(dev, res);
	if (IS_ERR_OR_NULL(cameric->csihost_base))
		return PTR_ERR(cameric->csihost_base);


	printk(KERN_INFO "%s 6\n", __func__);
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "register");
	if (!res)
		return -EINVAL;
	printk(KERN_INFO "%s 7\n", __func__);
	cameric->base_addr = devm_ioremap_resource(dev, res);
	if (IS_ERR_OR_NULL(cameric->base_addr))
		return PTR_ERR(cameric->base_addr);
	printk(KERN_INFO "%s 8\n", __func__);

	of_id = of_match_node(cameric_of_match, np);
	if (!of_id)
		return -EINVAL;
	cameric->drv_data = of_id->data;
	//cameric->id = of_alias_get_id(np, "camera");
	//if (!dev->of_node) {
	//	cameric->drv_data = cameric_get_drvdata(pdev);
	//	cameric->id = pdev->id;
	//}

	if (!cameric->drv_data /*|| cameric->id >= cameric->drv_data->num_entities ||
			cameric->id < 0*/) {
		dev_err(dev, "Invalid driver data or device id (%d)\n",
				cameric->id);
		return -EINVAL;
	}
	printk(KERN_INFO "%s 9\n", __func__);
	if (!dev->of_node)
		cameric->variant = cameric->drv_data->variant[cameric->id];
	printk(KERN_INFO "%s 10\n", __func__);




	init_waitqueue_head(&cameric->irq_queue);
	spin_lock_init(&cameric->slock);
	mutex_init(&cameric->lock);

	ret = cameric_clk_get(cameric);
	if (ret)
		return ret;
	printk(KERN_INFO "%s 11\n", __func__);



	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res == NULL) {
		dev_err(dev, "Failed to get IRQ resource\n");
		return -ENXIO;
	}
	printk(KERN_INFO "%s 12\n", __func__);
	cameric->irq = platform_get_irq_byname(pdev, "cif_isp10_irq");
	if (IS_ERR_VALUE(cameric->irq)) {
		dev_err(&pdev->dev, "unable to get IRQ(error %d)\n", ret);

	}

	printk(KERN_INFO "%s 121\n", __func__);
	ret = devm_request_threaded_irq(dev,
			cameric->irq,
			cameric_irq_handler,
			NULL,
			0,
			dev_name(dev),
			cameric);



	//ret = devm_request_irq(dev, res->start, cameric_irq_handler,
		//	       0, dev_name(dev), cameric);
	if (ret < 0) {
		dev_err(dev, "failed to install irq (%d)\n", ret);
		goto err_sclk;
	}
	printk(KERN_INFO "%s 13\n", __func__);
	ret = cameric_initialize_capture_subdev(cameric);
	if (ret < 0)
		goto err_sclk;
	printk(KERN_INFO "%s 14\n", __func__);
	platform_set_drvdata(pdev, cameric);
	pm_runtime_enable(dev);
	printk(KERN_INFO "%s 15\n", __func__);
	if (!pm_runtime_enabled(dev)) {
		ret = clk_enable(cameric->clock[CLK_GATE]);
		if (ret < 0)
			goto err_sd;
	}
	printk(KERN_INFO "%s 16\n", __func__);
	//vb2_dma_contig_set_max_seg_size(dev, DMA_BIT_MASK(32));

	dev_dbg(dev, "cameric.%d registered successfully\n", cameric->id);
	return 0;
	printk(KERN_INFO "%s 17\n", __func__);
err_sd:
	cameric_unregister_capture_subdev(cameric);
err_sclk:
	clk_disable(cameric->clock[CLK_BUS]);
	cameric_clk_put(cameric);
	return ret;
}

#ifdef CONFIG_PM
static int cameric_runtime_resume(struct device *dev)
{
	struct cameric_dev *cameric =	dev_get_drvdata(dev);
	printk(KERN_INFO "%s\n", __func__);
	dbg("cameric%d: state: 0x%lx", cameric->id, cameric->state);

	/* Enable clocks and perform basic initialization */
	clk_enable(cameric->clock[CLK_GATE]);
	cameric_hw_reset(cameric);

	/* Resume the capture or mem-to-mem device */
	if (cameric_capture_busy(cameric))
		return cameric_capture_resume(cameric);

	return cameric_m2m_resume(cameric);
}

static int cameric_runtime_suspend(struct device *dev)
{
	struct cameric_dev *cameric =	dev_get_drvdata(dev);
	int ret = 0;
	printk(KERN_INFO "%s\n", __func__);
	if (cameric_capture_busy(cameric))
		ret = cameric_capture_suspend(cameric);
	else
		ret = cameric_m2m_suspend(cameric);
	if (!ret)
		clk_disable(cameric->clock[CLK_GATE]);

	dbg("cameric%d: state: 0x%lx", cameric->id, cameric->state);
	return ret;
}
#endif

#ifdef CONFIG_PM_SLEEP
static int cameric_resume(struct device *dev)
{
	struct cameric_dev *cameric =	dev_get_drvdata(dev);
	unsigned long flags;
	printk(KERN_INFO "%s\n", __func__);
	dbg("cameric%d: state: 0x%lx", cameric->id, cameric->state);

	/* Do not resume if the device was idle before system suspend */
	spin_lock_irqsave(&cameric->slock, flags);
	if (!test_and_clear_bit(ST_LPM, &cameric->state) ||
	    (!cameric_m2m_active(cameric) && !cameric_capture_busy(cameric))) {
		spin_unlock_irqrestore(&cameric->slock, flags);
		return 0;
	}
	cameric_hw_reset(cameric);
	spin_unlock_irqrestore(&cameric->slock, flags);

	if (cameric_capture_busy(cameric))
		return cameric_capture_resume(cameric);

	return cameric_m2m_resume(cameric);
}

static int cameric_suspend(struct device *dev)
{
	struct cameric_dev *cameric =	dev_get_drvdata(dev);
	printk(KERN_INFO "%s\n", __func__);
	dbg("cameric%d: state: 0x%lx", cameric->id, cameric->state);

	if (test_and_set_bit(ST_LPM, &cameric->state))
		return 0;
	if (cameric_capture_busy(cameric))
		return cameric_capture_suspend(cameric);

	return cameric_m2m_suspend(cameric);
}
#endif /* CONFIG_PM_SLEEP */

static int cameric_remove(struct platform_device *pdev)
{
	struct cameric_dev *cameric = platform_get_drvdata(pdev);
	printk(KERN_INFO "%s\n", __func__);
	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		clk_disable(cameric->clock[CLK_GATE]);
	pm_runtime_set_suspended(&pdev->dev);

	cameric_unregister_capture_subdev(cameric);
	//vb2_dma_contig_clear_max_seg_size(&pdev->dev);

	clk_disable(cameric->clock[CLK_BUS]);
	cameric_clk_put(cameric);

	dev_info(&pdev->dev, "driver unloaded\n");
	return 0;
}

/* Image pixel limits, similar across several cameric HW revisions. */
static const struct cameric_pix_limit s5p_pix_limit[4] = {
	[0] = {
		.scaler_en_w	= 3264,
		.scaler_dis_w	= 8192,
		.out_rot_en_w	= 1920,
		.out_rot_dis_w	= 4224,
	},
	[1] = {
		.scaler_en_w	= 4224,
		.scaler_dis_w	= 8192,
		.out_rot_en_w	= 1920,
		.out_rot_dis_w	= 4224,
	},
	[2] = {
		.scaler_en_w	= 1920,
		.scaler_dis_w	= 8192,
		.out_rot_en_w	= 1280,
		.out_rot_dis_w	= 1920,
	},
};

static const struct cameric_variant cameric0_variant_s5pv210 = {
	.has_inp_rot	 = 1,
	.has_out_rot	 = 1,
	.has_cam_if	 = 1,
	.min_inp_pixsize = 16,
	.min_out_pixsize = 16,
	.hor_offs_align	 = 8,
	.min_vsize_align = 16,
	.pix_limit	 = &s5p_pix_limit[1],
};

static const struct cameric_variant cameric1_variant_s5pv210 = {
	.has_inp_rot	 = 1,
	.has_out_rot	 = 1,
	.has_cam_if	 = 1,
	.has_mainscaler_ext = 1,
	.min_inp_pixsize = 16,
	.min_out_pixsize = 16,
	.hor_offs_align	 = 1,
	.min_vsize_align = 1,
	.pix_limit	 = &s5p_pix_limit[2],
};

static const struct cameric_variant cameric2_variant_s5pv210 = {
	.has_cam_if	 = 1,
	.min_inp_pixsize = 16,
	.min_out_pixsize = 16,
	.hor_offs_align	 = 8,
	.min_vsize_align = 16,
	.pix_limit	 = &s5p_pix_limit[2],
};

static const struct cameric_drvdata cameric_drvdata_rk3399 = {
	.variant = {
		[0] = &cameric0_variant_s5pv210,
		[1] = &cameric1_variant_s5pv210,
		[2] = &cameric2_variant_s5pv210,
	},
	.num_entities	= 3,
	.lclk_frequency	= 166000000UL,
	.out_buf_count	= 4,
	.dma_pix_hoff	= 1,
};

static const struct cameric_drvdata cameric_drvdata_rk3288 = {
	.num_entities	= 4,
	.lclk_frequency = 166000000UL,
	.dma_pix_hoff	= 1,
	.cistatus2	= 1,
	.alpha_color	= 1,
	.out_buf_count	= 32,
};

static const struct of_device_id cameric_of_match[] = {
	{
		.compatible = "cameric,rk3288",
		.data = &cameric_drvdata_rk3288,
	},	{
		.compatible = "rockchip,rk3399-cameric",
		.data = &cameric_drvdata_rk3399,
	},
	{ /* sentinel */ },
};

static const struct dev_pm_ops cameric_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(cameric_suspend, cameric_resume)
	SET_RUNTIME_PM_OPS(cameric_runtime_suspend, cameric_runtime_resume, NULL)
};

static struct platform_driver cameric_driver = {
	.probe		= cameric_probe,
	.remove		= cameric_remove,
	.driver = {
		.of_match_table = cameric_of_match,
		.name		= CAMERIC_DRIVER_NAME,
		.pm     	= &cameric_pm_ops,
	}
};

int __init cameric_register_driver(void)
{printk(KERN_INFO "%s\n", __func__);
	return platform_driver_register(&cameric_driver);
}

void __exit cameric_unregister_driver(void)
{printk(KERN_INFO "%s\n", __func__);
	platform_driver_unregister(&cameric_driver);
}
