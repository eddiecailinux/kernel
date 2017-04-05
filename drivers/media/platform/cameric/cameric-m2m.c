/*
 * Samsung S5P/EXYNOS4 SoC series CAMERIC (video postprocessor) driver
 *
 * Copyright (C) 2012 - 2013 Samsung Electronics Co., Ltd.
 * Sylwester Nawrocki <s.nawrocki@samsung.com>
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
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>

//#include "common.h"
#include "cameric-core.h"
#include "cameric-reg.h"
#include "media-dev.h"

static unsigned int get_m2m_fmt_flags(unsigned int stream_type)
{
	if (stream_type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return FMT_FLAGS_M2M_IN;
	else
		return FMT_FLAGS_M2M_OUT;
}

void cameric_m2m_job_finish(struct cameric_ctx *ctx, int vb_state)
{
	struct vb2_v4l2_buffer *src_vb, *dst_vb;

	if (!ctx || !ctx->fh.m2m_ctx)
		return;

	src_vb = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
	dst_vb = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);

	if (src_vb)
		v4l2_m2m_buf_done(src_vb, vb_state);
	if (dst_vb)
		v4l2_m2m_buf_done(dst_vb, vb_state);
	if (src_vb && dst_vb)
		v4l2_m2m_job_finish(ctx->cameric_dev->m2m.m2m_dev,
				    ctx->fh.m2m_ctx);
}

/* Complete the transaction which has been scheduled for execution. */
static void cameric_m2m_shutdown(struct cameric_ctx *ctx)
{
	struct cameric_dev *cameric = ctx->cameric_dev;

	if (!cameric_m2m_pending(cameric))
		return;

	cameric_ctx_state_set(CAMERIC_CTX_SHUT, ctx);

	wait_event_timeout(cameric->irq_queue,
			!cameric_ctx_state_is_set(CAMERIC_CTX_SHUT, ctx),
			CAMERIC_SHUTDOWN_TIMEOUT);
}

static int start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct cameric_ctx *ctx = q->drv_priv;
	int ret;

	ret = pm_runtime_get_sync(&ctx->cameric_dev->pdev->dev);
	return ret > 0 ? 0 : ret;
}

static void stop_streaming(struct vb2_queue *q)
{
	struct cameric_ctx *ctx = q->drv_priv;


	cameric_m2m_shutdown(ctx);
	cameric_m2m_job_finish(ctx, VB2_BUF_STATE_ERROR);
	pm_runtime_put(&ctx->cameric_dev->pdev->dev);
}

static void cameric_device_run(void *priv)
{
	struct vb2_v4l2_buffer *src_vb, *dst_vb;
	struct cameric_ctx *ctx = priv;
	struct cameric_frame *sf, *df;
	struct cameric_dev *cameric;
	unsigned long flags;
	int ret;

	if (WARN(!ctx, "Null context\n"))
		return;

	cameric = ctx->cameric_dev;
	spin_lock_irqsave(&cameric->slock, flags);

	set_bit(ST_M2M_PEND, &cameric->state);
	sf = &ctx->s_frame;
	df = &ctx->d_frame;

	if (ctx->state & CAMERIC_PARAMS) {
		/* Prepare the DMA offsets for scaler */
		cameric_prepare_dma_offset(ctx, sf);
		cameric_prepare_dma_offset(ctx, df);
	}

	src_vb = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	ret = cameric_prepare_addr(ctx, &src_vb->vb2_buf, sf, &sf->paddr);
	if (ret)
		goto dma_unlock;

	dst_vb = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	ret = cameric_prepare_addr(ctx, &dst_vb->vb2_buf, df, &df->paddr);
	if (ret)
		goto dma_unlock;

	dst_vb->vb2_buf.timestamp = src_vb->vb2_buf.timestamp;
	dst_vb->flags &= ~V4L2_BUF_FLAG_TSTAMP_SRC_MASK;
	dst_vb->flags |=
		src_vb->flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK;

	/* Reconfigure hardware if the context has changed. */
	if (cameric->m2m.ctx != ctx) {
		ctx->state |= CAMERIC_PARAMS;
		cameric->m2m.ctx = ctx;
	}

	if (ctx->state & CAMERIC_PARAMS) {
		cameric_set_yuv_order(ctx);
		cameric_hw_set_input_path(ctx);
		cameric_hw_set_in_dma(ctx);
		ret = cameric_set_scaler_info(ctx);
		if (ret)
			goto dma_unlock;
		cameric_hw_set_prescaler(ctx);
		cameric_hw_set_mainscaler(ctx);
		cameric_hw_set_target_format(ctx);
		cameric_hw_set_rotation(ctx);
		cameric_hw_set_effect(ctx);
		cameric_hw_set_out_dma(ctx);
		if (cameric->drv_data->alpha_color)
			cameric_hw_set_rgb_alpha(ctx);
		cameric_hw_set_output_path(ctx);
	}
	cameric_hw_set_input_addr(cameric, &sf->paddr);
	cameric_hw_set_output_addr(cameric, &df->paddr, -1);

	cameric_activate_capture(ctx);
	ctx->state &= (CAMERIC_CTX_M2M | CAMERIC_CTX_CAP);
	cameric_hw_activate_input_dma(cameric, true);

dma_unlock:
	spin_unlock_irqrestore(&cameric->slock, flags);
}

static void cameric_job_abort(void *priv)
{
	cameric_m2m_shutdown(priv);
}

static int cameric_queue_setup(struct vb2_queue *vq,
			    unsigned int *num_buffers, unsigned int *num_planes,
			    unsigned int sizes[], struct device *alloc_devs[])
{
	struct cameric_ctx *ctx = vb2_get_drv_priv(vq);
	struct cameric_frame *f;
	int i;

	f = ctx_get_frame(ctx, vq->type);
	if (IS_ERR(f))
		return PTR_ERR(f);
	/*
	 * Return number of non-contiguous planes (plane buffers)
	 * depending on the configured color format.
	 */
	if (!f->fmt)
		return -EINVAL;

	*num_planes = f->fmt->memplanes;
	for (i = 0; i < f->fmt->memplanes; i++)
		sizes[i] = f->payload[i];
	return 0;
}

static int cameric_buf_prepare(struct vb2_buffer *vb)
{
	struct cameric_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct cameric_frame *frame;
	int i;

	frame = ctx_get_frame(ctx, vb->vb2_queue->type);
	if (IS_ERR(frame))
		return PTR_ERR(frame);

	for (i = 0; i < frame->fmt->memplanes; i++)
		vb2_set_plane_payload(vb, i, frame->payload[i]);

	return 0;
}

static void cameric_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct cameric_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}

static const struct vb2_ops cameric_qops = {
	.queue_setup	 = cameric_queue_setup,
	.buf_prepare	 = cameric_buf_prepare,
	.buf_queue	 = cameric_buf_queue,
	.wait_prepare	 = vb2_ops_wait_prepare,
	.wait_finish	 = vb2_ops_wait_finish,
	.stop_streaming	 = stop_streaming,
	.start_streaming = start_streaming,
};

/*
 * V4L2 ioctl handlers
 */
extern void __cameric_vidioc_querycap(struct device *dev, struct v4l2_capability *cap,
						unsigned int caps);
static int cameric_m2m_querycap(struct file *file, void *fh,
				     struct v4l2_capability *cap)
{
	struct cameric_dev *cameric = video_drvdata(file);
	unsigned int caps;

	/*
	 * This is only a mem-to-mem video device. The capture and output
	 * device capability flags are left only for backward compatibility
	 * and are scheduled for removal.
	 */
	caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_M2M_MPLANE |
		V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_VIDEO_OUTPUT_MPLANE;

	__cameric_vidioc_querycap(&cameric->pdev->dev, cap, caps);
	return 0;
}

static int cameric_m2m_enum_fmt_mplane(struct file *file, void *priv,
				    struct v4l2_fmtdesc *f)
{
	struct cameric_fmt *fmt;

	fmt = cameric_find_format(NULL, NULL, get_m2m_fmt_flags(f->type),
			       f->index);
	if (!fmt)
		return -EINVAL;

	strncpy(f->description, fmt->name, sizeof(f->description) - 1);
	f->pixelformat = fmt->fourcc;
	return 0;
}

static int cameric_m2m_g_fmt_mplane(struct file *file, void *fh,
				 struct v4l2_format *f)
{
	struct cameric_ctx *ctx = fh_to_ctx(fh);
	struct cameric_frame *frame = ctx_get_frame(ctx, f->type);

	if (IS_ERR(frame))
		return PTR_ERR(frame);

	__cameric_get_format(frame, f);
	return 0;
}

static int cameric_try_fmt_mplane(struct cameric_ctx *ctx, struct v4l2_format *f)
{
	struct cameric_dev *cameric = ctx->cameric_dev;
	const struct cameric_variant *variant = cameric->variant;
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	struct cameric_fmt *fmt;
	u32 max_w, mod_x, mod_y;

	if (!IS_M2M(f->type))
		return -EINVAL;

	fmt = cameric_find_format(&pix->pixelformat, NULL,
			       get_m2m_fmt_flags(f->type), 0);
	if (WARN(fmt == NULL, "Pixel format lookup failed"))
		return -EINVAL;

	if (pix->field == V4L2_FIELD_ANY)
		pix->field = V4L2_FIELD_NONE;
	else if (pix->field != V4L2_FIELD_NONE)
		return -EINVAL;

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		max_w = variant->pix_limit->scaler_dis_w;
		mod_x = ffs(variant->min_inp_pixsize) - 1;
	} else {
		max_w = variant->pix_limit->out_rot_dis_w;
		mod_x = ffs(variant->min_out_pixsize) - 1;
	}

	if (tiled_fmt(fmt)) {
		mod_x = 6; /* 64 x 32 pixels tile */
		mod_y = 5;
	} else {
		if (variant->min_vsize_align == 1)
			mod_y = cameric_fmt_is_rgb(fmt->color) ? 0 : 1;
		else
			mod_y = ffs(variant->min_vsize_align) - 1;
	}

	v4l_bound_align_image(&pix->width, 16, max_w, mod_x,
		&pix->height, 8, variant->pix_limit->scaler_dis_w, mod_y, 0);

	cameric_adjust_mplane_format(fmt, pix->width, pix->height, &f->fmt.pix_mp);
	return 0;
}

static int cameric_m2m_try_fmt_mplane(struct file *file, void *fh,
				   struct v4l2_format *f)
{
	struct cameric_ctx *ctx = fh_to_ctx(fh);
	return cameric_try_fmt_mplane(ctx, f);
}

static void __set_frame_format(struct cameric_frame *frame, struct cameric_fmt *fmt,
			       struct v4l2_pix_format_mplane *pixm)
{
	int i;

	for (i = 0; i < fmt->memplanes; i++) {
		frame->bytesperline[i] = pixm->plane_fmt[i].bytesperline;
		frame->payload[i] = pixm->plane_fmt[i].sizeimage;
	}

	frame->f_width = pixm->width;
	frame->f_height	= pixm->height;
	frame->o_width = pixm->width;
	frame->o_height = pixm->height;
	frame->width = pixm->width;
	frame->height = pixm->height;
	frame->offs_h = 0;
	frame->offs_v = 0;
	frame->fmt = fmt;
}

static int cameric_m2m_s_fmt_mplane(struct file *file, void *fh,
				 struct v4l2_format *f)
{
	struct cameric_ctx *ctx = fh_to_ctx(fh);
	struct cameric_dev *cameric = ctx->cameric_dev;
	struct cameric_fmt *fmt;
	struct vb2_queue *vq;
	struct cameric_frame *frame;
	int ret;

	ret = cameric_try_fmt_mplane(ctx, f);
	if (ret)
		return ret;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);

	if (vb2_is_busy(vq)) {
		v4l2_err(&cameric->m2m.vfd, "queue (%d) busy\n", f->type);
		return -EBUSY;
	}

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		frame = &ctx->s_frame;
	else
		frame = &ctx->d_frame;

	fmt = cameric_find_format(&f->fmt.pix_mp.pixelformat, NULL,
			       get_m2m_fmt_flags(f->type), 0);
	if (!fmt)
		return -EINVAL;

	__set_frame_format(frame, fmt, &f->fmt.pix_mp);

	/* Update RGB Alpha control state and value range */
	cameric_alpha_ctrl_update(ctx);

	return 0;
}

static int cameric_m2m_cropcap(struct file *file, void *fh,
			    struct v4l2_cropcap *cr)
{
	struct cameric_ctx *ctx = fh_to_ctx(fh);
	struct cameric_frame *frame;

	frame = ctx_get_frame(ctx, cr->type);
	if (IS_ERR(frame))
		return PTR_ERR(frame);

	cr->bounds.left = 0;
	cr->bounds.top = 0;
	cr->bounds.width = frame->o_width;
	cr->bounds.height = frame->o_height;
	cr->defrect = cr->bounds;

	return 0;
}

static int cameric_m2m_g_crop(struct file *file, void *fh, struct v4l2_crop *cr)
{
	struct cameric_ctx *ctx = fh_to_ctx(fh);
	struct cameric_frame *frame;

	frame = ctx_get_frame(ctx, cr->type);
	if (IS_ERR(frame))
		return PTR_ERR(frame);

	cr->c.left = frame->offs_h;
	cr->c.top = frame->offs_v;
	cr->c.width = frame->width;
	cr->c.height = frame->height;

	return 0;
}

static int cameric_m2m_try_crop(struct cameric_ctx *ctx, struct v4l2_crop *cr)
{
	struct cameric_dev *cameric = ctx->cameric_dev;
	struct cameric_frame *f;
	u32 min_size, halign, depth = 0;
	int i;

	if (cr->c.top < 0 || cr->c.left < 0) {
		v4l2_err(&cameric->m2m.vfd,
			"doesn't support negative values for top & left\n");
		return -EINVAL;
	}
	if (cr->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		f = &ctx->d_frame;
	else if (cr->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		f = &ctx->s_frame;
	else
		return -EINVAL;

	min_size = (f == &ctx->s_frame) ?
		cameric->variant->min_inp_pixsize : cameric->variant->min_out_pixsize;

	/* Get pixel alignment constraints. */
	if (cameric->variant->min_vsize_align == 1)
		halign = cameric_fmt_is_rgb(f->fmt->color) ? 0 : 1;
	else
		halign = ffs(cameric->variant->min_vsize_align) - 1;

	for (i = 0; i < f->fmt->memplanes; i++)
		depth += f->fmt->depth[i];

	v4l_bound_align_image(&cr->c.width, min_size, f->o_width,
			      ffs(min_size) - 1,
			      &cr->c.height, min_size, f->o_height,
			      halign, 64/(ALIGN(depth, 8)));

	/* adjust left/top if cropping rectangle is out of bounds */
	if (cr->c.left + cr->c.width > f->o_width)
		cr->c.left = f->o_width - cr->c.width;
	if (cr->c.top + cr->c.height > f->o_height)
		cr->c.top = f->o_height - cr->c.height;

	cr->c.left = round_down(cr->c.left, min_size);
	cr->c.top  = round_down(cr->c.top, cameric->variant->hor_offs_align);

	dbg("l:%d, t:%d, w:%d, h:%d, f_w: %d, f_h: %d",
	    cr->c.left, cr->c.top, cr->c.width, cr->c.height,
	    f->f_width, f->f_height);

	return 0;
}

static int cameric_m2m_s_crop(struct file *file, void *fh, const struct v4l2_crop *crop)
{
	struct cameric_ctx *ctx = fh_to_ctx(fh);
	struct cameric_dev *cameric = ctx->cameric_dev;
	struct v4l2_crop cr = *crop;
	struct cameric_frame *f;
	int ret;

	ret = cameric_m2m_try_crop(ctx, &cr);
	if (ret)
		return ret;

	f = (cr.type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) ?
		&ctx->s_frame : &ctx->d_frame;

	/* Check to see if scaling ratio is within supported range */
	if (cr.type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		ret = cameric_check_scaler_ratio(ctx, cr.c.width,
				cr.c.height, ctx->d_frame.width,
				ctx->d_frame.height, ctx->rotation);
	} else {
		ret = cameric_check_scaler_ratio(ctx, ctx->s_frame.width,
				ctx->s_frame.height, cr.c.width,
				cr.c.height, ctx->rotation);
	}
	if (ret) {
		v4l2_err(&cameric->m2m.vfd, "Out of scaler range\n");
		return -EINVAL;
	}

	f->offs_h = cr.c.left;
	f->offs_v = cr.c.top;
	f->width  = cr.c.width;
	f->height = cr.c.height;

	cameric_ctx_state_set(CAMERIC_PARAMS, ctx);

	return 0;
}

static const struct v4l2_ioctl_ops cameric_m2m_ioctl_ops = {
	.vidioc_querycap		= cameric_m2m_querycap,
	.vidioc_enum_fmt_vid_cap_mplane	= cameric_m2m_enum_fmt_mplane,
	.vidioc_enum_fmt_vid_out_mplane	= cameric_m2m_enum_fmt_mplane,
	.vidioc_g_fmt_vid_cap_mplane	= cameric_m2m_g_fmt_mplane,
	.vidioc_g_fmt_vid_out_mplane	= cameric_m2m_g_fmt_mplane,
	.vidioc_try_fmt_vid_cap_mplane	= cameric_m2m_try_fmt_mplane,
	.vidioc_try_fmt_vid_out_mplane	= cameric_m2m_try_fmt_mplane,
	.vidioc_s_fmt_vid_cap_mplane	= cameric_m2m_s_fmt_mplane,
	.vidioc_s_fmt_vid_out_mplane	= cameric_m2m_s_fmt_mplane,
	.vidioc_reqbufs			= v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf		= v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf			= v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf			= v4l2_m2m_ioctl_dqbuf,
	.vidioc_expbuf			= v4l2_m2m_ioctl_expbuf,
	.vidioc_streamon		= v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff		= v4l2_m2m_ioctl_streamoff,
	.vidioc_g_crop			= cameric_m2m_g_crop,
	.vidioc_s_crop			= cameric_m2m_s_crop,
	.vidioc_cropcap			= cameric_m2m_cropcap

};

static int queue_init(void *priv, struct vb2_queue *src_vq,
		      struct vb2_queue *dst_vq)
{
	struct cameric_ctx *ctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->ops = &cameric_qops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->cameric_dev->lock;
	src_vq->dev = &ctx->cameric_dev->pdev->dev;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->ops = &cameric_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->cameric_dev->lock;
	dst_vq->dev = &ctx->cameric_dev->pdev->dev;

	return vb2_queue_init(dst_vq);
}

static int cameric_m2m_set_default_format(struct cameric_ctx *ctx)
{
	struct v4l2_pix_format_mplane pixm = {
		.pixelformat	= V4L2_PIX_FMT_RGB32,
		.width		= 800,
		.height		= 600,
		.plane_fmt[0]	= {
			.bytesperline = 800 * 4,
			.sizeimage = 800 * 4 * 600,
		},
	};
	struct cameric_fmt *fmt;

	fmt = cameric_find_format(&pixm.pixelformat, NULL, FMT_FLAGS_M2M, 0);
	if (!fmt)
		return -EINVAL;

	__set_frame_format(&ctx->s_frame, fmt, &pixm);
	__set_frame_format(&ctx->d_frame, fmt, &pixm);

	return 0;
}

static int cameric_m2m_open(struct file *file)
{
	struct cameric_dev *cameric = video_drvdata(file);
	struct cameric_ctx *ctx;
	int ret = -EBUSY;

	pr_debug("pid: %d, state: %#lx\n", task_pid_nr(current), cameric->state);

	if (mutex_lock_interruptible(&cameric->lock))
		return -ERESTARTSYS;
	/*
	 * Don't allow simultaneous open() of the mem-to-mem and the
	 * capture video node that belong to same CAMERIC IP instance.
	 */
	if (test_bit(ST_CAPT_BUSY, &cameric->state))
		goto unlock;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		ret = -ENOMEM;
		goto unlock;
	}
	v4l2_fh_init(&ctx->fh, &cameric->m2m.vfd);
	ctx->cameric_dev = cameric;

	/* Default color format */
	ctx->s_frame.fmt = cameric_get_format(0);
	ctx->d_frame.fmt = cameric_get_format(0);

	ret = cameric_ctrls_create(ctx);
	if (ret)
		goto error_fh;

	/* Use separate control handler per file handle */
	ctx->fh.ctrl_handler = &ctx->ctrls.handler;
	file->private_data = &ctx->fh;
	v4l2_fh_add(&ctx->fh);

	/* Setup the device context for memory-to-memory mode */
	ctx->state = CAMERIC_CTX_M2M;
	ctx->flags = 0;
	ctx->in_path = CAMERIC_IO_DMA;
	ctx->out_path = CAMERIC_IO_DMA;
	ctx->scaler.enabled = 1;

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(cameric->m2m.m2m_dev, ctx, queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto error_c;
	}

	if (cameric->m2m.refcnt++ == 0)
		set_bit(ST_M2M_RUN, &cameric->state);

	ret = cameric_m2m_set_default_format(ctx);
	if (ret < 0)
		goto error_m2m_ctx;

	mutex_unlock(&cameric->lock);
	return 0;

error_m2m_ctx:
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
error_c:
	cameric_ctrls_delete(ctx);
	v4l2_fh_del(&ctx->fh);
error_fh:
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
unlock:
	mutex_unlock(&cameric->lock);
	return ret;
}

static int cameric_m2m_release(struct file *file)
{
	struct cameric_ctx *ctx = fh_to_ctx(file->private_data);
	struct cameric_dev *cameric = ctx->cameric_dev;

	dbg("pid: %d, state: 0x%lx, refcnt= %d",
		task_pid_nr(current), cameric->state, cameric->m2m.refcnt);

	mutex_lock(&cameric->lock);

	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	cameric_ctrls_delete(ctx);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);

	if (--cameric->m2m.refcnt <= 0)
		clear_bit(ST_M2M_RUN, &cameric->state);
	kfree(ctx);

	mutex_unlock(&cameric->lock);
	return 0;
}

static const struct v4l2_file_operations cameric_m2m_fops = {
	.owner		= THIS_MODULE,
	.open		= cameric_m2m_open,
	.release	= cameric_m2m_release,
	.poll		= v4l2_m2m_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= v4l2_m2m_fop_mmap,
};

static struct v4l2_m2m_ops m2m_ops = {
	.device_run	= cameric_device_run,
	.job_abort	= cameric_job_abort,
};

int cameric_register_m2m_device(struct cameric_dev *cameric,
			     struct v4l2_device *v4l2_dev)
{
	struct video_device *vfd = &cameric->m2m.vfd;
	int ret;

	cameric->v4l2_dev = v4l2_dev;

	memset(vfd, 0, sizeof(*vfd));
	vfd->fops = &cameric_m2m_fops;
	vfd->ioctl_ops = &cameric_m2m_ioctl_ops;
	vfd->v4l2_dev = v4l2_dev;
	vfd->minor = -1;
	vfd->release = video_device_release_empty;
	vfd->lock = &cameric->lock;
	vfd->vfl_dir = VFL_DIR_M2M;

	snprintf(vfd->name, sizeof(vfd->name), "cameric.%d.m2m", cameric->id);
	video_set_drvdata(vfd, cameric);

	cameric->m2m.m2m_dev = v4l2_m2m_init(&m2m_ops);
	if (IS_ERR(cameric->m2m.m2m_dev)) {
		v4l2_err(v4l2_dev, "failed to initialize v4l2-m2m device\n");
		return PTR_ERR(cameric->m2m.m2m_dev);
	}

	ret = media_entity_pads_init(&vfd->entity, 0, NULL);
	if (ret)
		goto err_me;

	ret = video_register_device(vfd, VFL_TYPE_GRABBER, -1);
	if (ret)
		goto err_vd;

	v4l2_info(v4l2_dev, "Registered %s as /dev/%s\n",
		  vfd->name, video_device_node_name(vfd));
	return 0;

err_vd:
	media_entity_cleanup(&vfd->entity);
err_me:
	v4l2_m2m_release(cameric->m2m.m2m_dev);
	return ret;
}

void cameric_unregister_m2m_device(struct cameric_dev *cameric)
{
	if (!cameric)
		return;

	if (cameric->m2m.m2m_dev)
		v4l2_m2m_release(cameric->m2m.m2m_dev);

	if (video_is_registered(&cameric->m2m.vfd)) {
		video_unregister_device(&cameric->m2m.vfd);
		media_entity_cleanup(&cameric->m2m.vfd.entity);
	}
}
