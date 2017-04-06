/*
 * Samsung S5P/EXYNOS4 SoC series camera interface (camera capture) driver
 *
 * Copyright (C) 2010 - 2012 Samsung Electronics Co., Ltd.
 * Sylwester Nawrocki <s.nawrocki@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/bug.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/pm_runtime.h>
#include <linux/list.h>
#include <linux/slab.h>

#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>

//#include "common.h"
#include "cameric-core.h"
#include "cameric-reg.h"
#include "media-dev.h"

static int cameric_capture_hw_init(struct cameric_dev *cameric)
{
	struct cameric_source_info *si = &cameric->vid_cap.source_config;
	struct cameric_ctx *ctx = cameric->vid_cap.ctx;
	int ret;
	unsigned long flags;

	if (ctx == NULL || ctx->s_frame.fmt == NULL)
		return -EINVAL;

	if (si->cameric_bus_type == CAMERIC_BUS_TYPE_ISP_WRITEBACK) {
		ret = cameric_hw_camblk_cfg_writeback(cameric);
		if (ret < 0)
			return ret;
	}

	spin_lock_irqsave(&cameric->slock, flags);
	cameric_prepare_dma_offset(ctx, &ctx->d_frame);
	cameric_set_yuv_order(ctx);

	cameric_hw_set_camera_polarity(cameric, si);
	cameric_hw_set_camera_type(cameric, si);
	cameric_hw_set_camera_source(cameric, si);
	cameric_hw_set_camera_offset(cameric, &ctx->s_frame);

	ret = cameric_set_scaler_info(ctx);
	if (!ret) {
		cameric_hw_set_input_path(ctx);
		cameric_hw_set_prescaler(ctx);
		cameric_hw_set_mainscaler(ctx);
		cameric_hw_set_target_format(ctx);
		cameric_hw_set_rotation(ctx);
		cameric_hw_set_effect(ctx);
		cameric_hw_set_output_path(ctx);
		cameric_hw_set_out_dma(ctx);
		if (cameric->drv_data->alpha_color)
			cameric_hw_set_rgb_alpha(ctx);
		clear_bit(ST_CAPT_APPLY_CFG, &cameric->state);
	}
	spin_unlock_irqrestore(&cameric->slock, flags);
	return ret;
}

/*
 * Reinitialize the driver so it is ready to start the streaming again.
 * Set cameric->state to indicate stream off and the hardware shut down state.
 * If not suspending (@suspend is false), return any buffers to videobuf2.
 * Otherwise put any owned buffers onto the pending buffers queue, so they
 * can be re-spun when the device is being resumed. Also perform CAMERIC
 * software reset and disable streaming on the whole pipeline if required.
 */
static int cameric_capture_state_cleanup(struct cameric_dev *cameric, bool suspend)
{
	struct cameric_vid_cap *cap = &cameric->vid_cap;
	struct cameric_vid_buffer *buf;
	unsigned long flags;
	bool streaming;

	spin_lock_irqsave(&cameric->slock, flags);
	streaming = cameric->state & (1 << ST_CAPT_ISP_STREAM);

	cameric->state &= ~(1 << ST_CAPT_RUN | 1 << ST_CAPT_SHUT |
			 1 << ST_CAPT_STREAM | 1 << ST_CAPT_ISP_STREAM);
	if (suspend)
		cameric->state |= (1 << ST_CAPT_SUSPENDED);
	else
		cameric->state &= ~(1 << ST_CAPT_PEND | 1 << ST_CAPT_SUSPENDED);

	/* Release unused buffers */
	while (!suspend && !list_empty(&cap->pending_buf_q)) {
		buf = cameric_pending_queue_pop(cap);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}
	/* If suspending put unused buffers onto pending queue */
	while (!list_empty(&cap->active_buf_q)) {
		buf = cameric_active_queue_pop(cap);
		if (suspend)
			cameric_pending_queue_add(cap, buf);
		else
			vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}

	cameric_hw_reset(cameric);
	cap->buf_index = 0;

	spin_unlock_irqrestore(&cameric->slock, flags);

	if (streaming)
		return cameric_pipeline_call(&cap->ve, set_stream, 0);
	else
		return 0;
}

static int cameric_stop_capture(struct cameric_dev *cameric, bool suspend)
{
	unsigned long flags;

	if (!cameric_capture_active(cameric))
		return 0;

	spin_lock_irqsave(&cameric->slock, flags);
	set_bit(ST_CAPT_SHUT, &cameric->state);
	cameric_deactivate_capture(cameric);
	spin_unlock_irqrestore(&cameric->slock, flags);

	wait_event_timeout(cameric->irq_queue,
			   !test_bit(ST_CAPT_SHUT, &cameric->state),
			   (2*HZ/10)); /* 200 ms */

	return cameric_capture_state_cleanup(cameric, suspend);
}

/**
 * cameric_capture_config_update - apply the camera interface configuration
 *
 * To be called from within the interrupt handler with cameric.slock
 * spinlock held. It updates the camera pixel crop, rotation and
 * image flip in H/W.
 */
static int cameric_capture_config_update(struct cameric_ctx *ctx)
{
	struct cameric_dev *cameric = ctx->cameric_dev;
	int ret;

	cameric_hw_set_camera_offset(cameric, &ctx->s_frame);

	ret = cameric_set_scaler_info(ctx);
	if (ret)
		return ret;

	cameric_hw_set_prescaler(ctx);
	cameric_hw_set_mainscaler(ctx);
	cameric_hw_set_target_format(ctx);
	cameric_hw_set_rotation(ctx);
	cameric_hw_set_effect(ctx);
	cameric_prepare_dma_offset(ctx, &ctx->d_frame);
	cameric_hw_set_out_dma(ctx);
	if (cameric->drv_data->alpha_color)
		cameric_hw_set_rgb_alpha(ctx);

	clear_bit(ST_CAPT_APPLY_CFG, &cameric->state);
	return ret;
}

void cameric_capture_irq_handler(struct cameric_dev *cameric, int deq_buf)
{
	struct cameric_vid_cap *cap = &cameric->vid_cap;
	struct cameric_pipeline *p = to_cameric_pipeline(cap->ve.pipe);
	struct v4l2_subdev *csis = p->subdevs[IDX_CSIS];
	struct cameric_frame *f = &cap->ctx->d_frame;
	struct cameric_vid_buffer *v_buf;

	if (test_and_clear_bit(ST_CAPT_SHUT, &cameric->state)) {
		wake_up(&cameric->irq_queue);
		goto done;
	}

	if (!list_empty(&cap->active_buf_q) &&
	    test_bit(ST_CAPT_RUN, &cameric->state) && deq_buf) {
		v_buf = cameric_active_queue_pop(cap);

		v_buf->vb.vb2_buf.timestamp = ktime_get_ns();
		v_buf->vb.sequence = cap->frame_count++;

		vb2_buffer_done(&v_buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
	}

	if (!list_empty(&cap->pending_buf_q)) {

		v_buf = cameric_pending_queue_pop(cap);
		cameric_hw_set_output_addr(cameric, &v_buf->paddr, cap->buf_index);
		v_buf->index = cap->buf_index;

		/* Move the buffer to the capture active queue */
		cameric_active_queue_add(cap, v_buf);

		dbg("next frame: %d, done frame: %d",
		    cameric_hw_get_frame_index(cameric), v_buf->index);

		if (++cap->buf_index >= CAMERIC_MAX_OUT_BUFS)
			cap->buf_index = 0;
	}
	/*
	 * Set up a buffer at MIPI-CSIS if current image format
	 * requires the frame embedded data capture.
	 */
	if (f->fmt->mdataplanes && !list_empty(&cap->active_buf_q)) {
		unsigned int plane = ffs(f->fmt->mdataplanes) - 1;
		unsigned int size = f->payload[plane];
		s32 index = cameric_hw_get_frame_index(cameric);
		void *vaddr;

		list_for_each_entry(v_buf, &cap->active_buf_q, list) {
			if (v_buf->index != index)
				continue;
			vaddr = vb2_plane_vaddr(&v_buf->vb.vb2_buf, plane);
			v4l2_subdev_call(csis, video, s_rx_buffer,
					 vaddr, &size);
			break;
		}
	}

	if (cap->active_buf_cnt == 0) {
		if (deq_buf)
			clear_bit(ST_CAPT_RUN, &cameric->state);

		if (++cap->buf_index >= CAMERIC_MAX_OUT_BUFS)
			cap->buf_index = 0;
	} else {
		set_bit(ST_CAPT_RUN, &cameric->state);
	}

	if (test_bit(ST_CAPT_APPLY_CFG, &cameric->state))
		cameric_capture_config_update(cap->ctx);
done:
	if (cap->active_buf_cnt == 1) {
		cameric_deactivate_capture(cameric);
		clear_bit(ST_CAPT_STREAM, &cameric->state);
	}

	dbg("frame: %d, active_buf_cnt: %d",
	    cameric_hw_get_frame_index(cameric), cap->active_buf_cnt);
}


static int start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct cameric_ctx *ctx = q->drv_priv;
	struct cameric_dev *cameric = ctx->cameric_dev;
	struct cameric_vid_cap *vid_cap = &cameric->vid_cap;
	int min_bufs;
	int ret;

	vid_cap->frame_count = 0;

	ret = cameric_capture_hw_init(cameric);
	if (ret) {
		cameric_capture_state_cleanup(cameric, false);
		return ret;
	}

	set_bit(ST_CAPT_PEND, &cameric->state);

	min_bufs = cameric->vid_cap.reqbufs_count > 1 ? 2 : 1;

	if (vid_cap->active_buf_cnt >= min_bufs &&
	    !test_and_set_bit(ST_CAPT_STREAM, &cameric->state)) {
		cameric_activate_capture(ctx);

		if (!test_and_set_bit(ST_CAPT_ISP_STREAM, &cameric->state))
			return cameric_pipeline_call(&vid_cap->ve, set_stream, 1);
	}

	return 0;
}

static void stop_streaming(struct vb2_queue *q)
{
	struct cameric_ctx *ctx = q->drv_priv;
	struct cameric_dev *cameric = ctx->cameric_dev;

	if (!cameric_capture_active(cameric))
		return;

	cameric_stop_capture(cameric, false);
}

int cameric_capture_suspend(struct cameric_dev *cameric)
{
	bool suspend = cameric_capture_busy(cameric);

	int ret = cameric_stop_capture(cameric, suspend);
	if (ret)
		return ret;
	return cameric_pipeline_call(&cameric->vid_cap.ve, close);
}

static void buffer_queue(struct vb2_buffer *vb);

int cameric_capture_resume(struct cameric_dev *cameric)
{
	struct cameric_vid_cap *vid_cap = &cameric->vid_cap;
	struct cameric_video_entity *ve = &vid_cap->ve;
	struct cameric_vid_buffer *buf;
	int i;

	if (!test_and_clear_bit(ST_CAPT_SUSPENDED, &cameric->state))
		return 0;

	INIT_LIST_HEAD(&cameric->vid_cap.active_buf_q);
	vid_cap->buf_index = 0;
	cameric_pipeline_call(ve, open, &ve->vdev.entity, false);
	cameric_capture_hw_init(cameric);

	clear_bit(ST_CAPT_SUSPENDED, &cameric->state);

	for (i = 0; i < vid_cap->reqbufs_count; i++) {
		if (list_empty(&vid_cap->pending_buf_q))
			break;
		buf = cameric_pending_queue_pop(vid_cap);
		buffer_queue(&buf->vb.vb2_buf);
	}
	return 0;

}

static int queue_setup(struct vb2_queue *vq,
		       unsigned int *num_buffers, unsigned int *num_planes,
		       unsigned int sizes[], struct device *alloc_devs[])
{
	struct cameric_ctx *ctx = vq->drv_priv;
	struct cameric_frame *frame = &ctx->d_frame;
	struct cameric_fmt *fmt = frame->fmt;
	unsigned long wh = frame->f_width * frame->f_height;
	int i;

	if (fmt == NULL)
		return -EINVAL;

	if (*num_planes) {
		if (*num_planes != fmt->memplanes)
			return -EINVAL;
		for (i = 0; i < *num_planes; i++)
			if (sizes[i] < (wh * fmt->depth[i]) / 8)
				return -EINVAL;
		return 0;
	}

	*num_planes = fmt->memplanes;

	for (i = 0; i < fmt->memplanes; i++) {
		unsigned int size = (wh * fmt->depth[i]) / 8;

		if (cameric_fmt_is_user_defined(fmt->color))
			sizes[i] = frame->payload[i];
		else
			sizes[i] = max_t(u32, size, frame->payload[i]);
	}

	return 0;
}

static int buffer_prepare(struct vb2_buffer *vb)
{
	struct vb2_queue *vq = vb->vb2_queue;
	struct cameric_ctx *ctx = vq->drv_priv;
	int i;

	if (ctx->d_frame.fmt == NULL)
		return -EINVAL;

	for (i = 0; i < ctx->d_frame.fmt->memplanes; i++) {
		unsigned long size = ctx->d_frame.payload[i];

		if (vb2_plane_size(vb, i) < size) {
			v4l2_err(&ctx->cameric_dev->vid_cap.ve.vdev,
				 "User buffer too small (%ld < %ld)\n",
				 vb2_plane_size(vb, i), size);
			return -EINVAL;
		}
		vb2_set_plane_payload(vb, i, size);
	}

	return 0;
}

static void buffer_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct cameric_vid_buffer *buf
		= container_of(vbuf, struct cameric_vid_buffer, vb);
	struct cameric_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct cameric_dev *cameric = ctx->cameric_dev;
	struct cameric_vid_cap *vid_cap = &cameric->vid_cap;
	struct cameric_video_entity *ve = &vid_cap->ve;
	unsigned long flags;
	int min_bufs;

	spin_lock_irqsave(&cameric->slock, flags);
	cameric_prepare_addr(ctx, &buf->vb.vb2_buf, &ctx->d_frame, &buf->paddr);

	if (!test_bit(ST_CAPT_SUSPENDED, &cameric->state) &&
	    !test_bit(ST_CAPT_STREAM, &cameric->state) &&
	    vid_cap->active_buf_cnt < CAMERIC_MAX_OUT_BUFS) {
		/* Setup the buffer directly for processing. */
		int buf_id = (vid_cap->reqbufs_count == 1) ? -1 :
				vid_cap->buf_index;

		cameric_hw_set_output_addr(cameric, &buf->paddr, buf_id);
		buf->index = vid_cap->buf_index;
		cameric_active_queue_add(vid_cap, buf);

		if (++vid_cap->buf_index >= CAMERIC_MAX_OUT_BUFS)
			vid_cap->buf_index = 0;
	} else {
		cameric_pending_queue_add(vid_cap, buf);
	}

	min_bufs = vid_cap->reqbufs_count > 1 ? 2 : 1;


	if (vb2_is_streaming(&vid_cap->vbq) &&
	    vid_cap->active_buf_cnt >= min_bufs &&
	    !test_and_set_bit(ST_CAPT_STREAM, &cameric->state)) {
		int ret;

		cameric_activate_capture(ctx);
		spin_unlock_irqrestore(&cameric->slock, flags);

		if (test_and_set_bit(ST_CAPT_ISP_STREAM, &cameric->state))
			return;

		ret = cameric_pipeline_call(ve, set_stream, 1);
		if (ret < 0)
			v4l2_err(&ve->vdev, "stream on failed: %d\n", ret);
		return;
	}
	spin_unlock_irqrestore(&cameric->slock, flags);
}

static const struct vb2_ops cameric_capture_qops = {
	.queue_setup		= queue_setup,
	.buf_prepare		= buffer_prepare,
	.buf_queue		= buffer_queue,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
	.start_streaming	= start_streaming,
	.stop_streaming		= stop_streaming,
};

static int cameric_capture_set_default_format(struct cameric_dev *cameric);

static int cameric_capture_open(struct file *file)
{
	struct cameric_dev *cameric = video_drvdata(file);
	struct cameric_vid_cap *vc = &cameric->vid_cap;
	struct cameric_video_entity *ve = &vc->ve;
	int ret = -EBUSY;

	dbg("pid: %d, state: 0x%lx", task_pid_nr(current), cameric->state);

	mutex_lock(&cameric->lock);

	if (cameric_m2m_active(cameric))
		goto unlock;

	set_bit(ST_CAPT_BUSY, &cameric->state);
	ret = pm_runtime_get_sync(&cameric->pdev->dev);
	if (ret < 0)
		goto unlock;

	ret = v4l2_fh_open(file);
	if (ret) {
		pm_runtime_put_sync(&cameric->pdev->dev);
		goto unlock;
	}

	if (v4l2_fh_is_singular_file(file)) {
		cameric_md_graph_lock(ve);

		ret = cameric_pipeline_call(ve, open, &ve->vdev.entity, true);

		if (ret == 0 && vc->user_subdev_api && vc->inh_sensor_ctrls) {
			/*
			 * Recreate controls of the the video node to drop
			 * any controls inherited from the sensor subdev.
			 */
			cameric_ctrls_delete(vc->ctx);

			ret = cameric_ctrls_create(vc->ctx);
			if (ret == 0)
				vc->inh_sensor_ctrls = false;
		}
		if (ret == 0)
			ve->vdev.entity.use_count++;

		cameric_md_graph_unlock(ve);

		if (ret == 0)
			ret = cameric_capture_set_default_format(cameric);

		if (ret < 0) {
			clear_bit(ST_CAPT_BUSY, &cameric->state);
			pm_runtime_put_sync(&cameric->pdev->dev);
			v4l2_fh_release(file);
		}
	}
unlock:
	mutex_unlock(&cameric->lock);
	return ret;
}

static int cameric_capture_release(struct file *file)
{
	struct cameric_dev *cameric = video_drvdata(file);
	struct cameric_vid_cap *vc = &cameric->vid_cap;
	bool close = v4l2_fh_is_singular_file(file);
	int ret;

	dbg("pid: %d, state: 0x%lx", task_pid_nr(current), cameric->state);

	mutex_lock(&cameric->lock);

	if (close && vc->streaming) {
		media_pipeline_stop(&vc->ve.vdev.entity);
		vc->streaming = false;
	}

	ret = _vb2_fop_release(file, NULL);

	if (close) {
		clear_bit(ST_CAPT_BUSY, &cameric->state);
		cameric_pipeline_call(&vc->ve, close);
		clear_bit(ST_CAPT_SUSPENDED, &cameric->state);

		cameric_md_graph_lock(&vc->ve);
		vc->ve.vdev.entity.use_count--;
		cameric_md_graph_unlock(&vc->ve);
	}

	pm_runtime_put_sync(&cameric->pdev->dev);
	mutex_unlock(&cameric->lock);

	return ret;
}

static const struct v4l2_file_operations cameric_capture_fops = {
	.owner		= THIS_MODULE,
	.open		= cameric_capture_open,
	.release	= cameric_capture_release,
	.poll		= vb2_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= vb2_fop_mmap,
};

/*
 * Format and crop negotiation helpers
 */

static struct cameric_fmt *cameric_capture_try_format(struct cameric_ctx *ctx,
						u32 *width, u32 *height,
						u32 *code, u32 *fourcc, int pad)
{
	bool rotation = ctx->rotation == 90 || ctx->rotation == 270;
	struct cameric_dev *cameric = ctx->cameric_dev;
	const struct cameric_variant *var = cameric->variant;
	const struct cameric_pix_limit *pl = var->pix_limit;
	struct cameric_frame *dst = &ctx->d_frame;
	u32 depth, min_w, max_w, min_h, align_h = 3;
	u32 mask = FMT_FLAGS_CAM;
	struct cameric_fmt *ffmt;

	/* Conversion from/to JPEG or User Defined format is not supported */
	if (code && ctx->s_frame.fmt && pad == CAMERIC_SD_PAD_SOURCE &&
	    cameric_fmt_is_user_defined(ctx->s_frame.fmt->color))
		*code = ctx->s_frame.fmt->mbus_code;

	if (fourcc && *fourcc != V4L2_PIX_FMT_JPEG && pad == CAMERIC_SD_PAD_SOURCE)
		mask |= FMT_FLAGS_M2M;

	if (pad == CAMERIC_SD_PAD_SINK_FIFO)
		mask = FMT_FLAGS_WRITEBACK;

	ffmt = cameric_find_format(fourcc, code, mask, 0);
	if (WARN_ON(!ffmt))
		return NULL;

	if (code)
		*code = ffmt->mbus_code;
	if (fourcc)
		*fourcc = ffmt->fourcc;

	if (pad != CAMERIC_SD_PAD_SOURCE) {
		max_w = cameric_fmt_is_user_defined(ffmt->color) ?
			pl->scaler_dis_w : pl->scaler_en_w;
		/* Apply the camera input interface pixel constraints */
		v4l_bound_align_image(width, max_t(u32, *width, 32), max_w, 4,
				      height, max_t(u32, *height, 32),
				      CAMERIC_CAMIF_MAX_HEIGHT,
				      cameric_fmt_is_user_defined(ffmt->color) ?
				      3 : 1,
				      0);
		return ffmt;
	}
	/* Can't scale or crop in transparent (JPEG) transfer mode */
	if (cameric_fmt_is_user_defined(ffmt->color)) {
		*width  = ctx->s_frame.f_width;
		*height = ctx->s_frame.f_height;
		return ffmt;
	}
	/* Apply the scaler and the output DMA constraints */
	max_w = rotation ? pl->out_rot_en_w : pl->out_rot_dis_w;
	if (ctx->state & CAMERIC_COMPOSE) {
		min_w = dst->offs_h + dst->width;
		min_h = dst->offs_v + dst->height;
	} else {
		min_w = var->min_out_pixsize;
		min_h = var->min_out_pixsize;
	}
	if (var->min_vsize_align == 1 && !rotation)
		align_h = cameric_fmt_is_rgb(ffmt->color) ? 0 : 1;

	depth = cameric_get_format_depth(ffmt);
	v4l_bound_align_image(width, min_w, max_w,
			      ffs(var->min_out_pixsize) - 1,
			      height, min_h, CAMERIC_CAMIF_MAX_HEIGHT,
			      align_h,
			      64/(ALIGN(depth, 8)));

	dbg("pad%d: code: 0x%x, %dx%d. dst fmt: %dx%d",
	    pad, code ? *code : 0, *width, *height,
	    dst->f_width, dst->f_height);

	return ffmt;
}

static void cameric_capture_try_selection(struct cameric_ctx *ctx,
				       struct v4l2_rect *r,
				       int target)
{
	bool rotate = ctx->rotation == 90 || ctx->rotation == 270;
	struct cameric_dev *cameric = ctx->cameric_dev;
	const struct cameric_variant *var = cameric->variant;
	const struct cameric_pix_limit *pl = var->pix_limit;
	struct cameric_frame *sink = &ctx->s_frame;
	u32 max_w, max_h, min_w = 0, min_h = 0, min_sz;
	u32 align_sz = 0, align_h = 4;
	u32 max_sc_h, max_sc_v;

	/* In JPEG transparent transfer mode cropping is not supported */
	if (cameric_fmt_is_user_defined(ctx->d_frame.fmt->color)) {
		r->width  = sink->f_width;
		r->height = sink->f_height;
		r->left   = r->top = 0;
		return;
	}
	if (target == V4L2_SEL_TGT_COMPOSE) {
		if (ctx->rotation != 90 && ctx->rotation != 270)
			align_h = 1;
		max_sc_h = min(SCALER_MAX_HRATIO, 1 << (ffs(sink->width) - 3));
		max_sc_v = min(SCALER_MAX_VRATIO, 1 << (ffs(sink->height) - 1));
		min_sz = var->min_out_pixsize;
	} else {
		u32 depth = cameric_get_format_depth(sink->fmt);
		align_sz = 64/ALIGN(depth, 8);
		min_sz = var->min_inp_pixsize;
		min_w = min_h = min_sz;
		max_sc_h = max_sc_v = 1;
	}
	/*
	 * For the compose rectangle the following constraints must be met:
	 * - it must fit in the sink pad format rectangle (f_width/f_height);
	 * - maximum downscaling ratio is 64;
	 * - maximum crop size depends if the rotator is used or not;
	 * - the sink pad format width/height must be 4 multiple of the
	 *   prescaler ratios determined by sink pad size and source pad crop,
	 *   the prescaler ratio is returned by cameric_get_scaler_factor().
	 */
	max_w = min_t(u32,
		      rotate ? pl->out_rot_en_w : pl->out_rot_dis_w,
		      rotate ? sink->f_height : sink->f_width);
	max_h = min_t(u32, CAMERIC_CAMIF_MAX_HEIGHT, sink->f_height);

	if (target == V4L2_SEL_TGT_COMPOSE) {
		min_w = min_t(u32, max_w, sink->f_width / max_sc_h);
		min_h = min_t(u32, max_h, sink->f_height / max_sc_v);
		if (rotate) {
			swap(max_sc_h, max_sc_v);
			swap(min_w, min_h);
		}
	}
	v4l_bound_align_image(&r->width, min_w, max_w, ffs(min_sz) - 1,
			      &r->height, min_h, max_h, align_h,
			      align_sz);
	/* Adjust left/top if crop/compose rectangle is out of bounds */
	r->left = clamp_t(u32, r->left, 0, sink->f_width - r->width);
	r->top  = clamp_t(u32, r->top, 0, sink->f_height - r->height);
	r->left = round_down(r->left, var->hor_offs_align);

	dbg("target %#x: (%d,%d)/%dx%d, sink fmt: %dx%d",
	    target, r->left, r->top, r->width, r->height,
	    sink->f_width, sink->f_height);
}

/* Called with the media graph mutex held or entity->stream_count > 0. */
struct v4l2_subdev *cameric_find_remote_sensor(struct media_entity *entity)
{
	struct media_pad *pad = &entity->pads[0];
	struct v4l2_subdev *sd;

	while (pad->flags & MEDIA_PAD_FL_SINK) {
		/* source pad */
		pad = media_entity_remote_pad(pad);
		if (!pad || !is_media_entity_v4l2_subdev(pad->entity))
			break;

		sd = media_entity_to_v4l2_subdev(pad->entity);

		if (sd->grp_id == GRP_ID_CAMERIC_IS_SENSOR ||
		    sd->grp_id == GRP_ID_SENSOR)
			return sd;
		/* sink pad */
		pad = &sd->entity.pads[0];
	}
	return NULL;
}
EXPORT_SYMBOL(cameric_find_remote_sensor);

void __cameric_vidioc_querycap(struct device *dev, struct v4l2_capability *cap,
						unsigned int caps)
{
	strlcpy(cap->driver, dev->driver->name, sizeof(cap->driver));
	strlcpy(cap->card, dev->driver->name, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info),
				"platform:%s", dev_name(dev));
	cap->device_caps = caps;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
}

/*
 * The video node ioctl operations
 */
static int cameric_cap_querycap(struct file *file, void *priv,
					struct v4l2_capability *cap)
{
	struct cameric_dev *cameric = video_drvdata(file);

	__cameric_vidioc_querycap(&cameric->pdev->dev, cap, V4L2_CAP_STREAMING |
					V4L2_CAP_VIDEO_CAPTURE_MPLANE);
	return 0;
}

static int cameric_cap_enum_fmt_mplane(struct file *file, void *priv,
				    struct v4l2_fmtdesc *f)
{
	struct cameric_fmt *fmt;

	fmt = cameric_find_format(NULL, NULL, FMT_FLAGS_CAM | FMT_FLAGS_M2M,
			       f->index);
	if (!fmt)
		return -EINVAL;
	strncpy(f->description, fmt->name, sizeof(f->description) - 1);
	f->pixelformat = fmt->fourcc;
	if (fmt->fourcc == MEDIA_BUS_FMT_JPEG_1X8)
		f->flags |= V4L2_FMT_FLAG_COMPRESSED;
	return 0;
}

static struct media_entity *cameric_pipeline_get_head(struct media_entity *me)
{
	struct media_pad *pad = &me->pads[0];

	while (!(pad->flags & MEDIA_PAD_FL_SOURCE)) {
		pad = media_entity_remote_pad(pad);
		if (!pad)
			break;
		me = pad->entity;
		pad = &me->pads[0];
	}

	return me;
}

/**
 * cameric_pipeline_try_format - negotiate and/or set formats at pipeline
 *                            elements
 * @ctx: CAMERIC capture context
 * @tfmt: media bus format to try/set on subdevs
 * @fmt_id: cameric pixel format id corresponding to returned @tfmt (output)
 * @set: true to set format on subdevs, false to try only
 */
static int cameric_pipeline_try_format(struct cameric_ctx *ctx,
				    struct v4l2_mbus_framefmt *tfmt,
				    struct cameric_fmt **fmt_id,
				    bool set)
{
	struct cameric_dev *cameric = ctx->cameric_dev;
	struct cameric_pipeline *p = to_cameric_pipeline(cameric->vid_cap.ve.pipe);
	struct v4l2_subdev *sd = p->subdevs[IDX_SENSOR];
	struct v4l2_subdev_format sfmt;
	struct v4l2_mbus_framefmt *mf = &sfmt.format;
	struct media_entity *me;
	struct cameric_fmt *ffmt;
	struct media_pad *pad;
	int ret, i = 1;
	u32 fcc;

	if (WARN_ON(!sd || !tfmt))
		return -EINVAL;

	memset(&sfmt, 0, sizeof(sfmt));
	sfmt.format = *tfmt;
	sfmt.which = set ? V4L2_SUBDEV_FORMAT_ACTIVE : V4L2_SUBDEV_FORMAT_TRY;

	me = cameric_pipeline_get_head(&sd->entity);

	while (1) {
		ffmt = cameric_find_format(NULL, mf->code != 0 ? &mf->code : NULL,
					FMT_FLAGS_CAM, i++);
		if (ffmt == NULL) {
			/*
			 * Notify user-space if common pixel code for
			 * host and sensor does not exist.
			 */
			return -EINVAL;
		}
		mf->code = tfmt->code = ffmt->mbus_code;

		/* set format on all pipeline subdevs */
		while (me != &cameric->vid_cap.subdev.entity) {
			sd = media_entity_to_v4l2_subdev(me);

			sfmt.pad = 0;
			ret = v4l2_subdev_call(sd, pad, set_fmt, NULL, &sfmt);
			if (ret)
				return ret;

			if (me->pads[0].flags & MEDIA_PAD_FL_SINK) {
				sfmt.pad = me->num_pads - 1;
				mf->code = tfmt->code;
				ret = v4l2_subdev_call(sd, pad, set_fmt, NULL,
									&sfmt);
				if (ret)
					return ret;
			}

			pad = media_entity_remote_pad(&me->pads[sfmt.pad]);
			if (!pad)
				return -EINVAL;
			me = pad->entity;
		}

		if (mf->code != tfmt->code)
			continue;

		fcc = ffmt->fourcc;
		tfmt->width  = mf->width;
		tfmt->height = mf->height;
		ffmt = cameric_capture_try_format(ctx, &tfmt->width, &tfmt->height,
					NULL, &fcc, CAMERIC_SD_PAD_SINK_CAM);
		ffmt = cameric_capture_try_format(ctx, &tfmt->width, &tfmt->height,
					NULL, &fcc, CAMERIC_SD_PAD_SOURCE);
		if (ffmt && ffmt->mbus_code)
			mf->code = ffmt->mbus_code;
		if (mf->width != tfmt->width || mf->height != tfmt->height)
			continue;
		tfmt->code = mf->code;
		break;
	}

	if (fmt_id && ffmt)
		*fmt_id = ffmt;
	*tfmt = *mf;

	return 0;
}

/**
 * cameric_get_sensor_frame_desc - query the sensor for media bus frame parameters
 * @sensor: pointer to the sensor subdev
 * @plane_fmt: provides plane sizes corresponding to the frame layout entries
 * @try: true to set the frame parameters, false to query only
 *
 * This function is used by this driver only for compressed/blob data formats.
 */
static int cameric_get_sensor_frame_desc(struct v4l2_subdev *sensor,
				      struct v4l2_plane_pix_format *plane_fmt,
				      unsigned int num_planes, bool try)
{
	struct v4l2_mbus_frame_desc fd;
	int i, ret;
	int pad;

	for (i = 0; i < num_planes; i++)
		fd.entry[i].length = plane_fmt[i].sizeimage;

	pad = sensor->entity.num_pads - 1;
	if (try)
		ret = v4l2_subdev_call(sensor, pad, set_frame_desc, pad, &fd);
	else
		ret = v4l2_subdev_call(sensor, pad, get_frame_desc, pad, &fd);

	if (ret < 0)
		return ret;

	if (num_planes != fd.num_entries)
		return -EINVAL;

	for (i = 0; i < num_planes; i++)
		plane_fmt[i].sizeimage = fd.entry[i].length;

	if (fd.entry[0].length > CAMERIC_MAX_JPEG_BUF_SIZE) {
		v4l2_err(sensor->v4l2_dev,  "Unsupported buffer size: %u\n",
			 fd.entry[0].length);

		return -EINVAL;
	}

	return 0;
}

static int cameric_cap_g_fmt_mplane(struct file *file, void *fh,
				 struct v4l2_format *f)
{
	struct cameric_dev *cameric = video_drvdata(file);

	__cameric_get_format(&cameric->vid_cap.ctx->d_frame, f);
	return 0;
}

/*
 * Try or set format on the cameric.X.capture video node and additionally
 * on the whole pipeline if @try is false.
 * Locking: the caller must _not_ hold the graph mutex.
 */
static int __video_try_or_set_format(struct cameric_dev *cameric,
				     struct v4l2_format *f, bool try,
				     struct cameric_fmt **inp_fmt,
				     struct cameric_fmt **out_fmt)
{
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	struct cameric_vid_cap *vc = &cameric->vid_cap;
	struct cameric_video_entity *ve = &vc->ve;
	struct cameric_ctx *ctx = vc->ctx;
	unsigned int width = 0, height = 0;
	int ret = 0;

	/* Pre-configure format at the camera input interface, for JPEG only */
	if (cameric_jpeg_fourcc(pix->pixelformat)) {
		cameric_capture_try_format(ctx, &pix->width, &pix->height,
					NULL, &pix->pixelformat,
					CAMERIC_SD_PAD_SINK_CAM);
		if (try) {
			width = pix->width;
			height = pix->height;
		} else {
			ctx->s_frame.f_width = pix->width;
			ctx->s_frame.f_height = pix->height;
		}
	}

	/* Try the format at the scaler and the DMA output */
	*out_fmt = cameric_capture_try_format(ctx, &pix->width, &pix->height,
					  NULL, &pix->pixelformat,
					  CAMERIC_SD_PAD_SOURCE);
	if (*out_fmt == NULL)
		return -EINVAL;

	/* Restore image width/height for JPEG (no resizing supported). */
	if (try && cameric_jpeg_fourcc(pix->pixelformat)) {
		pix->width = width;
		pix->height = height;
	}

	/* Try to match format at the host and the sensor */
	if (!vc->user_subdev_api) {
		struct v4l2_mbus_framefmt mbus_fmt;
		struct v4l2_mbus_framefmt *mf;

		mf = try ? &mbus_fmt : &cameric->vid_cap.ci_fmt;

		mf->code = (*out_fmt)->mbus_code;
		mf->width = pix->width;
		mf->height = pix->height;

		cameric_md_graph_lock(ve);
		ret = cameric_pipeline_try_format(ctx, mf, inp_fmt, try);
		cameric_md_graph_unlock(ve);

		if (ret < 0)
			return ret;

		pix->width = mf->width;
		pix->height = mf->height;
	}

	cameric_adjust_mplane_format(*out_fmt, pix->width, pix->height, pix);

	if ((*out_fmt)->flags & FMT_FLAGS_COMPRESSED) {
		struct v4l2_subdev *sensor;

		cameric_md_graph_lock(ve);

		sensor = __cameric_md_get_subdev(ve->pipe, IDX_SENSOR);
		if (sensor)
			cameric_get_sensor_frame_desc(sensor, pix->plane_fmt,
						   (*out_fmt)->memplanes, try);
		else
			ret = -EPIPE;

		cameric_md_graph_unlock(ve);
	}

	return ret;
}

static int cameric_cap_try_fmt_mplane(struct file *file, void *fh,
				   struct v4l2_format *f)
{
	struct cameric_dev *cameric = video_drvdata(file);
	struct cameric_fmt *out_fmt = NULL, *inp_fmt = NULL;

	return __video_try_or_set_format(cameric, f, true, &inp_fmt, &out_fmt);
}

static void cameric_capture_mark_jpeg_xfer(struct cameric_ctx *ctx,
					enum cameric_color_fmt color)
{
	bool jpeg = cameric_fmt_is_user_defined(color);

	ctx->scaler.enabled = !jpeg;
	cameric_ctrls_activate(ctx, !jpeg);

	if (jpeg)
		set_bit(ST_CAPT_JPEG, &ctx->cameric_dev->state);
	else
		clear_bit(ST_CAPT_JPEG, &ctx->cameric_dev->state);
}

static int __cameric_capture_set_format(struct cameric_dev *cameric,
				     struct v4l2_format *f)
{
	struct cameric_vid_cap *vc = &cameric->vid_cap;
	struct cameric_ctx *ctx = vc->ctx;
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	struct cameric_frame *ff = &ctx->d_frame;
	struct cameric_fmt *inp_fmt = NULL;
	int ret, i;

	if (vb2_is_busy(&cameric->vid_cap.vbq))
		return -EBUSY;

	ret = __video_try_or_set_format(cameric, f, false, &inp_fmt, &ff->fmt);
	if (ret < 0)
		return ret;

	/* Update RGB Alpha control state and value range */
	cameric_alpha_ctrl_update(ctx);

	for (i = 0; i < ff->fmt->memplanes; i++) {
		ff->bytesperline[i] = pix->plane_fmt[i].bytesperline;
		ff->payload[i] = pix->plane_fmt[i].sizeimage;
	}

	set_frame_bounds(ff, pix->width, pix->height);
	/* Reset the composition rectangle if not yet configured */
	if (!(ctx->state & CAMERIC_COMPOSE))
		set_frame_crop(ff, 0, 0, pix->width, pix->height);

	cameric_capture_mark_jpeg_xfer(ctx, ff->fmt->color);

	/* Reset cropping and set format at the camera interface input */
	if (!vc->user_subdev_api) {
		ctx->s_frame.fmt = inp_fmt;
		set_frame_bounds(&ctx->s_frame, pix->width, pix->height);
		set_frame_crop(&ctx->s_frame, 0, 0, pix->width, pix->height);
	}

	return ret;
}

static int cameric_cap_s_fmt_mplane(struct file *file, void *priv,
				 struct v4l2_format *f)
{
	struct cameric_dev *cameric = video_drvdata(file);

	return __cameric_capture_set_format(cameric, f);
}

static int cameric_cap_enum_input(struct file *file, void *priv,
			       struct v4l2_input *i)
{
	struct cameric_dev *cameric = video_drvdata(file);
	struct cameric_video_entity *ve = &cameric->vid_cap.ve;
	struct v4l2_subdev *sd;

	if (i->index != 0)
		return -EINVAL;

	i->type = V4L2_INPUT_TYPE_CAMERA;
	cameric_md_graph_lock(ve);
	sd = __cameric_md_get_subdev(ve->pipe, IDX_SENSOR);
	cameric_md_graph_unlock(ve);

	if (sd)
		strlcpy(i->name, sd->name, sizeof(i->name));

	return 0;
}

static int cameric_cap_s_input(struct file *file, void *priv, unsigned int i)
{
	return i == 0 ? i : -EINVAL;
}

static int cameric_cap_g_input(struct file *file, void *priv, unsigned int *i)
{
	*i = 0;
	return 0;
}

/**
 * cameric_pipeline_validate - check for formats inconsistencies
 *                          between source and sink pad of each link
 *
 * Return 0 if all formats match or -EPIPE otherwise.
 */
static int cameric_pipeline_validate(struct cameric_dev *cameric)
{
	struct v4l2_subdev_format sink_fmt, src_fmt;
	struct cameric_vid_cap *vc = &cameric->vid_cap;
	struct v4l2_subdev *sd = &vc->subdev;
	struct cameric_pipeline *p = to_cameric_pipeline(vc->ve.pipe);
	struct media_pad *sink_pad, *src_pad;
	int i, ret;

	while (1) {
		/*
		 * Find current entity sink pad and any remote sink pad linked
		 * to it. We stop if there is no sink pad in current entity or
		 * it is not linked to any other remote entity.
		 */
		src_pad = NULL;

		for (i = 0; i < sd->entity.num_pads; i++) {
			struct media_pad *p = &sd->entity.pads[i];

			if (p->flags & MEDIA_PAD_FL_SINK) {
				sink_pad = p;
				src_pad = media_entity_remote_pad(sink_pad);
				if (src_pad)
					break;
			}
		}

		if (!src_pad || !is_media_entity_v4l2_subdev(src_pad->entity))
			break;

		/* Don't call CAMERIC subdev operation to avoid nested locking */
		if (sd == &vc->subdev) {
			struct cameric_frame *ff = &vc->ctx->s_frame;
			sink_fmt.format.width = ff->f_width;
			sink_fmt.format.height = ff->f_height;
			sink_fmt.format.code = ff->fmt ? ff->fmt->mbus_code : 0;
		} else {
			sink_fmt.pad = sink_pad->index;
			sink_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
			ret = v4l2_subdev_call(sd, pad, get_fmt, NULL, &sink_fmt);
			if (ret < 0 && ret != -ENOIOCTLCMD)
				return -EPIPE;
		}

		/* Retrieve format at the source pad */
		sd = media_entity_to_v4l2_subdev(src_pad->entity);
		src_fmt.pad = src_pad->index;
		src_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
		ret = v4l2_subdev_call(sd, pad, get_fmt, NULL, &src_fmt);
		if (ret < 0 && ret != -ENOIOCTLCMD)
			return -EPIPE;

		if (src_fmt.format.width != sink_fmt.format.width ||
		    src_fmt.format.height != sink_fmt.format.height ||
		    src_fmt.format.code != sink_fmt.format.code)
			return -EPIPE;

		if (sd == p->subdevs[IDX_SENSOR] &&
		    cameric_user_defined_mbus_fmt(src_fmt.format.code)) {
			struct v4l2_plane_pix_format plane_fmt[CAMERIC_MAX_PLANES];
			struct cameric_frame *frame = &vc->ctx->d_frame;
			unsigned int i;

			ret = cameric_get_sensor_frame_desc(sd, plane_fmt,
							 frame->fmt->memplanes,
							 false);
			if (ret < 0)
				return -EPIPE;

			for (i = 0; i < frame->fmt->memplanes; i++)
				if (frame->payload[i] < plane_fmt[i].sizeimage)
					return -EPIPE;
		}
	}
	return 0;
}

static int cameric_cap_streamon(struct file *file, void *priv,
			     enum v4l2_buf_type type)
{
	struct cameric_dev *cameric = video_drvdata(file);
	struct cameric_vid_cap *vc = &cameric->vid_cap;
	struct media_entity *entity = &vc->ve.vdev.entity;
	struct cameric_source_info *si = NULL;
	struct v4l2_subdev *sd;
	int ret;

	if (cameric_capture_active(cameric))
		return -EBUSY;

	ret = media_pipeline_start(entity, &vc->ve.pipe->mp);
	if (ret < 0)
		return ret;

	sd = __cameric_md_get_subdev(vc->ve.pipe, IDX_SENSOR);
	if (sd)
		si = v4l2_get_subdev_hostdata(sd);

	if (si == NULL) {
		ret = -EPIPE;
		goto err_p_stop;
	}
	/*
	 * Save configuration data related to currently attached image
	 * sensor or other data source, e.g. CAMERIC-IS.
	 */
	vc->source_config = *si;

	if (vc->input == GRP_ID_CAMERIC_IS)
		vc->source_config.cameric_bus_type = CAMERIC_BUS_TYPE_ISP_WRITEBACK;

	if (vc->user_subdev_api) {
		ret = cameric_pipeline_validate(cameric);
		if (ret < 0)
			goto err_p_stop;
	}

	ret = vb2_ioctl_streamon(file, priv, type);
	if (!ret) {
		vc->streaming = true;
		return ret;
	}

err_p_stop:
	media_pipeline_stop(entity);
	return ret;
}

static int cameric_cap_streamoff(struct file *file, void *priv,
			    enum v4l2_buf_type type)
{
	struct cameric_dev *cameric = video_drvdata(file);
	struct cameric_vid_cap *vc = &cameric->vid_cap;
	int ret;

	ret = vb2_ioctl_streamoff(file, priv, type);
	if (ret < 0)
		return ret;

	media_pipeline_stop(&vc->ve.vdev.entity);
	vc->streaming = false;
	return 0;
}

static int cameric_cap_reqbufs(struct file *file, void *priv,
			    struct v4l2_requestbuffers *reqbufs)
{
	struct cameric_dev *cameric = video_drvdata(file);
	int ret;

	ret = vb2_ioctl_reqbufs(file, priv, reqbufs);

	if (!ret)
		cameric->vid_cap.reqbufs_count = reqbufs->count;

	return ret;
}

static int cameric_cap_g_selection(struct file *file, void *fh,
				struct v4l2_selection *s)
{
	struct cameric_dev *cameric = video_drvdata(file);
	struct cameric_ctx *ctx = cameric->vid_cap.ctx;
	struct cameric_frame *f = &ctx->s_frame;

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;

	switch (s->target) {
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		f = &ctx->d_frame;
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = f->o_width;
		s->r.height = f->o_height;
		return 0;

	case V4L2_SEL_TGT_COMPOSE:
		f = &ctx->d_frame;
	case V4L2_SEL_TGT_CROP:
		s->r.left = f->offs_h;
		s->r.top = f->offs_v;
		s->r.width = f->width;
		s->r.height = f->height;
		return 0;
	}

	return -EINVAL;
}

/* Return 1 if rectangle a is enclosed in rectangle b, or 0 otherwise. */
static int enclosed_rectangle(struct v4l2_rect *a, struct v4l2_rect *b)
{
	if (a->left < b->left || a->top < b->top)
		return 0;
	if (a->left + a->width > b->left + b->width)
		return 0;
	if (a->top + a->height > b->top + b->height)
		return 0;

	return 1;
}

static int cameric_cap_s_selection(struct file *file, void *fh,
				struct v4l2_selection *s)
{
	struct cameric_dev *cameric = video_drvdata(file);
	struct cameric_ctx *ctx = cameric->vid_cap.ctx;
	struct v4l2_rect rect = s->r;
	struct cameric_frame *f;
	unsigned long flags;

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;

	if (s->target == V4L2_SEL_TGT_COMPOSE)
		f = &ctx->d_frame;
	else if (s->target == V4L2_SEL_TGT_CROP)
		f = &ctx->s_frame;
	else
		return -EINVAL;

	cameric_capture_try_selection(ctx, &rect, s->target);

	if (s->flags & V4L2_SEL_FLAG_LE &&
	    !enclosed_rectangle(&rect, &s->r))
		return -ERANGE;

	if (s->flags & V4L2_SEL_FLAG_GE &&
	    !enclosed_rectangle(&s->r, &rect))
		return -ERANGE;

	s->r = rect;
	spin_lock_irqsave(&cameric->slock, flags);
	set_frame_crop(f, s->r.left, s->r.top, s->r.width,
		       s->r.height);
	spin_unlock_irqrestore(&cameric->slock, flags);

	set_bit(ST_CAPT_APPLY_CFG, &cameric->state);
	return 0;
}

static const struct v4l2_ioctl_ops cameric_capture_ioctl_ops = {
	.vidioc_querycap		= cameric_cap_querycap,

	.vidioc_enum_fmt_vid_cap_mplane	= cameric_cap_enum_fmt_mplane,
	.vidioc_try_fmt_vid_cap_mplane	= cameric_cap_try_fmt_mplane,
	.vidioc_s_fmt_vid_cap_mplane	= cameric_cap_s_fmt_mplane,
	.vidioc_g_fmt_vid_cap_mplane	= cameric_cap_g_fmt_mplane,

	.vidioc_reqbufs			= cameric_cap_reqbufs,
	.vidioc_querybuf		= vb2_ioctl_querybuf,
	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
	.vidioc_expbuf			= vb2_ioctl_expbuf,
	.vidioc_prepare_buf		= vb2_ioctl_prepare_buf,
	.vidioc_create_bufs		= vb2_ioctl_create_bufs,

	.vidioc_streamon		= cameric_cap_streamon,
	.vidioc_streamoff		= cameric_cap_streamoff,

	.vidioc_g_selection		= cameric_cap_g_selection,
	.vidioc_s_selection		= cameric_cap_s_selection,

	.vidioc_enum_input		= cameric_cap_enum_input,
	.vidioc_s_input			= cameric_cap_s_input,
	.vidioc_g_input			= cameric_cap_g_input,
};

/* Capture subdev media entity operations */
static int cameric_link_setup(struct media_entity *entity,
			   const struct media_pad *local,
			   const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct cameric_dev *cameric = v4l2_get_subdevdata(sd);
	struct cameric_vid_cap *vc = &cameric->vid_cap;
	struct v4l2_subdev *sensor;

	if (!is_media_entity_v4l2_subdev(remote->entity))
		return -EINVAL;

	if (WARN_ON(cameric == NULL))
		return 0;

	dbg("%s --> %s, flags: 0x%x. input: 0x%x",
	    local->entity->name, remote->entity->name, flags,
	    cameric->vid_cap.input);

	if (!(flags & MEDIA_LNK_FL_ENABLED)) {
		cameric->vid_cap.input = 0;
		return 0;
	}

	if (vc->input != 0)
		return -EBUSY;

	vc->input = sd->grp_id;

	if (vc->user_subdev_api || vc->inh_sensor_ctrls)
		return 0;

	/* Inherit V4L2 controls from the image sensor subdev. */
	sensor = cameric_find_remote_sensor(&vc->subdev.entity);
	if (sensor == NULL)
		return 0;

	return v4l2_ctrl_add_handler(&vc->ctx->ctrls.handler,
				     sensor->ctrl_handler, NULL);
}

static const struct media_entity_operations cameric_sd_media_ops = {
	.link_setup = cameric_link_setup,
};

/**
 * cameric_sensor_notify - v4l2_device notification from a sensor subdev
 * @sd: pointer to a subdev generating the notification
 * @notification: the notification type, must be S5P_CAMERIC_TX_END_NOTIFY
 * @arg: pointer to an u32 type integer that stores the frame payload value
 *
 * The End Of Frame notification sent by sensor subdev in its still capture
 * mode. If there is only a single VSYNC generated by the sensor at the
 * beginning of a frame transmission, CAMERIC does not issue the LastIrq
 * (end of frame) interrupt. And this notification is used to complete the
 * frame capture and returning a buffer to user-space. Subdev drivers should
 * call this notification from their last 'End of frame capture' interrupt.
 */
void cameric_sensor_notify(struct v4l2_subdev *sd, unsigned int notification,
			void *arg)
{
	struct cameric_source_info	*si;
	struct cameric_vid_buffer *buf;
	struct cameric_md *fmd;
	struct cameric_dev *cameric;
	unsigned long flags;

	if (sd == NULL)
		return;

	si = v4l2_get_subdev_hostdata(sd);
	fmd = entity_to_cameric_mdev(&sd->entity);

	spin_lock_irqsave(&fmd->slock, flags);

	cameric = si ? source_to_sensor_info(si)->host : NULL;
#if 0
	if (cameric && arg && notification == S5P_CAMERIC_TX_END_NOTIFY &&
	    test_bit(ST_CAPT_PEND, &cameric->state)) {
		unsigned long irq_flags;
		spin_lock_irqsave(&cameric->slock, irq_flags);
		if (!list_empty(&cameric->vid_cap.active_buf_q)) {
			buf = list_entry(cameric->vid_cap.active_buf_q.next,
					 struct cameric_vid_buffer, list);
			vb2_set_plane_payload(&buf->vb.vb2_buf, 0,
					      *((u32 *)arg));
		}
		cameric_capture_irq_handler(cameric, 1);
		cameric_deactivate_capture(cameric);
		spin_unlock_irqrestore(&cameric->slock, irq_flags);
	}
#endif
	spin_unlock_irqrestore(&fmd->slock, flags);
}

static int cameric_subdev_enum_mbus_code(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_mbus_code_enum *code)
{
	struct cameric_fmt *fmt;

	fmt = cameric_find_format(NULL, NULL, FMT_FLAGS_CAM, code->index);
	if (!fmt)
		return -EINVAL;
	code->code = fmt->mbus_code;
	return 0;
}

static int cameric_subdev_get_fmt(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_format *fmt)
{
	struct cameric_dev *cameric = v4l2_get_subdevdata(sd);
	struct cameric_ctx *ctx = cameric->vid_cap.ctx;
	struct cameric_frame *ff = &ctx->s_frame;
	struct v4l2_mbus_framefmt *mf;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		mf = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
		fmt->format = *mf;
		return 0;
	}

	mf = &fmt->format;
	mutex_lock(&cameric->lock);

	switch (fmt->pad) {
	case CAMERIC_SD_PAD_SOURCE:
		if (!WARN_ON(ff->fmt == NULL))
			mf->code = ff->fmt->mbus_code;
		/* Sink pads crop rectangle size */
		mf->width = ff->width;
		mf->height = ff->height;
		break;
	case CAMERIC_SD_PAD_SINK_FIFO:
		*mf = cameric->vid_cap.wb_fmt;
		break;
	case CAMERIC_SD_PAD_SINK_CAM:
	default:
		*mf = cameric->vid_cap.ci_fmt;
		break;
	}

	mutex_unlock(&cameric->lock);
	mf->colorspace = V4L2_COLORSPACE_JPEG;

	return 0;
}

static int cameric_subdev_set_fmt(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_format *fmt)
{
	struct cameric_dev *cameric = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *mf = &fmt->format;
	struct cameric_vid_cap *vc = &cameric->vid_cap;
	struct cameric_ctx *ctx = vc->ctx;
	struct cameric_frame *ff;
	struct cameric_fmt *ffmt;

	dbg("pad%d: code: 0x%x, %dx%d",
	    fmt->pad, mf->code, mf->width, mf->height);

	if (fmt->pad == CAMERIC_SD_PAD_SOURCE && vb2_is_busy(&vc->vbq))
		return -EBUSY;

	mutex_lock(&cameric->lock);
	ffmt = cameric_capture_try_format(ctx, &mf->width, &mf->height,
				       &mf->code, NULL, fmt->pad);
	mutex_unlock(&cameric->lock);
	mf->colorspace = V4L2_COLORSPACE_JPEG;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		mf = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
		*mf = fmt->format;
		return 0;
	}
	/* There must be a bug in the driver if this happens */
	if (WARN_ON(ffmt == NULL))
		return -EINVAL;

	/* Update RGB Alpha control state and value range */
	cameric_alpha_ctrl_update(ctx);

	cameric_capture_mark_jpeg_xfer(ctx, ffmt->color);
	if (fmt->pad == CAMERIC_SD_PAD_SOURCE) {
		ff = &ctx->d_frame;
		/* Sink pads crop rectangle size */
		mf->width = ctx->s_frame.width;
		mf->height = ctx->s_frame.height;
	} else {
		ff = &ctx->s_frame;
	}

	mutex_lock(&cameric->lock);
	set_frame_bounds(ff, mf->width, mf->height);

	if (fmt->pad == CAMERIC_SD_PAD_SINK_FIFO)
		vc->wb_fmt = *mf;
	else if (fmt->pad == CAMERIC_SD_PAD_SINK_CAM)
		vc->ci_fmt = *mf;

	ff->fmt = ffmt;

	/* Reset the crop rectangle if required. */
	if (!(fmt->pad == CAMERIC_SD_PAD_SOURCE && (ctx->state & CAMERIC_COMPOSE)))
		set_frame_crop(ff, 0, 0, mf->width, mf->height);

	if (fmt->pad != CAMERIC_SD_PAD_SOURCE)
		ctx->state &= ~CAMERIC_COMPOSE;

	mutex_unlock(&cameric->lock);
	return 0;
}

static int cameric_subdev_get_selection(struct v4l2_subdev *sd,
				     struct v4l2_subdev_pad_config *cfg,
				     struct v4l2_subdev_selection *sel)
{
	struct cameric_dev *cameric = v4l2_get_subdevdata(sd);
	struct cameric_ctx *ctx = cameric->vid_cap.ctx;
	struct cameric_frame *f = &ctx->s_frame;
	struct v4l2_rect *r = &sel->r;
	struct v4l2_rect *try_sel;

	if (sel->pad == CAMERIC_SD_PAD_SOURCE)
		return -EINVAL;

	mutex_lock(&cameric->lock);

	switch (sel->target) {
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		f = &ctx->d_frame;
	case V4L2_SEL_TGT_CROP_BOUNDS:
		r->width = f->o_width;
		r->height = f->o_height;
		r->left = 0;
		r->top = 0;
		mutex_unlock(&cameric->lock);
		return 0;

	case V4L2_SEL_TGT_CROP:
		try_sel = v4l2_subdev_get_try_crop(sd, cfg, sel->pad);
		break;
	case V4L2_SEL_TGT_COMPOSE:
		try_sel = v4l2_subdev_get_try_compose(sd, cfg, sel->pad);
		f = &ctx->d_frame;
		break;
	default:
		mutex_unlock(&cameric->lock);
		return -EINVAL;
	}

	if (sel->which == V4L2_SUBDEV_FORMAT_TRY) {
		sel->r = *try_sel;
	} else {
		r->left = f->offs_h;
		r->top = f->offs_v;
		r->width = f->width;
		r->height = f->height;
	}

	dbg("target %#x: l:%d, t:%d, %dx%d, f_w: %d, f_h: %d",
	    sel->pad, r->left, r->top, r->width, r->height,
	    f->f_width, f->f_height);

	mutex_unlock(&cameric->lock);
	return 0;
}

static int cameric_subdev_set_selection(struct v4l2_subdev *sd,
				     struct v4l2_subdev_pad_config *cfg,
				     struct v4l2_subdev_selection *sel)
{
	struct cameric_dev *cameric = v4l2_get_subdevdata(sd);
	struct cameric_ctx *ctx = cameric->vid_cap.ctx;
	struct cameric_frame *f = &ctx->s_frame;
	struct v4l2_rect *r = &sel->r;
	struct v4l2_rect *try_sel;
	unsigned long flags;

	if (sel->pad == CAMERIC_SD_PAD_SOURCE)
		return -EINVAL;

	mutex_lock(&cameric->lock);
	cameric_capture_try_selection(ctx, r, V4L2_SEL_TGT_CROP);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		try_sel = v4l2_subdev_get_try_crop(sd, cfg, sel->pad);
		break;
	case V4L2_SEL_TGT_COMPOSE:
		try_sel = v4l2_subdev_get_try_compose(sd, cfg, sel->pad);
		f = &ctx->d_frame;
		break;
	default:
		mutex_unlock(&cameric->lock);
		return -EINVAL;
	}

	if (sel->which == V4L2_SUBDEV_FORMAT_TRY) {
		*try_sel = sel->r;
	} else {
		spin_lock_irqsave(&cameric->slock, flags);
		set_frame_crop(f, r->left, r->top, r->width, r->height);
		set_bit(ST_CAPT_APPLY_CFG, &cameric->state);
		if (sel->target == V4L2_SEL_TGT_COMPOSE)
			ctx->state |= CAMERIC_COMPOSE;
		spin_unlock_irqrestore(&cameric->slock, flags);
	}

	dbg("target %#x: (%d,%d)/%dx%d", sel->target, r->left, r->top,
	    r->width, r->height);

	mutex_unlock(&cameric->lock);
	return 0;
}

static const struct v4l2_subdev_pad_ops cameric_subdev_pad_ops = {
	.enum_mbus_code = cameric_subdev_enum_mbus_code,
	.get_selection = cameric_subdev_get_selection,
	.set_selection = cameric_subdev_set_selection,
	.get_fmt = cameric_subdev_get_fmt,
	.set_fmt = cameric_subdev_set_fmt,
};

static const struct v4l2_subdev_ops cameric_subdev_ops = {
	.pad = &cameric_subdev_pad_ops,
};

/* Set default format at the sensor and host interface */
static int cameric_capture_set_default_format(struct cameric_dev *cameric)
{
	struct v4l2_format fmt = {
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.fmt.pix_mp = {
			.width		= CAMERIC_DEFAULT_WIDTH,
			.height		= CAMERIC_DEFAULT_HEIGHT,
			.pixelformat	= V4L2_PIX_FMT_YUYV,
			.field		= V4L2_FIELD_NONE,
			.colorspace	= V4L2_COLORSPACE_JPEG,
		},
	};

	return __cameric_capture_set_format(cameric, &fmt);
}

/* cameric->lock must be already initialized */
static int cameric_register_capture_device(struct cameric_dev *cameric,
				 struct v4l2_device *v4l2_dev)
{
	struct video_device *vfd = &cameric->vid_cap.ve.vdev;
	struct vb2_queue *q = &cameric->vid_cap.vbq;
	struct cameric_ctx *ctx;
	struct cameric_vid_cap *vid_cap;
	struct cameric_fmt *fmt;
	int ret = -ENOMEM;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->cameric_dev	 = cameric;
	ctx->in_path	 = CAMERIC_IO_CAMERA;
	ctx->out_path	 = CAMERIC_IO_DMA;
	ctx->state	 = CAMERIC_CTX_CAP;
	ctx->s_frame.fmt = cameric_find_format(NULL, NULL, FMT_FLAGS_CAM, 0);
	ctx->d_frame.fmt = ctx->s_frame.fmt;

	memset(vfd, 0, sizeof(*vfd));
	snprintf(vfd->name, sizeof(vfd->name), "cameric.%d.capture", cameric->id);

	vfd->fops	= &cameric_capture_fops;
	vfd->ioctl_ops	= &cameric_capture_ioctl_ops;
	vfd->v4l2_dev	= v4l2_dev;
	vfd->minor	= -1;
	vfd->release	= video_device_release_empty;
	vfd->queue	= q;
	vfd->lock	= &cameric->lock;

	video_set_drvdata(vfd, cameric);
	vid_cap = &cameric->vid_cap;
	vid_cap->active_buf_cnt = 0;
	vid_cap->reqbufs_count = 0;
	vid_cap->ctx = ctx;

	INIT_LIST_HEAD(&vid_cap->pending_buf_q);
	INIT_LIST_HEAD(&vid_cap->active_buf_q);

	memset(q, 0, sizeof(*q));
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	q->drv_priv = ctx;
	q->ops = &cameric_capture_qops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct cameric_vid_buffer);
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->lock = &cameric->lock;
	q->dev = &cameric->pdev->dev;

	ret = vb2_queue_init(q);
	if (ret)
		goto err_free_ctx;

	/* Default format configuration */
	fmt = cameric_find_format(NULL, NULL, FMT_FLAGS_CAM, 0);
	vid_cap->ci_fmt.width = CAMERIC_DEFAULT_WIDTH;
	vid_cap->ci_fmt.height = CAMERIC_DEFAULT_HEIGHT;
	vid_cap->ci_fmt.code = fmt->mbus_code;

	ctx->s_frame.width = CAMERIC_DEFAULT_WIDTH;
	ctx->s_frame.height = CAMERIC_DEFAULT_HEIGHT;
	ctx->s_frame.fmt = fmt;

	fmt = cameric_find_format(NULL, NULL, FMT_FLAGS_WRITEBACK, 0);
	vid_cap->wb_fmt = vid_cap->ci_fmt;
	vid_cap->wb_fmt.code = fmt->mbus_code;

	vid_cap->vd_pad.flags = MEDIA_PAD_FL_SINK;
	vfd->entity.function = MEDIA_ENT_F_PROC_VIDEO_SCALER;
	ret = media_entity_pads_init(&vfd->entity, 1, &vid_cap->vd_pad);
	if (ret)
		goto err_free_ctx;

	ret = cameric_ctrls_create(ctx);
	if (ret)
		goto err_me_cleanup;

	ret = video_register_device(vfd, VFL_TYPE_GRABBER, -1);
	if (ret)
		goto err_ctrl_free;

	v4l2_info(v4l2_dev, "Registered %s as /dev/%s\n",
		  vfd->name, video_device_node_name(vfd));

	vfd->ctrl_handler = &ctx->ctrls.handler;
	return 0;

err_ctrl_free:
	cameric_ctrls_delete(ctx);
err_me_cleanup:
	media_entity_cleanup(&vfd->entity);
err_free_ctx:
	kfree(ctx);
	return ret;
}

static int cameric_capture_subdev_registered(struct v4l2_subdev *sd)
{
	struct cameric_dev *cameric = v4l2_get_subdevdata(sd);
	int ret;

	if (cameric == NULL)
		return -ENXIO;

	//ret = cameric_register_m2m_device(cameric, sd->v4l2_dev);
	//if (ret)
		return ret;

	cameric->vid_cap.ve.pipe = v4l2_get_subdev_hostdata(sd);

	ret = cameric_register_capture_device(cameric, sd->v4l2_dev);
	if (ret) {
		//cameric_unregister_m2m_device(cameric);
		cameric->vid_cap.ve.pipe = NULL;
	}

	return ret;
}

static void cameric_capture_subdev_unregistered(struct v4l2_subdev *sd)
{
	struct cameric_dev *cameric = v4l2_get_subdevdata(sd);
	struct video_device *vdev;

	if (cameric == NULL)
		return;

	mutex_lock(&cameric->lock);

	//cameric_unregister_m2m_device(cameric);
	vdev = &cameric->vid_cap.ve.vdev;

	if (video_is_registered(vdev)) {
		video_unregister_device(vdev);
		media_entity_cleanup(&vdev->entity);
		cameric_ctrls_delete(cameric->vid_cap.ctx);
		cameric->vid_cap.ve.pipe = NULL;
	}
	kfree(cameric->vid_cap.ctx);
	cameric->vid_cap.ctx = NULL;

	mutex_unlock(&cameric->lock);
}

static const struct v4l2_subdev_internal_ops cameric_capture_sd_internal_ops = {
	.registered = cameric_capture_subdev_registered,
	.unregistered = cameric_capture_subdev_unregistered,
};

int cameric_initialize_capture_subdev(struct cameric_dev *cameric)
{
	struct v4l2_subdev *sd = &cameric->vid_cap.subdev;
	int ret;

	v4l2_subdev_init(sd, &cameric_subdev_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(sd->name, sizeof(sd->name), "CAMERIC.%d", cameric->id);

	cameric->vid_cap.sd_pads[CAMERIC_SD_PAD_SINK_CAM].flags = MEDIA_PAD_FL_SINK;
	cameric->vid_cap.sd_pads[CAMERIC_SD_PAD_SINK_FIFO].flags = MEDIA_PAD_FL_SINK;
	cameric->vid_cap.sd_pads[CAMERIC_SD_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sd->entity, CAMERIC_SD_PADS_NUM,
				cameric->vid_cap.sd_pads);
	if (ret)
		return ret;

	sd->entity.ops = &cameric_sd_media_ops;
	sd->internal_ops = &cameric_capture_sd_internal_ops;
	v4l2_set_subdevdata(sd, cameric);
	return 0;
}

void cameric_unregister_capture_subdev(struct cameric_dev *cameric)
{
	struct v4l2_subdev *sd = &cameric->vid_cap.subdev;

	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_set_subdevdata(sd, NULL);
}
