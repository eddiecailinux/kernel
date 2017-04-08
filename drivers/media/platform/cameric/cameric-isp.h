/*
 * Samsung EXYNOS4x12 CAMERIC-IS (Imaging Subsystem) driver
 *
 * Copyright (C) 2013 Samsung Electronics Co., Ltd.
 *
 * Authors: Sylwester Nawrocki <s.nawrocki@samsung.com>
 *          Younghwan Joo <yhwan.joo@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef CAMERIC_ISP_H_
#define CAMERIC_ISP_H_

#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/videodev2.h>

#include <media/media-entity.h>
#include <media/videobuf2-v4l2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mediabus.h>
#include <media/drv-intf/cameric.h>

extern int cameric_isp_debug;

#define isp_dbg(level, dev, fmt, arg...) \
	v4l2_dbg(level, cameric_isp_debug, dev, fmt, ## arg)

/* FIXME: revisit these constraints */
#define CAMERIC_ISP_SINK_WIDTH_MIN		(16 + 8)
#define CAMERIC_ISP_SINK_HEIGHT_MIN	(12 + 8)
#define CAMERIC_ISP_SOURCE_WIDTH_MIN	8
#define CAMERIC_ISP_SOURCE_HEIGHT_MIN	8
#define CAMERIC_ISP_CAC_MARGIN_WIDTH	16
#define CAMERIC_ISP_CAC_MARGIN_HEIGHT	12

#define CAMERIC_ISP_SINK_WIDTH_MAX		(4000 - 16)
#define CAMERIC_ISP_SINK_HEIGHT_MAX	(4000 + 12)
#define CAMERIC_ISP_SOURCE_WIDTH_MAX	4000
#define CAMERIC_ISP_SOURCE_HEIGHT_MAX	4000

#define CAMERIC_ISP_NUM_FORMATS		3
#define CAMERIC_ISP_REQ_BUFS_MIN		2
#define CAMERIC_ISP_REQ_BUFS_MAX		32

#define CAMERIC_ISP_SD_PAD_SINK		0
#define CAMERIC_ISP_SD_PAD_SRC_FIFO	1
#define CAMERIC_ISP_SD_PAD_SRC_DMA		2
#define CAMERIC_ISP_SD_PADS_NUM		3
#define CAMERIC_ISP_MAX_PLANES		1

/**
 * struct cameric_isp_frame - source/target frame properties
 * @width: full image width
 * @height: full image height
 * @rect: crop/composition rectangle
 */
struct cameric_isp_frame {
	u16 width;
	u16 height;
	struct v4l2_rect rect;
};

struct cameric_isp_ctrls {
	struct v4l2_ctrl_handler handler;

	/* Auto white balance */
	struct v4l2_ctrl *auto_wb;
	/* Auto ISO control cluster */
	struct {
		struct v4l2_ctrl *auto_iso;
		struct v4l2_ctrl *iso;
	};
	/* Adjust - contrast */
	struct v4l2_ctrl *contrast;
	/* Adjust - saturation */
	struct v4l2_ctrl *saturation;
	/* Adjust - sharpness */
	struct v4l2_ctrl *sharpness;
	/* Adjust - brightness */
	struct v4l2_ctrl *brightness;
	/* Adjust - hue */
	struct v4l2_ctrl *hue;

	/* Auto/manual exposure */
	struct v4l2_ctrl *auto_exp;
	/* Manual exposure value */
	struct v4l2_ctrl *exposure;
	/* AE/AWB lock/unlock */
	struct v4l2_ctrl *aewb_lock;
	/* Exposure metering mode */
	struct v4l2_ctrl *exp_metering;
	/* AFC */
	struct v4l2_ctrl *afc;
	/* ISP image effect */
	struct v4l2_ctrl *colorfx;
};

struct isp_video_buf {
	struct vb2_v4l2_buffer vb;
	dma_addr_t dma_addr[CAMERIC_ISP_MAX_PLANES];
	unsigned int index;
};

#define to_isp_video_buf(_b) container_of(_b, struct isp_video_buf, vb)

#define CAMERIC_ISP_MAX_BUFS	4

/**
 * struct cameric_is_video - cameric-is video device structure
 * @vdev: video_device structure
 * @type: video device type (CAPTURE/OUTPUT)
 * @pad: video device media (sink) pad
 * @pending_buf_q: pending buffers queue head
 * @active_buf_q: a queue head of buffers scheduled in hardware
 * @vb_queue: vb2 buffer queue
 * @active_buf_count: number of video buffers scheduled in hardware
 * @frame_count: counter of frames dequeued to user space
 * @reqbufs_count: number of buffers requested with REQBUFS ioctl
 * @format: current pixel format
 */
struct cameric_is_video {
	struct cameric_video_entity ve;
	enum v4l2_buf_type	type;
	struct media_pad	pad;
	struct list_head	pending_buf_q;
	struct list_head	active_buf_q;
	struct vb2_queue	vb_queue;
	unsigned int		reqbufs_count;
	unsigned int		buf_count;
	unsigned int		buf_mask;
	unsigned int		frame_count;
	int			streaming;
	struct isp_video_buf	*buffers[CAMERIC_ISP_MAX_BUFS];
	const struct cameric_fmt	*format;
	struct v4l2_pix_format_mplane pixfmt;
};

/* struct cameric_isp:state bit definitions */
#define ST_ISP_VID_CAP_BUF_PREP		0
#define ST_ISP_VID_CAP_STREAMING	1

/**
 * struct cameric_isp - CAMERIC-IS ISP data structure
 * @pdev: pointer to CAMERIC-IS platform device
 * @subdev: ISP v4l2_subdev
 * @subdev_pads: the ISP subdev media pads
 * @test_pattern: test pattern controls
 * @ctrls: v4l2 controls structure
 * @video_lock: mutex serializing video device and the subdev operations
 * @cac_margin_x: horizontal CAC margin in pixels
 * @cac_margin_y: vertical CAC margin in pixels
 * @state: driver state flags
 * @video_capture: the ISP block video capture device
 */
struct cameric_isp {
	struct platform_device		*pdev;
	struct v4l2_subdev		subdev;
	struct media_pad		subdev_pads[CAMERIC_ISP_SD_PADS_NUM];
	struct v4l2_mbus_framefmt	src_fmt;
	struct v4l2_mbus_framefmt	sink_fmt;
	struct v4l2_ctrl		*test_pattern;
	struct cameric_isp_ctrls		ctrls;

	struct mutex			video_lock;
	struct mutex			subdev_lock;

	unsigned int			cac_margin_x;
	unsigned int			cac_margin_y;

	unsigned long			state;

	struct cameric_is_video		video_capture;
};

#define ctrl_to_cameric_isp(_ctrl) \
	container_of(ctrl->handler, struct cameric_isp, ctrls.handler)

struct cameric_is;

int cameric_isp_subdev_create(struct cameric_isp *isp);
void cameric_isp_subdev_destroy(struct cameric_isp *isp);
void cameric_isp_irq_handler(struct cameric_is *is);
int cameric_is_create_controls(struct cameric_isp *isp);
int cameric_is_delete_controls(struct cameric_isp *isp);
const struct cameric_fmt *cameric_isp_find_format(const u32 *pixelformat,
					const u32 *mbus_code, int index);
#endif /* CAMERIC_ISP_H_ */
