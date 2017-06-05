/*
 *************************************************************************
 * Rockchip driver for CIF ISP 1.0
 * (Based on Intel driver for sofiaxxx)
 *
 * Copyright (C) 2015 Intel Mobile Communications GmbH
 * Copyright (C) 2016 Fuzhou Rockchip Electronics Co., Ltd.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *************************************************************************
 */

#ifndef _CIF_ISP10_COMMON_H
#define _CIF_ISP10_COMMON_H

#include <media/v4l2-ctrls.h>
#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-dma-contig.h>

#define CIF_ISP10_V4L2_SP_DEV_MAJOR 0
#define CIF_ISP10_V4L2_ISP_DEV_MAJOR 1
#define CIF_ISP10_V4L2_MP_DEV_MAJOR 2
#define CIF_ISP10_V4L2_DMA_DEV_MAJOR 3

#define SP_DEV 0
#define MP_DEV 1
#define DMA_DEV 2
#define ISP_DEV 3


struct cif_isp10_v4l2_fh;


#define GRP_ID_SENSOR           (1 << 8)
#define GRP_ID_MIPIPHY          (1 << 9)
#define GRP_ID_ISP              (1 << 10)
#define GRP_ID_ISP_MP			(1 << 11)
#define GRP_ID_ISP_SP			(1 << 12)

enum isp_subdev_index {
    IDX_SENSOR,
    IDX_MIPIPHY,
    IDX_ISP,
    IDX_MAX,
};


/*
 * struct cif_isp10_pipeline - An ISP hardware pipeline
 * @entities: Bitmask of entities in the pipeline (indexed by entity ID)
 */
struct cif_isp10_pipeline {
    struct media_pipeline pipe;
	struct v4l2_subdev *subdevs[IDX_MAX];
};

/* One structure per video node */
struct cif_isp10_v4l2_node {
	struct vb2_queue buf_queue;
	struct mutex qlock;
	struct video_device vdev;
	struct media_pad pad;
	struct cif_isp10_pipeline *pipe;
	int users;
	struct cif_isp10_v4l2_fh *owner;
};

static inline struct cif_isp10_v4l2_node *vdev_to_node(struct video_device *vdev)
{
	return container_of(vdev, struct cif_isp10_v4l2_node, vdev);
}

static inline struct cif_isp10_v4l2_node *queue_to_node(struct vb2_queue *q)
{
	return container_of(q, struct cif_isp10_v4l2_node, buf_queue);
}
#endif
