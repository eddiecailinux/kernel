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

#include <media/v4l2-common.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>
#include "cif_isp10_common.h"
#include "cif_isp10.h"
#include "cif_isp10_regs.h"
#include "cif_isp10_version.h"
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <media/v4l2-controls_rockchip.h>
#include <linux/pm_runtime.h>
#include <linux/pagemap.h>
#include <linux/slab.h>

#include "cif_isp10_mipiphy.h"

/* One structure per open file handle */
struct cif_isp10_v4l2_fh {
	enum cif_isp10_stream_id stream_id;
	struct v4l2_fh fh;
};

/* One structure per device */
struct cif_isp10_v4l2_device {
	struct cif_isp10_v4l2_node node[4];
};

/* spinlock define */
spinlock_t iowrite32_verify_lock;

static struct cif_isp10_v4l2_fh *to_fh(struct file *file)
{
	if (!file || !file->private_data)
		return NULL;

	return container_of(file->private_data, struct cif_isp10_v4l2_fh, fh);
}

static struct cif_isp10_v4l2_node *fh_to_node(struct cif_isp10_v4l2_fh *fh)
{
	struct video_device *vdev = fh ? fh->fh.vdev : NULL;

	if (!fh || !vdev)
		return NULL;

	return container_of(vdev, struct cif_isp10_v4l2_node, vdev);
}

static struct cif_isp10_buffer *to_cif_isp10_vb(struct vb2_v4l2_buffer *vb)
{
	return container_of(vb, struct cif_isp10_buffer, vb);
}

static struct vb2_queue *to_vb2_queue(
	struct file *file)
{
	struct cif_isp10_v4l2_fh *fh = to_fh(file);
	struct video_device *vdev = fh ? fh->fh.vdev : NULL;
	struct cif_isp10_v4l2_node *node = fh_to_node(fh);
	struct vb2_queue *q;

	if (unlikely(!vdev)) {
		cif_isp10_pltfrm_pr_err(NULL,
			"vdev is NULL\n");
		WARN_ON(1);
	}
	q = &node->buf_queue;
	if (unlikely(!q)) {
		cif_isp10_pltfrm_pr_err(NULL,
			"buffer queue is NULL\n");
		WARN_ON(1);
	}

	return q;
}

static enum cif_isp10_stream_id to_stream_id(
	struct file *file)
{
	struct cif_isp10_v4l2_fh *fh;

	if (unlikely(!file)) {
		cif_isp10_pltfrm_pr_err(NULL,
			"NULL file handle\n");
		WARN_ON(1);
	}
	fh = to_fh(file);
	if (unlikely(!fh)) {
		cif_isp10_pltfrm_pr_err(NULL,
			"fh is NULL\n");
		WARN_ON(1);
	}

	return fh->stream_id;
}

static struct cif_isp10_device *to_cif_isp10_device(
	struct vb2_queue *queue)
{
	return queue->drv_priv;
}

static enum cif_isp10_stream_id to_cif_isp10_stream_id(
	struct vb2_queue *queue)
{
	struct cif_isp10_v4l2_node *node =
		container_of(queue, struct cif_isp10_v4l2_node, buf_queue);
	struct video_device *vdev =
		&node->vdev;

	if (!strcmp(vdev->name, SP_VDEV_NAME))
		return CIF_ISP10_STREAM_SP;
	if (!strcmp(vdev->name, MP_VDEV_NAME))
		return CIF_ISP10_STREAM_MP;
	if (!strcmp(vdev->name, DMA_VDEV_NAME))
		return CIF_ISP10_STREAM_DMA;

	cif_isp10_pltfrm_pr_err(NULL,
		"unsupported/unknown device name %s\n", vdev->name);
	return -EINVAL;
}

static const char *cif_isp10_v4l2_buf_type_string(
	enum v4l2_buf_type buf_type)
{
	switch (buf_type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		return "VIDEO_CAPTURE";
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		return "VIDEO_OUTPUT";
	default:
		break;
	}
	return "UNKNOWN/UNSUPPORTED";
}

static int cif_isp10_v4l2_cid2cif_isp10_cid(u32 v4l2_cid)
{
	switch (v4l2_cid) {
	case V4L2_CID_FLASH_LED_MODE:
		return CIF_ISP10_CID_FLASH_MODE;
	case V4L2_CID_AUTOGAIN:
		return CIF_ISP10_CID_AUTO_GAIN;
	case V4L2_EXPOSURE_AUTO:
		return CIF_ISP10_CID_AUTO_EXPOSURE;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		return CIF_ISP10_CID_AUTO_WHITE_BALANCE;
	case V4L2_CID_BLACK_LEVEL:
		return CIF_ISP10_CID_BLACK_LEVEL;
	case V4L2_CID_WHITE_BALANCE_TEMPERATURE:
		return CIF_ISP10_CID_WB_TEMPERATURE;
	case V4L2_CID_EXPOSURE:
		return CIF_ISP10_CID_EXPOSURE_TIME;
	case V4L2_CID_GAIN:
		return CIF_ISP10_CID_ANALOG_GAIN;
	case V4L2_CID_FOCUS_ABSOLUTE:
		return CIF_ISP10_CID_FOCUS_ABSOLUTE;
	case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
		return CIF_ISP10_CID_AUTO_N_PRESET_WHITE_BALANCE;
	case V4L2_CID_SCENE_MODE:
		return CIF_ISP10_CID_SCENE_MODE;
	case V4L2_CID_COLORFX:
		return CIF_ISP10_CID_IMAGE_EFFECT;
	case V4L2_CID_JPEG_COMPRESSION_QUALITY:
		return CIF_ISP10_CID_JPEG_QUALITY;
	case V4L2_CID_HFLIP:
		return CIF_ISP10_CID_HFLIP;
	case V4L2_CID_VFLIP:
		return CIF_ISP10_CID_VFLIP;
	case V4L2_CID_ISO_SENSITIVITY:
		return CIF_ISP10_CID_ISO_SENSITIVITY;
	case RK_V4L2_CID_AUTO_FPS:
		return CIF_ISP10_CID_AUTO_FPS;
	default:
		cif_isp10_pltfrm_pr_err(NULL,
			"unknown/unsupported V4L2 CID 0x%x\n",
			v4l2_cid);
		break;
	}
	return -EINVAL;
}

static enum cif_isp10_image_effect cif_isp10_v4l2_colorfx2cif_isp10_ie(
	u32 v4l2_colorfx)
{
	switch (v4l2_colorfx) {
	case V4L2_COLORFX_SEPIA:
		return CIF_ISP10_IE_SEPIA;
	case V4L2_COLORFX_BW:
		return CIF_ISP10_IE_BW;
	case V4L2_COLORFX_NEGATIVE:
		return CIF_ISP10_IE_NEGATIVE;
	case V4L2_COLORFX_EMBOSS:
		return CIF_ISP10_IE_EMBOSS;
	case V4L2_COLORFX_SKETCH:
		return CIF_ISP10_IE_SKETCH;
	case V4L2_COLORFX_NONE:
		return CIF_ISP10_IE_NONE;
	default:
		cif_isp10_pltfrm_pr_err(NULL,
			"unknown/unsupported V4L2 COLORFX %d\n",
			v4l2_colorfx);
		break;
	}
	return -EINVAL;
}

static enum cif_isp10_pix_fmt cif_isp10_v4l2_pix_fmt2cif_isp10_pix_fmt(
	u32 v4l2_pix_fmt, struct vb2_queue *queue)
{
/*struct cif_isp10_v4l2_node *node =
 *	container_of(queue, struct cif_isp10_v4l2_node, buf_queue);
 *	struct video_device *vdev =
 *	&node->vdev;
 */

	switch (v4l2_pix_fmt) {
	case V4L2_PIX_FMT_GREY:
		return CIF_YUV400;
	case V4L2_PIX_FMT_YUV420:
		return CIF_YUV420P;
	case V4L2_PIX_FMT_YVU420:
		return CIF_YVU420P;
	case V4L2_PIX_FMT_NV12:
		return CIF_YUV420SP;
	case V4L2_PIX_FMT_NV21:
		return CIF_YVU420SP;
	case V4L2_PIX_FMT_YUYV:
		return CIF_YUV422I;
	case V4L2_PIX_FMT_UYVY:
		return CIF_UYV422I;
	case V4L2_PIX_FMT_YUV422P:
		return CIF_YUV422P;
	case V4L2_PIX_FMT_NV16:
		return CIF_YUV422SP;
	case V4L2_PIX_FMT_YUV444:
		return CIF_YUV444P;
	case V4L2_PIX_FMT_NV24:
		return CIF_YUV444SP;
	case V4L2_PIX_FMT_RGB565:
		return CIF_RGB565;
	case V4L2_PIX_FMT_RGB24:
		return CIF_RGB888;
	case V4L2_PIX_FMT_SBGGR8:
		return CIF_BAYER_SBGGR8;
	case V4L2_PIX_FMT_SGBRG8:
		return CIF_BAYER_SGBRG8;
	case V4L2_PIX_FMT_SGRBG8:
		return CIF_BAYER_SGRBG8;
	case V4L2_PIX_FMT_SRGGB8:
		return CIF_BAYER_SRGGB8;
	case V4L2_PIX_FMT_SBGGR10:
		return CIF_BAYER_SBGGR10;
	case V4L2_PIX_FMT_SGBRG10:
		return CIF_BAYER_SGBRG10;
	case V4L2_PIX_FMT_SGRBG10:
		return CIF_BAYER_SGRBG10;
	case V4L2_PIX_FMT_SRGGB10:
		return CIF_BAYER_SRGGB10;
	case V4L2_PIX_FMT_SBGGR12:
		return CIF_BAYER_SBGGR12;
	case V4L2_PIX_FMT_SGBRG12:
		return CIF_BAYER_SGBRG12;
	case V4L2_PIX_FMT_SGRBG12:
		return CIF_BAYER_SGRBG12;
	case V4L2_PIX_FMT_SRGGB12:
		return CIF_BAYER_SRGGB12;
	case V4L2_PIX_FMT_JPEG:
		return CIF_JPEG;
	default:
		cif_isp10_pltfrm_pr_err(NULL,
			"unknown or unsupported V4L2 pixel format %c%c%c%c\n",
			(u8)(v4l2_pix_fmt & 0xff),
			(u8)((v4l2_pix_fmt >> 8) & 0xff),
			(u8)((v4l2_pix_fmt >> 16) & 0xff),
			(u8)((v4l2_pix_fmt >> 24) & 0xff));
		return CIF_UNKNOWN_FORMAT;
	}
}


static const struct v4l2_subdev_pad_ops cifisp_isp_stream_subdev_pad_ops = {
        //.enum_mbus_code = fimc_lite_subdev_enum_mbus_code,
        //.get_selection = fimc_lite_subdev_get_selection,
        //.set_selection = fimc_lite_subdev_set_selection,
        //.get_fmt = fimc_lite_subdev_get_fmt,
        //.set_fmt = fimc_lite_subdev_set_fmt,
};

static const struct v4l2_subdev_video_ops cifisp_isp_stream_subdev_video_ops = {
        //.s_stream = fimc_lite_subdev_s_stream,
};

static const struct v4l2_subdev_core_ops cifisp_isp_stream_core_ops = {
        //.log_status = fimc_lite_log_status,
        //.s_power
};

static struct v4l2_subdev_ops cifisp_isp_stream_subdev_ops = {
        .core = &cifisp_isp_stream_core_ops,
        .video = &cifisp_isp_stream_subdev_video_ops,
        .pad = &cifisp_isp_stream_subdev_pad_ops,
};

static int cifisp_isp_stream_link_setup(struct media_entity *entity,
                                const struct media_pad *local,
                                const struct media_pad *remote, u32 flags)
{
	//TODO
	int ret = 0;
	return ret;
}

static const struct media_entity_operations cifisp_isp_stream_subdev_media_ops = {
        .link_setup = cifisp_isp_stream_link_setup,
};

static int register_stream_subdev(
	struct cif_isp10_stream *stream,
	struct v4l2_device *v4l2_dev
)
{
	struct cif_isp10_device *dev = NULL;
	struct v4l2_subdev *sd = NULL;
	struct v4l2_subdev_ops *subdev_ops = NULL;
	const struct media_entity_operations *me_ops = NULL;
	struct cif_isp10_stream_subdev *strm_subdev = NULL;
	const char *sd_name = '\0';
	int ret;

	if (stream->id == CIF_ISP10_STREAM_SP) {
		dev = container_of(stream, struct cif_isp10_device, sp_stream);
		sd = &dev->sp_strm_subdev.subdev;
		strm_subdev = &dev->sp_strm_subdev;
		subdev_ops = &cifisp_isp_stream_subdev_ops;
		me_ops	= &cifisp_isp_stream_subdev_media_ops;
		sd->grp_id = GRP_ID_ISP_SP;
		sd_name = "cif_isp10_subdev_sp";
	} else if (stream->id == CIF_ISP10_STREAM_MP) {
		dev = container_of(stream, struct cif_isp10_device, mp_stream);
		sd = &dev->mp_strm_subdev.subdev;
		strm_subdev = &dev->mp_strm_subdev;
		subdev_ops = &cifisp_isp_stream_subdev_ops;
		me_ops	= &cifisp_isp_stream_subdev_media_ops;
		sd->grp_id = GRP_ID_ISP_SP;
		sd_name = "cif_isp10_subdev_mp";
	} else {
		return -EFAULT;
	}
	v4l2_subdev_init(sd, subdev_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(sd->name, sizeof(sd->name), sd_name);

	strm_subdev->pads[0].flags = MEDIA_PAD_FL_SINK;
	strm_subdev->pads[1].flags = MEDIA_PAD_FL_SOURCE;
	//ret = media_entity_init(&sd->entity, 2,
	//						strm_subdev->pads, 0);
	ret = media_entity_pads_init(&sd->entity, 2, strm_subdev->pads);
	if (ret)
			return ret;

	//have internal ops ?
	//sd->internal_ops = &fimc_lite_subdev_internal_ops;
	sd->entity.ops = me_ops;
	sd->owner = THIS_MODULE;
	v4l2_set_subdevdata(sd, stream);

	//create pipeline ?
	//set subdev host data ?
	//v4l2_set_subdev_hostdata(sd, ep);

	ret = v4l2_device_register_subdev(v4l2_dev, sd);
	if (ret) {
		media_entity_cleanup(&sd->entity);
		v4l2_set_subdevdata(sd, NULL);
		v4l2_err(v4l2_dev, "Failed to register %s\n", sd_name);
	}
	return ret;

}

static void unregister_stream_subdev(struct cif_isp10_stream *stream)
{
	struct cif_isp10_device *dev = NULL;
	struct v4l2_subdev *sd = NULL;

	if (stream->id == CIF_ISP10_STREAM_SP) {
		dev = container_of(stream, struct cif_isp10_device, sp_stream);
		sd = &dev->sp_strm_subdev.subdev;
	} else if (stream->id == CIF_ISP10_STREAM_MP) {
		dev = container_of(stream, struct cif_isp10_device, mp_stream);
		sd = &dev->mp_strm_subdev.subdev;
	} else
		return;

	media_entity_cleanup(&sd->entity);
	v4l2_device_unregister_subdev(sd);
	v4l2_set_subdevdata(sd, NULL);
}

static int cif_isp10_v4l2_register_video_device(
	struct cif_isp10_device *dev,
	struct video_device *vdev,
	const char *name,
	int qtype,
	int major,
	const struct v4l2_file_operations *fops,
	const struct v4l2_ioctl_ops *ioctl_ops)
{
	int ret;
	struct cif_isp10_v4l2_node *node = vdev_to_node(vdev);

	vdev->release = video_device_release;
	strlcpy(vdev->name, name, sizeof(vdev->name));
	vdev->fops = fops;
	video_set_drvdata(vdev, dev);
	vdev->minor = -1;
	vdev->ioctl_ops = ioctl_ops;
	vdev->v4l2_dev = &dev->v4l2_dev;
	if (qtype == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		vdev->vfl_dir = VFL_DIR_TX;
		node->pad.flags = MEDIA_PAD_FL_SOURCE;
	} else {
		vdev->vfl_dir = VFL_DIR_RX;
		node->pad.flags = MEDIA_PAD_FL_SINK;
	}

	//ret = media_entity_init(&vdev->entity, 1, &node->pad, 0);
	ret = media_entity_pads_init(&vdev->entity, 1, &node->pad);
	if (ret < 0)
			return ret;

	ret = video_register_device(vdev, VFL_TYPE_GRABBER, major);
	if (IS_ERR_VALUE(ret)) {
		cif_isp10_pltfrm_pr_err(NULL,
			"video_register_device failed with error %d\n", ret);
		goto err;
	}

	cif_isp10_pltfrm_pr_info(NULL,
		"video device video%d.%d (%s) successfully registered\n",
		major, vdev->minor, name);

	return 0;
err:
	video_device_release(vdev);
	cif_isp10_pltfrm_pr_err(NULL,
		"failed with err %d\n", ret);
	return ret;
}

static int cif_isp10_v4l2_streamon(
	struct file *file,
	void *priv,
	enum v4l2_buf_type buf_type)
{
	int ret;
	struct vb2_queue *queue = to_vb2_queue(file);
	struct cif_isp10_device *dev = to_cif_isp10_device(queue);
	static u32 streamon_cnt_sp;
	static u32 streamon_cnt_mp;
	static u32 streamon_cnt_dma;
	//struct cif_isp10_v4l2_fh *fh = to_fh(file);
	//struct cif_isp10_v4l2_node *node = fh_to_node(fh);
	u32 stream_ids = to_stream_id(file);

/*
	if (node->owner != fh)
		return -EBUSY;
*/

	cif_isp10_pltfrm_pr_dbg(dev->dev, "%s(%d)\n",
		cif_isp10_v4l2_buf_type_string(queue->type),
		(stream_ids & CIF_ISP10_STREAM_MP) ? ++streamon_cnt_mp :
		((stream_ids & CIF_ISP10_STREAM_SP) ? ++streamon_cnt_sp :
		++streamon_cnt_dma));

	ret = vb2_streamon(queue, buf_type);
	if (IS_ERR_VALUE(ret)) {
		cif_isp10_pltfrm_pr_err(dev->dev,
			"videobuf_streamon failed\n");
		goto err;
	}

	{
		struct cif_isp10_strm_fmt strm_fmt;

		strm_fmt.frm_fmt.pix_fmt = CIF_YUV422I;
		strm_fmt.frm_fmt.width = 800;
		strm_fmt.frm_fmt.height = 600;
		strm_fmt.frm_fmt.quantization = 0;
		//TODO(zsq): revise: set default fmt.
		ret = cif_isp10_s_fmt(dev,
			to_stream_id(file),
			&strm_fmt,
			16 / 8 * strm_fmt.frm_fmt.width);
		if (IS_ERR_VALUE(ret))
			goto err;
	}

	ret = cif_isp10_streamon(dev, stream_ids);
	if (IS_ERR_VALUE(ret)) {
		goto err;
	}

	return 0;
err:
	(void)vb2_queue_release(queue);
	cif_isp10_pltfrm_pr_err(dev->dev, "failed with error %d\n", ret);
	return ret;
}

static int cif_isp10_v4l2_do_streamoff(
	struct file *file)
{
	int ret = 0;
	int err;
	struct vb2_queue *queue = to_vb2_queue(file);
	struct cif_isp10_device *dev = to_cif_isp10_device(queue);
	struct cif_isp10_v4l2_fh *fh = to_fh(file);
	struct cif_isp10_v4l2_node *node = fh_to_node(fh);
	u32 stream_ids = to_stream_id(file);

	cif_isp10_pltfrm_pr_dbg(dev->dev, "%s\n",
		cif_isp10_v4l2_buf_type_string(queue->type));

	if (node->owner != fh)
		return -EBUSY;

	err = cif_isp10_streamoff(dev, stream_ids);
	if (IS_ERR_VALUE(err))
		ret = -EFAULT;
	err = vb2_streamoff(queue, queue->type);
	if (IS_ERR_VALUE(err)) {
		cif_isp10_pltfrm_pr_err(dev->dev,
			"videobuf_streamoff failed with error %d\n", err);
		ret = -EFAULT;
	}

	vb2_queue_release(queue);

	if (IS_ERR_VALUE(ret))
		cif_isp10_pltfrm_pr_err(dev->dev,
			"failed with error %d\n", ret);

	return ret;
}

static int cif_isp10_v4l2_streamoff(
	struct file *file,
	void *priv,
	enum v4l2_buf_type buf_type)
{
	int ret = cif_isp10_v4l2_do_streamoff(file);

	if (IS_ERR_VALUE(ret))
		cif_isp10_pltfrm_pr_err(NULL,
			"failed with error %d\n", ret);

	return ret;
}

static int cif_isp10_v4l2_s_ctrl(
	struct file *file,
	void *priv,
	struct v4l2_control *vc)
{
	struct vb2_queue *queue = to_vb2_queue(file);
	struct cif_isp10_device *dev = to_cif_isp10_device(queue);
	enum cif_isp10_cid id =
		cif_isp10_v4l2_cid2cif_isp10_cid(vc->id);
	int val = vc->value;

	if (IS_ERR_VALUE(id))
		return id;

	switch (vc->id) {
	case V4L2_CID_COLORFX:
		val = cif_isp10_v4l2_colorfx2cif_isp10_ie(val);
		break;
	case V4L2_CID_FLASH_LED_MODE:
		if (vc->value == V4L2_FLASH_LED_MODE_NONE)
			val = CIF_ISP10_FLASH_MODE_OFF;
		else if (vc->value == V4L2_FLASH_LED_MODE_FLASH)
			val = CIF_ISP10_FLASH_MODE_FLASH;
		else if (vc->value == V4L2_FLASH_LED_MODE_TORCH)
			val = CIF_ISP10_FLASH_MODE_TORCH;
		else
			val = -EINVAL;
		break;
	default:
		break;
	}

	return cif_isp10_s_ctrl(dev, id, val);
}

static int cif_isp10_v4l2_s_fmt(
	struct file *file,
	void *priv,
	struct v4l2_format *f)
{
	int ret;
	struct vb2_queue *queue = to_vb2_queue(file);
	struct cif_isp10_device *dev = to_cif_isp10_device(queue);
	struct cif_isp10_v4l2_fh *fh = to_fh(file);
	struct cif_isp10_v4l2_node *node = fh_to_node(fh);
	struct cif_isp10_strm_fmt strm_fmt;

	cif_isp10_pltfrm_pr_dbg(NULL,
		"%s\n",
		cif_isp10_v4l2_buf_type_string(queue->type));

	if (node->owner && node->owner != fh)
		return -EBUSY;

	strm_fmt.frm_fmt.pix_fmt =
		cif_isp10_v4l2_pix_fmt2cif_isp10_pix_fmt(
			f->fmt.pix.pixelformat, queue);
	strm_fmt.frm_fmt.width = f->fmt.pix.width;
	strm_fmt.frm_fmt.height = f->fmt.pix.height;
/* strm_fmt.frm_fmt.quantization = f->fmt.pix.quantization; */
	strm_fmt.frm_fmt.quantization = 0;
	ret = cif_isp10_s_fmt(dev,
		to_stream_id(file),
		&strm_fmt,
		f->fmt.pix.bytesperline);
	if (IS_ERR_VALUE(ret))
		goto err;

	return 0;
err:
	cif_isp10_pltfrm_pr_err(NULL,
		"failed with error %d\n", ret);
	return ret;
}

/* existence of this function is checked by V4L2 */
static int cif_isp10_v4l2_g_fmt(
	struct file *file,
	void *priv,
	struct v4l2_format *f)
{
	return -EFAULT;
}

static int cif_isp10_v4l2_enum_framesizes(
	struct file *file,
	void *priv,
	struct v4l2_frmsizeenum *fsize)
{
	/* THIS FUNCTION IS UNDER CONSTRUCTION */
	int ret;
	struct vb2_queue *queue = to_vb2_queue(file);
	struct cif_isp10_device *dev = to_cif_isp10_device(queue);

	if (dev->num_sensors == 0) {
		cif_isp10_pltfrm_pr_err(NULL,
			"input has not yet been selected, cannot enumerate formats\n");
		ret = -ENODEV;
		goto err;
	}

	return -EINVAL;
err:
	cif_isp10_pltfrm_pr_err(NULL, "failed with error %d\n", ret);
	return ret;
}

/************************************************************/
static int cif_isp10_v4l2_vb2_queue_setup(struct vb2_queue *queue,
			const void *parg,
			unsigned int *num_buffers, unsigned int *num_planes,
			unsigned int sizes[], void *alloc_ctxs[])
{
	int ret;
	struct cif_isp10_device *dev = to_cif_isp10_device(queue);
	enum cif_isp10_stream_id strm = to_cif_isp10_stream_id(queue);

	if (*num_planes == 0) { /* Called from VIDIOC_REQBUFS */
		*num_buffers = 4;
		*num_planes = 1;
		ret = cif_isp10_calc_min_out_buff_size(dev, strm, &sizes[0]);
		alloc_ctxs[0] = dev->alloc_ctx;
		if (ret)
			return -EINVAL;
	} else { /* Called from VIDIOC_CREATE_BUFS */
		if (*num_buffers < 4 || *num_planes != 1) {
			cif_isp10_pltfrm_pr_warn(NULL, "Buffers invalid\n");
			return -EINVAL;
		}
	}

	cif_isp10_pltfrm_pr_dbg(NULL, "%s count %d, size %d\n",
		cif_isp10_v4l2_buf_type_string(queue->type),
		*num_buffers, sizes[0]);

	return 0;
}

static void cif_isp10_v4l2_vb2_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct cif_isp10_buffer* ispbuf = to_cif_isp10_vb(vbuf);
	struct vb2_queue *queue = vb->vb2_queue;
	struct cif_isp10_device *dev = to_cif_isp10_device(queue);
	enum cif_isp10_stream_id strm = to_cif_isp10_stream_id(queue);

	cif_isp10_pltfrm_pr_dbg(NULL,
		"buffer type %s\n",
		cif_isp10_v4l2_buf_type_string(queue->type));

	if (IS_ERR_VALUE(cif_isp10_qbuf(dev, strm, ispbuf)))
		cif_isp10_pltfrm_pr_err(NULL, "failed\n");

	vb2_set_plane_payload(vb, 0, 800 * 600 * 2); //TODO(zsq): revise
}

static void cif_isp10_v4l2_vb2_stop_streaming(struct vb2_queue *queue)
{
	struct cif_isp10_v4l2_node *node;
	enum cif_isp10_stream_id strm = to_cif_isp10_stream_id(queue);
	struct cif_isp10_stream *stream = NULL;
	struct cif_isp10_device *dev;

	node = queue_to_node(queue);

	dev = video_get_drvdata(&node->vdev);

	switch (strm) {
	case CIF_ISP10_STREAM_SP:
		stream = &dev->sp_stream;
		break;
	case CIF_ISP10_STREAM_MP:
		stream = &dev->mp_stream;
		break;
	case CIF_ISP10_STREAM_DMA:
		stream = &dev->dma_stream;
		break;
	default:
		cif_isp10_pltfrm_pr_err(NULL,
			"unknown/unsupported stream ID %d\n", strm);
		return;
	}

	//TODO(zsq): lock
	spin_lock(&dev->vbq_lock);
	while (!list_empty(&stream->buf_queue)) {
		struct cif_isp10_buffer *buf;
		buf = list_first_entry(&stream->buf_queue, struct cif_isp10_buffer, queue);
		list_del(&buf->queue);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}
	stream->curr_buf = NULL;
	stream->next_buf = NULL;
	spin_unlock(&dev->vbq_lock);
}

static struct vb2_ops cif_isp10_v4l2_vb2_ops = {
	.queue_setup	= cif_isp10_v4l2_vb2_queue_setup,
	.buf_queue	= cif_isp10_v4l2_vb2_queue,
	//.buf_cleanup	= cif_isp10_v4l2_vb2_release,
	//.buf_init	= cif_isp10_v4l2_vb2_init,
	.wait_prepare	= vb2_ops_wait_prepare,
	.wait_finish	= vb2_ops_wait_finish,
	.stop_streaming	= cif_isp10_v4l2_vb2_stop_streaming,
};

static int cif_isp10_init_vb2_queue(struct vb2_queue *q,
	struct cif_isp10_device *dev,
	enum v4l2_buf_type buf_type)
{
	struct cif_isp10_v4l2_node *node;

	memset(q, 0, sizeof(*q));
	node = queue_to_node(q);
	mutex_init(&node->qlock);

	q->type = buf_type;
	q->io_modes = VB2_MMAP | VB2_USERPTR;
	q->drv_priv = dev;
	q->ops = &cif_isp10_v4l2_vb2_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct vb2_buffer);
	q->min_buffers_needed	= 4;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->lock = &node->qlock;

	return vb2_queue_init(q);
}

/* fops **********************************************************************/
static int cif_isp10_v4l2_open(
	struct file *file)
{
	int ret;
	struct video_device *vdev = video_devdata(file);
	struct cif_isp10_device *dev = video_get_drvdata(vdev);
	struct cif_isp10_v4l2_fh *fh;
	struct cif_isp10_v4l2_node *node;
	enum v4l2_buf_type buf_type;
	enum cif_isp10_stream_id stream_id;
	struct cif_isp10_v4l2_device *cif_isp10_v4l2_dev =
		(struct cif_isp10_v4l2_device *)dev->nodes;

	cif_isp10_pltfrm_pr_dbg(NULL,
		"video device video%d.%d (%s)\n",
		vdev->num, vdev->minor, vdev->name);

	if (vdev->minor == cif_isp10_v4l2_dev->node[SP_DEV].vdev.minor) {
		buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		stream_id = CIF_ISP10_STREAM_SP;
	} else if (vdev->minor == cif_isp10_v4l2_dev->node[MP_DEV].vdev.minor) {
		buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		stream_id = CIF_ISP10_STREAM_MP;
	} else if (vdev->minor == cif_isp10_v4l2_dev->node[DMA_DEV].vdev.minor) {
		buf_type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
		stream_id = CIF_ISP10_STREAM_DMA;
	} else {
		cif_isp10_pltfrm_pr_err(NULL,
			"invalid video device video%d.%d (%s)\n",
			vdev->num, vdev->minor, vdev->name);
		ret = -EINVAL;
		goto err;
	}

	fh = kzalloc(sizeof(*fh), GFP_KERNEL);
	if (!fh) {
		cif_isp10_pltfrm_pr_err(NULL,
			"memory allocation failed\n");
		return -ENOMEM;
	}
	fh->stream_id = stream_id;

	file->private_data = &fh->fh;
	v4l2_fh_init(&fh->fh, vdev);
	v4l2_fh_add(&fh->fh);

	node = fh_to_node(fh);
	if (++node->users > 1)
		return 0;

	/* First open of the device, so initialize everything */
	node->owner = fh;

	//dev->alloc_ctx = vb2_dma_contig_init_ctx(dev->dev);
	cif_isp10_init_vb2_queue(to_vb2_queue(file), dev, buf_type);
	vdev->queue = to_vb2_queue(file);

	ret = cif_isp10_init(dev, to_stream_id(file));
	if (IS_ERR_VALUE(ret)) {
		v4l2_fh_del(&fh->fh);
		v4l2_fh_exit(&fh->fh);
		kfree(fh);
		node->users--;
		goto err;
	}

	return 0;
err:
	kfree(fh);
	node->owner = NULL;
	cif_isp10_pltfrm_pr_err(NULL,
		"failed with error %d\n", ret);
	return ret;
}

static int cif_isp10_v4l2_release(struct file *file)
{
	int ret = 0;
	struct vb2_queue *queue = to_vb2_queue(file);
	struct cif_isp10_device *dev = to_cif_isp10_device(queue);
	struct cif_isp10_v4l2_fh *fh = to_fh(file);
	struct cif_isp10_v4l2_node *node = fh_to_node(fh);
	enum cif_isp10_stream_id stream_id = to_stream_id(file);

	cif_isp10_pltfrm_pr_dbg(dev->dev, "%s\n",
		cif_isp10_v4l2_buf_type_string(queue->type));

	if (node->users) {
		--node->users;
	} else {
		cif_isp10_pltfrm_pr_warn(dev->dev,
			"number of users for this device is already 0\n");
		return 0;
	}

	if (!node->users) {
		if (queue->streaming)
			if (IS_ERR_VALUE(cif_isp10_v4l2_do_streamoff(file)))
				cif_isp10_pltfrm_pr_warn(dev->dev,
					"streamoff failed\n");

		/* Last close, so uninitialize hardware */
		ret = cif_isp10_release(dev, stream_id);
	}

	if (node->owner == fh)
		node->owner = NULL;

	v4l2_fh_del(&fh->fh);
	v4l2_fh_exit(&fh->fh);
	kfree(fh);

	if (IS_ERR_VALUE(ret))
		cif_isp10_pltfrm_pr_err(dev->dev,
			"failed with error %d\n", ret);
	return ret;
}

/*
static unsigned int cif_isp10_v4l2_poll(
	struct file *file,
	struct poll_table_struct *wait)
{
	struct cif_isp10_v4l2_fh *fh = to_fh(file);
	int ret = 0;
	struct vb2_queue *queue = to_vb2_queue(file);
	unsigned long req_events = poll_requested_events(wait);

	cif_isp10_pltfrm_pr_dbg(NULL, "%s\n",
		cif_isp10_v4l2_buf_type_string(queue->type));

	if (v4l2_event_pending(&fh->fh))
		ret = POLLPRI;
	else if (req_events & POLLPRI)
		poll_wait(file, &fh->fh.wait, wait);

	if (!(req_events & (POLLIN | POLLOUT | POLLRDNORM)))
		return ret;

	ret |= vb2_fop_poll(file, wait);
	if (ret & POLLERR) {
		cif_isp10_pltfrm_pr_err(NULL,
			"videobuf_poll_stream failed with error 0x%x\n", ret);
	}
	return ret;
}
*/

/*
 * VMA operations.
 */
/*
static void cif_isp10_v4l2_vm_open(struct vm_area_struct *vma)
{
	struct cif_isp10_metadata_s *metadata =
		(struct cif_isp10_metadata_s *)vma->vm_private_data;

	metadata->vmas++;
}

static void cif_isp10_v4l2_vm_close(struct vm_area_struct *vma)
{
	struct cif_isp10_metadata_s *metadata =
		(struct cif_isp10_metadata_s *)vma->vm_private_data;

	metadata->vmas--;
}

static const struct vm_operations_struct cif_isp10_vm_ops = {
	.open		= cif_isp10_v4l2_vm_open,
	.close		= cif_isp10_v4l2_vm_close,
};

int cif_isp10_v4l2_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct vb2_queue *queue = to_vb2_queue(file);
	struct cif_isp10_device *dev = to_cif_isp10_device(queue);
	enum cif_isp10_stream_id strm = to_stream_id(file);
	int retval;

	retval = cif_isp10_mmap(dev, strm, vma);
	if (retval < 0)
		goto done;

	vma->vm_ops          = &cif_isp10_vm_ops;
	vma->vm_flags       |= VM_DONTEXPAND | VM_DONTDUMP;
	cif_isp10_v4l2_vm_open(vma);

done:
	return retval;
}
*/

const struct v4l2_file_operations cif_isp10_v4l2_fops = {
	.open = cif_isp10_v4l2_open,
	.unlocked_ioctl = video_ioctl2,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = video_ioctl2,
#endif
	.release = cif_isp10_v4l2_release,
	//.poll = cif_isp10_v4l2_poll,
	//.mmap = cif_isp10_v4l2_mmap,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
};

/*TBD: clean up code below this line******************************************/

static int cif_isp10_v4l2_querycap(struct file *file,
			 void *priv, struct v4l2_capability *cap)
{
	int ret = 0;
	struct vb2_queue *queue = to_vb2_queue(file);
	struct video_device *vdev = video_devdata(file);
	struct cif_isp10_device *dev = to_cif_isp10_device(queue);
	u32 stream_ids = to_stream_id(file);

	strcpy(cap->driver, DRIVER_NAME);
	strlcpy(cap->card, vdev->name, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info),
		"platform:" DRIVER_NAME "-%03i",
		dev->dev_id);

	if (stream_ids == CIF_ISP10_STREAM_SP)
		cap->device_caps = V4L2_CAP_VIDEO_CAPTURE |
			V4L2_CAP_STREAMING;
	else if (stream_ids == CIF_ISP10_STREAM_MP)
		cap->device_caps = V4L2_CAP_VIDEO_CAPTURE |
			V4L2_CAP_STREAMING;
	else if (stream_ids == CIF_ISP10_STREAM_DMA)
		cap->device_caps = V4L2_CAP_VIDEO_M2M;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE |
			    V4L2_CAP_STREAMING |
			    V4L2_CAP_VIDEO_M2M |
			    V4L2_CAP_DEVICE_CAPS;

	return ret;
}

static int cif_isp10_v4l2_subscribe_event(struct v4l2_fh *fh,
				const struct v4l2_event_subscription *sub)
{
	if (sub->type != V4L2_EVENT_FRAME_SYNC)
		return -EINVAL;
	return v4l2_event_subscribe(fh, sub, 16, NULL);
}

static int cif_isp10_v4l2_unsubscribe_event(struct v4l2_fh *fh,
				const struct v4l2_event_subscription *sub)
{
	return v4l2_event_unsubscribe(fh, sub);
}

static void cif_isp10_v4l2_event(
	struct cif_isp10_device *dev,
	__u32 frame_sequence)
{
	struct cif_isp10_v4l2_device *cif_isp10_v4l2_dev =
		(struct cif_isp10_v4l2_device *)dev->nodes;
	struct v4l2_event ev;

	memset(&ev, 0, sizeof(ev));
	ev.type = V4L2_EVENT_FRAME_SYNC;
	ev.u.frame_sync.frame_sequence = frame_sequence;
	v4l2_event_queue(&cif_isp10_v4l2_dev->node[SP_DEV].vdev, &ev);
}

static void cif_isp10_v4l2_requeue_bufs(
	struct cif_isp10_device *dev,
	enum cif_isp10_stream_id stream_id)
{
	struct vb2_buffer *buf;
	struct vb2_queue *q = NULL;
	struct vb2_v4l2_buffer *vbuf;
	struct cif_isp10_buffer *isp_buf;
	struct cif_isp10_v4l2_device *cif_isp10_v4l2_dev =
		(struct cif_isp10_v4l2_device *)dev->nodes;

	printk("TODO(zsq): revise %s\n", __func__);
	return;

	if (stream_id == CIF_ISP10_STREAM_SP)
		q = &cif_isp10_v4l2_dev->node[SP_DEV].buf_queue;
	else if (stream_id == CIF_ISP10_STREAM_MP)
		q = &cif_isp10_v4l2_dev->node[MP_DEV].buf_queue;
	else if (stream_id == CIF_ISP10_STREAM_DMA)
		q = &cif_isp10_v4l2_dev->node[DMA_DEV].buf_queue;
	else
		WARN_ON(1);

	dev = to_cif_isp10_device(q);

	list_for_each_entry(buf, &q->queued_list, queued_entry) {
		vbuf = to_vb2_v4l2_buffer(buf);
		isp_buf = to_cif_isp10_vb(vbuf);
		if (!IS_ERR_VALUE(cif_isp10_qbuf(
			to_cif_isp10_device(q), stream_id, isp_buf))) {
			spin_lock(&dev->vbreq_lock);
			if ((buf->state == VB2_BUF_STATE_QUEUED) ||
			    (buf->state == VB2_BUF_STATE_ACTIVE) ||
			    (buf->state == VB2_BUF_STATE_DONE))
				buf->state = VB2_BUF_STATE_QUEUED;
			else
				cif_isp10_pltfrm_pr_err(NULL,
					"ERR: buf->state is: %d\n",
					buf->state);
			spin_unlock(&dev->vbreq_lock);
		} else {
			cif_isp10_pltfrm_pr_err(NULL,
				"failed for buffer %d\n", buf->index);
		}
	}
}

//TODO remove RK_VIDIOC_SENSOR_MODE_DATA, revise later

static int cif_isp10_v4l2_enum_input(struct file *file, void *priv,
			   struct v4l2_input *input)
{
	struct vb2_queue *queue = to_vb2_queue(file);
	struct cif_isp10_device *dev = to_cif_isp10_device(queue);
	const char *inp_name;

	if ((queue->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) &&
		(queue->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)) {
		cif_isp10_pltfrm_pr_err(NULL,
			"wrong buffer queue %d\n", queue->type);
		return -EINVAL;
	}

	inp_name = cif_isp10_g_input_name(dev, input->index);
	if (IS_ERR_OR_NULL(inp_name))
		return -EINVAL;

	input->type = V4L2_INPUT_TYPE_CAMERA;
	input->std = V4L2_STD_UNKNOWN;
	strncpy(input->name, inp_name, sizeof(input->name)-1);

	return 0;
}

/* ================================================================= */

static int mainpath_g_ctrl(
	struct file *file,
	void *priv,
	struct v4l2_control *vc)
{
	int ret = -EINVAL;

	switch (vc->id) {
	default:
		return -EINVAL;
	}
	return ret;
}

#ifdef NOT_YET
static int mainpath_try_fmt_cap(struct v4l2_format *f)
{
	int ifmt = 0;
	struct v4l2_pix_format *pix = &f->fmt.pix;

	cif_isp10_pltfrm_pr_dbg(NULL, "\n");

	for (ifmt = 0; ifmt < get_cif_isp10_output_format_size(); ifmt++) {
		if (pix->pixelformat ==
		get_cif_isp10_output_format(ifmt)->fourcc)
			break;
	}

	if (ifmt == get_cif_isp10_output_format_size())
		ifmt = 0;

	pix->bytesperline = pix->width *
		get_cif_isp10_output_format(ifmt)->depth / 8;

	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_YUV422P:
	case V4L2_PIX_FMT_NV16:
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YVU420:
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_GREY:
	case V4L2_PIX_FMT_YUV444:
	case V4L2_PIX_FMT_NV24:
	case V4L2_PIX_FMT_JPEG:
		pix->colorspace = V4L2_COLORSPACE_JPEG;
		break;
	case V4L2_PIX_FMT_RGB32:
	case V4L2_PIX_FMT_BGR32:
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB565X:
	case V4L2_PIX_FMT_SGRBG10:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		break;
	default:
		WARN_ON(1);
		break;
	}

	return 0;
}
#endif

static int cif_isp10_v4l2_enum_fmt_cap(struct file *file, void *fh,
			     struct v4l2_fmtdesc *f)
{
	int ret = 0;
	int xgold_num_format = 0;

	xgold_num_format = get_cif_isp10_output_format_desc_size();
	if ((f->index >= xgold_num_format) ||
	(get_cif_isp10_output_format_desc(f->index)->pixelformat == 0)) {
		cif_isp10_pltfrm_pr_err(NULL, "index %d\n", f->index);
		return -EINVAL;
	}
	strlcpy(f->description,
		get_cif_isp10_output_format_desc(f->index)->description,
			sizeof(f->description));
	f->pixelformat =
	get_cif_isp10_output_format_desc(f->index)->pixelformat;
	f->flags = get_cif_isp10_output_format_desc(f->index)->flags;

	return ret;
}

int cif_isp10_v4l2_cropcap(
	struct file *file,
	void *fh,
	struct v4l2_cropcap *a)
{
	int ret = 0;
	struct vb2_queue *queue = to_vb2_queue(file);
	struct cif_isp10_device *dev = to_cif_isp10_device(queue);
	u32 target_width, target_height;
	u32 h_offs, v_offs;

	if ((dev->config.input_sel == CIF_ISP10_INP_DMA) ||
		(dev->config.input_sel == CIF_ISP10_INP_DMA_IE)) {
		/* calculate cropping for aspect ratio */
		ret = cif_isp10_calc_isp_cropping(dev,
			&dev->isp_dev.input_width, &dev->isp_dev.input_height,
			&h_offs, &v_offs);

		/* Get output size */
		ret = cif_isp10_get_target_frm_size(dev,
			&target_width, &target_height);
		if (ret < 0) {
			cif_isp10_pltfrm_pr_err(dev->dev,
				"failed to get target frame size\n");
			return ret;
		}

		cif_isp10_pltfrm_pr_dbg(dev->dev,
			"CIF_IN_W=%d, CIF_IN_H=%d, ISP_IN_W=%d, ISP_IN_H=%d, target_width=%d, target_height=%d\n",
			dev->config.isp_config.input->width,
			dev->config.isp_config.input->height,
			dev->isp_dev.input_width,
			dev->isp_dev.input_height,
			target_width,
			target_height);

		/* This is the input to Bayer after input formatter cropping */
		a->defrect.top = 0;
		a->defrect.left = 0;
		a->defrect.width = dev->isp_dev.input_width;
		a->defrect.height = dev->isp_dev.input_height;
		/* This is the minimum cropping window for the IS module */
		a->bounds.width = 2;
		a->bounds.height = 2;
		a->bounds.top = (a->defrect.height - a->bounds.height) / 2;
		a->bounds.left = (a->defrect.width - a->bounds.width) / 2;

		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	} else if (!CIF_ISP10_INP_IS_DMA(dev->config.input_sel)) {
		/* calculate cropping for aspect ratio */
		ret = cif_isp10_calc_isp_cropping(dev,
			&dev->isp_dev.input_width, &dev->isp_dev.input_height,
			&h_offs, &v_offs);

		/* Get output size */
		ret = cif_isp10_get_target_frm_size(dev,
			&target_width, &target_height);
		if (ret < 0) {
			cif_isp10_pltfrm_pr_err(dev->dev,
				"failed to get target frame size\n");
			return ret;
		}

		/* This is the input to Bayer after input formatter cropping */
		a->defrect.top =
			v_offs + dev->config.isp_config.input->defrect.top;
		a->defrect.left =
			h_offs + dev->config.isp_config.input->defrect.left;
		a->defrect.width = dev->isp_dev.input_width;
		a->defrect.height = dev->isp_dev.input_height;

		a->bounds.top = 0;
		a->bounds.left = 0;
		a->bounds.width = dev->config.isp_config.input->width;
		a->bounds.height = dev->config.isp_config.input->height;
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	} else {
		cif_isp10_pltfrm_pr_err(dev->dev,
			"invalid input\n");
	}

	cif_isp10_pltfrm_pr_dbg(dev->dev,
		"v4l2_cropcap: defrect(%d,%d,%d,%d) bounds(%d,%d,%d,%d)\n",
		a->defrect.width,
		a->defrect.height,
		a->defrect.left,
		a->defrect.top,
		a->bounds.width,
		a->bounds.height,
		a->bounds.left,
		a->bounds.top);

	return ret;
}

int cif_isp10_v4l2_g_crop(struct file *file, void *fh, struct v4l2_crop *a)
{
	return 0;
}

/*
 * This is a write only function, so the upper layer
 * will ignore the changes to 'a'. So don't use 'a' to pass
 * the actual cropping parameters, the upper layer
 * should call g_crop to get the actual window.
 */
int cif_isp10_v4l2_s_crop(
	struct file *file,
	void *fh,
	const struct v4l2_crop *a)
{
	return 0;
}

const struct v4l2_ioctl_ops cif_isp10_v4l2_sp_ioctlops = {
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_streamon = cif_isp10_v4l2_streamon,
	.vidioc_streamoff = cif_isp10_v4l2_streamoff,
	.vidioc_enum_input = cif_isp10_v4l2_enum_input,
	.vidioc_s_ctrl = cif_isp10_v4l2_s_ctrl,
	.vidioc_s_fmt_vid_overlay = cif_isp10_v4l2_s_fmt,
	.vidioc_g_fmt_vid_overlay = cif_isp10_v4l2_g_fmt,
	.vidioc_g_fmt_vid_cap = cif_isp10_v4l2_g_fmt,
	.vidioc_s_fmt_vid_cap = cif_isp10_v4l2_s_fmt,
	.vidioc_querycap = cif_isp10_v4l2_querycap,
	.vidioc_cropcap = cif_isp10_v4l2_cropcap,
	.vidioc_s_crop = cif_isp10_v4l2_s_crop,
	.vidioc_g_crop = cif_isp10_v4l2_g_crop,
	.vidioc_subscribe_event = cif_isp10_v4l2_subscribe_event,
	.vidioc_unsubscribe_event = cif_isp10_v4l2_unsubscribe_event,
};

const struct v4l2_ioctl_ops cif_isp10_v4l2_mp_ioctlops = {
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_streamon = cif_isp10_v4l2_streamon,
	.vidioc_streamoff = cif_isp10_v4l2_streamoff,
	.vidioc_enum_input = cif_isp10_v4l2_enum_input,
	.vidioc_g_ctrl = mainpath_g_ctrl,
	.vidioc_s_ctrl = cif_isp10_v4l2_s_ctrl,
	.vidioc_s_fmt_vid_cap = cif_isp10_v4l2_s_fmt,
	.vidioc_g_fmt_vid_cap = cif_isp10_v4l2_g_fmt,
	.vidioc_enum_fmt_vid_cap = cif_isp10_v4l2_enum_fmt_cap,
	.vidioc_enum_framesizes = cif_isp10_v4l2_enum_framesizes,
	.vidioc_querycap = cif_isp10_v4l2_querycap,
	.vidioc_cropcap = cif_isp10_v4l2_cropcap,
	.vidioc_s_crop = cif_isp10_v4l2_s_crop,
	.vidioc_g_crop = cif_isp10_v4l2_g_crop,
};

const struct v4l2_ioctl_ops cif_isp10_v4l2_dma_ioctlops = {
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_streamon = cif_isp10_v4l2_streamon,
	.vidioc_streamoff = cif_isp10_v4l2_streamoff,
	.vidioc_s_fmt_vid_out = cif_isp10_v4l2_s_fmt,
	.vidioc_g_fmt_vid_out = cif_isp10_v4l2_g_fmt,
	.vidioc_cropcap = cif_isp10_v4l2_cropcap,
	.vidioc_s_crop = cif_isp10_v4l2_s_crop,
	.vidioc_g_crop = cif_isp10_v4l2_g_crop,
};

//TODO(zsq): add soc private data (rk3288 vs rk3399)
static const struct of_device_id cif_isp10_v4l2_of_match[] = {
	{.compatible = "rockchip,rk3288-cif-isp"},
	{.compatible = "rockchip,rk3399-cif-isp"},
	{},
};

static unsigned int cif_isp10_v4l2_dev_cnt;

/*
 * sub devices:
 * camera module: sensor, vcm, flashlight
 * mipi phy
 * isp
 * isp 3a statistics (video node)
 * isp main path (video node)
 * isp self path (video node)
 */

/*
 * media_entity_init
 * media_entity_cleanup
 * media_entity_create_link
 * media_entity_remote_pad
 * media_entity_type
 * media_entity_to_v4l2_subdev
 * media_entity_pipeline_start
 * media_entity_pipeline_stop
 * media_entity_graph_walk_start
 * media_entity_graph_walk_next
 * struct media_entity_operations
 * media_device::link_notify
 * v4l2_device_register_subdev
 * v4l2_subdev_init
 * v4l2_async_notifier_register
 * v4l2_device_register_subdev_nodes
 * of_graph_get_next_endpoint
 * of_graph_get_remote_port
 */

/*
 * pipeline management
 */

static int	__isp_pipeline_prepare(struct cif_isp10_pipeline *p,
	struct media_entity *me) 
{
	struct v4l2_subdev *sd;
	struct v4l2_subdev *sensor = NULL;
	int i;
	
	for (i = 0; i < IDX_MAX; i++)
			p->subdevs[i] = NULL;
	
	while (1) {
		struct media_pad *pad = NULL;

		/* Find remote source pad */
		for (i = 0; i < me->num_pads; i++) {
				struct media_pad *spad = &me->pads[i];
				if (!(spad->flags & MEDIA_PAD_FL_SINK))
						continue;
				pad = media_entity_remote_pad(spad);
				if (pad)
						break;
		}

		//if (pad == NULL ||
		//	media_entity_type(pad->entity) != MEDIA_ENT_T_V4L2_SUBDEV)
		//		break;
		if (pad == NULL || is_media_entity_v4l2_subdev(pad->entity))
			break;
		sd = media_entity_to_v4l2_subdev(pad->entity);

		switch (sd->grp_id) {
		case GRP_ID_SENSOR:
				sensor = sd;
				p->subdevs[IDX_SENSOR] = sd;
				break;
		case GRP_ID_MIPIPHY:
				p->subdevs[IDX_MIPIPHY] = sd;
				break;
		case GRP_ID_ISP:
				p->subdevs[IDX_ISP] = sd;
				break;
		default:
				break;
		}
		me = &sd->entity;
		if (me->num_pads == 1)
				break;
	}
	return 0;
}

static int isp_pipeline_suspend(struct cif_isp10_pipeline *p) 
{
	//TODO
	return 0;
}

static int isp_pipeline_resume(struct cif_isp10_pipeline *p) 
{
	//TODO
	return 0;
}

static int __subdev_set_power(struct v4l2_subdev *sd, int on)
{
	int *use_count;
	int ret;

	if (sd == NULL)
	    return -ENXIO;

	use_count = &sd->entity.use_count;
	if (on && (*use_count)++ > 0)
	    return 0;
	else if (!on && (*use_count == 0 || --(*use_count) > 0))
	    return 0;
	ret = v4l2_subdev_call(sd, core, s_power, on);

	return ret != -ENOIOCTLCMD ? ret : 0;
}

static int isp_pipeline_s_power(struct cif_isp10_pipeline *p, bool on)
{
	static const u8 seq[2][IDX_MAX] = {
		{ IDX_ISP, IDX_SENSOR, IDX_MIPIPHY },
		{ IDX_ISP, IDX_SENSOR, IDX_MIPIPHY },
	};
	int i, ret = 0;

	if (p->subdevs[IDX_SENSOR] == NULL)
		return -ENXIO;
	
	for (i = 0; i < IDX_MAX - 1; i++) {
		unsigned int idx = seq[on][i];

		ret = __subdev_set_power(p->subdevs[idx], on);
		if (ret < 0 && ret != -ENXIO)
				goto error;
	}
	return 0;
	error:
	for (; i >= 0; i--) {
		unsigned int idx = seq[on][i];
		__subdev_set_power(p->subdevs[idx], !on);
	}
	return ret;
}

int  isp_pipeline_open(struct cif_isp10_pipeline *p, struct media_entity *me, bool prepare) 
{
	int ret;
	struct v4l2_subdev *sd;
	
	if (WARN_ON(p == NULL || me == NULL))
			return -EINVAL;
	
	if (prepare)
			__isp_pipeline_prepare(p, me);
	
	sd = p->subdevs[IDX_SENSOR];
	if (sd == NULL)
			return -EINVAL;
	
	ret = isp_pipeline_s_power(p, 1);
	if (!ret)
			return 0;
	
	return ret;
}

int  isp_pipeline_close(struct cif_isp10_pipeline *p) 
{
	int ret;
	
	ret = isp_pipeline_s_power(p, 0);
	
	return ret == -ENXIO ? 0 : ret;
}

int  isp_pipeline_set_stream(struct cif_isp10_pipeline *p, bool on) 
{
	static const u8 seq[2][IDX_MAX] = {
			{ IDX_ISP, IDX_MIPIPHY, IDX_SENSOR },
			{ IDX_MIPIPHY, IDX_SENSOR, IDX_ISP },
	};
	int i, ret = 0;

	if (p->subdevs[IDX_SENSOR] == NULL)
			return -ENODEV;

	for (i = 0; i < IDX_MAX; i++) {
			unsigned int idx = seq[on][i];

			ret = v4l2_subdev_call(p->subdevs[idx], video, s_stream, on);

			if (ret < 0 && ret != -ENOIOCTLCMD && ret != -ENODEV)
					goto error;
	}
	return 0;
error:
	for (; i >= 0; i--) {
			unsigned int idx = seq[on][i];
			v4l2_subdev_call(p->subdevs[idx], video, s_stream, !on);
	}
	return ret;
}

static int cif_isp10_create_links(struct cif_isp10_device *dev)
{
	int ret;
	unsigned int flags = 0;
	struct media_entity *source, *sink;
	struct cif_isp10_v4l2_device *cif_isp10_v4l2_dev =
		(struct cif_isp10_v4l2_device *)(dev->nodes);

	//note: entities source pad link should be established prior to sink pad link.
	//isp sink links shoud be established first.
	flags = MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE;

	//TODO: create sensor->mipiphy links
	source = &dev->subdevs[CIF_ISP10_SD_SENSOR]->entity;
	sink = &dev->subdevs[CIF_ISP10_SD_PHY_CSI]->entity;
	//ret = media_entity_create_link(source, 0, //TODO(zsq) Find the source pad
	//			       sink, MIPIPHY_PAD_SINK, flags);
	ret = media_create_pad_link(source, 0, //TODO(zsq) Find the source pad
				       sink, MIPIPHY_PAD_SINK, flags);

	if (ret)
		return ret;
	//TODO: create sensor->isp links
	//TODO: create mipiphy->isp links
	source = &dev->subdevs[CIF_ISP10_SD_PHY_CSI]->entity;
	sink = &dev->isp_sub_dev.entity;
	//ret = media_entity_create_link(source, MIPIPHY_PAD_SOURCE,
	//			       sink, CIF_ISP10_ISP_PAD_SINK_MIPI, flags);
	ret = media_create_pad_link(source, MIPIPHY_PAD_SOURCE,
				       sink, CIF_ISP10_ISP_PAD_SINK_MIPI, flags);
	if (ret)
		return ret;
	//create isp internal links

	//SP links
	source = &dev->isp_sub_dev.entity ;
	sink = &dev->sp_strm_subdev.subdev.entity;
	//ret = media_entity_create_link(source, CIF_ISP10_ISP_PAD_SOURCE_SP,
	//			       sink, 0, flags);
	ret = media_create_pad_link(source, CIF_ISP10_ISP_PAD_SOURCE_SP,
				       sink, 0, flags);
	if (ret)
		return ret;

	source = &dev->sp_strm_subdev.subdev.entity;
	sink = &cif_isp10_v4l2_dev->node[SP_DEV].vdev.entity;;
	//ret = media_entity_create_link(source, 1, sink, 0, flags);
	ret = media_create_pad_link(source, 1, sink, 0, flags);
	if (ret)
		return ret;

	//MP links
	source = &dev->isp_sub_dev.entity ;
	sink = &dev->mp_strm_subdev.subdev.entity;
	//ret = media_entity_create_link(source, CIF_ISP10_ISP_PAD_SOURCE_MP,
	//			       sink, 0, flags);
	ret = media_create_pad_link(source, CIF_ISP10_ISP_PAD_SOURCE_MP,
				       sink, 0, flags);
	if (ret)
		return ret;

	source = &dev->mp_strm_subdev.subdev.entity;
	sink = &cif_isp10_v4l2_dev->node[MP_DEV].vdev.entity;;
	//ret = media_entity_create_link(source, 1, sink, 0, flags);
	ret = media_create_pad_link(source, 1, sink, 0, flags);
	if (ret)
		return ret;

	//3A stats links
	source = &dev->isp_sub_dev.entity ;
	sink = &cif_isp10_v4l2_dev->node[ISP_DEV].vdev.entity;;
	//ret = media_entity_create_link(source, CIF_ISP10_ISP_PAD_SOURCE_STATS,
	//			       sink, 0, flags);
	ret = media_create_pad_link(source, CIF_ISP10_ISP_PAD_SOURCE_STATS,
				       sink, 0, flags);
	return ret;
}

static int subdev_notifier_complete(struct v4l2_async_notifier *notifier)
{
	struct cif_isp10_device *dev;
	int ret;

	dev = container_of(notifier, struct cif_isp10_device, notifier);

	mutex_lock(&dev->media_dev.graph_mutex);
	// create links first ?
	ret = cif_isp10_create_links(dev);
	if (ret < 0)
		goto unlock;
	ret = v4l2_device_register_subdev_nodes(&dev->v4l2_dev);
	if (ret)
		goto unlock;

	v4l2_info(&dev->v4l2_dev, "Async subdev notifier completed\n");

unlock:
	mutex_unlock(&dev->media_dev.graph_mutex);
	//TODO(zsq): free links

	return ret;
}

static int subdev_notifier_bound(struct v4l2_async_notifier *notifier,
				 struct v4l2_subdev *subdev,
				 struct v4l2_async_subdev *asd)
{
	struct cif_isp10_device *isp_dev;

	isp_dev = container_of(asd, struct cif_isp10_device, asd);

	/* TODO: Assuming remote sensor is a media entity */
	if (isp_dev->subdevs[CIF_ISP10_SD_SENSOR]) {
		v4l2_err(subdev, "Only one sensor is supported now\n");
		return -EINVAL;
	}
	isp_dev->subdevs[CIF_ISP10_SD_SENSOR] = subdev;
	isp_dev->num_sensors ++;
	v4l2_info(subdev, "Async registered subdev\n");

	return 0;
}

static int isp_subdev_notifier(struct cif_isp10_device *isp_dev,
			       struct device_node *parent)
{
	struct v4l2_async_notifier *notifier = &isp_dev->notifier;
	struct v4l2_async_subdev *asd = &isp_dev->asd;
	struct device *dev = isp_dev->dev;
	struct device_node *node = NULL;

	//TODO use v4l2_of_parse_endpoint()
	node = of_graph_get_next_endpoint(parent, NULL);
	if (!node) {
		dev_err(dev, "Can't find the sensor endpoint\n");
		return -EINVAL;
	}

	/* Now only one external subdev supported */
	notifier->subdevs = devm_kzalloc(dev, sizeof(*notifier->subdevs),
					 GFP_KERNEL);
	if (!notifier->subdevs)
		return -ENOMEM;

	asd->match_type = V4L2_ASYNC_MATCH_OF;
	asd->match.of.node = of_graph_get_remote_port_parent(node);
	of_node_put(node);
	if (!asd->match.of.node) {
		dev_err(dev, "Can't get remote endpoint parent\n");
		return -EINVAL;
	}
	notifier->subdevs[0] = asd;
	notifier->num_subdevs = 1;
	notifier->bound = subdev_notifier_bound;
	notifier->complete = subdev_notifier_complete;

	return v4l2_async_notifier_register(&isp_dev->v4l2_dev, notifier);
}

//TODO(zsq): for rk3288 isp, two mipiphy can switch on-the-fly
static int register_mipiphy_subdev(struct cif_isp10_device *isp_dev)
{
	struct media_entity *me;
	struct v4l2_subdev *sd;
	struct device *dev = isp_dev->dev;
	struct platform_device *mipi_pdev;
	struct device_node *of_mipi;
	int ret;

	ret = of_platform_populate(dev->of_node, NULL, NULL, dev);
	if (ret) {
		dev_err(dev, "Failed to populate child mipiphy(%d)\n", ret);
		return ret;
	}

	of_mipi = of_get_next_available_child(dev->of_node, NULL);
	if (!of_mipi) {
		dev_err(dev, "Mipiphy as child node is expected\n");
		return -EINVAL;
	}
	mipi_pdev = of_find_device_by_node(of_mipi);
	if (!mipi_pdev) {
		dev_err(dev, "Defer for mipiphy device is not ready\n");
		ret = -EPROBE_DEFER;
		goto of_put;
	}

	me = platform_get_drvdata(mipi_pdev);
	BUG_ON(!me);
	sd = media_entity_to_v4l2_subdev(me);
	ret = v4l2_device_register_subdev(&isp_dev->v4l2_dev, sd);
	if (ret) {
		v4l2_err(&isp_dev->v4l2_dev, "Failed to register phy subdev\n");
		goto of_put;
	}
	isp_dev->subdevs[CIF_ISP10_SD_PHY_CSI] = sd;

	ret = isp_subdev_notifier(isp_dev, of_mipi);
	if (ret) {
		v4l2_err(sd, "Failed to register subdev notifier(%d)\n", ret);
		goto unregister_sd;
	}
	of_node_put(of_mipi);

	return 0;

unregister_sd:
	v4l2_device_unregister_subdev(sd);
of_put:
	of_node_put(of_mipi);

	isp_dev->subdevs[CIF_ISP10_SD_PHY_CSI] = NULL;

	return ret;
}

static inline struct cif_isp10_device *isp_to_cif_isp10_device(
	struct cif_isp10_isp_dev *isp_dev)
{

	struct cif_isp10_device *dev =
		container_of(isp_dev, struct cif_isp10_device, isp_dev);
	return dev;
}

static int register_stream_subdevs(struct cif_isp10_device *dev)
{
	int ret;
	struct cif_isp10_v4l2_device* cif_isp10_v4l2_dev =
		(struct cif_isp10_v4l2_device*)dev->nodes;
	ret = register_stream_subdev(&dev->mp_stream, &dev->v4l2_dev);
	if (ret)
		return ret;
	ret = cif_isp10_v4l2_register_video_device(
		dev,
		&cif_isp10_v4l2_dev->node[MP_DEV].vdev,
		MP_VDEV_NAME,
		V4L2_CAP_VIDEO_CAPTURE,
		CIF_ISP10_V4L2_MP_DEV_MAJOR,
		&cif_isp10_v4l2_fops,
		&cif_isp10_v4l2_mp_ioctlops);
	if (ret)
		goto e_r_mp_subdev;
	
	ret = register_stream_subdev(&dev->sp_stream, &dev->v4l2_dev);
	if (ret)
		goto e_r_mp_vdev;
	ret = cif_isp10_v4l2_register_video_device(
		dev,
		&cif_isp10_v4l2_dev->node[SP_DEV].vdev,
		SP_VDEV_NAME,
		V4L2_CAP_VIDEO_CAPTURE,
		CIF_ISP10_V4L2_SP_DEV_MAJOR,
		&cif_isp10_v4l2_fops,
		&cif_isp10_v4l2_sp_ioctlops);
	if (ret)
		goto e_r_sp_subdev;
	
	ret = cif_isp10_v4l2_register_video_device(
		dev,
		&cif_isp10_v4l2_dev->node[DMA_DEV].vdev,
		DMA_VDEV_NAME,
		V4L2_CAP_VIDEO_OUTPUT,
		CIF_ISP10_V4L2_DMA_DEV_MAJOR,
		&cif_isp10_v4l2_fops,
		&cif_isp10_v4l2_dma_ioctlops);
	if (ret)
		goto e_r_sp_vdev;

	return 0;

e_r_sp_vdev:
	media_entity_cleanup(&cif_isp10_v4l2_dev->node[SP_DEV].vdev.entity);
	video_unregister_device(&cif_isp10_v4l2_dev->node[SP_DEV].vdev);
e_r_sp_subdev:
	unregister_stream_subdev(&dev->sp_stream);
e_r_mp_vdev:
	media_entity_cleanup(&cif_isp10_v4l2_dev->node[MP_DEV].vdev.entity);
	video_unregister_device(&cif_isp10_v4l2_dev->node[MP_DEV].vdev);
e_r_mp_subdev:
	unregister_stream_subdev(&dev->mp_stream);
	return ret;
}

static void unregister_stream_subdevs(struct cif_isp10_device *dev)
{
	struct cif_isp10_v4l2_device* cif_isp10_v4l2_dev =
		(struct cif_isp10_v4l2_device*)dev->nodes;
	unregister_stream_subdev(&dev->mp_stream);
	media_entity_cleanup(&cif_isp10_v4l2_dev->node[MP_DEV].vdev.entity);
	video_unregister_device(&cif_isp10_v4l2_dev->node[MP_DEV].vdev);
	unregister_stream_subdev(&dev->sp_stream);
	media_entity_cleanup(&cif_isp10_v4l2_dev->node[SP_DEV].vdev.entity);
	video_unregister_device(&cif_isp10_v4l2_dev->node[SP_DEV].vdev);
	unregister_stream_subdev(&dev->dma_stream);
	media_entity_cleanup(&cif_isp10_v4l2_dev->node[DMA_DEV].vdev.entity);
	video_unregister_device(&cif_isp10_v4l2_dev->node[DMA_DEV].vdev);
}


static int register_platform_subdevs(struct cif_isp10_device *dev)
{
	int ret;
	struct cif_isp10_isp_dev *isp_dev = &dev->isp_dev;
	struct cif_isp10_v4l2_device* cif_isp10_v4l2_dev =
		(struct cif_isp10_v4l2_device*)dev->nodes;
	//register isp v4l2 subdev
	ret = register_cifisp_isp_subdev(dev, &dev->v4l2_dev);
	if (ret)
		goto err;
	ret = register_stream_subdevs(dev);
	if (ret)
		goto err_isp_subdev;
	// register  ISP(3A stats)dev
	ret = register_cifisp_device(isp_dev,
		&cif_isp10_v4l2_dev->node[ISP_DEV].vdev,
		&dev->v4l2_dev,
		dev->config.base_addr);
	if (ret)
		goto err_stream_subdevs;
	// register mipi dev
	ret = register_mipiphy_subdev(dev);
	if (ret)
		goto err_isp_device;
	return 0;
err_isp_device:
	unregister_cifisp_device(&cif_isp10_v4l2_dev->node[ISP_DEV].vdev);
err_stream_subdevs:
	unregister_stream_subdevs(dev);
err_isp_subdev:
	unregister_cifisp_isp_subdev(dev);
err:
	return ret;
}


static void unregister_sensor_subdevs(struct cif_isp10_device *dev)
{
//TODO: subdev and entity
}

static void unregister_mipiphy_subdev(struct cif_isp10_device *dev)
{
	//TODO
}

static void unregister_platform_subdevs(struct cif_isp10_device *dev)
{
	struct cif_isp10_v4l2_device* cif_isp10_v4l2_dev =
		(struct cif_isp10_v4l2_device*)dev->nodes;
	unregister_mipiphy_subdev(dev);
	unregister_cifisp_device(&cif_isp10_v4l2_dev->node[ISP_DEV].vdev);
	unregister_stream_subdevs(dev);
	unregister_cifisp_isp_subdev(dev);
}

static int cif_isp10_v4l2_md_link_notify(struct media_link *link, unsigned int flags,
                                unsigned int notification)
{
	//TODO
	return 0;
}

static int cif_isp10_v4l2_drv_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct device_node *node = pdev->dev.of_node;
	struct cif_isp10_device *dev = NULL;
	struct cif_isp10_v4l2_device *cif_isp10_v4l2_dev;
	struct v4l2_device *v4l2_dev;
	int ret;

	cif_isp10_pltfrm_pr_info(NULL, "probing...\n");

	cif_isp10_v4l2_dev = devm_kzalloc(
				&pdev->dev,
				sizeof(struct cif_isp10_v4l2_device),
				GFP_KERNEL);
	if (IS_ERR_OR_NULL(cif_isp10_v4l2_dev)) {
		ret = -ENOMEM;
		goto err;
	}

	match = of_match_node(cif_isp10_v4l2_of_match, node);
	dev = cif_isp10_create(&pdev->dev,
		cif_isp10_v4l2_event,
		cif_isp10_v4l2_requeue_bufs);
	if (IS_ERR_OR_NULL(dev)) {
		ret = -ENODEV;
		goto err;
	}

	dev->dev_id = cif_isp10_v4l2_dev_cnt;
	dev->isp_dev.dev_id = &dev->dev_id;
	dev->nodes = (void *)cif_isp10_v4l2_dev;
	spin_lock_init(&dev->vbq_lock);
	spin_lock_init(&dev->vbreq_lock);
	spin_lock_init(&iowrite32_verify_lock);

	// 1.  init v4l cif_isp10_device->v4l2_dev->mdev, notify
	//      init v4l cif_isp10_device->media_dev
	strlcpy(dev->media_dev.model, "ROCKCHIP ISP",
			sizeof(dev->media_dev.model));
	//dev->media_dev.link_notify = cif_isp10_v4l2_md_link_notify;
	//dev->media_dev.ops->link_notify = cif_isp10_v4l2_md_link_notify;
	dev->media_dev.dev = &pdev->dev;
	
	v4l2_dev = &dev->v4l2_dev;
	v4l2_dev->mdev = &dev->media_dev;
	//TODO: set v4l2 dev notify, maybe not neccesary.
	v4l2_dev->notify = NULL;
	strlcpy(v4l2_dev->name, "rk-isp10", sizeof(v4l2_dev->name));
	
	// 2.  register v4l2 dev, register media dev
	ret = v4l2_device_register(dev->dev, &dev->v4l2_dev);
	if (IS_ERR_VALUE(ret)) {
		cif_isp10_pltfrm_pr_err(NULL,
			"V4L2 device registration failed\n");
		goto err;
	}
	
	ret = media_device_register(&dev->media_dev);
	if (ret < 0) {
			v4l2_err(v4l2_dev, "Failed to register media device: %d\n", ret);
			goto err_v4l2_dev;
	}

	dev->num_sensors = 0;

	// 3.  create & register platefom subdev (from of_node)
	ret = register_platform_subdevs(dev);
	if (ret)
		goto err_mdev;

	pm_runtime_enable(&pdev->dev);
	cif_isp10_v4l2_dev_cnt++;

	return 0;

err_mdev:
	media_device_unregister(&dev->media_dev);
err_v4l2_dev:
	v4l2_device_unregister(&dev->v4l2_dev);
err:
	cif_isp10_destroy(dev);
	return ret;
}


/* ======================================================================== */

static int cif_isp10_v4l2_drv_remove(struct platform_device *pdev)
{
	struct cif_isp10_device *cif_isp10_dev =
		(struct cif_isp10_device *)platform_get_drvdata(pdev);
	struct cif_isp10_v4l2_device *cif_isp10_v4l2_dev =
		(struct cif_isp10_v4l2_device *)cif_isp10_dev->nodes;

	if (IS_ERR_VALUE(cif_isp10_release(cif_isp10_dev,
		CIF_ISP10_ALL_STREAMS)))
		cif_isp10_pltfrm_pr_warn(cif_isp10_dev->dev,
			"CIF power off failed\n");
	
	media_entity_cleanup(&cif_isp10_v4l2_dev->node[SP_DEV].vdev.entity);
	video_unregister_device(&cif_isp10_v4l2_dev->node[SP_DEV].vdev);
	media_entity_cleanup(&cif_isp10_v4l2_dev->node[MP_DEV].vdev.entity);
	video_unregister_device(&cif_isp10_v4l2_dev->node[MP_DEV].vdev);
	media_entity_cleanup(&cif_isp10_v4l2_dev->node[DMA_DEV].vdev.entity);
	video_unregister_device(&cif_isp10_v4l2_dev->node[DMA_DEV].vdev);
	unregister_sensor_subdevs(cif_isp10_dev);
	unregister_platform_subdevs(cif_isp10_dev);
	//unregister_cifisp_device(&cif_isp10_v4l2_dev->node[ISP_DEV].vdev);
	media_device_unregister(&cif_isp10_dev->media_dev);
	v4l2_device_unregister(&cif_isp10_dev->v4l2_dev);
	cif_isp10_destroy(cif_isp10_dev);

	cif_isp10_v4l2_dev_cnt--;
	return 0;
}

static int cif_isp10_v4l2_drv_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	int ret = 0;
	struct cif_isp10_device *cif_isp10_dev =
		(struct cif_isp10_device *)platform_get_drvdata(pdev);

	cif_isp10_pltfrm_pr_dbg(cif_isp10_dev->dev, "\n");

	ret = cif_isp10_suspend(cif_isp10_dev);
	if (IS_ERR_VALUE(ret))
		goto err;

	//TODO
	isp_pipeline_suspend(&cif_isp10_dev->pipe);

	cif_isp10_pltfrm_pinctrl_set_state(&pdev->dev,
		CIF_ISP10_PINCTRL_STATE_SLEEP);

	return 0;
err:
	cif_isp10_pltfrm_pr_err(cif_isp10_dev->dev,
		"failed with error %d\n", ret);
	return ret;
}

static int cif_isp10_v4l2_drv_resume(struct platform_device *pdev)
{
	int ret = 0;
	struct cif_isp10_device *cif_isp10_dev =
		(struct cif_isp10_device *)platform_get_drvdata(pdev);

	cif_isp10_pltfrm_pr_dbg(cif_isp10_dev->dev, "\n");

	if (cif_isp10_dev->num_sensors == 0) {
		cif_isp10_pltfrm_pr_err(
			cif_isp10_dev->dev,
			"cif_isp10_dev img_src is null!\n");
		goto err;
	}

	ret = cif_isp10_resume(cif_isp10_dev);
	if (IS_ERR_VALUE(ret))
		goto err;

	//TODO
	isp_pipeline_resume(&cif_isp10_dev->pipe);

	cif_isp10_pltfrm_pinctrl_set_state(&pdev->dev,
		CIF_ISP10_PINCTRL_STATE_DEFAULT);

	return 0;
err:
	cif_isp10_pltfrm_pr_err(cif_isp10_dev->dev,
		"failed with error %d\n", ret);
	return ret;
}

static int cif_isp10_runtime_suspend(struct device *dev)
{
	cif_isp10_pltfrm_pr_dbg(dev, "\n");
	return cif_isp10_pltfrm_pm_set_state(dev, CIF_ISP10_PM_STATE_SUSPENDED);
}

static int cif_isp10_runtime_resume(struct device *dev)
{
	cif_isp10_pltfrm_pr_dbg(dev, "\n");
	return cif_isp10_pltfrm_pm_set_state(dev, CIF_ISP10_PM_STATE_SW_STNDBY);
}

static const struct dev_pm_ops cif_isp10_dev_pm_ops = {
	SET_RUNTIME_PM_OPS(cif_isp10_runtime_suspend,
			   cif_isp10_runtime_resume, NULL)
};

static struct platform_driver cif_isp10_v4l2_plat_drv = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(cif_isp10_v4l2_of_match),
		.pm = &cif_isp10_dev_pm_ops,
		   },
	.probe = cif_isp10_v4l2_drv_probe,
	.remove = cif_isp10_v4l2_drv_remove,
	.suspend = cif_isp10_v4l2_drv_suspend,
	.resume = cif_isp10_v4l2_drv_resume,
};

/* ======================================================================== */
static int cif_isp10_v4l2_init(void)
{
	int ret;

	ret = platform_driver_register(&cif_isp10_v4l2_plat_drv);
	if (ret) {
		cif_isp10_pltfrm_pr_err(NULL,
			"cannot register platform driver, failed with %d\n",
			ret);
		return -ENODEV;
	}

	return ret;
}

/* ======================================================================== */
static void __exit cif_isp10_v4l2_exit(void)
{
	platform_driver_unregister(&cif_isp10_v4l2_plat_drv);
}

device_initcall_sync(cif_isp10_v4l2_init);
module_exit(cif_isp10_v4l2_exit);

MODULE_DESCRIPTION("V4L2 interface for CIF ISP10 driver");
MODULE_AUTHOR("George");
MODULE_LICENSE("GPL");
