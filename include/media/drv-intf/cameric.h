/*
 * Cameric interface driver header
 *
 * Copyright (C) 2017
 * Eddie Cai <eddie.cai@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef CAMERIC_H_
#define CAMERIC_H_

#include <media/media-entity.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-mediabus.h>

/*
 * Enumeration of data inputs to the camera subsystem.
 */
enum cameric_input {
	CAMERIC_INPUT_PARALLEL_0	= 1,
	CAMERIC_INPUT_PARALLEL_1,
	CAMERIC_INPUT_MIPI_CSI2_0	= 3,
	CAMERIC_INPUT_MIPI_CSI2_1,
	CAMERIC_INPUT_WRITEBACK_A	= 5,
	CAMERIC_INPUT_WRITEBACK_B,
	CAMERIC_INPUT_WRITEBACK_ISP = 5,
};

/*
 * Enumeration of the cameric data bus types.
 */
enum cameric_bus_type {
	/* Camera parallel bus */
	CAMERIC_BUS_TYPE_ITU_601 = 1,
	/* Camera parallel bus with embedded synchronization */
	CAMERIC_BUS_TYPE_ITU_656,
	/* Camera MIPI-CSI2 serial bus */
	CAMERIC_BUS_TYPE_MIPI_CSI2,
	/* FIFO link from LCD controller (WriteBack A) */
	CAMERIC_BUS_TYPE_LCD_WRITEBACK_A,
	/* FIFO link from LCD controller (WriteBack B) */
	CAMERIC_BUS_TYPE_LCD_WRITEBACK_B,
	/* FIFO link from cameric-IS */
	CAMERIC_BUS_TYPE_ISP_WRITEBACK = CAMERIC_BUS_TYPE_LCD_WRITEBACK_B,
};

#define cameric_input_is_parallel(x) ((x) == 1 || (x) == 2)
#define cameric_input_is_mipi_csi(x) ((x) == 3 || (x) == 4)

/*
 * The subdevices' group IDs.
 */
#define GRP_ID_SENSOR		(1 << 8)
#define GRP_ID_CAMERIC_IS_SENSOR	(1 << 9)
#define GRP_ID_WRITEBACK	(1 << 10)
#define GRP_ID_CSIS		(1 << 11)
#define GRP_ID_CAMERIC		(1 << 12)
#define GRP_ID_FLITE		(1 << 13)
#define GRP_ID_CAMERIC_IS		(1 << 14)

/**
 * struct cameric_source_info - video source description required for the host
 *			     interface configuration
 *
 * @cameric_bus_type: cameric camera input type
 * @sensor_bus_type: image sensor bus type, MIPI, ITU-R BT.601 etc.
 * @flags: the parallel sensor bus flags defining signals polarity (V4L2_MBUS_*)
 * @mux_id: cameric camera interface multiplexer index (separate for MIPI and ITU)
 */
struct cameric_source_info {
	enum cameric_bus_type cameric_bus_type;
	enum cameric_bus_type sensor_bus_type;
	u16 flags;
	u16 mux_id;
};

/*
 * v4l2_device notification id. This is only for internal use in the kernel.
 * Sensor subdevs should issue S5P_cameric_TX_END_NOTIFY notification in single
 * frame capture mode when there is only one VSYNC pulse issued by the sensor
 * at begining of the frame transmission.
 */
#define S5P_cameric_TX_END_NOTIFY _IO('e', 0)

#define cameric_MAX_PLANES	3

/**
 * struct cameric_fmt - color format data structure
 * @mbus_code: media bus pixel code, -1 if not applicable
 * @name: format description
 * @fourcc: fourcc code for this format, 0 if not applicable
 * @color: the driver's private color format id
 * @memplanes: number of physically non-contiguous data planes
 * @colplanes: number of physically contiguous data planes
 * @colorspace: v4l2 colorspace (V4L2_COLORSPACE_*)
 * @depth: per plane driver's private 'number of bits per pixel'
 * @mdataplanes: bitmask indicating meta data plane(s), (1 << plane_no)
 * @flags: flags indicating which operation mode format applies to
 */
struct cameric_fmt {
	u32 mbus_code;
	char	*name;
	u32	fourcc;
	u32	color;
	u16	memplanes;
	u16	colplanes;
	u8	colorspace;
	u8	depth[cameric_MAX_PLANES];
	u16	mdataplanes;
	u16	flags;
#define FMT_FLAGS_CAM		(1 << 0)
#define FMT_FLAGS_M2M_IN	(1 << 1)
#define FMT_FLAGS_M2M_OUT	(1 << 2)
#define FMT_FLAGS_M2M		(1 << 1 | 1 << 2)
#define FMT_HAS_ALPHA		(1 << 3)
#define FMT_FLAGS_COMPRESSED	(1 << 4)
#define FMT_FLAGS_WRITEBACK	(1 << 5)
#define FMT_FLAGS_RAW_BAYER	(1 << 6)
#define FMT_FLAGS_YUV		(1 << 7)
};

struct cameric_media_pipeline;

/*
 * Media pipeline operations to be called from within a video node,  i.e. the
 * last entity within the pipeline. Implemented by related media device driver.
 */
struct cameric_media_pipeline_ops {
	int (*prepare)(struct cameric_media_pipeline *p,
						struct media_entity *me);
	int (*unprepare)(struct cameric_media_pipeline *p);
	int (*open)(struct cameric_media_pipeline *p, struct media_entity *me,
							bool resume);
	int (*close)(struct cameric_media_pipeline *p);
	int (*set_stream)(struct cameric_media_pipeline *p, bool state);
};

struct cameric_video_entity {
	struct video_device vdev;
	struct cameric_media_pipeline *pipe;
};

struct cameric_media_pipeline {
	struct media_pipeline mp;
	const struct cameric_media_pipeline_ops *ops;
};

static inline struct cameric_video_entity *vdev_to_cameric_video_entity(
					struct video_device *vdev)
{
	return container_of(vdev, struct cameric_video_entity, vdev);
}

#define cameric_pipeline_call(ent, op, args...)				  \
	(!(ent) ? -ENOENT : (((ent)->pipe->ops && (ent)->pipe->ops->op) ? \
	(ent)->pipe->ops->op(((ent)->pipe), ##args) : -ENOIOCTLCMD))	  \

#endif /* S5P_cameric_H_ */
