/*
 * Copyright (C) 2011 - 2012 Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef cameric_MDEVICE_H_
#define cameric_MDEVICE_H_

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/drv-intf/cameric.h>

#include "cameric-core.h"
#include "mipi-csis.h"

#define CAMERIC_OF_NODE_NAME	"cameric"
#define CAMERIC_LITE_OF_NODE_NAME	"cameric-lite"
#define CAMERIC_IS_OF_NODE_NAME	"cameric-is"
#define CSIS_OF_NODE_NAME	"csis"

#define PINCTRL_STATE_IDLE	"idle"

#define CAMERIC_MAX_SENSORS		4
#define CAMERIC_MAX_CAMCLKS		2
#define DEFAULT_SENSOR_CLK_FREQ	24000000U

/* LCD/ISP Writeback clocks (PIXELASYNCMx) */
enum {
	CLK_IDX_WB_A,
	CLK_IDX_WB_B,
	CAMERIC_MAX_WBCLKS
};

enum cameric_subdev_index {
	IDX_SENSOR,
	IDX_CSIS,
	//IDX_FLITE,
	IDX_IS_ISP,
	IDX_CAMERIC,
	IDX_MAX,
};

/*
 * This structure represents a chain of media entities, including a data
 * source entity (e.g. an image sensor subdevice), a data capture entity
 * - a video capture device node and any remaining entities.
 */
struct cameric_pipeline {
	struct cameric_media_pipeline ep;
	struct list_head list;
	struct media_entity *vdev_entity;
	struct v4l2_subdev *subdevs[IDX_MAX];
};

#define to_cameric_pipeline(_ep) container_of(_ep, struct cameric_pipeline, ep)

struct cameric_csis_info {
	struct v4l2_subdev *sd;
	int id;
};

struct cameric_camclk_info {
	struct clk *clock;
	int use_count;
	unsigned long frequency;
};

/**
 * struct cameric_sensor_info - image data source subdev information
 * @pdata: sensor's atrributes passed as media device's platform data
 * @asd: asynchronous subdev registration data structure
 * @subdev: image sensor v4l2 subdev
 * @host: cameric device the sensor is currently linked to
 *
 * This data structure applies to image sensor and the writeback subdevs.
 */
struct cameric_sensor_info {
	struct cameric_source_info pdata;
	struct v4l2_async_subdev asd;
	struct v4l2_subdev *subdev;
	struct cameric_dev *host;
};

struct cam_clk {
	struct clk_hw hw;
	struct cameric_md *cmd;
};
#define to_cam_clk(_hw) container_of(_hw, struct cam_clk, hw)

/**
 * struct cameric_md - cameric media device information
 * @csis: MIPI CSIS subdevs data
 * @sensor: array of registered sensor subdevs
 * @num_sensors: actual number of registered sensors
 * @camclk: external sensor clock information
 * @cameric: array of registered cameric devices
 * @cameric_is: cameric-is data structure
 * @use_isp: set to true when cameric-IS subsystem is used
 * @pmf: handle to the CAMCLK clock control cameric helper device
 * @media_dev: top level media device
 * @v4l2_dev: top level v4l2_device holding up the subdevs
 * @pdev: platform device this media device is hooked up into
 * @pinctrl: camera port pinctrl handle
 * @state_default: pinctrl default state handle
 * @state_idle: pinctrl idle state handle
 * @cam_clk_provider: CAMCLK clock provider structure
 * @user_subdev_api: true if subdevs are not configured by the host driver
 * @slock: spinlock protecting @sensor array
 */
struct cameric_md {
	struct cameric_csis_info csis[CSIS_MAX_ENTITIES];
	struct cameric_sensor_info sensor[CAMERIC_MAX_SENSORS];
	int num_sensors;
	struct cameric_camclk_info camclk[CAMERIC_MAX_CAMCLKS];
	struct clk *wbclk[CAMERIC_MAX_WBCLKS];
	//struct cameric_lite *cameric_lite[CAMERIC_LITE_MAX_DEVS];
	struct cameric_dev *cameric[CAMERIC_MAX_DEVS];
	struct cameric_is *cameric_is;
	bool use_isp;
	struct device *pmf;
	struct media_device media_dev;
	struct v4l2_device v4l2_dev;
	struct platform_device *pdev;
	int irq;
	struct list_head csi0_configs;
	struct list_head csi1_configs;
	s32 exp_time;
	u16 gain;
	struct cameric_pinctrl {
		struct pinctrl *pinctrl;
		struct pinctrl_state *state_default;
		struct pinctrl_state *state_idle;
		struct pinctrl_state *pins_sleep;
		struct pinctrl_state *pins_inactive;
	} pinctl;

	struct cam_clk_provider {
		struct clk *clks[CAMERIC_MAX_CAMCLKS];
		struct clk_onecell_data clk_data;
		struct device_node *of_node;
		struct cam_clk camclk[CAMERIC_MAX_CAMCLKS];
		int num_clocks;
	} clk_provider;

	struct v4l2_async_notifier subdev_notifier;
	struct v4l2_async_subdev *async_subdevs[CAMERIC_MAX_SENSORS];

	bool user_subdev_api;
	spinlock_t slock;
	struct list_head pipelines;
	struct media_graph link_setup_graph;
};

static inline
struct cameric_sensor_info *source_to_sensor_info(struct cameric_source_info *si)
{
	return container_of(si, struct cameric_sensor_info, pdata);
}

static inline struct cameric_md *entity_to_cameric_mdev(struct media_entity *me)
{
	return me->graph_obj.mdev == NULL ? NULL :
		container_of(me->graph_obj.mdev, struct cameric_md, media_dev);
}

static inline struct cameric_md *notifier_to_cameric_md(struct v4l2_async_notifier *n)
{
	return container_of(n, struct cameric_md, subdev_notifier);
}

static inline void cameric_md_graph_lock(struct cameric_video_entity *ve)
{
	mutex_lock(&ve->vdev.entity.graph_obj.mdev->graph_mutex);
}

static inline void cameric_md_graph_unlock(struct cameric_video_entity *ve)
{
	mutex_unlock(&ve->vdev.entity.graph_obj.mdev->graph_mutex);
}

int cameric_md_set_camclk(struct v4l2_subdev *sd, bool on);

#ifdef CONFIG_OF
static inline bool cameric_md_is_isp_available(struct device_node *node)
{
	node = of_get_child_by_name(node, CAMERIC_IS_OF_NODE_NAME);
	return node ? of_device_is_available(node) : false;
}
#else
#define cameric_md_is_isp_available(node) (false)
#endif /* CONFIG_OF */

static inline struct v4l2_subdev *__cameric_md_get_subdev(
				struct cameric_media_pipeline *ep,
				unsigned int index)
{
	struct cameric_pipeline *p = to_cameric_pipeline(ep);

	if (!p || index >= IDX_MAX)
		return NULL;
	else
		return p->subdevs[index];
}

#endif
