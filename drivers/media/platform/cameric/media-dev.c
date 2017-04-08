/*
 * cameric host interface media device driver
 *
 * Copyright (C) 2017 Eddie Cai <eddie.cai.linux@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, either version 2 of the License,
 * or (at your option) any later version.
 */

#include <linux/bug.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-of.h>
#include <media/media-device.h>
#include <media/drv-intf/cameric.h>

#include "media-dev.h"
#include "cameric-core.h"
#include "cameric-is.h"
//#include "cameric-lite.h"
//#include "mipi-csis.h"

/* Set up image sensor subdev -> cameric capture node notifications. */
static void __setup_sensor_notification(struct cameric_md *cmd,
					struct v4l2_subdev *sensor,
					struct v4l2_subdev *cameric_sd)
{
	struct cameric_source_info *src_inf;
	struct cameric_sensor_info *md_si;
	unsigned long flags;
	printk(KERN_INFO "%s \n", __func__);
	src_inf = v4l2_get_subdev_hostdata(sensor);
	if (!src_inf || WARN_ON(cmd == NULL))
		return;

	md_si = source_to_sensor_info(src_inf);
	spin_lock_irqsave(&cmd->slock, flags);
	md_si->host = v4l2_get_subdevdata(cameric_sd);
	spin_unlock_irqrestore(&cmd->slock, flags);
}

/**
 * cameric_pipeline_prepare - update pipeline information with subdevice pointers
 * @me: media entity terminating the pipeline
 *
 * Caller holds the graph mutex.
 */
static void cameric_pipeline_prepare(struct cameric_pipeline *p,
					struct media_entity *me)
{
	struct cameric_md *cmd = entity_to_cameric_mdev(me);
	struct v4l2_subdev *sd;
	struct v4l2_subdev *sensor = NULL;
	int i;
	printk(KERN_INFO "%s \n", __func__);
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

		if (!pad || !is_media_entity_v4l2_subdev(pad->entity))
			break;
		sd = media_entity_to_v4l2_subdev(pad->entity);

		switch (sd->grp_id) {
		case GRP_ID_SENSOR:
			sensor = sd;
			/* fall through */
		case GRP_ID_CAMERIC_IS_SENSOR:
			p->subdevs[IDX_SENSOR] = sd;
			break;
		case GRP_ID_CSIS:
			p->subdevs[IDX_CSIS] = sd;
			break;
		//case GRP_ID_FLITE:
			//p->subdevs[IDX_FLITE] = sd;
			//break;
		case GRP_ID_CAMERIC:
			p->subdevs[IDX_CAMERIC] = sd;
			break;
		case GRP_ID_CAMERIC_IS:
			p->subdevs[IDX_IS_ISP] = sd;
			break;
		default:
			break;
		}
		me = &sd->entity;
		if (me->num_pads == 1)
			break;
	}

	if (sensor && p->subdevs[IDX_CAMERIC])
		__setup_sensor_notification(cmd, sensor, p->subdevs[IDX_CAMERIC]);
}

/**
 * __subdev_set_power - change power state of a single subdev
 * @sd: subdevice to change power state for
 * @on: 1 to enable power or 0 to disable
 *
 * Return result of s_power subdev operation or -ENXIO if sd argument
 * is NULL. Return 0 if the subdevice does not implement s_power.
 */
static int __subdev_set_power(struct v4l2_subdev *sd, int on)
{
	int *use_count;
	int ret;
	printk(KERN_INFO "%s \n", __func__);
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

/**
 * cameric_pipeline_s_power - change power state of all pipeline subdevs
 * @cameric: cameric device terminating the pipeline
 * @state: true to power on, false to power off
 *
 * Needs to be called with the graph mutex held.
 */
static int cameric_pipeline_s_power(struct cameric_pipeline *p, bool on)
{
	static const u8 seq[2][IDX_MAX - 1] = {
		{ IDX_IS_ISP, IDX_SENSOR, IDX_CSIS },
		{ IDX_CSIS, IDX_SENSOR, IDX_IS_ISP },
	};
	int i, ret = 0;
	printk(KERN_INFO "%s \n", __func__);
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

/**
 * __cameric_pipeline_enable - enable power of all pipeline subdevs
 *			    and the sensor clock
 * @ep: video pipeline structure
 * @cmd: cameric media device
 *
 * Called with the graph mutex held.
 */
static int __cameric_pipeline_enable(struct cameric_media_pipeline *ep,
				  struct cameric_md *cmd)
{
	struct cameric_pipeline *p = to_cameric_pipeline(ep);
	int ret;
	printk(KERN_INFO "%s \n", __func__);
	/* Enable PXLASYNC clock if this pipeline includes cameric-IS */
	if (!IS_ERR(cmd->wbclk[CLK_IDX_WB_B]) && p->subdevs[IDX_IS_ISP]) {
		ret = clk_prepare_enable(cmd->wbclk[CLK_IDX_WB_B]);
		if (ret < 0)
			return ret;
	}

	ret = cameric_pipeline_s_power(p, 1);
	if (!ret)
		return 0;

	if (!IS_ERR(cmd->wbclk[CLK_IDX_WB_B]) && p->subdevs[IDX_IS_ISP])
		clk_disable_unprepare(cmd->wbclk[CLK_IDX_WB_B]);

	return ret;
}

/**
 * __cameric_pipeline_open - update the pipeline information, enable power
 *                        of all pipeline subdevs and the sensor clock
 * @me: media entity to start graph walk with
 * @prepare: true to walk the current pipeline and acquire all subdevs
 *
 * Called with the graph mutex held.
 */
static int __cameric_pipeline_open(struct cameric_media_pipeline *ep,
				struct media_entity *me, bool prepare)
{
	struct cameric_md *cmd = entity_to_cameric_mdev(me);
	struct cameric_pipeline *p = to_cameric_pipeline(ep);
	struct v4l2_subdev *sd;
	printk(KERN_INFO "%s \n", __func__);
	if (WARN_ON(p == NULL || me == NULL))
		return -EINVAL;

	if (prepare)
		cameric_pipeline_prepare(p, me);

	sd = p->subdevs[IDX_SENSOR];
	if (sd == NULL) {
		pr_warn("%s(): No sensor subdev\n", __func__);
		/*
		 * Pipeline open cannot fail so as to make it possible
		 * for the user space to configure the pipeline.
		 */
		return 0;
	}

	return __cameric_pipeline_enable(ep, cmd);
}

/**
 * __cameric_pipeline_close - disable the sensor clock and pipeline power
 * @cameric: cameric device terminating the pipeline
 *
 * Disable power of all subdevs and turn the external sensor clock off.
 */
static int __cameric_pipeline_close(struct cameric_media_pipeline *ep)
{
	struct cameric_pipeline *p = to_cameric_pipeline(ep);
	struct v4l2_subdev *sd = p ? p->subdevs[IDX_SENSOR] : NULL;
	struct cameric_md *cmd;
	int ret;
	printk(KERN_INFO "%s \n", __func__);
	if (sd == NULL) {
		pr_warn("%s(): No sensor subdev\n", __func__);
		return 0;
	}

	ret = cameric_pipeline_s_power(p, 0);

	cmd = entity_to_cameric_mdev(&sd->entity);

	/* Disable PXLASYNC clock if this pipeline includes cameric-IS */
	if (!IS_ERR(cmd->wbclk[CLK_IDX_WB_B]) && p->subdevs[IDX_IS_ISP])
		clk_disable_unprepare(cmd->wbclk[CLK_IDX_WB_B]);

	return ret == -ENXIO ? 0 : ret;
}

/**
 * __cameric_pipeline_s_stream - call s_stream() on pipeline subdevs
 * @pipeline: video pipeline structure
 * @on: passed as the s_stream() callback argument
 */
static int __cameric_pipeline_s_stream(struct cameric_media_pipeline *ep, bool on)
{
	static const u8 seq[2][IDX_MAX] = {
		{ IDX_CAMERIC, IDX_SENSOR, IDX_IS_ISP, IDX_CSIS },
		{ IDX_CSIS, IDX_CAMERIC, IDX_SENSOR, IDX_IS_ISP },
	};
	struct cameric_pipeline *p = to_cameric_pipeline(ep);
	struct cameric_md *cmd = entity_to_cameric_mdev(&p->subdevs[IDX_CSIS]->entity);
	enum cameric_subdev_index sd_id;
	int i, ret = 0;
	printk(KERN_INFO "%s \n", __func__);
	if (p->subdevs[IDX_SENSOR] == NULL) {
		if (!cmd->user_subdev_api) {
			/*
			 * Sensor must be already discovered if we
			 * aren't in the user_subdev_api mode
			 */
			return -ENODEV;
		}

		/* Get pipeline sink entity */
		if (p->subdevs[IDX_CAMERIC])
			sd_id = IDX_CAMERIC;
		else if (p->subdevs[IDX_IS_ISP])
			sd_id = IDX_IS_ISP;
		//else if (p->subdevs[IDX_FLITE])
			//sd_id = IDX_FLITE;
		else
			return -ENODEV;

		/*
		 * Sensor could have been linked between open and STREAMON -
		 * check if this is the case.
		 */
		cameric_pipeline_prepare(p, &p->subdevs[sd_id]->entity);

		if (p->subdevs[IDX_SENSOR] == NULL)
			return -ENODEV;

		ret = __cameric_pipeline_enable(ep, cmd);
		if (ret < 0)
			return ret;

	}

	for (i = 0; i < IDX_MAX; i++) {
		unsigned int idx = seq[on][i];

		ret = v4l2_subdev_call(p->subdevs[idx], video, s_stream, on);

		if (ret < 0 && ret != -ENOIOCTLCMD && ret != -ENODEV)
			goto error;
	}

	return 0;
error:
	cameric_pipeline_s_power(p, !on);
	for (; i >= 0; i--) {
		unsigned int idx = seq[on][i];
		v4l2_subdev_call(p->subdevs[idx], video, s_stream, !on);
	}
	return ret;
}

/* Media pipeline operations for the cameric/cameric-LITE video device driver */
static const struct cameric_media_pipeline_ops cameric_pipeline_ops = {
	.open		= __cameric_pipeline_open,
	.close		= __cameric_pipeline_close,
	.set_stream	= __cameric_pipeline_s_stream,
};

static struct cameric_media_pipeline *cameric_md_pipeline_create(
						struct cameric_md *cmd)
{
	struct cameric_pipeline *p;
	printk(KERN_INFO "%s \n", __func__);
	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (!p)
		return NULL;

	list_add_tail(&p->list, &cmd->pipelines);

	p->ep.ops = &cameric_pipeline_ops;
	return &p->ep;
}

static void cameric_md_pipelines_free(struct cameric_md *cmd)
{printk(KERN_INFO "%s \n", __func__);
	while (!list_empty(&cmd->pipelines)) {
		struct cameric_pipeline *p;

		p = list_entry(cmd->pipelines.next, typeof(*p), list);
		list_del(&p->list);
		kfree(p);
	}
}

/* Parse port node and register as a sub-device any sensor specified there. */
static int cameric_md_parse_port_node(struct cameric_md *cmd,
				   struct device_node *port,
				   unsigned int index)
{
	struct cameric_source_info *pd = &cmd->sensor[index].pdata;
	struct device_node *rem, *ep, *np;
	struct v4l2_of_endpoint endpoint;
	int ret;
	printk(KERN_INFO "%s \n", __func__);
	/* Assume here a port node can have only one endpoint node. */
	ep = of_get_next_child(port, NULL);
	if (!ep)
		return 0;

	ret = v4l2_of_parse_endpoint(ep, &endpoint);
	if (ret) {
		of_node_put(ep);
		return ret;
	}

	if (WARN_ON(endpoint.base.port == 0) || index >= CAMERIC_MAX_SENSORS) {
		of_node_put(ep);
		return -EINVAL;
	}

	pd->mux_id = (endpoint.base.port - 1) & 0x1;

	rem = of_graph_get_remote_port_parent(ep);
	of_node_put(ep);
	if (rem == NULL) {
		v4l2_info(&cmd->v4l2_dev, "Remote device at %s not found\n",
							ep->full_name);
		return 0;
	}

	if (cameric_input_is_parallel(endpoint.base.port)) {
		if (endpoint.bus_type == V4L2_MBUS_PARALLEL)
			pd->sensor_bus_type = CAMERIC_BUS_TYPE_ITU_601;
		else
			pd->sensor_bus_type = CAMERIC_BUS_TYPE_ITU_656;
		pd->flags = endpoint.bus.parallel.flags;
	} else if (cameric_input_is_mipi_csi(endpoint.base.port)) {
		/*
		 * MIPI CSI-2: only input mux selection and
		 * the sensor's clock frequency is needed.
		 */
		pd->sensor_bus_type = CAMERIC_BUS_TYPE_MIPI_CSI2;
	} else {
		v4l2_err(&cmd->v4l2_dev, "Wrong port id (%u) at node %s\n",
			 endpoint.base.port, rem->full_name);
	}
	/*
	 * For cameric-IS handled sensors, that are placed under i2c-isp device
	 * node, cameric is connected to the cameric-IS through its ISP Writeback
	 * input. Sensors are attached to the cameric-LITE hostdata interface
	 * directly or through MIPI-CSIS, depending on the external media bus
	 * used. This needs to be handled in a more reliable way, not by just
	 * checking parent's node name.
	 */
	np = of_get_parent(rem);

	if (np && !of_node_cmp(np->name, "i2c-isp"))
		pd->cameric_bus_type = CAMERIC_BUS_TYPE_ISP_WRITEBACK;
	else
		pd->cameric_bus_type = pd->sensor_bus_type;

	if (WARN_ON(index >= ARRAY_SIZE(cmd->sensor))) {
		of_node_put(rem);
		return -EINVAL;
	}

	cmd->sensor[index].asd.match_type = V4L2_ASYNC_MATCH_OF;
	cmd->sensor[index].asd.match.of.node = rem;
	cmd->async_subdevs[index] = &cmd->sensor[index].asd;

	cmd->num_sensors++;

	of_node_put(rem);
	return 0;
}


/* Register all SoC external sub-devices */
static int cameric_md_register_sensor_entities(struct cameric_md *cmd)
{
	struct device_node *parent = cmd->pdev->dev.of_node;
	struct device_node *node, *ports;
	int index = 0;
	int ret;
	printk(KERN_INFO "%s 1\n", __func__);
	/*
	 * Runtime resume one of the cameric entities to make sure
	 * the sclk_cam clocks are not globally disabled.
	 */
	if (!cmd->pmf)
		return -ENXIO;
	printk(KERN_INFO "%s 2\n", __func__);
	ret = pm_runtime_get_sync(cmd->pmf);
	if (ret < 0)
		return ret;
	printk(KERN_INFO "%s 3\n", __func__);
	cmd->num_sensors = 0;

	/* Attach sensors linked to MIPI CSI-2 receivers */
	for_each_available_child_of_node(parent, node) {
		struct device_node *port;
		printk(KERN_INFO "%s 4: node name:%s\n", __func__, node->name);
		if (of_node_cmp(node->name, "csis"))
			continue;
		printk(KERN_INFO "%s 5 \n", __func__);
		/* The csis node can have only port subnode. */
		port = of_get_next_child(node, NULL);
		if (!port)
			continue;
		printk(KERN_INFO "%s 6 \n", __func__);
		ret = cameric_md_parse_port_node(cmd, port, index);
		if (ret < 0) {
			of_node_put(node);
			goto rpm_put;
		}
		printk(KERN_INFO "%s 7 \n", __func__);
		index++;
	}
	printk(KERN_INFO "%s 8 \n", __func__);
	/* Attach sensors listed in the parallel-ports node */
	ports = of_get_child_by_name(parent, "parallel-ports");
	if (!ports)
		goto rpm_put;
	printk(KERN_INFO "%s 9 \n", __func__);
	for_each_child_of_node(ports, node) {
		printk(KERN_INFO "%s 10 \n", __func__);
		ret = cameric_md_parse_port_node(cmd, node, index);
		if (ret < 0) {
			of_node_put(node);
			break;
		}
		index++;
	}
	printk(KERN_INFO "%s 11 \n", __func__);
rpm_put:
	pm_runtime_put(cmd->pmf);
	return ret;
}

static int __of_get_csis_id(struct device_node *np)
{
	u32 reg = 0;
	printk(KERN_INFO "%s \n", __func__);
	np = of_get_child_by_name(np, "port");
	if (!np)
		return -EINVAL;
	of_property_read_u32(np, "reg", &reg);
	return reg - CAMERIC_INPUT_MIPI_CSI2_0;
}

static int register_cameric_entity(struct cameric_md *cmd, struct cameric_dev *cameric)
{
	struct v4l2_subdev *sd;
	struct cameric_media_pipeline *ep;
	int ret;
	printk(KERN_INFO "%s 1\n", __func__);
	if (WARN_ON(cameric->id >= CAMERIC_MAX_DEVS || cmd->cameric[cameric->id]))
		return -EBUSY;
	printk(KERN_INFO "%s 2\n", __func__);
	sd = &cameric->vid_cap.subdev;
	sd->grp_id = GRP_ID_CAMERIC;
	printk(KERN_INFO "%s 3, function:%x\n", __func__, sd->entity.function);
	ep = cameric_md_pipeline_create(cmd);
	if (!ep)
		return -ENOMEM;
	printk(KERN_INFO "%s 4\n", __func__);
	v4l2_set_subdev_hostdata(sd, ep);
	printk(KERN_INFO "%s 5\n", __func__);
	ret = v4l2_device_register_subdev(&cmd->v4l2_dev, sd);
	printk(KERN_INFO "%s 6\n", __func__);
	if (!ret) {
		if (!cmd->pmf && cameric->pdev)
			cmd->pmf = &cameric->pdev->dev;
		cmd->cameric[cameric->id] = cameric;
		cameric->vid_cap.user_subdev_api = cmd->user_subdev_api;
	} else {
		v4l2_err(&cmd->v4l2_dev, "Failed to register cameric.%d (%d)\n",
			 cameric->id, ret);
	}
	printk(KERN_INFO "%s 7\n", __func__);
	return ret;
}

static int register_csis_entity(struct cameric_md *cmd,
				struct platform_device *pdev,
				struct v4l2_subdev *sd)
{
	struct device_node *node = pdev->dev.of_node;
	int id, ret;
	printk(KERN_INFO "%s \n", __func__);
	id = node ? __of_get_csis_id(node) : max(0, pdev->id);

	if (WARN_ON(id < 0 || id >= CSIS_MAX_ENTITIES))
		return -ENOENT;

	if (WARN_ON(cmd->csis[id].sd))
		return -EBUSY;

	sd->grp_id = GRP_ID_CSIS;
	ret = v4l2_device_register_subdev(&cmd->v4l2_dev, sd);
	if (!ret)
		cmd->csis[id].sd = sd;
	else
		v4l2_err(&cmd->v4l2_dev,
			 "Failed to register MIPI-CSIS.%d (%d)\n", id, ret);
	return ret;
}

static int register_cameric_is_entity(struct cameric_md *cmd, struct cameric_is *is)
{
	struct v4l2_subdev *sd = &is->isp.subdev;
	struct cameric_media_pipeline *ep;
	int ret;
	printk(KERN_INFO "%s \n", __func__);
	/* Allocate pipeline object for the ISP capture video node. */
	ep = cameric_md_pipeline_create(cmd);
	if (!ep)
		return -ENOMEM;

	v4l2_set_subdev_hostdata(sd, ep);

	ret = v4l2_device_register_subdev(&cmd->v4l2_dev, sd);
	if (ret) {
		v4l2_err(&cmd->v4l2_dev,
			 "Failed to register cameric-ISP (%d)\n", ret);
		return ret;
	}

	cmd->cameric_is = is;
	return 0;
}

static int cameric_md_register_platform_entity(struct cameric_md *cmd,
					    struct platform_device *pdev,
					    int plat_entity)
{
	struct device *dev = &pdev->dev;
	int ret = -EPROBE_DEFER;
	void *drvdata;
	printk(KERN_INFO "%s 1\n", __func__);
	/* Lock to ensure dev->driver won't change. */
	device_lock(dev);
	printk(KERN_INFO "%s 2, driver=%x\n", __func__, dev->driver);
	if (!dev->driver || !try_module_get(dev->driver->owner))
		goto dev_unlock;
	printk(KERN_INFO "%s 3\n", __func__);
	drvdata = dev_get_drvdata(dev);
	printk(KERN_INFO "%s 4\n", __func__);
	/* Some subdev didn't probe successfully id drvdata is NULL */
	if (drvdata) {
		switch (plat_entity) {
		case IDX_CAMERIC:
			ret = register_cameric_entity(cmd, drvdata);
			break;
		case IDX_CSIS:
			ret = register_csis_entity(cmd, pdev, drvdata);
			break;
		case IDX_IS_ISP:
			ret = register_cameric_is_entity(cmd, drvdata);
			break;
		default:
			ret = -ENODEV;
		}
	}
	printk(KERN_INFO "%s 5n", __func__);
	module_put(dev->driver->owner);
	printk(KERN_INFO "%s 6\n", __func__);
dev_unlock:
	device_unlock(dev);
	if (ret == -EPROBE_DEFER)
		dev_info(&cmd->pdev->dev, "deferring %s device registration\n",
			dev_name(dev));
	else if (ret < 0)
		dev_err(&cmd->pdev->dev, "%s device registration failed (%d)\n",
			dev_name(dev), ret);
	return ret;
}

/* Register cameric, cameric-LITE and CSIS media entities */
static int cameric_md_register_platform_entities(struct cameric_md *cmd,
					      struct device_node *parent)
{
	struct device_node *node;
	int ret = 0;
	printk(KERN_INFO "%s 1, parent node name;%s\n", __func__, parent->name);
	for_each_available_child_of_node(parent, node) {
		struct platform_device *pdev;
		int plat_entity = -1;
		printk(KERN_INFO "%s 21: %s\n", __func__, node->name);
		pdev = of_find_device_by_node(node);
		if (!pdev)
			continue;
		printk(KERN_INFO "%s 22: %s\n", __func__, node->name);
		/* If driver of any entity isn't ready try all again later. */
		if (!strcmp(node->name, CSIS_OF_NODE_NAME))
			plat_entity = IDX_CSIS;
		else if	(!strcmp(node->name, CAMERIC_IS_OF_NODE_NAME))
			plat_entity = IDX_IS_ISP;
		else if	(!strcmp(node->name, CAMERIC_OF_NODE_NAME))
			plat_entity = IDX_CAMERIC;
		printk(KERN_INFO "%s 23, plat_entity=%d\n", __func__, plat_entity);
		if (plat_entity >= 0)
			ret = cameric_md_register_platform_entity(cmd, pdev,
							plat_entity);
		printk(KERN_INFO "%s 24\n", __func__);
		put_device(&pdev->dev);
		if (ret < 0) {
			of_node_put(node);
			break;
		}
		printk(KERN_INFO "%s 25\n", __func__);
	}
	printk(KERN_INFO "%s 3\n", __func__);
	return ret;
}

static void cameric_md_unregister_entities(struct cameric_md *cmd)
{
	int i;
	printk(KERN_INFO "%s \n", __func__);
	for (i = 0; i < CAMERIC_MAX_DEVS; i++) {
		struct cameric_dev *dev = cmd->cameric[i];
		if (dev == NULL)
			continue;
		v4l2_device_unregister_subdev(&dev->vid_cap.subdev);
		dev->vid_cap.ve.pipe = NULL;
		cmd->cameric[i] = NULL;
	}
#if 0
	for (i = 0; i < cameric_LITE_MAX_DEVS; i++) {
		struct cameric_lite *dev = cmd->cameric_lite[i];
		if (dev == NULL)
			continue;
		v4l2_device_unregister_subdev(&dev->subdev);
		dev->ve.pipe = NULL;
		cmd->cameric_lite[i] = NULL;
	}
#endif
	for (i = 0; i < CSIS_MAX_ENTITIES; i++) {
		if (cmd->csis[i].sd == NULL)
			continue;
		v4l2_device_unregister_subdev(cmd->csis[i].sd);
		cmd->csis[i].sd = NULL;
	}

	if (cmd->cameric_is)
		v4l2_device_unregister_subdev(&cmd->cameric_is->isp.subdev);

	v4l2_info(&cmd->v4l2_dev, "Unregistered all entities\n");
}

/**
 * __cameric_md_create_cameric_links - create links to all cameric entities
 * @cmd: cameric media device
 * @source: the source entity to create links to all cameric entities from
 * @sensor: sensor subdev linked to cameric[cameric_id] entity, may be null
 * @pad: the source entity pad index
 * @link_mask: bitmask of the cameric devices for which link should be enabled
 */
static int __cameric_md_create_cameric_sink_links(struct cameric_md *cmd,
					    struct media_entity *source,
					    struct v4l2_subdev *sensor,
					    int pad, int link_mask)
{
	struct cameric_source_info *si = NULL;
	struct media_entity *sink;
	unsigned int flags = 0;
	int i, ret = 0;
	printk(KERN_INFO "%s \n", __func__);
	if (sensor) {
		si = v4l2_get_subdev_hostdata(sensor);
		/* Skip direct cameric links in the logical cameric-IS sensor path */
		if (si && si->cameric_bus_type == CAMERIC_BUS_TYPE_ISP_WRITEBACK)
			ret = 1;
	}

	for (i = 0; !ret && i < CAMERIC_MAX_DEVS; i++) {
		if (!cmd->cameric[i])
			continue;
		/*
		 * Some cameric variants are not fitted with camera capture
		 * interface. Skip creating a link from sensor for those.
		 */
		if (!cmd->cameric[i]->variant->has_cam_if)
			continue;

		flags = ((1 << i) & link_mask) ? MEDIA_LNK_FL_ENABLED : 0;

		sink = &cmd->cameric[i]->vid_cap.subdev.entity;
		ret = media_create_pad_link(source, pad, sink,
					      CAMERIC_SD_PAD_SINK_CAM, flags);
		if (ret)
			return ret;

		/* Notify cameric capture subdev entity */
		ret = media_entity_call(sink, link_setup, &sink->pads[0],
					&source->pads[pad], flags);
		if (ret)
			break;

		v4l2_info(&cmd->v4l2_dev, "created link [%s] %c> [%s]\n",
			  source->name, flags ? '=' : '-', sink->name);
	}

	return 0;
}

/* Create cameric-IS links */
static int __cameric_md_create_cameric_is_links(struct cameric_md *cmd)
{
	struct cameric_isp *isp = &cmd->cameric_is->isp;
	struct media_entity *source, *sink;
	int i, ret;
	printk(KERN_INFO "%s \n", __func__);
	source = &isp->subdev.entity;

	for (i = 0; i < CAMERIC_MAX_DEVS; i++) {
		if (cmd->cameric[i] == NULL)
			continue;

		/* Link from cameric-IS-ISP subdev to cameric */
		sink = &cmd->cameric[i]->vid_cap.subdev.entity;
		ret = media_create_pad_link(source, CAMERIC_ISP_SD_PAD_SRC_FIFO,
					       sink, CAMERIC_SD_PAD_SINK_FIFO, 0);
		if (ret)
			return ret;
	}

	/* Link from cameric-IS-ISP subdev to cameric-is-isp.capture video node */
	sink = &isp->video_capture.ve.vdev.entity;

	/* Skip this link if the cameric-is-isp video node driver isn't built-in */
	if (sink->num_pads == 0)
		return 0;

	return media_create_pad_link(source, CAMERIC_ISP_SD_PAD_SRC_DMA,
					sink, 0, 0);
}

/**
 * cameric_md_create_links - create default links between registered entities
 *
 * Parallel interface sensor entities are connected directly to cameric capture
 * entities. The sensors using MIPI CSIS bus are connected through immutable
 * link with CSI receiver entity specified by mux_id. Any registered CSIS
 * entity has a link to each registered cameric capture entity. Enabled links
 * are created by default between each subsequent registered sensor and
 * subsequent cameric capture entity. The number of default active links is
 * determined by the number of available sensors or cameric entities,
 * whichever is less.
 */
static int cameric_md_create_links(struct cameric_md *cmd)
{
	struct v4l2_subdev *csi_sensors[CSIS_MAX_ENTITIES] = { NULL };
	struct v4l2_subdev *sensor, *csis;
	struct cameric_source_info *pdata;
	struct media_entity *source, *sink;
	int i, pad, cameric_id = 0, ret = 0;
	u32 flags, link_mask = 0;
	printk(KERN_INFO "%s \n", __func__);
	for (i = 0; i < cmd->num_sensors; i++) {
		if (cmd->sensor[i].subdev == NULL)
			continue;

		sensor = cmd->sensor[i].subdev;
		pdata = v4l2_get_subdev_hostdata(sensor);
		if (!pdata)
			continue;

		source = NULL;

		switch (pdata->sensor_bus_type) {
		case CAMERIC_BUS_TYPE_MIPI_CSI2:
			if (WARN(pdata->mux_id >= CSIS_MAX_ENTITIES,
				"Wrong CSI channel id: %d\n", pdata->mux_id))
				return -EINVAL;

			csis = cmd->csis[pdata->mux_id].sd;
			if (WARN(csis == NULL,
				 "MIPI-CSI interface specified but rk-csis module is not loaded!\n"))
				return -EINVAL;

			pad = sensor->entity.num_pads - 1;
			ret = media_create_pad_link(&sensor->entity, pad,
					      &csis->entity, CSIS_PAD_SINK,
					      MEDIA_LNK_FL_IMMUTABLE |
					      MEDIA_LNK_FL_ENABLED);
			if (ret)
				return ret;

			v4l2_info(&cmd->v4l2_dev, "created link [%s] => [%s]\n",
				  sensor->entity.name, csis->entity.name);

			source = NULL;
			csi_sensors[pdata->mux_id] = sensor;
			break;

		case CAMERIC_BUS_TYPE_ITU_601...CAMERIC_BUS_TYPE_ITU_656:
			source = &sensor->entity;
			pad = 0;
			break;

		default:
			v4l2_err(&cmd->v4l2_dev, "Wrong bus_type: %x\n",
				 pdata->sensor_bus_type);
			return -EINVAL;
		}
		if (source == NULL)
			continue;

		link_mask = 1 << cameric_id++;
		ret = __cameric_md_create_cameric_sink_links(cmd, source, sensor,
						       pad, link_mask);
	}

	for (i = 0; i < CSIS_MAX_ENTITIES; i++) {
		if (cmd->csis[i].sd == NULL)
			continue;

		source = &cmd->csis[i].sd->entity;
		pad = CSIS_PAD_SOURCE;
		sensor = csi_sensors[i];

		link_mask = 1 << cameric_id++;
		ret = __cameric_md_create_cameric_sink_links(cmd, source, sensor,
						       pad, link_mask);
	}

	/* Create immutable links between each cameric's subdev and video node */
	flags = MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED;
	for (i = 0; i < CAMERIC_MAX_DEVS; i++) {
		if (!cmd->cameric[i])
			continue;

		source = &cmd->cameric[i]->vid_cap.subdev.entity;
		sink = &cmd->cameric[i]->vid_cap.ve.vdev.entity;

		ret = media_create_pad_link(source, CAMERIC_SD_PAD_SOURCE,
					      sink, 0, flags);
		if (ret)
			break;
	}

	if (cmd->use_isp)
		ret = __cameric_md_create_cameric_is_links(cmd);

	return ret;
}

/*
 * The peripheral sensor and CAM_BLK (PIXELASYNCMx) clocks management.
 */
static void cameric_md_put_clocks(struct cameric_md *cmd)
{
	int i = CAMERIC_MAX_CAMCLKS;
	printk(KERN_INFO "%s \n", __func__);
	while (--i >= 0) {
		if (IS_ERR(cmd->camclk[i].clock))
			continue;
		clk_put(cmd->camclk[i].clock);
		cmd->camclk[i].clock = ERR_PTR(-EINVAL);
	}

	/* Writeback (PIXELASYNCMx) clocks */
	for (i = 0; i < CAMERIC_MAX_WBCLKS; i++) {
		if (IS_ERR(cmd->wbclk[i]))
			continue;
		clk_put(cmd->wbclk[i]);
		cmd->wbclk[i] = ERR_PTR(-EINVAL);
	}
}

static int cameric_md_get_clocks(struct cameric_md *cmd)
{
	struct device *dev = &cmd->pdev->dev;
	char clk_name[32];
	struct clk *clock;
	int i, ret = 0;
	printk(KERN_INFO "%s \n", __func__);
	for (i = 0; i < CAMERIC_MAX_CAMCLKS; i++)
		cmd->camclk[i].clock = ERR_PTR(-EINVAL);

	for (i = 0; i < CAMERIC_MAX_CAMCLKS; i++) {
		snprintf(clk_name, sizeof(clk_name), "sclk_cam%u", i);
		clock = clk_get(dev, clk_name);

		if (IS_ERR(clock)) {
			dev_err(dev, "Failed to get clock: %s\n", clk_name);
			ret = PTR_ERR(clock);
			break;
		}
		cmd->camclk[i].clock = clock;
	}
	if (ret)
		cameric_md_put_clocks(cmd);

	if (!cmd->use_isp)
		return 0;
	/*
	 * For now get only PIXELASYNCM1 clock (Writeback B/ISP),
	 * leave PIXELASYNCM0 out for the LCD Writeback driver.
	 */
	cmd->wbclk[CLK_IDX_WB_A] = ERR_PTR(-EINVAL);

	for (i = CLK_IDX_WB_B; i < CAMERIC_MAX_WBCLKS; i++) {
		snprintf(clk_name, sizeof(clk_name), "pxl_async%u", i);
		clock = clk_get(dev, clk_name);
		if (IS_ERR(clock)) {
			v4l2_err(&cmd->v4l2_dev, "Failed to get clock: %s\n",
				  clk_name);
			ret = PTR_ERR(clock);
			break;
		}
		cmd->wbclk[i] = clock;
	}
	if (ret)
		cameric_md_put_clocks(cmd);

	return ret;
}

static int __cameric_md_modify_pipeline(struct media_entity *entity, bool enable)
{

	struct cameric_video_entity *ve;
	struct cameric_pipeline *p;
	struct video_device *vdev;
	int ret;
	printk(KERN_INFO "%s \n", __func__);
	vdev = media_entity_to_video_device(entity);
	if (vdev->entity.use_count == 0)
		return 0;

	ve = vdev_to_cameric_video_entity(vdev);
	p = to_cameric_pipeline(ve->pipe);
	/*
	 * Nothing to do if we are disabling the pipeline, some link
	 * has been disconnected and p->subdevs array is cleared now.
	 */
	if (!enable && p->subdevs[IDX_SENSOR] == NULL)
		return 0;

	if (enable)
		ret = __cameric_pipeline_open(ve->pipe, entity, true);
	else
		ret = __cameric_pipeline_close(ve->pipe);

	if (ret == 0 && !enable)
		memset(p->subdevs, 0, sizeof(p->subdevs));

	return ret;
}

/* Locking: called with entity->graph_obj.mdev->graph_mutex mutex held. */
static int __cameric_md_modify_pipelines(struct media_entity *entity, bool enable,
				      struct media_graph *graph)
{
	printk(KERN_INFO "%s \n", __func__);
	struct media_entity *entity_err = entity;
	int ret;

	/*
	 * Walk current graph and call the pipeline open/close routine for each
	 * opened video node that belongs to the graph of entities connected
	 * through active links. This is needed as we cannot power on/off the
	 * subdevs in random order.
	 */
	media_graph_walk_start(graph, entity);

	while ((entity = media_graph_walk_next(graph))) {
		if (!is_media_entity_v4l2_video_device(entity))
			continue;

		ret  = __cameric_md_modify_pipeline(entity, enable);

		if (ret < 0)
			goto err;
	}

	return 0;

err:
	media_graph_walk_start(graph, entity_err);

	while ((entity_err = media_graph_walk_next(graph))) {
		if (!is_media_entity_v4l2_video_device(entity_err))
			continue;

		__cameric_md_modify_pipeline(entity_err, !enable);

		if (entity_err == entity)
			break;
	}

	return ret;
}

static int cameric_md_link_notify(struct media_link *link, unsigned int flags,
				unsigned int notification)
{
	struct media_graph *graph =
		&container_of(link->graph_obj.mdev, struct cameric_md,
			      media_dev)->link_setup_graph;
	struct media_entity *sink = link->sink->entity;
	int ret = 0;
	printk(KERN_INFO "%s \n", __func__);
	/* Before link disconnection */
	if (notification == MEDIA_DEV_NOTIFY_PRE_LINK_CH) {
		ret = media_graph_walk_init(graph,
						   link->graph_obj.mdev);
		if (ret)
			return ret;
		if (!(flags & MEDIA_LNK_FL_ENABLED))
			ret = __cameric_md_modify_pipelines(sink, false, graph);
#if 0
		else
			/* TODO: Link state change validation */
#endif
	/* After link activation */
	} else if (notification == MEDIA_DEV_NOTIFY_POST_LINK_CH) {
		if (link->flags & MEDIA_LNK_FL_ENABLED)
			ret = __cameric_md_modify_pipelines(sink, true, graph);
		media_graph_walk_cleanup(graph);
	}

	return ret ? -EPIPE : 0;
}

static const struct media_device_ops cameric_md_ops = {
	.link_notify = cameric_md_link_notify,
};

static ssize_t cameric_md_sysfs_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct cameric_md *cmd = platform_get_drvdata(pdev);
	printk(KERN_INFO "%s \n", __func__);
	if (cmd->user_subdev_api)
		return strlcpy(buf, "Sub-device API (sub-dev)\n", PAGE_SIZE);

	return strlcpy(buf, "V4L2 video node only API (vid-dev)\n", PAGE_SIZE);
}

static ssize_t cameric_md_sysfs_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct cameric_md *cmd = platform_get_drvdata(pdev);
	bool subdev_api;
	int i;
	printk(KERN_INFO "%s \n", __func__);
	if (!strcmp(buf, "vid-dev\n"))
		subdev_api = false;
	else if (!strcmp(buf, "sub-dev\n"))
		subdev_api = true;
	else
		return count;

	cmd->user_subdev_api = subdev_api;
	for (i = 0; i < CAMERIC_MAX_DEVS; i++)
		if (cmd->cameric[i])
			cmd->cameric[i]->vid_cap.user_subdev_api = subdev_api;
	return count;
}
/*
 * This device attribute is to select video pipeline configuration method.
 * There are following valid values:
 *  vid-dev - for V4L2 video node API only, subdevice will be configured
 *  by the host driver.
 *  sub-dev - for media controller API, subdevs must be configured in user
 *  space before starting streaming.
 */
static DEVICE_ATTR(subdev_conf_mode, S_IWUSR | S_IRUGO,
		   cameric_md_sysfs_show, cameric_md_sysfs_store);

static int cameric_md_get_pinctrl(struct cameric_md *cmd)
{
	struct device *dev = &cmd->pdev->dev;
	struct cameric_pinctrl *pctl = &cmd->pinctl;

	printk(KERN_INFO "%s 1\n", __func__);
	pctl->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(pctl->pinctrl))
		return PTR_ERR(pctl->pinctrl);
	printk(KERN_INFO "%s 2\n", __func__);
	pctl->state_default = pinctrl_lookup_state(pctl->pinctrl,
					PINCTRL_STATE_DEFAULT);
	if (IS_ERR(pctl->state_default))
		return PTR_ERR(pctl->state_default);
	else
		pinctrl_select_state(pctl->pinctrl, pctl->state_default);
	printk(KERN_INFO "%s 3\n", __func__);
	pctl->state_idle = pinctrl_lookup_state(pctl->pinctrl,
					PINCTRL_STATE_IDLE);
	if (IS_ERR(pctl->state_idle))
		return PTR_ERR(pctl->state_idle);
	printk(KERN_INFO "%s 4\n", __func__);
	pctl->pins_sleep = pinctrl_lookup_state(pctl->pinctrl,
					PINCTRL_STATE_SLEEP);
	if (IS_ERR(pctl->pins_sleep))
		return PTR_ERR(pctl->pins_sleep);
	printk(KERN_INFO "%s 5\n", __func__);
	return 0;
}

static int cam_clk_prepare(struct clk_hw *hw)
{
	struct cam_clk *camclk = to_cam_clk(hw);
	int ret;
	printk(KERN_INFO "%s \n", __func__);
	if (camclk->cmd->pmf == NULL)
		return -ENODEV;

	ret = pm_runtime_get_sync(camclk->cmd->pmf);
	return ret < 0 ? ret : 0;
}

static void cam_clk_unprepare(struct clk_hw *hw)
{
	struct cam_clk *camclk = to_cam_clk(hw);
	printk(KERN_INFO "%s \n", __func__);
	if (camclk->cmd->pmf == NULL)
		return;

	pm_runtime_put_sync(camclk->cmd->pmf);
}

static const struct clk_ops cam_clk_ops = {
	.prepare = cam_clk_prepare,
	.unprepare = cam_clk_unprepare,
};

static void cameric_md_unregister_clk_provider(struct cameric_md *cmd)
{
	struct cam_clk_provider *cp = &cmd->clk_provider;
	unsigned int i;
	printk(KERN_INFO "%s \n", __func__);
	if (cp->of_node)
		of_clk_del_provider(cp->of_node);

	for (i = 0; i < cp->num_clocks; i++)
		clk_unregister(cp->clks[i]);
}

static int cameric_md_register_clk_provider(struct cameric_md *cmd)
{
	struct cam_clk_provider *cp = &cmd->clk_provider;
	struct device *dev = &cmd->pdev->dev;
	int i, ret;
	printk(KERN_INFO "%s \n", __func__);
	for (i = 0; i < CAMERIC_MAX_CAMCLKS; i++) {
		struct cam_clk *camclk = &cp->camclk[i];
		struct clk_init_data init;
		const char *p_name;

		ret = of_property_read_string_index(dev->of_node,
					"clock-output-names", i, &init.name);
		if (ret < 0)
			break;

		p_name = __clk_get_name(cmd->camclk[i].clock);

		/* It's safe since clk_register() will duplicate the string. */
		init.parent_names = &p_name;
		init.num_parents = 1;
		init.ops = &cam_clk_ops;
		init.flags = CLK_SET_RATE_PARENT;
		camclk->hw.init = &init;
		camclk->cmd = cmd;

		cp->clks[i] = clk_register(NULL, &camclk->hw);
		if (IS_ERR(cp->clks[i])) {
			dev_err(dev, "failed to register clock: %s (%ld)\n",
					init.name, PTR_ERR(cp->clks[i]));
			ret = PTR_ERR(cp->clks[i]);
			goto err;
		}
		cp->num_clocks++;
	}

	if (cp->num_clocks == 0) {
		dev_warn(dev, "clk provider not registered\n");
		return 0;
	}

	cp->clk_data.clks = cp->clks;
	cp->clk_data.clk_num = cp->num_clocks;
	cp->of_node = dev->of_node;
	ret = of_clk_add_provider(dev->of_node, of_clk_src_onecell_get,
				  &cp->clk_data);
	if (ret == 0)
		return 0;
err:
	cameric_md_unregister_clk_provider(cmd);
	return ret;
}

static int subdev_notifier_bound(struct v4l2_async_notifier *notifier,
				 struct v4l2_subdev *subdev,
				 struct v4l2_async_subdev *asd)
{
	struct cameric_md *cmd = notifier_to_cameric_md(notifier);
	struct cameric_sensor_info *si = NULL;
	int i;
	printk(KERN_INFO "%s \n", __func__);
	/* Find platform data for this sensor subdev */
	for (i = 0; i < ARRAY_SIZE(cmd->sensor); i++)
		if (cmd->sensor[i].asd.match.of.node == subdev->dev->of_node)
			si = &cmd->sensor[i];

	if (si == NULL)
		return -EINVAL;

	v4l2_set_subdev_hostdata(subdev, &si->pdata);

	if (si->pdata.cameric_bus_type == CAMERIC_BUS_TYPE_ISP_WRITEBACK)
		subdev->grp_id = GRP_ID_CAMERIC_IS_SENSOR;
	else
		subdev->grp_id = GRP_ID_SENSOR;

	si->subdev = subdev;

	v4l2_info(&cmd->v4l2_dev, "Registered sensor subdevice: %s (%d)\n",
		  subdev->name, cmd->num_sensors);

	cmd->num_sensors++;

	return 0;
}

static int subdev_notifier_complete(struct v4l2_async_notifier *notifier)
{
	struct cameric_md *cmd = notifier_to_cameric_md(notifier);
	int ret;
	printk(KERN_INFO "%s \n", __func__);
	mutex_lock(&cmd->media_dev.graph_mutex);

	ret = cameric_md_create_links(cmd);
	if (ret < 0)
		goto unlock;

	ret = v4l2_device_register_subdev_nodes(&cmd->v4l2_dev);
unlock:
	mutex_unlock(&cmd->media_dev.graph_mutex);
	if (ret < 0)
		return ret;

	return media_device_register(&cmd->media_dev);
}

static int cameric_md_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct v4l2_device *v4l2_dev;
	struct cameric_md *cmd;
	int ret;
	printk(KERN_INFO "%s 1, device node name:%s\n", __func__, dev->of_node->name);
	cmd = devm_kzalloc(dev, sizeof(*cmd), GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;
	printk(KERN_INFO "%s 2\n", __func__);
	spin_lock_init(&cmd->slock);
	INIT_LIST_HEAD(&cmd->pipelines);
	cmd->pdev = pdev;
	printk(KERN_INFO "%s 3\n", __func__);
	strlcpy(cmd->media_dev.model, "rockchip cameric",
		sizeof(cmd->media_dev.model));
	cmd->media_dev.ops = &cameric_md_ops;
	cmd->media_dev.dev = dev;
	printk(KERN_INFO "%s 4\n", __func__);
	v4l2_dev = &cmd->v4l2_dev;
	v4l2_dev->mdev = &cmd->media_dev;
	v4l2_dev->notify = cameric_sensor_notify;
	strlcpy(v4l2_dev->name, "rk-cameric-md", sizeof(v4l2_dev->name));
	printk(KERN_INFO "%s 5\n", __func__);
	cmd->use_isp = cameric_md_is_isp_available(dev->of_node);
	cmd->user_subdev_api = true;
	printk(KERN_INFO "%s 6\n", __func__);
	media_device_init(&cmd->media_dev);
	printk(KERN_INFO "%s 7\n", __func__);
	ret = v4l2_device_register(dev, &cmd->v4l2_dev);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "Failed to register v4l2_device: %d\n", ret);
		return ret;
	}
	printk(KERN_INFO "%s 8\n", __func__);
	//ret = cameric_md_get_clocks(cmd);
	//if (ret)
	//	goto err_md;
	//printk(KERN_INFO "%s 9\n", __func__);
	cameric_md_get_pinctrl(cmd);
	//if (ret < 0) {
		//if (ret != EPROBE_DEFER)
			//dev_err(dev, "Failed to get pinctrl: %d\n", ret);
		//goto err_clk;
	//}
	printk(KERN_INFO "%s 10\n", __func__);
	platform_set_drvdata(pdev, cmd);
	printk(KERN_INFO "%s 11\n", __func__);
	ret = cameric_md_register_platform_entities(cmd, dev->of_node);
	if (ret)
		goto err_clk;
	printk(KERN_INFO "%s 12\n", __func__);
	ret = cameric_md_register_sensor_entities(cmd);
	if (ret)
		goto err_m_ent;
	printk(KERN_INFO "%s 13\n", __func__);
	ret = device_create_file(&pdev->dev, &dev_attr_subdev_conf_mode);
	if (ret)
		goto err_m_ent;
	/*
	 * cameric platform devices need to be registered before the sclk_cam
	 * clocks provider, as one of these devices needs to be activated
	 * to enable the clock.
	 */
	ret = cameric_md_register_clk_provider(cmd);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "clock provider registration failed\n");
		goto err_attr;
	}
	printk(KERN_INFO "%s 14\n", __func__);
	if (cmd->num_sensors > 0) {
		cmd->subdev_notifier.subdevs = cmd->async_subdevs;
		cmd->subdev_notifier.num_subdevs = cmd->num_sensors;
		cmd->subdev_notifier.bound = subdev_notifier_bound;
		cmd->subdev_notifier.complete = subdev_notifier_complete;
		cmd->num_sensors = 0;

		ret = v4l2_async_notifier_register(&cmd->v4l2_dev,
						&cmd->subdev_notifier);
		if (ret)
			goto err_clk_p;
	}
	printk(KERN_INFO "%s 15\n", __func__);
	return 0;

err_clk_p:
	cameric_md_unregister_clk_provider(cmd);
err_attr:
	device_remove_file(&pdev->dev, &dev_attr_subdev_conf_mode);
err_clk:
	cameric_md_put_clocks(cmd);
err_m_ent:
	cameric_md_unregister_entities(cmd);
err_md:
	media_device_cleanup(&cmd->media_dev);
	v4l2_device_unregister(&cmd->v4l2_dev);
	return ret;
}

static int cameric_md_remove(struct platform_device *pdev)
{
	struct cameric_md *cmd = platform_get_drvdata(pdev);
	printk(KERN_INFO "%s\n", __func__);
	if (!cmd)
		return 0;

	cameric_md_unregister_clk_provider(cmd);
	v4l2_async_notifier_unregister(&cmd->subdev_notifier);

	v4l2_device_unregister(&cmd->v4l2_dev);
	device_remove_file(&pdev->dev, &dev_attr_subdev_conf_mode);
	cameric_md_unregister_entities(cmd);
	cameric_md_pipelines_free(cmd);
	media_device_unregister(&cmd->media_dev);
	media_device_cleanup(&cmd->media_dev);
	cameric_md_put_clocks(cmd);

	return 0;
}

static const struct platform_device_id cameric_driver_ids[] __always_unused = {
	{ .name = "rk-cameric-md" },
	{ },
};
MODULE_DEVICE_TABLE(platform, cameric_driver_ids);

static const struct of_device_id cameric_md_of_match[] = {
	{ .compatible = "rockchip,cameric" },
	{ },
};
MODULE_DEVICE_TABLE(of, cameric_md_of_match);

static struct platform_driver cameric_md_driver = {
	.probe		= cameric_md_probe,
	.remove		= cameric_md_remove,
	.driver = {
		.of_match_table = of_match_ptr(cameric_md_of_match),
		.name		= "cameric-md",
	}
};

static int __init cameric_md_init(void)
{
	int ret;
	printk(KERN_INFO "%s 1\n", __func__);
	request_module("cameric-csis");
	printk(KERN_INFO "%s 2\n", __func__);
	ret = cameric_register_driver();
	printk(KERN_INFO "%s 3\n", __func__);
	if (ret)
		return ret;
	printk(KERN_INFO "%s 4\n", __func__);
	ret = platform_driver_register(&cameric_md_driver);
	printk(KERN_INFO "%s 5 ret = %d\n", __func__, ret);
	return ret;
}

static void __exit cameric_md_exit(void)
{
	printk(KERN_INFO "%s 1\n", __func__);
	platform_driver_unregister(&cameric_md_driver);
	printk(KERN_INFO "%s 2\n", __func__);
	cameric_unregister_driver();
	printk(KERN_INFO "%s 3\n", __func__);
}

module_init(cameric_md_init);
module_exit(cameric_md_exit);

MODULE_AUTHOR("Eddie Cai <eddie.cai.linux@gmail.com>");
MODULE_DESCRIPTION("cameric host interface media device driver host interface/video postprocessor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("2.0.1");
