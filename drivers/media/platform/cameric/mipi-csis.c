/*
 * Samsung CAMERIC_/EXYNOS SoC series MIPI-CSI receiver driver
 *
 * Copyright (C) 2011 - 2013 Samsung Electronics Co., Ltd.
 * Author: Sylwester Nawrocki <s.nawrocki@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/memory.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/videodev2.h>
#include <media/drv-intf/cameric.h>
#include <media/v4l2-of.h>
#include <media/v4l2-subdev.h>

#include "mipi-csis.h"

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-2)");

/* Register map definition */

/* CSIS global control */
#define CAMERIC_CSIS_CTRL			0x00
#define CAMERIC_CSIS_CTRL_DPDN_DEFAULT	(0 << 31)
#define CAMERIC_CSIS_CTRL_DPDN_SWAP		(1 << 31)
#define CAMERIC_CSIS_CTRL_ALIGN_32BIT	(1 << 20)
#define CAMERIC_CSIS_CTRL_UPDATE_SHADOW	(1 << 16)
#define CAMERIC_CSIS_CTRL_WCLK_EXTCLK	(1 << 8)
#define CAMERIC_CSIS_CTRL_RESET		(1 << 4)
#define CAMERIC_CSIS_CTRL_ENABLE		(1 << 0)

/* D-PHY control */
#define CAMERIC_CSIS_DPHYCTRL		0x04
#define CAMERIC_CSIS_DPHYCTRL_HSS_MASK	(0x1f << 27)
#define CAMERIC_CSIS_DPHYCTRL_ENABLE		(0x1f << 0)

#define CAMERIC_CSIS_CONFIG			0x08
#define CAMERIC_CSIS_CFG_FMT_YCBCR422_8BIT	(0x1e << 2)
#define CAMERIC_CSIS_CFG_FMT_RAW8		(0x2a << 2)
#define CAMERIC_CSIS_CFG_FMT_RAW10		(0x2b << 2)
#define CAMERIC_CSIS_CFG_FMT_RAW12		(0x2c << 2)
/* User defined formats, x = 1...4 */
#define CAMERIC_CSIS_CFG_FMT_USER(x)		((0x30 + x - 1) << 2)
#define CAMERIC_CSIS_CFG_FMT_MASK		(0x3f << 2)
#define CAMERIC_CSIS_CFG_NR_LANE_MASK	3

/* Interrupt mask */
#define CAMERIC_CSIS_INTMSK			0x10
#define CAMERIC_CSIS_INTMSK_EVEN_BEFORE	(1 << 31)
#define CAMERIC_CSIS_INTMSK_EVEN_AFTER	(1 << 30)
#define CAMERIC_CSIS_INTMSK_ODD_BEFORE	(1 << 29)
#define CAMERIC_CSIS_INTMSK_ODD_AFTER	(1 << 28)
#define CAMERIC_CSIS_INTMSK_FRAME_START	(1 << 27)
#define CAMERIC_CSIS_INTMSK_FRAME_END	(1 << 26)
#define CAMERIC_CSIS_INTMSK_ERR_SOT_HS	(1 << 12)
#define CAMERIC_CSIS_INTMSK_ERR_LOST_FS	(1 << 5)
#define CAMERIC_CSIS_INTMSK_ERR_LOST_FE	(1 << 4)
#define CAMERIC_CSIS_INTMSK_ERR_OVER		(1 << 3)
#define CAMERIC_CSIS_INTMSK_ERR_ECC		(1 << 2)
#define CAMERIC_CSIS_INTMSK_ERR_CRC		(1 << 1)
#define CAMERIC_CSIS_INTMSK_ERR_UNKNOWN	(1 << 0)
#define CAMERIC_CSIS_INTMSK_RK3288_EN_ALL	0xf000103f
#define CAMERIC_CSIS_INTMSK_RK3399_EN_ALL	0xfc00103f

/* Interrupt source */
#define CAMERIC_CSIS_INTSRC			0x14
#define CAMERIC_CSIS_INTSRC_EVEN_BEFORE	(1 << 31)
#define CAMERIC_CSIS_INTSRC_EVEN_AFTER	(1 << 30)
#define CAMERIC_CSIS_INTSRC_EVEN		(0x3 << 30)
#define CAMERIC_CSIS_INTSRC_ODD_BEFORE	(1 << 29)
#define CAMERIC_CSIS_INTSRC_ODD_AFTER	(1 << 28)
#define CAMERIC_CSIS_INTSRC_ODD		(0x3 << 28)
#define CAMERIC_CSIS_INTSRC_NON_IMAGE_DATA	(0xf << 28)
#define CAMERIC_CSIS_INTSRC_FRAME_START	(1 << 27)
#define CAMERIC_CSIS_INTSRC_FRAME_END	(1 << 26)
#define CAMERIC_CSIS_INTSRC_ERR_SOT_HS	(0xf << 12)
#define CAMERIC_CSIS_INTSRC_ERR_LOST_FS	(1 << 5)
#define CAMERIC_CSIS_INTSRC_ERR_LOST_FE	(1 << 4)
#define CAMERIC_CSIS_INTSRC_ERR_OVER		(1 << 3)
#define CAMERIC_CSIS_INTSRC_ERR_ECC		(1 << 2)
#define CAMERIC_CSIS_INTSRC_ERR_CRC		(1 << 1)
#define CAMERIC_CSIS_INTSRC_ERR_UNKNOWN	(1 << 0)
#define CAMERIC_CSIS_INTSRC_ERRORS		0xf03f

/* Pixel resolution */
#define CAMERIC_CSIS_RESOL			0x2c
#define CSIS_MAX_PIX_WIDTH		0xffff
#define CSIS_MAX_PIX_HEIGHT		0xffff

/* Non-image packet data buffers */
#define CAMERIC_CSIS_PKTDATA_ODD		0x2000
#define CAMERIC_CSIS_PKTDATA_EVEN		0x3000
#define CAMERIC_CSIS_PKTDATA_SIZE		SZ_4K

enum {
	CSIS_CLK_MUX,
	CSIS_CLK_GATE,
};

static char *csi_clock_name[] = {
	[CSIS_CLK_MUX]  = "sclk_csis",
	[CSIS_CLK_GATE] = "csis",
};
#define NUM_CSIS_CLOCKS	ARRAY_SIZE(csi_clock_name)
#define DEFAULT_SCLK_CSIS_FREQ	166000000UL

static const char * const csis_supply_name[] = {
	"vddcore",  /* CSIS Core (1.0V, 1.1V or 1.2V) suppply */
	"vddio",    /* CSIS I/O and PLL (1.8V) supply */
};
#define CSIS_NUM_SUPPLIES ARRAY_SIZE(csis_supply_name)

enum {
	ST_POWERED	= 1,
	ST_STREAMING	= 2,
	ST_SUSPENDED	= 4,
};

struct cameric_csis_event {
	u32 mask;
	const char * const name;
	unsigned int counter;
};

static const struct cameric_csis_event cameric_csis_events[] = {
	/* Errors */
	{ CAMERIC_CSIS_INTSRC_ERR_SOT_HS,	"SOT Error" },
	{ CAMERIC_CSIS_INTSRC_ERR_LOST_FS,	"Lost Frame Start Error" },
	{ CAMERIC_CSIS_INTSRC_ERR_LOST_FE,	"Lost Frame End Error" },
	{ CAMERIC_CSIS_INTSRC_ERR_OVER,	"FIFO Overflow Error" },
	{ CAMERIC_CSIS_INTSRC_ERR_ECC,	"ECC Error" },
	{ CAMERIC_CSIS_INTSRC_ERR_CRC,	"CRC Error" },
	{ CAMERIC_CSIS_INTSRC_ERR_UNKNOWN,	"Unknown Error" },
	/* Non-image data receive events */
	{ CAMERIC_CSIS_INTSRC_EVEN_BEFORE,	"Non-image data before even frame" },
	{ CAMERIC_CSIS_INTSRC_EVEN_AFTER,	"Non-image data after even frame" },
	{ CAMERIC_CSIS_INTSRC_ODD_BEFORE,	"Non-image data before odd frame" },
	{ CAMERIC_CSIS_INTSRC_ODD_AFTER,	"Non-image data after odd frame" },
	/* Frame start/end */
	{ CAMERIC_CSIS_INTSRC_FRAME_START,	"Frame Start" },
	{ CAMERIC_CSIS_INTSRC_FRAME_END,	"Frame End" },
};
#define CAMERIC_CSIS_NUM_EVENTS ARRAY_SIZE(cameric_csis_events)

struct csis_pktbuf {
	u32 *data;
	unsigned int len;
};

struct csis_drvdata {
	/* Mask of all used interrupts in CAMERIC_CSIS_INTMSK register */
	u32 interrupt_mask;
};

/**
 * struct csis_state - the driver's internal state data structure
 * @lock: mutex serializing the subdev and power management operations,
 *        protecting @format and @flags members
 * @pads: CSIS pads array
 * @sd: v4l2_subdev associated with CSIS device instance
 * @index: the hardware instance index
 * @pdev: CSIS platform device
 * @phy: pointer to the CSIS generic PHY
 * @regs: mmaped I/O registers memory
 * @supplies: CSIS regulator supplies
 * @clock: CSIS clocks
 * @irq: requested cameric_-mipi-csis irq number
 * @interrupt_mask: interrupt mask of the all used interrupts
 * @flags: the state variable for power and streaming control
 * @clock_frequency: device bus clock frequency
 * @hs_settle: HS-RX settle time
 * @num_lanes: number of MIPI-CSI data lanes used
 * @max_num_lanes: maximum number of MIPI-CSI data lanes supported
 * @wclk_ext: CSI wrapper clock: 0 - bus clock, 1 - external SCLK_CAM
 * @csis_fmt: current CSIS pixel format
 * @format: common media bus format for the source and sink pad
 * @slock: spinlock protecting structure members below
 * @pkt_buf: the frame embedded (non-image) data buffer
 * @events: MIPI-CSIS event (error) counters
 */
struct csis_state {
	struct mutex lock;
	struct media_pad pads[CSIS_PADS_NUM];
	struct v4l2_subdev sd;
	u8 index;
	struct platform_device *pdev;
	struct phy *phy;
	void __iomem *regs;
	struct regulator_bulk_data supplies[CSIS_NUM_SUPPLIES];
	struct clk *clock[NUM_CSIS_CLOCKS];
	int irq;
	u32 interrupt_mask;
	u32 flags;

	u32 clk_frequency;
	u32 hs_settle;
	u32 num_lanes;
	u32 max_num_lanes;
	u8 wclk_ext;

	const struct csis_pix_format *csis_fmt;
	struct v4l2_mbus_framefmt format;

	spinlock_t slock;
	struct csis_pktbuf pkt_buf;
	struct cameric_csis_event events[CAMERIC_CSIS_NUM_EVENTS];
};

/**
 * struct csis_pix_format - CSIS pixel format description
 * @pix_width_alignment: horizontal pixel alignment, width will be
 *                       multiple of 2^pix_width_alignment
 * @code: corresponding media bus code
 * @fmt_reg: CAMERIC_CSIS_CONFIG register value
 * @data_alignment: MIPI-CSI data alignment in bits
 */
struct csis_pix_format {
	unsigned int pix_width_alignment;
	u32 code;
	u32 fmt_reg;
	u8 data_alignment;
};

static const struct csis_pix_format cameric_csis_formats[] = {
	{
		.code = MEDIA_BUS_FMT_VYUY8_2X8,
		.fmt_reg = CAMERIC_CSIS_CFG_FMT_YCBCR422_8BIT,
		.data_alignment = 32,
	}, {
		.code = MEDIA_BUS_FMT_JPEG_1X8,
		.fmt_reg = CAMERIC_CSIS_CFG_FMT_USER(1),
		.data_alignment = 32,
	}, {
		.code = MEDIA_BUS_FMT_S5C_UYVY_JPEG_1X8,
		.fmt_reg = CAMERIC_CSIS_CFG_FMT_USER(1),
		.data_alignment = 32,
	}, {
		.code = MEDIA_BUS_FMT_SGRBG8_1X8,
		.fmt_reg = CAMERIC_CSIS_CFG_FMT_RAW8,
		.data_alignment = 24,
	}, {
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
		.fmt_reg = CAMERIC_CSIS_CFG_FMT_RAW10,
		.data_alignment = 24,
	}, {
		.code = MEDIA_BUS_FMT_SGRBG12_1X12,
		.fmt_reg = CAMERIC_CSIS_CFG_FMT_RAW12,
		.data_alignment = 24,
	}
};

#define cameric_csis_write(__csis, __r, __v) writel(__v, __csis->regs + __r)
#define cameric_csis_read(__csis, __r) readl(__csis->regs + __r)

static struct csis_state *sd_to_csis_state(struct v4l2_subdev *sdev)
{
	return container_of(sdev, struct csis_state, sd);
}

static const struct csis_pix_format *find_csis_format(
	struct v4l2_mbus_framefmt *mf)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cameric_csis_formats); i++)
		if (mf->code == cameric_csis_formats[i].code)
			return &cameric_csis_formats[i];
	return NULL;
}

static void cameric_csis_enable_interrupts(struct csis_state *state, bool on)
{
	u32 val = cameric_csis_read(state, CAMERIC_CSIS_INTMSK);
	if (on)
		val |= state->interrupt_mask;
	else
		val &= ~state->interrupt_mask;
	cameric_csis_write(state, CAMERIC_CSIS_INTMSK, val);
}

static void cameric_csis_reset(struct csis_state *state)
{
	u32 val = cameric_csis_read(state, CAMERIC_CSIS_CTRL);

	cameric_csis_write(state, CAMERIC_CSIS_CTRL, val | CAMERIC_CSIS_CTRL_RESET);
	udelay(10);
}

static void cameric_csis_system_enable(struct csis_state *state, int on)
{
	u32 val, mask;

	val = cameric_csis_read(state, CAMERIC_CSIS_CTRL);
	if (on)
		val |= CAMERIC_CSIS_CTRL_ENABLE;
	else
		val &= ~CAMERIC_CSIS_CTRL_ENABLE;
	cameric_csis_write(state, CAMERIC_CSIS_CTRL, val);

	val = cameric_csis_read(state, CAMERIC_CSIS_DPHYCTRL);
	val &= ~CAMERIC_CSIS_DPHYCTRL_ENABLE;
	if (on) {
		mask = (1 << (state->num_lanes + 1)) - 1;
		val |= (mask & CAMERIC_CSIS_DPHYCTRL_ENABLE);
	}
	cameric_csis_write(state, CAMERIC_CSIS_DPHYCTRL, val);
}

/* Called with the state.lock mutex held */
static void __cameric_csis_set_format(struct csis_state *state)
{
	struct v4l2_mbus_framefmt *mf = &state->format;
	u32 val;

	v4l2_dbg(1, debug, &state->sd, "fmt: %#x, %d x %d\n",
		 mf->code, mf->width, mf->height);

	/* Color format */
	val = cameric_csis_read(state, CAMERIC_CSIS_CONFIG);
	val = (val & ~CAMERIC_CSIS_CFG_FMT_MASK) | state->csis_fmt->fmt_reg;
	cameric_csis_write(state, CAMERIC_CSIS_CONFIG, val);

	/* Pixel resolution */
	val = (mf->width << 16) | mf->height;
	cameric_csis_write(state, CAMERIC_CSIS_RESOL, val);
}

static void cameric_csis_set_hsync_settle(struct csis_state *state, int settle)
{
	u32 val = cameric_csis_read(state, CAMERIC_CSIS_DPHYCTRL);

	val = (val & ~CAMERIC_CSIS_DPHYCTRL_HSS_MASK) | (settle << 27);
	cameric_csis_write(state, CAMERIC_CSIS_DPHYCTRL, val);
}

static void cameric_csis_set_params(struct csis_state *state)
{
	u32 val;

	val = cameric_csis_read(state, CAMERIC_CSIS_CONFIG);
	val = (val & ~CAMERIC_CSIS_CFG_NR_LANE_MASK) | (state->num_lanes - 1);
	cameric_csis_write(state, CAMERIC_CSIS_CONFIG, val);

	__cameric_csis_set_format(state);
	cameric_csis_set_hsync_settle(state, state->hs_settle);

	val = cameric_csis_read(state, CAMERIC_CSIS_CTRL);
	if (state->csis_fmt->data_alignment == 32)
		val |= CAMERIC_CSIS_CTRL_ALIGN_32BIT;
	else /* 24-bits */
		val &= ~CAMERIC_CSIS_CTRL_ALIGN_32BIT;

	val &= ~CAMERIC_CSIS_CTRL_WCLK_EXTCLK;
	if (state->wclk_ext)
		val |= CAMERIC_CSIS_CTRL_WCLK_EXTCLK;
	cameric_csis_write(state, CAMERIC_CSIS_CTRL, val);

	/* Update the shadow register. */
	val = cameric_csis_read(state, CAMERIC_CSIS_CTRL);
	cameric_csis_write(state, CAMERIC_CSIS_CTRL, val | CAMERIC_CSIS_CTRL_UPDATE_SHADOW);
}

static void cameric_csis_clk_put(struct csis_state *state)
{
	int i;

	for (i = 0; i < NUM_CSIS_CLOCKS; i++) {
		if (IS_ERR(state->clock[i]))
			continue;
		clk_unprepare(state->clock[i]);
		clk_put(state->clock[i]);
		state->clock[i] = ERR_PTR(-EINVAL);
	}
}

static int cameric_csis_clk_get(struct csis_state *state)
{
	struct device *dev = &state->pdev->dev;
	int i, ret;

	for (i = 0; i < NUM_CSIS_CLOCKS; i++)
		state->clock[i] = ERR_PTR(-EINVAL);

	for (i = 0; i < NUM_CSIS_CLOCKS; i++) {
		state->clock[i] = clk_get(dev, csi_clock_name[i]);
		if (IS_ERR(state->clock[i])) {
			ret = PTR_ERR(state->clock[i]);
			goto err;
		}
		ret = clk_prepare(state->clock[i]);
		if (ret < 0) {
			clk_put(state->clock[i]);
			state->clock[i] = ERR_PTR(-EINVAL);
			goto err;
		}
	}
	return 0;
err:
	cameric_csis_clk_put(state);
	dev_err(dev, "failed to get clock: %s\n", csi_clock_name[i]);
	return ret;
}

static void dump_regs(struct csis_state *state, const char *label)
{
	struct {
		u32 offset;
		const char * const name;
	} registers[] = {
		{ 0x00, "CTRL" },
		{ 0x04, "DPHYCTRL" },
		{ 0x08, "CONFIG" },
		{ 0x0c, "DPHYSTS" },
		{ 0x10, "INTMSK" },
		{ 0x2c, "RESOL" },
		{ 0x38, "SDW_CONFIG" },
	};
	u32 i;

	v4l2_info(&state->sd, "--- %s ---\n", label);

	for (i = 0; i < ARRAY_SIZE(registers); i++) {
		u32 cfg = cameric_csis_read(state, registers[i].offset);
		v4l2_info(&state->sd, "%10s: 0x%08x\n", registers[i].name, cfg);
	}
}

static void cameric_csis_start_stream(struct csis_state *state)
{
	cameric_csis_reset(state);
	cameric_csis_set_params(state);
	cameric_csis_system_enable(state, true);
	cameric_csis_enable_interrupts(state, true);
}

static void cameric_csis_stop_stream(struct csis_state *state)
{
	cameric_csis_enable_interrupts(state, false);
	cameric_csis_system_enable(state, false);
}

static void cameric_csis_clear_counters(struct csis_state *state)
{
	unsigned long flags;
	int i;

	spin_lock_irqsave(&state->slock, flags);
	for (i = 0; i < CAMERIC_CSIS_NUM_EVENTS; i++)
		state->events[i].counter = 0;
	spin_unlock_irqrestore(&state->slock, flags);
}

static void cameric_csis_log_counters(struct csis_state *state, bool non_errors)
{
	int i = non_errors ? CAMERIC_CSIS_NUM_EVENTS : CAMERIC_CSIS_NUM_EVENTS - 4;
	unsigned long flags;

	spin_lock_irqsave(&state->slock, flags);

	for (i--; i >= 0; i--) {
		if (state->events[i].counter > 0 || debug)
			v4l2_info(&state->sd, "%s events: %d\n",
				  state->events[i].name,
				  state->events[i].counter);
	}
	spin_unlock_irqrestore(&state->slock, flags);
}

/*
 * V4L2 subdev operations
 */
static int cameric_csis_s_power(struct v4l2_subdev *sd, int on)
{
	struct csis_state *state = sd_to_csis_state(sd);
	struct device *dev = &state->pdev->dev;

	if (on)
		return pm_runtime_get_sync(dev);

	return pm_runtime_put_sync(dev);
}

static int cameric_csis_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct csis_state *state = sd_to_csis_state(sd);
	int ret = 0;

	v4l2_dbg(1, debug, sd, "%s: %d, state: 0x%x\n",
		 __func__, enable, state->flags);

	if (enable) {
		cameric_csis_clear_counters(state);
		ret = pm_runtime_get_sync(&state->pdev->dev);
		if (ret && ret != 1)
			return ret;
	}

	mutex_lock(&state->lock);
	if (enable) {
		if (state->flags & ST_SUSPENDED) {
			ret = -EBUSY;
			goto unlock;
		}
		cameric_csis_start_stream(state);
		state->flags |= ST_STREAMING;
	} else {
		cameric_csis_stop_stream(state);
		state->flags &= ~ST_STREAMING;
		if (debug > 0)
			cameric_csis_log_counters(state, true);
	}
unlock:
	mutex_unlock(&state->lock);
	if (!enable)
		pm_runtime_put(&state->pdev->dev);

	return ret == 1 ? 0 : ret;
}

static int cameric_csis_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(cameric_csis_formats))
		return -EINVAL;

	code->code = cameric_csis_formats[code->index].code;
	return 0;
}

static struct csis_pix_format const *cameric_csis_try_format(
	struct v4l2_mbus_framefmt *mf)
{
	struct csis_pix_format const *csis_fmt;

	csis_fmt = find_csis_format(mf);
	if (csis_fmt == NULL)
		csis_fmt = &cameric_csis_formats[0];

	mf->code = csis_fmt->code;
	v4l_bound_align_image(&mf->width, 1, CSIS_MAX_PIX_WIDTH,
			      csis_fmt->pix_width_alignment,
			      &mf->height, 1, CSIS_MAX_PIX_HEIGHT, 1,
			      0);
	return csis_fmt;
}

static struct v4l2_mbus_framefmt *__cameric_csis_get_format(
		struct csis_state *state, struct v4l2_subdev_pad_config *cfg,
		enum v4l2_subdev_format_whence which)
{
	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return cfg ? v4l2_subdev_get_try_format(&state->sd, cfg, 0) : NULL;

	return &state->format;
}

static int cameric_csis_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct csis_state *state = sd_to_csis_state(sd);
	struct csis_pix_format const *csis_fmt;
	struct v4l2_mbus_framefmt *mf;

	mf = __cameric_csis_get_format(state, cfg, fmt->which);

	if (fmt->pad == CSIS_PAD_SOURCE) {
		if (mf) {
			mutex_lock(&state->lock);
			fmt->format = *mf;
			mutex_unlock(&state->lock);
		}
		return 0;
	}
	csis_fmt = cameric_csis_try_format(&fmt->format);
	if (mf) {
		mutex_lock(&state->lock);
		*mf = fmt->format;
		if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
			state->csis_fmt = csis_fmt;
		mutex_unlock(&state->lock);
	}
	return 0;
}

static int cameric_csis_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct csis_state *state = sd_to_csis_state(sd);
	struct v4l2_mbus_framefmt *mf;

	mf = __cameric_csis_get_format(state, cfg, fmt->which);
	if (!mf)
		return -EINVAL;

	mutex_lock(&state->lock);
	fmt->format = *mf;
	mutex_unlock(&state->lock);
	return 0;
}

static int cameric_csis_s_rx_buffer(struct v4l2_subdev *sd, void *buf,
			       unsigned int *size)
{
	struct csis_state *state = sd_to_csis_state(sd);
	unsigned long flags;

	*size = min_t(unsigned int, *size, CAMERIC_CSIS_PKTDATA_SIZE);

	spin_lock_irqsave(&state->slock, flags);
	state->pkt_buf.data = buf;
	state->pkt_buf.len = *size;
	spin_unlock_irqrestore(&state->slock, flags);

	return 0;
}

static int cameric_csis_log_status(struct v4l2_subdev *sd)
{
	struct csis_state *state = sd_to_csis_state(sd);

	mutex_lock(&state->lock);
	cameric_csis_log_counters(state, true);
	if (debug && (state->flags & ST_POWERED))
		dump_regs(state, __func__);
	mutex_unlock(&state->lock);
	return 0;
}

static const struct v4l2_subdev_core_ops cameric_csis_core_ops = {
	.s_power = cameric_csis_s_power,
	.log_status = cameric_csis_log_status,
};

static const struct v4l2_subdev_pad_ops cameric_csis_pad_ops = {
	.enum_mbus_code = cameric_csis_enum_mbus_code,
	.get_fmt = cameric_csis_get_fmt,
	.set_fmt = cameric_csis_set_fmt,
};

static const struct v4l2_subdev_video_ops cameric_csis_video_ops = {
	.s_rx_buffer = cameric_csis_s_rx_buffer,
	.s_stream = cameric_csis_s_stream,
};

static const struct v4l2_subdev_ops cameric_csis_subdev_ops = {
	.core = &cameric_csis_core_ops,
	.pad = &cameric_csis_pad_ops,
	.video = &cameric_csis_video_ops,
};

static irqreturn_t cameric_csis_irq_handler(int irq, void *dev_id)
{
	struct csis_state *state = dev_id;
	struct csis_pktbuf *pktbuf = &state->pkt_buf;
	unsigned long flags;
	u32 status;

	status = cameric_csis_read(state, CAMERIC_CSIS_INTSRC);
	spin_lock_irqsave(&state->slock, flags);

	if ((status & CAMERIC_CSIS_INTSRC_NON_IMAGE_DATA) && pktbuf->data) {
		u32 offset;

		if (status & CAMERIC_CSIS_INTSRC_EVEN)
			offset = CAMERIC_CSIS_PKTDATA_EVEN;
		else
			offset = CAMERIC_CSIS_PKTDATA_ODD;

		memcpy(pktbuf->data, (u8 __force *)state->regs + offset,
		       pktbuf->len);
		pktbuf->data = NULL;
		rmb();
	}

	/* Update the event/error counters */
	if ((status & CAMERIC_CSIS_INTSRC_ERRORS) || debug) {
		int i;
		for (i = 0; i < CAMERIC_CSIS_NUM_EVENTS; i++) {
			if (!(status & state->events[i].mask))
				continue;
			state->events[i].counter++;
			v4l2_dbg(2, debug, &state->sd, "%s: %d\n",
				 state->events[i].name,
				 state->events[i].counter);
		}
		v4l2_dbg(2, debug, &state->sd, "status: %08x\n", status);
	}
	spin_unlock_irqrestore(&state->slock, flags);

	cameric_csis_write(state, CAMERIC_CSIS_INTSRC, status);
	return IRQ_HANDLED;
}

static int cameric_csis_parse_dt(struct platform_device *pdev,
			    struct csis_state *state)
{
	struct device_node *node = pdev->dev.of_node;
	struct v4l2_of_endpoint endpoint;
	int ret;

	if (of_property_read_u32(node, "clock-frequency",
				 &state->clk_frequency))
		state->clk_frequency = DEFAULT_SCLK_CSIS_FREQ;
	if (of_property_read_u32(node, "bus-width",
				 &state->max_num_lanes))
		return -EINVAL;

	node = of_graph_get_next_endpoint(node, NULL);
	if (!node) {
		dev_err(&pdev->dev, "No port node at %s\n",
				pdev->dev.of_node->full_name);
		return -EINVAL;
	}
	/* Get port node and validate MIPI-CSI channel id. */
	ret = v4l2_of_parse_endpoint(node, &endpoint);
	if (ret)
		goto err;

	state->index = endpoint.base.port - CAMERIC_INPUT_MIPI_CSI2_0;
	if (state->index >= CSIS_MAX_ENTITIES) {
		ret = -ENXIO;
		goto err;
	}

	/* Get MIPI CSI-2 bus configration from the endpoint node. */
	of_property_read_u32(node, "samsung,csis-hs-settle",
					&state->hs_settle);
	state->wclk_ext = of_property_read_bool(node,
					"samsung,csis-wclk");

	state->num_lanes = endpoint.bus.mipi_csi2.num_data_lanes;

err:
	of_node_put(node);
	return ret;
}

static int cameric_csis_pm_resume(struct device *dev, bool runtime);
static const struct of_device_id cameric_csis_of_match[];

static int cameric_csis_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id;
	const struct csis_drvdata *drv_data;
	struct device *dev = &pdev->dev;
	struct resource *mem_res;
	struct csis_state *state;
	int ret = -ENOMEM;
	int i;
	printk(KERN_INFO "%s \n", __func__);
	state = devm_kzalloc(dev, sizeof(*state), GFP_KERNEL);
	if (!state)
		return -ENOMEM;

	mutex_init(&state->lock);
	spin_lock_init(&state->slock);
	state->pdev = pdev;

	of_id = of_match_node(cameric_csis_of_match, dev->of_node);
	if (WARN_ON(of_id == NULL))
		return -EINVAL;

	drv_data = of_id->data;
	state->interrupt_mask = drv_data->interrupt_mask;

	ret = cameric_csis_parse_dt(pdev, state);
	if (ret < 0)
		return ret;

	if (state->num_lanes == 0 || state->num_lanes > state->max_num_lanes) {
		dev_err(dev, "Unsupported number of data lanes: %d (max. %d)\n",
			state->num_lanes, state->max_num_lanes);
		return -EINVAL;
	}

	state->phy = devm_phy_get(dev, "csis");
	if (IS_ERR(state->phy))
		return PTR_ERR(state->phy);

	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	state->regs = devm_ioremap_resource(dev, mem_res);
	if (IS_ERR(state->regs))
		return PTR_ERR(state->regs);

	state->irq = platform_get_irq(pdev, 0);
	if (state->irq < 0) {
		dev_err(dev, "Failed to get irq\n");
		return state->irq;
	}

	for (i = 0; i < CSIS_NUM_SUPPLIES; i++)
		state->supplies[i].supply = csis_supply_name[i];

	ret = devm_regulator_bulk_get(dev, CSIS_NUM_SUPPLIES,
				 state->supplies);
	if (ret)
		return ret;

	ret = cameric_csis_clk_get(state);
	if (ret < 0)
		return ret;

	if (state->clk_frequency)
		ret = clk_set_rate(state->clock[CSIS_CLK_MUX],
				   state->clk_frequency);
	else
		dev_WARN(dev, "No clock frequency specified!\n");
	if (ret < 0)
		goto e_clkput;

	ret = clk_enable(state->clock[CSIS_CLK_MUX]);
	if (ret < 0)
		goto e_clkput;

	ret = devm_request_irq(dev, state->irq, cameric_csis_irq_handler,
			       0, dev_name(dev), state);
	if (ret) {
		dev_err(dev, "Interrupt request failed\n");
		goto e_clkdis;
	}

	v4l2_subdev_init(&state->sd, &cameric_csis_subdev_ops);
	state->sd.owner = THIS_MODULE;
	snprintf(state->sd.name, sizeof(state->sd.name), "%s.%d",
		 CSIS_SUBDEV_NAME, state->index);
	state->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	state->csis_fmt = &cameric_csis_formats[0];

	state->format.code = cameric_csis_formats[0].code;
	state->format.width = CAMERIC_CSIS_DEF_PIX_WIDTH;
	state->format.height = CAMERIC_CSIS_DEF_PIX_HEIGHT;

	state->sd.entity.function = MEDIA_ENT_F_IO_V4L;
	state->pads[CSIS_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	state->pads[CSIS_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&state->sd.entity,
				CSIS_PADS_NUM, state->pads);
	if (ret < 0)
		goto e_clkdis;

	/* This allows to retrieve the platform device id by the host driver */
	v4l2_set_subdevdata(&state->sd, pdev);

	/* .. and a pointer to the subdev. */
	platform_set_drvdata(pdev, &state->sd);
	memcpy(state->events, cameric_csis_events, sizeof(state->events));

	pm_runtime_enable(dev);
	if (!pm_runtime_enabled(dev)) {
		ret = cameric_csis_pm_resume(dev, true);
		if (ret < 0)
			goto e_m_ent;
	}

	dev_info(&pdev->dev, "lanes: %d, hs_settle: %d, wclk: %d, freq: %u\n",
		 state->num_lanes, state->hs_settle, state->wclk_ext,
		 state->clk_frequency);
	return 0;

e_m_ent:
	media_entity_cleanup(&state->sd.entity);
e_clkdis:
	clk_disable(state->clock[CSIS_CLK_MUX]);
e_clkput:
	cameric_csis_clk_put(state);
	return ret;
}

static int cameric_csis_pm_suspend(struct device *dev, bool runtime)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct v4l2_subdev *sd = platform_get_drvdata(pdev);
	struct csis_state *state = sd_to_csis_state(sd);
	int ret = 0;
	printk(KERN_INFO "%s \n", __func__);
	v4l2_dbg(1, debug, sd, "%s: flags: 0x%x\n",
		 __func__, state->flags);

	mutex_lock(&state->lock);
	if (state->flags & ST_POWERED) {
		cameric_csis_stop_stream(state);
		ret = phy_power_off(state->phy);
		if (ret)
			goto unlock;
		ret = regulator_bulk_disable(CSIS_NUM_SUPPLIES,
					     state->supplies);
		if (ret)
			goto unlock;
		clk_disable(state->clock[CSIS_CLK_GATE]);
		state->flags &= ~ST_POWERED;
		if (!runtime)
			state->flags |= ST_SUSPENDED;
	}
 unlock:
	mutex_unlock(&state->lock);
	return ret ? -EAGAIN : 0;
}

static int cameric_csis_pm_resume(struct device *dev, bool runtime)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct v4l2_subdev *sd = platform_get_drvdata(pdev);
	struct csis_state *state = sd_to_csis_state(sd);
	int ret = 0;
	printk(KERN_INFO "%s \n", __func__);
	v4l2_dbg(1, debug, sd, "%s: flags: 0x%x\n",
		 __func__, state->flags);

	mutex_lock(&state->lock);
	if (!runtime && !(state->flags & ST_SUSPENDED))
		goto unlock;

	if (!(state->flags & ST_POWERED)) {
		ret = regulator_bulk_enable(CSIS_NUM_SUPPLIES,
					    state->supplies);
		if (ret)
			goto unlock;
		ret = phy_power_on(state->phy);
		if (!ret) {
			state->flags |= ST_POWERED;
		} else {
			regulator_bulk_disable(CSIS_NUM_SUPPLIES,
					       state->supplies);
			goto unlock;
		}
		clk_enable(state->clock[CSIS_CLK_GATE]);
	}
	if (state->flags & ST_STREAMING)
		cameric_csis_start_stream(state);

	state->flags &= ~ST_SUSPENDED;
 unlock:
	mutex_unlock(&state->lock);
	return ret ? -EAGAIN : 0;
}

#ifdef CONFIG_PM_SLEEP
static int cameric_csis_suspend(struct device *dev)
{printk(KERN_INFO "%s \n", __func__);
	return cameric_csis_pm_suspend(dev, false);
}

static int cameric_csis_resume(struct device *dev)
{printk(KERN_INFO "%s \n", __func__);
	return cameric_csis_pm_resume(dev, false);
}
#endif

#ifdef CONFIG_PM
static int cameric_csis_runtime_suspend(struct device *dev)
{printk(KERN_INFO "%s \n", __func__);
	return cameric_csis_pm_suspend(dev, true);
}

static int cameric_csis_runtime_resume(struct device *dev)
{printk(KERN_INFO "%s \n", __func__);
	return cameric_csis_pm_resume(dev, true);
}
#endif

static int cameric_csis_remove(struct platform_device *pdev)
{
	struct v4l2_subdev *sd = platform_get_drvdata(pdev);
	struct csis_state *state = sd_to_csis_state(sd);
	printk(KERN_INFO "%s \n", __func__);
	pm_runtime_disable(&pdev->dev);
	cameric_csis_pm_suspend(&pdev->dev, true);
	clk_disable(state->clock[CSIS_CLK_MUX]);
	pm_runtime_set_suspended(&pdev->dev);
	cameric_csis_clk_put(state);

	media_entity_cleanup(&state->sd.entity);

	return 0;
}

static const struct dev_pm_ops cameric_csis_pm_ops = {
	SET_RUNTIME_PM_OPS(cameric_csis_runtime_suspend, cameric_csis_runtime_resume,
			   NULL)
	SET_SYSTEM_SLEEP_PM_OPS(cameric_csis_suspend, cameric_csis_resume)
};

static const struct csis_drvdata cameric_rk3288_csis_drvdata = {
	.interrupt_mask = CAMERIC_CSIS_INTMSK_RK3288_EN_ALL,
};

static const struct csis_drvdata cameric_rk3399_csis_drvdata = {
	.interrupt_mask = CAMERIC_CSIS_INTMSK_RK3399_EN_ALL,
};

static const struct of_device_id cameric_csis_of_match[] = {
	{
		.compatible = "rockchip,rk3288-csis",
		.data = &cameric_rk3288_csis_drvdata,
	}, {
		.compatible = "rockchip,rk3399-csis",
		.data = &cameric_rk3399_csis_drvdata,
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, cameric_csis_of_match);

static struct platform_driver cameric_csis_driver = {
	.probe		= cameric_csis_probe,
	.remove		= cameric_csis_remove,
	.driver		= {
		.of_match_table = cameric_csis_of_match,
		.name		= CSIS_DRIVER_NAME,
		.pm		= &cameric_csis_pm_ops,
	},
};

module_platform_driver(cameric_csis_driver);

MODULE_AUTHOR("Sylwester Nawrocki <s.nawrocki@samsung.com>");
MODULE_DESCRIPTION("Samsung CAMERIC_/EXYNOS SoC MIPI-CSI2 receiver driver");
MODULE_LICENSE("GPL");
