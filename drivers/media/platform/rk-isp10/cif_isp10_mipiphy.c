/*
 * Rockchip MIPI PHY driver
 *
 * Copyright (C) 2017 Fuzhou Rockchip Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <media/media-entity.h>
#include <media/v4l2-subdev.h>

#include "cif_isp10_mipiphy.h"

//TODO(zsq) this too
#define rkisp_remote_subdev_call(pad, o, f, args...) ({		\
	int __ret;						\
	do {							\
		struct v4l2_subdev *r_sd;			\
		struct media_pad *r_pad;			\
		struct media_entity *r_entity;			\
								\
		r_pad = media_entity_remote_pad(pad);		\
		r_entity = r_pad->entity;			\
		r_sd = media_entity_to_v4l2_subdev(r_entity);	\
		__ret = v4l2_subdev_call(r_sd, o, f, args);	\
		if (__ret)					\
			v4l2_err(r_sd, "Failed to call" #f "(%d)\n", __ret); \
	} while(0);						\
	__ret;})

enum reg_name {
	LANES_ENALBE = 0,
	FORCE_RX_MODE,
	FORCE_TX_STOP_MODE,
	TURN_AROUND_DISABLE,
	TURN_AROUND_REQUEST,
	TEST_DIN,
	TEST_ENABLE,
	TEST_CLOCK,
	TEST_CLEAR,
	TEST_DOUT,
};

struct phy_reg {
	u32 name;	/* enum reg_name */
	u32 reg;	/* reg offset addr */
	u32 bitmask;
	u32 shift;
};

struct phy_reg rk3399_rx0_regs[] = {
	{
		.name		= LANES_ENALBE,
		.reg		= 0x6254, 	/* SOC_CON21 */
		.bitmask	= GENMASK(3, 0),
		.shift		= 0,
	}, {
		.name		= FORCE_RX_MODE,
		.reg		= 0x6254, 	/* SOC_CON21 */
		.bitmask	= GENMASK(7, 4),
		.shift		= 4,
	}, {
		.name		= FORCE_TX_STOP_MODE,
		.reg		= 0x6254, 	/* SOC_CON21 */
		.bitmask	= GENMASK(11, 8),
		.shift		= 8,
	}, {
		.name		= TURN_AROUND_DISABLE,
		.reg		= 0x6254, 	/* SOC_CON21 */
		.bitmask	= GENMASK(15, 12),
		.shift		= 12,
	}, {
		.name		= TURN_AROUND_REQUEST,
		.reg		= 0x6224, 	/* SOC_CON9 */
		.bitmask	= GENMASK(3, 0),
		.shift		= 0,
	}, {
		.name		= TEST_DIN,
		.reg		= 0x6264, 	/* SOC_CON25 */
		.bitmask	= GENMASK(7, 0),
		.shift		= 0,
	}, {
		.name		= TEST_ENABLE,
		.reg		= 0x6264, 	/* SOC_CON25 */
		.bitmask	= GENMASK(8, 8),
		.shift		= 8,
	}, {
		.name		= TEST_CLOCK,
		.reg		= 0x6264, 	/* SOC_CON25 */
		.bitmask	= GENMASK(9, 9),
		.shift		= 9,
	}, {
		.name		= TEST_CLEAR,
		.reg		= 0x6264, 	/* SOC_CON25 */
		.bitmask	= GENMASK(10, 10),
		.shift		= 10,
	}, {
		.name		= TEST_DOUT,
		.reg		= 0xe2a4, 	/* SOC_STATUS1 */
		.bitmask	= GENMASK(7, 0),
		.shift		= 0,
	},
};

struct mipiphy_hsfreqrange {
	u32 range_h;
	u8 cfg_bit;
};

static struct mipiphy_hsfreqrange mipi_phy_hsfreq_range[] = {
	{  90, 0x00}, { 100, 0x10}, { 110, 0x20}, { 130, 0x01},
	{ 140, 0x11}, { 150, 0x21}, { 170, 0x02}, { 180, 0x12},
	{ 200, 0x22}, { 220, 0x03}, { 240, 0x13}, { 250, 0x23},
	{ 270, 0x04}, { 300, 0x14}, { 330, 0x05}, { 360, 0x15},
	{ 400, 0x25}, { 450, 0x06}, { 500, 0x16}, { 550, 0x07},
	{ 600, 0x17}, { 650, 0x08}, { 700, 0x18}, { 750, 0x09},
	{ 800, 0x19}, { 850, 0x29}, { 900, 0x39}, { 950, 0x0a},
	{1000, 0x1a}, {1050, 0x2a}, {1100, 0x3a}, {1150, 0x0b},
	{1200, 0x1b}, {1250, 0x2b}, {1300, 0x3b}, {1350, 0x0c},
	{1400, 0x1c}, {1450, 0x2c}, {1500, 0x3c}
};

#define MIPIPHY_STATE_OFF 0
#define MIPIPHY_STATE_POWERD 1
#define MIPIPHY_STATE_STREAMING 2

struct mipiphy_priv {
	struct device		*dev;
	struct regmap		*regmap_grf;
	struct phy_reg		*regs;
	struct clk		*clk_phy_ref;
	struct clk		*clk_phy_cfg;
	u32			lanes;
	u32			bit_rate;

	int			state;
	struct v4l2_subdev	sd;
	struct media_pad	pads[MIPIPHY_PADS_NUM];
};
#define to_mipiphy(subdev) container_of(subdev, struct mipiphy_priv, sd)

static inline void write_reg(struct regmap *base, struct phy_reg *preg, u8 val)
{
	regmap_write(base, preg->reg, val << preg->shift | preg->bitmask << 16);
}

static void mipiphy_wr_reg(struct mipiphy_priv *priv,
			  u8 test_code, u8 test_data)
{
	struct phy_reg *regs = priv->regs;
	struct regmap *grf = priv->regmap_grf;

	/*
	 * With the falling edge on TESTCLK, the TESTDIN[7:0] signal content
	 * is latched internally as the current test code. Test data is
	 * programmed internally by rising edge on TESTCLK.
	 */
	write_reg(grf, &regs[TEST_CLOCK], 1);
	write_reg(grf, &regs[TEST_DIN], test_code);
	write_reg(grf, &regs[TEST_ENABLE], 1);
	write_reg(grf, &regs[TEST_CLOCK], 0);
	write_reg(grf, &regs[TEST_ENABLE], 0);

	write_reg(grf, &regs[TEST_DIN], test_data);
	write_reg(grf, &regs[TEST_CLOCK], 1);
}

static int mipiphy_s_stream(struct v4l2_subdev *sd, int on)
{
	struct mipiphy_priv *priv = to_mipiphy(sd);
	struct regmap *grf = priv->regmap_grf;
	struct phy_reg *regs = priv->regs;
	int hsfreqrange, i;
	int lanes = priv->lanes;
	int bit_rate = priv->bit_rate;

	if (!on) {
		if (priv->state == MIPIPHY_STATE_STREAMING) {
			priv->state = MIPIPHY_STATE_POWERD; //TODO(zsq): to stop streaming
		}

		return 0;
	}

	if (priv->state == MIPIPHY_STATE_STREAMING)
		return 0;

	hsfreqrange = 0;
	for (i = 0; i < ARRAY_SIZE(mipi_phy_hsfreq_range); i++) {
		if (mipi_phy_hsfreq_range[i].range_h > bit_rate) {
			hsfreqrange = mipi_phy_hsfreq_range[i].cfg_bit;
			break;
		}
	}

	write_reg(grf, &regs[FORCE_RX_MODE], 0);
	write_reg(grf, &regs[FORCE_TX_STOP_MODE], 0);
	/* Disable lan turn around, which is ignored in receive mode */
	write_reg(grf, &regs[TURN_AROUND_REQUEST], 0);
	write_reg(grf, &regs[TURN_AROUND_DISABLE], 0xf);

	write_reg(grf, &regs[LANES_ENALBE], GENMASK(lanes - 1, 0));

	/* phy start */
	write_reg(grf, &regs[TEST_CLOCK], 1);
	write_reg(grf, &regs[TEST_CLEAR], 1);
	usleep_range(100, 150);
	write_reg(grf, &regs[TEST_CLEAR], 0);
	usleep_range(100, 150);

	/* set clock lane */
	/* HS hsfreqrange & lane 0  settle bypass */
	mipiphy_wr_reg(priv, 0x34, 0);
	mipiphy_wr_reg(priv, 0x44, hsfreqrange << 1); // HS RX lane0
	mipiphy_wr_reg(priv, 0x54, 0); // HS RX lane1
	mipiphy_wr_reg(priv, 0x84, 0); // HS RX lane2
	mipiphy_wr_reg(priv, 0x94, 0); // HS RX lane3
	mipiphy_wr_reg(priv, 0x75, 0x04);

	/* Normal operation */
	mipiphy_wr_reg(priv, 0x0, 0);

	printk("ZSQ : %s/%d\n", __func__, __LINE__);
	priv->state = MIPIPHY_STATE_STREAMING;

	return 0;
}

static int mipiphy_s_power(struct v4l2_subdev *sd, int on)
{
	struct mipiphy_priv *priv = to_mipiphy(sd);
	int ret;

	priv->lanes = 2; //TODO(zsq): how to associate with sensor?
	priv->bit_rate = 1000; //TODO(zsq): as above, Mbps

	if (on) {
		if (priv->state > MIPIPHY_STATE_OFF)
			return 0;

		ret = clk_prepare_enable(priv->clk_phy_cfg);
		if (ret) {
			v4l2_err(&priv->sd, "Fail to enable phy_cfg clock\n");
			return ret;
		}
		ret = clk_prepare_enable(priv->clk_phy_ref);
		if (ret) {
			v4l2_err(&priv->sd, "Fail to enable phy_ref clock\n");
			clk_disable_unprepare(priv->clk_phy_cfg);
			return ret;
		}

		priv->state = MIPIPHY_STATE_POWERD;
	} else {
		if (priv->state == MIPIPHY_STATE_OFF)
			return 0;

		clk_disable_unprepare(priv->clk_phy_ref);
		clk_disable_unprepare(priv->clk_phy_cfg);
		priv->state = MIPIPHY_STATE_OFF;
	}
	printk("ZSQ : %s/%d\n", __func__, __LINE__);

	return 0;
}

static struct v4l2_subdev_core_ops mipiphy_core_ops = {
	.s_power = mipiphy_s_power,
};

static struct v4l2_subdev_video_ops mipiphy_video_ops = {
	.s_stream = mipiphy_s_stream,
};

static struct v4l2_subdev_ops mipiphy_subdev_ops = {
	.core	= &mipiphy_core_ops,
	.video	= &mipiphy_video_ops,
};

static const struct of_device_id rockchip_mipiphy_match_id[] = {
	{
		.compatible = "rockchip,rk3399-isp-mipi-phy",
		.data = &rk3399_rx0_regs,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, rockchip_mipiphy_match_id);

static int rockchip_mipiphy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct v4l2_subdev *sd;
	struct mipiphy_priv *priv;
	struct regmap *grf;
	const struct of_device_id *of_id;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		return -ENOMEM;
	}
	priv->dev = dev;
	priv->state = MIPIPHY_STATE_OFF;

	grf = syscon_regmap_lookup_by_phandle(dev->of_node, "rockchip,grf");
	if (IS_ERR(grf)) {
		dev_err(dev, "Can't find GRF syscon\n");
		return -ENODEV;
	}
	priv->regmap_grf = grf;

	priv->clk_phy_ref = devm_clk_get(dev, "phy-ref");
	if (IS_ERR(priv->clk_phy_ref)) {
		dev_err(dev, "Unable to get phy-ref clock\n");
		return -EINVAL;
	}
	priv->clk_phy_cfg = devm_clk_get(dev, "phy-cfg");
	if (IS_ERR(priv->clk_phy_cfg)) {
		dev_err(dev, "Unable to phy-cfg clock\n");
		return -EINVAL;
	}

	of_id = of_match_device(rockchip_mipiphy_match_id, dev);
	if (!of_id)
		return -EINVAL;
	priv->regs = (struct phy_reg *)of_id->data;

	sd = &priv->sd;
	v4l2_subdev_init(sd, &mipiphy_subdev_ops);
	snprintf(sd->name, sizeof(sd->name), "rkisp-mipiphy");
	sd->dev = dev;

	priv->pads[MIPIPHY_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	priv->pads[MIPIPHY_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_init(&sd->entity, MIPIPHY_PADS_NUM, priv->pads, 0);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, &sd->entity);

	dev_info(dev, "RK3399 mipiphy probed\n");

	return 0;
}

static struct platform_driver rockchip_isp_mipiphy_driver = {
	.probe		= rockchip_mipiphy_probe,
	//TODO remove
	.driver		= {
		.name	= "rockchip-isp-mipi-phy",
		.of_match_table = rockchip_mipiphy_match_id,
	},
};

module_platform_driver(rockchip_isp_mipiphy_driver);

MODULE_AUTHOR("Rockchip Camera/ISP team");
MODULE_DESCRIPTION("Rockchip ISP MIPI PHY driver");
MODULE_LICENSE("GPL v2");
