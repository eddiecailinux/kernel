/*
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <media/v4l2-subdev.h>


#include "ov_camera_module.h"

struct ov5695_priv {
	struct v4l2_subdev	subdev;
	struct clk		*mclk;
	struct regulator	*supply;
	struct gpio_desc	*reset_gpio;
};
#define to_ov5695(sd) container_of(sd, struct ov5695_priv, subdev)

static struct ov_camera_module ov5695;
static struct ov_camera_module_custom_config ov5695_custom_config;

static int ov5695_read_reg(struct i2c_client *client, u16 reg, u8 *val);
static ssize_t ov5695_dump_registers(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	u8 val;

	ov5695_read_reg(client, 0x300b, &val);
	return sprintf(buf, "nothing\n");
}

static DEVICE_ATTR(dump_registers, S_IRUSR, ov5695_dump_registers, NULL);

static struct attribute *ov5695_attributes[] = {
	&dev_attr_dump_registers.attr,
	NULL
};

static struct attribute_group ov5695_attribute_group = {
	.attrs = ov5695_attributes,
};

static struct ov_camera_module_reg
	ov5695_init_tab_1296_972[] =
{
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0103, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0100, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0300, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0301, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0302, 0x69},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0303, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0304, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0305, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0307, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x030b, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x030c, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x030d, 0x1e},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x030e, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x030f, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0312, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3000, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3002, 0x21},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3022, 0x51},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3106, 0x15},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3107, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3108, 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3500, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3501, 0x45},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3502, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3503, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3504, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3505, 0x8c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3507, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3508, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3509, 0x10},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x350c, 0x00},
	//{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3500, 0x00},
	//{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3501, 0x3d},
	//{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3502, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x350d, 0x80},
	//{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x350a, 0x00},
	//{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x350b, 0x40},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3510, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3511, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3512, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3601, 0x55},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3602, 0x58},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3614, 0x30},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3615, 0x77},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3621, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3624, 0x40},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3633, 0x0c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3634, 0x0c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3635, 0x0c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3636, 0x0c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3638, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3639, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x363a, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x363b, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x363c, 0xff},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x363d, 0xfa},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3650, 0x44},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3651, 0x44},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3652, 0x44},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3653, 0x44},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3654, 0x44},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3655, 0x44},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3656, 0x44},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3657, 0x44},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3660, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3661, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3662, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x366a, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x366e, 0x0c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3673, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3700, 0x14},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3703, 0x0c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3715, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3733, 0x10},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3734, 0x40},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x373f, 0xa0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3765, 0x20},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x37a1, 0x1d},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x37a8, 0x26},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x37ab, 0x14},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x37c2, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x37cb, 0x09},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x37cc, 0x13},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x37cd, 0x1f},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x37ce, 0x1f},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3800, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3801, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3802, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3803, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3804, 0x0a},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3805, 0x3f},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3806, 0x07},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3807, 0xaf},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3808, 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3809, 0x10},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380a, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380b, 0xcc},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380c, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380d, 0xa0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380e, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380f, 0xb8},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3810, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3811, 0x06},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3812, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3813, 0x06},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3814, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3815, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3816, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3817, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3818, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3819, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x381a, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x381b, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3820, 0x8b},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3821, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c80, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c82, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c83, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c88, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3d85, 0x14},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3f02, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3f03, 0x10},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4008, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4009, 0x09},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x404e, 0x20},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4501, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4502, 0x10},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4800, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x481f, 0x2a},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4837, 0x13},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5000, 0x17},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5780, 0x3e},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5781, 0x0f},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5782, 0x44},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5783, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5784, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5785, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5786, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5787, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5788, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5789, 0x0f},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x578a, 0xfd},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x578b, 0xf5},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x578c, 0xf5},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x578d, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x578e, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x578f, 0x0c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5790, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5791, 0x06},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5792, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5793, 0x52},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5794, 0xa3},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5b00, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5b01, 0x1c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5b02, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5b03, 0x7f},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5b05, 0x6c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5e10, 0xfc},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4010, 0xf1},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3503, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3505, 0x8c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3507, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3508, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3509, 0xf8},
	//{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3811, 0x06},
	//{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5b03, 0xf0},
	//{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0100, 0x01},

	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5301, 0x45},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x366e, 0x0c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3800, 0x00}, // xstart = 0
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3801, 0x00}, // x start
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3802, 0x00}, // y start = 0
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3803, 0x00}, // y start
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3804, 0x0a}, // xend = 2623
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3805, 0x3f}, // xend
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3806, 0x07}, // yend = 1955
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3807, 0xaf}, // yend
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3808, 0x05}, // x output size = 1296
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3809, 0x10}, // x output size
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380a, 0x03}, // y output size = 968
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380b, 0xcc}, // y output size
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380c, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380d, 0xa0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380e, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380f, 0xb8},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3811, 0x06}, // isp x win
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3813, 0x06}, // isp y win
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3814, 0x03}, // x inc
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3816, 0x03}, // y inc
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3817, 0x01}, // hsync start
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3820, 0x8b}, // flip off, v bin off
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3821, 0x01}, // mirror on, h bin on
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4501, 0x00}, // black line number
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4008, 0x02}, // blc level trigger
	//{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x350b, 0x80}, // gain = 8x
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4009, 0x09}, // MIPI global timing
	//{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0100, 0x01},

};

static struct ov_camera_module_config ov5695_configs[] = {
	{
		.name = "1296x972",
		.frm_fmt = {
			.width = 1296,
			.height = 972,
			.code = MEDIA_BUS_FMT_SBGGR10_1X10
		},
		.frm_intrvl = {
			.interval = {
				.numerator = 1,
				.denominator = 18
			}
		},
		.auto_exp_enabled = false,
		.auto_gain_enabled = false,
		.auto_wb_enabled = false,
		.reg_table = (void *)ov5695_init_tab_1296_972,
		.reg_table_num_entries =
			sizeof(ov5695_init_tab_1296_972) /
			sizeof(ov5695_init_tab_1296_972[0]),
		.reg_diff_table = NULL,
		.reg_diff_table_num_entries = 0,
		.v_blanking_time_us = 7251,
		PLTFRM_CAM_ITF_MIPI_CFG(0, 2, 840, 12000000)
	},
};

static int ov5695_read_reg(struct i2c_client *client, u16 reg, u8 *val)
{
	struct i2c_msg msg[2];
	u8 buf[2];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xFF;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = 1;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret >= 0) {
		*val = buf[0];
		dev_info(&client->dev, "Read register 0x%4x, value 0x%2x\n", reg, *val);
		return 0;
	}

	dev_info(&client->dev, "read reg(0x%x val:0x%x) failed !\n", reg, *val);

	return ret;
}

static int ov5695_s_power(struct ov5695_priv *priv, int on)
{
	int ret;

	if (on) {
		clk_prepare_enable(priv->mclk);
		ret = regulator_enable(priv->supply);
		mdelay(1);
		gpiod_set_value_cansleep(priv->reset_gpio, 1);
		mdelay(1);
		gpiod_set_value_cansleep(priv->reset_gpio, 0);
		mdelay(1); /* Wait for modules settle */
	} else {
		clk_disable_unprepare(priv->mclk);
		ret = regulator_disable(priv->supply);
	}

	return 0;
}

static int ov5695_g_VTS(struct ov_camera_module *cam_mod, u32 *vts)
{
	return 0;
}

#if 0
int update_awb_gain(struct ov_camera_module *cam_mod)
{
	return 0;
}

int update_lenc(struct ov_camera_module *cam_mod)
{
	return 0;
}
#endif

static int ov5695_start_streaming(struct ov_camera_module *cam_mod)
{
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod,
		"active config=%s\n", cam_mod->active_config->name);

#if 0
	if (otp_ptr && otp_ptr->otp_en == 1 &&
	cam_mod->update_config &&
	cam_mod->active_config->soft_reset) {
		ov_camera_module_pr_debug(cam_mod,
					"apply otp data for R2A module...\n");
		update_awb_gain(cam_mod);
		update_lenc(cam_mod);
	}
#endif
	ret = ov5695_g_VTS(cam_mod, &cam_mod->vts_min);
	if (IS_ERR_VALUE(ret))
		goto err;

	if (IS_ERR_VALUE(ov_camera_module_write_reg(cam_mod, 0x0100, 1)))
		goto err;

	msleep(25);

	return 0;
err:
	ov_camera_module_pr_err(cam_mod, "failed with error (%d)\n",
		ret);
	return ret;
}

static int ov5695_stop_streaming(struct ov_camera_module *cam_mod)
{
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod, "\n");

	ret = ov_camera_module_write_reg(cam_mod, 0x0100, 0);
	if (IS_ERR_VALUE(ret))
		goto err;

	msleep(25);

	return 0;
err:
	ov_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}

static int ov5695_g_ctrl(struct ov_camera_module *cam_mod, u32 ctrl_id)
{
	return 0;
}

static int ov5695_s_ctrl(struct ov_camera_module *cam_mod, u32 ctrl_id)
{
	return 0;
}

static struct v4l2_subdev_core_ops ov5695_core_ops = {
	.g_ctrl = ov_camera_module_g_ctrl,
	.s_ctrl = ov_camera_module_s_ctrl,
	.s_ext_ctrls = ov_camera_module_s_ext_ctrls,
	.s_power = ov_camera_module_s_power,
	.ioctl = ov_camera_module_ioctl,
};

static struct v4l2_subdev_video_ops ov5695_video_ops = {
	.s_frame_interval = ov_camera_module_s_frame_interval,
	.s_stream = ov_camera_module_s_stream
};

static struct v4l2_subdev_pad_ops ov5695_pad_ops = {
	.enum_frame_interval = ov_camera_module_enum_frameintervals,
	.get_fmt = ov_camera_module_g_fmt,
	.set_fmt = ov_camera_module_s_fmt,
};

static struct v4l2_subdev_ops ov5695_subdev_ops = {
	.core	= &ov5695_core_ops,
	.video	= &ov5695_video_ops,
	.pad	= &ov5695_pad_ops,
};

static struct ov_camera_module_custom_config ov5695_custom_config = {
	.start_streaming = ov5695_start_streaming,
	.stop_streaming = ov5695_stop_streaming,
	.s_ctrl = ov5695_s_ctrl,
	.g_ctrl = ov5695_g_ctrl,
	.configs = ov5695_configs,
	.num_configs = ARRAY_SIZE(ov5695_configs),
	.power_up_delays_ms = {5, 20, 0}
};

static int ov5695_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct ov5695_priv *priv;
	u8 value;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(dev, "Failed to allocate private data!\n");
		return -ENOMEM;
	}

	priv->mclk = devm_clk_get(dev, "clk_cif_out");
	if (IS_ERR(priv->mclk)) {
		dev_err(dev, "Failed to get clk_cif_out!\n");
		return -EINVAL;
	}

	priv->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(priv->reset_gpio)) {
		dev_err(dev, "Failed to get reset-gpios!\n");
		return -EINVAL;
	}

	priv->supply = devm_regulator_get(&client->dev, "power");
	if (IS_ERR(priv->supply)) {
		dev_err(dev, "Failed to get power-supply!\n");
		return -EINVAL;
	}

	ret = sysfs_create_group(&client->dev.kobj, &ov5695_attribute_group);
	if (ret) {
		dev_err(dev, "create sysfs failed\n");
	}

	v4l2_i2c_subdev_init(&ov5695.sd, client, &ov5695_subdev_ops);

	/* Check device id */
	ov5695_s_power(priv, 1);
	ret = ov5695_read_reg(client, 0x300b, &value);
	ret = ov5695_read_reg(client, 0x300c, &value);
	//ov5695_s_power(priv, 0);

	if (ret) {
		dev_info(dev, "Read registers failed\n");
		return -EINVAL;
	}

	ov5695.custom = ov5695_custom_config;

	//i2c_set_clientdata(client, priv);
	dev_info(dev, "Probe successfully\n");

	return 0;
}

static int ov5695_remove(struct i2c_client *client)
{
/*
	struct ov5695_priv *priv = i2c_get_clientdata(client);

	ov5695_s_power(priv, 0);
*/

	return 0;
}

static const struct i2c_device_id ov5695_id[] = {
	{ "ov5695", 0 },
	{ }
};

static const struct of_device_id ov5695_of_match[] = {
	{ .compatible = "omnivision,ov5695-v4l2-i2c-subdev" },
	{},
};

static struct i2c_driver ov5695_i2c_driver = {
	.driver = {
		.name = "ov5695",
		.owner = THIS_MODULE,
		.of_match_table = ov5695_of_match
	},
	.probe		= &ov5695_probe,
	.remove		= &ov5695_remove,
	.id_table	= ov5695_id,
};

module_i2c_driver(ov5695_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for OmniVision OV5695");
MODULE_LICENSE("GPL v2");
