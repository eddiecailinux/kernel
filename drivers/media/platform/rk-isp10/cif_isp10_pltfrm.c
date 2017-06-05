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

#ifndef CONFIG_OF
#error "this driver requires a kernel with device tree support"
#endif

#include <linux/clk.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/list.h>
#include "cif_isp10.h"
#include <linux/platform_data/rk_isp10_platform.h>
#include "cif_isp10_regs.h"

struct cif_isp10_pltfrm_csi_config {
	struct list_head list;
	u32 pps;
	struct cif_isp10_csi_config csi_config;
};

struct cif_isp10_pltfrm_data {
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sleep;
	struct pinctrl_state *pins_inactive;
	void __iomem *base_addr;
	int irq;
	struct {
		int mis;
		int (*isr)(unsigned int mis, void *cntxt);
	} irq_handlers[4];
	struct list_head csi0_configs;
	struct list_head csi1_configs;
	s32 exp_time;
	u16 gain;
};

static irqreturn_t cif_isp10_pltfrm_irq_handler(int irq, void *cntxt)
{
	unsigned int i, mis_val;
	int ret;
	struct device *dev = cntxt;
	struct cif_isp10_pltfrm_data *pdata =
		dev_get_platdata(dev);
	void *cif_isp10_dev = dev_get_drvdata(dev);

	if (irq != pdata->irq)
		return IRQ_NONE;

	for (i = 0; i < ARRAY_SIZE(pdata->irq_handlers); i++) {
		if (IS_ERR_VALUE(pdata->irq_handlers[i].mis))
			break;

		if (IS_ERR_OR_NULL(pdata->irq_handlers[i].isr)) {
			cif_isp10_pltfrm_pr_err(NULL,
				"ISR for IRQ #%d not set\n", irq);
			break;
		}

		mis_val = cif_ioread32(pdata->base_addr +
			pdata->irq_handlers[i].mis);
		if (mis_val == 0)
			continue;

		ret = pdata->irq_handlers[i].isr(mis_val, cif_isp10_dev);
		if (IS_ERR_VALUE(ret)) {
			cif_isp10_pltfrm_pr_err(NULL,
				"ISR for IRQ #%d failed with error %d\n",
				irq, ret);
		}
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

const char *cif_isp10_pltfrm_pm_state_string(
	enum cif_isp10_pm_state pm_state)
{
	switch (pm_state) {
	case CIF_ISP10_PM_STATE_OFF:
		return "CIF_ISP10_PM_STATE_OFF";
	case CIF_ISP10_PM_STATE_SUSPENDED:
		return "CIF_ISP10_PM_STATE_SUSPENDED";
	case CIF_ISP10_PM_STATE_SW_STNDBY:
		return "CIF_ISP10_PM_STATE_SW_STNDBY";
	case CIF_ISP10_PM_STATE_STREAMING:
		return "CIF_ISP10_PM_STATE_STREAMING";
	default:
		return "PM_STATE_UNKNOWN";
	}
}

inline void cif_isp10_pltfrm_write_reg(
	struct device *dev,
	u32 data,
	CIF_ISP10_PLTFRM_MEM_IO_ADDR addr)
{
	iowrite32(data, addr);
}

inline void cif_isp10_pltfrm_write_reg_OR(
	struct device *dev,
	u32 data,
	CIF_ISP10_PLTFRM_MEM_IO_ADDR addr)
{
	cif_isp10_pltfrm_write_reg(dev,
		(ioread32(addr) | data), addr);
}

inline void cif_isp10_pltfrm_write_reg_AND(
	struct device *dev,
	u32 data,
	CIF_ISP10_PLTFRM_MEM_IO_ADDR addr)
{
	cif_isp10_pltfrm_write_reg(dev,
		(ioread32(addr) & data), addr);
}

inline u32 cif_isp10_pltfrm_read_reg(
	struct device *dev,
	CIF_ISP10_PLTFRM_MEM_IO_ADDR addr)
{
	return ioread32(addr);
}

int cif_isp10_pltfrm_dev_init(
	struct cif_isp10_device *cif_isp10_dev,
	struct device **_dev,
	void __iomem **reg_base_addr)
{
	int ret;
	struct cif_isp10_pltfrm_data *pdata;
	struct device *dev = *_dev;
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct resource *res;
	void __iomem *base_addr;
	unsigned int i, irq;

	dev_set_drvdata(dev, cif_isp10_dev);
	cif_isp10_dev->dev = dev;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		cif_isp10_pltfrm_pr_err(dev,
			"could not allocate memory for platform data\n");
		ret = -ENOMEM;
		goto err;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		cif_isp10_pltfrm_pr_err(NULL,
			"platform_get_resource(IO) failed\n");
		ret = -ENODEV;
		goto err;
	}
	base_addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR_OR_NULL(base_addr)) {
		cif_isp10_pltfrm_pr_err(NULL, "devm_ioremap_resource failed\n");
		if (IS_ERR(base_addr))
			ret = PTR_ERR(base_addr);
		else
			ret = -ENODEV;
	}
	*reg_base_addr = base_addr;
	pdata->base_addr = base_addr;

	irq = platform_get_irq(pdev, 0);
	if (IS_ERR_VALUE(irq)) {
		ret = irq;
		cif_isp10_pltfrm_pr_err(NULL, "platform_get_irq(IRQ) failed\n");
		goto err;
	}

	ret = devm_request_threaded_irq(dev,
			irq,
			cif_isp10_pltfrm_irq_handler,
			NULL,
			0,
			dev_driver_string(dev),
			dev);
	if (IS_ERR_VALUE(ret)) {
		cif_isp10_pltfrm_pr_err(NULL,
		"devm_request_threaded_irq failed\n");
		goto err;
	}
	pdata->irq = irq;

	pdata->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR_OR_NULL(pdata->pinctrl)) {
		pdata->pins_default = pinctrl_lookup_state(pdata->pinctrl,
			PINCTRL_STATE_DEFAULT);
		if (IS_ERR(pdata->pins_default))
			cif_isp10_pltfrm_pr_err(dev,
						"could not get default pinstate\n");

		pdata->pins_sleep = pinctrl_lookup_state(pdata->pinctrl,
			PINCTRL_STATE_SLEEP);
		if (IS_ERR(pdata->pins_sleep))
			cif_isp10_pltfrm_pr_warn(dev,
						"could not get pins_sleep pinstate\n");

		pdata->pins_inactive = pinctrl_lookup_state(pdata->pinctrl,
			"inactive");
		if (IS_ERR(pdata->pins_inactive))
			cif_isp10_pltfrm_pr_warn(dev,
						"could not get pins_inactive pinstate\n");

		if (!IS_ERR_OR_NULL(pdata->pins_default))
			pinctrl_select_state(pdata->pinctrl,
				pdata->pins_default);
	}

	for (i = 0; i < ARRAY_SIZE(pdata->irq_handlers); i++)
		pdata->irq_handlers[i].mis = -EINVAL;

	dev->platform_data = pdata;

	INIT_LIST_HEAD(&pdata->csi0_configs);
	INIT_LIST_HEAD(&pdata->csi1_configs);

	return 0;
err:
	cif_isp10_pltfrm_pr_err(NULL, "failed with error %d\n", ret);
	if (!IS_ERR_OR_NULL(pdata))
		devm_kfree(dev, pdata);
	return ret;
}

int cif_isp10_pltfrm_soc_init(
	struct cif_isp10_device *cif_isp10_dev)
{
	struct device *dev = cif_isp10_dev->dev;
	int i, ret = 0;

	cif_isp10_dev->isp_clk = devm_clk_get(dev, "clk-isp");
	if (IS_ERR(cif_isp10_dev->isp_clk)) {
		dev_err(dev, "Failed to get clk-isp\n");
		return PTR_ERR(cif_isp10_dev->isp_clk);
	}
	/* Assuming clk-isp be the first clock in dt */
	for (i = 0; i < CIF_ISP10_MAX_BUS_CLK; i++) {
		struct clk *clk;
		clk = of_clk_get(dev->of_node, i + 1);
		if (IS_ERR(clk)) {
			cif_isp10_dev->bus_clk[i] = ERR_PTR(-EINVAL);
			break;
		} else {
			cif_isp10_dev->bus_clk[i] = clk;
		}
	}

	return ret;
}

static int cif_isp10_disable_sys_clk(struct cif_isp10_device *cif_isp10_dev)
{
	struct clk *clk;
	int i;

	return 0; //TODO(zsq)

	clk_disable_unprepare(cif_isp10_dev->isp_clk);

	for (i=0; i<CIF_ISP10_MAX_BUS_CLK; i++) {
		clk = cif_isp10_dev->bus_clk[i];

		if (IS_ERR(clk))
			break;
		clk_disable_unprepare(clk);
	}

	return 0;
}

static int cif_isp10_enable_sys_clk(struct cif_isp10_device *cif_isp10_dev)
{
	struct device *dev = cif_isp10_dev->dev;
	struct clk *clk;
	int i, ret;

	/* Enable all bus clk for ISP */
	for (i = 0; i < CIF_ISP10_MAX_BUS_CLK; i++) {
		clk = cif_isp10_dev->bus_clk[i];
		if (IS_ERR(clk))
			break;

		ret = clk_prepare_enable(clk);
		if (ret < 0)
			goto disable_clk;
	}

	/* Enable ISP clk */
	ret = clk_prepare_enable(cif_isp10_dev->isp_clk);
	if (ret < 0)
		goto disable_clk;

	return 0;

disable_clk:
	dev_err(dev, "Failed to enable clk, %d\n", ret);
	cif_isp10_disable_sys_clk(cif_isp10_dev);

	return ret;
}

int cif_isp10_pltfrm_pm_set_state(
	struct device *dev,
	enum cif_isp10_pm_state pm_state)
{
	int ret = 0;
	struct cif_isp10_device *cif_isp10_dev = dev_get_drvdata(dev);

	switch (pm_state) {
	case CIF_ISP10_PM_STATE_OFF:
	case CIF_ISP10_PM_STATE_SUSPENDED:
		cif_isp10_disable_sys_clk(cif_isp10_dev);
		break;
	case CIF_ISP10_PM_STATE_SW_STNDBY:
	case CIF_ISP10_PM_STATE_STREAMING:
		ret = cif_isp10_enable_sys_clk(cif_isp10_dev);
		break;
	default:
		cif_isp10_pltfrm_pr_err(dev,
			"unknown or unsupported PM state %d\n", pm_state);
		return -EINVAL;
	}

	if (IS_ERR_VALUE(ret))
		cif_isp10_pltfrm_pr_err(dev,
			"setting pm state to %s failed with error %d\n",
			cif_isp10_pltfrm_pm_state_string(pm_state), ret);
	else
		cif_isp10_pltfrm_pr_dbg(dev,
			"successfully changed pm state to %s\n",
			cif_isp10_pltfrm_pm_state_string(pm_state));
	return ret;
}

int cif_isp10_pltfrm_pinctrl_set_state(
	struct device *dev,
	enum cif_isp10_pinctrl_state pinctrl_state)
{
	int ret = 0;
	struct cif_isp10_pltfrm_data *pdata = dev_get_platdata(dev);

	cif_isp10_pltfrm_pr_dbg(dev,
		"set pinctrl state to %d\n", pinctrl_state);

	if (!pdata) {
		cif_isp10_pltfrm_pr_err(dev,
			"unable to retrieve CIF platform data\n");
		ret = -EINVAL;
		goto err;
	}
	if (IS_ERR_OR_NULL(pdata->pinctrl))
		return 0;

	switch (pinctrl_state) {
	case CIF_ISP10_PINCTRL_STATE_SLEEP:
		if (!IS_ERR_OR_NULL(pdata->pins_sleep))
			ret = pinctrl_select_state(pdata->pinctrl,
				pdata->pins_sleep);
		break;
	case CIF_ISP10_PINCTRL_STATE_ACTIVE:
	case CIF_ISP10_PINCTRL_STATE_DEFAULT:
		if (!IS_ERR_OR_NULL(pdata->pins_default))
			ret = pinctrl_select_state(pdata->pinctrl,
				pdata->pins_default);
		break;
	case CIF_ISP10_PINCTRL_STATE_INACTIVE:
		if (!IS_ERR_OR_NULL(pdata->pins_inactive))
			ret = pinctrl_select_state(pdata->pinctrl,
				pdata->pins_inactive);
		break;
	default:
		cif_isp10_pltfrm_pr_err(dev,
			"unknown or unsupported pinctrl state %d\n",
			pinctrl_state);
		ret = -EINVAL;
		goto err;
	}

	if (IS_ERR_VALUE(ret))
		goto err;

	return 0;
err:
	cif_isp10_pltfrm_pr_err(dev, "failed with error %d\n", ret);
	return ret;
}

int cif_isp10_pltfrm_irq_register_isr(
	struct device *dev,
	unsigned int mis,
	int (*isr)(unsigned int mis, void *cntxt),
	void *cntxt)
{
	int ret = 0;
	unsigned int i;
	int slot = -EINVAL;
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct cif_isp10_pltfrm_data *pdata =
		dev_get_platdata(&pdev->dev);
	bool skip_request_irq = false;
	const char *irq_name;

	switch (mis) {
	case CIF_MIPI_MIS:
		irq_name = "CIF_ISP10_MIPI_IRQ";
		break;
	case CIF_ISP_MIS:
		irq_name = "CIF_ISP10_ISP_IRQ";
		break;
	case CIF_MI_MIS:
		irq_name = "CIF_ISP10_MI_IRQ";
		break;
	default:
		cif_isp10_pltfrm_pr_err(dev,
			"unknown or unsupported IRQ %d\n", mis);
		ret = -EINVAL;
		goto err;
	}
	cif_isp10_pltfrm_pr_dbg(dev,
		"registering ISR for IRQ %s\n", irq_name);

	for (i = 0; i < ARRAY_SIZE(pdata->irq_handlers); i++) {
		if (IS_ERR_VALUE(slot) &&
			IS_ERR_VALUE(pdata->irq_handlers[i].mis))
			slot = (int)i;
		if (pdata->irq_handlers[i].mis == mis) {
			cif_isp10_pltfrm_pr_dbg(dev,
				"overwriting ISR for IRQ %s\n", irq_name);
			slot = (int)i;
			skip_request_irq = true;
			break;
		}
	}
	if (IS_ERR_VALUE(slot)) {
		if (!isr)
			return 0;
		cif_isp10_pltfrm_pr_err(dev,
			"cannot register ISR for IRQ %s, too many ISRs already registered\n",
			irq_name);
		ret = -EFAULT;
		goto err;
	}
	pdata->irq_handlers[slot].isr = isr;
	if (!isr) {
		pdata->irq_handlers[slot].mis = -EINVAL;
		skip_request_irq = true;
	} else {
		pdata->irq_handlers[slot].mis = mis;
	}

	return 0;
err:
	cif_isp10_pltfrm_pr_err(dev, "failed with error %d\n", ret);
	return ret;
}

const char *cif_isp10_pltfrm_get_device_type(
	struct device *dev)
{
	return dev->of_node->type;
}

const char *cif_isp10_pltfrm_dev_string(
	struct device *dev)
{
	return dev_driver_string(dev);
}
