// SPDX-License-Identifier: GPL-2.0-only
/*
 * Minimal platform side of the FPC1553 fingerprint sensor used by sheng.
 *
 * The Android fpc1552 module does not transfer image data. Its userspace
 * HAL talks to the fpcsheng QSEE application and uses this device only for
 * power, reset and IRQ state.  Keep those sysfs names and transitions here
 * because the HAL opens them by name.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>

struct fpc1553 {
	struct device *dev;
	struct pinctrl *pinctrl;
	struct pinctrl_state *reset_low;
	struct pinctrl_state *reset_high;
	struct pinctrl_state *irq_active;
	struct regulator *vdd;
	struct gpio_desc *irq_gpiod;
	struct mutex lock; /* Serialize power and IRQ state changes. */
	int irq;
	bool irq_requested;
	bool prepared;
	bool irq_enabled;
	bool irq_wake;
	bool wakeup_enabled;
	bool fingerdown_wait;
	bool screen_on;
	bool regulator_mode;
	bool regulator_enabled;
};

static irqreturn_t fpc1553_irq_handler(int irq, void *data);

static int fpc1553_select_reset(struct fpc1553 *fpc, bool high)
{
	struct pinctrl_state *state = high ? fpc->reset_high : fpc->reset_low;

	if (IS_ERR_OR_NULL(state))
		return -EINVAL;
	return pinctrl_select_state(fpc->pinctrl, state);
}

static int fpc1553_hw_reset(struct fpc1553 *fpc)
{
	int ret;

	ret = fpc1553_select_reset(fpc, true);
	if (ret)
		return ret;
	usleep_range(100, 200);

	ret = fpc1553_select_reset(fpc, false);
	if (ret)
		return ret;
	usleep_range(5000, 8000);

	ret = fpc1553_select_reset(fpc, true);
	if (ret)
		return ret;
	usleep_range(5000, 8000);

	dev_info(fpc->dev, "IRQ after reset %d\n",
		 gpiod_get_raw_value_cansleep(fpc->irq_gpiod));
	return 0;
}

static int fpc1553_request_irq(struct fpc1553 *fpc)
{
	struct device *dev = fpc->dev;
	int ret;

	if (fpc->irq_requested)
		return 0;

	if (!fpc->irq_gpiod) {
		fpc->irq_gpiod = gpiod_get(dev, "irq", GPIOD_IN);
		if (IS_ERR(fpc->irq_gpiod)) {
			ret = PTR_ERR(fpc->irq_gpiod);
			fpc->irq_gpiod = NULL;
			return ret;
		}
	}

	if (!IS_ERR_OR_NULL(fpc->irq_active)) {
		ret = pinctrl_select_state(fpc->pinctrl, fpc->irq_active);
		if (ret)
			goto err_gpio;
	}

	ret = gpiod_direction_input(fpc->irq_gpiod);
	if (ret)
		goto err_gpio;

	fpc->irq = gpiod_to_irq(fpc->irq_gpiod);
	if (fpc->irq < 0) {
		ret = fpc->irq;
		goto err_gpio;
	}

	ret = request_threaded_irq(fpc->irq, NULL, fpc1553_irq_handler,
				   IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				   "fpc1553", fpc);
	if (ret)
		goto err_gpio;

	fpc->irq_requested = true;
	fpc->irq_enabled = true;
	dev_info(dev, "fpc requested irq %d\n", fpc->irq);
	return 0;

err_gpio:
	if (fpc->irq_gpiod) {
		gpiod_put(fpc->irq_gpiod);
		fpc->irq_gpiod = NULL;
	}
	return ret;
}

static void fpc1553_release_irq(struct fpc1553 *fpc)
{
	if (fpc->irq_requested) {
		if (fpc->irq_enabled) {
			disable_irq(fpc->irq);
			fpc->irq_enabled = false;
		}
		if (fpc->irq_wake) {
			irq_set_irq_wake(fpc->irq, 0);
			fpc->irq_wake = false;
		}
		free_irq(fpc->irq, fpc);
		fpc->irq_requested = false;
	}

	if (fpc->irq_gpiod) {
		gpiod_put(fpc->irq_gpiod);
		fpc->irq_gpiod = NULL;
	}
	fpc->irq_enabled = false;
}

static int fpc1553_set_power(struct fpc1553 *fpc, bool enable)
{
	int ret;

	if (!fpc->regulator_mode)
		return 0;

	if (enable) {
		if (fpc->regulator_enabled)
			return 0;
		ret = regulator_set_load(fpc->vdd, 100000);
		if (ret < 0)
			dev_warn(fpc->dev, "regulator set load failed: %d\n", ret);
		ret = regulator_enable(fpc->vdd);
		if (!ret)
			fpc->regulator_enabled = true;
		return ret;
	}

	if (fpc->regulator_enabled) {
		ret = regulator_disable(fpc->vdd);
		if (!ret)
			fpc->regulator_enabled = false;
		return ret;
	}
	return 0;
}

static int fpc1553_prepare(struct fpc1553 *fpc, bool enable)
{
	int ret = 0;

	mutex_lock(&fpc->lock);
	if (enable) {
		if (fpc->prepared)
			goto out;

		ret = fpc1553_request_irq(fpc);
		if (ret)
			goto out;
		if (fpc->irq_requested && !fpc->irq_wake) {
			ret = irq_set_irq_wake(fpc->irq, 1);
			if (ret) {
				dev_warn(fpc->dev, "failed to enable IRQ wake: %d\n",
					 ret);
				ret = 0;
			} else {
				fpc->irq_wake = true;
			}
		}

		ret = fpc1553_select_reset(fpc, false);
		if (ret)
			goto err_resources;
		ret = fpc1553_set_power(fpc, true);
		if (ret)
			goto err_resources;
		usleep_range(100, 1000);
		ret = fpc1553_select_reset(fpc, true);
		if (!ret)
			ret = fpc1553_hw_reset(fpc);
		if (!ret)
			fpc->prepared = true;
		else
			goto err_resources;
	} else {
		if (!fpc->prepared)
			goto out;
		if (fpc->irq_requested && fpc->irq_enabled) {
			disable_irq(fpc->irq);
			fpc->irq_enabled = false;
		}
		ret = fpc1553_select_reset(fpc, false);
		if (!ret)
			ret = fpc1553_set_power(fpc, false);
		fpc->prepared = false;
	}
	goto out;

err_resources:
	fpc->prepared = false;
	fpc1553_select_reset(fpc, false);
	fpc1553_set_power(fpc, false);
	fpc1553_release_irq(fpc);
out:
	mutex_unlock(&fpc->lock);
	return ret;
}

static irqreturn_t fpc1553_irq_handler(int irq, void *data)
{
	struct fpc1553 *fpc = data;

	sysfs_notify(&fpc->dev->kobj, NULL, "irq");
	return IRQ_HANDLED;
}

static ssize_t irq_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct fpc1553 *fpc = dev_get_drvdata(dev);

	if (!fpc->irq_gpiod)
		return -ENODEV;
	return sysfs_emit(buf, "%d\n",
			  gpiod_get_raw_value_cansleep(fpc->irq_gpiod));
}

static ssize_t irq_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	/* The Android handler writes "ack"; the GPIO IRQ is already acked. */
	return count;
}

static ssize_t irq_enable_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct fpc1553 *fpc = dev_get_drvdata(dev);

	mutex_lock(&fpc->lock);
	if (buf[0] == '0' && fpc->irq_requested && fpc->irq_enabled) {
		disable_irq(fpc->irq);
		fpc->irq_enabled = false;
	} else if (buf[0] == '1' && fpc->irq_requested && !fpc->irq_enabled) {
		enable_irq(fpc->irq);
		fpc->irq_enabled = true;
	}
	mutex_unlock(&fpc->lock);
	return count;
}

static ssize_t wakeup_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct fpc1553 *fpc = dev_get_drvdata(dev);

	mutex_lock(&fpc->lock);
	fpc->wakeup_enabled = sysfs_streq(buf, "enable");
	mutex_unlock(&fpc->lock);
	return count;
}

static ssize_t fingerdown_wait_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct fpc1553 *fpc = dev_get_drvdata(dev);

	mutex_lock(&fpc->lock);
	if (!fpc->prepared) {
		mutex_unlock(&fpc->lock);
		return -EINVAL;
	}
	if (sysfs_streq(buf, "enable")) {
		fpc->fingerdown_wait = true;
	} else if (sysfs_streq(buf, "disable")) {
		fpc->fingerdown_wait = false;
	} else {
		mutex_unlock(&fpc->lock);
		return -EINVAL;
	}
	mutex_unlock(&fpc->lock);
	return count;
}

static ssize_t power_cfg_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct fpc1553 *fpc = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%s\n", fpc->regulator_mode ? "3V3" : "1V8");
}

static ssize_t power_cfg_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct fpc1553 *fpc = dev_get_drvdata(dev);

	mutex_lock(&fpc->lock);
	if (sysfs_streq(buf, "3V3")) {
		fpc->regulator_mode = true;
	} else if (sysfs_streq(buf, "1V8")) {
		fpc->regulator_mode = false;
	} else {
		mutex_unlock(&fpc->lock);
		return -EINVAL;
	}
	mutex_unlock(&fpc->lock);
	dev_info(dev, "fpc set power_cfg: %d, rc: 0\n", fpc->regulator_mode);
	return count;
}

static ssize_t power_ctrl_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct fpc1553 *fpc = dev_get_drvdata(dev);
	int enabled;

	mutex_lock(&fpc->lock);
	enabled = fpc->vdd && regulator_is_enabled(fpc->vdd) > 0;
	mutex_unlock(&fpc->lock);
	return sysfs_emit(buf, "%d\n", enabled);
}

static ssize_t power_ctrl_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct fpc1553 *fpc = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&fpc->lock);
	if (sysfs_streq(buf, "enable")) {
		ret = fpc1553_set_power(fpc, true);
		if (!ret)
			usleep_range(100000, 200000);
	} else if (sysfs_streq(buf, "disable") ||
		   sysfs_streq(buf, "force_disable")) {
		if (sysfs_streq(buf, "force_disable")) {
			ret = regulator_force_disable(fpc->vdd);
			if (!ret)
				fpc->regulator_enabled = false;
		} else {
			ret = fpc1553_set_power(fpc, false);
		}
	} else if (sysfs_streq(buf, "power_on_reset")) {
		ret = fpc1553_set_power(fpc, true);
		if (!ret) {
			usleep_range(100000, 200000);
			ret = fpc1553_hw_reset(fpc);
		}
	} else {
		ret = -EINVAL;
	}
	mutex_unlock(&fpc->lock);
	return ret ? ret : count;
}

static ssize_t hw_reset_store(struct device *dev,
			      struct device_attribute *attr, const char *buf,
			      size_t count)
{
	struct fpc1553 *fpc = dev_get_drvdata(dev);
	int ret;

	if (!sysfs_streq(buf, "reset"))
		return -EINVAL;
	mutex_lock(&fpc->lock);
	ret = fpc1553_set_power(fpc, false);
	if (!ret)
		usleep_range(100000, 200000);
	if (!ret)
		ret = fpc1553_set_power(fpc, true);
	if (!ret)
		usleep_range(100000, 200000);
	if (!ret)
		ret = fpc1553_hw_reset(fpc);
	mutex_unlock(&fpc->lock);
	return ret ? ret : count;
}

static ssize_t device_prepare_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int ret;

	if (sysfs_streq(buf, "enable"))
		ret = fpc1553_prepare(dev_get_drvdata(dev), true);
	else if (sysfs_streq(buf, "disable"))
		ret = fpc1553_prepare(dev_get_drvdata(dev), false);
	else
		ret = -EINVAL;
	return ret ? ret : count;
}

static ssize_t request_vreg_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct fpc1553 *fpc = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&fpc->lock);
	if (sysfs_streq(buf, "enable"))
		ret = fpc1553_request_irq(fpc);
	else if (sysfs_streq(buf, "disable"))
		fpc1553_release_irq(fpc);
	else
		ret = -EINVAL;
	mutex_unlock(&fpc->lock);
	return ret ? ret : count;
}

static ssize_t pinctl_set_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct fpc1553 *fpc = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&fpc->lock);
	if (sysfs_streq(buf, "fpc1020_reset_reset"))
		ret = fpc1553_select_reset(fpc, false);
	else if (sysfs_streq(buf, "fpc1020_reset_active"))
		ret = fpc1553_select_reset(fpc, true);
	else
		ret = -EINVAL;
	mutex_unlock(&fpc->lock);
	return ret ? ret : count;
}

static ssize_t clk_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	return count;
}

static ssize_t handle_wakelock_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	return count;
}

static ssize_t screen_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct fpc1553 *fpc = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", fpc->screen_on);
}

static ssize_t vendor_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "fpc1552\n");
}

static ssize_t regulator_enable_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct fpc1553 *fpc = dev_get_drvdata(dev);
	char name[16];
	char action;
	int ret;

	if (sscanf(buf, "%15[^,],%c", name, &action) != 2 ||
	    strcmp(name, "vdd_ana") || (action != 'e' && action != 'd'))
		return -EINVAL;
	mutex_lock(&fpc->lock);
	ret = fpc1553_set_power(fpc, action == 'e');
	mutex_unlock(&fpc->lock);
	return ret ? ret : count;
}

static DEVICE_ATTR_RW(irq);
static DEVICE_ATTR_WO(irq_enable);
static DEVICE_ATTR_WO(wakeup_enable);
static DEVICE_ATTR_WO(fingerdown_wait);
static DEVICE_ATTR_RW(power_cfg);
static DEVICE_ATTR_RW(power_ctrl);
static DEVICE_ATTR_WO(hw_reset);
static DEVICE_ATTR_WO(device_prepare);
static DEVICE_ATTR_WO(request_vreg);
static DEVICE_ATTR_WO(pinctl_set);
static DEVICE_ATTR_WO(regulator_enable);
static DEVICE_ATTR_WO(clk_enable);
static DEVICE_ATTR_WO(handle_wakelock);
static DEVICE_ATTR_RO(screen);
static DEVICE_ATTR_RO(vendor);

static struct attribute *fpc1553_attrs[] = {
	&dev_attr_irq.attr,
	&dev_attr_irq_enable.attr,
	&dev_attr_wakeup_enable.attr,
	&dev_attr_fingerdown_wait.attr,
	&dev_attr_power_cfg.attr,
	&dev_attr_power_ctrl.attr,
	&dev_attr_hw_reset.attr,
	&dev_attr_device_prepare.attr,
	&dev_attr_request_vreg.attr,
	&dev_attr_pinctl_set.attr,
	&dev_attr_regulator_enable.attr,
	&dev_attr_clk_enable.attr,
	&dev_attr_handle_wakelock.attr,
	&dev_attr_screen.attr,
	&dev_attr_vendor.attr,
	NULL,
};

static const struct attribute_group fpc1553_attr_group = {
	.attrs = fpc1553_attrs,
};

static int fpc1553_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fpc1553 *fpc;
	int ret;

	fpc = devm_kzalloc(dev, sizeof(*fpc), GFP_KERNEL);
	if (!fpc)
		return -ENOMEM;
	fpc->dev = dev;
	fpc->screen_on = true;
	mutex_init(&fpc->lock);
	platform_set_drvdata(pdev, fpc);

	fpc->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(fpc->pinctrl))
		return dev_err_probe(dev, PTR_ERR(fpc->pinctrl),
				    "failed to get pinctrl\n");
	fpc->reset_low = pinctrl_lookup_state(fpc->pinctrl,
					      "fpc1020_reset_reset");
	fpc->reset_high = pinctrl_lookup_state(fpc->pinctrl,
					       "fpc1020_reset_active");
	fpc->irq_active = pinctrl_lookup_state(fpc->pinctrl,
					       "fpc1020_irq_active");
	if (IS_ERR(fpc->reset_low) || IS_ERR(fpc->reset_high))
		return dev_err_probe(dev, -EINVAL, "missing reset pinctrl states\n");

	fpc->vdd = devm_regulator_get(dev, "fp_vdd_vreg");
	if (IS_ERR(fpc->vdd))
		return dev_err_probe(dev, PTR_ERR(fpc->vdd),
				    "failed to get fp_vdd_vreg\n");
	fpc->regulator_mode = true;
	device_init_wakeup(dev, true);

	ret = sysfs_create_group(&dev->kobj, &fpc1553_attr_group);
	if (ret)
		return ret;

	dev_info(dev, "fpc1553 platform side ready\n");
	return 0;
}

static void fpc1553_remove(struct platform_device *pdev)
{
	struct fpc1553 *fpc = platform_get_drvdata(pdev);

	mutex_lock(&fpc->lock);
	if (fpc->prepared) {
		if (fpc->irq_requested && fpc->irq_enabled) {
			disable_irq(fpc->irq);
			fpc->irq_enabled = false;
		}
		fpc1553_select_reset(fpc, false);
		fpc1553_set_power(fpc, false);
		fpc->prepared = false;
	}
	fpc1553_release_irq(fpc);
	mutex_unlock(&fpc->lock);
	sysfs_remove_group(&pdev->dev.kobj, &fpc1553_attr_group);
	device_init_wakeup(&pdev->dev, false);
}

static const struct of_device_id fpc1553_of_match[] = {
	{ .compatible = "fpc,fpc1020" },
	{ }
};
MODULE_DEVICE_TABLE(of, fpc1553_of_match);

static struct platform_driver fpc1553_driver = {
	.probe = fpc1553_probe,
	.remove = fpc1553_remove,
	.driver = {
		.name = "fpc1553",
		.of_match_table = fpc1553_of_match,
	},
};
module_platform_driver(fpc1553_driver);

MODULE_DESCRIPTION("Xiaomi sheng FPC1553 platform support");
MODULE_AUTHOR("siergtc");
MODULE_LICENSE("GPL");
