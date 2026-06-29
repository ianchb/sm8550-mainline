/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __SOC_QCOM_BATTMGR_H__
#define __SOC_QCOM_BATTMGR_H__

#include <linux/errno.h>
#include <linux/kconfig.h>
#include <linux/types.h>

#if IS_REACHABLE(CONFIG_BATTERY_QCOM_BATTMGR)
int qcom_battmgr_set_keyboard_plugin(bool attached);
#else
static inline int qcom_battmgr_set_keyboard_plugin(bool attached)
{
	return -EOPNOTSUPP;
}
#endif

#endif
