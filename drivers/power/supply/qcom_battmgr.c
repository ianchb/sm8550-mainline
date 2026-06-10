// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2019-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022, Linaro Ltd
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */
#include <linux/auxiliary_bus.h>
#include <linux/hex.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvmem-consumer.h>
#include <linux/of_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/soc/qcom/pdr.h>
#include <linux/soc/qcom/pmic_glink.h>
#include <linux/math.h>
#include <linux/string.h>
#include <linux/unaligned.h>
#include <linux/units.h>

#define BATTMGR_CHEMISTRY_LEN	4
#define BATTMGR_STRING_LEN	128

enum qcom_battmgr_variant {
	QCOM_BATTMGR_SC8280XP,
	QCOM_BATTMGR_SM8350,
	QCOM_BATTMGR_SM8550,
	QCOM_BATTMGR_X1E80100,
	XIAOMI_BATTMGR_SM8550
};

#define BATTMGR_BAT_STATUS		0x1

#define BATTMGR_REQUEST_NOTIFICATION	0x4

#define BATTMGR_NOTIFICATION		0x7
#define NOTIF_BAT_PROPERTY		0x30
#define NOTIF_USB_PROPERTY		0x32
#define NOTIF_WLS_PROPERTY		0x34
#define NOTIF_BAT_STATUS		0x80
#define NOTIF_BAT_INFO			0x81
#define NOTIF_BAT_CHARGING_STATE	0x83

#define BATTMGR_BAT_INFO		0x9

#define BATTMGR_BAT_DISCHARGE_TIME	0xc

#define BATTMGR_BAT_CHARGE_TIME		0xd

#define BATTMGR_BAT_PROPERTY_GET	0x30
#define BATTMGR_BAT_PROPERTY_SET	0x31
#define BATT_STATUS			0
#define BATT_HEALTH			1
#define BATT_PRESENT			2
#define BATT_CHG_TYPE			3
#define BATT_CAPACITY			4
#define BATT_SOH			5
#define BATT_VOLT_OCV			6
#define BATT_VOLT_NOW			7
#define BATT_VOLT_MAX			8
#define BATT_CURR_NOW			9
#define BATT_CHG_CTRL_LIM		10
#define BATT_CHG_CTRL_LIM_MAX		11
#define BATT_TEMP			12
#define BATT_TECHNOLOGY			13
#define BATT_CHG_COUNTER		14
#define BATT_CYCLE_COUNT		15
#define BATT_CHG_FULL_DESIGN		16
#define BATT_CHG_FULL			17
#define BATT_MODEL_NAME			18
#define BATT_TTF_AVG			19
#define BATT_TTE_AVG			20
#define BATT_RESISTANCE			21
#define BATT_POWER_NOW			22
#define BATT_POWER_AVG			23
#define BATT_CHG_CTRL_EN		24
#define BATT_CHG_CTRL_START_THR		25
#define BATT_CHG_CTRL_END_THR		26

#define BATTMGR_USB_PROPERTY_GET	0x32
#define BATTMGR_USB_PROPERTY_SET	0x33
#define USB_ONLINE			0
#define USB_VOLT_NOW			1
#define USB_VOLT_MAX			2
#define USB_CURR_NOW			3
#define USB_CURR_MAX			4
#define USB_INPUT_CURR_LIMIT		5
#define USB_TYPE			6
#define USB_ADAP_TYPE			7
#define USB_MOISTURE_DET_EN		8
#define USB_MOISTURE_DET_STS		9

#define BATTMGR_WLS_PROPERTY_GET	0x34
#define BATTMGR_WLS_PROPERTY_SET	0x35
#define WLS_ONLINE			0
#define WLS_VOLT_NOW			1
#define WLS_VOLT_MAX			2
#define WLS_CURR_NOW			3
#define WLS_CURR_MAX			4
#define WLS_TYPE			5
#define WLS_BOOST_EN			6

#define BATTMGR_CHG_CTRL_LIMIT_EN	0x48
#define CHARGE_CTRL_START_THR_MIN	50
#define CHARGE_CTRL_START_THR_MAX	95
#define CHARGE_CTRL_END_THR_MIN		55
#define CHARGE_CTRL_END_THR_MAX		100
#define CHARGE_CTRL_DELTA_SOC		5

/* Xiaomi vendor battery properties use a separate qti_battery_charger opcode pair. */
#define BATTMGR_XM_PROPERTY_GET		0x50
#define BATTMGR_XM_PROPERTY_SET		0x51

/* Xiaomi battery properties used by the vendor qti_battery_charger driver. */
#define XM_BATT_VERIFY_DIGEST			0x01
#define XM_BATT_AUTHENTIC			0x03
#define XM_BATT_REAL_TYPE			0x07
#define XM_BATT_VERIFY_PROCESS			0x0c
#define XM_BATT_VDM_CMD_CHARGER_VERSION		0x0d
#define XM_BATT_VDM_CMD_CHARGER_VOLTAGE		0x0e
#define XM_BATT_VDM_CMD_CHARGER_TEMP		0x0f
#define XM_BATT_VDM_CMD_SESSION_SEED		0x10
#define XM_BATT_VDM_CMD_AUTHENTICATION		0x11
#define XM_BATT_VDM_CMD_VERIFIED		0x12
#define XM_BATT_VDM_CMD_REMOVE_COMPENSATION	0x13
#define XM_BATT_VDM_CMD_REVERSE_AUTHEN		0x14
#define XM_BATT_ADAPTER_ID			0x16
#define XM_BATT_ADAPTER_SVID			0x17
#define XM_BATT_PD_VERIFED			0x18
#define XM_BATT_PDO2				0x19
#define XM_BATT_UVDM_STATE			0x1a
#define XM_BATT_BQ2597X_CHIP_OK		0x1b
#define XM_BATT_BQ2597X_SLAVE_CHIP_OK		0x1c
#define XM_BATT_BQ2597X_BUS_CURRENT		0x1d
#define XM_BATT_BQ2597X_SLAVE_BUS_CURRENT	0x1e
#define XM_BATT_BQ2597X_BUS_DELTA		0x1f
#define XM_BATT_BQ2597X_BUS_VOLTAGE		0x20
#define XM_BATT_BQ2597X_BATTERY_PRESENT		0x21
#define XM_BATT_BQ2597X_SLAVE_BATTERY_PRESENT	0x22
#define XM_BATT_BQ2597X_BATTERY_VOLTAGE		0x23
#define XM_BATT_FASTCHGMODE			0x2b
#define XM_BATT_QUICK_CHARGE_TYPE		0x31
#define XM_BATT_APDO_MAX			0x32
#define XM_BATT_POWER_MAX			0x33
#define XM_BATT_PEN_MAC_L			0x53
#define XM_BATT_PEN_MAC_H			0x54
#define XM_BATT_PEN_TX_IOUT			0x55
#define XM_BATT_PEN_TX_VOUT			0x56
#define XM_BATT_PEN_SOC				0x57
#define XM_BATT_PEN_HALL3			0x58
#define XM_BATT_PEN_HALL4			0x59
#define XM_BATT_PEN_TX_SS			0x5a
#define XM_BATT_PEN_PLACE_ERR			0x5b
#define XM_BATT_PEN_FAKE_SS			0x5c
#define XM_BATT_FG1_RM				0x8a
#define XM_BATT_FG1_FCC			0x8b
#define XM_BATT_FG_FAST_CHARGE			0x8f
#define XM_BATT_SLAVE_AUTHENTIC		0xa5
#define XM_BATT_FG2_RM				0xa6
#define XM_BATT_FG2_FCC			0xa7
#define XM_BATT_PPS_PTF			0xe7

struct qcom_battmgr_enable_request {
	struct pmic_glink_hdr hdr;
	__le32 battery_id;
	__le32 power_state;
	__le32 low_capacity;
	__le32 high_capacity;
};

struct qcom_battmgr_property_request {
	struct pmic_glink_hdr hdr;
	__le32 battery;
	__le32 property;
	__le32 value;
};

struct qcom_battmgr_xiaomi_vdm_request {
	struct pmic_glink_hdr hdr;
	__le32 property;
	__le32 data[4];
};

struct qcom_battmgr_xiaomi_digest_request {
	struct pmic_glink_hdr hdr;
	__le32 property;
	u8 digest[32];
	u8 slave;
	u8 reserved[3];
};

struct qcom_battmgr_update_request {
	struct pmic_glink_hdr hdr;
	__le32 battery_id;
};

struct qcom_battmgr_charge_time_request {
	struct pmic_glink_hdr hdr;
	__le32 battery_id;
	__le32 percent;
	__le32 reserved;
};

struct qcom_battmgr_discharge_time_request {
	struct pmic_glink_hdr hdr;
	__le32 battery_id;
	__le32 rate; /* 0 for current rate */
	__le32 reserved;
};

struct qcom_battmgr_charge_ctrl_request {
	struct pmic_glink_hdr hdr;
	__le32 enable;
	__le32 target_soc;
	__le32 delta_soc;
};

struct qcom_battmgr_message {
	struct pmic_glink_hdr hdr;
	union {
		struct {
			__le32 property;
			__le32 value;
			__le32 result;
		} intval;
		struct {
			__le32 property;
			char model[BATTMGR_STRING_LEN];
		} strval;
		struct {
			/*
			 * 0: mWh
			 * 1: mAh
			 */
			__le32 power_unit;
			__le32 design_capacity;
			__le32 last_full_capacity;
			/*
			 * 0 nonrechargable
			 * 1 rechargable
			 */
			__le32 battery_tech;
			__le32 design_voltage; /* mV */
			__le32 capacity_low;
			__le32 capacity_warning;
			__le32 cycle_count;
			/* thousandth of percent */
			__le32 accuracy;
			__le32 max_sample_time_ms;
			__le32 min_sample_time_ms;
			__le32 max_average_interval_ms;
			__le32 min_average_interval_ms;
			/* granularity between low and warning */
			__le32 capacity_granularity1;
			/* granularity between warning and full */
			__le32 capacity_granularity2;
			/*
			 * 0: no
			 * 1: cold
			 * 2: hot
			 */
			__le32 swappable;
			__le32 capabilities;
			char model_number[BATTMGR_STRING_LEN];
			char serial_number[BATTMGR_STRING_LEN];
			char battery_type[BATTMGR_STRING_LEN];
			char oem_info[BATTMGR_STRING_LEN];
			char battery_chemistry[BATTMGR_CHEMISTRY_LEN];
			char uid[BATTMGR_STRING_LEN];
			__le32 critical_bias;
			u8 day;
			u8 month;
			__le16 year;
			__le32 battery_id;
		} info;
		struct {
			/*
			 * BIT(0) discharging
			 * BIT(1) charging
			 * BIT(2) critical low
			 */
			__le32 battery_state;
			/* mWh or mAh, based on info->power_unit */
			__le32 capacity;
			__le32 rate;
			/* mv */
			__le32 battery_voltage;
			/*
			 * BIT(0) power online
			 * BIT(1) discharging
			 * BIT(2) charging
			 * BIT(3) battery critical
			 */
			__le32 power_state;
			/*
			 * 1: AC
			 * 2: USB
			 * 3: Wireless
			 */
			__le32 charging_source;
			__le32 temperature;
		} status;
		__le32 time;
		__le32 notification;
	};
};

#define BATTMGR_CHARGING_SOURCE_AC	1
#define BATTMGR_CHARGING_SOURCE_USB	2
#define BATTMGR_CHARGING_SOURCE_WIRELESS 3

enum qcom_battmgr_unit {
	QCOM_BATTMGR_UNIT_mWh = 0,
	QCOM_BATTMGR_UNIT_mAh = 1
};

struct qcom_battmgr_info {
	bool valid;

	bool present;
	unsigned int charge_type;
	unsigned int design_capacity;
	unsigned int last_full_capacity;
	unsigned int voltage_max_design;
	unsigned int voltage_max;
	unsigned int capacity_low;
	unsigned int capacity_warning;
	unsigned int cycle_count;
	unsigned int charge_count;
	unsigned int charge_ctrl_start;
	unsigned int charge_ctrl_end;
	char model_number[BATTMGR_STRING_LEN];
	char serial_number[BATTMGR_STRING_LEN];
	char oem_info[BATTMGR_STRING_LEN];
	unsigned char technology;
	unsigned char day;
	unsigned char month;
	unsigned short year;
};

struct qcom_battmgr_status {
	unsigned int status;
	unsigned int health;
	unsigned int capacity;
	unsigned int percent;
	int current_now;
	int power_now;
	unsigned int voltage_now;
	unsigned int voltage_ocv;
	unsigned int temperature;
	unsigned int resistance;
	unsigned int soh_percent;

	unsigned int discharge_time;
	unsigned int charge_time;
};

struct qcom_battmgr_ac {
	bool online;
};

struct qcom_battmgr_usb {
	bool online;
	unsigned int voltage_now;
	unsigned int voltage_max;
	unsigned int current_now;
	unsigned int current_max;
	unsigned int current_limit;
	unsigned int usb_type;
};

struct qcom_battmgr_wireless {
	bool online;
	unsigned int voltage_now;
	unsigned int voltage_max;
	unsigned int current_now;
	unsigned int current_max;
};

struct qcom_battmgr {
	struct device *dev;
	struct pmic_glink_client *client;

	enum qcom_battmgr_variant variant;

	struct power_supply *ac_psy;
	struct power_supply *bat_psy;
	struct power_supply *usb_psy;
	struct power_supply *wls_psy;

	enum qcom_battmgr_unit unit;

	int error;
	struct completion ack;

	bool service_up;

	struct qcom_battmgr_info info;
	struct qcom_battmgr_status status;
	struct qcom_battmgr_ac ac;
	struct qcom_battmgr_usb usb;
	struct qcom_battmgr_wireless wireless;

	bool xiaomi_prop_pending;
	unsigned int xiaomi_prop;
	u32 xiaomi_prop_value;
	u8 xiaomi_prop_data[32];
	size_t xiaomi_prop_data_len;
	u8 xiaomi_verify_slave;

	struct work_struct enable_work;

	/*
	 * @lock is used to prevent concurrent power supply requests to the
	 * firmware, as it then stops responding.
	 */
	struct mutex lock;
};

static int qcom_battmgr_request(struct qcom_battmgr *battmgr, void *data, size_t len)
{
	unsigned long left;
	int ret;

	reinit_completion(&battmgr->ack);

	battmgr->error = 0;

	ret = pmic_glink_send(battmgr->client, data, len);
	if (ret < 0)
		return ret;

	left = wait_for_completion_timeout(&battmgr->ack, HZ);
	if (!left)
		return -ETIMEDOUT;

	return battmgr->error;
}

static int qcom_battmgr_request_property(struct qcom_battmgr *battmgr, int opcode,
					 int property, u32 value)
{
	struct qcom_battmgr_property_request request = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(opcode),
		.battery = cpu_to_le32(0),
		.property = cpu_to_le32(property),
		.value = cpu_to_le32(value),
	};

	return qcom_battmgr_request(battmgr, &request, sizeof(request));
}

static int qcom_battmgr_xiaomi_request_property(struct qcom_battmgr *battmgr, int opcode,
						int property, u32 value, u32 *out_value)
{
	int ret;

	if (battmgr->variant != XIAOMI_BATTMGR_SM8550)
		return -EOPNOTSUPP;

	if (!battmgr->service_up)
		return -EAGAIN;

	mutex_lock(&battmgr->lock);
	battmgr->xiaomi_prop_pending = true;
	battmgr->xiaomi_prop = property;
	battmgr->xiaomi_prop_value = 0;
	battmgr->xiaomi_prop_data_len = 0;

	ret = qcom_battmgr_request_property(battmgr, opcode, property, value);
	if (!ret && out_value)
		*out_value = battmgr->xiaomi_prop_value;

	battmgr->xiaomi_prop_pending = false;
	mutex_unlock(&battmgr->lock);

	return ret;
}

static int qcom_battmgr_xiaomi_update_capacity(struct qcom_battmgr *battmgr)
{
	u32 fg1_rm, fg1_fcc, fg2_rm, fg2_fcc;
	u64 rm, fcc;
	int ret;

	ret = qcom_battmgr_xiaomi_request_property(battmgr, BATTMGR_XM_PROPERTY_GET,
						   XM_BATT_FG1_RM, 0, &fg1_rm);
	if (ret < 0)
		return ret;

	ret = qcom_battmgr_xiaomi_request_property(battmgr, BATTMGR_XM_PROPERTY_GET,
						   XM_BATT_FG1_FCC, 0, &fg1_fcc);
	if (ret < 0)
		return ret;

	ret = qcom_battmgr_xiaomi_request_property(battmgr, BATTMGR_XM_PROPERTY_GET,
						   XM_BATT_FG2_RM, 0, &fg2_rm);
	if (ret < 0)
		return ret;

	ret = qcom_battmgr_xiaomi_request_property(battmgr, BATTMGR_XM_PROPERTY_GET,
						   XM_BATT_FG2_FCC, 0, &fg2_fcc);
	if (ret < 0)
		return ret;

	rm = (u64)fg1_rm + fg2_rm;
	fcc = (u64)fg1_fcc + fg2_fcc;
	if (!fcc)
		return -ENODATA;

	battmgr->status.percent = min_t(unsigned int,
					DIV_ROUND_CLOSEST_ULL(rm * 100, fcc),
					100);

	return 0;
}

static int qcom_battmgr_xiaomi_request_vdm(struct qcom_battmgr *battmgr, int opcode,
					   int property, const __le32 *data,
					   __le32 *out_data)
{
	struct qcom_battmgr_xiaomi_vdm_request request = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(opcode),
		.property = cpu_to_le32(property),
	};
	int ret;

	if (battmgr->variant != XIAOMI_BATTMGR_SM8550)
		return -EOPNOTSUPP;

	if (!battmgr->service_up)
		return -EAGAIN;

	if (data)
		memcpy(request.data, data, sizeof(request.data));

	mutex_lock(&battmgr->lock);
	battmgr->xiaomi_prop_pending = true;
	battmgr->xiaomi_prop = property;
	battmgr->xiaomi_prop_value = 0;
	battmgr->xiaomi_prop_data_len = 0;

	ret = qcom_battmgr_request(battmgr, &request, sizeof(request));
	if (!ret && out_data) {
		if (battmgr->xiaomi_prop_data_len < sizeof(request.data)) {
			ret = -ENODATA;
		} else {
			memcpy(out_data, battmgr->xiaomi_prop_data, sizeof(request.data));
		}
	}

	battmgr->xiaomi_prop_pending = false;
	mutex_unlock(&battmgr->lock);

	return ret;
}

static int qcom_battmgr_xiaomi_request_digest(struct qcom_battmgr *battmgr, int opcode,
					      const u8 *digest, u8 slave,
					      u8 *out_digest)
{
	struct qcom_battmgr_xiaomi_digest_request request = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(opcode),
		.property = cpu_to_le32(XM_BATT_VERIFY_DIGEST),
		.slave = slave,
	};
	int ret;

	if (battmgr->variant != XIAOMI_BATTMGR_SM8550)
		return -EOPNOTSUPP;

	if (!battmgr->service_up)
		return -EAGAIN;

	if (digest)
		memcpy(request.digest, digest, sizeof(request.digest));

	mutex_lock(&battmgr->lock);
	battmgr->xiaomi_prop_pending = true;
	battmgr->xiaomi_prop = XM_BATT_VERIFY_DIGEST;
	battmgr->xiaomi_prop_value = 0;
	battmgr->xiaomi_prop_data_len = 0;

	ret = qcom_battmgr_request(battmgr, &request, sizeof(request));
	if (!ret && out_digest) {
		if (battmgr->xiaomi_prop_data_len < sizeof(request.digest)) {
			ret = -ENODATA;
		} else {
			memcpy(out_digest, battmgr->xiaomi_prop_data,
			       sizeof(request.digest));
		}
	}

	battmgr->xiaomi_prop_pending = false;
	mutex_unlock(&battmgr->lock);

	return ret;
}

static int qcom_battmgr_update_status(struct qcom_battmgr *battmgr)
{
	struct qcom_battmgr_update_request request = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(BATTMGR_BAT_STATUS),
		.battery_id = cpu_to_le32(0),
	};

	return qcom_battmgr_request(battmgr, &request, sizeof(request));
}

static int qcom_battmgr_update_info(struct qcom_battmgr *battmgr)
{
	struct qcom_battmgr_update_request request = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(BATTMGR_BAT_INFO),
		.battery_id = cpu_to_le32(0),
	};

	return qcom_battmgr_request(battmgr, &request, sizeof(request));
}

static int qcom_battmgr_update_charge_time(struct qcom_battmgr *battmgr)
{
	struct qcom_battmgr_charge_time_request request = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(BATTMGR_BAT_CHARGE_TIME),
		.battery_id = cpu_to_le32(0),
		.percent = cpu_to_le32(100),
	};

	return qcom_battmgr_request(battmgr, &request, sizeof(request));
}

static int qcom_battmgr_update_discharge_time(struct qcom_battmgr *battmgr)
{
	struct qcom_battmgr_discharge_time_request request = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(BATTMGR_BAT_DISCHARGE_TIME),
		.battery_id = cpu_to_le32(0),
		.rate = cpu_to_le32(0),
	};

	return qcom_battmgr_request(battmgr, &request, sizeof(request));
}

static const u8 sm8350_bat_prop_map[] = {
	[POWER_SUPPLY_PROP_STATUS] = BATT_STATUS,
	[POWER_SUPPLY_PROP_HEALTH] = BATT_HEALTH,
	[POWER_SUPPLY_PROP_PRESENT] = BATT_PRESENT,
	[POWER_SUPPLY_PROP_CHARGE_TYPE] = BATT_CHG_TYPE,
	[POWER_SUPPLY_PROP_CAPACITY] = BATT_CAPACITY,
	[POWER_SUPPLY_PROP_VOLTAGE_OCV] = BATT_VOLT_OCV,
	[POWER_SUPPLY_PROP_VOLTAGE_NOW] = BATT_VOLT_NOW,
	[POWER_SUPPLY_PROP_VOLTAGE_MAX] = BATT_VOLT_MAX,
	[POWER_SUPPLY_PROP_CURRENT_NOW] = BATT_CURR_NOW,
	[POWER_SUPPLY_PROP_TEMP] = BATT_TEMP,
	[POWER_SUPPLY_PROP_TECHNOLOGY] = BATT_TECHNOLOGY,
	[POWER_SUPPLY_PROP_CHARGE_COUNTER] =  BATT_CHG_COUNTER,
	[POWER_SUPPLY_PROP_CYCLE_COUNT] = BATT_CYCLE_COUNT,
	[POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN] =  BATT_CHG_FULL_DESIGN,
	[POWER_SUPPLY_PROP_CHARGE_FULL] = BATT_CHG_FULL,
	[POWER_SUPPLY_PROP_MODEL_NAME] = BATT_MODEL_NAME,
	[POWER_SUPPLY_PROP_TIME_TO_FULL_AVG] = BATT_TTF_AVG,
	[POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG] = BATT_TTE_AVG,
	[POWER_SUPPLY_PROP_INTERNAL_RESISTANCE] = BATT_RESISTANCE,
	[POWER_SUPPLY_PROP_STATE_OF_HEALTH] = BATT_SOH,
	[POWER_SUPPLY_PROP_POWER_NOW] = BATT_POWER_NOW,
	[POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD] = BATT_CHG_CTRL_START_THR,
	[POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD] = BATT_CHG_CTRL_END_THR,
};

static int qcom_battmgr_bat_sm8350_update(struct qcom_battmgr *battmgr,
					  enum power_supply_property psp)
{
	unsigned int prop;
	int ret;

	if (psp >= ARRAY_SIZE(sm8350_bat_prop_map))
		return -EINVAL;

	prop = sm8350_bat_prop_map[psp];

	if (battmgr->variant == XIAOMI_BATTMGR_SM8550 &&
	    psp == POWER_SUPPLY_PROP_CAPACITY) {
		ret = qcom_battmgr_xiaomi_update_capacity(battmgr);
		if (!ret)
			return 0;
	}

	mutex_lock(&battmgr->lock);
	ret = qcom_battmgr_request_property(battmgr, BATTMGR_BAT_PROPERTY_GET, prop, 0);
	mutex_unlock(&battmgr->lock);

	return ret;
}

static int qcom_battmgr_bat_sc8280xp_update(struct qcom_battmgr *battmgr,
					    enum power_supply_property psp)
{
	int ret;

	mutex_lock(&battmgr->lock);

	if (!battmgr->info.valid) {
		ret = qcom_battmgr_update_info(battmgr);
		if (ret < 0)
			goto out_unlock;
		battmgr->info.valid = true;
	}

	ret = qcom_battmgr_update_status(battmgr);
	if (ret < 0)
		goto out_unlock;

	if (psp == POWER_SUPPLY_PROP_TIME_TO_FULL_AVG) {
		ret = qcom_battmgr_update_charge_time(battmgr);
		if (ret < 0) {
			ret = -ENODATA;
			goto out_unlock;
		}
	}

	if (psp == POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG) {
		ret = qcom_battmgr_update_discharge_time(battmgr);
		if (ret < 0) {
			ret = -ENODATA;
			goto out_unlock;
		}
	}

out_unlock:
	mutex_unlock(&battmgr->lock);
	return ret;
}

static int qcom_battmgr_bat_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	struct qcom_battmgr *battmgr = power_supply_get_drvdata(psy);
	enum qcom_battmgr_unit unit = battmgr->unit;
	int ret;

	if (!battmgr->service_up)
		return -EAGAIN;

	if (battmgr->variant == QCOM_BATTMGR_SC8280XP ||
	    battmgr->variant == QCOM_BATTMGR_X1E80100)
		ret = qcom_battmgr_bat_sc8280xp_update(battmgr, psp);
	else
		ret = qcom_battmgr_bat_sm8350_update(battmgr, psp);
	if (ret < 0)
		return ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = battmgr->status.status;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = battmgr->info.charge_type;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = battmgr->status.health;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = battmgr->info.present;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = battmgr->info.technology;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = battmgr->info.cycle_count;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = battmgr->info.voltage_max_design;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = battmgr->info.voltage_max;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = battmgr->status.voltage_now;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		val->intval = battmgr->status.voltage_ocv;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = battmgr->status.current_now;
		break;
	case POWER_SUPPLY_PROP_POWER_NOW:
		val->intval = battmgr->status.power_now;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		if (unit != QCOM_BATTMGR_UNIT_mAh &&
			battmgr->variant != XIAOMI_BATTMGR_SM8550)
			return -ENODATA;
		val->intval = battmgr->info.design_capacity;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		if (unit != QCOM_BATTMGR_UNIT_mAh &&
			battmgr->variant != XIAOMI_BATTMGR_SM8550)
			return -ENODATA;
		val->intval = battmgr->info.last_full_capacity;
		break;
	case POWER_SUPPLY_PROP_CHARGE_EMPTY:
		if (unit != QCOM_BATTMGR_UNIT_mAh)
			return -ENODATA;
		val->intval = battmgr->info.capacity_low;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		if (unit != QCOM_BATTMGR_UNIT_mAh)
			return -ENODATA;
		val->intval = battmgr->status.capacity;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		val->intval = battmgr->info.charge_count;
		break;
	case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
		if (unit != QCOM_BATTMGR_UNIT_mWh)
			return -ENODATA;
		val->intval = battmgr->info.design_capacity;
		break;
	case POWER_SUPPLY_PROP_ENERGY_FULL:
		if (unit != QCOM_BATTMGR_UNIT_mWh)
			return -ENODATA;
		val->intval = battmgr->info.last_full_capacity;
		break;
	case POWER_SUPPLY_PROP_ENERGY_EMPTY:
		if (unit != QCOM_BATTMGR_UNIT_mWh)
			return -ENODATA;
		val->intval = battmgr->info.capacity_low;
		break;
	case POWER_SUPPLY_PROP_ENERGY_NOW:
		if (unit != QCOM_BATTMGR_UNIT_mWh)
			return -ENODATA;
		val->intval = battmgr->status.capacity;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (battmgr->status.percent == (unsigned int)-1)
			return -ENODATA;
		val->intval = battmgr->status.percent;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = battmgr->status.temperature;
		break;
	case POWER_SUPPLY_PROP_INTERNAL_RESISTANCE:
		val->intval = battmgr->status.resistance;
		break;
	case POWER_SUPPLY_PROP_STATE_OF_HEALTH:
		val->intval = battmgr->status.soh_percent;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		val->intval = battmgr->status.discharge_time;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
		val->intval = battmgr->status.charge_time;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
		val->intval = battmgr->info.charge_ctrl_start;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		val->intval = battmgr->info.charge_ctrl_end;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURE_YEAR:
		val->intval = battmgr->info.year;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURE_MONTH:
		val->intval = battmgr->info.month;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURE_DAY:
		val->intval = battmgr->info.day;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = battmgr->info.model_number;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = battmgr->info.oem_info;
		break;
	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		val->strval = battmgr->info.serial_number;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int qcom_battmgr_set_charge_control(struct qcom_battmgr *battmgr,
					   u32 target_soc, u32 delta_soc)
{
	struct qcom_battmgr_charge_ctrl_request request = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(BATTMGR_CHG_CTRL_LIMIT_EN),
		.enable = cpu_to_le32(1),
		.target_soc = cpu_to_le32(target_soc),
		.delta_soc = cpu_to_le32(delta_soc),
	};

	return qcom_battmgr_request(battmgr, &request, sizeof(request));
}

static int qcom_battmgr_set_charge_start_threshold(struct qcom_battmgr *battmgr, int start_soc)
{
	u32 target_soc, delta_soc;
	int ret;

	start_soc = clamp(start_soc, CHARGE_CTRL_START_THR_MIN, CHARGE_CTRL_START_THR_MAX);

	/*
	 * If the new start threshold is larger than the old end threshold,
	 * move the end threshold one step (DELTA_SOC) after the new start
	 * threshold.
	 */
	if (start_soc > battmgr->info.charge_ctrl_end) {
		target_soc = start_soc + CHARGE_CTRL_DELTA_SOC;
		target_soc = min_t(u32, target_soc, CHARGE_CTRL_END_THR_MAX);
		delta_soc = target_soc - start_soc;
		delta_soc = min_t(u32, delta_soc, CHARGE_CTRL_DELTA_SOC);
	} else {
		target_soc =  battmgr->info.charge_ctrl_end;
		delta_soc = battmgr->info.charge_ctrl_end - start_soc;
	}

	mutex_lock(&battmgr->lock);
	ret = qcom_battmgr_set_charge_control(battmgr, target_soc, delta_soc);
	mutex_unlock(&battmgr->lock);
	if (!ret) {
		battmgr->info.charge_ctrl_start = start_soc;
		battmgr->info.charge_ctrl_end = target_soc;
	}

	return 0;
}

static int qcom_battmgr_set_charge_end_threshold(struct qcom_battmgr *battmgr, int end_soc)
{
	u32 delta_soc = CHARGE_CTRL_DELTA_SOC;
	int ret;

	end_soc = clamp(end_soc, CHARGE_CTRL_END_THR_MIN, CHARGE_CTRL_END_THR_MAX);

	if (battmgr->info.charge_ctrl_start && end_soc > battmgr->info.charge_ctrl_start)
		delta_soc = end_soc - battmgr->info.charge_ctrl_start;

	mutex_lock(&battmgr->lock);
	ret = qcom_battmgr_set_charge_control(battmgr, end_soc, delta_soc);
	mutex_unlock(&battmgr->lock);
	if (!ret) {
		battmgr->info.charge_ctrl_start = end_soc - delta_soc;
		battmgr->info.charge_ctrl_end = end_soc;
	}

	return 0;
}

static int qcom_battmgr_charge_control_thresholds_init(struct qcom_battmgr *battmgr)
{
	int ret;
	u8 en, end_soc, start_soc, delta_soc;

	ret = nvmem_cell_read_u8(battmgr->dev->parent, "charge_limit_en", &en);
	if (!ret && en != 0) {
		ret = nvmem_cell_read_u8(battmgr->dev->parent, "charge_limit_end", &end_soc);
		if (ret < 0)
			return ret;

		ret = nvmem_cell_read_u8(battmgr->dev->parent, "charge_limit_delta", &delta_soc);
		if (ret < 0)
			return ret;

		if (delta_soc >= end_soc)
			return -EINVAL;

		start_soc = end_soc - delta_soc;
		end_soc = clamp(end_soc, CHARGE_CTRL_END_THR_MIN, CHARGE_CTRL_END_THR_MAX);
		start_soc = clamp(start_soc, CHARGE_CTRL_START_THR_MIN, CHARGE_CTRL_START_THR_MAX);

		battmgr->info.charge_ctrl_start = start_soc;
		battmgr->info.charge_ctrl_end = end_soc;
	}

	return 0;
}

static int qcom_battmgr_bat_is_writeable(struct power_supply *psy,
					 enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		return 1;
	default:
		return 0;
	}

	return 0;
}

static int qcom_battmgr_bat_set_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 const union power_supply_propval *pval)
{
	struct qcom_battmgr *battmgr = power_supply_get_drvdata(psy);

	if (!battmgr->service_up)
		return -EAGAIN;

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
		return qcom_battmgr_set_charge_start_threshold(battmgr, pval->intval);
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		return qcom_battmgr_set_charge_end_threshold(battmgr, pval->intval);
	default:
		return -EINVAL;
	}

	return 0;
}

enum qcom_battmgr_xiaomi_attr_format {
	QCOM_BATTMGR_XIAOMI_DEC,
	QCOM_BATTMGR_XIAOMI_HEX,
	QCOM_BATTMGR_XIAOMI_USB_TYPE,
};

struct qcom_battmgr_xiaomi_attr {
	struct device_attribute dev_attr;
	unsigned int prop;
	enum qcom_battmgr_xiaomi_attr_format format;
};

static const char *qcom_battmgr_xiaomi_usb_type_string(unsigned int val)
{
	switch (val) {
	case POWER_SUPPLY_USB_TYPE_UNKNOWN:
		return "Unknown";
	case POWER_SUPPLY_USB_TYPE_SDP:
		return "SDP";
	case POWER_SUPPLY_USB_TYPE_DCP:
		return "DCP";
	case POWER_SUPPLY_USB_TYPE_CDP:
		return "CDP";
	case POWER_SUPPLY_USB_TYPE_ACA:
		return "ACA";
	case POWER_SUPPLY_USB_TYPE_C:
		return "C";
	case POWER_SUPPLY_USB_TYPE_PD:
		return "PD";
	case POWER_SUPPLY_USB_TYPE_PD_DRP:
		return "PD_DRP";
	case POWER_SUPPLY_USB_TYPE_PD_PPS:
		return "PD_PPS";
	case POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID:
		return "BrickID";
	default:
		return NULL;
	}
}

static ssize_t qcom_battmgr_xiaomi_prop_show(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct qcom_battmgr_xiaomi_attr *xiaomi_attr =
		container_of(attr, struct qcom_battmgr_xiaomi_attr, dev_attr);
	struct qcom_battmgr *battmgr = dev_get_drvdata(dev);
	const char *str;
	u32 val;
	int ret;

	ret = qcom_battmgr_xiaomi_request_property(battmgr, BATTMGR_XM_PROPERTY_GET,
						   xiaomi_attr->prop, 0, &val);
	if (ret < 0)
		return ret;

	switch (xiaomi_attr->format) {
	case QCOM_BATTMGR_XIAOMI_HEX:
		return sysfs_emit(buf, "%08x\n", val);
	case QCOM_BATTMGR_XIAOMI_USB_TYPE:
		str = qcom_battmgr_xiaomi_usb_type_string(val);
		if (str)
			return sysfs_emit(buf, "%s\n", str);

		return sysfs_emit(buf, "%u\n", val);
	case QCOM_BATTMGR_XIAOMI_DEC:
	default:
		return sysfs_emit(buf, "%u\n", val);
	}
}

static ssize_t qcom_battmgr_xiaomi_prop_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t count)
{
	struct qcom_battmgr_xiaomi_attr *xiaomi_attr =
		container_of(attr, struct qcom_battmgr_xiaomi_attr, dev_attr);
	struct qcom_battmgr *battmgr = dev_get_drvdata(dev);
	u32 val;
	int ret;

	ret = kstrtou32(buf, 0, &val);
	if (ret)
		return ret;

	ret = qcom_battmgr_xiaomi_request_property(battmgr, BATTMGR_XM_PROPERTY_SET,
						   xiaomi_attr->prop, val, NULL);
	if (ret < 0)
		return ret;

	return count;
}

static int qcom_battmgr_xiaomi_parse_vdm_payload(const char *hex, __le32 data[4])
{
	u8 bytes[16];
	int i, ret;

	if (strlen(hex) != sizeof(bytes) * 2)
		return -EINVAL;

	ret = hex2bin(bytes, hex, sizeof(bytes));
	if (ret)
		return ret;

	for (i = 0; i < 4; i++)
		data[i] = cpu_to_le32(get_unaligned_be32(&bytes[i * sizeof(__be32)]));

	return 0;
}

static ssize_t request_vdm_cmd_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct qcom_battmgr *battmgr = dev_get_drvdata(dev);
	__le32 auth[4];
	u32 state, val;
	int ret;

	ret = qcom_battmgr_xiaomi_request_property(battmgr, BATTMGR_XM_PROPERTY_GET,
						   XM_BATT_UVDM_STATE, 0, &state);
	if (ret < 0)
		return ret;

	switch (state) {
	case 1:
		ret = qcom_battmgr_xiaomi_request_property(battmgr, BATTMGR_XM_PROPERTY_GET,
							   XM_BATT_VDM_CMD_CHARGER_VERSION,
							   0, &val);
		if (ret < 0)
			return ret;
		return sysfs_emit(buf, "%u,%u\n", state, val);
	case 2:
		ret = qcom_battmgr_xiaomi_request_property(battmgr, BATTMGR_XM_PROPERTY_GET,
							   XM_BATT_VDM_CMD_CHARGER_VOLTAGE,
							   0, &val);
		if (ret < 0)
			return ret;
		return sysfs_emit(buf, "%u,%u\n", state, val);
	case 3:
		ret = qcom_battmgr_xiaomi_request_property(battmgr, BATTMGR_XM_PROPERTY_GET,
							   XM_BATT_VDM_CMD_CHARGER_TEMP,
							   0, &val);
		if (ret < 0)
			return ret;
		return sysfs_emit(buf, "%u,%u\n", state, val);
	case 5:
		ret = qcom_battmgr_xiaomi_request_vdm(battmgr, BATTMGR_XM_PROPERTY_GET,
						      XM_BATT_VDM_CMD_AUTHENTICATION,
						      NULL, auth);
		if (ret < 0)
			return ret;
		return sysfs_emit(buf, "%u,%08x%08x%08x%08x\n", state,
				  le32_to_cpu(auth[0]), le32_to_cpu(auth[1]),
				  le32_to_cpu(auth[2]), le32_to_cpu(auth[3]));
	default:
		return sysfs_emit(buf, "%u,Null\n", state);
	}
}

static ssize_t request_vdm_cmd_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct qcom_battmgr *battmgr = dev_get_drvdata(dev);
	char hex[33] = {};
	__le32 data[4];
	unsigned int cmd;
	u8 bytes[4];
	u32 val = 0;
	int prop;
	int ret;

	ret = sscanf(buf, "%u,%32s", &cmd, hex);
	if (ret < 1)
		return -EINVAL;

	switch (cmd) {
	case 1:
		prop = XM_BATT_VDM_CMD_CHARGER_VERSION;
		break;
	case 2:
		prop = XM_BATT_VDM_CMD_CHARGER_VOLTAGE;
		break;
	case 3:
		prop = XM_BATT_VDM_CMD_CHARGER_TEMP;
		break;
	case 4:
		prop = XM_BATT_VDM_CMD_SESSION_SEED;
		goto send_vdm;
	case 5:
		prop = XM_BATT_VDM_CMD_AUTHENTICATION;
		goto send_vdm;
	case 6:
		prop = XM_BATT_VDM_CMD_VERIFIED;
		goto send_value;
	case 7:
		prop = XM_BATT_VDM_CMD_REMOVE_COMPENSATION;
		goto send_value;
	case 8:
		prop = XM_BATT_VDM_CMD_REVERSE_AUTHEN;
		goto send_vdm;
	default:
		return -EINVAL;
	}

	ret = qcom_battmgr_xiaomi_request_property(battmgr, BATTMGR_XM_PROPERTY_SET,
						   prop, 0, NULL);
	if (ret < 0)
		return ret;

	return count;

send_value:
	if (strlen(hex) != sizeof(bytes) * 2)
		return -EINVAL;

	ret = hex2bin(bytes, hex, sizeof(bytes));
	if (ret)
		return ret;

	val = get_unaligned_le32(bytes);
	ret = qcom_battmgr_xiaomi_request_property(battmgr, BATTMGR_XM_PROPERTY_SET,
						   prop, val, NULL);
	if (ret < 0)
		return ret;

	return count;

send_vdm:
	ret = qcom_battmgr_xiaomi_parse_vdm_payload(hex, data);
	if (ret)
		return ret;

	ret = qcom_battmgr_xiaomi_request_vdm(battmgr, BATTMGR_XM_PROPERTY_SET,
					      prop, data, NULL);
	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR_RW(request_vdm_cmd);

static ssize_t verify_slave_flag_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct qcom_battmgr *battmgr = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", battmgr->xiaomi_verify_slave);
}

static ssize_t verify_slave_flag_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct qcom_battmgr *battmgr = dev_get_drvdata(dev);
	u8 val;
	int ret;

	ret = kstrtou8(buf, 0, &val);
	if (ret)
		return ret;

	battmgr->xiaomi_verify_slave = !!val;

	return count;
}

static DEVICE_ATTR_RW(verify_slave_flag);

static ssize_t verify_digest_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct qcom_battmgr *battmgr = dev_get_drvdata(dev);
	u8 digest[32];
	char hex[65];
	int ret;

	ret = qcom_battmgr_xiaomi_request_digest(battmgr, BATTMGR_XM_PROPERTY_GET,
						 NULL, battmgr->xiaomi_verify_slave,
						 digest);
	if (ret < 0)
		return ret;

	bin2hex(hex, digest, sizeof(digest));
	hex[sizeof(hex) - 1] = '\0';

	return sysfs_emit(buf, "%s\n", hex);
}

static ssize_t verify_digest_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct qcom_battmgr *battmgr = dev_get_drvdata(dev);
	char hex[65] = {};
	u8 digest[32];
	int ret;

	if (sscanf(buf, "%64s", hex) != 1)
		return -EINVAL;

	if (strlen(hex) != sizeof(digest) * 2)
		return -EINVAL;

	ret = hex2bin(digest, hex, sizeof(digest));
	if (ret)
		return ret;

	ret = qcom_battmgr_xiaomi_request_digest(battmgr, BATTMGR_XM_PROPERTY_SET,
						 digest, battmgr->xiaomi_verify_slave,
						 NULL);
	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR_RW(verify_digest);

#define QCOM_BATTMGR_XIAOMI_ATTR_RO(_name, _prop, _format)		\
	static struct qcom_battmgr_xiaomi_attr qcom_battmgr_xiaomi_attr_##_name = { \
		.dev_attr = __ATTR(_name, 0444, qcom_battmgr_xiaomi_prop_show, NULL), \
		.prop = _prop,						\
		.format = _format,					\
	}

#define QCOM_BATTMGR_XIAOMI_ATTR_RW(_name, _prop, _format)		\
	static struct qcom_battmgr_xiaomi_attr qcom_battmgr_xiaomi_attr_##_name = { \
		.dev_attr = __ATTR(_name, 0644, qcom_battmgr_xiaomi_prop_show, \
				   qcom_battmgr_xiaomi_prop_store),	\
		.prop = _prop,						\
		.format = _format,					\
	}

#define QCOM_BATTMGR_XIAOMI_ATTR_WO(_name, _prop, _format)		\
	static struct qcom_battmgr_xiaomi_attr qcom_battmgr_xiaomi_attr_##_name = { \
		.dev_attr = __ATTR(_name, 0200, NULL, qcom_battmgr_xiaomi_prop_store), \
		.prop = _prop,						\
		.format = _format,					\
	}

QCOM_BATTMGR_XIAOMI_ATTR_RW(authentic, XM_BATT_AUTHENTIC, QCOM_BATTMGR_XIAOMI_DEC);
QCOM_BATTMGR_XIAOMI_ATTR_RO(real_type, XM_BATT_REAL_TYPE, QCOM_BATTMGR_XIAOMI_USB_TYPE);
QCOM_BATTMGR_XIAOMI_ATTR_RW(verify_process, XM_BATT_VERIFY_PROCESS, QCOM_BATTMGR_XIAOMI_DEC);
QCOM_BATTMGR_XIAOMI_ATTR_RO(adapter_id, XM_BATT_ADAPTER_ID, QCOM_BATTMGR_XIAOMI_HEX);
QCOM_BATTMGR_XIAOMI_ATTR_RO(adapter_svid, XM_BATT_ADAPTER_SVID, QCOM_BATTMGR_XIAOMI_DEC);
QCOM_BATTMGR_XIAOMI_ATTR_RW(pd_verifed, XM_BATT_PD_VERIFED, QCOM_BATTMGR_XIAOMI_DEC);
QCOM_BATTMGR_XIAOMI_ATTR_RO(pdo2, XM_BATT_PDO2, QCOM_BATTMGR_XIAOMI_HEX);
QCOM_BATTMGR_XIAOMI_ATTR_RO(bq2597x_chip_ok, XM_BATT_BQ2597X_CHIP_OK, QCOM_BATTMGR_XIAOMI_DEC);
QCOM_BATTMGR_XIAOMI_ATTR_RO(bq2597x_slave_chip_ok, XM_BATT_BQ2597X_SLAVE_CHIP_OK,
			    QCOM_BATTMGR_XIAOMI_DEC);
QCOM_BATTMGR_XIAOMI_ATTR_RO(bq2597x_bus_current, XM_BATT_BQ2597X_BUS_CURRENT,
			    QCOM_BATTMGR_XIAOMI_DEC);
QCOM_BATTMGR_XIAOMI_ATTR_RO(bq2597x_slave_bus_current, XM_BATT_BQ2597X_SLAVE_BUS_CURRENT,
			    QCOM_BATTMGR_XIAOMI_DEC);
QCOM_BATTMGR_XIAOMI_ATTR_RO(bq2597x_bus_delta, XM_BATT_BQ2597X_BUS_DELTA, QCOM_BATTMGR_XIAOMI_DEC);
QCOM_BATTMGR_XIAOMI_ATTR_RO(bq2597x_bus_voltage, XM_BATT_BQ2597X_BUS_VOLTAGE,
			    QCOM_BATTMGR_XIAOMI_DEC);
QCOM_BATTMGR_XIAOMI_ATTR_RO(bq2597x_battery_present, XM_BATT_BQ2597X_BATTERY_PRESENT,
			    QCOM_BATTMGR_XIAOMI_DEC);
QCOM_BATTMGR_XIAOMI_ATTR_RO(bq2597x_slave_battery_present, XM_BATT_BQ2597X_SLAVE_BATTERY_PRESENT,
			    QCOM_BATTMGR_XIAOMI_DEC);
QCOM_BATTMGR_XIAOMI_ATTR_RO(bq2597x_battery_voltage, XM_BATT_BQ2597X_BATTERY_VOLTAGE,
			    QCOM_BATTMGR_XIAOMI_DEC);
QCOM_BATTMGR_XIAOMI_ATTR_RO(fastchg_mode, XM_BATT_FASTCHGMODE, QCOM_BATTMGR_XIAOMI_DEC);
QCOM_BATTMGR_XIAOMI_ATTR_RO(quick_charge_type, XM_BATT_QUICK_CHARGE_TYPE, QCOM_BATTMGR_XIAOMI_DEC);
QCOM_BATTMGR_XIAOMI_ATTR_RO(apdo_max, XM_BATT_APDO_MAX, QCOM_BATTMGR_XIAOMI_DEC);
QCOM_BATTMGR_XIAOMI_ATTR_RO(power_max, XM_BATT_POWER_MAX, QCOM_BATTMGR_XIAOMI_DEC);
QCOM_BATTMGR_XIAOMI_ATTR_RO(pen_mac_l, XM_BATT_PEN_MAC_L, QCOM_BATTMGR_XIAOMI_HEX);
QCOM_BATTMGR_XIAOMI_ATTR_RO(pen_mac_h, XM_BATT_PEN_MAC_H, QCOM_BATTMGR_XIAOMI_HEX);
QCOM_BATTMGR_XIAOMI_ATTR_RO(tx_iout, XM_BATT_PEN_TX_IOUT, QCOM_BATTMGR_XIAOMI_DEC);
QCOM_BATTMGR_XIAOMI_ATTR_RO(tx_vout, XM_BATT_PEN_TX_VOUT, QCOM_BATTMGR_XIAOMI_DEC);
QCOM_BATTMGR_XIAOMI_ATTR_RO(pen_soc, XM_BATT_PEN_SOC, QCOM_BATTMGR_XIAOMI_DEC);
QCOM_BATTMGR_XIAOMI_ATTR_RO(pen_hall3, XM_BATT_PEN_HALL3, QCOM_BATTMGR_XIAOMI_DEC);
QCOM_BATTMGR_XIAOMI_ATTR_RO(pen_hall4, XM_BATT_PEN_HALL4, QCOM_BATTMGR_XIAOMI_DEC);
QCOM_BATTMGR_XIAOMI_ATTR_RO(pen_tx_ss, XM_BATT_PEN_TX_SS, QCOM_BATTMGR_XIAOMI_DEC);
QCOM_BATTMGR_XIAOMI_ATTR_RO(pen_place_err, XM_BATT_PEN_PLACE_ERR, QCOM_BATTMGR_XIAOMI_DEC);
QCOM_BATTMGR_XIAOMI_ATTR_RW(fake_ss, XM_BATT_PEN_FAKE_SS, QCOM_BATTMGR_XIAOMI_DEC);
QCOM_BATTMGR_XIAOMI_ATTR_WO(fg_fastcharge, XM_BATT_FG_FAST_CHARGE, QCOM_BATTMGR_XIAOMI_DEC);
QCOM_BATTMGR_XIAOMI_ATTR_RW(slave_authentic, XM_BATT_SLAVE_AUTHENTIC, QCOM_BATTMGR_XIAOMI_DEC);
QCOM_BATTMGR_XIAOMI_ATTR_RW(pps_ptf, XM_BATT_PPS_PTF, QCOM_BATTMGR_XIAOMI_DEC);

static struct attribute *qcom_battmgr_xiaomi_attrs[] = {
	&qcom_battmgr_xiaomi_attr_authentic.dev_attr.attr,
	&qcom_battmgr_xiaomi_attr_real_type.dev_attr.attr,
	&qcom_battmgr_xiaomi_attr_verify_process.dev_attr.attr,
	&qcom_battmgr_xiaomi_attr_adapter_id.dev_attr.attr,
	&qcom_battmgr_xiaomi_attr_adapter_svid.dev_attr.attr,
	&qcom_battmgr_xiaomi_attr_pd_verifed.dev_attr.attr,
	&qcom_battmgr_xiaomi_attr_pdo2.dev_attr.attr,
	&qcom_battmgr_xiaomi_attr_bq2597x_chip_ok.dev_attr.attr,
	&qcom_battmgr_xiaomi_attr_bq2597x_slave_chip_ok.dev_attr.attr,
	&qcom_battmgr_xiaomi_attr_bq2597x_bus_current.dev_attr.attr,
	&qcom_battmgr_xiaomi_attr_bq2597x_slave_bus_current.dev_attr.attr,
	&qcom_battmgr_xiaomi_attr_bq2597x_bus_delta.dev_attr.attr,
	&qcom_battmgr_xiaomi_attr_bq2597x_bus_voltage.dev_attr.attr,
	&qcom_battmgr_xiaomi_attr_bq2597x_battery_present.dev_attr.attr,
	&qcom_battmgr_xiaomi_attr_bq2597x_slave_battery_present.dev_attr.attr,
	&qcom_battmgr_xiaomi_attr_bq2597x_battery_voltage.dev_attr.attr,
	&qcom_battmgr_xiaomi_attr_fastchg_mode.dev_attr.attr,
	&qcom_battmgr_xiaomi_attr_quick_charge_type.dev_attr.attr,
	&qcom_battmgr_xiaomi_attr_apdo_max.dev_attr.attr,
	&qcom_battmgr_xiaomi_attr_power_max.dev_attr.attr,
	&qcom_battmgr_xiaomi_attr_pen_mac_l.dev_attr.attr,
	&qcom_battmgr_xiaomi_attr_pen_mac_h.dev_attr.attr,
	&qcom_battmgr_xiaomi_attr_tx_iout.dev_attr.attr,
	&qcom_battmgr_xiaomi_attr_tx_vout.dev_attr.attr,
	&qcom_battmgr_xiaomi_attr_pen_soc.dev_attr.attr,
	&qcom_battmgr_xiaomi_attr_pen_hall3.dev_attr.attr,
	&qcom_battmgr_xiaomi_attr_pen_hall4.dev_attr.attr,
	&qcom_battmgr_xiaomi_attr_pen_tx_ss.dev_attr.attr,
	&qcom_battmgr_xiaomi_attr_pen_place_err.dev_attr.attr,
	&qcom_battmgr_xiaomi_attr_fake_ss.dev_attr.attr,
	&qcom_battmgr_xiaomi_attr_fg_fastcharge.dev_attr.attr,
	&qcom_battmgr_xiaomi_attr_slave_authentic.dev_attr.attr,
	&qcom_battmgr_xiaomi_attr_pps_ptf.dev_attr.attr,
	&dev_attr_request_vdm_cmd.attr,
	&dev_attr_verify_slave_flag.attr,
	&dev_attr_verify_digest.attr,
	NULL
};

static const struct attribute_group qcom_battmgr_xiaomi_attr_group = {
	.name = "xiaomi",
	.attrs = qcom_battmgr_xiaomi_attrs,
};

static const enum power_supply_property sc8280xp_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_EMPTY,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
	POWER_SUPPLY_PROP_ENERGY_FULL,
	POWER_SUPPLY_PROP_ENERGY_EMPTY,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_MANUFACTURE_YEAR,
	POWER_SUPPLY_PROP_MANUFACTURE_MONTH,
	POWER_SUPPLY_PROP_MANUFACTURE_DAY,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
};

static const struct power_supply_desc sc8280xp_bat_psy_desc = {
	.name = "qcom-battmgr-bat",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = sc8280xp_bat_props,
	.num_properties = ARRAY_SIZE(sc8280xp_bat_props),
	.get_property = qcom_battmgr_bat_get_property,
};

static const enum power_supply_property x1e80100_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_EMPTY,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
	POWER_SUPPLY_PROP_ENERGY_FULL,
	POWER_SUPPLY_PROP_ENERGY_EMPTY,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_MANUFACTURE_YEAR,
	POWER_SUPPLY_PROP_MANUFACTURE_MONTH,
	POWER_SUPPLY_PROP_MANUFACTURE_DAY,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD,
};

static const struct power_supply_desc x1e80100_bat_psy_desc = {
	.name = "qcom-battmgr-bat",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = x1e80100_bat_props,
	.num_properties = ARRAY_SIZE(x1e80100_bat_props),
	.get_property = qcom_battmgr_bat_get_property,
	.set_property = qcom_battmgr_bat_set_property,
	.property_is_writeable = qcom_battmgr_bat_is_writeable,
};

static const enum power_supply_property sm8350_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_INTERNAL_RESISTANCE,
	POWER_SUPPLY_PROP_STATE_OF_HEALTH,
	POWER_SUPPLY_PROP_POWER_NOW,
};

static const struct power_supply_desc sm8350_bat_psy_desc = {
	.name = "qcom-battmgr-bat",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = sm8350_bat_props,
	.num_properties = ARRAY_SIZE(sm8350_bat_props),
	.get_property = qcom_battmgr_bat_get_property,
};

static const enum power_supply_property sm8550_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_INTERNAL_RESISTANCE,
	POWER_SUPPLY_PROP_STATE_OF_HEALTH,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD,
};

static const struct power_supply_desc sm8550_bat_psy_desc = {
	.name = "qcom-battmgr-bat",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = sm8550_bat_props,
	.num_properties = ARRAY_SIZE(sm8550_bat_props),
	.get_property = qcom_battmgr_bat_get_property,
	.set_property = qcom_battmgr_bat_set_property,
	.property_is_writeable = qcom_battmgr_bat_is_writeable,
};

static int qcom_battmgr_ac_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct qcom_battmgr *battmgr = power_supply_get_drvdata(psy);
	int ret;

	if (!battmgr->service_up)
		return -EAGAIN;

	ret = qcom_battmgr_bat_sc8280xp_update(battmgr, psp);
	if (ret)
		return ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = battmgr->ac.online;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const enum power_supply_property sc8280xp_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static const struct power_supply_desc sc8280xp_ac_psy_desc = {
	.name = "qcom-battmgr-ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = sc8280xp_ac_props,
	.num_properties = ARRAY_SIZE(sc8280xp_ac_props),
	.get_property = qcom_battmgr_ac_get_property,
};

static const u8 sm8350_usb_prop_map[] = {
	[POWER_SUPPLY_PROP_ONLINE] = USB_ONLINE,
	[POWER_SUPPLY_PROP_VOLTAGE_NOW] = USB_VOLT_NOW,
	[POWER_SUPPLY_PROP_VOLTAGE_MAX] = USB_VOLT_MAX,
	[POWER_SUPPLY_PROP_CURRENT_NOW] = USB_CURR_NOW,
	[POWER_SUPPLY_PROP_CURRENT_MAX] = USB_CURR_MAX,
	[POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT] = USB_INPUT_CURR_LIMIT,
	[POWER_SUPPLY_PROP_USB_TYPE] = USB_TYPE,
};

static int qcom_battmgr_usb_sm8350_update(struct qcom_battmgr *battmgr,
					  enum power_supply_property psp)
{
	unsigned int prop;
	int ret;

	if (psp >= ARRAY_SIZE(sm8350_usb_prop_map))
		return -EINVAL;

	prop = sm8350_usb_prop_map[psp];

	mutex_lock(&battmgr->lock);
	ret = qcom_battmgr_request_property(battmgr, BATTMGR_USB_PROPERTY_GET, prop, 0);
	mutex_unlock(&battmgr->lock);

	return ret;
}

static int qcom_battmgr_usb_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	struct qcom_battmgr *battmgr = power_supply_get_drvdata(psy);
	int ret;

	if (!battmgr->service_up)
		return -EAGAIN;

	if (battmgr->variant == QCOM_BATTMGR_SC8280XP ||
	    battmgr->variant == QCOM_BATTMGR_X1E80100)
		ret = qcom_battmgr_bat_sc8280xp_update(battmgr, psp);
	else
		ret = qcom_battmgr_usb_sm8350_update(battmgr, psp);
	if (ret)
		return ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = battmgr->usb.online;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = battmgr->usb.voltage_now;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = battmgr->usb.voltage_max;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = battmgr->usb.current_now;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = battmgr->usb.current_max;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		val->intval = battmgr->usb.current_limit;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = battmgr->usb.usb_type;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const enum power_supply_property sc8280xp_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static const struct power_supply_desc sc8280xp_usb_psy_desc = {
	.name = "qcom-battmgr-usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = sc8280xp_usb_props,
	.num_properties = ARRAY_SIZE(sc8280xp_usb_props),
	.get_property = qcom_battmgr_usb_get_property,
	.usb_types = BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN) |
		     BIT(POWER_SUPPLY_USB_TYPE_SDP)     |
		     BIT(POWER_SUPPLY_USB_TYPE_DCP)     |
		     BIT(POWER_SUPPLY_USB_TYPE_CDP)     |
		     BIT(POWER_SUPPLY_USB_TYPE_ACA)     |
		     BIT(POWER_SUPPLY_USB_TYPE_C)       |
		     BIT(POWER_SUPPLY_USB_TYPE_PD)      |
		     BIT(POWER_SUPPLY_USB_TYPE_PD_DRP)  |
		     BIT(POWER_SUPPLY_USB_TYPE_PD_PPS)  |
		     BIT(POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID),
};

static const enum power_supply_property sm8350_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_USB_TYPE,
};

static const struct power_supply_desc sm8350_usb_psy_desc = {
	.name = "qcom-battmgr-usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = sm8350_usb_props,
	.num_properties = ARRAY_SIZE(sm8350_usb_props),
	.get_property = qcom_battmgr_usb_get_property,
	.usb_types = BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN) |
		     BIT(POWER_SUPPLY_USB_TYPE_SDP)     |
		     BIT(POWER_SUPPLY_USB_TYPE_DCP)     |
		     BIT(POWER_SUPPLY_USB_TYPE_CDP)     |
		     BIT(POWER_SUPPLY_USB_TYPE_ACA)     |
		     BIT(POWER_SUPPLY_USB_TYPE_C)       |
		     BIT(POWER_SUPPLY_USB_TYPE_PD)      |
		     BIT(POWER_SUPPLY_USB_TYPE_PD_DRP)  |
		     BIT(POWER_SUPPLY_USB_TYPE_PD_PPS)  |
		     BIT(POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID),
};

static const u8 sm8350_wls_prop_map[] = {
	[POWER_SUPPLY_PROP_ONLINE] = WLS_ONLINE,
	[POWER_SUPPLY_PROP_VOLTAGE_NOW] = WLS_VOLT_NOW,
	[POWER_SUPPLY_PROP_VOLTAGE_MAX] = WLS_VOLT_MAX,
	[POWER_SUPPLY_PROP_CURRENT_NOW] = WLS_CURR_NOW,
	[POWER_SUPPLY_PROP_CURRENT_MAX] = WLS_CURR_MAX,
};

static int qcom_battmgr_wls_sm8350_update(struct qcom_battmgr *battmgr,
					  enum power_supply_property psp)
{
	unsigned int prop;
	int ret;

	if (psp >= ARRAY_SIZE(sm8350_wls_prop_map))
		return -EINVAL;

	prop = sm8350_wls_prop_map[psp];

	mutex_lock(&battmgr->lock);
	ret = qcom_battmgr_request_property(battmgr, BATTMGR_WLS_PROPERTY_GET, prop, 0);
	mutex_unlock(&battmgr->lock);

	return ret;
}

static int qcom_battmgr_wls_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	struct qcom_battmgr *battmgr = power_supply_get_drvdata(psy);
	int ret;

	if (!battmgr->service_up)
		return -EAGAIN;

	if (battmgr->variant == QCOM_BATTMGR_SC8280XP ||
	    battmgr->variant == QCOM_BATTMGR_X1E80100)
		ret = qcom_battmgr_bat_sc8280xp_update(battmgr, psp);
	else
		ret = qcom_battmgr_wls_sm8350_update(battmgr, psp);
	if (ret < 0)
		return ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = battmgr->wireless.online;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = battmgr->wireless.voltage_now;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = battmgr->wireless.voltage_max;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = battmgr->wireless.current_now;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = battmgr->wireless.current_max;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const enum power_supply_property sc8280xp_wls_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static const struct power_supply_desc sc8280xp_wls_psy_desc = {
	.name = "qcom-battmgr-wls",
	.type = POWER_SUPPLY_TYPE_WIRELESS,
	.properties = sc8280xp_wls_props,
	.num_properties = ARRAY_SIZE(sc8280xp_wls_props),
	.get_property = qcom_battmgr_wls_get_property,
};

static const enum power_supply_property sm8350_wls_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static const struct power_supply_desc sm8350_wls_psy_desc = {
	.name = "qcom-battmgr-wls",
	.type = POWER_SUPPLY_TYPE_WIRELESS,
	.properties = sm8350_wls_props,
	.num_properties = ARRAY_SIZE(sm8350_wls_props),
	.get_property = qcom_battmgr_wls_get_property,
};

static void qcom_battmgr_notification(struct qcom_battmgr *battmgr,
				      const struct qcom_battmgr_message *msg,
				      int len)
{
	size_t payload_len = len - sizeof(struct pmic_glink_hdr);
	unsigned int notification;

	if (payload_len != sizeof(msg->notification)) {
		dev_warn(battmgr->dev, "ignoring notification with invalid length\n");
		return;
	}

	notification = le32_to_cpu(msg->notification);
	notification &= 0xff;
	switch (notification) {
	case NOTIF_BAT_INFO:
		battmgr->info.valid = false;
		fallthrough;
	case NOTIF_BAT_STATUS:
	case NOTIF_BAT_PROPERTY:
	case NOTIF_BAT_CHARGING_STATE:
		power_supply_changed(battmgr->bat_psy);
		break;
	case NOTIF_USB_PROPERTY:
		power_supply_changed(battmgr->usb_psy);
		break;
	case NOTIF_WLS_PROPERTY:
		power_supply_changed(battmgr->wls_psy);
		break;
	default:
		if (battmgr->variant != XIAOMI_BATTMGR_SM8550)
			dev_err(battmgr->dev, "unknown notification: %#x\n", notification);
		break;
	}
}

static void qcom_battmgr_sc8280xp_strcpy(char *dest, const char *src)
{
	size_t len = src[0];

	/* Some firmware versions return Pascal-style strings */
	if (len < BATTMGR_STRING_LEN && len == strnlen(src + 1, BATTMGR_STRING_LEN - 1)) {
		memcpy(dest, src + 1, len);
		dest[len] = '\0';
	} else {
		memcpy(dest, src, BATTMGR_STRING_LEN);
	}
}

static unsigned int qcom_battmgr_sc8280xp_parse_technology(const char *chemistry)
{
	if ((!strncmp(chemistry, "LIO", BATTMGR_CHEMISTRY_LEN)) ||
	    (!strncmp(chemistry, "OOI", BATTMGR_CHEMISTRY_LEN)))
		return POWER_SUPPLY_TECHNOLOGY_LION;
	if (!strncmp(chemistry, "LIP", BATTMGR_CHEMISTRY_LEN) ||
	    !strncmp(chemistry, "LiP", BATTMGR_CHEMISTRY_LEN))
		return POWER_SUPPLY_TECHNOLOGY_LIPO;

	pr_err("Unknown battery technology '%s'\n", chemistry);
	return POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
}

static unsigned int qcom_battmgr_sc8280xp_convert_temp(unsigned int temperature)
{
	return DIV_ROUND_CLOSEST(temperature, 10);
}

static void qcom_battmgr_sc8280xp_callback(struct qcom_battmgr *battmgr,
					   const struct qcom_battmgr_message *resp,
					   size_t len)
{
	unsigned int opcode = le32_to_cpu(resp->hdr.opcode);
	unsigned int source;
	unsigned int state;
	size_t payload_len = len - sizeof(struct pmic_glink_hdr);

	if (payload_len < sizeof(__le32)) {
		dev_warn(battmgr->dev, "invalid payload length for %#x: %zd\n",
			 opcode, len);
		return;
	}

	switch (opcode) {
	case BATTMGR_REQUEST_NOTIFICATION:
		battmgr->error = 0;
		break;
	case BATTMGR_BAT_INFO:
		/* some firmware versions report an extra __le32 at the end of the payload */
		if (payload_len != sizeof(resp->info) &&
		    payload_len != (sizeof(resp->info) + sizeof(__le32))) {
			dev_warn(battmgr->dev,
				 "invalid payload length for battery information request: %zd\n",
				 payload_len);
			battmgr->error = -ENODATA;
			return;
		}

		battmgr->unit = le32_to_cpu(resp->info.power_unit);

		battmgr->info.present = true;
		battmgr->info.design_capacity = le32_to_cpu(resp->info.design_capacity) * 1000;
		battmgr->info.last_full_capacity = le32_to_cpu(resp->info.last_full_capacity) * 1000;
		battmgr->info.voltage_max_design = le32_to_cpu(resp->info.design_voltage) * 1000;
		battmgr->info.capacity_low = le32_to_cpu(resp->info.capacity_low) * 1000;
		battmgr->info.cycle_count = le32_to_cpu(resp->info.cycle_count);
		qcom_battmgr_sc8280xp_strcpy(battmgr->info.model_number, resp->info.model_number);
		qcom_battmgr_sc8280xp_strcpy(battmgr->info.serial_number, resp->info.serial_number);
		battmgr->info.technology = qcom_battmgr_sc8280xp_parse_technology(resp->info.battery_chemistry);
		qcom_battmgr_sc8280xp_strcpy(battmgr->info.oem_info, resp->info.oem_info);
		battmgr->info.day = resp->info.day;
		battmgr->info.month = resp->info.month;
		battmgr->info.year = le16_to_cpu(resp->info.year);
		break;
	case BATTMGR_BAT_STATUS:
		if (payload_len != sizeof(resp->status)) {
			dev_warn(battmgr->dev,
				 "invalid payload length for battery status request: %zd\n",
				 payload_len);
			battmgr->error = -ENODATA;
			return;
		}

		state = le32_to_cpu(resp->status.battery_state);
		if (state & BIT(0))
			battmgr->status.status = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (state & BIT(1))
			battmgr->status.status = POWER_SUPPLY_STATUS_CHARGING;
		else
			battmgr->status.status = POWER_SUPPLY_STATUS_NOT_CHARGING;

		battmgr->status.capacity = le32_to_cpu(resp->status.capacity) * 1000;
		battmgr->status.power_now = le32_to_cpu(resp->status.rate) * 1000;
		battmgr->status.voltage_now = le32_to_cpu(resp->status.battery_voltage) * 1000;
		battmgr->status.temperature = qcom_battmgr_sc8280xp_convert_temp(le32_to_cpu(resp->status.temperature));

		source = le32_to_cpu(resp->status.charging_source);
		battmgr->ac.online = source == BATTMGR_CHARGING_SOURCE_AC;
		battmgr->usb.online = source == BATTMGR_CHARGING_SOURCE_USB;
		battmgr->wireless.online = source == BATTMGR_CHARGING_SOURCE_WIRELESS;
		if (battmgr->info.last_full_capacity != 0) {
			/*
			 * 100 * battmgr->status.capacity can overflow a 32bit
			 * unsigned integer. FW readings are in m{W/A}h, which
			 * are multiplied by 1000 converting them to u{W/A}h,
			 * the format the power_supply API expects.
			 * To avoid overflow use the original value for dividend
			 * and convert the divider back to m{W/A}h, which can be
			 * done without any loss of precision.
			 */
			battmgr->status.percent =
				(100 * le32_to_cpu(resp->status.capacity)) /
				(battmgr->info.last_full_capacity / 1000);
		} else {
			/*
			 * Let the sysfs handler know no data is available at
			 * this time.
			 */
			battmgr->status.percent = (unsigned int)-1;
		}
		break;
	case BATTMGR_BAT_DISCHARGE_TIME:
		battmgr->status.discharge_time = le32_to_cpu(resp->time);
		break;
	case BATTMGR_BAT_CHARGE_TIME:
		battmgr->status.charge_time = le32_to_cpu(resp->time);
		break;
	case BATTMGR_CHG_CTRL_LIMIT_EN:
		battmgr->error = 0;
		break;
	default:
		dev_warn(battmgr->dev, "unknown message %#x\n", opcode);
		break;
	}

	complete(&battmgr->ack);
}

static void qcom_battmgr_sm8350_callback(struct qcom_battmgr *battmgr,
					 const struct qcom_battmgr_message *resp,
					 size_t len)
{
	unsigned int property;
	unsigned int opcode = le32_to_cpu(resp->hdr.opcode);
	size_t payload_len = len - sizeof(struct pmic_glink_hdr);
	unsigned int val;

	if (payload_len < sizeof(__le32)) {
		dev_warn(battmgr->dev, "invalid payload length for %#x: %zd\n",
			 opcode, len);
		return;
	}

	switch (opcode) {
	case BATTMGR_XM_PROPERTY_SET:
	case BATTMGR_XM_PROPERTY_GET:
		property = le32_to_cpu(resp->intval.property);
		if (battmgr->variant != XIAOMI_BATTMGR_SM8550) {
			dev_warn(battmgr->dev, "unexpected Xiaomi property response\n");
			break;
		}

		if (!battmgr->xiaomi_prop_pending || property != battmgr->xiaomi_prop) {
			dev_dbg(battmgr->dev, "unsolicited Xiaomi property %#x response\n", property);
			return;
		}

		if (payload_len == sizeof(resp->intval)) {
			battmgr->error = le32_to_cpu(resp->intval.result);
			if (!battmgr->error)
				battmgr->xiaomi_prop_value = le32_to_cpu(resp->intval.value);
			break;
		}

		if (payload_len > sizeof(resp->intval.property)) {
			size_t data_len = min_t(size_t,
						payload_len - sizeof(resp->intval.property),
						sizeof(battmgr->xiaomi_prop_data));

			battmgr->error = 0;
			battmgr->xiaomi_prop_value = le32_to_cpu(resp->intval.value);
			memcpy(battmgr->xiaomi_prop_data, &resp->intval.value,
			       data_len);
			battmgr->xiaomi_prop_data_len = data_len;
			if (property == XM_BATT_VERIFY_DIGEST)
				dev_dbg(battmgr->dev,
					"verify digest response opcode=%#x len=%zu data=%*phN\n",
					opcode, payload_len, (int)data_len,
					battmgr->xiaomi_prop_data);
			break;
		}

		dev_warn(battmgr->dev,
			 "invalid payload length for Xiaomi property %#x request: %zd\n",
			 property, payload_len);
		battmgr->error = -ENODATA;
		break;
	case BATTMGR_BAT_PROPERTY_SET:
	case BATTMGR_BAT_PROPERTY_GET:
		property = le32_to_cpu(resp->intval.property);
		if (opcode == BATTMGR_BAT_PROPERTY_SET) {
			if (battmgr->variant != XIAOMI_BATTMGR_SM8550)
				dev_warn(battmgr->dev, "unexpected battery property set response\n");
			break;
		}

		if (battmgr->variant == XIAOMI_BATTMGR_SM8550) {
			/* Xiaomi added BATT_CONSTANT_CURRENT after BATT_CHG_CTRL_LIM_MAX according
			 * to their code, but it also seems that two properties got removed?
			 * Fix properties after BATT_CHG_CTRL_LIM_MAX by decreasing property by 1
			 */
			if (property > BATT_CHG_CTRL_LIM_MAX)
				property--;
		}
		if (property == BATT_MODEL_NAME) {
			if (payload_len != sizeof(resp->strval)) {
				dev_warn(battmgr->dev,
					 "invalid payload length for BATT_MODEL_NAME request: %zd\n",
					 payload_len);
				battmgr->error = -ENODATA;
				return;
			}
		} else {
			if (payload_len != sizeof(resp->intval)) {
				dev_warn(battmgr->dev,
					 "invalid payload length for %#x request: %zd\n",
					 property, payload_len);
				battmgr->error = -ENODATA;
				return;
			}

			battmgr->error = le32_to_cpu(resp->intval.result);
			if (battmgr->error)
				goto out_complete;
		}

		switch (property) {
		case BATT_STATUS:
			battmgr->status.status = le32_to_cpu(resp->intval.value);
			break;
		case BATT_HEALTH:
			battmgr->status.health = le32_to_cpu(resp->intval.value);
			break;
		case BATT_PRESENT:
			battmgr->info.present = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CHG_TYPE:
			battmgr->info.charge_type = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CAPACITY:
			battmgr->status.percent = le32_to_cpu(resp->intval.value) / 100;
			break;
		case BATT_SOH:
			battmgr->status.soh_percent = le32_to_cpu(resp->intval.value);
			break;
		case BATT_VOLT_OCV:
			battmgr->status.voltage_ocv = le32_to_cpu(resp->intval.value);
			break;
		case BATT_VOLT_NOW:
			battmgr->status.voltage_now = le32_to_cpu(resp->intval.value);
			break;
		case BATT_VOLT_MAX:
			battmgr->info.voltage_max = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CURR_NOW:
			if (battmgr->variant == XIAOMI_BATTMGR_SM8550)
				battmgr->status.current_now = -le32_to_cpu(resp->intval.value); /* FG1 + FG2 */
			else
				battmgr->status.current_now = le32_to_cpu(resp->intval.value);
			break;
		case BATT_TEMP:
			val = le32_to_cpu(resp->intval.value);
			battmgr->status.temperature = DIV_ROUND_CLOSEST(val, 10);
			break;
		case BATT_TECHNOLOGY:
			battmgr->info.technology = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CHG_COUNTER:
			battmgr->info.charge_count = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CYCLE_COUNT:
			battmgr->info.cycle_count = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CHG_FULL_DESIGN:
			battmgr->info.design_capacity = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CHG_FULL:
			battmgr->info.last_full_capacity = le32_to_cpu(resp->intval.value);
			break;
		case BATT_MODEL_NAME:
			strscpy(battmgr->info.model_number, resp->strval.model, BATTMGR_STRING_LEN);
			break;
		case BATT_TTF_AVG:
			battmgr->status.charge_time = le32_to_cpu(resp->intval.value);
			break;
		case BATT_TTE_AVG:
			battmgr->status.discharge_time = le32_to_cpu(resp->intval.value);
			break;
		case BATT_RESISTANCE:
			battmgr->status.resistance = le32_to_cpu(resp->intval.value);
			break;
		case BATT_POWER_NOW:
			battmgr->status.power_now = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CHG_CTRL_START_THR:
			battmgr->info.charge_ctrl_start = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CHG_CTRL_END_THR:
			battmgr->info.charge_ctrl_end = le32_to_cpu(resp->intval.value);
			break;
		default:
			if (battmgr->variant != XIAOMI_BATTMGR_SM8550)
				dev_warn(battmgr->dev, "unknown property %#x\n", property);
			break;
		}
		break;
	case BATTMGR_USB_PROPERTY_GET:
		property = le32_to_cpu(resp->intval.property);
		if (payload_len != sizeof(resp->intval)) {
			dev_warn(battmgr->dev,
				 "invalid payload length for %#x request: %zd\n",
				 property, payload_len);
			battmgr->error = -ENODATA;
			return;
		}

		battmgr->error = le32_to_cpu(resp->intval.result);
		if (battmgr->error)
			goto out_complete;

		switch (property) {
		case USB_ONLINE:
			battmgr->usb.online = le32_to_cpu(resp->intval.value);
			break;
		case USB_VOLT_NOW:
			battmgr->usb.voltage_now = le32_to_cpu(resp->intval.value);
			break;
		case USB_VOLT_MAX:
			battmgr->usb.voltage_max = le32_to_cpu(resp->intval.value);
			break;
		case USB_CURR_NOW:
			battmgr->usb.current_now = le32_to_cpu(resp->intval.value);
			break;
		case USB_CURR_MAX:
			battmgr->usb.current_max = le32_to_cpu(resp->intval.value);
			break;
		case USB_INPUT_CURR_LIMIT:
			battmgr->usb.current_limit = le32_to_cpu(resp->intval.value);
			break;
		case USB_TYPE:
			battmgr->usb.usb_type = le32_to_cpu(resp->intval.value);
			break;
		default:
			dev_warn(battmgr->dev, "unknown property %#x\n", property);
			break;
		}
		break;
	case BATTMGR_WLS_PROPERTY_GET:
		property = le32_to_cpu(resp->intval.property);
		if (payload_len != sizeof(resp->intval)) {
			dev_warn(battmgr->dev,
				 "invalid payload length for %#x request: %zd\n",
				 property, payload_len);
			battmgr->error = -ENODATA;
			return;
		}

		battmgr->error = le32_to_cpu(resp->intval.result);
		if (battmgr->error)
			goto out_complete;

		switch (property) {
		case WLS_ONLINE:
			battmgr->wireless.online = le32_to_cpu(resp->intval.value);
			break;
		case WLS_VOLT_NOW:
			battmgr->wireless.voltage_now = le32_to_cpu(resp->intval.value);
			break;
		case WLS_VOLT_MAX:
			battmgr->wireless.voltage_max = le32_to_cpu(resp->intval.value);
			break;
		case WLS_CURR_NOW:
			battmgr->wireless.current_now = le32_to_cpu(resp->intval.value);
			break;
		case WLS_CURR_MAX:
			battmgr->wireless.current_max = le32_to_cpu(resp->intval.value);
			break;
		default:
			dev_warn(battmgr->dev, "unknown property %#x\n", property);
			break;
		}
		break;
	case BATTMGR_REQUEST_NOTIFICATION:
	case BATTMGR_CHG_CTRL_LIMIT_EN:
		battmgr->error = 0;
		break;
	default:
		if (battmgr->variant != XIAOMI_BATTMGR_SM8550)
			dev_warn(battmgr->dev, "unknown message %#x\n", opcode);
		break;
	}

out_complete:
	complete(&battmgr->ack);
}

static void qcom_battmgr_callback(const void *data, size_t len, void *priv)
{
	const struct pmic_glink_hdr *hdr = data;
	struct qcom_battmgr *battmgr = priv;
	unsigned int opcode = le32_to_cpu(hdr->opcode);

	if (opcode == BATTMGR_NOTIFICATION)
		qcom_battmgr_notification(battmgr, data, len);
	else if (battmgr->variant == QCOM_BATTMGR_SC8280XP ||
		 battmgr->variant == QCOM_BATTMGR_X1E80100)
		qcom_battmgr_sc8280xp_callback(battmgr, data, len);
	else
		qcom_battmgr_sm8350_callback(battmgr, data, len);
}

static void qcom_battmgr_enable_worker(struct work_struct *work)
{
	struct qcom_battmgr *battmgr = container_of(work, struct qcom_battmgr, enable_work);
	struct qcom_battmgr_enable_request req = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_NOTIFY),
		.hdr.opcode = cpu_to_le32(BATTMGR_REQUEST_NOTIFICATION),
	};
	int ret;

	ret = qcom_battmgr_request(battmgr, &req, sizeof(req));
	if (ret)
		dev_err(battmgr->dev, "failed to request power notifications\n");
}

static void qcom_battmgr_pdr_notify(void *priv, int state)
{
	struct qcom_battmgr *battmgr = priv;

	if (state == SERVREG_SERVICE_STATE_UP) {
		battmgr->service_up = true;
		schedule_work(&battmgr->enable_work);
	} else {
		battmgr->service_up = false;
	}
}

static const struct of_device_id qcom_battmgr_of_variants[] = {
	{ .compatible = "qcom,glymur-pmic-glink", .data = (void *)QCOM_BATTMGR_X1E80100 },
	{ .compatible = "qcom,kaanapali-pmic-glink", .data = (void *)QCOM_BATTMGR_SM8550 },
	{ .compatible = "qcom,sc8180x-pmic-glink", .data = (void *)QCOM_BATTMGR_SC8280XP },
	{ .compatible = "qcom,sc8280xp-pmic-glink", .data = (void *)QCOM_BATTMGR_SC8280XP },
	{ .compatible = "qcom,sm8550-pmic-glink", .data = (void *)QCOM_BATTMGR_SM8550 },
	{ .compatible = "qcom,x1e80100-pmic-glink", .data = (void *)QCOM_BATTMGR_X1E80100 },
	{ .compatible = "xiaomi,sm8550-pmic-glink", .data = (void *)XIAOMI_BATTMGR_SM8550 },
	/* Unmatched devices falls back to QCOM_BATTMGR_SM8350 */
	{}
};

static char *qcom_battmgr_battery[] = { "battery" };

static int qcom_battmgr_probe(struct auxiliary_device *adev,
			      const struct auxiliary_device_id *id)
{
	const struct power_supply_desc *psy_desc;
	struct power_supply_config psy_cfg_supply = {};
	struct power_supply_config psy_cfg = {};
	const struct of_device_id *match;
	struct qcom_battmgr *battmgr;
	struct device *dev = &adev->dev;
	int ret;

	battmgr = devm_kzalloc(dev, sizeof(*battmgr), GFP_KERNEL);
	if (!battmgr)
		return -ENOMEM;

	battmgr->dev = dev;
	dev_set_drvdata(dev, battmgr);

	psy_cfg.drv_data = battmgr;
	psy_cfg.fwnode = dev_fwnode(&adev->dev);

	psy_cfg_supply.drv_data = battmgr;
	psy_cfg_supply.fwnode = dev_fwnode(&adev->dev);
	psy_cfg_supply.supplied_to = qcom_battmgr_battery;
	psy_cfg_supply.num_supplicants = 1;

	INIT_WORK(&battmgr->enable_work, qcom_battmgr_enable_worker);
	mutex_init(&battmgr->lock);
	init_completion(&battmgr->ack);

	match = of_match_device(qcom_battmgr_of_variants, dev->parent);
	if (match)
		battmgr->variant = (unsigned long)match->data;
	else
		battmgr->variant = QCOM_BATTMGR_SM8350;

	ret = qcom_battmgr_charge_control_thresholds_init(battmgr);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "failed to init battery charge control thresholds\n");

	if (battmgr->variant == XIAOMI_BATTMGR_SM8550) {
		ret = devm_device_add_group(dev, &qcom_battmgr_xiaomi_attr_group);
		if (ret < 0)
			return dev_err_probe(dev, ret, "failed to add Xiaomi battery attributes\n");
	}

	if (battmgr->variant == QCOM_BATTMGR_SC8280XP ||
	    battmgr->variant == QCOM_BATTMGR_X1E80100) {
		if (battmgr->variant == QCOM_BATTMGR_X1E80100)
			psy_desc = &x1e80100_bat_psy_desc;
		else
			psy_desc = &sc8280xp_bat_psy_desc;

		battmgr->bat_psy = devm_power_supply_register(dev, psy_desc, &psy_cfg);
		if (IS_ERR(battmgr->bat_psy))
			return dev_err_probe(dev, PTR_ERR(battmgr->bat_psy),
					     "failed to register battery power supply\n");

		battmgr->ac_psy = devm_power_supply_register(dev, &sc8280xp_ac_psy_desc, &psy_cfg_supply);
		if (IS_ERR(battmgr->ac_psy))
			return dev_err_probe(dev, PTR_ERR(battmgr->ac_psy),
					     "failed to register AC power supply\n");

		battmgr->usb_psy = devm_power_supply_register(dev, &sc8280xp_usb_psy_desc, &psy_cfg_supply);
		if (IS_ERR(battmgr->usb_psy))
			return dev_err_probe(dev, PTR_ERR(battmgr->usb_psy),
					     "failed to register USB power supply\n");

		battmgr->wls_psy = devm_power_supply_register(dev, &sc8280xp_wls_psy_desc, &psy_cfg_supply);
		if (IS_ERR(battmgr->wls_psy))
			return dev_err_probe(dev, PTR_ERR(battmgr->wls_psy),
					     "failed to register wireless charing power supply\n");
	} else {
		if (battmgr->variant == QCOM_BATTMGR_SM8550 ||
		    battmgr->variant == XIAOMI_BATTMGR_SM8550)
			psy_desc = &sm8550_bat_psy_desc;
		else
			psy_desc = &sm8350_bat_psy_desc;

		battmgr->bat_psy = devm_power_supply_register(dev, psy_desc, &psy_cfg);
		if (IS_ERR(battmgr->bat_psy))
			return dev_err_probe(dev, PTR_ERR(battmgr->bat_psy),
					     "failed to register battery power supply\n");

		battmgr->usb_psy = devm_power_supply_register(dev, &sm8350_usb_psy_desc, &psy_cfg_supply);
		if (IS_ERR(battmgr->usb_psy))
			return dev_err_probe(dev, PTR_ERR(battmgr->usb_psy),
					     "failed to register USB power supply\n");

		battmgr->wls_psy = devm_power_supply_register(dev, &sm8350_wls_psy_desc, &psy_cfg_supply);
		if (IS_ERR(battmgr->wls_psy))
			return dev_err_probe(dev, PTR_ERR(battmgr->wls_psy),
					     "failed to register wireless charing power supply\n");
	}

	battmgr->client = devm_pmic_glink_client_alloc(dev, PMIC_GLINK_OWNER_BATTMGR,
						       qcom_battmgr_callback,
						       qcom_battmgr_pdr_notify,
						       battmgr);
	if (IS_ERR(battmgr->client))
		return PTR_ERR(battmgr->client);

	pmic_glink_client_register(battmgr->client);

	return 0;
}

static const struct auxiliary_device_id qcom_battmgr_id_table[] = {
	{ .name = "pmic_glink.power-supply", },
	{},
};
MODULE_DEVICE_TABLE(auxiliary, qcom_battmgr_id_table);

static struct auxiliary_driver qcom_battmgr_driver = {
	.name = "pmic_glink_power_supply",
	.probe = qcom_battmgr_probe,
	.id_table = qcom_battmgr_id_table,
};

module_auxiliary_driver(qcom_battmgr_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Qualcomm PMIC GLINK battery manager driver");
MODULE_LICENSE("GPL");
