/*
 * Copyright (C) 2010 - 2022 Novatek, Inc.
 *
 * $Revision: 103375 $
 * $Date: 2022-07-29 10:34:16 +0800 (週五, 29 七月 2022) $
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#ifndef 	_LINUX_NVT_TOUCH_H
#define		_LINUX_NVT_TOUCH_H

#include <linux/delay.h>
#include <linux/kfifo.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#if defined(CONFIG_DRM_PANEL)
#include <drm/drm_panel.h>
#endif

#include "nt36xxx_mem_map.h"

struct proc_dir_entry;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
#define HAVE_PROC_OPS
#endif

#define NVT_DEBUG 0

//---GPIO number---
#define NVTTOUCH_RST_PIN 980
#define NVTTOUCH_INT_PIN 943

//---INT trigger mode---
//#define IRQ_TYPE_EDGE_RISING 1
//#define IRQ_TYPE_EDGE_FALLING 2
#define INT_TRIGGER_TYPE IRQ_TYPE_EDGE_RISING

//---bus transfer length---
#define BUS_TRANSFER_LENGTH  256

//---SPI driver info.---
#define NVT_SPI_NAME "NVT-ts-spi"

#if NVT_DEBUG
#define NVT_LOG(fmt, args...)    pr_err("[%s] %s %d: " fmt, NVT_SPI_NAME, __func__, __LINE__, ##args)
#else
#define NVT_LOG(fmt, args...)    pr_debug("[%s] %s %d: " fmt, NVT_SPI_NAME, __func__, __LINE__, ##args)
#endif
#define NVT_ERR(fmt, args...)    pr_err("[%s] %s %d: " fmt, NVT_SPI_NAME, __func__, __LINE__, ##args)

//---Touch info.---
#define TOUCH_DEFAULT_NUM_X 60
#define TOUCH_DEFAULT_NUM_Y 40
#define TOUCH_DEFAULT_MAX_WIDTH 3048
#define TOUCH_DEFAULT_MAX_HEIGHT 2032

//---Customerized func.---
#define BOOT_UPDATE_FIRMWARE 1
#define BOOT_UPDATE_FIRMWARE_NAME "novatek/nt36532e.bin"
#define POINT_DATA_LEN 760

//---ESD Protect.---
#define NVT_TOUCH_WDT_RECOVERY 1

#if BOOT_UPDATE_FIRMWARE
#define SIZE_4KB 4096
#define FLASH_SECTOR_SIZE SIZE_4KB
#define FW_BIN_VER_OFFSET (fw_need_write_size - SIZE_4KB)
#define FW_BIN_VER_BAR_OFFSET (FW_BIN_VER_OFFSET + 1)
#define NVT_FLASH_END_FLAG_LEN 3
#define NVT_FLASH_END_FLAG_ADDR (fw_need_write_size - NVT_FLASH_END_FLAG_LEN)
#endif

enum nvt_ic_state {
	NVT_IC_SUSPEND_IN,
	NVT_IC_SUSPEND_OUT,
	NVT_IC_RESUME_IN,
	NVT_IC_RESUME_OUT,
	NVT_IC_INIT,
};

struct nvt_ts_data {
	struct spi_device *client;
	struct delayed_work nvt_fwu_work;
	int ic_state;
	bool dev_pm_suspend;
	struct completion dev_pm_suspend_completion;
	uint16_t addr;
#if defined(CONFIG_DRM_PANEL)
	struct drm_panel_follower panel_follower;
#endif
	const char *fw_name;
	uint8_t fw_ver;
	uint8_t x_num;
	uint8_t y_num;
	uint16_t abs_x_max;
	uint16_t abs_y_max;
	uint32_t int_trigger_type;
	struct gpio_desc *irq_gpiod;
	uint32_t irq_flags;
	int32_t reset_gpio;
	uint32_t reset_flags;
	struct mutex lock;
	const struct nvt_ts_mem_map *mmap;
	uint8_t hw_crc;
	uint8_t auto_copy;
	uint16_t nvt_pid;
	uint8_t *rbuf;
	uint8_t *xbuf;
	struct mutex xbuf_lock;
	bool irq_enabled;
	uint8_t debug_flag;
	int result_type;
	int panel_index;
	uint32_t chip_ver_trim_addr;
	uint32_t swrst_sif_addr;
	uint32_t crc_err_flag_addr;
	struct workqueue_struct *event_wq;
	struct work_struct resume_work;
	struct mutex power_supply_lock;
	struct work_struct power_supply_work;
	struct notifier_block power_supply_notifier;
	int power_supply_status;
	bool power_supply_registered;
	bool panel_on;
	struct mutex thp_lock; /* protects the frame buffer and stream FIFO */
	u8 *thp_frame;
	bool thp_capture_enabled;
	bool thp_stylus_enabled;
	bool thp_frame_valid;
	bool thp_epoch_pending;
	u64 thp_frame_count;
	u64 thp_epoch_count;
	u64 thp_read_errors;
	u64 thp_header_errors;
	u64 thp_stream_drops;
	u16 thp_header_crc;
	u32 thp_magic;
	ktime_t thp_timestamp;
	struct kfifo thp_stream_fifo;
	void *thp_stream_buf;
	wait_queue_head_t thp_stream_wait;
	struct proc_dir_entry *thp_raw_proc;
	struct proc_dir_entry *thp_stream_proc;
	struct proc_dir_entry *thp_status_proc;
	struct proc_dir_entry *thp_stylus_proc;
};

typedef enum {
	RESET_STATE_INIT = 0xA0,// IC reset
	RESET_STATE_REK,		// ReK baseline
	RESET_STATE_REK_FINISH,	// baseline is ready
	RESET_STATE_NORMAL_RUN,	// normal run
	RESET_STATE_MAX  = 0xAF
} RST_COMPLETE_STATE;

typedef enum {
    EVENT_MAP_HOST_CMD                      = 0x50,
    EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE   = 0x51,
    EVENT_MAP_RESET_COMPLETE                = 0x60,
    EVENT_MAP_FWINFO                        = 0x78,
    EVENT_MAP_PROJECTID                     = 0x9A,
} SPI_EVENT_MAP;

//---SPI READ/WRITE---
#define SPI_WRITE_MASK(a)	(a | 0x80)
#define SPI_READ_MASK(a)	(a & 0x7F)

#define DUMMY_BYTES (1)
#define NVT_TRANSFER_LEN	(63*1024)
#define NVT_READ_LEN		(8*1024)
#define NVT_XBUF_LEN		(NVT_TRANSFER_LEN+1+DUMMY_BYTES)

typedef enum {
	NVTWRITE = 0,
	NVTREAD  = 1
} NVT_SPI_RW;

//---extern structures---
extern struct nvt_ts_data *ts;

//---extern functions---
int32_t CTP_SPI_READ(struct spi_device *client, uint8_t *buf, uint16_t len);
int32_t CTP_SPI_WRITE(struct spi_device *client, uint8_t *buf, uint16_t len);
void nvt_bootloader_reset(void);
void nvt_eng_reset(void);
void nvt_sw_reset(void);
void nvt_sw_reset_idle(void);
void nvt_boot_ready(void);
void nvt_fw_crc_enable(void);
void nvt_tx_auto_copy_mode(void);
void nvt_read_fw_history(uint32_t fw_history_addr);
int32_t nvt_update_firmware(const char *firmware_name);
void nvt_thp_mark_epoch(void);
int32_t nvt_check_fw_reset_state(RST_COMPLETE_STATE check_reset_state);
int32_t nvt_get_fw_info(void);
int32_t nvt_clear_fw_status(void);
int32_t nvt_check_fw_status(void);
int32_t nvt_set_page(uint32_t addr);
int32_t nvt_wait_auto_copy(void);
int32_t nvt_write_addr(uint32_t addr, uint8_t data);
int32_t nvt_read_reg(nvt_ts_reg_t reg, uint8_t *val);
int32_t nvt_check_spi_dma_tx_info(void);
int32_t nvt_check_tx_auto_copy(void);
int nvt_set_custom_cmd(u8 cmd, u16 value);
int nvt_thp_restore_stylus(void);
void nvt_power_supply_restore(void);
void nvt_set_doze_delay(u16 value);
void Boot_Update_Firmware(struct work_struct *work);
void thp_parse_frame(uint16_t* touch_matrix);
#endif /* _LINUX_NVT_TOUCH_H */
