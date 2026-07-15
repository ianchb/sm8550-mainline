/*
 * Copyright (C) 2010 - 2022 Novatek, Inc.
 *
 * $Revision: 102158 $
 * $Date: 2022-07-07 11:10:06 +0800 (週四, 07 七月 2022) $
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/poll.h>
#include <linux/gpio/consumer.h>
#include <linux/of_irq.h>
#include <linux/unaligned.h>
#include <linux/vmalloc.h>
#include <uapi/linux/sched/types.h>
#include "nt36xxx.h"

#if defined(CONFIG_DRM_PANEL)
#include <drm/drm_panel.h>
#endif

static int32_t nvt_ts_suspend(struct device *dev);
static int32_t nvt_ts_resume(struct device *dev);
static void nvt_irq_enable(bool enable);
struct nvt_ts_data *ts;

#if BOOT_UPDATE_FIRMWARE
static struct workqueue_struct *nvt_fwu_wq;
extern void Boot_Update_Firmware(struct work_struct *work);
#endif

#if defined(CONFIG_DRM_PANEL)
static struct drm_panel_follower_funcs nt36xxx_panel_follower_funcs;
#endif
#define NVT_THP_HEADER_LEN 257
#define NVT_THP_PAYLOAD_LEN 5160
#define NVT_THP_FRAME_LEN (NVT_THP_HEADER_LEN + NVT_THP_PAYLOAD_LEN)
#define NVT_THP_STREAM_SIZE (4 * 1024 * 1024)
#define NVT_THP_STREAM_MAGIC 0x3150544e

struct nvt_thp_stream_header {
	__le32 magic;
	__le16 header_len;
	__le16 frame_len;
	__le64 sequence;
	__le64 timestamp_ns;
	__le16 header_crc;
	__le16 flags;
	__le32 firmware_magic;
} __packed;

static bool nvt_thp_header_valid(const u8 *frame, u16 *header_crc,
				 u32 *magic)
{
	u16 crc = get_unaligned_le16(frame + NVT_THP_HEADER_LEN + 4);
	u32 marker = get_unaligned_le32(frame + NVT_THP_HEADER_LEN + 8);
	u16 crc_inv = get_unaligned_le16(frame + NVT_THP_HEADER_LEN + 12);
	u32 marker_inv = get_unaligned_le32(frame + NVT_THP_HEADER_LEN + 16);

	*header_crc = crc;
	*magic = marker;

	return crc_inv == (u16)~crc && marker_inv == ~marker;
}

static int nvt_thp_read_frame(u8 *fw_state)
{
	int ret;

	mutex_lock(&ts->thp_lock);
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR);
	ts->thp_frame[0] = 0;
	ret = CTP_SPI_READ(ts->client, ts->thp_frame, NVT_THP_FRAME_LEN);
	if (ret < 0)
		ts->thp_read_errors++;
	else {
		memcpy(fw_state, ts->thp_frame, 7);
		ts->thp_timestamp = ktime_get_boottime();
	}
	mutex_unlock(&ts->thp_lock);

	return ret;
}

static void nvt_thp_publish_frame(void)
{
	struct nvt_thp_stream_header header;
	size_t record_len = sizeof(header) + NVT_THP_FRAME_LEN;
	u16 header_crc;
	u32 magic;
	bool valid;

	mutex_lock(&ts->thp_lock);
	valid = nvt_thp_header_valid(ts->thp_frame, &header_crc, &magic);
	ts->thp_frame_count++;
	ts->thp_frame_valid = valid;
	ts->thp_header_crc = header_crc;
	ts->thp_magic = magic;
	if (!valid)
		ts->thp_header_errors++;

	header.magic = cpu_to_le32(NVT_THP_STREAM_MAGIC);
	header.header_len = cpu_to_le16(sizeof(header));
	header.frame_len = cpu_to_le16(NVT_THP_FRAME_LEN);
	header.sequence = cpu_to_le64(ts->thp_frame_count);
	header.timestamp_ns = cpu_to_le64(ktime_to_ns(ts->thp_timestamp));
	header.header_crc = cpu_to_le16(header_crc);
	header.flags = cpu_to_le16(valid ? BIT(0) : 0);
	header.firmware_magic = cpu_to_le32(magic);

	if (kfifo_avail(&ts->thp_stream_fifo) < record_len) {
		ts->thp_stream_drops++;
	} else {
		kfifo_in(&ts->thp_stream_fifo, &header, sizeof(header));
		kfifo_in(&ts->thp_stream_fifo, ts->thp_frame,
			 NVT_THP_FRAME_LEN);
	}
	mutex_unlock(&ts->thp_lock);
	wake_up_interruptible(&ts->thp_stream_wait);
}

static ssize_t nvt_thp_raw_write(struct file *file, const char __user *buf,
				 size_t count, loff_t *ppos)
{
	bool enable;
	int ret;

	if (!ts)
		return -ENODEV;

	ret = kstrtobool_from_user(buf, count, &enable);
	if (ret)
		return ret;

	mutex_lock(&ts->thp_lock);
	if (enable && !ts->thp_capture_enabled)
		kfifo_reset(&ts->thp_stream_fifo);
	WRITE_ONCE(ts->thp_capture_enabled, enable);
	mutex_unlock(&ts->thp_lock);
	wake_up_interruptible(&ts->thp_stream_wait);
	NVT_ERR("THP passive capture %s, event buffer 0x%06X\n",
		enable ? "enabled" : "disabled", ts->mmap->EVENT_BUF_ADDR);

	return count;
}

static ssize_t nvt_thp_stylus_read(struct file *file, char __user *buf,
				   size_t count, loff_t *ppos)
{
	char value[3];
	int len;

	if (!ts)
		return -ENODEV;

	len = scnprintf(value, sizeof(value), "%u\n",
			READ_ONCE(ts->thp_stylus_enabled));
	return simple_read_from_buffer(buf, count, ppos, value, len);
}

static int nvt_thp_set_pen_switch(u8 value)
{
	u8 command[3] = { 0x50, 0x7b, value };
	u8 ack[2] = { 0x50, 0xff };
	int retry;
	int ret = 0;

	ret = nvt_set_page(ts->mmap->EVENT_BUF_ADDR | 0x50);
	if (ret < 0)
		return ret;

	for (retry = 0; retry < 10; retry++) {
		ret = CTP_SPI_WRITE(ts->client, command, sizeof(command));
		if (ret < 0)
			break;

		usleep_range(10000, 11000);
		ack[1] = 0xff;
		ret = CTP_SPI_READ(ts->client, ack, sizeof(ack));
		if (ret < 0)
			break;
		if (!ack[1]) {
			ret = 0;
			break;
		}
	}

	nvt_set_page(ts->mmap->EVENT_BUF_ADDR);
	if (ret < 0)
		return ret;

	return ack[1] ? -ETIMEDOUT : 0;
}

static ssize_t nvt_thp_stylus_write(struct file *file,
				    const char __user *buf, size_t count,
				    loff_t *ppos)
{
	bool irq_was_enabled;
	bool enable;
	int ret;

	if (!ts)
		return -ENODEV;

	ret = kstrtobool_from_user(buf, count, &enable);
	if (ret)
		return ret;

	irq_was_enabled = READ_ONCE(ts->irq_enabled);
	if (irq_was_enabled)
		nvt_irq_enable(false);
	mutex_lock(&ts->lock);
	ret = nvt_set_custom_cmd(0x04, enable);
	if (!ret && enable)
		ret = nvt_thp_set_pen_switch(1);
	if (!ret)
		ts->thp_stylus_enabled = enable;
	mutex_unlock(&ts->lock);
	if (irq_was_enabled)
		nvt_irq_enable(true);

	if (ret)
		return ret;

	NVT_ERR("THP stylus scanning %s\n", enable ? "enabled" : "disabled");
	return count;
}

int nvt_thp_restore_stylus(void)
{
	int ret;

	if (!READ_ONCE(ts->thp_stylus_enabled))
		return 0;

	ret = nvt_set_custom_cmd(0x04, 1);
	if (!ret)
		ret = nvt_thp_set_pen_switch(1);
	if (ret)
		NVT_ERR("failed to restore THP stylus state: %d\n", ret);

	return ret;
}

static bool nvt_thp_stream_ready(void)
{
	return !kfifo_is_empty(&ts->thp_stream_fifo) ||
	       !READ_ONCE(ts->thp_capture_enabled);
}

static ssize_t nvt_thp_stream_read(struct file *file, char __user *buf,
				   size_t count, loff_t *ppos)
{
	unsigned int copied = 0;
	int ret;

	if (!ts)
		return -ENODEV;
	if (!count)
		return 0;

	if (file->f_flags & O_NONBLOCK) {
		if (kfifo_is_empty(&ts->thp_stream_fifo))
			return -EAGAIN;
	} else {
		ret = wait_event_interruptible(ts->thp_stream_wait,
					       nvt_thp_stream_ready());
		if (ret)
			return ret;
	}

	mutex_lock(&ts->thp_lock);
	ret = kfifo_to_user(&ts->thp_stream_fifo, buf, count, &copied);
	mutex_unlock(&ts->thp_lock);

	return ret ? ret : copied;
}

static __poll_t nvt_thp_stream_poll(struct file *file, poll_table *wait)
{
	__poll_t mask = 0;

	if (!ts)
		return EPOLLERR;

	poll_wait(file, &ts->thp_stream_wait, wait);
	if (!kfifo_is_empty(&ts->thp_stream_fifo))
		mask |= EPOLLIN | EPOLLRDNORM;
	if (!READ_ONCE(ts->thp_capture_enabled))
		mask |= EPOLLHUP;

	return mask;
}

static int nvt_thp_status_show(struct seq_file *m, void *v)
{
	if (!ts)
		return -ENODEV;

	mutex_lock(&ts->thp_lock);
	seq_printf(m, "enabled: %u\n", ts->thp_capture_enabled);
	seq_printf(m, "event_buffer: 0x%06x\n", ts->mmap->EVENT_BUF_ADDR);
	seq_printf(m, "firmware: %s\n", ts->fw_name);
	seq_printf(m, "stylus_enabled: %u\n", ts->thp_stylus_enabled);
	seq_puts(m, "kernel_input: disabled\n");
	seq_printf(m, "raw_frame_length: %u\n", NVT_THP_FRAME_LEN);
	seq_printf(m, "stream_frame_length: %u\n", NVT_THP_FRAME_LEN);
	seq_printf(m, "frames: %llu\n",
		   (unsigned long long)ts->thp_frame_count);
	seq_printf(m, "read_errors: %llu\n",
		   (unsigned long long)ts->thp_read_errors);
	seq_printf(m, "header_errors: %llu\n",
		   (unsigned long long)ts->thp_header_errors);
	seq_printf(m, "stream_bytes: %u\n", kfifo_len(&ts->thp_stream_fifo));
	seq_printf(m, "stream_drops: %llu\n",
		   (unsigned long long)ts->thp_stream_drops);
	seq_printf(m, "latest_valid: %u\n", ts->thp_frame_valid);
	seq_printf(m, "latest_header_crc: 0x%04x\n", ts->thp_header_crc);
	seq_printf(m, "latest_magic: 0x%08x\n", ts->thp_magic);
	seq_printf(m, "latest_timestamp_ns: %lld\n",
		   (long long)ktime_to_ns(ts->thp_timestamp));
	mutex_unlock(&ts->thp_lock);

	return 0;
}

static int nvt_thp_status_open(struct inode *inode, struct file *file)
{
	return single_open(file, nvt_thp_status_show, NULL);
}

static const struct proc_ops nvt_thp_raw_ops = {
	.proc_write = nvt_thp_raw_write,
	.proc_lseek = noop_llseek,
};

static const struct proc_ops nvt_thp_stream_ops = {
	.proc_read = nvt_thp_stream_read,
	.proc_poll = nvt_thp_stream_poll,
	.proc_lseek = noop_llseek,
};

static const struct proc_ops nvt_thp_stylus_ops = {
	.proc_read = nvt_thp_stylus_read,
	.proc_write = nvt_thp_stylus_write,
	.proc_lseek = default_llseek,
};

static const struct proc_ops nvt_thp_status_ops = {
	.proc_open = nvt_thp_status_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

uint32_t ENG_RST_ADDR  = 0x7FFF80;
uint32_t SPI_RD_FAST_ADDR = 0;	//read from dtsi

static uint8_t bTouchIsAwake = 0;

/*******************************************************
Description:
	Novatek touchscreen irq enable/disable function.

return:
	n.a.
*******************************************************/
static void nvt_irq_enable(bool enable)
{
	if (enable) {
		if (!ts->irq_enabled) {
			enable_irq(ts->client->irq);
			ts->irq_enabled = true;
		}
	} else {
		if (ts->irq_enabled) {
			disable_irq(ts->client->irq);
			ts->irq_enabled = false;
		}
	}
}

/*******************************************************
Description:
	Novatek touchscreen spi read/write core function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static inline int32_t spi_read_write(struct spi_device *client, uint8_t *buf, size_t len , NVT_SPI_RW rw)
{
	struct spi_message m;
	struct spi_transfer t = {
		.len    = len,
	};

	memset(ts->xbuf, 0, len + DUMMY_BYTES);
	memcpy(ts->xbuf, buf, len);

	switch (rw) {
		case NVTREAD:
			t.tx_buf = ts->xbuf;
			t.rx_buf = ts->rbuf;
			t.len    = (len + DUMMY_BYTES);
			break;

		case NVTWRITE:
			t.tx_buf = ts->xbuf;
			break;
	}

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spi_sync(client, &m);
}

/*******************************************************
Description:
	Novatek touchscreen spi read function.

return:
	Executive outcomes. 2---succeed. -5---I/O error
*******************************************************/
int32_t CTP_SPI_READ(struct spi_device *client, uint8_t *buf, uint16_t len)
{
	int32_t ret = -1;
	int32_t retries = 0;

	mutex_lock(&ts->xbuf_lock);

	buf[0] = SPI_READ_MASK(buf[0]);

	while (retries < 5) {
		ret = spi_read_write(client, buf, len, NVTREAD);
		if (ret == 0) break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		NVT_ERR("read error, ret=%d\n", ret);
		ret = -EIO;
	} else {
		memcpy((buf+1), (ts->rbuf+2), (len-1));
	}

	mutex_unlock(&ts->xbuf_lock);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen spi write function.

return:
	Executive outcomes. 1---succeed. -5---I/O error
*******************************************************/
int32_t CTP_SPI_WRITE(struct spi_device *client, uint8_t *buf, uint16_t len)
{
	int32_t ret = -1;
	int32_t retries = 0;

	mutex_lock(&ts->xbuf_lock);

	buf[0] = SPI_WRITE_MASK(buf[0]);

	while (retries < 5) {
		ret = spi_read_write(client, buf, len, NVTWRITE);
		if (ret == 0)	break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		NVT_ERR("error, ret=%d\n", ret);
		ret = -EIO;
	}

	mutex_unlock(&ts->xbuf_lock);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen set index/page/addr address.

return:
	Executive outcomes. 0---succeed. -5---access fail.
*******************************************************/
int32_t nvt_set_page(uint32_t addr)
{
	uint8_t buf[4] = {0};

	buf[0] = 0xFF;	//set index/page/addr command
	buf[1] = (addr >> 15) & 0xFF;
	buf[2] = (addr >> 7) & 0xFF;

	return CTP_SPI_WRITE(ts->client, buf, 3);
}

/*******************************************************
Description:
	Novatek touchscreen write data to specify address.

return:
	Executive outcomes. 0---succeed. -5---access fail.
*******************************************************/
int32_t nvt_write_addr(uint32_t addr, uint8_t data)
{
	int32_t ret = 0;
	uint8_t buf[4] = {0};

	//---set xdata index---
	buf[0] = 0xFF;	//set index/page/addr command
	buf[1] = (addr >> 15) & 0xFF;
	buf[2] = (addr >> 7) & 0xFF;
	ret = CTP_SPI_WRITE(ts->client, buf, 3);
	if (ret) {
		NVT_ERR("set page 0x%06X failed, ret = %d\n", addr, ret);
		return ret;
	}

	//---write data to index---
	buf[0] = addr & (0x7F);
	buf[1] = data;
	ret = CTP_SPI_WRITE(ts->client, buf, 2);
	if (ret) {
		NVT_ERR("write data to 0x%06X failed, ret = %d\n", addr, ret);
		return ret;
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen read value to specific register.

return:
	Executive outcomes. 0---succeed. -5---access fail.
*******************************************************/
int32_t nvt_read_reg(nvt_ts_reg_t reg, uint8_t *val)
{
	int32_t ret = 0;
	uint32_t addr = 0;
	uint8_t mask = 0;
	uint8_t shift = 0;
	uint8_t buf[8] = {0};
	uint8_t temp = 0;

	addr = reg.addr;
	mask = reg.mask;
	/* get shift */
	temp = reg.mask;
	shift = 0;
	while (1) {
		if ((temp >> shift) & 0x01)
			break;
		if (shift == 8) {
			NVT_ERR("mask all bits zero!\n");
			ret = -1;
			break;
		}
		shift++;
	}
	/* read the byte of the register is in */
	nvt_set_page(addr);
	buf[0] = addr & 0xFF;
	buf[1] = 0x00;
	ret = CTP_SPI_READ(ts->client, buf, 2);
	if (ret < 0) {
		NVT_ERR("CTP_SPI_READ failed!(%d)\n", ret);
		goto nvt_read_register_exit;
	}
	/* get register's value in its field of the byte */
	*val = (buf[1] & mask) >> shift;

nvt_read_register_exit:
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen clear status & enable fw crc function.

return:
	N/A.
*******************************************************/
void nvt_fw_crc_enable(void)
{
	uint8_t buf[8] = {0};

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR);

	//---clear fw reset status---
	buf[0] = EVENT_MAP_RESET_COMPLETE & (0x7F);
	buf[1] = 0x00;
	buf[2] = 0x00;
	buf[3] = 0x00;
	buf[4] = 0x00;
	buf[5] = 0x00;
	buf[6] = 0x00;
	CTP_SPI_WRITE(ts->client, buf, 7);

	//---enable fw crc---
	buf[0] = EVENT_MAP_HOST_CMD & (0x7F);
	buf[1] = 0xAE;	//enable fw crc command
	buf[2] = 0x00;
	CTP_SPI_WRITE(ts->client, buf, 3);
}

/*******************************************************
Description:
	Novatek touchscreen set boot ready function.

return:
	N/A.
*******************************************************/
void nvt_boot_ready(void)
{
	//---write BOOT_RDY status cmds---
	nvt_write_addr(ts->mmap->BOOT_RDY_ADDR, 1);

	mdelay(5);

	if (ts->hw_crc == HWCRC_NOSUPPORT) {
		//---write BOOT_RDY status cmds---
		nvt_write_addr(ts->mmap->BOOT_RDY_ADDR, 0);

		//---write POR_CD cmds---
		nvt_write_addr(ts->mmap->POR_CD_ADDR, 0xA0);
	}
}

/*******************************************************
Description:
	Novatek touchscreen enable auto copy mode function.

return:
	N/A.
*******************************************************/
void nvt_tx_auto_copy_mode(void)
{
	if (ts->auto_copy == CHECK_SPI_DMA_TX_INFO) {
		//---write TX_AUTO_COPY_EN cmds---
		nvt_write_addr(ts->mmap->TX_AUTO_COPY_EN, 0x69);
	} else if (ts->auto_copy == CHECK_TX_AUTO_COPY_EN) {
		//---write SPI_MST_AUTO_COPY cmds---
		nvt_write_addr(ts->mmap->TX_AUTO_COPY_EN, 0x56);
	}

	NVT_LOG("tx auto copy mode %d enable\n", ts->auto_copy);
}

/*******************************************************
Description:
	Novatek touchscreen check spi dma tx info function.

return:
	Executive outcomes. 0---succeed. -1---fail.
*******************************************************/
int32_t nvt_check_spi_dma_tx_info(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 200;

	if (ts->mmap->SPI_DMA_TX_INFO == 0) {
		NVT_ERR("error, SPI_DMA_TX_INFO = 0\n");
		return -1;
	}

	for (i = 0; i < retry; i++) {
		//---set xdata index to SPI_DMA_TX_INFO---
		nvt_set_page(ts->mmap->SPI_DMA_TX_INFO);

		//---read spi dma status---
		buf[0] = ts->mmap->SPI_DMA_TX_INFO & 0x7F;
		buf[1] = 0xFF;
		CTP_SPI_READ(ts->client, buf, 2);

		if (buf[1] == 0x00)
			break;

		usleep_range(1000, 1000);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

/*******************************************************
Description:
	Novatek touchscreen check tx auto copy state function.

return:
	Executive outcomes. 0---succeed. -1---fail.
*******************************************************/
int32_t nvt_check_tx_auto_copy(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 200;

	if (ts->mmap->TX_AUTO_COPY_EN == 0) {
		NVT_ERR("error, TX_AUTO_COPY_EN = 0\n");
		return -1;
	}

	for (i = 0; i < retry; i++) {
		//---set xdata index to SPI_MST_AUTO_COPY---
		nvt_set_page(ts->mmap->TX_AUTO_COPY_EN);

		//---read auto copy status---
		buf[0] = ts->mmap->TX_AUTO_COPY_EN & 0x7F;
		buf[1] = 0xFF;
		CTP_SPI_READ(ts->client, buf, 2);

		if (buf[1] == 0x00)
			break;

		usleep_range(1000, 1000);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

/*******************************************************
Description:
	Novatek touchscreen wait auto copy finished function.

return:
	Executive outcomes. 0---succeed. -1---fail.
*******************************************************/
int32_t nvt_wait_auto_copy(void)
{
	if (ts->auto_copy == CHECK_SPI_DMA_TX_INFO) {
		return nvt_check_spi_dma_tx_info();
	} else if (ts->auto_copy == CHECK_TX_AUTO_COPY_EN) {
		return nvt_check_tx_auto_copy();
	} else {
		NVT_ERR("failed, not support mode %d!\n", ts->auto_copy);
		return -1;
	}
}

/*******************************************************
Description:
	Novatek touchscreen eng reset cmd
    function.

return:
	n.a.
*******************************************************/
void nvt_eng_reset(void)
{
	//---eng reset cmds to ENG_RST_ADDR---
	nvt_write_addr(ENG_RST_ADDR, 0x5A);

	mdelay(1);	//wait tMCU_Idle2TP_REX_Hi after TP_RST
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU
    function.

return:
	n.a.
*******************************************************/
void nvt_sw_reset(void)
{
	//---software reset cmds to SWRST_SIF_ADDR---
	nvt_write_addr(ts->swrst_sif_addr, 0x55);

	msleep(10);
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU then into idle mode
    function.

return:
	n.a.
*******************************************************/
void nvt_sw_reset_idle(void)
{
	//---MCU idle cmds to SWRST_SIF_ADDR---
	nvt_write_addr(ts->swrst_sif_addr, 0xAA);

	msleep(15);
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU (boot) function.

return:
	n.a.
*******************************************************/
void nvt_bootloader_reset(void)
{
	//---reset cmds to SWRST_SIF_ADDR---
	nvt_write_addr(ts->swrst_sif_addr, 0x69);

	mdelay(5);	//wait tBRST2FR after Bootload RST

	if (SPI_RD_FAST_ADDR) {
		/* disable SPI_RD_FAST */
		nvt_write_addr(SPI_RD_FAST_ADDR, 0x00);
	}

	NVT_LOG("end\n");
}

/*******************************************************
Description:
	Novatek touchscreen clear FW status function.

return:
	Executive outcomes. 0---succeed. -1---fail.
*******************************************************/
int32_t nvt_clear_fw_status(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 20;

	for (i = 0; i < retry; i++) {
		//---set xdata index to EVENT BUF ADDR---
		nvt_set_page(ts->mmap->EVENT_BUF_ADDR);

		//---clear fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		CTP_SPI_WRITE(ts->client, buf, 2);

		//---read fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0xFF;
		CTP_SPI_READ(ts->client, buf, 2);

		if (buf[1] == 0x00)
			break;

		usleep_range(10000, 10000);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

/*******************************************************
Description:
	Novatek touchscreen check FW status function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int32_t nvt_check_fw_status(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 50;

	usleep_range(20000, 20000);

	for (i = 0; i < retry; i++) {
		//---set xdata index to EVENT BUF ADDR---
		nvt_set_page(ts->mmap->EVENT_BUF_ADDR);

		//---read fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		CTP_SPI_READ(ts->client, buf, 2);

		if ((buf[1] & 0xF0) == 0xA0)
			break;

		usleep_range(10000, 10000);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

/*******************************************************
Description:
	Novatek touchscreen check FW reset state function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int32_t nvt_check_fw_reset_state(RST_COMPLETE_STATE check_reset_state)
{
	uint8_t buf[8] = {0};
	int32_t ret = 0;
	int32_t retry = 0;
	int32_t retry_max = (check_reset_state == RESET_STATE_INIT) ? 10 : 50;

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR);

	while (1) {
		//---read reset state---
		buf[0] = EVENT_MAP_RESET_COMPLETE;
		buf[1] = 0x00;
		CTP_SPI_READ(ts->client, buf, 6);

		if ((buf[1] >= check_reset_state) && (buf[1] <= RESET_STATE_MAX)) {
			ret = 0;
			break;
		}

		retry++;
		if(unlikely(retry > retry_max)) {
			NVT_ERR("error, retry=%d, buf[1]=0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
				retry, buf[1], buf[2], buf[3], buf[4], buf[5]);
			ret = -1;
			break;
		}

		usleep_range(10000, 10000);
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen get firmware related information
	function.

return:
	Executive outcomes. 0---success. -1---fail.
*******************************************************/
int32_t nvt_get_fw_info(void)
{
	uint8_t buf[64] = {0};
	uint32_t retry_count = 0;
	int32_t ret = 0;

info_retry:
	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR);

	//---read fw info---
	buf[0] = EVENT_MAP_FWINFO;
	CTP_SPI_READ(ts->client, buf, 39);
	if ((buf[1] + buf[2]) != 0xFF) {
		NVT_ERR("FW info is broken! fw_ver=0x%02X, ~fw_ver=0x%02X\n", buf[1], buf[2]);
		if (retry_count < 3) {
			retry_count++;
			NVT_ERR("retry_count=%d\n", retry_count);
			goto info_retry;
		} else {
			ts->fw_ver = 0;
			ts->x_num = TOUCH_DEFAULT_NUM_X;
			ts->y_num = TOUCH_DEFAULT_NUM_Y;
			ts->abs_x_max = TOUCH_DEFAULT_MAX_WIDTH;
			ts->abs_y_max = TOUCH_DEFAULT_MAX_HEIGHT;
			NVT_ERR("Set default fw_ver=%d, abs_x_max=%d, abs_y_max=%d!\n",
					ts->fw_ver, ts->abs_x_max, ts->abs_y_max);
			ret = -1;
			goto out;
		}
	}
	ts->fw_ver = buf[1];
	ts->x_num = buf[3];
	ts->y_num = buf[4];
	ts->abs_x_max = (uint16_t)((buf[5] << 8) | buf[6]);
	ts->abs_y_max = (uint16_t)((buf[7] << 8) | buf[8]);
	ts->nvt_pid = (uint16_t)((buf[36] << 8) | buf[35]);
	NVT_LOG("fw_ver=0x%02X, fw_type=0x%02X, PID=0x%04X\n", ts->fw_ver, buf[14], ts->nvt_pid);

	ret = 0;
out:

	return ret;
}

int nvt_set_custom_cmd(u8 cmd, u16 value) {
	u8 buf[6] = {0};
	u32 retries = 0;
	int ret, i = 0;

	NVT_LOG("++\n");

	msleep(35);

	ret = nvt_set_page(ts->mmap->EVENT_BUF_ADDR);
	if (ret) {
		NVT_ERR("nvt_set_page fail! ret=%d\n", ret);
		goto nvt_set_custom_cmd_out;
	}

	while (retries < 5) {
		//first byte is offset, it's processed by spi loop
		buf[0] = 0x50;
		//event loop will read 0xBF and call xiaomi's cmd parser
		buf[1] = 0xBF;
		//0x51 is a command
		buf[2] = cmd;
		//0x52 must be 0
		buf[3] = 0x00;
		//0x53-0x54 is a short value
		buf[4] = value & 0xFF;
		buf[5] = (value >> 8) & 0xFF;

		ret = CTP_SPI_WRITE(ts->client, (u8 *)buf, 6);
		if (ret) {
			NVT_ERR("CTP_SPI_WRITE fail! ret=%d\n", ret);
			goto nvt_set_custom_cmd_out;
		}

		usleep_range_state(20000, 21000, TASK_UNINTERRUPTIBLE);

		//offset is 0x50
		buf[0] = 0x50;
		//read status
		buf[1] = 0xFF;
		ret = CTP_SPI_READ(ts->client, (u8 *)buf, 2);
		if (ret) {
			NVT_ERR("CTP_SPI_READ fail! ret=%d\n", ret);
			goto nvt_set_custom_cmd_out;
		}

		if (!buf[1]) break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		NVT_ERR("send cmd failed, buf[1] = 0x%02X\n", buf[1]);
		return -1;
	} else {
		NVT_LOG("send cmd success, tried %d times\n", i);
	}

nvt_set_custom_cmd_out:
	NVT_LOG("--\n");

	return ret;
}

void nvt_set_doze_delay(u16 value) {
	u8 buf[3] = {0};

	nvt_set_page(0x113718);
	buf[0] = 0x113718 & 0x7F;
	buf[1] = value & 0xFF;
	buf[2] = value >> 8;
	CTP_SPI_WRITE(ts->client, buf, 3);
}

/*******************************************************
Description:
	Novatek touchscreen parse device tree function.

return:
	n.a.
*******************************************************/
#ifdef CONFIG_OF
static int32_t nvt_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	int32_t ret = 0;

	ts->irq_gpiod = fwnode_gpiod_get_index(dev_fwnode(dev), "novatek,irq", 0,
					       GPIOD_IN, "NVT-int");
	if (IS_ERR(ts->irq_gpiod)) {
		NVT_ERR("Failed to request NVT-int GPIO\n");
		return PTR_ERR(ts->irq_gpiod);
	}
	NVT_LOG("novatek,irq-gpio: gpiod obtained\n");

	ret = of_property_read_u32(np, "novatek,spi-rd-fast-addr", &SPI_RD_FAST_ADDR);
	if (ret) {
		NVT_LOG("not support novatek,spi-rd-fast-addr\n");
		SPI_RD_FAST_ADDR = 0;
		ret = 0;
	} else {
		NVT_LOG("SPI_RD_FAST_ADDR=0x%06X\n", SPI_RD_FAST_ADDR);
	}

	ret = of_property_read_string(np, "firmware-name", &ts->fw_name);
	if (ret) {
		NVT_LOG("Unable to get touchscreen firmware name\n");
		ts->fw_name = BOOT_UPDATE_FIRMWARE_NAME;
	}

	return ret;
}
#else
static int32_t nvt_parse_dt(struct device *dev)
{
	ts->irq_gpiod = gpio_to_desc(NVTTOUCH_INT_PIN);
	if (!ts->irq_gpiod) {
		NVT_ERR("Failed to get NVT-int GPIO descriptor\n");
		return -EINVAL;
	}
	return 0;
}
#endif

/*******************************************************
Description:
	Novatek touchscreen config and request gpio

return:
	Executive outcomes. 0---succeed. not 0---failed.
*******************************************************/
static int nvt_gpio_config(struct nvt_ts_data *ts)
{
	int32_t ret = 0;

	if (IS_ERR(ts->irq_gpiod)) {
		NVT_ERR("Invalid NVT-int GPIO descriptor\n");
		return PTR_ERR(ts->irq_gpiod);
	}

	/* Ensure GPIO is configured as input */
	ret = gpiod_direction_input(ts->irq_gpiod);
	if (ret) {
		NVT_ERR("Failed to set NVT-int GPIO direction\n");
		return ret;
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen deconfig gpio

return:
	n.a.
*******************************************************/
static void nvt_gpio_deconfig(struct nvt_ts_data *ts)
{
	if (!IS_ERR(ts->irq_gpiod))
		gpiod_put(ts->irq_gpiod);
}

static uint8_t nvt_fw_recovery(uint8_t *point_data)
{
	uint8_t i = 0;
	uint8_t detected = true;

	/* check pattern */
	for (i=1 ; i<7 ; i++) {
		if (point_data[i] != 0x77) {
			detected = false;
			break;
		}
	}

	return detected;
}

#if NVT_TOUCH_WDT_RECOVERY
static uint8_t recovery_cnt = 0;
static uint8_t nvt_wdt_fw_recovery(uint8_t *point_data)
{
   uint32_t recovery_cnt_max = 10;
   uint8_t recovery_enable = false;
   uint8_t i = 0;

   recovery_cnt++;

   /* check pattern */
   for (i=1 ; i<7 ; i++) {
       if ((point_data[i] != 0xFD) && (point_data[i] != 0xFE)) {
           recovery_cnt = 0;
           break;
       }
   }

   if (recovery_cnt > recovery_cnt_max){
       recovery_enable = true;
       recovery_cnt = 0;
   }

   return recovery_enable;
}

void nvt_read_fw_history(uint32_t fw_history_addr)
{
	uint8_t i = 0;
	uint8_t buf[65];
	char str[128];

	if (fw_history_addr == 0)
		return;

    nvt_set_page(fw_history_addr);

    buf[0] = (uint8_t) (fw_history_addr & 0x7F);
    CTP_SPI_READ(ts->client, buf, 64+1);	//read 64bytes history

	//print all data
	NVT_ERR("fw history 0x%X: \n", fw_history_addr);
	for (i = 0; i < 4; i++) {
		snprintf(str, sizeof(str),
				"%02X %02X %02X %02X %02X %02X %02X %02X  "
				"%02X %02X %02X %02X %02X %02X %02X %02X\n",
				buf[1+i*16], buf[2+i*16], buf[3+i*16], buf[4+i*16],
				buf[5+i*16], buf[6+i*16], buf[7+i*16], buf[8+i*16],
				buf[9+i*16], buf[10+i*16], buf[11+i*16], buf[12+i*16],
				buf[13+i*16], buf[14+i*16], buf[15+i*16], buf[16+i*16]);
		NVT_ERR("%s", str);
	}

	nvt_set_page(ts->mmap->EVENT_BUF_ADDR);
}
#endif	/* #if NVT_TOUCH_WDT_RECOVERY */

/*******************************************************
Description:
	Novatek touchscreen work function.

return:
	n.a.
*******************************************************/
static irqreturn_t nvt_ts_work_func(int irq, void *data)
{
	int32_t ret = -1;
	uint8_t fw_state[7] = { 0 };
	bool frame_captured = false;

	mutex_lock(&ts->lock);

	if (ts->dev_pm_suspend) {
		ret = wait_for_completion_timeout(&ts->dev_pm_suspend_completion, msecs_to_jiffies(500));
		if (!ret) {
			NVT_ERR("system(spi) can't finished resuming procedure, skip it\n");
			goto XFER_ERROR;
		}
	}

	if (READ_ONCE(ts->thp_capture_enabled)) {
		ret = nvt_thp_read_frame(fw_state);
		frame_captured = ret >= 0;
	} else {
		nvt_set_page(ts->mmap->EVENT_BUF_ADDR);
		fw_state[0] = 0;
		ret = CTP_SPI_READ(ts->client, fw_state, sizeof(fw_state));
	}
	if (ret < 0) {
		NVT_ERR("CTP_SPI_READ failed.(%d)\n", ret);
		goto XFER_ERROR;
	}
#if NVT_TOUCH_WDT_RECOVERY
	/* ESD protect by WDT */
	if (nvt_wdt_fw_recovery(fw_state)) {
		NVT_ERR("Recover for fw reset, %02X\n", fw_state[1]);
		if (fw_state[1] == 0xFE) {
			nvt_sw_reset_idle();
		}
		nvt_read_fw_history(ts->mmap->MMAP_HISTORY_EVENT0);
		nvt_read_fw_history(ts->mmap->MMAP_HISTORY_EVENT1);
		nvt_update_firmware(ts->fw_name);
		//enable idle baseline update
		nvt_set_custom_cmd(0x19, 0x00);
		nvt_set_doze_delay(120);
		//enter doze mode
		nvt_set_custom_cmd(0x01, 0x02);
		nvt_thp_restore_stylus();
		goto XFER_ERROR;
	}
#endif /* #if NVT_TOUCH_WDT_RECOVERY */

	/* ESD protect by FW handshake */
	if (nvt_fw_recovery(fw_state)) {
		goto XFER_ERROR;
	}

	if (frame_captured)
		nvt_thp_publish_frame();

XFER_ERROR:

	mutex_unlock(&ts->lock);
	return IRQ_HANDLED;
}


/*******************************************************
Description:
	Novatek touchscreen check chip version trim function.

return:
	Executive outcomes. 0---NVT IC. -1---not NVT IC.
*******************************************************/
static int32_t nvt_ts_check_chip_ver_trim(struct nvt_ts_hw_reg_addr_info hw_regs)
{
	uint8_t buf[8] = {0};
	int32_t retry = 0;
	int32_t list = 0;
	int32_t i = 0;
	int32_t found_nvt_chip = 0;
	int32_t ret = -1;
	uint8_t enb_casc = 0;

	/* hw reg mapping */
	ts->chip_ver_trim_addr = hw_regs.chip_ver_trim_addr;
	ts->swrst_sif_addr = hw_regs.swrst_sif_addr;
	ts->crc_err_flag_addr = hw_regs.crc_err_flag_addr;

	NVT_LOG("check chip ver trim with chip_ver_trim_addr=0x%06x, "
			"swrst_sif_addr=0x%06x, crc_err_flag_addr=0x%06x\n",
			ts->chip_ver_trim_addr, ts->swrst_sif_addr, ts->crc_err_flag_addr);

	//---Check for 5 times---
	for (retry = 5; retry > 0; retry--) {

		nvt_bootloader_reset();

		nvt_set_page(ts->chip_ver_trim_addr);

		buf[0] = ts->chip_ver_trim_addr & 0x7F;
		buf[1] = 0x00;
		buf[2] = 0x00;
		buf[3] = 0x00;
		buf[4] = 0x00;
		buf[5] = 0x00;
		buf[6] = 0x00;
		CTP_SPI_WRITE(ts->client, buf, 7);

		buf[0] = ts->chip_ver_trim_addr & 0x7F;
		buf[1] = 0x00;
		buf[2] = 0x00;
		buf[3] = 0x00;
		buf[4] = 0x00;
		buf[5] = 0x00;
		buf[6] = 0x00;
		CTP_SPI_READ(ts->client, buf, 7);
		NVT_LOG("buf[1]=0x%02X, buf[2]=0x%02X, buf[3]=0x%02X, buf[4]=0x%02X, buf[5]=0x%02X, buf[6]=0x%02X\n",
			buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);

		// compare read chip id on supported list
		for (list = 0; list < (sizeof(trim_id_table) / sizeof(struct nvt_ts_trim_id_table)); list++) {
			found_nvt_chip = 0;

			// compare each byte
			for (i = 0; i < NVT_ID_BYTE_MAX; i++) {
				if (trim_id_table[list].mask[i]) {
					if (buf[i + 1] != trim_id_table[list].id[i])
						break;
				}
			}

			if (i == NVT_ID_BYTE_MAX) {
				found_nvt_chip = 1;
			}

			if (found_nvt_chip) {
				NVT_LOG("This is NVT touch IC\n");
				if (trim_id_table[list].mmap->ENB_CASC_REG.addr) {
					/* check single or cascade */
					nvt_read_reg(trim_id_table[list].mmap->ENB_CASC_REG, &enb_casc);
					/* NVT_LOG("ENB_CASC=0x%02X\n", enb_casc); */
					if (enb_casc & 0x01) {
						NVT_LOG("Single Chip\n");
						ts->mmap = trim_id_table[list].mmap;
					} else {
						NVT_LOG("Cascade Chip\n");
						ts->mmap = trim_id_table[list].mmap_casc;
					}
				} else {
					/* for chip that do not have ENB_CASC */
					ts->mmap = trim_id_table[list].mmap;
				}
				ts->hw_crc = trim_id_table[list].hwinfo->hw_crc;
				ts->auto_copy = trim_id_table[list].hwinfo->auto_copy;

				/* hw reg re-mapping */
				ts->chip_ver_trim_addr = trim_id_table[list].hwinfo->hw_regs->chip_ver_trim_addr;
				ts->swrst_sif_addr = trim_id_table[list].hwinfo->hw_regs->swrst_sif_addr;
				ts->crc_err_flag_addr = trim_id_table[list].hwinfo->hw_regs->crc_err_flag_addr;

				NVT_LOG("set reg chip_ver_trim_addr=0x%06x, "
						"swrst_sif_addr=0x%06x, crc_err_flag_addr=0x%06x\n",
						ts->chip_ver_trim_addr, ts->swrst_sif_addr, ts->crc_err_flag_addr);

				ret = 0;
				goto out;
			} else {
				ts->mmap = NULL;
				ret = -1;
			}
		}

		msleep(10);
	}

out:
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen check chip version trim loop
	function. Check chip version trim via hw regs table.

return:
	Executive outcomes. 0---NVT IC. -1---not NVT IC.
*******************************************************/
static int32_t nvt_ts_check_chip_ver_trim_loop(void) {
    uint8_t i = 0;
	int32_t ret = 0;

	struct nvt_ts_hw_reg_addr_info hw_regs_table[] = {
		hw_reg_addr_info,
		hw_reg_addr_info_old_w_isp,
		hw_reg_addr_info_legacy_w_isp
	};

    for (i = 0; i < (sizeof(hw_regs_table) / sizeof(struct nvt_ts_hw_reg_addr_info)); i++) {
        //---check chip version trim---
        ret = nvt_ts_check_chip_ver_trim(hw_regs_table[i]);
		if (!ret) {
			break;
		}
    }

	return ret;
}

static void nvt_resume_work(struct work_struct *work)
{
	struct nvt_ts_data *ts_core = container_of(work, struct nvt_ts_data, resume_work);
	nvt_ts_resume(&ts_core->client->dev);
}

/*******************************************************
Description:
	Novatek touchscreen driver probe function.

return:
	Executive outcomes. 0---succeed. negative---failed
*******************************************************/
static int32_t nvt_ts_probe(struct spi_device *client)
{
	int32_t ret = 0;
	int32_t retry_count = 0;

	NVT_LOG("start\n");

	ts = (struct nvt_ts_data *)kzalloc(sizeof(struct nvt_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		NVT_ERR("failed to allocated memory for nvt ts data\n");
		return -ENOMEM;
	}

	ts->xbuf = (uint8_t *)kzalloc(NVT_XBUF_LEN, GFP_KERNEL);
	if (ts->xbuf == NULL) {
		NVT_ERR("kzalloc for xbuf failed!\n");
		ret = -ENOMEM;
		goto err_malloc_xbuf;
	}

	ts->rbuf = (uint8_t *)kzalloc(NVT_READ_LEN, GFP_KERNEL);
	if(ts->rbuf == NULL) {
		NVT_ERR("kzalloc for rbuf failed!\n");
		ret = -ENOMEM;
		goto err_malloc_rbuf;
	}

	ts->client = client;
	spi_set_drvdata(client, ts);

	//---prepare for spi parameter---
	if (ts->client->controller->flags & SPI_CONTROLLER_HALF_DUPLEX) {
		NVT_ERR("Full duplex not supported by master\n");
		ret = -EIO;
		goto err_ckeck_full_duplex;
	}
	ts->client->bits_per_word = 8;
	ts->client->mode = SPI_MODE_0;

	ret = spi_setup(ts->client);
	if (ret < 0) {
		NVT_ERR("Failed to perform SPI setup\n");
		goto err_spi_setup;
	}

	NVT_LOG("mode=%d, max_speed_hz=%d\n", ts->client->mode, ts->client->max_speed_hz);

	//---parse dts---
	ret = nvt_parse_dt(&client->dev);
	if (ret) {
		NVT_ERR("parse dt error\n");
		goto err_spi_setup;
	}

	//---request and config GPIOs---
	ret = nvt_gpio_config(ts);
	if (ret) {
		NVT_ERR("gpio config error!\n");
		goto err_gpio_config_failed;
	}

	mutex_init(&ts->lock);
	mutex_init(&ts->xbuf_lock);
	mutex_init(&ts->thp_lock);
	init_waitqueue_head(&ts->thp_stream_wait);

#if defined(CONFIG_DRM_PANEL)
	/* If the device follows a DRM panel, configure panel follower */
	if (drm_is_panel_follower(&client->dev)) {
		ts->panel_follower.funcs = &nt36xxx_panel_follower_funcs;
		devm_drm_panel_add_follower(&client->dev, &ts->panel_follower);
	}
#endif

	//---eng reset before TP_RESX high
	nvt_eng_reset();

	// need 10ms delay after POR(power on reset)
	msleep(10);

	while (!ts->panel_on) {
		if(retry_count > 5) {
			ret = -EPROBE_DEFER;
			NVT_ERR("panel wait failed, ret=%d\n", ret);
			goto err_panelwait_failed;
		}
		NVT_LOG("panel is off, retry=%d\n", retry_count);
		retry_count++;
		msleep(200);
	}

	//---check chip version trim---
	ret = nvt_ts_check_chip_ver_trim_loop();
	if (ret) {
		NVT_ERR("chip is not identified\n");
		ret = -EINVAL;
		goto err_chipvertrim_failed;
	}

	ts->thp_frame = kzalloc(NVT_THP_FRAME_LEN, GFP_KERNEL);
	if (!ts->thp_frame) {
		ret = -ENOMEM;
		goto err_chipvertrim_failed;
	}
	ts->thp_stream_buf = vzalloc(NVT_THP_STREAM_SIZE);
	if (!ts->thp_stream_buf) {
		ret = -ENOMEM;
		goto err_chipvertrim_failed;
	}
	kfifo_init(&ts->thp_stream_fifo, ts->thp_stream_buf,
		   NVT_THP_STREAM_SIZE);

	ts->x_num = TOUCH_DEFAULT_NUM_X;
	ts->y_num = TOUCH_DEFAULT_NUM_Y;
	ts->abs_x_max = TOUCH_DEFAULT_MAX_WIDTH;
	ts->abs_y_max = TOUCH_DEFAULT_MAX_HEIGHT;

	ts->int_trigger_type = INT_TRIGGER_TYPE;

	//---set int-pin & request irq---
	client->irq = gpiod_to_irq(ts->irq_gpiod);
	if (client->irq) {
		NVT_LOG("int_trigger_type=%d\n", ts->int_trigger_type);
		ts->irq_enabled = true;
		ret = request_threaded_irq(client->irq, NULL, nvt_ts_work_func,
				ts->int_trigger_type | IRQF_ONESHOT, NVT_SPI_NAME, ts);
		if (ret != 0) {
			NVT_ERR("request irq failed. ret=%d\n", ret);
			goto err_int_request_failed;
		} else {
			nvt_irq_enable(false);
			NVT_LOG("request irq %d succeed\n", client->irq);
		}
	}

	ts->ic_state = NVT_IC_INIT;
	ts->dev_pm_suspend = false;
	init_completion(&ts->dev_pm_suspend_completion);

#if BOOT_UPDATE_FIRMWARE
	nvt_fwu_wq = alloc_workqueue("nvt_fwu_wq", WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (!nvt_fwu_wq) {
		NVT_ERR("nvt_fwu_wq create workqueue failed\n");
		ret = -ENOMEM;
		goto err_create_nvt_fwu_wq_failed;
	}
	INIT_DELAYED_WORK(&ts->nvt_fwu_work, Boot_Update_Firmware);
	// please make sure boot update start after display reset(RESX) sequence
	queue_delayed_work(nvt_fwu_wq, &ts->nvt_fwu_work, 0);
#endif

	ts->event_wq = alloc_workqueue("nvt-event-queue",
		WQ_UNBOUND | WQ_HIGHPRI | WQ_CPU_INTENSIVE, 1);
	if (!ts->event_wq) {
		NVT_ERR("Can not create work thread for suspend/resume!!");
		ret = -ENOMEM;
		goto err_alloc_work_thread_failed;
	}

	INIT_WORK(&ts->resume_work, nvt_resume_work);

	ts->thp_raw_proc = proc_create("nvt_thp_raw", 0200, NULL,
				       &nvt_thp_raw_ops);
	ts->thp_stream_proc = proc_create("nvt_thp_stream", 0400, NULL,
					  &nvt_thp_stream_ops);
	ts->thp_status_proc = proc_create("nvt_thp_status", 0400, NULL,
					  &nvt_thp_status_ops);
	ts->thp_stylus_proc = proc_create("nvt_thp_stylus", 0600, NULL,
					  &nvt_thp_stylus_ops);
	if (!ts->thp_raw_proc || !ts->thp_stream_proc ||
	    !ts->thp_status_proc || !ts->thp_stylus_proc) {
		ret = -ENOMEM;
		goto err_create_thp_proc;
	}

	bTouchIsAwake = 1;
	NVT_LOG("end\n");

	nvt_irq_enable(true);

	return 0;

err_create_thp_proc:
	if (ts->thp_status_proc)
		proc_remove(ts->thp_status_proc);
	if (ts->thp_stylus_proc)
		proc_remove(ts->thp_stylus_proc);
	if (ts->thp_stream_proc)
		proc_remove(ts->thp_stream_proc);
	if (ts->thp_raw_proc)
		proc_remove(ts->thp_raw_proc);
	destroy_workqueue(ts->event_wq);
	ts->event_wq = NULL;

err_alloc_work_thread_failed:
#if BOOT_UPDATE_FIRMWARE
	if (nvt_fwu_wq) {
		cancel_delayed_work_sync(&ts->nvt_fwu_work);
		destroy_workqueue(nvt_fwu_wq);
		nvt_fwu_wq = NULL;
	}
err_create_nvt_fwu_wq_failed:
#endif
	free_irq(client->irq, ts);
err_int_request_failed:
err_panelwait_failed:
err_chipvertrim_failed:
	vfree(ts->thp_stream_buf);
	ts->thp_stream_buf = NULL;
	kfree(ts->thp_frame);
	ts->thp_frame = NULL;
	mutex_destroy(&ts->thp_lock);
	mutex_destroy(&ts->xbuf_lock);
	mutex_destroy(&ts->lock);
	nvt_gpio_deconfig(ts);
err_gpio_config_failed:
err_spi_setup:
err_ckeck_full_duplex:
	if (ts->rbuf) {
		kfree(ts->rbuf);
		ts->rbuf = NULL;
	}
err_malloc_rbuf:
	if (ts->xbuf) {
		kfree(ts->xbuf);
		ts->xbuf = NULL;
	}
err_malloc_xbuf:
	if (ts) {
		kfree(ts);
		ts = NULL;
	}
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen driver release function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static void nvt_ts_remove(struct spi_device *client)
{
	NVT_LOG("Removing driver...\n");

	bTouchIsAwake = 0;
	WRITE_ONCE(ts->thp_capture_enabled, false);
	wake_up_interruptible(&ts->thp_stream_wait);
	proc_remove(ts->thp_status_proc);
	proc_remove(ts->thp_stylus_proc);
	proc_remove(ts->thp_stream_proc);
	proc_remove(ts->thp_raw_proc);

	destroy_workqueue(ts->event_wq);
	ts->event_wq = NULL;

#if BOOT_UPDATE_FIRMWARE
	if (nvt_fwu_wq) {
		cancel_delayed_work_sync(&ts->nvt_fwu_work);
		destroy_workqueue(nvt_fwu_wq);
		nvt_fwu_wq = NULL;
	}
#endif

	nvt_irq_enable(false);
	free_irq(client->irq, ts);

	mutex_destroy(&ts->xbuf_lock);
	mutex_destroy(&ts->lock);
	mutex_destroy(&ts->thp_lock);

	nvt_gpio_deconfig(ts);

	spi_set_drvdata(client, NULL);

	if (ts->rbuf) {
		kfree(ts->rbuf);
		ts->rbuf = NULL;
	}

	if (ts->xbuf) {
		kfree(ts->xbuf);
		ts->xbuf = NULL;
	}

	kfree(ts->thp_frame);
	ts->thp_frame = NULL;
	vfree(ts->thp_stream_buf);
	ts->thp_stream_buf = NULL;

	if (ts) {
		kfree(ts);
		ts = NULL;
	}

	return;
}

static void nvt_ts_shutdown(struct spi_device *client)
{
	NVT_LOG("Shutdown driver...\n");

	nvt_irq_enable(false);

	destroy_workqueue(ts->event_wq);
	ts->event_wq = NULL;

#if BOOT_UPDATE_FIRMWARE
	if (nvt_fwu_wq) {
		cancel_delayed_work_sync(&ts->nvt_fwu_work);
		destroy_workqueue(nvt_fwu_wq);
		nvt_fwu_wq = NULL;
	}
#endif
}

/*******************************************************
Description:
	Novatek touchscreen driver suspend function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_suspend(struct device *dev)
{
	if (!bTouchIsAwake) {
		NVT_LOG("Touch is already suspend\n");
		return 0;
	}

	pm_stay_awake(dev);
	ts->ic_state = NVT_IC_SUSPEND_IN;

	nvt_irq_enable(false);

	mutex_lock(&ts->lock);

	NVT_LOG("start\n");

	bTouchIsAwake = 0;

	mutex_unlock(&ts->lock);

	msleep(50);

	if (likely(ts->ic_state == NVT_IC_SUSPEND_IN))
		ts->ic_state = NVT_IC_SUSPEND_OUT;
	else
		NVT_ERR("IC state may error,caused by suspend/resume flow, please CHECK!!");

	NVT_LOG("end\n");
	pm_relax(dev);

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen driver resume function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_resume(struct device *dev)
{
	if (bTouchIsAwake) {
		NVT_LOG("Touch is already resume\n");
		return 0;
	}

	if (ts->fw_name == NULL) {
		NVT_ERR("fw has not been loaded and cannot be resume!\n");
		return 0;
	}

	if (ts->dev_pm_suspend)
		pm_stay_awake(dev);

	mutex_lock(&ts->lock);

	NVT_LOG("start\n");

	ts->ic_state = NVT_IC_RESUME_IN;

	// please make sure display reset(RESX) sequence and mipi dsi cmds sent before this
	if (nvt_update_firmware(ts->fw_name)) {
		NVT_ERR("download firmware failed, ignore check fw state\n");
	} else {
		nvt_check_fw_reset_state(RESET_STATE_REK);
	}

	nvt_irq_enable(true);

	bTouchIsAwake = 1;

	if (likely(ts->ic_state == NVT_IC_RESUME_IN)) {
		ts->ic_state = NVT_IC_RESUME_OUT;
	} else {
		NVT_ERR("IC state may error,caused by suspend/resume flow, please CHECK!!");
	}

	//enable idle baseline update
	nvt_set_custom_cmd(0x19, 0x00);
	nvt_set_doze_delay(120);
	//enter doze mode
	nvt_set_custom_cmd(0x01, 0x02);
	nvt_thp_restore_stylus();

	mutex_unlock(&ts->lock);

	NVT_LOG("end\n");

	if (ts->dev_pm_suspend)
		pm_relax(dev);
	return 0;
}


#if defined(CONFIG_DRM_PANEL)
static int panel_prepared(struct drm_panel_follower *follower)
{
	struct nvt_ts_data *ts = container_of(follower, struct nvt_ts_data, panel_follower);

	ts->panel_on = true;

	if (!ts->event_wq) {
		return 0;
	}

	flush_workqueue(ts->event_wq);
	queue_work(ts->event_wq, &ts->resume_work);

	return 0;
}

static int panel_unpreparing(struct drm_panel_follower *follower)
{
	struct nvt_ts_data *ts = container_of(follower, struct nvt_ts_data, panel_follower);

	ts->panel_on = false;

	if (ts->event_wq)
		flush_workqueue(ts->event_wq);
	return nvt_ts_suspend(&ts->client->dev);
}

static struct drm_panel_follower_funcs nt36xxx_panel_follower_funcs = {
	.panel_prepared = panel_prepared,
	.panel_unpreparing = panel_unpreparing,
};
#endif

static const struct spi_device_id nvt_ts_id[] = {
	{ NVT_SPI_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, nvt_ts_id);

#ifdef CONFIG_OF
static struct of_device_id nvt_match_table[] = {
	{ .compatible = "novatek,NVT-ts-spi",},
	{ },
};
MODULE_DEVICE_TABLE(of, nvt_match_table);
#endif

#ifdef CONFIG_PM
static int nvt_pm_suspend(struct device *dev)
{
	NVT_LOG("system enters into pm_suspend");
	ts->dev_pm_suspend = true;
	reinit_completion(&ts->dev_pm_suspend_completion);
	NVT_LOG("system enters into pm_suspend2");
	return 0;
}

static int nvt_pm_resume(struct device *dev)
{
	NVT_LOG("system resume from pm_suspend");
	ts->dev_pm_suspend = false;
	complete(&ts->dev_pm_suspend_completion);
	NVT_LOG("system resume from pm_suspend2");
	return 0;
}

static const struct dev_pm_ops nvt_dev_pm_ops = {
	.suspend = nvt_pm_suspend,
	.resume = nvt_pm_resume,
};
#endif

static struct spi_driver nvt_spi_driver = {
	.probe		= nvt_ts_probe,
	.remove		= nvt_ts_remove,
	.shutdown	= nvt_ts_shutdown,
	.id_table	= nvt_ts_id,
	.driver = {
		.name	= NVT_SPI_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = nvt_match_table,
#endif
#ifdef CONFIG_PM
		.pm = &nvt_dev_pm_ops,
#endif
	},
};

module_spi_driver(nvt_spi_driver);

MODULE_DESCRIPTION("Novatek Touchscreen Driver");
MODULE_LICENSE("GPL");
