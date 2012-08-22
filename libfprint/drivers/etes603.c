/*
 * EgisTec ES603 driver for libfprint
 * Copyright (C) 2012 Patrick Marlier
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/* TODO LIST */
/* - Document main functions */
/* - Detect sweep direction (support only one direction currently) */
/* - GPL vs LGPL 3 vs LGPL 2.1 */
/* - Use different ways to detect fingers */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/time.h>
#include <libusb.h>

#define FP_COMPONENT "etes603"
#include <fp_internal.h>

/* ES603 info
 *  Sensor area: 192 x 4 pixels 
 *  Sensor gray: 16 gray levels/sensor pixel
 *  Sensor resolution: 508 dpi 
 *  USB Manufacturer ID: 1C7A
 *  USB Product ID: 0603
 */

/* Possible compatibility:
 *  LTT-SS500/SS501 (except 16 pixel instead of 4) ?
 *  1C7A 0803 ?
 *  CD15 6711 ?
 */

/* libusb defines */
#define EP_IN              0x81
#define EP_OUT             0x02
#define BULK_TIMEOUT       1000  /* TODO 1000 ms is not enough sometimes for full frame since the sensor is waiting moves. */

/* es603 defines */
#define FRAME_SIZE         384   /* size in bytes */
#define FRAME_WIDTH        192   /* pixels per row */
#define FRAME_HEIGHT       4     /* number of rows */
#define FRAMEFULL_SIZE     64000 /* size in bytes */
#define FRAMEFULL_WIDTH    256   /* pixels per row */
#define FRAMEFULL_HEIGHT   500   /* number of rows */

#define GAIN_SMALL_INIT    0x23  /* Initial small gain */
#define DCOFFSET_MIN       0x00  /* Minimum value for DCoffset */
#define DCOFFSET_MAX       0x35  /* Maximum value for DCoffset */
#define DTVRT_MAX          0x3A  /* Maximum value for DTVRT */

/* es603 commands */
#define CMD_READ_REG       0x01
#define CMD_WRITE_REG      0x02
#define CMD_READ_BUF       0x03
#define CMD_READ_BUF2      0x06
#define CMD_20             0x20
#define CMD_25             0x25
#define CMD_60             0x60 /* ??? */

#define CMD_OK             0x01

/* es603 registers */
#define REG_MAX            0x18 /* Maximum number of register to read/write in one command */
#define REG_MODE_CONTROL   0x02 /* Mode control */
#define REG_03             0x03 /* Contact register? */
#define REG_04             0x04 /* ? */
#define REG_10             0x10 /* MVS/FRMBUF Control Reg */
#define REG_1A             0x1A /* ? */
/* BEGIN init_sensor */
#define REG_20             0x20 /* (def: 0x00) */
#define REG_21             0x21 /* small_gain (def: 0x23) */
#define REG_22             0x22 /* normal_gain (def: 0x21) */
#define REG_23             0x23 /* large_gain (def: 0x20) */
#define REG_24             0x24 /* (def: 0x14) */
#define REG_25             0x25 /* (def: 0x6A) */
#define REG_26             0x26 /* VRB again? (def: 0x00) */
#define REG_27             0x27 /* VRT?? (def: 0x00) */
#define REG_28             0x28 /* (def: 0x00) */
#define REG_29             0x29 /* (def: 0xC0) */
#define REG_2A             0x2A /* (def: 0x50) */
#define REG_2B             0x2B /* (def: 0x50) */
#define REG_2C             0x2C /* (def: 0x4D) */
#define REG_2D             0x2D /* (def: 0x03) */
#define REG_2E             0x2E /* (def: 0x06) */
#define REG_2F             0x2F /* (def: 0x06) */
#define REG_30             0x30 /* (def: 0x10) */
#define REG_31             0x31 /* (def: 0x02) */
#define REG_32             0x32 /* (def: 0x14) */
#define REG_33             0x33 /* (def: 0x34) */
#define REG_34             0x34 /* (def: 0x01) */
#define REG_35             0x35 /* (def: 0x08) */
#define REG_36             0x36 /* (def: 0x03) */
#define REG_37             0x37 /* (def: 0x21) */
/* END init_sensor */

#define REG_ENC1           0x41 /* Encryption 1 */
#define REG_ENC2           0x42 
#define REG_ENC3           0x43
#define REG_ENC4           0x44
#define REG_ENC5           0x45
#define REG_ENC6           0x46
#define REG_ENC7           0x47
#define REG_ENC8           0x48 /* Encryption 8 */

#define REG_50             0x50
#define REG_51             0x51
#define REG_59             0x59
#define REG_5A             0x5A
#define REG_5B             0x5B

#define REG_INFO0          0x70 /* Sensor model byte0 */
#define REG_INFO1          0x71 /* Sensor model byte1 */
#define REG_INFO2          0x72 /* Sensor model byte2 */
#define REG_INFO3          0x73 /* Sensor model byte3 */

#define REG_93             0x93
#define REG_94             0x94

#define REG_GAIN           0xE0 /* Gain */
#define REG_VRT            0xE1 /* VRT */
#define REG_VRB            0xE2 /* VRB */
#define REG_DTVRT          0xE3 /* DTVRT, used for contact gain */
#define REG_VCO_CONTROL    0xE5 /* VCO_CONTROL: possible value 0x13, 0x14 (REALTIME) */
#define REG_DCOFFSET       0xE6 /* DCoffset */

#define REG_F0             0xF0 /* init:0x00 close:0x01 */
#define REG_F2             0xF2 /* init:0x00 close:0x4E */

#define REG_MODE_SLEEP     0x30 /* Sleep mode */
#define REG_MODE_CONTACT   0x31 /* Contact mode */
#define REG_MODE_SENSOR    0x33 /* Sensor mode */
#define REG_MODE_34        0x34 /* Full Frame mode (Fly-Estimation®) */

/* Contact sensor paramter */
#define CS_DETECT_TIMEOUT  5000 /* Waiting time to detect contact (ms) */
#define CS_DETECT_DELAY    5    /* Delay between each test (ms) */

/* This structure must be packed because it is a the raw message sent.
 * __attribute__((packed)) could be added if required. */
struct egis_msg {
  uint8_t magic[5]; /* out: 'EGIS' 0x09 / in: 'SIGE' 0x0A */
  uint8_t cmd;
  union {
    struct {
      uint8_t nb;
      uint8_t regs[REG_MAX];
    } egis_readreg;
    struct {
      uint8_t regs[REG_MAX];
    } sige_readreg;
    struct {
      uint8_t nb;
      struct {
        uint8_t reg;
        uint8_t val;
      } regs[REG_MAX];
    } egis_writereg;
    struct {
      uint8_t val[6];
    } egis_readbuf;
    struct {
      uint8_t val[5];
    } sige_misc;
    uint8_t padding[0x40-6]; /* Ensure size of 0x40 */
  };
};


/* Forward declarations */
static uint8_t *process_frame(uint8_t *buffer, uint8_t *newf);
static int process_frame_empty(uint8_t *f, int mode);
static int contact_detect(libusb_device_handle *dev);

/* Structure to keep information between asynchronous functions. */
struct etes603_data {
	unsigned int deactivating; /* TODO could be merge with state? */
	unsigned int state;
	unsigned int mode; /* Full frame (0) or merging frames (1) */
	uint8_t *braw; /* Pointer to raw buffer */
	uint8_t *braw_cur; /* Current position in the raw buffer */
	uint8_t *braw_end; /* End of the raw buffer */
	/* TODO can probably keep registers value, particularly control_mode */
};

/* Transform array of uint8_t to a string */
static void sprint_data(char *str, size_t str_sz, uint8_t *data, size_t data_sz)
{
	unsigned int i;
	char *cstr = str;
	/* Ensure string ends by NULL byte */
	*cstr = 0;
	for (i = 0; i < data_sz && cstr < str+str_sz; i++) {
		snprintf(cstr, 4, "%02X ", data[i]);
		cstr += 3;
	}
}

/* Synchronous function  */

static int sync_transfer(libusb_device_handle *dev, unsigned char ep,
		struct egis_msg *msg, unsigned int size) 
{
	int ret, actual_length;
	unsigned char *data = (unsigned char *)msg;

#ifdef DEBUG
	if (!dev) {
		fp_err("libusb not initialized");
		return -1;
	}
#endif

	ret = libusb_bulk_transfer(dev, ep, data, size, &actual_length,
			BULK_TIMEOUT);

	if (ret < 0) {
		fp_err("Bulk write error %d", ret);
		return -2;
	}

#ifdef DEBUG_TRANSFER
	char dbg_data[60]; /* Format is "00 11 ... \0" */
	sprint_data(dbg_data, 60, data, actual_length);
	if (ep == EP_OUT)
		fp_dbg("send: %s (%d bytes)", dbg_data, actual_length);
	else if (ep == EP_IN)
		fp_dbg("recv %s (%d bytes)", dbg_data, actual_length);
#endif

	return actual_length;
}

/* The size of the message header is 5 plus 1 for the command. */
#define MSG_HEADER_SIZE 6

/*
 * Prepare the header of the message to be sent to the device.
 */
static void msg_header_prepare(struct egis_msg *msg)
{
	msg->magic[0] = 'E';
	msg->magic[1] = 'G';
	msg->magic[2] = 'I';
	msg->magic[3] = 'S';
	msg->magic[4] = 0x09;
}

/*
 * Check that the header of the received message is correct.
 */
static int msg_header_check(struct egis_msg *msg)
{
	if (msg->magic[0] == 'S' && msg->magic[1] == 'I'
	    && msg->magic[2] == 'G' && msg->magic[3] == 'E'
	    && msg->magic[4] == 0x0A)
		return 0;
	return 1;
}



/*
 * Prepare message to ask for a frame.
 */
static void msg_read_buffer(struct egis_msg *msg, uint8_t cmd, uint8_t v1,
	uint8_t v2, uint8_t v3, uint8_t v4, uint8_t v5)
{
	msg_header_prepare(msg);
	/* cmd can be CMD_READ_BUF or CMD_READ_BUF2 */
	msg->cmd = cmd;
	msg->egis_readbuf.val[0] = 0x01;
	msg->egis_readbuf.val[1] = v1;
	msg->egis_readbuf.val[2] = v2;
	msg->egis_readbuf.val[3] = v3;
	msg->egis_readbuf.val[4] = v4;
	/* This value is useless if CMD_READ_BUF2 (request is 1 byte smaller) */
	msg->egis_readbuf.val[5] = v5;
}

/*
 * Ask synchronously the sensor for a frame.
 */
static int sync_read_buffer(libusb_device_handle *dev, uint8_t v1, uint8_t v2,
	uint8_t gain, uint8_t v4, uint8_t v5, uint8_t *buf)
{
	struct egis_msg msg;
	int ret, i;

	msg_read_buffer(&msg, CMD_READ_BUF, v1, v2, gain, v4, v5);

	ret = sync_transfer(dev, EP_OUT, &msg, MSG_HEADER_SIZE + 6);
	if (ret < 0) {
		fp_err("sync_transfer EP_OUT failed");
		goto err;
	}
	for (i = 0 ; i < FRAME_SIZE; i += ret) {
		ret = sync_transfer(dev, EP_IN, (struct egis_msg *)(buf+i), FRAME_SIZE/* - i*/); 
		if (ret < 0) {
			fp_err("sync_transfer EP_IN failed");
			goto err;
		}
	}

	return 0;
err:
	return -1;
}

/*
 * Ask synchronously the sensor for a full frame.
 */
static int sync_read_buffer_full(libusb_device_handle *dev, uint8_t *buf)
{
	/* Full frame in once */
	struct egis_msg msg;
	int ret, i;

	/* ??? meaning of these values? always same on tested device. */
	msg_read_buffer(&msg, CMD_READ_BUF2, 0xF4, 0x02, 0x01, 0x64, 0x00); 

	ret = sync_transfer(dev, EP_OUT, &msg, MSG_HEADER_SIZE + 5);
	if (ret < 0) {
		fp_err("sync_transfer EP_OUT failed");
		goto err;
	}
	for (i = 0 ; i < FRAMEFULL_SIZE; i += ret) {
		ret = sync_transfer(dev, EP_IN, (struct egis_msg *)(buf+i), FRAMEFULL_SIZE/* - i*/); 
		if (ret < 0) {
			fp_err("sync_transfer EP_IN failed");
			goto err;
		}
	}

	return 0;
err:
	return -1;
}

/*
 * Read synchronously a register from the sensor.
 */
static int sync_read_registers(libusb_device_handle *dev, int n_args, ... /* int reg, int *val */)
{
	va_list ap;
	struct egis_msg msg;
	int ret, i;

	if (n_args == 0 || n_args % 2 != 0 || n_args > REG_MAX * 2) {
		fp_err("wrong number of arguments (%d)", n_args);
		goto err;
	}

	msg_header_prepare(&msg);
	msg.cmd = CMD_READ_REG;
	msg.egis_readreg.nb = n_args / 2;

	va_start(ap, n_args);
	for (i = 0; i < n_args / 2; i++) {
		msg.egis_readreg.regs[i] = va_arg(ap, int);
		va_arg(ap, uint8_t*);
	}
	va_end(ap);

	ret = sync_transfer(dev, EP_OUT, &msg, MSG_HEADER_SIZE + 1 + n_args / 2);
	if (ret < 0) {
		fp_err("sync_transfer EP_OUT failed");
		goto err;
	}
	ret = sync_transfer(dev, EP_IN, &msg, sizeof(msg));
	if (ret < 0) {
		fp_err("sync_transfer EP_IN failed");
		goto err;
	}
	if (msg_header_check(&msg)) {
		fp_err("msg_header_check failed");
		goto err;
	}
	if (msg.cmd != CMD_OK) {
		fp_warn("CMD_OK failed");
		goto err;
	}

	va_start(ap, n_args);
	for (i = 0; i < n_args / 2; i++) {
		uint8_t *val;
		va_arg(ap, int);
		val = va_arg(ap, uint8_t*);
		*val = msg.sige_readreg.regs[i];
	}
	va_end(ap);

	return 0;
err:
	return -1;
}

/*
 * Write synchronously a register from the sensor.
 */
static int sync_write_registers(libusb_device_handle *dev, int n_args, ... /*int reg, int val*/)
{
	va_list ap;
	struct egis_msg msg;
	int ret, i;

	if (n_args == 0 || n_args % 2 != 0 || n_args > REG_MAX * 2) {
		fp_err("wrong number of arguments (%d)", n_args);
		goto err;
	}

	msg_header_prepare(&msg);
	msg.cmd = CMD_WRITE_REG;
	msg.egis_writereg.nb = n_args / 2;

	va_start(ap, n_args);
	for (i = 0; i < n_args / 2; i++) {
		msg.egis_writereg.regs[i].reg = va_arg(ap, int);
		msg.egis_writereg.regs[i].val = va_arg(ap, int);
	}
	va_end(ap);

	ret = sync_transfer(dev, EP_OUT, &msg, MSG_HEADER_SIZE + 1 + n_args);
	if (ret < 0) {
		fp_err("sync_transfer EP_OUT failed");
		goto err;
	}
	ret = sync_transfer(dev, EP_IN, &msg, sizeof(msg));
	if (ret < 0) {
		fp_err("sync_transfer EP_IN failed");
		goto err;
	}
	if (msg_header_check(&msg)) {
		fp_err("msg_header_check failed");
		goto err;
	}
	if (msg.cmd != CMD_OK) {
		fp_warn("CMD_OK failed");
		goto err;
	}
	return 0;
err:
	return -1;
}

/*
 * Check the model of the sensor.
 */
static int sensor_check(libusb_device_handle *dev)
{
	uint8_t reg_70, reg_71, reg_72, reg_73;

	if (sync_read_registers(dev, 8, REG_INFO0, &reg_70, REG_INFO1, &reg_71,
				REG_INFO2, &reg_72, REG_INFO3, &reg_73)) {
		fp_err("cannot read device registers.");
		goto err;
	}

	/* Check device */
	if (reg_70 != 0x4A || reg_71 != 0x44 || reg_72 != 0x49 || reg_73 != 0x31) {
		fp_err("unknown device parameters (REG_70:%02X REG_71:%02X "
		       "REG_FIRMWARE:%02X REG_VERSION:%02X)",
		       reg_70, reg_71, reg_72, reg_73);
		/* TODO does not make it fails the time found all compatible devices. */
		/* goto err; */
	}
	return 0;
err:
	return -1;
}

/*
 * Ask command 0x20 to the sensor.
 */
static int sensor_get_cmd20(libusb_device_handle *dev)
{
	struct egis_msg msg;
	int ret;

	msg_header_prepare(&msg);
	msg.cmd = CMD_20;

	ret = sync_transfer(dev, EP_OUT, &msg, MSG_HEADER_SIZE);
	if (ret < 0) {
		fp_err("sync_transfer EP_OUT failed");
		goto err;
	}
	ret = sync_transfer(dev, EP_IN, &msg, sizeof(msg));
	if (ret < 0) {
		fp_err("sync_transfer EP_IN failed");
		goto err;
	}
	if (msg_header_check(&msg)) {
		fp_err("msg_header_check failed");
		goto err;
	}
	/* TODO is it status or flashtype/flashinfo or ? */
	if (msg.cmd != 0x05
			|| msg.sige_misc.val[0] != 0x00
			|| msg.sige_misc.val[1] != 0x00) {
		fp_warn("unexpected answer CMD_20 from device (%02X %02X %02X)", msg.cmd,
				msg.sige_misc.val[0], msg.sige_misc.val[1]);
	}

	return 0;
err:
	return -1;
}

/*
 * Ask command 0x25 to the sensor.
 */
static int sensor_get_cmd25(libusb_device_handle *dev)
{
	struct egis_msg msg;
	int ret;

	msg_header_prepare(&msg);
	msg.cmd = CMD_25;

	ret = sync_transfer(dev, EP_OUT, &msg, MSG_HEADER_SIZE);
	if (ret < 0) {
		fp_err("sync_transfer EP_OUT failed");
		goto err;
	}
	ret = sync_transfer(dev, EP_IN, &msg, sizeof(msg));
	if (ret < 0) {
		fp_err("sync_transfer EP_IN failed");
		goto err;
	}
	if (msg_header_check(&msg)) {
		fp_err("msg_header_check failed");
		goto err;
	}
	if (msg.cmd != CMD_OK) {
		fp_err("CMD_OK failed");
		goto err;
	}
	/* TODO is it flashtype or status or ? */
	if (msg.sige_misc.val[0] != 0x00) {
		fp_warn("unexpected answer for CMD_25 (%02X)", msg.sige_misc.val[0]);
	}
	return 0;
err:
	return -1;
}

/*
 * Ask command 0x60 to the sensor.
 */
static int sensor_get_cmd60(libusb_device_handle *dev, uint8_t cmd, uint8_t val, uint8_t *reg)
{
	struct egis_msg msg;
	int ret;

	msg_header_prepare(&msg);
	msg.cmd = CMD_60;
	/* cmd 0x01: read / cmd 0x02: write */
	msg.sige_misc.val[0] = cmd;
	msg.sige_misc.val[1] = val;

	ret = sync_transfer(dev, EP_OUT, &msg, MSG_HEADER_SIZE + cmd - 1);
	if (ret < 0) {
		fp_err("sync_transfer EP_OUT failed");
		goto err;
	}
	ret = sync_transfer(dev, EP_IN, &msg, sizeof(msg));
	if (ret < 0) {
		fp_err("sync_transfer EP_IN failed");
		goto err;
	}
	if (msg_header_check(&msg)) {
		fp_err("msg_header_check failed");
		goto err;
	}
	if (msg.cmd != CMD_OK) {
		fp_err("CMD_OK failed");
		goto err;
	}
	/* Read cmd, so set read value to reg */
	if (cmd == 1 && reg != NULL) {
		*reg = msg.sige_misc.val[0];
	}
	return 0;
err:
	return -1;
}

/*
 * Change the mode of the sensor.
 */
static int sync_write_mode_control(libusb_device_handle *dev, uint8_t mode)
{
	if (sync_write_registers(dev, 2, REG_MODE_CONTROL, mode))
		return -1;
	return 0;
}

/*
 * Initialize the sensor by setting some registers.
 */
static int init_sensor(libusb_device_handle *dev)
{
	if (sync_write_mode_control(dev, REG_MODE_SLEEP))
		return -1;
	if (sync_write_registers(dev, 2, REG_50, 0x0F))
		return -2;
	if (sync_write_registers(dev, 2, REG_GAIN, 0x04))
		return -3;
	if (sync_write_registers(dev, 2, REG_VRT, 0x08))
		return -4;
	if (sync_write_registers(dev, 2, REG_VRB, 0x0D))
		return -5;
	if (sync_write_registers(dev, 2, REG_VCO_CONTROL, 0x14))
		return -6;
	if (sync_write_registers(dev, 2, REG_DCOFFSET, 0x36))
		return -7;
	if (sync_write_registers(dev, 2, REG_F0, 0x00))
		return -8;
	if (sync_write_registers(dev, 2, REG_F2, 0x00))
		return -9;
	return 0;
}

/*
 * This function sets encryption registers to no encryption.
 */
static int init_enc(libusb_device_handle *dev)
{
	/* Initialize encryption. */
	/* set register from 0x41 to 0x48 (0x8 regs) */
	if (sync_write_registers(dev, 16, 
			REG_ENC1, 0x12, REG_ENC2, 0x34, REG_ENC3, 0x56, REG_ENC4, 0x78, 
			REG_ENC5, 0x90, REG_ENC6, 0xAB,	REG_ENC7, 0xCD, REG_ENC8, 0xEF)) {
		fp_err("Failed");
		return -1;
	}
	return 0;
}

/*
 * This function set registers 0x20 to 0x37 to default values.
 */
static int init_regs(libusb_device_handle *dev)
{
	/* set register from 0x20 to 0x37 (0x18 regs) */
	if (sync_write_registers(dev, 48,
			REG_20, 0x00, REG_21, 0x23, REG_22, 0x21, REG_23, 0x20,
			REG_24, 0x14, REG_25, 0x6A, REG_26, 0x00, REG_27, 0x00,
			REG_28, 0x00, REG_29, 0xC0, REG_2A, 0x50, REG_2B, 0x50,
			REG_2C, 0x4D, REG_2D, 0x03, REG_2E, 0x06, REG_2F, 0x06,
			REG_30, 0x10, REG_31, 0x02, REG_32, 0x14, REG_33, 0x34,
			REG_34, 0x01, REG_35, 0x08, REG_36, 0x03, REG_37, 0x21)) {
		fp_err("Failed");
		return -1;
	}
	return 0;
}

/*
 * This function tunes the DCoffset value and adjusts the gain value if required.
 */
static int tune_dc(libusb_device_handle *dev)
{
	uint8_t buf[FRAME_SIZE];
	uint8_t min, max;
	uint8_t dcoffset, gain;
	//uint8_t reg_e0, reg_e1, reg_e2;

	/* The default gain should work but reduces it if reach dcoffset limit  */
	for (gain = GAIN_SMALL_INIT; ; gain--) {
		min = DCOFFSET_MIN;
		max = DCOFFSET_MAX;

		/* dichotomic search to find at which value the frame become almost black. */
		while (min + 1 < max) {
			dcoffset = (max + min) / 2;
			if (sync_write_registers(dev, 2, REG_DCOFFSET, dcoffset))
				goto err_tunedc;
			/* 0x15 0x10 are constant in all frames. */
			if (sync_read_buffer(dev, 0xC0, 0x01, gain, 0x15, 0x10, buf))
				goto err_tunedc;
			if (process_frame_empty(buf, 0))
				max = dcoffset;
			else
				min = dcoffset;
		}
		if (max < DCOFFSET_MAX) {
			dcoffset = max + 1;
			break;
		}
	}
	fp_dbg("DCoffset=0x%02X Gain=0x%02X", dcoffset, gain);
	/* Set registers */
	/* ??? how reg21 / reg22 are calculated */
	if (sync_write_registers(dev, 4, REG_21, 0x23, REG_22, 0x21))
		goto err_write;
	if (sync_write_registers(dev, 2, REG_GAIN, gain))
		goto err_write;
	if (sync_write_registers(dev, 2, REG_DCOFFSET, dcoffset))
		goto err_write;
	
	/* This read happens in captured traffic but is it used? */
	//sync_read_registers(dev, 6, REG_GAIN, &reg_e0, REG_VRT, &reg_e1, REG_VRB, &reg_e2);

	return 0;
err_tunedc:
	fp_err("Error tuning DC/gain parameter");
	return -1;
err_write:
	fp_err("Error setting DC/gain registers");
	return -2;
}

/*
 * This function tunes the value for DTVRT and adjusts DCOFFSET if needed.
 */
static int tune_dtvrt(libusb_device_handle *dev)
{
	uint8_t dcoffset;
	uint8_t reg_e5;
	uint8_t reg_50, reg_51;
	uint8_t reg_59, reg_5a, reg_5b;

	uint8_t dtvrt;

	/* Read DCoffset, it is not done in traces but need it (no global save of registers). */
	if (sync_read_registers(dev, 2, REG_DCOFFSET, &dcoffset))
		goto err_read;

	/* Save registers to reset it at the end. */
	if (sync_read_registers(dev, 2, REG_VCO_CONTROL, &reg_e5)) /* 0x13 */
		goto err_read;
	if (sync_read_registers(dev, 4, REG_50, &reg_50, REG_51, &reg_51)) /* 0x0F 0x32 */
		goto err_read;
	if (sync_read_registers(dev, 6, REG_59, &reg_59, REG_5A, &reg_5a, REG_5B, &reg_5b)) /* 0x18 0x08 0x10 */
		goto err_read;

	if (sync_write_mode_control(dev, REG_MODE_SLEEP))
		goto err_write;
	/* In traces, REG_DCOFFSET is set but not required. */
	if (sync_write_registers(dev, 2, REG_VCO_CONTROL, 0x13)) /* always 0x13 */
		goto err_write;
	if (sync_write_registers(dev, 2, REG_50, 0x8F)) /* reg_50 | 0x80  */
		goto err_write;
	if (sync_write_registers(dev, 2, REG_51, 0x31)) /* reg_51 & 0xF7 ? */
		goto err_write;
	if (sync_write_registers(dev, 6, REG_59, 0x18, REG_5A, 0x08, REG_5B, 0x00)) /* always 0x18 */
		goto err_write;

	if (sync_write_mode_control(dev, REG_MODE_CONTACT))
		goto err_write;

	/* DTVRT Tuning */
	dtvrt = DTVRT_MAX;
	if (sync_write_registers(dev, 2, REG_DTVRT, dtvrt))
		goto err_tune;
	/* Arbitrary lowest value for dcoffet */
	while (!contact_detect(dev) && dcoffset > 0x10) {
		if (dtvrt > 5) {
			dtvrt -= 5;
		} else if (dtvrt > 1) {
			dtvrt -= 1;
		} else {
			/* Decrease DCoffset if cannot adjust a value for DTVRT */
			dcoffset--;
			dtvrt = DTVRT_MAX;
			fp_dbg("Decrease DCoffset=0x%02X to tune DTVRT", dcoffset);
			if (sync_write_registers(dev, 2, REG_DCOFFSET, dcoffset))
				goto err_tune;
		}
		if (sync_write_registers(dev, 2, REG_DTVRT, dtvrt))
			goto err_tune;
	}
	fp_dbg("DTVRT=0x%02X DCoffset=0x%02X", dtvrt, dcoffset);

	if (sync_write_mode_control(dev, REG_MODE_SLEEP))
		goto err_write;

	/* Reset values of registers, saved at the beginning of this function. */
	if (sync_write_registers(dev, 2, REG_VCO_CONTROL, 0x13))
		goto err_write;
	if (sync_write_registers(dev, 4, REG_50, 0x0F, REG_51, 0x32))
		goto err_write;
	if (sync_write_registers(dev, 6, REG_59, 0x18, REG_5A, 0x08, REG_5B, 0x10))
		goto err_write;
	/* Set value found for DTVRT */
	if (sync_write_registers(dev, 2, REG_DTVRT, dtvrt))
		goto err_write;

	return 0;
err_read:
	fp_err("cannot read registers");
	return -1;
err_write:
	fp_err("cannot write registers");
	return -2;
err_tune:
	fp_err("error tuning DTVRT/DCoffset (cannot write registers)");
	return -3;
}

/*
 * Tune value of VRT and VRB for contrast and brightness.
 */
static int tune_vrb(libusb_device_handle *dev)
{
	uint8_t buf[FRAME_SIZE];
	/*reg_e1 = vrt*/
	/* VRT=0x0A and VRB=0x10 are starting values */
	/* vrt_max=0x3F  vrb_max=0x3A */
	uint8_t reg_e0, reg_e1 = 0x0A, reg_e2 = 0x10, reg_e6;
	int i, j;
	double hist[16];
	double white_mean, black_mean;

#ifdef DUMP_CALIBRATE
	FILE *f;
	f = fopen("calibrate.bin", "w");
#endif
	fp_dbg("Experimental tuning of VRT/VRB");

	if (sync_read_registers(dev, 2, REG_GAIN, &reg_e0))
		goto err;

	/* Reduce DCoffset by 1 */
	/* TODO is it required? or my DCoffset tuning is wrong? */
	if (sync_read_registers(dev, 2, REG_DCOFFSET, &reg_e6))
		goto err;
	if (sync_write_registers(dev, 2, REG_DCOFFSET, reg_e6-1))
		goto err;

	while (reg_e1 < 0x16 && reg_e2 < 0x1a) {
		fp_dbg("Testing VRT=0x%02X VRB=0x%02X values", reg_e1, reg_e2);	
		/* Clean histogram */
		for (i = 0; i < 16; i++)
			hist[i] = 0.0;

		/* Capture frame */
		if (sync_read_buffer(dev, 0xC0, 0x01, reg_e0, reg_e1, reg_e2, buf)) /* reg_e0=0x23 is in_sensor_normal_gain/in_sensor_small_gain */
			goto err;
#ifdef DUMP_CALIBRATE
		fwrite(buf, 1, 384, f);
#endif
		/* fill up histogram, 4 rows of the frame */
		for (j = 0; j < 4; j++) {
			/* only center pixels (0x50 pixels) */
			for (i = 0x08 + j * 0x60; i < 0x58 + j * 0x60; i++)
				hist[buf[i] >> 4]++;
		}
		/* histogram average */
		for (i = 0; i < 16; i++) {
			hist[i] = hist[i] / (0x50*4);
		}
		/* average black/white pixels (full black and full white excluded) */
		black_mean = white_mean = 0.0;
		for (i = 1; i < 8; i++)
			black_mean += hist[i];
		for (i = 8; i < 15; i++)
			white_mean += hist[i];
		fp_dbg("fullblack=%6f black=%6f grey=%6f white=%6f fullwhite=%6f", hist[0], black_mean, black_mean+white_mean, white_mean, hist[15]);

		/* Tuning VRT/VRB -> contrast and brightness */
		if (hist[0] + black_mean > 0.95) {
			fp_dbg("Image is too dark, reduce DCoffset?");
			break;
			//reg_e6--;
			//sync_write_registers(dev, 2, REG_DCOFFSET, reg_e6-1);
		}
		if (hist[15] > 0.95) {
			fp_dbg("Image is too bright, what to do?");
		}
		if ((black_mean > 0.1) && (white_mean > 0.1) && (black_mean + white_mean > 0.4)) {
			/* The image seems balanced. */
			break;
		}
		//if (vrt >= 2*vrb - 0x0a) { vrt++; vrb++; } else { vrt++; }
		if (reg_e1 >= 2 * reg_e2 - 0x0a) {
			reg_e1++; reg_e2++;
		} else {
			reg_e1++;
		}
		/* Check maximum for vrt/vrb */
		/* TODO if maximum is reached, leave with an error? */
		if (reg_e1 > 0x3f) /* vrt_max = 0x3f */
			reg_e1 = 0x3f;
		if (reg_e2 > 0x3a) /* vrb_max = 0x3a */
			reg_e2 = 0x3a;

	}
	fp_dbg("VRT VRB values are not corrected with tuning, set to fixed values");
	/* should found value vrt=0x16 vrb=0x19 */
	//reg_e1 = 0x16; reg_e2 = 0x19;
	
#ifdef DUMP_CALIBRATE
	fclose(f);
#endif
	/* vrt_max=0x3F  vrb_max=0x3A */
	/* Reset the DCOffset */
	if (sync_write_registers(dev, 2, REG_DCOFFSET, reg_e6))
		goto err;

	/* In traces, REG_26/REG_27 are set, purpose? values? */
	//sync_write_registers(dev, 4, REG_26, 0x11, REG_27, 0x00);
	/* Set Gain/VRT/VRB values found */
	if (sync_write_registers(dev, 6, REG_GAIN, reg_e0, REG_VRT, reg_e1, REG_VRB, reg_e2))
		goto err;
	/* In traces, it is read again, why? */
	//sync_read_registers(dev, 6, REG_GAIN, &reg_e0, REG_VRT, &reg_e1, REG_VRB, &reg_e2); /* 0x23 0x0B 0x11 */

	return 0;
err:
	fp_err("cannot read/write registers");
	return -1;
}

/*
 * This functions sets registers with tunned values.
 * TODO probably not required at all
 */
static int check_regs(libusb_device_handle *dev)
{
	/* In traces, registers are set once again. */
//	sync_write_registers(dev, 2, REG_GAIN, 0x23); /* from tune_vrb */
//	sync_write_registers(dev, 2, REG_VRT, 0x0B); /* from tune_vrb */
//	sync_write_registers(dev, 2, REG_VRB, 0x11); /* from tune_vrb */
//	sync_write_registers(dev, 2, REG_DTVRT, 0x08); /* from tune_dtvrt */
//	sync_write_registers(dev, 2, REG_DCOFFSET, 0x31); /* from tune_dc but could be modified in tune_dtvrt */
//	sync_write_registers(dev, 2, REG_21, 0x23); /* from tune_dc */
//	sync_write_registers(dev, 2, REG_26, 0x11); /* from tune_vrb */
	return 0;
}

static int sensor_realtime(libusb_device_handle *dev)
{
	/* TODO Set parameter found for realtime */
	if (sync_write_mode_control(dev, REG_MODE_SLEEP))
		return -1;
//	if (sync_write_registers(dev, 2, REG_DCOFFSET, 0x32)) /* DCOffset */
//		return -2;
//	if (sync_write_registers(dev, 6, REG_GAIN, 0x23, REG_VRT, 0x0C, REG_VRB, 0x12)) /* Gain 23, VRT 0c, VRB 12 */
//		return -3;
	/* Set to Realtime mode (0x14). */
	if (sync_write_registers(dev, 2, REG_VCO_CONTROL, 0x14))
		return -4;
	return 0;
}

static int frame_configure(libusb_device_handle *dev)
{
	/* REG_04 frame configuration */
	return sync_write_registers(dev, 2, REG_04, 0x00);
}

static int frame_capture(libusb_device_handle *dev, uint8_t *buf)
{
	return sync_read_buffer(dev, 0xC0, 0x00, 0x00, 0x00, 0x00, buf);
}

static int frame_prepare_capture(libusb_device_handle *dev)
{
	if (sensor_realtime(dev))
		return -2;
	if (frame_configure(dev))
		return -3;
	if (sync_write_mode_control(dev, REG_MODE_SENSOR))
		return -4;
	return 0;
}

static int frame_full_configure(libusb_device_handle *dev)
{
	/* REG_10 is required to get a good full frame (exact meaning?) */
	return sync_write_registers(dev, 2, REG_10, 0x92);
}


/*
 * This function opens the sensor and initialize it.
 */
static int sensor_open(libusb_device_handle *dev)
{
	int ret;

	if ((ret = sensor_check(dev)) != 0) {
		fp_err("sensor_check failed (err=%d)", ret);
		goto err;
	}
	if ((ret = sensor_get_cmd20(dev)) != 0) {
		fp_err("sensor_get_cmd20 failed (err=%d)", ret);
		goto err;
	}
	if ((ret = sensor_get_cmd25(dev)) != 0) {
		fp_err("sensor_get_cmd25 failed (err=%d)", ret);
		goto err;
	}
	if ((ret = init_sensor(dev)) != 0) {
		fp_err("init_sensor failed (err=%d)", ret);
		goto err;
	}
	if ((ret = init_enc(dev)) != 0) {
		fp_err("init_enc failed (err=%d)", ret);
		goto err;
	}
	if ((ret = init_regs(dev)) != 0) {
		fp_err("init_regs failed (err=%d)", ret);
		goto err;
	}
	if ((ret = tune_dc(dev)) != 0) {
		fp_err("tune_dc failed (err=%d)", ret);
		goto err;
	}
	if ((ret = tune_dtvrt(dev)) != 0) {
		fp_err("tune_dtvrt failed (err=%d)", ret);
		goto err;
	}
	if ((ret = tune_vrb(dev)) != 0) {
		fp_err("tune_vrb failed (err=%d)", ret);
		goto err;
	}
	if ((ret = check_regs(dev)) != 0) {
		fp_err("check_regs failed (err=%d)", ret);
		goto err;
	}
	/* Configure full frame (set register value for this session) */
	if ((ret = frame_full_configure(dev)) != 0) {
		fp_err("frame_full_configure failed (err=%d)", ret);
		goto err;
	}

	return 0;
err:
	return -1;
}


/*
 * Detect the contact by reading register 0x03.
 */
static int contact_detect(libusb_device_handle *dev)
{
	uint8_t reg_03;
	if (sync_read_registers(dev, 2, REG_03, &reg_03))
		return -1;
	/* 83,A3:no 93,B3:yes */
	return (reg_03 >> 4) & 0x1;
}

/*
 * Initialize the finger contact sensor.
 */
static int contact_polling_init(libusb_device_handle *dev)
{
	uint8_t reg_50;
	/* TODO fixed value for written regs?  */
	if (sync_write_mode_control(dev, REG_MODE_SLEEP)
	    || sync_write_registers(dev, 2, REG_VCO_CONTROL, 0x13)
	    || sync_write_registers(dev, 2, REG_59, 0x18)
	    || sync_write_registers(dev, 2, REG_5A, 0x08)
	    || sync_write_registers(dev, 2, REG_5B, 0x10))
		return -1;

	if (sync_write_mode_control(dev, REG_MODE_CONTACT)
	    || sync_read_registers(dev, 2, REG_50, &reg_50) /* 0F */
	    || sync_write_registers(dev, 2, REG_50, 0x8F))  /* ((reg_50 & 7F) | 0x80) */
		return -2;
	return 0;
}

/*
 * Change the sensor mode after contact detection.
 */
static int contact_polling_exit(libusb_device_handle *dev)
{
	/* Set VCO_CONTROL to Realtime mode (should it reset value before polling?) */
	if (sync_write_mode_control(dev, REG_MODE_SLEEP)
	    || sync_write_registers(dev, 2, REG_VCO_CONTROL, 0x14))
		return -1;
	return 0;
}

/*
 * Detect finger contact with the sensor.
 * Returns 1 if contact detected or 0 if timeout.
 */
static int contact_polling(libusb_device_handle *dev)
{
	/* Contact polling power consumption is lower than capturing a lot of
	 * frame. From website: Typical 15 mA @ USB2.0 imaging/navigating and
	 * Typical <500uA finger detect mode. */
	struct timeval endtime, curtime = {.tv_sec = CS_DETECT_TIMEOUT / 1000, .tv_usec = 0};
	int contact;

	if (contact_polling_init(dev)) {
		fp_err("cannot initialize polling.");
		contact = -1;
		goto end;
	}

	if (gettimeofday(&endtime, NULL)) {
		fp_err("gettimeofday failed with %d", errno);
		contact = -2;
		goto end;
	}
	timeradd(&endtime, &curtime, &endtime);

	do {
		contact = contact_detect(dev);
		if (contact == 1) {
			goto end;
		}

		/* 5 ms between each detect */
		usleep(CS_DETECT_DELAY * 1000);

		if (gettimeofday(&curtime, NULL)) {
			fp_err("gettimeofday failed with %d", errno);
			contact = -2;
			goto end;
		}
	} while (timercmp(&curtime, &endtime, <));

end:
	contact_polling_exit(dev);
		
	return contact;
}

/*
 * Ask the sensor for a full frame.
 */
static int image_capture_full(libusb_device_handle *dev, uint8_t *buf)
{
	int i;
	uint8_t *bend, *bcur;
#if 0
	uint8_t *fstart, *fend;
#endif

	if (frame_full_configure(dev))
		return -1;

	if (sync_write_mode_control(dev, REG_MODE_34))
		return -2;

	if (sync_read_buffer_full(dev, buf))
		return -3;

	bend = buf + 64000;
	bcur = buf;
	while (bcur < bend) {
		unsigned int acc = 0;
		/* Sum bytes of 2 lines */
		for (i = 0; i < 256; i++)
			acc = bcur[i];
		/* If threshold reach, we assume fingerprint starts */
		if (acc > 255 * 8)
			break;
		bcur += 256;
	}
#if 0
	fstart = bcur;
	while (bcur < bend) {
		unsigned int acc = 0;
		/* Sum bytes of 2 lines */
		for (i = 0; i < 256; i++)
			acc = bcur[i];
		/* If threshold reach, we assume fingerprint ends */
		if (acc < 255 * 8)
			break;
		bcur += 256;
	}
	fend = bcur;

	/* If the end of buffer has still fingerprint, acquire new frame */
	if (fend == bend) {
		if (sync_read_buffer_full(dev, bcur))
			return -2;
		bend += 64000;
		while (bcur < bend) {
			unsigned int acc = 0;
			/* Sum bytes of 2 lines */
			for (i = 0; i < 256; i++)
				acc = bcur[i];
			/* If threshold reach, we assume fingerprint ends */
			if (acc < 255 * 8)
				break;
			bcur += 256;
		}
		fend = bcur;
	}
#endif	

	/* Set the sensor to sleep mode */
	if (sync_write_mode_control(dev, REG_MODE_SLEEP))
		return -1;

	/* Set VCO_CONTROL to realtime mode (maybe not needed) */
	if (sync_write_registers(dev, 2, REG_VCO_CONTROL, 0x14))
		return -1;

	return 0;
}

/*
 * Do a full finger capture using frames and reconstruction.
 * This function uses synchronous tranfers.
 */
static int image_capture(libusb_device_handle *dev, uint8_t *braw, int size)
{
	uint8_t bframe[FRAME_SIZE];
	uint8_t *braw_cur = braw;
	uint8_t *braw_end = braw + size;

	/* TODO could add check if cancel is asked */
	if (frame_prepare_capture(dev))
		goto err;
	if (frame_capture(dev, bframe))
		goto err;
	memcpy(braw, bframe, FRAME_SIZE);

	/* Wait a non-empty frame. */
	do {
		if (frame_capture(dev, bframe))
			goto err;
	} while (process_frame_empty(bframe, 1));

	/* While is not aborted and the frame is not empty and that the buffer is not full, retrieve and process frame. */
	do {
		braw_cur = process_frame(braw_cur, bframe);
		if (frame_capture(dev, bframe))
			goto err;
	} while (!process_frame_empty(bframe, 1) && (braw_cur + FRAME_SIZE) < braw_end);

	/* Set the sensor in sleep mode (needed?) */
	if (sync_write_mode_control(dev, REG_MODE_SLEEP))
		return -1;

	return 0;
err:
	fp_err("Error communicating with the device");
	return -1;
}


/*
 *
 */
static int sensor_move_detect(libusb_device_handle *dev)
{
	int ret;
	/* TODO different ways to detect move: contact sensor, looking into frames, or... */
	ret = contact_polling(dev);
	return ret;
}

/*
 * Close the sensor by setting some registers.
 */
static int sensor_close(libusb_device_handle *dev)
{
	/* TODO fixed values? */
	sync_write_registers(dev, 24, REG_DCOFFSET, 0x31, REG_GAIN, 0x23,
		REG_DTVRT, 0x0D, REG_51, 0x30, REG_VCO_CONTROL, 0x13, REG_F0, 0x01,
		REG_F2, 0x4E, REG_50, 0x8F, REG_59, 0x18, REG_5A, 0x08, REG_5B, 0x10,
		REG_MODE_CONTROL, REG_MODE_CONTACT);

	return 0;
}


/* Processing functions */

/* Return the brightness of a frame */
static unsigned int process_get_brightness(uint8_t *f)
{
	int i;
	unsigned int sum = 0;
        for (i = 0; i < FRAME_SIZE; i++) {
                sum += f[i] & 0x0F;
                sum += f[i] >> 4;
        }
	return sum;
}



/*
 * Return true if the frame is almost empty.
 * If mode is 0, it is high sensibility for device tuning.
 * Otherwise, for capture mode.
 */
static int process_frame_empty(uint8_t *f, int mode)
{
	/* int threshold could be replaced by mode == MODE_TUNING || MODE_CAPTURE */
	unsigned int sum = process_get_brightness(f);
	/* FIXME: when tuning DC needs more sensibility than capturing */
	/* Allow an average of 'threshold' luminosity per pixel */
	if (mode) {
		/* mode capture */
		if (sum < FRAME_SIZE * 2)
			return 1;
	} else {
		/* mode tuning */
		if (sum < FRAME_SIZE)
			return 1;
	}
	return 0;
}


/* Return the number of new lines in 'src' */
static int process_find_dup(uint8_t *dst, uint8_t *src)
{
	int i,j,v;

	unsigned int sum_error;
	unsigned int nb = 4;
	unsigned int min_error = process_get_brightness(src) / 6; /* This value is found by tests */
	/* 384 bytes / 196 px width / 4 bits value */
	/* Scan 4 lines, assuming first that all 4 lines match. */
	//printf("sum_error=");
	for (i = 0; i < 4; i++) {
		sum_error = 0;
		for (j = 0; j < 96 * (4 - i); j++) {
			/* subtract */
			v = (int)(dst[j] & 0x0F) - (int)(src[j] & 0x0F);
			/* note: can use ^2 */
			v = v >= 0 ? v : -v;
			sum_error += v;
			v = (int)(dst[j] >> 4) - (int)(src[j] >> 4);
			v = v >= 0 ? v : -v;
			sum_error += v;
		}

		sum_error *= 127; /* increase value before divide (use int and value will be small) */
		//printf("%8d (%8d) ", sum_error / j, sum_error );
		sum_error = sum_error / j; /* Avg error/pixel */
		if (sum_error < min_error) {
			min_error = sum_error;
			nb = i;
			//break;
		}
		dst += 96; /* Next line */
	}
	//printf("-> newline=%d brightness=%d\n", nb, process_get_brightness(src));
	return nb;
}

/*
 * first 'merge' bytes from src and dst are merged then raw copy.
 */
static void merge_and_append(uint8_t *dst, uint8_t *src, size_t merge)
{
	size_t i;
	uint8_t pxl, pxh;
	/* Sanity checks */
	if (merge > FRAME_SIZE)
		merge = FRAME_SIZE;
	/* Merge */
	for (i = 0; i < merge; i++) {
		pxl = ((dst[i] & 0x0F) + (src[i] & 0x0F)) >> 1;
		pxh = ((dst[i] >> 4) + (src[i] >> 4)) >> 1;
		dst[i] = pxl | (pxh << 4);
	}
	/* Raw copy */
	for (i = merge; i < FRAME_SIZE; i++) {
		dst[i] = src[i];
	}
}



/* 'dst' must point to the last buffer and have 384 bytes free at the end to append data.
 * 'src' must point to the frame received (384 bytes). */
static uint8_t *process_frame(uint8_t *dst, uint8_t *src)
{
	int new_line;

	/* TODO sweep direction to determine... merging will be diff. */

	new_line = process_find_dup(dst, src);
	dst += 96 * new_line;
	/* merge_and_append give a better result than just copying */
	merge_and_append(dst, src, (4-new_line)*96);
	/* memcpy(dst, src, FRAME_SIZE); */

	return dst;
}

/* Transform 4 bits image to 32bits RGB image */
static void process_transform4_to_32(uint8_t *input, unsigned int input_size, uint32_t *output)
{
	unsigned int i, j = 0;
	for (i = 0; i < input_size; i++, j += 2) {
		/* 16 gray levels transform to 256 levels using << 4 */
		uint8_t dl = input[i] << 4;
		uint8_t dh = input[i] & 0xF0;
		/* RGB all same value */
		output[j]   = dh | (dh << 8) | (dh << 16);
		output[j+1] = dl | (dl << 8) | (dl << 16);
	}
}

/* Transform 4bits image to 8bits image */
static void process_transform4_to_8(uint8_t *input, unsigned int input_size, uint8_t *output)
{
	unsigned int i, j = 0;
	for (i = 0; i < input_size; i++, j += 2) {
		/* 16 gray levels transform to 256 levels using << 4 */
		output[j] = input[i] & 0xF0;
		output[j+1] = input[i] << 4;
	}
}


/* libfprint stuff */

/* Called when the deactivation was requested. */
static void complete_deactivation(struct fp_img_dev *dev)
{
	struct etes603_data *pdata = dev->priv;

	/* TODO I guess I should not completely deinit the sensor? */
	/* TODO Change at least to SLEEP_MODE ? */

	pdata->deactivating = FALSE;
	fpi_imgdev_deactivate_complete(dev);
}

/* Transforms the raw data to fprint data and submit it to fprint. */
static int transform_to_fpi(struct fp_img_dev *dev)
{
	struct fp_img *img;
	struct etes603_data *pdata = dev->priv;

	if (pdata->mode == 1) {
		/* Assembled frames */
		/* braw_cur points to the last frame so needs to adjust to end */
		unsigned int size = pdata->braw_cur + FRAME_SIZE - pdata->braw;
		/* es603 has 2 pixels per byte. */
		img = fpi_img_new(size * 2);
		img->flags = FP_IMG_COLORS_INVERTED | FP_IMG_V_FLIPPED;
		img->height = size * 2 / FRAME_WIDTH;
		/* img->width could be set 256 always but need a new function to handle this */
		img->width = FRAME_WIDTH;
		process_transform4_to_8(pdata->braw, size, (uint8_t*)img->data);
	} else  {
		/* Full Frame */
		img = fpi_img_new(FRAMEFULL_SIZE * 2);
		/* Images received are white on black, so invert it (FP_IMG_COLORS_INVERTED) */
		/* TODO for different sweep direction ? FP_IMG_V_FLIPPED | FP_IMG_H_FLIPPED */
		img->flags = FP_IMG_COLORS_INVERTED | FP_IMG_V_FLIPPED;
		/* img->width can only be changed when -1 was set at init */
		img->width = FRAMEFULL_WIDTH;
		img->height = FRAMEFULL_HEIGHT;
		process_transform4_to_8(pdata->braw, FRAMEFULL_SIZE, (uint8_t*)img->data);
	}
	/* Send image to fpi */
	fpi_imgdev_image_captured(dev, img);
	/* Indicate that the finger is removed. */
	fpi_imgdev_report_finger_status(dev, FALSE);

	return 0;
}

/* state for asynchronous functions */
#define STATE_INIT                     1
#define STATE_FINGER_REQ_SEND          2
#define STATE_FINGER_REQ_RECV          3
#define STATE_FINGER_ANS               4
#define STATE_CAPTURING_REQ_SEND       5
#define STATE_CAPTURING_REQ_RECV       6
#define STATE_CAPTURING_ANS            7
#define STATE_INIT_FULL_REQ_SEND       8
#define STATE_INIT_FULL_REQ_RECV       9
#define STATE_INIT_FULL_ANS            10
#define STATE_CAPTURING_FULL_REQ_SEND  11
#define STATE_CAPTURING_FULL_REQ_RECV  12
#define STATE_CAPTURING_FULL_ANS       13
#define STATE_DEACTIVATING             14

static int async_read_buffer(struct fp_img_dev *dev, unsigned char ep,
		unsigned char *msg_data, unsigned int msg_size);

/* Asynchronous function callback for asynchronous read buffer. */
static void async_read_buffer_cb(struct libusb_transfer *transfer)
{
	struct fp_img_dev *dev = transfer->user_data;
	struct etes603_data *pdata = dev->priv;
	struct egis_msg *msg;
	char dbg_data[60]; /* Format is "00 11 ... \0" */

	/* Check status except if initial state (entrypoint) */
	if (pdata->state != STATE_INIT
	    && transfer->status != LIBUSB_TRANSFER_COMPLETED) {
		fp_warn("transfer is not completed (state=%d/status=%d)", pdata->state, transfer->status);
		goto err;
	}

	/* To ensure non-fragmented answer, LIBUSB_TRANSFER_SHORT_NOT_OK is used. */
	/* now do the real stuff */

goback:

	if (pdata->deactivating) {
		/* TODO libusb_cancel_transfer should not be required here since the
		 * transfer is completed. */
		complete_deactivation(dev);
		goto err;
	}

	switch (pdata->state) {
		case STATE_INIT:
			pdata->state = STATE_FINGER_REQ_SEND;
			/* no break, we are continuing with next step */

		case STATE_FINGER_REQ_SEND:
			fp_dbg("STATE_FINGER_REQ_SEND:");
			msg = malloc(sizeof(struct egis_msg));
			msg_read_buffer((struct egis_msg *)msg, CMD_READ_BUF, 0xC0, 0, 0, 0, 0);
			if (async_read_buffer(dev, EP_OUT, (unsigned char *)msg, MSG_HEADER_SIZE + 6)) {
				goto err;
			}
			pdata->state = STATE_FINGER_REQ_RECV;
			break;

		case STATE_FINGER_REQ_RECV:
			fp_dbg("STATE_FINGER_REQ_RECV: %d", transfer->actual_length);
			/* The request succeeds. */
			msg = malloc(FRAME_SIZE);
			memset(msg, 0, FRAME_SIZE);
			/* Now ask for receiving data. */
			if (async_read_buffer(dev, EP_IN, (unsigned char *)msg, FRAME_SIZE)) {
				goto err;
			}
			pdata->state = STATE_FINGER_ANS;
			break;

		case STATE_FINGER_ANS:
			sprint_data(dbg_data, 60, transfer->buffer, transfer->actual_length);
			fp_dbg("STATE_FINGER_ANS: %s (size %d)", dbg_data, transfer->actual_length);
			if (process_frame_empty(transfer->buffer, 1)) {
				/* No finger, request a new frame. */
				pdata->state = STATE_FINGER_REQ_SEND;
				goto goback;
			}
			/* Indicate that the finger is present. */
			fpi_imgdev_report_finger_status(dev, TRUE);
			/* Select mode for capturing. */
			if (pdata->mode == 1) {
				pdata->state = STATE_CAPTURING_REQ_SEND;
			} else {
				pdata->state = STATE_INIT_FULL_REQ_SEND;
				goto MODE_FULL_FRAME;
			}
			/* no break, continue to state STATE_CAPTURING_REQ_SEND. */

		case STATE_CAPTURING_REQ_SEND:
			fp_dbg("STATE_CAPTURING_REQ_SEND:");
			msg = malloc(sizeof(struct egis_msg));
			msg_read_buffer(msg, CMD_READ_BUF, 0xC0, 0, 0, 0, 0);
			if (async_read_buffer(dev, EP_OUT, (unsigned char *)msg, MSG_HEADER_SIZE + 6)) {
				goto err;
			}
			pdata->state = STATE_CAPTURING_REQ_RECV;
			break;
		
		case STATE_CAPTURING_REQ_RECV:
			fp_dbg("STATE_CAPTURING_REQ_RECV:");
			/* The request succeeds. */
			msg = malloc(FRAME_SIZE);
			memset(msg, 0, FRAME_SIZE);
			/* Receiving data. */
			if (async_read_buffer(dev, EP_IN, (unsigned char *)msg, FRAME_SIZE)) {
				goto err;
			}
			pdata->state = STATE_CAPTURING_ANS;
			break;

		case STATE_CAPTURING_ANS:
			sprint_data(dbg_data, 60, transfer->buffer, transfer->actual_length);
			fp_dbg("STATE_CAPTURING_ANS: %s (size %d)", dbg_data, transfer->actual_length);
			if (process_frame_empty(transfer->buffer, 1)) {
				/* Finger leaves, send final image. */
				transform_to_fpi(dev);
				pdata->state = STATE_DEACTIVATING;
				break;
			}
			/* Merge new frame with current image. */
			pdata->braw_cur = process_frame(pdata->braw_cur, transfer->buffer);	
			if ((pdata->braw_cur + FRAME_SIZE) >= pdata->braw_end) {
				fp_warn("STATE_CAPTURING_ANS: Buffer is full %p %p (%p)",pdata->braw_cur,pdata->braw_end,(pdata->braw_cur + FRAME_SIZE) );
				/* Buffer is full, send final image. */
				transform_to_fpi(dev);
				pdata->state = STATE_DEACTIVATING;
				break;
			}
			/* Ask new frame. */
			pdata->state = STATE_CAPTURING_REQ_SEND;
			goto goback;	

		case STATE_INIT_FULL_REQ_SEND:
MODE_FULL_FRAME:
			/* Change to mode REG_MODE_34 */
			fp_dbg("STATE_INIT_FULL_REQ_SEND:");
			msg = malloc(sizeof(struct egis_msg));
			msg_header_prepare(msg);
			msg->cmd = CMD_WRITE_REG;
			msg->egis_writereg.nb = 0x01;
			msg->egis_writereg.regs[0].reg = REG_MODE_CONTROL;
			msg->egis_writereg.regs[0].val = REG_MODE_34;
			if (async_read_buffer(dev, EP_OUT, (unsigned char *)msg, MSG_HEADER_SIZE + 3)) {
				goto err;
			}
			pdata->state = STATE_INIT_FULL_REQ_RECV;
			break;

		case STATE_INIT_FULL_REQ_RECV:
			fp_dbg("STATE_INIT_FULL_REQ_RECV:");
			/* The request succeeds. */
			msg = malloc(sizeof(struct egis_msg));
			memset(msg, 0, sizeof(struct egis_msg));
			/* Receiving data. */
			if (async_read_buffer(dev, EP_IN, (unsigned char *)msg, MSG_HEADER_SIZE)) {
				goto err;
			}
			pdata->state = STATE_INIT_FULL_ANS;
			break;

		case STATE_INIT_FULL_ANS:
			sprint_data(dbg_data, 60, transfer->buffer, transfer->actual_length);
			fp_dbg("STATE_INIT_FULL_ANS: %s (size %d)", dbg_data, transfer->actual_length);
			msg = (struct egis_msg *)transfer->buffer;
			if (msg_header_check(msg))
				goto err;
			if (msg->cmd != CMD_OK)
				goto err;
			pdata->state = STATE_CAPTURING_FULL_REQ_SEND;
			/* continuing, now ask for the full frame */

		case STATE_CAPTURING_FULL_REQ_SEND:
			fp_dbg("STATE_CAPTURING_FULL_REQ_SEND:");
			msg = malloc(sizeof(struct egis_msg));
			msg_read_buffer(msg, CMD_READ_BUF2, 0xF4, 0x02, 0x01, 0x64, 0x00); 
			if (async_read_buffer(dev, EP_OUT, (unsigned char *)msg, MSG_HEADER_SIZE + 5)) {
				goto err;
			}
			pdata->state = STATE_CAPTURING_FULL_REQ_RECV;
			break;
		
		case STATE_CAPTURING_FULL_REQ_RECV:
			fp_dbg("STATE_CAPTURING_FULL_REQ_RECV:");
			/* The request succeeds. */
			msg = malloc(FRAMEFULL_SIZE);
			memset(msg, 0, FRAMEFULL_SIZE);
			/* Receiving data. */
			if (async_read_buffer(dev, EP_IN, (unsigned char *)msg, FRAMEFULL_SIZE)) {
				goto err;
			}
			pdata->state = STATE_CAPTURING_FULL_ANS;
			break;

		case STATE_CAPTURING_FULL_ANS:
			sprint_data(dbg_data, 60, transfer->buffer, transfer->actual_length);
			fp_dbg("STATE_CAPTURING_FULL_ANS: %s (size %d)", dbg_data, transfer->actual_length);
			memcpy(pdata->braw, transfer->buffer, transfer->actual_length);
			/* Set STATE_DEACTIVATING before sending image because
			 * deactivation is called when image is sent. */
			pdata->state = STATE_DEACTIVATING;
			transform_to_fpi(dev);
			break;

		case STATE_DEACTIVATING:
			fp_dbg("STATE_DEACTIVATING:");
			/* TODO change sensor to sleep mode but not complete deactivation */
			break;

		default:
			fp_err("Umknown state");
			goto err;
	}

	/* No need to free buffer or transfer, LIBUSB_TRANSFER_FREE_BUFFER and
	 * LIBUSB_TRANSFER_FREE_TRANSFER are used. */

	return;
err:
	pdata->state = STATE_DEACTIVATING;
	fp_err("Error occured in async process");
	fpi_imgdev_session_error(dev, -EIO);
	return;
}

/* Asynchronous read buffer transfer. */
static int async_read_buffer(struct fp_img_dev *dev, unsigned char ep,
		unsigned char *msg_data, unsigned int msg_size)
{
	int ret;
	char dbg_data[60]; /* Format is "00 11 ... \0" */
	struct libusb_transfer *transfer = libusb_alloc_transfer(0);

	if (!transfer)
		return -ENOMEM;

	if (ep == EP_OUT) {
		/* Display data on debug mode for output */
		sprint_data(dbg_data, 60, msg_data, msg_size);
		fp_dbg("-> %s", dbg_data);
	}

	libusb_fill_bulk_transfer(transfer, dev->udev, ep, msg_data, msg_size,
			async_read_buffer_cb, dev, BULK_TIMEOUT);
	transfer->flags = LIBUSB_TRANSFER_SHORT_NOT_OK
			| LIBUSB_TRANSFER_FREE_BUFFER
			| LIBUSB_TRANSFER_FREE_TRANSFER;

	ret = libusb_submit_transfer(transfer);
	if (ret) {
		libusb_free_transfer(transfer);
		return -1;
	}
	return 0;
}

static int dev_activate(struct fp_img_dev *dev, enum fp_imgdev_state state)
{
	char *mode;
	struct libusb_transfer fake_transfer;
	struct etes603_data *pdata = dev->priv;

	/* TODO See how to manage state */
	if (state != IMGDEV_STATE_INACTIVE) {
		fp_err("The driver is in state %d", state);
		/*return -1;*/
	}

	if (pdata == NULL || pdata->braw == NULL || pdata->braw_end < pdata->braw) {
		fp_warn("dev->priv is not initialized properly.");
		return -1;
	}

	/* Reset info and data */
	pdata->deactivating = FALSE;
	pdata->braw_cur = pdata->braw;
	memset(pdata->braw, 0, (pdata->braw_end - pdata->braw));
	/* Use default mode (Full Frame) or use environment defined mode */
	pdata->mode = 0;
	if ((mode = getenv("ETES603_MODE")) != NULL) {
		if (mode[0] == '1') 
			pdata->mode = 1;
	}

	/* Preparing capture */
	frame_prepare_capture(dev->udev);

	/* Enable an entrypoint in the asynchronous mess. */	
	pdata->state = STATE_INIT;
	fake_transfer.user_data = dev;
	async_read_buffer_cb(&fake_transfer);

	fpi_imgdev_activate_complete(dev, 0);
	return 0;
}

static void dev_deactivate(struct fp_img_dev *dev)
{
	struct etes603_data *pdata = dev->priv;
	/* complete_deactivation can be called asynchronously. */
	if (pdata->state != STATE_DEACTIVATING)
		pdata->deactivating = TRUE;
	else
		complete_deactivation(dev);
}

static int dev_init(struct fp_img_dev *dev, unsigned long driver_data)
{
	int ret;
	struct etes603_data *pdata;
	/* ??? see driver_data used for. */

	ret = libusb_claim_interface(dev->udev, 0);
	if (ret != LIBUSB_SUCCESS) {
		fp_err("libusb_claim_interface failed on interface 0 (err=%d)", ret);
		return ret;
	}

	/* FIXME this should not be useful? do it if an error happens */
	ret = libusb_reset_device(dev->udev);
        if (ret != LIBUSB_SUCCESS) {
                fp_err("libusb_reset_device failed (err=%d)", ret);
                return ret;
        }

	if ((pdata = malloc(sizeof(struct etes603_data))) == NULL) {
		return -ENOMEM;
	}
	/* Allocate a buffer of 1000 frames */
	if ((pdata->braw = malloc(FRAME_SIZE * 1000)) == NULL) {
		return -ENOMEM;
	}
	pdata->braw_end = pdata->braw + (FRAME_SIZE * 1000);
	pdata->braw_cur = pdata->braw;

	dev->priv = pdata;

	/* Note: it does make sense to use asynchronous method for initializing
	 * the device. It also simplifies a lot the design of the driver. */
	/* Initialize the sensor */
	ret = sensor_open(dev->udev);
	if (ret) {
		fp_err("cannot open sensor (err=%d)", ret);
		/* The init process may be abort in the middle, force closing it. */
		sensor_close(dev->udev);
		return ret;
	}

	fpi_imgdev_open_complete(dev, 0);
	return 0;
}

static void dev_deinit(struct fp_img_dev *dev)
{
	struct etes603_data *pdata = dev->priv;

	sensor_close(dev->udev);

	/* Free private data. */
	free(pdata->braw);
	free(pdata);
	dev->priv = NULL;

	libusb_release_interface(dev->udev, 0);
	fpi_imgdev_close_complete(dev);
}

static const struct usb_id id_table[] = {
	{ .vendor = 0x1c7a, .product = 0x0603 }, /* EgisTec (aka Lightuning) ES603 */
	{ 0, 0, 0, },
};

struct fp_img_driver etes603_driver = {
	.driver = {
		.id = 11,
		.name = FP_COMPONENT,
		.full_name = "EgisTec ES603",
		.id_table = id_table,
		.scan_type = FP_SCAN_TYPE_SWIPE,
	},
	.flags = 0,
	.img_height = -1,
	.img_width = -1, /* Once set it cannot be changed (except with -1) */

	.open = dev_init,
	.close = dev_deinit,
	.activate = dev_activate,
	.deactivate = dev_deactivate,
};

