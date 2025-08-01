/*
 * Driver for the CH341 USB-I2C adapter
 *
 * Copyright (c) 2025 Hans de Goede <hansg@kernel.org>
 * Copyright (c) 2016 Tse Lun Bien
 *
 * Derived from:
 *  i2c-ch341.c
 *  Copyright (c) 2014 Marco Gittler
 *
 *  i2c-tiny-usb.c
 *  Copyright (C) 2006-2007 Till Harbaum (Till@Harbaum.org)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, version 2.
 */

#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/usb.h>

#define CH341_I2C_LOW_SPEED 0      // low speed - 20kHz
#define CH341_I2C_STANDARD_SPEED 1 // standard speed - 100kHz
#define CH341_I2C_FAST_SPEED 2     // fast speed - 400kHz
#define CH341_I2C_HIGH_SPEED 3     // high speed - 750kHz

#define CH341_CMD_I2C_STREAM 0xAA

#define CH341_CMD_I2C_STM_STA 0x74
#define CH341_CMD_I2C_STM_STO 0x75
#define CH341_CMD_I2C_STM_OUT 0x80
#define CH341_CMD_I2C_STM_IN 0xC0
#define CH341_CMD_I2C_STM_SET 0x60
#define CH341_CMD_I2C_STM_END 0x00

#define CH341_MAX_BULK_PACKET_SIZE 32

/* Structure to hold all of our device specific stuff */
struct i2c_ch341_usb {
	struct usb_device *usb_dev;  /* the usb device for this device */
	struct usb_interface *iface; /* the interface for this device */
	struct i2c_adapter adapter;  /* i2c related things */

	int ep_in;
	int ep_out;

	u8 in_buf[32];
	u8 out_buf[32];
};

static int ch341_xfer(struct i2c_ch341_usb *dev, int out_len, int in_len);

static int ch341_i2c_xfer_msg(struct i2c_adapter *adapter, struct i2c_msg *msg, bool stop)
{
	struct i2c_ch341_usb *dev = (struct i2c_ch341_usb *)adapter->algo_data;
	int ret, act, n;

	n = 0;
	dev->out_buf[n++] = CH341_CMD_I2C_STREAM;
	dev->out_buf[n++] = CH341_CMD_I2C_STM_STA;
	/* OUT with len 0 means send 1 byte and receive 1 status byte with ack info */
	dev->out_buf[n++] = CH341_CMD_I2C_STM_OUT;
	dev->out_buf[n++] = (msg->addr << 1) | (msg->flags & I2C_M_RD);
	if (msg->len) {
		if (msg->flags & I2C_M_RD) {
			/*
			 * The last command must be an STM_IN with 0 len to read
			 * the last byte. Without this the adapter gets confused.
			 */
			if (msg->len > 1)
				dev->out_buf[n++] = CH341_CMD_I2C_STM_IN | (msg->len - 1);
			dev->out_buf[n++] = CH341_CMD_I2C_STM_IN;
		} else {
			dev->out_buf[n++] = CH341_CMD_I2C_STM_OUT | msg->len;
			memcpy(&dev->out_buf[n], msg->buf, msg->len);
			n += msg->len;
		}
	}

	if (stop)
		dev->out_buf[n++] = CH341_CMD_I2C_STM_STO;
	dev->out_buf[n++] = CH341_CMD_I2C_STM_END;

	act = 0;
	ret = usb_bulk_msg(dev->usb_dev, usb_sndbulkpipe(dev->usb_dev, dev->ep_out),
			   dev->out_buf, n, &act, 2000);
	if (ret < 0 || act != n) {
		dev_err(&adapter->dev, "bulk out %d/%d error %d\n", act, n, ret);
		return (ret < 0) ? ret : -EIO;
	}

	act = 0;
	n = 1 + ((msg->flags & I2C_M_RD) ? msg->len : 0);
	ret = usb_bulk_msg(dev->usb_dev, usb_rcvbulkpipe(dev->usb_dev, dev->ep_in),
			   dev->in_buf, 32, &act, 2000);
	if (ret < 0 || act != n) {
		dev_err(&adapter->dev, "bulk in %d/%d error %d\n", act, n, ret);
		return (ret < 0) ? ret : -EIO;
	}

	if (dev->in_buf[0] & 0x80)
		return -EREMOTEIO;

	if (msg->flags & I2C_M_RD)
		memcpy(msg->buf, &dev->in_buf[1], msg->len);

	return 0;
}

static int ch341_i2c_xfer(struct i2c_adapter *adapter, struct i2c_msg *msgs,
			  int num)
{
	int i, ret;

	for (i = 0; i < num; i++) {
		ret = ch341_i2c_xfer_msg(adapter, &msgs[i], i == (num - 1));
		if (ret)
			return ret;
	}

	return num;
}

static u32 ch341_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm ch341_i2c_algorithm = {
	.master_xfer = ch341_i2c_xfer,
	.functionality = ch341_i2c_func,
};

static const struct i2c_adapter_quirks ch341_i2c_quirks = {
	.max_read_len = CH341_MAX_BULK_PACKET_SIZE - 1, /* -1 for status byte */
	.max_write_len = CH341_MAX_BULK_PACKET_SIZE - 7, /* -7 for proto overhead */
};

static const struct usb_device_id i2c_ch341_usb_table[] = {
	{ USB_DEVICE(0x1a86, 0x5512) },
	{ }
};
MODULE_DEVICE_TABLE(usb, i2c_ch341_usb_table);

static int ch341_xfer(struct i2c_ch341_usb *dev, int out_len, int in_len)
{
	int retval;
	int actual;

	dev_dbg(&dev->adapter.dev, "bulk_out %d bytes, bulk_in %d bytes\n",
		out_len, (in_len == 0) ? 0 : 32);

	retval = usb_bulk_msg(dev->usb_dev,
			      usb_sndbulkpipe(dev->usb_dev, dev->ep_out),
			      dev->out_buf, out_len, &actual, 2000);
	if (retval < 0)
		return retval;

	if (in_len == 0)
		return actual;

	memset(dev->in_buf, 0, sizeof(dev->in_buf));
	retval = usb_bulk_msg(dev->usb_dev,
			      usb_rcvbulkpipe(dev->usb_dev, dev->ep_in),
			      dev->in_buf, 32, &actual, 2000);
	if (retval < 0)
		return retval;

	return actual;
}

static void i2c_ch341_usb_free(struct i2c_ch341_usb *dev)
{
	usb_put_dev(dev->usb_dev);
	kfree(dev);
}

static int i2c_ch341_usb_probe(struct usb_interface *iface,
			       const struct usb_device_id *id)
{
	struct i2c_ch341_usb *dev;
	int retval = -ENOMEM;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (dev == NULL) {
		dev_err(&iface->dev, "Out of memory\n");
		goto error;
	}

	dev->usb_dev = usb_get_dev(interface_to_usbdev(iface));
	dev->iface = iface;

	dev->ep_out = iface->cur_altsetting->endpoint[1].desc.bEndpointAddress;
	dev->ep_in = iface->cur_altsetting->endpoint[0].desc.bEndpointAddress;

	/* save our data pointer in this interface device */
	usb_set_intfdata(iface, dev);

	/* setup i2c adapter description */
	dev->adapter.owner = THIS_MODULE;
	dev->adapter.class = I2C_CLASS_HWMON;
	dev->adapter.algo = &ch341_i2c_algorithm;
	dev->adapter.algo_data = dev;
	dev->adapter.quirks = &ch341_i2c_quirks;
	snprintf(dev->adapter.name, sizeof(dev->adapter.name),
		 "i2c-ch341-usb at bus %03d device %03d",
		 dev->usb_dev->bus->busnum, dev->usb_dev->devnum);

	dev->adapter.dev.parent = &dev->iface->dev;

	/* and finally attach to i2c layer */
	i2c_add_adapter(&dev->adapter);

	/* set ch341 i2c speed */
	dev->out_buf[0] = CH341_CMD_I2C_STREAM;
	dev->out_buf[1] = CH341_CMD_I2C_STM_SET | CH341_I2C_STANDARD_SPEED;
	dev->out_buf[2] = CH341_CMD_I2C_STM_END;
	retval = ch341_xfer(dev, 3, 0);
	if (retval < 0) {
		dev_err(&dev->adapter.dev, "failure setting speed\n");
		retval = -EIO;
		goto error;
	}

	dev_info(&dev->adapter.dev, "connected i2c-ch341-usb device\n");

	return 0;

error:
	if (dev)
		i2c_ch341_usb_free(dev);

	return retval;
}

static void i2c_ch341_usb_disconnect(struct usb_interface *iface)
{
	struct i2c_ch341_usb *dev = usb_get_intfdata(iface);

	i2c_del_adapter(&dev->adapter);
	usb_set_intfdata(iface, NULL);
	i2c_ch341_usb_free(dev);

	dev_dbg(&iface->dev, "disconnected\n");
}

static struct usb_driver i2c_ch341_usb_driver = {
	.name = "i2c-ch341-usb",
	.probe = i2c_ch341_usb_probe,
	.disconnect = i2c_ch341_usb_disconnect,
	.id_table = i2c_ch341_usb_table,
};
module_usb_driver(i2c_ch341_usb_driver);

MODULE_AUTHOR("Tse Lun Bien <allanbian@gmail.com>");
MODULE_AUTHOR("Hans de Goede <hansg@kernel.org>");
MODULE_DESCRIPTION("i2c-ch341-usb driver");
MODULE_LICENSE("GPL");
