/*
 * Copyright (c) 2015-2017, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __IO_USB_H__
#define __IO_USB_H__

#define IO_USB_TIMEOUT		0xFFFFF
/* need to call the USB Handler 4 times after to have
 * received the detach request
 */
#define DETACH_TIMEOUT		0x100

#define USB_DFU_MAX_XFER_SIZE	1024

int register_io_dev_usb(const io_dev_connector_t **dev_con);

#endif /* __IO_USB_H__ */
