/*
 * Copyright (c) 2015-2017, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __USB_CTX_H
#define __USB_CTX_H

#include <usb_core.h>

struct usb_ctx {
	usb_handle_t *pusbd_device_ctx;
	pcd_handle_t *phpcd_ctx;
};

#endif /* __USB_CTX_H */
