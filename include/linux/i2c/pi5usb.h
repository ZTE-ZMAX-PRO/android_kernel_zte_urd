/*
 * fusb301.h -- FUSB301 USB TYPE-C Controller device driver
 *
 * Copyright (C) 2015 Fairchild semiconductor Co.Ltd
 * Author: Chris Jeong <chris.jeong@fairchildsemi.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __PI5USB_H__
#define __PI5USB_H__

#define PI5USB_ATTACH	1
#define PI5USB_DETACH	0

enum pi5usb_type{
	PI5USB_TYPE_STANDBY = 0,
	PI5USB_TYPE_DEVICE,
	PI5USB_TYPE_HOST,
	PI5USB_TYPE_AUDIO,
	PI5USB_TYPE_ACCESSORY,
	PI5USB_TYPE_VBUS
};

#endif

