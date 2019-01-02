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

#ifndef __TUSB320_H__
#define __TUSB320_H__

#define TUSB320_ATTACH	1
#define TUSB320_DETACH	0

enum tusb320_type{
	TUSB320_TYPE_NONE = 0,
	TUSB320_TYPE_SOURCE,
	TUSB320_TYPE_SINK,
	TUSB320_TYPE_ACC
};

enum tusb320_bc_lvl{
	TUSB320_BC_LVL_RA = 0,
	TUSB320_BC_LVL_USB,
	TUSB320_BC_LVL_1P5,
	TUSB320_BC_LVL_3A
};

#endif

