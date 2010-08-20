/* include/linux/bma150.h
 *
 * Tri-axial acceleration sensor(BMA150) driver
 *
 * Copyright (C) 2008 LGE, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __BMA150_H
#define __BMA150_H

#define ACCEL_IO_TYPE	'B'

#define ACCEL_IOC_ENABLE	_IO(ACCEL_IO_TYPE, 1)
#define ACCEL_IOC_DISABLE	_IO(ACCEL_IO_TYPE, 2)
#define ACCEL_IOCS_SAMPLERATE	_IOW(ACCEL_IO_TYPE, 3, int) /* in ms */
#define ACCEL_IOCG_SAMPLERATE	_IO(ACCEL_IO_TYPE, 4) /* in ms */

#endif
