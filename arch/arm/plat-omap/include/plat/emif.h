/*
 * EMIF register definitions for TI81xx and AM33xx
 *
 * Copyright (C) 2011 Texas Instruments, Inc. - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __EMIF_H
#define __EMIF_H

#define EMIF4_0_SDRAM_MGMT_CTRL         (0x38)
#define EMIF4_0_SDRAM_MGMT_CTRL_SHD     (0x3C)

#define SELF_REFRESH_ENABLE(m)		(0x2 << 8 | (m << 4))
#define SELF_REFRESH_DISABLE		(0x0 << 8)

#endif /* __EMIF_H */
