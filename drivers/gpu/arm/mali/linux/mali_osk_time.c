/*
 * Copyright (C) 2010, 2013-2014 ARM Limited. All rights reserved.
 * 
 * This program is free software and is provided to you under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation, and any use by you of this program is subject to the terms of such GNU licence.
 * 
 * A copy of the licence is included with the program, and can also be obtained from Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/**
 * @file mali_osk_time.c
 * Implementation of the OS abstraction layer for the kernel device driver
 */

#include "mali_osk.h"
#include <linux/timekeeping.h>
#include <asm/delay.h>

mali_bool _mali_osk_time_after_eq(unsigned long ticka, unsigned long tickb)
{
	return time_after_eq(ticka, tickb) ?
	       MALI_TRUE : MALI_FALSE;
}

unsigned long _mali_osk_time_mstoticks(u32 ms)
{
	return msecs_to_jiffies(ms);
}

u32 _mali_osk_time_tickstoms(unsigned long ticks)
{
	return jiffies_to_msecs(ticks);
}

unsigned long _mali_osk_time_tickcount(void)
{
	return jiffies;
}

void _mali_osk_time_ubusydelay(u32 usecs)
{
	udelay(usecs);
}

u64 _mali_osk_time_get_ns(void)
{
	return ktime_get_ns();
}

u64 _mali_osk_boot_time_get_ns(void)
{
	return ktime_get_boot_ns();
}
