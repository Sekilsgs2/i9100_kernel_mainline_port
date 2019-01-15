/**
 * Copyright (C) 2010-2014 ARM Limited. All rights reserved.
 * 
 * This program is free software and is provided to you under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation, and any use by you of this program is subject to the terms of such GNU licence.
 * 
 * A copy of the licence is included with the program, and can also be obtained from Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/**
 * @file mali_osk_pm.c
 * Implementation of the callback functions from common power management
 */

#include <linux/sched.h>

#ifdef CONFIG_PM
#include <linux/pm_runtime.h>
#endif /* CONFIG_PM */
#include <linux/platform_device.h>
#include <linux/version.h>
#include "mali_osk.h"
#include "mali_kernel_common.h"
#include "mali_kernel_linux.h"

/* Can NOT run in atomic context */
_mali_osk_errcode_t _mali_osk_pm_dev_ref_get_sync(void)
{
#ifdef CONFIG_PM
	int err;
	MALI_DEBUG_ASSERT_POINTER(mali_platform_device);
	err = pm_runtime_get_sync(&(mali_platform_device->dev));
	if (0 > err) {
		MALI_PRINT_ERROR("Mali OSK PM: pm_runtime_get_sync() returned error code %d\n", err);
		return _MALI_OSK_ERR_FAULT;
	}
#endif
	return _MALI_OSK_ERR_OK;
}

/* Can run in atomic context */
_mali_osk_errcode_t _mali_osk_pm_dev_ref_get_async(void)
{
#ifdef CONFIG_PM
	int err;
	MALI_DEBUG_ASSERT_POINTER(mali_platform_device);
	err = pm_runtime_get(&(mali_platform_device->dev));
	if (0 > err && -EINPROGRESS != err) {
		MALI_PRINT_ERROR("Mali OSK PM: pm_runtime_get() returned error code %d\n", err);
		return _MALI_OSK_ERR_FAULT;
	}
#endif
	return _MALI_OSK_ERR_OK;
}


/* Can run in atomic context */
void _mali_osk_pm_dev_ref_put(void)
{
#ifdef CONFIG_PM
	MALI_DEBUG_ASSERT_POINTER(mali_platform_device);
	pm_runtime_mark_last_busy(&(mali_platform_device->dev));
	pm_runtime_put_autosuspend(&(mali_platform_device->dev));
#endif
}

void _mali_osk_pm_dev_barrier(void)
{
#ifdef CONFIG_PM
	pm_runtime_barrier(&(mali_platform_device->dev));
#endif
}
