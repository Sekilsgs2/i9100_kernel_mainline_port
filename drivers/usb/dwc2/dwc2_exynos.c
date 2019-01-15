
// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * platform.c - DesignWare HS OTG Controller platform driver
 *
 * Copyright (C) Matthijs Kooijman <matthijs@stdin.nl>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The names of the above-listed copyright holders may not be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/of_device.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>
#include <linux/platform_data/s3c-hsotg.h>
#include <linux/reset.h>
#include <linux/usb/of.h>
#include <linux/extcon.h>


#define EXTCON_DEV_NAME                        "max8997-muic"

struct dwc2_exynos {
        struct device           *dev;

        struct extcon_dev       *edev;
        struct notifier_block   id_nb;
}


static int dwc2_i9100_id_notifier(struct notifier_block *nb,
        unsigned long event, void *ptr)
{
        u32 gintsts;
        unsigned long flags;

        struct dwc2_hsotg *hsotg = container_of(nb, struct dwc2_hsotg, id_nb);

        if (event) {
                dev_info(hsotg->dev, "ATTACHED USB-HOST\n");
                hsotg->ex_host = 1;
                dwc2_force_mode(hsotg, true);

                /* Need to disable SOF interrupt immediately */
                gintsts = dwc2_readl(hsotg, GINTSTS);
                gintsts &= ~GINTSTS_CONIDSTSCHNG;
                dwc2_writel(hsotg, gintsts, GINTSTS);

        }

        else {
                hsotg->ex_host = 0;
                dev_info(hsotg->dev, "DETACHED??? USB-HOST\n");
                dwc2_force_mode(hsotg, false);

                /* Need to disable SOF interrupt immediately */
                gintsts = dwc2_readl(hsotg, GINTSTS);
                gintsts &= ~GINTSTS_CONIDSTSCHNG;
                dwc2_writel(hsotg, gintsts, GINTSTS);

        }

        return NOTIFY_DONE;
}


