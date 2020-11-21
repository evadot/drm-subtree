/*-
 * Copyright (c) 2015 Michal Meloun
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/malloc.h>

#include <machine/bus.h>

#include <dev/extres/clk/clk.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem.h>
#include <dev/gpio/gpiobusvar.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/drm/nvidia/tegra_drm.h>

#include <gnu/dts/include/dt-bindings/gpio/gpio.h>


int
tegra_drm_encoder_attach(struct tegra_drm_encoder *output, phandle_t node)
{
	int rv;
	phandle_t ddc;

	/* XXX parse output panel here */

	rv = OF_getencprop_alloc(node, "nvidia,edid",
	    (void **)&output->edid);

	/* EDID exist but have invalid size */
	if ((rv >= 0) && (rv != sizeof(struct edid))) {
		device_printf(output->dev,
		    "Malformed \"nvidia,edid\" property\n");
		if (output->edid != NULL)
			free(output->edid, M_OFWPROP);
		return (ENXIO);
	}

	gpio_pin_get_by_ofw_property(output->dev, node, "nvidia,hpd-gpio",
	    &output->gpio_hpd);
	ddc = 0;
	OF_getencprop(node, "nvidia,ddc-i2c-bus", &ddc, sizeof(ddc));
	if (ddc > 0) {
		output->ddc = i2c_bsd_adapter(OF_device_from_xref(ddc));
	}
	if ((output->edid == NULL) && (output->ddc == NULL))
		return (ENXIO);

	if (output->gpio_hpd != NULL) {
		output->connector.polled =
//		    DRM_CONNECTOR_POLL_HPD;
		    DRM_CONNECTOR_POLL_DISCONNECT |
		    DRM_CONNECTOR_POLL_CONNECT;
	}

	return (0);
}

int tegra_drm_encoder_init(struct tegra_drm_encoder *output,
    struct tegra_drm *drm)
{

	if (output->panel) {
		/* attach panel */
	}
	return (0);
}

int tegra_drm_encoder_exit(struct tegra_drm_encoder *output,
    struct tegra_drm *drm)
{

	if (output->panel) {
		/* detach panel */
	}
	return (0);
}
