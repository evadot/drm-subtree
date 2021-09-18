/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2021 Jesper Schmitz Mouridsen  <jsm@FreeBSD.org>
 * Copyright (c) 2019 Jonathan A. Kollasch <jakllsch@kollasch.net>

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
 *
 * $FreeBSD$
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");
#include "rk_edp.h"
#include "syscon_if.h"
#include "dev/drm/bridges/anxdp/anx_dp.h"
#include "dw_hdmi_if.h"
//#include "iicbus_if.h"
#define	 RK3399_GRF_SOC_CON20		0x6250
#define  EDP_LCDC_SEL			BIT(5)


static struct ofw_compat_data   rkedp_compat_data[] = {
	{"rockchip,rk3399-edp",     1},
	{NULL,                      0}

};

static struct resource_spec rk_edp_spec[]  = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE | RF_SHAREABLE },
	{ -1,0}
};
static int rk_edp_probe(device_t dev);
static int rk_edp_attach(device_t dev);

static int rk_edp_add_encoder(device_t dev, struct drm_crtc *crtc, struct drm_device *drm);

#define	to_rk_anxdp_softc(x)	container_of(x, struct rk_edp_softc, sc_base)
#define	to_anxdp_encoder(x)	container_of(x, struct anxdp_softc, sc_encoder)


static void rk_anxdp_select_input(struct rk_edp_softc *sc, u_int crtc_index)
{
	const uint32_t write_mask = EDP_LCDC_SEL << 16;
	const uint32_t write_val = crtc_index == 0 ? EDP_LCDC_SEL : 0;
	SYSCON_WRITE_4(sc->grf, RK3399_GRF_SOC_CON20, write_mask | write_val);

}


static void
rk_anxdp_encoder_prepare(struct drm_encoder *encoder)
{
	printf("%s:%d %s\n",__FILE__,__LINE__,__FUNCTION__);
	struct anxdp_softc *sc_base;
	struct rk_edp_softc *sc;
	sc_base = container_of(encoder, struct anxdp_softc, sc_encoder);
	sc = container_of(sc_base, struct rk_edp_softc, sc_base);

	const u_int crtc_index = drm_crtc_index(encoder->crtc);

	rk_anxdp_select_input(sc, crtc_index);
}

static const struct drm_encoder_funcs rk_anxdp_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static const struct drm_encoder_helper_funcs rk_anxdp_encoder_helper_funcs = {
	.prepare = rk_anxdp_encoder_prepare,
};

#define	to_rk_edp_softc(x)	container_of(x, struct rk_edp_softc, sc_base)
#define	to_rk_edp_encoder(x)	container_of(x, struct rk_edp_softc, sc_encoder)

static device_method_t
rk_edp_methods[] = {
	DEVMETHOD(device_probe,		rk_edp_probe),
	DEVMETHOD(device_attach,	rk_edp_attach),
	DEVMETHOD(dw_hdmi_add_encoder,  rk_edp_add_encoder),
	DEVMETHOD_END
};

static int
rk_edp_probe(device_t dev){

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, rkedp_compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "RockChip edp");
	return (BUS_PROBE_DEFAULT);
}
static int
rk_edp_attach(device_t dev)
{
	struct rk_edp_softc *sc;
	phandle_t node;
	int error;


	sc = device_get_softc(dev);
	mtx_init(&sc->sc_base.mtx, device_get_nameunit(dev), "rk_edp", MTX_DEF);
	node = ofw_bus_get_node(dev);
	if (bus_alloc_resources(dev,rk_edp_spec, sc->sc_base.res)!=0) {
		device_printf(dev, "could not allocate resources\n");
		return (ENXIO);
	}

	sc->sc_base.sc_flags |= ANXDP_FLAG_ROCKCHIP;


	error = clk_get_by_ofw_name(dev, 0, "pclk", &sc->pclk);
	if (error!=0) {
		device_printf(dev,"could not get pclk error:%d\n",error);
		return -1;
	}
	error = clk_enable(sc->pclk);
	if (error!=0) {
		device_printf(dev,"could not enable pclk error:%d\n",error);
		return -1;
	}
	error = clk_get_by_ofw_name(dev, 0, "dp", &sc->dpclk);
	if (error!=0) {
		device_printf(dev,"could not get dp clock error:%d\n",error);
		return -1;
	}
	error = clk_enable(sc->dpclk);
	if (error!=0) {
		device_printf(dev,"could not enable dp error:%d\n",error);
		return -1;
	}
	error = clk_get_by_ofw_name(dev, 0, "grf", &sc->grfclk);
	if (error!=0) {
		device_printf(dev,"could not get grf clock error:%d\n",error);
		return -1;
	}
	error = clk_enable(sc->grfclk);
	if (error!=0) {
		device_printf(dev,"could not enable grp clok error:%d\n",error);
		return -1;
	}
	error = syscon_get_by_ofw_property(dev, node, "rockchip,grf", &sc->grf);
	if (error != 0) {
		device_printf(dev,"cannot get grf syscon: %d\n", error);
		return (ENXIO);
	}

	sc->dev=dev;

	sc->sc_base.sc_dev=dev;
	anxdp_attach(&sc->sc_base);
	return (0);

}

static int
rk_edp_add_encoder(device_t dev, struct drm_crtc *crtc, struct drm_device *drm)
{
	struct rk_edp_softc *sc;
	sc = device_get_softc(dev);
	drm_encoder_helper_add(&sc->sc_base.sc_encoder,&rk_anxdp_encoder_helper_funcs);
	sc->sc_base.sc_encoder.possible_crtcs = drm_crtc_mask(crtc);
	drm_encoder_init(drm, &sc->sc_base.sc_encoder, &rk_anxdp_encoder_funcs,
	    DRM_MODE_ENCODER_TMDS, NULL);
	rk_anxdp_select_input(sc,crtc->index);
	anxdp_add_bridge(&sc->sc_base,&sc->sc_base.sc_encoder);
	return (0);
}


static devclass_t rk_edp_devclass;
DEFINE_CLASS_1(rk_edp, rk_edp_driver, rk_edp_methods,
    sizeof(struct rk_edp_softc), anxdp_driver);

EARLY_DRIVER_MODULE(rk_edp, simplebus, rk_edp_driver,rk_edp_devclass,
    0,0,BUS_PASS_SUPPORTDEV + BUS_PASS_ORDER_EARLY);
MODULE_VERSION(rk_edp, 1);
