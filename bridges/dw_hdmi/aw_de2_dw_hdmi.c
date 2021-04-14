/*-
 * Copyright (c) 2019 Emmanuel Vadot <manu@FreeBSD.org>
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/resource.h>
#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/extres/clk/clk.h>
#include <dev/extres/hwreset/hwreset.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_print.h>

#include "dw_hdmireg.h"

#include "dw_hdmi_if.h"
#include "dw_hdmi_phy_if.h"

#include "iicbus_if.h"

/* Redefine msleep because of drmkpi */
#undef msleep
#define	msleep(chan, mtx, pri, wmesg, timo)				\
	_sleep((chan), &(mtx)->lock_object, (pri), (wmesg),		\
	    tick_sbt * (timo), 0, C_HARDCLOCK)

static struct ofw_compat_data aw_compat_data[] = {
	{ "allwinner,sun50i-a64-dw-hdmi",	1 },
	{ "allwinner,sun8i-h3-dw-hdmi",		1 },
	{ NULL,					0 }
};

static struct ofw_compat_data rk_compat_data[] = {
	{ "rockchip,rk3399-dw-hdmi",		1 },
	{ NULL,					0 }
};

struct dw_hdmi_softc {
	device_t	dev;
	device_t	phydev;		/* Optional */
	struct resource	*res[2];
	void *		intrhand;
	struct mtx	mtx;

	clk_t		clk_iahb;
	clk_t		clk_isfr;

	device_t		iicbus;
	struct i2c_adapter	*ddc;
	uint8_t			i2cm_stat;
	uint8_t			i2cm_addr;

	uint32_t		reg_width;

	struct drm_encoder	encoder;
	struct drm_connector	connector;
	struct drm_bridge	bridge;
	struct drm_display_mode	mode;
};

struct aw_dw_hdmi_softc {
	struct dw_hdmi_softc base_sc;

	clk_t		clk_tmds;	/* Allwinner specific */
	hwreset_t	reset_ctrl;	/* Allwinner specific */
};

struct rk_dw_hdmi_softc {
	struct dw_hdmi_softc base_sc;
};

static struct resource_spec dw_hdmi_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE | RF_SHAREABLE },
	{ -1, 0 }
};

#define	DW_HDMI_READ_1(sc, reg)		bus_read_1((sc)->res[0], (reg))
#define	DW_HDMI_WRITE_1(sc, reg, val)	bus_write_1((sc)->res[0], (reg), (val))
#define	DW_HDMI_READ_4(sc, reg)		bus_read_4((sc)->res[0], (reg))
#define	DW_HDMI_WRITE_4(sc, reg, val)	bus_write_4((sc)->res[0], (reg), (val))

#define	DW_HDMI_LOCK(sc)		mtx_lock(&(sc)->mtx)
#define	DW_HDMI_UNLOCK(sc)		mtx_unlock(&(sc)->mtx)

#define DDC_SEGMENT_ADDR 0x30

static int aw_de2_dw_hdmi_probe(device_t dev);
static int aw_de2_dw_hdmi_attach(device_t dev);
static int aw_de2_dw_hdmi_detach(device_t dev);

static int rk_dw_hdmi_probe(device_t dev);
static int rk_dw_hdmi_attach(device_t dev);
static int rk_dw_hdmi_detach(device_t dev);

static uint32_t dw_hdmi_read(struct dw_hdmi_softc *sc, uint32_t reg);
static void dw_hdmi_write(struct dw_hdmi_softc *sc, uint32_t reg, uint32_t val);

static enum drm_connector_status
dw_hdmi_connector_detect(struct drm_connector *connector, bool force)
{
	struct dw_hdmi_softc *sc;

	sc = container_of(connector, struct dw_hdmi_softc, connector);

	if (sc->phydev != NULL) {
		if (DW_HDMI_PHY_DETECT_HPD(sc->phydev))
			return (connector_status_connected);
	} else {
		/* Handle internal phy */
	}

	return (connector_status_disconnected);
}

static const struct drm_connector_funcs dw_hdmi_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = dw_hdmi_connector_detect,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static void
dw_hdmi_i2cm_init(struct dw_hdmi_softc *sc)
{

	/* I2CM Setup */
	dw_hdmi_write(sc, DW_HDMI_PHY_I2CM_INT_ADDR, 0x08);
	dw_hdmi_write(sc, DW_HDMI_PHY_I2CM_CTLINT_ADDR, 0x88);

	/* Soft reset */
	dw_hdmi_write(sc, DW_HDMI_I2CM_SOFTRSTZ, 0);

	/* standard speed mode */
	dw_hdmi_write(sc, DW_HDMI_I2CM_DIV, 0);

	dw_hdmi_write(sc, DW_HDMI_I2CM_INT,
	    DW_HDMI_I2CM_INT_DONE_POL);
	dw_hdmi_write(sc, DW_HDMI_I2CM_CLINT,
	    DW_HDMI_I2CM_CLINT_NACK_POL | DW_HDMI_I2CM_CLINT_ARB_POL);

	/* Clear interrupts */
	dw_hdmi_write(sc, DW_HDMI_IH_I2CM_STAT0,
	  DW_HDMI_IH_I2CM_STAT0_ERROR |
	  DW_HDMI_IH_I2CM_STAT0_DONE);
}

static int
dw_hdmi_i2c_write(struct dw_hdmi_softc *sc, uint8_t *buf, uint16_t len)
{
	int i, err = 0;

	for (i = 0; i < len; i++) {
		dw_hdmi_write(sc, DW_HDMI_I2CM_DATAO, buf[i]);
		dw_hdmi_write(sc, DW_HDMI_I2CM_ADDRESS, i);
		dw_hdmi_write(sc, DW_HDMI_I2CM_OP, DW_HDMI_I2CM_OP_WR);

		while (err == 0 && sc->i2cm_stat == 0) {
			err = msleep(sc, &sc->mtx, 0, "dw_hdmi_ddc", 10 * hz);
		}
		if (err || sc->i2cm_stat & DW_HDMI_IH_I2CM_STAT0_ERROR) {
			device_printf(sc->dev, "%s: error\n", __func__);
			return (ENXIO);
		}
	}
	return (0);
}

static int
dw_hdmi_i2c_read(struct dw_hdmi_softc *sc, uint8_t *buf, uint16_t len)
{
	int i, err = 0;

	for (i = 0; i < len; i++) {
		dw_hdmi_write(sc, DW_HDMI_I2CM_ADDRESS, sc->i2cm_addr++);
		dw_hdmi_write(sc, DW_HDMI_I2CM_OP, DW_HDMI_I2CM_OP_RD);

		while (err == 0 && sc->i2cm_stat == 0) {
			err = msleep(sc, &sc->mtx, 0, "dw_hdmi_ddc", 10 * hz);
		}
		if (err || sc->i2cm_stat & DW_HDMI_IH_I2CM_STAT0_ERROR) {
			device_printf(sc->dev, "%s: error\n", __func__);
			return (ENXIO);
		}

		buf[i] = dw_hdmi_read(sc, DW_HDMI_I2CM_DATAI);
		sc->i2cm_stat = 0;
	}

	return (0);
}

static int
dw_hdmi_transfer(device_t dev, struct iic_msg *msgs, uint32_t nmsgs)
{
	struct dw_hdmi_softc *sc;
	int i, ret;

	sc = device_get_softc(dev);
	DW_HDMI_LOCK(sc);

	sc->i2cm_addr = 0;
	for (i = 0; i < nmsgs; i++) {
		sc->i2cm_stat = 0;
		/* Unmute done and error interrups */
		dw_hdmi_write(sc, DW_HDMI_IH_MUTE_I2CM_STAT0, 0x00);

		/* Set DDC seg/addr */
		dw_hdmi_write(sc, DW_HDMI_I2CM_SLAVE, msgs[i].slave >> 1);
		dw_hdmi_write(sc, DW_HDMI_I2CM_SEGADDR, DDC_SEGMENT_ADDR);

		if (msgs[i].flags & IIC_M_RD)
			ret = dw_hdmi_i2c_read(sc, msgs[i].buf, msgs[i].len);
		else {
			if (msgs[i].len == 1) {
				sc->i2cm_addr = msgs[i].buf[0];
			} else 
				ret = dw_hdmi_i2c_write(sc, msgs[i].buf,
				    msgs[i].len);
		}

		if (ret != 0)
			break;
	}

	/* mute done and error interrups */
	dw_hdmi_write(sc, DW_HDMI_IH_MUTE_I2CM_STAT0, 0xFF);

	DW_HDMI_UNLOCK(sc);
	return (0);
}

static int
dw_hdmi_connector_get_modes(struct drm_connector *connector)
{
	struct dw_hdmi_softc *sc;
	struct edid *edid = NULL;
	int ret = 0;

	sc = container_of(connector, struct dw_hdmi_softc, connector);

	edid = drm_get_edid(connector, sc->ddc);
	drm_connector_update_edid_property(connector, edid);
	ret = drm_add_edid_modes(connector, edid);

	return (ret);
}

static const struct drm_connector_helper_funcs
    dw_hdmi_connector_helper_funcs = {
	.get_modes = dw_hdmi_connector_get_modes,
};

/* bridge funcs, should be in dw_hdmi */
static int
dw_hdmi_bridge_attach(struct drm_bridge *bridge)
{
	struct dw_hdmi_softc *sc;

	sc = container_of(bridge, struct dw_hdmi_softc, bridge);

	sc->connector.polled = DRM_CONNECTOR_POLL_HPD;
	drm_connector_helper_add(&sc->connector,
	    &dw_hdmi_connector_helper_funcs);

	drm_connector_init(bridge->dev, &sc->connector,
	    &dw_hdmi_connector_funcs, DRM_MODE_CONNECTOR_HDMIA);

	drm_connector_attach_encoder(&sc->connector, &sc->encoder);

	return (0);
}

/* TODO: Is there some mode that we don't support ? */
static enum drm_mode_status
dw_hdmi_bridge_mode_valid(struct drm_bridge *bridge,
			  const struct drm_display_mode *mode)
{
	struct dw_hdmi_softc *sc;

	sc = container_of(bridge, struct dw_hdmi_softc, bridge);

	return (MODE_OK);
}

static void
dw_hdmi_bridge_mode_set(struct drm_bridge *bridge,
  const struct drm_display_mode *orig_mode,
  const struct drm_display_mode *mode)
{
	struct dw_hdmi_softc *sc;

	sc = container_of(bridge, struct dw_hdmi_softc, bridge);

	/* Copy the mode, this will be set in bridge_enable function */
	memcpy(&sc->mode, mode, sizeof(struct drm_display_mode));
}

static void
dw_hdmi_bridge_disable(struct drm_bridge *bridge)
{
	struct dw_hdmi_softc *sc;

	sc = container_of(bridge, struct dw_hdmi_softc, bridge);
}

static inline void
dw_hdmi_dump_vp_regs(struct dw_hdmi_softc *sc)
{
	uint8_t	reg;

	DRM_DEBUG_DRIVER("%s: DW_HDMI VP Registers\n", __func__);
	reg = dw_hdmi_read(sc, DW_HDMI_VP_STATUS);
	DRM_DEBUG_DRIVER("%s: DW_HDMI_VP_STATUS: %x\n", __func__, reg);
	reg = dw_hdmi_read(sc, DW_HDMI_VP_PR_CD);
	DRM_DEBUG_DRIVER("%s: DW_HDMI_VP_PR_CD: %x\n", __func__, reg);
	reg = dw_hdmi_read(sc, DW_HDMI_VP_STUFF);
	DRM_DEBUG_DRIVER("%s: DW_HDMI_VP_STUFF: %x\n", __func__, reg);
	reg = dw_hdmi_read(sc, DW_HDMI_VP_REMAP);
	DRM_DEBUG_DRIVER("%s: DW_HDMI_VP_REMAP: %x\n", __func__, reg);
	reg = dw_hdmi_read(sc, DW_HDMI_VP_CONF);
	DRM_DEBUG_DRIVER("%s: DW_HDMI_VP_CONF: %x\n", __func__, reg);
	reg = dw_hdmi_read(sc, DW_HDMI_VP_MASK);
	DRM_DEBUG_DRIVER("%s: DW_HDMI_VP_MASK: %x\n", __func__, reg);
}

static inline void
dw_hdmi_dump_fc_regs(struct dw_hdmi_softc *sc)
{
	uint8_t	reg;

	DRM_DEBUG_DRIVER("%s: DW_HDMI FC Registers\n", __func__);
	reg = dw_hdmi_read(sc, DW_HDMI_FC_INVIDCONF);
	DRM_DEBUG_DRIVER("%s: DW_HDMI_FC_INVIDCONF: %d\n", __func__, reg);
	reg = dw_hdmi_read(sc, DW_HDMI_FC_INHACTIV0);
	DRM_DEBUG_DRIVER("%s: DW_HDMI_FC_INHACTIV0: %d\n", __func__, reg);
	reg = dw_hdmi_read(sc, DW_HDMI_FC_INHACTIV1);
	DRM_DEBUG_DRIVER("%s: DW_HDMI_FC_INHACTIV1: %d\n", __func__, reg);
	reg = dw_hdmi_read(sc, DW_HDMI_FC_INHBLANK0);
	DRM_DEBUG_DRIVER("%s: DW_HDMI_FC_INHBLANK0: %d\n", __func__, reg);
	reg = dw_hdmi_read(sc, DW_HDMI_FC_INHBLANK1);
	DRM_DEBUG_DRIVER("%s: DW_HDMI_FC_INHBLANK1: %d\n", __func__, reg);
	reg = dw_hdmi_read(sc, DW_HDMI_FC_INVACTIV0);
	DRM_DEBUG_DRIVER("%s: DW_HDMI_FC_INVACTIV1: %d\n", __func__, reg);
	reg = dw_hdmi_read(sc, DW_HDMI_FC_HSYNCINDELAY0);
	DRM_DEBUG_DRIVER("%s: DW_HDMI_FC_HSYNCINDELAY0: %d\n", __func__, reg);
	reg = dw_hdmi_read(sc, DW_HDMI_FC_HSYNCINDELAY1);
	DRM_DEBUG_DRIVER("%s: DW_HDMI_FC_HSYNCINDELAY1: %d\n", __func__, reg);
	reg = dw_hdmi_read(sc, DW_HDMI_FC_HSYNCINWIDTH0);
	DRM_DEBUG_DRIVER("%s: DW_HDMI_FC_HSYNCINWIDTH0: %d\n", __func__, reg);
	reg = dw_hdmi_read(sc, DW_HDMI_FC_HSYNCINWIDTH1);
	DRM_DEBUG_DRIVER("%s: DW_HDMI_FC_HSYNCINWIDTH1: %d\n", __func__, reg);
	reg = dw_hdmi_read(sc, DW_HDMI_FC_VSYNCINDELAY);
	DRM_DEBUG_DRIVER("%s: DW_HDMI_FC_VSYNCINDELAY: %d\n", __func__, reg);
	reg = dw_hdmi_read(sc, DW_HDMI_FC_VSYNCINWIDTH);
	DRM_DEBUG_DRIVER("%s: DW_HDMI_FC_VSYNCINWIDTH: %d\n", __func__, reg);
	reg = dw_hdmi_read(sc, DW_HDMI_FC_CTRLDUR);
	DRM_DEBUG_DRIVER("%s: DW_HDMI_FC_CTRLDUR: %d\n", __func__, reg);
	reg = dw_hdmi_read(sc, DW_HDMI_FC_EXCTRLDUR);
	DRM_DEBUG_DRIVER("%s: DW_HDMI_FC_EXCTRLDUR: %d\n", __func__, reg);
	reg = dw_hdmi_read(sc, DW_HDMI_FC_EXCTRLSPAC);
	DRM_DEBUG_DRIVER("%s: DW_HDMI_FC_EXCTRLSPAC: %d\n", __func__, reg);
	reg = dw_hdmi_read(sc, DW_HDMI_FC_CH0PREAM);
	DRM_DEBUG_DRIVER("%s: DW_HDMI_FC_CH0PREAM: %d\n", __func__, reg);
	reg = dw_hdmi_read(sc, DW_HDMI_FC_CH1PREAM);
	DRM_DEBUG_DRIVER("%s: DW_HDMI_FC_CH1PREAM: %d\n", __func__, reg);
	reg = dw_hdmi_read(sc, DW_HDMI_FC_CH2PREAM);
	DRM_DEBUG_DRIVER("%s: DW_HDMI_FC_CH2PREAM: %d\n", __func__, reg);
}

static void
dw_hdmi_bridge_enable(struct drm_bridge *bridge)
{
	struct dw_hdmi_softc *sc;
	uint8_t reg;

	sc = container_of(bridge, struct dw_hdmi_softc, bridge);

	DRM_DEBUG_DRIVER("%s: Mode information:\n"
	    "hdisplay: %d\n"
	    "vdisplay: %d\n"
	    "htotal: %d\n"
	    "vtotal: %d\n"
	    "hsync_start: %d\n"
	    "hsync_end: %d\n"
	    "vsync_start: %d\n"
	    "vsync_end: %d\n",
	    __func__,
	    sc->mode.hdisplay,
	    sc->mode.vdisplay,
	    sc->mode.htotal,
	    sc->mode.vtotal,
	    sc->mode.hsync_start,
	    sc->mode.hsync_end,
	    sc->mode.vsync_start,
	    sc->mode.vsync_end);

	/* VP stuff, need to find what's really needed */
	dw_hdmi_write(sc, DW_HDMI_VP_STUFF, 0x27);
	dw_hdmi_write(sc, DW_HDMI_VP_CONF, 0x47);

	/* AV composer setup */
	reg = (sc->mode.flags & DRM_MODE_FLAG_PVSYNC) ?
		DW_HDMI_FC_INVIDCONF_VSYNC_POL_HIGH : 0;
	reg |= (sc->mode.flags & DRM_MODE_FLAG_PHSYNC) ?
		DW_HDMI_FC_INVIDCONF_HSYNC_POL_HIGH : 0;
	reg |= DW_HDMI_FC_INVIDCONF_DATA_POL_HIGH;

	reg |= (sc->mode.flags & DRM_MODE_FLAG_INTERLACE) ?
		DW_HDMI_FC_INVIDCONF_INTERLACED_MODE : 0;

	/* Will need to depend on drm_detect_hdmi_monitor return value */
	reg |= DW_HDMI_FC_INVIDCONF_HDMI_MODE;
	dw_hdmi_write(sc, DW_HDMI_FC_INVIDCONF, reg);

	/* Frame composer setup */
	dw_hdmi_write(sc, DW_HDMI_FC_INHACTIV0, sc->mode.hdisplay & 0xFF);
	dw_hdmi_write(sc, DW_HDMI_FC_INHACTIV1, sc->mode.hdisplay >> 8);
	dw_hdmi_write(sc, DW_HDMI_FC_INHBLANK0,
	    (sc->mode.htotal - sc->mode.hdisplay) & 0xFF);
	dw_hdmi_write(sc, DW_HDMI_FC_INHBLANK1,
	    (sc->mode.htotal - sc->mode.hdisplay) >> 8);
	dw_hdmi_write(sc, DW_HDMI_FC_INVACTIV0, sc->mode.vdisplay & 0xFF);
	dw_hdmi_write(sc, DW_HDMI_FC_INVACTIV1, sc->mode.vdisplay >> 8);
	dw_hdmi_write(sc, DW_HDMI_FC_INVBLANK,
	    sc->mode.vtotal - sc->mode.vdisplay);
	dw_hdmi_write(sc, DW_HDMI_FC_HSYNCINDELAY0,
	    (sc->mode.hsync_start - sc->mode.hdisplay) & 0xFF);
	dw_hdmi_write(sc, DW_HDMI_FC_HSYNCINDELAY1,
	    (sc->mode.hsync_start - sc->mode.hdisplay) >> 8);
	dw_hdmi_write(sc, DW_HDMI_FC_HSYNCINWIDTH0,
	    (sc->mode.hsync_end - sc->mode.hsync_start) & 0xFF);
	dw_hdmi_write(sc, DW_HDMI_FC_HSYNCINWIDTH1,
	    (sc->mode.hsync_end - sc->mode.hsync_start) >> 8);
	dw_hdmi_write(sc, DW_HDMI_FC_VSYNCINDELAY,
	    sc->mode.vsync_start - sc->mode.vdisplay);
	dw_hdmi_write(sc, DW_HDMI_FC_VSYNCINWIDTH,
	    sc->mode.vsync_end - sc->mode.vsync_start);

	/* Configure the PHY */
	DW_HDMI_PHY_CONFIG(sc->phydev, &sc->mode);

	/* 12 pixel clock cycles */
	dw_hdmi_write(sc, DW_HDMI_FC_CTRLDUR, 12);
	/* 32 pixel clock cycles */
	dw_hdmi_write(sc, DW_HDMI_FC_EXCTRLDUR, 32);
	/* 1 50msec spacing */
	dw_hdmi_write(sc, DW_HDMI_FC_EXCTRLSPAC, 1);

	/* pream defaults */
	dw_hdmi_write(sc, DW_HDMI_FC_CH0PREAM, 11);
	dw_hdmi_write(sc, DW_HDMI_FC_CH1PREAM, 22);
	dw_hdmi_write(sc, DW_HDMI_FC_CH2PREAM, 33);

	/* Enable pixel clock and TMDS clock */
	reg = DW_HDMI_MC_CLKDIS_PREPCLK |
		DW_HDMI_MC_CLKDIS_AUDCLK |
		DW_HDMI_MC_CLKDIS_CSCCLK |
		DW_HDMI_MC_CLKDIS_CECCLK |
		DW_HDMI_MC_CLKDIS_HDCPCLK;
	reg &= ~DW_HDMI_MC_CLKDIS_PIXELCLK;
	dw_hdmi_write(sc, DW_HDMI_MC_CLKDIS, reg);

	reg &= ~DW_HDMI_MC_CLKDIS_TMDSCLK;
	dw_hdmi_write(sc, DW_HDMI_MC_CLKDIS, reg);

	if (__drm_debug & DRM_UT_DRIVER) {
		dw_hdmi_dump_vp_regs(sc);
		dw_hdmi_dump_fc_regs(sc);
	}
}

static const struct drm_bridge_funcs dw_hdmi_bridge_funcs = {
	.attach = dw_hdmi_bridge_attach,
	.enable = dw_hdmi_bridge_enable,
	.disable = dw_hdmi_bridge_disable,
	.mode_set = dw_hdmi_bridge_mode_set,
	.mode_valid = dw_hdmi_bridge_mode_valid,
};

/* Encoder funcs, belongs here */
static void aw_de2_dw_hdmi_encoder_mode_set(struct drm_encoder *encoder,
    struct drm_display_mode *mode,
    struct drm_display_mode *adj_mode)
{
	struct aw_dw_hdmi_softc *sc;
	struct dw_hdmi_softc *base_sc;
	uint64_t freq;

	base_sc = container_of(encoder, struct dw_hdmi_softc, encoder);
	sc = container_of(base_sc, struct aw_dw_hdmi_softc, base_sc);

	clk_get_freq(sc->clk_tmds, &freq);
	DRM_DEBUG_DRIVER("%s: Setting clock %s from %ju to %ju\n",
	    __func__,
	    clk_get_name(sc->clk_tmds),
	    freq,
	    (uintmax_t)mode->crtc_clock * 1000);
	clk_set_freq(sc->clk_tmds, mode->crtc_clock * 1000, CLK_SET_ROUND_ANY);
	clk_get_freq(sc->clk_tmds, &freq);
	DRM_DEBUG_DRIVER("%s: New clock %s is %ju\n",
	    __func__,
	    clk_get_name(sc->clk_tmds),
	    freq);
}

static const struct drm_encoder_helper_funcs
    aw_de2_dw_hdmi_encoder_helper_funcs = {
	.mode_set = aw_de2_dw_hdmi_encoder_mode_set,
};

static const struct drm_encoder_funcs aw_dw_hdmi_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int
aw_dw_hdmi_add_encoder(device_t dev, struct drm_crtc *crtc,
    struct drm_device *drm)
{
	struct aw_dw_hdmi_softc *sc;

	sc = device_get_softc(dev);

	drm_encoder_helper_add(&sc->base_sc.encoder,
	    &aw_de2_dw_hdmi_encoder_helper_funcs);
	sc->base_sc.encoder.possible_crtcs = drm_crtc_mask(crtc);
	drm_encoder_init(drm, &sc->base_sc.encoder, &aw_dw_hdmi_encoder_funcs,
	  DRM_MODE_ENCODER_TMDS, NULL);

	/* This part should be in dw_hdmi */
	sc->base_sc.bridge.funcs = &dw_hdmi_bridge_funcs;
	drm_bridge_attach(&sc->base_sc.encoder, &sc->base_sc.bridge, NULL);

	return (0);
}

static void rk_dw_hdmi_encoder_mode_set(struct drm_encoder *encoder,
    struct drm_display_mode *mode,
    struct drm_display_mode *adj_mode)
{
	struct rk_dw_hdmi_softc *sc;
	struct dw_hdmi_softc *base_sc;

	base_sc = container_of(encoder, struct dw_hdmi_softc, encoder);
	sc = container_of(base_sc, struct rk_dw_hdmi_softc, base_sc);
}

static const struct drm_encoder_helper_funcs rk_dw_hdmi_encoder_helper_funcs = {
	.mode_set = rk_dw_hdmi_encoder_mode_set,
};

static const struct drm_encoder_funcs rk_dw_hdmi_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int
rk_dw_hdmi_add_encoder(device_t dev, struct drm_crtc *crtc,
    struct drm_device *drm)
{
	struct rk_dw_hdmi_softc *sc;

	sc = device_get_softc(dev);

	drm_encoder_helper_add(&sc->base_sc.encoder,
	    &rk_dw_hdmi_encoder_helper_funcs);
	sc->base_sc.encoder.possible_crtcs = drm_crtc_mask(crtc);
	drm_encoder_init(drm, &sc->base_sc.encoder, &rk_dw_hdmi_encoder_funcs,
	  DRM_MODE_ENCODER_TMDS, NULL);

	/* This part should be in dw_hdmi */
	sc->base_sc.bridge.funcs = &dw_hdmi_bridge_funcs;
	drm_bridge_attach(&sc->base_sc.encoder, &sc->base_sc.bridge, NULL);

	return (0);
}

static void
dw_hdmi_intr(void *arg)
{
	struct dw_hdmi_softc *sc;

	sc = (struct dw_hdmi_softc *)arg;

	sc->i2cm_stat = dw_hdmi_read(sc, DW_HDMI_IH_I2CM_STAT0);
	if (sc->i2cm_stat != 0) {
		/* Ack interrupts */
		dw_hdmi_write(sc, DW_HDMI_IH_I2CM_STAT0, sc->i2cm_stat);
	}

	wakeup(sc);
}

/*
 * Driver routines
 */
static uint32_t
dw_hdmi_read(struct dw_hdmi_softc *sc, uint32_t reg)
{

	switch (sc->reg_width) {
	case 4:
		return (DW_HDMI_READ_4(sc, reg << 2));
		break;
	case 1:
	default:
		return (DW_HDMI_READ_1(sc, reg));
		break;
	}
}

static void
dw_hdmi_write(struct dw_hdmi_softc *sc, uint32_t reg, uint32_t val)
{

	switch (sc->reg_width) {
	case 4:
		DW_HDMI_WRITE_4(sc, reg << 2, val);
		break;
	case 1:
	default:
		DW_HDMI_WRITE_1(sc, reg, val);
		break;
	}
}

static int
dw_hdmi_attach(device_t dev)
{
	struct dw_hdmi_softc *sc;
	phandle_t node, phy;
	int error;
	uint16_t version;

	sc = device_get_softc(dev);
	sc->dev = dev;

	if (bus_alloc_resources(dev, dw_hdmi_spec, sc->res) != 0) {
		device_printf(dev, "cannot allocate resources for device\n");
		error = ENXIO;
		goto fail;
	}
	if (bus_setup_intr(dev, sc->res[1],
	    INTR_TYPE_MISC | INTR_MPSAFE, NULL, dw_hdmi_intr, sc,
	    &sc->intrhand)) {
		bus_release_resources(dev, dw_hdmi_spec, sc->res);
		device_printf(dev, "cannot setup interrupt handler\n");
		return (ENXIO);
	}

	mtx_init(&sc->mtx, device_get_nameunit(dev), "dw_hdmi", MTX_DEF);

	node = ofw_bus_get_node(dev);

	/* Clock and reset */
	if ((error = clk_get_by_ofw_name(dev, node, "iahb",
	    &sc->clk_iahb)) != 0) {
		device_printf(dev, "Cannot get iahb clock\n");
		goto fail;
	}
	if (clk_enable(sc->clk_iahb) != 0) {
		device_printf(dev, "Cannot enable iahb clock\n");
		goto fail;
	}
	if ((error = clk_get_by_ofw_name(dev, node, "isfr",
	    &sc->clk_isfr)) != 0) {
		device_printf(dev, "Cannot get isfr clock\n");
		goto fail;
	}
	if (clk_enable(sc->clk_isfr) != 0) {
		device_printf(dev, "Cannot enable isfr clock\n");
		goto fail;
	}

	/* Get the res-io-width */
	if (OF_getencprop(node, "reg-io-width", &sc->reg_width,
	    sizeof(uint32_t)) <= 0)
		sc->reg_width = 1;

	/* Get and init the phy */
	if (OF_hasprop(node, "phys")) {
		if (OF_getencprop(node, "phys", &phy, sizeof(phy)) == -1) {
			device_printf(dev, "Cannot get the phys property\n");
			error = ENXIO;
			goto fail;
		}
		sc->phydev = OF_device_from_xref(phy);
		if (sc->phydev == NULL) {
			device_printf(dev, "Cannot get the phy device\n");
			error = ENXIO;
			goto fail;
		}
		DW_HDMI_PHY_INIT(sc->phydev);
	} else {
		/* Use internal phy */
	}

	/* Register ourself */
	OF_device_register_xref(OF_xref_from_node(node), dev);

	if (bootverbose) {
		version = dw_hdmi_read(sc, DW_HDMI_DESIGN_ID) << 8;
		version |= dw_hdmi_read(sc, DW_HDMI_REVISION_ID);
		if (bootverbose) {
			device_printf(dev, "Version: %x\n", version);
			device_printf(dev, "Product ID0: %x, Product ID1: %x\n",
			    dw_hdmi_read(sc, DW_HDMI_PRODUCT_ID0),
			    dw_hdmi_read(sc, DW_HDMI_PRODUCT_ID1));
		}
	}

	dw_hdmi_i2cm_init(sc);

	/* Disable interrupts */
	dw_hdmi_write(sc, DW_HDMI_IH_FC_STAT0, 0xFF);
	dw_hdmi_write(sc, DW_HDMI_IH_FC_STAT1, 0xFF);
	dw_hdmi_write(sc, DW_HDMI_IH_FC_STAT2, 0xFF);
	dw_hdmi_write(sc, DW_HDMI_IH_AS_STAT0, 0xFF);
	dw_hdmi_write(sc, DW_HDMI_IH_PHY_STAT0, 0xFF);
	dw_hdmi_write(sc, DW_HDMI_IH_I2CM_STAT0, 0xFF);
	dw_hdmi_write(sc, DW_HDMI_IH_CEC_STAT0, 0xFF);
	dw_hdmi_write(sc, DW_HDMI_IH_VP_STAT0, 0xFF);
	dw_hdmi_write(sc, DW_HDMI_IH_I2CMPHY_STAT0, 0xFF);
	dw_hdmi_write(sc, DW_HDMI_IH_AHBDMAAUD_STAT0, 0xFF);

	/* Mute interrupts*/
	dw_hdmi_write(sc, DW_HDMI_IH_MUTE_FC_STAT0,
	  0xFF);
	dw_hdmi_write(sc, DW_HDMI_IH_MUTE_FC_STAT1,
	  0xFF);
	dw_hdmi_write(sc, DW_HDMI_IH_MUTE_FC_STAT2,
	  0xFF);
	dw_hdmi_write(sc, DW_HDMI_IH_MUTE_AS_STAT0,
	  0xFF);
	dw_hdmi_write(sc, DW_HDMI_IH_MUTE_PHY_STAT0,
	  0xFF);
	dw_hdmi_write(sc, DW_HDMI_IH_MUTE_I2CM_STAT0,
	  0xFF);
	dw_hdmi_write(sc, DW_HDMI_IH_MUTE_CEC_STAT0,
	  0xFF);
	dw_hdmi_write(sc, DW_HDMI_IH_MUTE_VP_STAT0,
	  0xFF);
	dw_hdmi_write(sc, DW_HDMI_IH_MUTE_I2CMPHY_STAT0,
	  0xFF);
	dw_hdmi_write(sc, DW_HDMI_IH_MUTE_AHBDMAAUD_STAT0,
	  0xFF);

	/* Unmute global interrupts */
	dw_hdmi_write(sc, DW_HDMI_IH_MUTE,
	  ~(DW_HDMI_IH_MUTE_ALL |
	    DW_HDMI_IH_MUTE_WAKEUP));

	/* If no ddc is provided by the driver use the internal one */
	if (sc->ddc == NULL) {
		if ((sc->iicbus = device_add_child(dev, "iicbus", -1)) == NULL){
			device_printf(dev,
			    "could not allocate iicbus instance\n");
			return (ENXIO);
		}
		sc->ddc = i2c_bsd_adapter(sc->iicbus);
	}

	return (0);
fail:
	return (error);
}

static int
dw_hdmi_detach(device_t dev)
{
	struct dw_hdmi_softc *sc;

	sc = device_get_softc(dev);

	bus_release_resources(dev, dw_hdmi_spec, sc->res);
	mtx_destroy(&sc->mtx);

	return (0);
}

/* Allwinner */

static int
aw_de2_dw_hdmi_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, aw_compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Allwinner SUN8I DW HDMI");
	return (BUS_PROBE_DEFAULT);
}

static int
aw_de2_dw_hdmi_attach(device_t dev)
{
	struct aw_dw_hdmi_softc *sc;
	phandle_t node;
	int error;

	sc = device_get_softc(dev);

	node = ofw_bus_get_node(dev);

	if ((error = clk_get_by_ofw_name(dev, node, "tmds",
	    &sc->clk_tmds)) != 0) {
		device_printf(dev, "Cannot get tmds clock\n");
		goto fail;
	}
	if (clk_enable(sc->clk_tmds) != 0) {
		device_printf(dev, "Cannot enable tmds clock\n");
		goto fail;
	}
	if ((error = hwreset_get_by_ofw_name(dev, node, "ctrl",
	    &sc->reset_ctrl)) != 0) {
		device_printf(dev, "Cannot get reset\n");
		goto fail;
	}
	if (hwreset_deassert(sc->reset_ctrl) != 0) {
		device_printf(dev, "Cannot deassert reset\n");
		goto fail;
	}

	error = dw_hdmi_attach(dev);
	if (error != 0)
		goto fail;
	return (0);

fail:
	aw_de2_dw_hdmi_detach(dev);
	return (error);
}

static int
aw_de2_dw_hdmi_detach(device_t dev)
{

	dw_hdmi_detach(dev);
	return (0);
}

static device_method_t aw_de2_dw_hdmi_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		aw_de2_dw_hdmi_probe),
	DEVMETHOD(device_attach,	aw_de2_dw_hdmi_attach),
	DEVMETHOD(device_detach,	aw_de2_dw_hdmi_detach),

	/* iicbus interface */
	DEVMETHOD(iicbus_transfer,	dw_hdmi_transfer),

	/* DW_HDMI interface */
	DEVMETHOD(dw_hdmi_add_encoder,	aw_dw_hdmi_add_encoder),

	DEVMETHOD_END
};

static driver_t aw_de2_dw_hdmi_driver = {
	"aw_de2_dw_hdmi",
	aw_de2_dw_hdmi_methods,
	sizeof(struct aw_dw_hdmi_softc),
};

static devclass_t aw_de2_dw_hdmi_devclass;

EARLY_DRIVER_MODULE(aw_de2_dw_hdmi, simplebus, aw_de2_dw_hdmi_driver,
  aw_de2_dw_hdmi_devclass, 0, 0, BUS_PASS_SUPPORTDEV + BUS_PASS_ORDER_EARLY);
MODULE_VERSION(aw_de2_dw_hdmi, 1);
MODULE_DEPEND(aw_de2_dw_hdmi, aw_de2_hdmi_phy, 1, 1, 1);

/* Rockchip */

static int
rk_dw_hdmi_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, rk_compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "RockChip DW HDMI");
	return (BUS_PROBE_DEFAULT);
}

static int
rk_dw_hdmi_attach(device_t dev)
{
	struct rk_dw_hdmi_softc *sc;
	phandle_t node;
	int error;

	sc = device_get_softc(dev);

	node = ofw_bus_get_node(dev);

	error = dw_hdmi_attach(dev);
	if (error != 0)
		goto fail;
	return (0);

fail:
	rk_dw_hdmi_detach(dev);
	return (error);
}

static int
rk_dw_hdmi_detach(device_t dev)
{

	dw_hdmi_detach(dev);
	return (0);
}

static device_method_t rk_dw_hdmi_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		rk_dw_hdmi_probe),
	DEVMETHOD(device_attach,	rk_dw_hdmi_attach),
	DEVMETHOD(device_detach,	rk_dw_hdmi_detach),

	/* iicbus interface */
	DEVMETHOD(iicbus_transfer,	dw_hdmi_transfer),

	/* DW_HDMI interface */
	DEVMETHOD(dw_hdmi_add_encoder,	rk_dw_hdmi_add_encoder),

	DEVMETHOD_END
};

static driver_t rk_dw_hdmi_driver = {
	"rk_dw_hdmi",
	rk_dw_hdmi_methods,
	sizeof(struct rk_dw_hdmi_softc),
};

static devclass_t rk_dw_hdmi_devclass;

EARLY_DRIVER_MODULE(rk_dw_hdmi, simplebus, rk_dw_hdmi_driver,
  rk_dw_hdmi_devclass, 0, 0, BUS_PASS_SUPPORTDEV + BUS_PASS_ORDER_EARLY);
MODULE_VERSION(rk_dw_hdmi, 1);
