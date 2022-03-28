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

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/intr.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/mutex.h>

#include <dev/extres/syscon/syscon.h>
#include <dev/iicbus/iiconf.h>

#include "syscon_if.h"
#include "iicbus_if.h"
#include "anx_dp.h"

#define	ANXDP_WRITE(sc, reg, val) bus_write_4((sc)->res[0], (reg), (val))
#define	ANXDP_READ(sc, reg) bus_read_4((sc)->res[0], (reg))


static void anxdp_init_aux(struct anxdp_softc * sc);


static void edp_connector_destroy(struct drm_connector *connector);
static void anxdp_bringup(struct anxdp_softc * const sc);


static void anxdp_init_hpd(struct anxdp_softc * const sc);
static inline const bool isrockchip(struct anxdp_softc * const sc);
static ssize_t anxdp_dp_aux_transfer(struct drm_dp_aux *dpaux, struct drm_dp_aux_msg *dpmsg);
static void anxdp_analog_power_up_all(struct anxdp_softc * const sc);
static void anxdp_init_hpd(struct anxdp_softc * const sc);
static int anxdp_await_pll_lock(struct anxdp_softc * const sc);
static void edp_bridge_enable(struct drm_bridge *bridge);
static void edp_macro_reset(struct anxdp_softc * const sc);
static void edp_bridge_mode_set(struct drm_bridge *bridge, const struct drm_display_mode *mode, const struct drm_display_mode *adjusted_mode);
static bool edp_bridge_mode_fixup(struct drm_bridge *bridge, const struct drm_display_mode *mode, struct drm_display_mode *adjusted_mode);
static void edp_bridge_pre_enable(struct drm_bridge *bridge);
static void edp_bridge_post_disable(struct drm_bridge *bridge);
static int edp_bridge_attach(struct drm_bridge *bridge, enum drm_bridge_attach_flags flags);
static void edp_bridge_disable(struct drm_bridge *bridge);
static void edp_train_link(struct anxdp_softc * const sc);

static const struct drm_bridge_funcs edp_bridge_funcs = {
	.attach = edp_bridge_attach,
	.enable = edp_bridge_enable,
	.pre_enable = edp_bridge_pre_enable,
	.disable = edp_bridge_disable,
	.post_disable = edp_bridge_post_disable,
	.mode_set = edp_bridge_mode_set,
	.mode_fixup = edp_bridge_mode_fixup,

};

static int anxdp_await_pll_lock(struct anxdp_softc * const sc)
{
	u_int timeout;

	for (timeout = 0; timeout < 100; timeout++) {
		if ((ANXDP_READ(sc,ANXDP_DEBUG_CTL) &
			PLL_LOCK) != 0)
			return 0;
		DELAY(20);
	}

	return ETIMEDOUT;

}


#define DP_LINK_CAP_ENHANCED_FRAMING (1 << 0)
struct drm_dp_link {
	unsigned char revision;
	unsigned int rate;
	unsigned int num_lanes;
	unsigned long capabilities;
};


static enum drm_connector_status
edp_connector_detect(struct drm_connector *connector, bool force)
{
#if 0
	struct anxdp_connector *anxdp_connector = to_anxdp_connector(connector);
	struct anxdp_softc * const sc = anxdp_connector->sc;

	/* XXX HPD */
#endif
	return connector_status_connected;
}


static int edp_connector_get_modes(struct drm_connector *connector)
{
	int error;
	struct edid *pedid = NULL;

	pedid = drm_get_edid(connector, connector->ddc);

	if (pedid == NULL)
		return 0;

	error = drm_add_edid_modes(connector, pedid);
	if (pedid != NULL)
		kfree(pedid);

	return error;
}


static const struct drm_connector_funcs edp_connector_funcs = {
	.detect = edp_connector_detect,
	.dpms = drm_helper_connector_dpms,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = edp_connector_destroy,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
	//	.late_register = edp_connector_late_register,
};
static const struct drm_connector_helper_funcs edp_connector_helper_funcs = {
	.get_modes = edp_connector_get_modes,
};
void
anxdp_add_bridge(struct anxdp_softc *sc,struct drm_encoder *encoder)
{
	sc->sc_bridge.funcs = &edp_bridge_funcs;
	drm_bridge_attach(encoder, &sc->sc_bridge, NULL,0);
}


static int
edp_bridge_attach(struct drm_bridge *bridge, enum drm_bridge_attach_flags flags)
{
	struct anxdp_softc *sc;
	sc = container_of(bridge, struct anxdp_softc, sc_bridge);

	sc->sc_connector.polled =    DRM_CONNECTOR_POLL_CONNECT | DRM_CONNECTOR_POLL_DISCONNECT;
	sc->sc_connector.interlace_allowed = 0;
	sc->sc_connector.doublescan_allowed = 0;

	drm_connector_helper_add(&sc->sc_connector,
	    &edp_connector_helper_funcs);

	drm_connector_init_with_ddc(bridge->dev,&sc->sc_connector,
	    &edp_connector_funcs, DRM_MODE_CONNECTOR_eDP,&sc->sc_dpaux.ddc);

	drm_connector_attach_encoder(&sc->sc_connector, &sc->sc_encoder);

	return (0);
}





static void
edp_bridge_enable(struct drm_bridge *bridge)
{

	struct anxdp_softc * sc = container_of(bridge,struct anxdp_softc,sc_bridge);

	uint32_t val;

	val = ANXDP_READ(sc,ANXDP_FUNC_EN_1);
	if (isrockchip(sc)) {
		val &= ~(RK_VID_CAP_FUNC_EN_N | RK_VID_FIFO_FUNC_EN_N);
	} else {
		val &= ~(MASTER_VID_FUNC_EN_N | SLAVE_VID_FUNC_EN_N);
		val |= MASTER_VID_FUNC_EN_N;
	}
	ANXDP_WRITE(sc, ANXDP_FUNC_EN_1, val);

	val = ANXDP_READ(sc,ANXDP_VIDEO_CTL_10);
	val &= ~(SLAVE_I_SCAN_CFG|SLAVE_VSYNC_P_CFG|SLAVE_HSYNC_P_CFG);
	if ((sc->sc_curmode.flags & DRM_MODE_FLAG_INTERLACE) != 0)
		val |= SLAVE_I_SCAN_CFG;
	if ((sc->sc_curmode.flags & DRM_MODE_FLAG_NVSYNC) != 0)
		val |= SLAVE_VSYNC_P_CFG;
	if ((sc->sc_curmode.flags & DRM_MODE_FLAG_NHSYNC) != 0)
		val |= SLAVE_HSYNC_P_CFG;
	ANXDP_WRITE(sc, ANXDP_VIDEO_CTL_10, val);

	ANXDP_WRITE(sc, ANXDP_SOC_GENERAL_CTL,
	    AUDIO_MODE_SPDIF_MODE | VIDEO_MODE_SLAVE_MODE);
	edp_train_link(sc);

	val = ANXDP_READ(sc,ANXDP_VIDEO_CTL_1);
	val |= VIDEO_EN;
	ANXDP_WRITE(sc, ANXDP_VIDEO_CTL_1, val);

	if (sc->sc_panel != NULL &&
	    sc->sc_panel->funcs != NULL &&
	    sc->sc_panel->funcs->enable != NULL)
		sc->sc_panel->funcs->enable(sc->sc_panel);
#if ANXDP_AUDIO // Audio is not tested on FreeBSD.
	ANXDP_READ(sc,
	    if (sc->sc_connector.monitor_audio)
		    anxdp_audio_init(sc);
#endif
	    }


static void
edp_connector_destroy(struct drm_connector *connector)
{
  drm_connector_unregister(connector);
  drm_connector_cleanup(connector);
}

static void edp_bridge_pre_enable(struct drm_bridge *bridge)
{
}

static void
edp_bridge_disable(struct drm_bridge *bridge)
{
}



static void edp_bridge_post_disable(struct drm_bridge *bridge)
{
}


static void
edp_bridge_mode_set(struct drm_bridge *bridge,
                    const struct drm_display_mode *mode, const struct drm_display_mode *adjusted_mode)
{
	struct anxdp_softc * sc = container_of(bridge,struct anxdp_softc,sc_bridge);

	memcpy(&sc->sc_curmode, mode, sizeof(struct drm_display_mode));

}

static bool
edp_bridge_mode_fixup(struct drm_bridge *bridge, const struct drm_display_mode *mode, struct drm_display_mode *adjusted_mode)
{
	return true;
}

static void edp_macro_reset(struct anxdp_softc * const sc)
{
	uint32_t val;

	val = ANXDP_READ(sc,ANXDP_PHY_TEST);
	val |= MACRO_RST;
	ANXDP_WRITE(sc, ANXDP_PHY_TEST, val);
	DELAY(10);
	val &= ~MACRO_RST;
	ANXDP_WRITE(sc, ANXDP_PHY_TEST, val);
}

static void
edp_link_start(struct anxdp_softc * const sc, struct drm_dp_link * const link)
{
	uint8_t training[4];
	uint32_t val;
	u8 values[2];
	int err;
	ANXDP_WRITE(sc, ANXDP_LINK_BW_SET, drm_dp_link_rate_to_bw_code(link->rate));
	ANXDP_WRITE(sc, ANXDP_LANE_COUNT_SET, link->num_lanes);
	values[0] = drm_dp_link_rate_to_bw_code(link->rate);
	values[1] = link->num_lanes;

	if (link->capabilities & DP_LINK_CAP_ENHANCED_FRAMING)
		values[1] |= DP_LANE_COUNT_ENHANCED_FRAME_EN;



	err = drm_dp_dpcd_write(&sc->sc_dpaux, DP_LINK_BW_SET, values, sizeof(values));
	if (err < 0) {
		return;
	}

	for (u_int i = 0; i < link->num_lanes; i++) {
		val = ANXDP_READ(sc,
		    ANXDP_LNx_LINK_TRAINING_CTL(i));
		val &= ~(PRE_EMPHASIS_SET(3)|DRIVE_CURRENT_SET(3));
		val |= PRE_EMPHASIS_SET(0);
		ANXDP_WRITE(sc,
		    ANXDP_LNx_LINK_TRAINING_CTL(i), val);
	}

	if (anxdp_await_pll_lock(sc) != 0) {
		printf("PLL lock timeout\n");
	}


	for (u_int i = 0; i < link->num_lanes; i++) {
		training[i] = DP_TRAIN_PRE_EMPH_LEVEL_0 |
			DP_TRAIN_VOLTAGE_SWING_LEVEL_0;
	}

	drm_dp_dpcd_write(&sc->sc_dpaux, DP_TRAINING_LANE0_SET, training,
	    link->num_lanes);
}

static void
edp_process_clock_recovery(struct anxdp_softc * const sc, struct drm_dp_link * const link)
{
	u_int i, tries;
	uint8_t link_status[DP_LINK_STATUS_SIZE];
	uint8_t training[4];

	ANXDP_WRITE(sc, ANXDP_TRAINING_PTN_SET,
	    SCRAMBLING_DISABLE | SW_TRAINING_PATTERN_SET_PTN1);
	drm_dp_dpcd_writeb(&sc->sc_dpaux, DP_TRAINING_PATTERN_SET,
	    DP_LINK_SCRAMBLING_DISABLE | DP_TRAINING_PATTERN_1);

	tries = 0;
	again:
	if (tries++ >= 10) {
		device_printf(sc->sc_dev, "cr fail\n");
		return;
	}
	drm_dp_link_train_clock_recovery_delay(sc->sc_dpcd);
	if (DP_LINK_STATUS_SIZE !=
	    drm_dp_dpcd_read_link_status(&sc->sc_dpaux, link_status)) {
		return;
	}
	if (!drm_dp_clock_recovery_ok(link_status, link->num_lanes)) {
		goto cr_fail;
	}

	return;

cr_fail:
	for (i = 0; i < link->num_lanes; i++) {
		uint8_t vs, pe;
		vs = drm_dp_get_adjust_request_voltage(link_status, i);
		pe = drm_dp_get_adjust_request_pre_emphasis(link_status, i);
		training[i] = vs | pe;
	}
	for (i = 0; i < link->num_lanes; i++) {
		ANXDP_WRITE(sc,
		    ANXDP_LNx_LINK_TRAINING_CTL(i), training[i]);
	}
	drm_dp_dpcd_write(&sc->sc_dpaux, DP_TRAINING_LANE0_SET, training,
	    link->num_lanes);
	goto again;
}

static void
edp_process_eq(struct anxdp_softc * const sc, struct drm_dp_link * const link)
{
	u_int i, tries;
	uint8_t link_status[DP_LINK_STATUS_SIZE];
	uint8_t training[4];

	ANXDP_WRITE(sc, ANXDP_TRAINING_PTN_SET,
	    SCRAMBLING_DISABLE | SW_TRAINING_PATTERN_SET_PTN2);
	drm_dp_dpcd_writeb(&sc->sc_dpaux, DP_TRAINING_PATTERN_SET,
	    DP_LINK_SCRAMBLING_DISABLE | DP_TRAINING_PATTERN_2);

	tries = 0;
again:
	if (tries++ >= 10) {
		device_printf(sc->sc_dev, "eq fail\n");
		return;
	}
	drm_dp_link_train_channel_eq_delay(sc->sc_dpcd);
	if (DP_LINK_STATUS_SIZE !=
	    drm_dp_dpcd_read_link_status(&sc->sc_dpaux, link_status)) {
		return;
	}
	if (!drm_dp_channel_eq_ok(link_status, link->num_lanes)) {
		goto eq_fail;
	}

	return;

eq_fail:
	for (i = 0; i < link->num_lanes; i++) {
		uint8_t vs, pe;
		vs = drm_dp_get_adjust_request_voltage(link_status, i);
		pe = drm_dp_get_adjust_request_pre_emphasis(link_status, i);
		training[i] = vs | pe;
	}
	for (i = 0; i < link->num_lanes; i++) {
		ANXDP_WRITE(sc,
		    ANXDP_LNx_LINK_TRAINING_CTL(i), training[i]);
	}
	drm_dp_dpcd_write(&sc->sc_dpaux, DP_TRAINING_LANE0_SET, training,
	    link->num_lanes);
	goto again;
}
static void
edp_train_link(struct anxdp_softc * const sc)
{
	int err;
	u8 values[3];
	u8 value;
	struct drm_dp_link link;

	edp_macro_reset(sc);
	drm_dp_dpcd_read(&sc->sc_dpaux, DP_DPCD_REV, values, sizeof(values));

	link.revision = values[0];
	link.rate = drm_dp_bw_code_to_link_rate(values[1]);
	link.num_lanes = values[2] & DP_MAX_LANE_COUNT_MASK;

	if (values[2] & DP_ENHANCED_FRAME_CAP)
		link.capabilities |= DP_LINK_CAP_ENHANCED_FRAMING;


	/* DP_SET_POWER register is only available on DPCD v1.1 and later */
	if (link.revision < 0x11)
		return;

	err = drm_dp_dpcd_readb(&sc->sc_dpaux, DP_SET_POWER, &value);
	if (err <0)
		return;

	value &= ~DP_SET_POWER_MASK;
	value |= DP_SET_POWER_D0;

	drm_dp_dpcd_writeb(&sc->sc_dpaux, DP_SET_POWER, value);


	/*
	 * According to the DP 1.1 specification, a "Sink Device must exit the
	 * power saving state within 1 ms" (Section 2.5.3.1, Table 5-52, "Sink
	 * Control Field" (register 0x600).
	 */
	usleep_range(10040, 2000);

	if (DP_RECEIVER_CAP_SIZE != drm_dp_dpcd_read(&sc->sc_dpaux, DP_DPCD_REV,
		sc->sc_dpcd, DP_RECEIVER_CAP_SIZE))
		return;

	edp_link_start(sc, &link);
	edp_process_clock_recovery(sc, &link);
	edp_process_eq(sc, &link);

	ANXDP_WRITE(sc, ANXDP_TRAINING_PTN_SET, 0);
	drm_dp_dpcd_writeb(&sc->sc_dpaux, DP_TRAINING_PATTERN_SET,
	    DP_TRAINING_PATTERN_DISABLE);
}


static void
anxdp_init_hpd(struct anxdp_softc * const sc)
{
	uint32_t sc3;

	ANXDP_WRITE(sc, ANXDP_COMMON_INT_STA_4, 0x7);
	ANXDP_WRITE(sc, ANXDP_DP_INT_STA, INT_HPD);

	sc3 = ANXDP_READ(sc,ANXDP_SYS_CTL_3);
	sc3 &= ~(F_HPD | HPD_CTRL);
	ANXDP_WRITE(sc, ANXDP_SYS_CTL_3, sc3);

	sc3 = ANXDP_READ(sc,ANXDP_SYS_CTL_3);
	sc3 |= F_HPD | HPD_CTRL;
	ANXDP_WRITE(sc, ANXDP_SYS_CTL_3, sc3);
}

static void
anxdp_analog_power_up_all(struct anxdp_softc * const sc)
{
	const bus_size_t pd_reg = isrockchip(sc) ? RKANXDP_PD : ANXDP_PHY_PD;

	ANXDP_WRITE(sc, pd_reg, DP_ALL_PD);
	DELAY(15);
	ANXDP_WRITE(sc, pd_reg,
	    DP_ALL_PD & ~DP_INC_BG);
	DELAY(15);
	ANXDP_WRITE(sc, pd_reg, 0);
}


static inline const bool
isrockchip(struct anxdp_softc * const sc)
{
	return (sc->sc_flags & ANXDP_FLAG_ROCKCHIP) != 0;
}

static ssize_t
anxdp_dp_aux_transfer(struct drm_dp_aux *dpaux, struct drm_dp_aux_msg *dpmsg)
{
	struct anxdp_softc * const sc = container_of(dpaux, struct anxdp_softc,sc_dpaux);
	size_t loop_timeout = 0;
	uint32_t val;
	size_t i;
	ssize_t ret = 0;

	ANXDP_WRITE(sc, ANXDP_BUFFER_DATA_CTL,BUF_CLR);
	val = AUX_LENGTH(dpmsg->size);
	if ((dpmsg->request & DP_AUX_I2C_MOT) != 0)
		val |= AUX_TX_COMM_MOT;

	switch (dpmsg->request & ~DP_AUX_I2C_MOT) {
	case DP_AUX_I2C_WRITE:
		break;
	case DP_AUX_I2C_READ:
		val |= AUX_TX_COMM_READ;
		break;
	case DP_AUX_NATIVE_WRITE:
		val |= AUX_TX_COMM_DP;
		break;
	case DP_AUX_NATIVE_READ:
		val |= AUX_TX_COMM_READ | AUX_TX_COMM_DP;
		break;
	}

	ANXDP_WRITE(sc, ANXDP_AUX_CH_CTL_1, val);
	ANXDP_WRITE(sc, ANXDP_AUX_ADDR_7_0,
	    AUX_ADDR_7_0(dpmsg->address));
	ANXDP_WRITE(sc, ANXDP_AUX_ADDR_15_8,
	    AUX_ADDR_15_8(dpmsg->address));
	ANXDP_WRITE(sc, ANXDP_AUX_ADDR_19_16,
	    AUX_ADDR_19_16(dpmsg->address));

	if (!(dpmsg->request & DP_AUX_I2C_READ)) {
		for (i = 0; i < dpmsg->size; i++) {
			ANXDP_WRITE(sc,
			    ANXDP_BUF_DATA(i),
			    ((const uint8_t *)(dpmsg->buffer))[i]);
			ret++;
		}
	}


	ANXDP_WRITE(sc, ANXDP_AUX_CH_CTL_2,
	    AUX_EN | ((dpmsg->size == 0) ? ADDR_ONLY : 0));

	loop_timeout = 0;
	val = ANXDP_READ(sc,ANXDP_AUX_CH_CTL_2);
	while ((val & AUX_EN) != 0) {
		if (++loop_timeout > 20000) {
			ret = -ETIMEDOUT;
			goto out;
		}
		DELAY(25);
		val = ANXDP_READ(sc,
		    ANXDP_AUX_CH_CTL_2);
	}

	loop_timeout = 0;
	val = ANXDP_READ(sc,ANXDP_DP_INT_STA);
	while (!(val & RPLY_RECEIV)) {
		if (++loop_timeout > 2000) {
			ret = -ETIMEDOUT;
			goto out;
		}
		DELAY(10);
		val = ANXDP_READ(sc,
		    ANXDP_DP_INT_STA);
	}

	ANXDP_WRITE(sc, ANXDP_DP_INT_STA,
	    RPLY_RECEIV);

	val = ANXDP_READ(sc,ANXDP_DP_INT_STA);
	if ((val & AUX_ERR) != 0) {
		ANXDP_WRITE(sc, ANXDP_DP_INT_STA,
		    AUX_ERR);
		ret = -EREMOTEIO;
		goto out;
	}

	val = ANXDP_READ(sc,ANXDP_AUX_CH_STA);
	if (AUX_STATUS(val) != 0) {
		ret = -EREMOTEIO;
		goto out;
	}

	if ((dpmsg->request & DP_AUX_I2C_READ)) {
		for (i = 0; i < dpmsg->size; i++) {
			val = ANXDP_READ(sc,
			    ANXDP_BUF_DATA(i));
			((uint8_t *)(dpmsg->buffer))[i] = val & 0xffU;
			ret++;
		}
	}

	val = ANXDP_READ(sc,ANXDP_AUX_RX_COMM);
	if (val == AUX_RX_COMM_AUX_DEFER)
		dpmsg->reply = DP_AUX_NATIVE_REPLY_DEFER;
	else if (val == AUX_RX_COMM_I2C_DEFER)
		dpmsg->reply = DP_AUX_I2C_REPLY_DEFER;
	else if ((dpmsg->request & ~DP_AUX_I2C_MOT) == DP_AUX_I2C_WRITE ||
	    (dpmsg->request & ~DP_AUX_I2C_MOT) == DP_AUX_I2C_READ)
		dpmsg->reply = DP_AUX_I2C_REPLY_ACK;
	else if ((dpmsg->request & ~DP_AUX_I2C_MOT) == DP_AUX_NATIVE_WRITE ||
	    (dpmsg->request & ~DP_AUX_I2C_MOT) == DP_AUX_NATIVE_READ)
		dpmsg->reply = DP_AUX_NATIVE_REPLY_ACK;

out:
	if (ret < 0)
		anxdp_init_aux(sc);
	return ret;

}
int
anxdp_attach(struct anxdp_softc *sc)
{
	phandle_t node;
	node = ofw_bus_get_node(sc->sc_dev);

	sc->sc_dpaux.name = "DP Aux";
	sc->sc_dpaux.dev = sc->sc_dev;
	sc->sc_dpaux.transfer=anxdp_dp_aux_transfer;

	if (drm_dp_aux_register(&sc->sc_dpaux) != 0) {
		device_printf(sc->sc_dev, "registering DP Aux failed\n");
	}
	OF_device_register_xref(OF_xref_from_node(node),sc->sc_dev);
	anxdp_bringup(sc);
	return 0;
}
static void
anxdp_bringup(struct anxdp_softc * const sc)
{
	uint32_t val;

	val = ANXDP_READ(sc,ANXDP_VIDEO_CTL_1);
	val &= ~VIDEO_EN;
	ANXDP_WRITE(sc, ANXDP_VIDEO_CTL_1, val);

	val = ANXDP_READ(sc,ANXDP_VIDEO_CTL_1);
	val &= ~VIDEO_MUTE;
	ANXDP_WRITE(sc, ANXDP_VIDEO_CTL_1, val);

	val = SW_FUNC_EN_N;
	if (isrockchip(sc)) {
		val |= RK_VID_CAP_FUNC_EN_N | RK_VID_FIFO_FUNC_EN_N;
	} else {
		val |= MASTER_VID_FUNC_EN_N | SLAVE_VID_FUNC_EN_N |
			AUD_FIFO_FUNC_EN_N | AUD_FUNC_EN_N | HDCP_FUNC_EN_N;
	}
	ANXDP_WRITE(sc, ANXDP_FUNC_EN_1, val);

	ANXDP_WRITE(sc, ANXDP_FUNC_EN_2,
	    SSC_FUNC_EN_N | AUX_FUNC_EN_N | SERDES_FIFO_FUNC_EN_N |
	    LS_CLK_DOMAIN_FUNC_EN_N);

	DELAY(30);

	ANXDP_WRITE(sc, ANXDP_M_AUD_GEN_FILTER_TH, 2);
	ANXDP_WRITE(sc, ANXDP_SOC_GENERAL_CTL, 0x101);

	ANXDP_WRITE(sc, ANXDP_TX_SW_RESET,RESET_DP_TX);

	ANXDP_WRITE(sc, ANXDP_ANALOG_CTL_1,
	    TX_TERMINAL_CTRL_50_OHM);
	ANXDP_WRITE(sc, ANXDP_ANALOG_CTL_2,
	    SEL_24M | TX_DVDD_BIT_1_0625V);
	if (isrockchip(sc)) {
		ANXDP_WRITE(sc, ANXDP_PLL_REG_1, REF_CLK_24M);
		ANXDP_WRITE(sc, ANXDP_PLL_REG_2, 0x95);
		ANXDP_WRITE(sc, ANXDP_PLL_REG_3, 0x40);
		ANXDP_WRITE(sc, ANXDP_PLL_REG_4, 0x58);
		ANXDP_WRITE(sc, ANXDP_PLL_REG_5, 0x22);
	}
	ANXDP_WRITE(sc, ANXDP_ANALOG_CTL_3,
	    DRIVE_DVDD_BIT_1_0625V | VCO_BIT_600_MICRO);
	ANXDP_WRITE(sc, ANXDP_PLL_FILTER_CTL_1,
	    PD_RING_OSC | AUX_TERMINAL_CTRL_50_OHM | TX_CUR1_2X | TX_CUR_16_MA);
	ANXDP_WRITE(sc, ANXDP_TX_AMP_TUNING_CTL, 0);

	val = ANXDP_READ(sc,ANXDP_FUNC_EN_1);
	val &= ~SW_FUNC_EN_N;
	ANXDP_WRITE(sc, ANXDP_FUNC_EN_1, val);
	anxdp_analog_power_up_all(sc);
	ANXDP_WRITE(sc, ANXDP_COMMON_INT_STA_1,
	    PLL_LOCK_CHG);

	val = ANXDP_READ(sc,ANXDP_DEBUG_CTL);
	val &= ~(F_PLL_LOCK | PLL_LOCK_CTRL);
	ANXDP_WRITE(sc, ANXDP_DEBUG_CTL, val);

	if (anxdp_await_pll_lock(sc) != 0) {
		device_printf(sc->sc_dev, "PLL lock timeout\n");
	}

	val = ANXDP_READ(sc,ANXDP_FUNC_EN_2);
	val &= ~(SERDES_FIFO_FUNC_EN_N | LS_CLK_DOMAIN_FUNC_EN_N |
	    AUX_FUNC_EN_N);
	ANXDP_WRITE(sc, ANXDP_FUNC_EN_2, val);

	anxdp_init_hpd(sc);
	anxdp_init_aux(sc);
}



void
anxdp_init_aux(struct anxdp_softc *sc)
{
	uint32_t fe2, pd, hrc;
	const bus_size_t pd_reg = isrockchip(sc) ? RKANXDP_PD : ANXDP_PHY_PD;
	const uint32_t pd_mask = isrockchip(sc) ? RK_AUX_PD : AUX_PD;

	ANXDP_WRITE(sc, ANXDP_DP_INT_STA,
	    RPLY_RECEIV | AUX_ERR);

	pd = ANXDP_READ(sc,pd_reg);
	pd |= pd_mask;
	ANXDP_WRITE(sc, pd_reg, pd);

	DELAY(11);

	pd = ANXDP_READ(sc,pd_reg);
	pd &= ~pd_mask;
	ANXDP_WRITE(sc, pd_reg, pd);

	fe2 = ANXDP_READ(sc,ANXDP_FUNC_EN_2);
	fe2 |= AUX_FUNC_EN_N;
	ANXDP_WRITE(sc, ANXDP_FUNC_EN_2, fe2);

	hrc = AUX_HW_RETRY_COUNT_SEL(0) | AUX_HW_RETRY_INTERVAL_600_US;
	if (!isrockchip(sc))
		hrc |= AUX_BIT_PERIOD_EXPECTED_DELAY(3);
	ANXDP_WRITE(sc, ANXDP_AUX_HW_RETRY_CTL, hrc);

	ANXDP_WRITE(sc, ANXDP_AUX_CH_DEFER_CTL,
	    DEFER_CTRL_EN | DEFER_COUNT(1));

	fe2 = ANXDP_READ(sc,ANXDP_FUNC_EN_2);
	fe2 &= ~AUX_FUNC_EN_N;
	ANXDP_WRITE(sc, ANXDP_FUNC_EN_2, fe2);

}

static device_method_t
anxdp_methods[] = {
};


DEFINE_CLASS_0(anxdp, anxdp_driver,anxdp_methods,sizeof(struct anxdp_softc));


