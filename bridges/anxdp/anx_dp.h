
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


#ifndef __ANXEDP_H__
#define	__ANXEDP_H__
#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#undef CONFIG_DRM_DP_CEC
#include <drm/drm_crtc_helper.h>
#include <drm/drm_dp_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_dp_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_panel.h>
#include <drm/drm_connector.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_edid.h>
#include <linux/bitops.h>
#define __BITS(hi,lo)	((~((~0)<<((hi)+1)))&((~0)<<(lo)))
#define __BIT BIT
#define __LOWEST_SET_BIT(__mask) ((((__mask) - 1) & (__mask)) ^ (__mask))
#define __SHIFTOUT(__x, __mask) (((__x) & (__mask)) / __LOWEST_SET_BIT(__mask))
#define __SHIFTIN(__x, __mask) ((__x) * __LOWEST_SET_BIT(__mask))
#define ANXDP_AUDIO 0
#define	ANXDP_DP_TX_VERSION	0x010
#define	ANXDP_TX_SW_RESET	0x014
#define	 RESET_DP_TX			__BIT(0)
#define	ANXDP_FUNC_EN_1		0x018
#define	 MASTER_VID_FUNC_EN_N		__BIT(7)
#define	 RK_VID_CAP_FUNC_EN_N		__BIT(6)
#define	 SLAVE_VID_FUNC_EN_N		__BIT(5)
#define	 RK_VID_FIFO_FUNC_EN_N		__BIT(5)
#define	 AUD_FIFO_FUNC_EN_N		__BIT(4)
#define	 AUD_FUNC_EN_N			__BIT(3)
#define	 HDCP_FUNC_EN_N			__BIT(2)
#define	 CRC_FUNC_EN_N			__BIT(1)
#define	 SW_FUNC_EN_N			__BIT(0)
#define	ANXDP_FUNC_EN_2		0x01c
#define	 SSC_FUNC_EN_N			__BIT(7)
#define	 AUX_FUNC_EN_N			__BIT(2)
#define	 SERDES_FIFO_FUNC_EN_N		__BIT(1)
#define	 LS_CLK_DOMAIN_FUNC_EN_N	__BIT(0)
#define	ANXDP_VIDEO_CTL_1	0x020
#define	 VIDEO_EN			__BIT(7)
#define	 VIDEO_MUTE			__BIT(6)
#define	ANXDP_VIDEO_CTL_2	0x024
#define	ANXDP_VIDEO_CTL_3	0x028
#define	ANXDP_VIDEO_CTL_4	0x02c
#define	ANXDP_VIDEO_CTL_8	0x03c
#define	ANXDP_VIDEO_CTL_10	0x044
#define	 F_SEL				__BIT(4)
#define	 SLAVE_I_SCAN_CFG		__BIT(2)
#define	 SLAVE_VSYNC_P_CFG		__BIT(1)
#define	 SLAVE_HSYNC_P_CFG		__BIT(0)
#define	ANXDP_PLL_REG_1		0x0fc
#define	 REF_CLK_24M			__BIT(0)
#define	RKANXDP_PD		0x12c
#define	 DP_INC_BG			__BIT(7)
#define	 DP_EXP_PD			__BIT(6)
#define	 DP_PHY_PD			__BIT(5)
#define	 RK_AUX_PD			__BIT(5)
#define	 AUX_PD				__BIT(4)
#define	 RK_PLL_PD			__BIT(4)
#define	 CHx_PD(x)			__BIT(x)	/* 0<=x<=3 */
#define  DP_ALL_PD			__BITS(7,0)
#define	ANXDP_LANE_MAP		0x35c
#define ANXDP_ANALOG_CTL_1	0x370
#define	 TX_TERMINAL_CTRL_50_OHM	__BIT(4)
#define ANXDP_ANALOG_CTL_2	0x374
#define	 SEL_24M			__BIT(3)
#define	 TX_DVDD_BIT_1_0625V		0x4
#define ANXDP_ANALOG_CTL_3	0x378
#define	 DRIVE_DVDD_BIT_1_0625V		(0x4 << 5)
#define	 VCO_BIT_600_MICRO		(0x5 << 0)
#define ANXDP_PLL_FILTER_CTL_1	0x37c
#define	 PD_RING_OSC			__BIT(6)
#define	 AUX_TERMINAL_CTRL_50_OHM	(2 << 4)
#define	 TX_CUR1_2X			__BIT(2)
#define	 TX_CUR_16_MA			3
#define ANXDP_TX_AMP_TUNING_CTL	0x380
#define	ANXDP_AUX_HW_RETRY_CTL	0x390
#define	 AUX_BIT_PERIOD_EXPECTED_DELAY(x) __SHIFTIN((x), __BITS(10,8))
#define	 AUX_HW_RETRY_INTERVAL_600_US	__SHIFTIN(0, __BITS(4,3))
#define	 AUX_HW_RETRY_INTERVAL_800_US	__SHIFTIN(1, __BITS(4,3))
#define	 AUX_HW_RETRY_INTERVAL_1000_US	__SHIFTIN(2, __BITS(4,3))
#define	 AUX_HW_RETRY_INTERVAL_1800_US	__SHIFTIN(3, __BITS(4,3))
#define	 AUX_HW_RETRY_COUNT_SEL(x)	__SHIFTIN((x), __BITS(2,0))
#define	ANXDP_COMMON_INT_STA_1	0x3c4
#define	 PLL_LOCK_CHG			__BIT(6)
#define	ANXDP_COMMON_INT_STA_2	0x3c8
#define	ANXDP_COMMON_INT_STA_3	0x3cc
#define	ANXDP_COMMON_INT_STA_4	0x3d0
#define	ANXDP_DP_INT_STA	0x3dc
#define	 INT_HPD			__BIT(6)
#define	 HW_TRAINING_FINISH		__BIT(5)
#define	 RPLY_RECEIV			__BIT(1)
#define	 AUX_ERR			__BIT(0)
#define	ANXDP_SYS_CTL_1		0x600
#define	 DET_STA			__BIT(2)
#define	 FORCE_DET			__BIT(1)
#define	 DET_CTRL			__BIT(0)
#define	ANXDP_SYS_CTL_2		0x604
#define	ANXDP_SYS_CTL_3		0x608
#define	 HPD_STATUS			__BIT(6)
#define	 F_HPD				__BIT(5)
#define	 HPD_CTRL			__BIT(4)
#define	 HDCP_RDY			__BIT(3)
#define	 STRM_VALID			__BIT(2)
#define	 F_VALID			__BIT(1)
#define	 VALID_CTRL			__BIT(0)
#define	ANXDP_SYS_CTL_4		0x60c
#define	ANXDP_PKT_SEND_CTL	0x640
#define	ANXDP_HDCP_CTL		0x648
#define	ANXDP_LINK_BW_SET	0x680
#define	ANXDP_LANE_COUNT_SET	0x684
#define	ANXDP_TRAINING_PTN_SET	0x688
#define	 SCRAMBLING_DISABLE		__BIT(5)
#define	 SW_TRAINING_PATTERN_SET_PTN2	__SHIFTIN(2, __BITS(1,0))
#define	 SW_TRAINING_PATTERN_SET_PTN1	__SHIFTIN(1, __BITS(1,0))
#define	ANXDP_LNx_LINK_TRAINING_CTL(x) (0x68c + 4 * (x)) /* 0 <= x <= 3 */
#define	 MAX_PRE_REACH			__BIT(5)
#define  PRE_EMPHASIS_SET(x)		__SHIFTIN((x), __BITS(4,3))
#define	 MAX_DRIVE_REACH		__BIT(2)
#define  DRIVE_CURRENT_SET(x)		__SHIFTIN((x), __BITS(1,0))
#define	ANXDP_DEBUG_CTL		0x6c0
#define	 PLL_LOCK			__BIT(4)
#define	 F_PLL_LOCK			__BIT(3)
#define	 PLL_LOCK_CTRL			__BIT(2)
#define	 PN_INV				__BIT(0)
#define	ANXDP_LINK_DEBUG_CTL	0x6e0
#define	ANXDP_PLL_CTL		0x71c
#define	ANXDP_PHY_PD		0x720
#define	ANXDP_PHY_TEST		0x724
#define	 MACRO_RST			__BIT(5)
#define ANXDP_M_AUD_GEN_FILTER_TH 0x778
#define	ANXDP_AUX_CH_STA	0x780
#define	 AUX_BUSY			__BIT(4)
#define	 AUX_STATUS(x)			__SHIFTOUT((x), __BITS(3,0))
#define	ANXDP_AUX_ERR_NUM	0x784
#define	ANXDP_AUX_CH_DEFER_CTL	0x788
#define	 DEFER_CTRL_EN			__BIT(7)
#define	 DEFER_COUNT(x)			__SHIFTIN((x), __BITS(6,0))
#define	ANXDP_AUX_RX_COMM	0x78c
#define	 AUX_RX_COMM_I2C_DEFER		__BIT(3)
#define	 AUX_RX_COMM_AUX_DEFER		__BIT(1)
#define	ANXDP_BUFFER_DATA_CTL	0x790
#define	 BUF_CLR			__BIT(7)
#define	 BUF_DATA_COUNT(x)		__SHIFTIN((x), __BITS(4,0))
#define	ANXDP_AUX_CH_CTL_1	0x794
#define	 AUX_LENGTH(x)			__SHIFTIN((x) - 1, __BITS(7,4))
#define AUX_TX_COMM_I2C_TRANSACTION		(0x0 << 3)

#define	 AUX_TX_COMM(x)			__SHIFTOUT(x, __BITS(3,0))
#define	 AUX_TX_COMM_DP			__BIT(3)
#define	 AUX_TX_COMM_MOT		__BIT(2)
#define	 AUX_TX_COMM_READ		__BIT(0)
#define	ANXDP_AUX_ADDR_7_0	0x798
#define	 AUX_ADDR_7_0(x)	 (((x) >> 0) & 0xff)
#define	ANXDP_AUX_ADDR_15_8	0x79c
#define	 AUX_ADDR_15_8(x)	 (((x) >> 8) & 0xff)
#define	ANXDP_AUX_ADDR_19_16	0x7a0
#define	 AUX_ADDR_19_16(x)	 (((x) >> 16) & 0xf)
#define	ANXDP_AUX_CH_CTL_2	0x7a4
#define	 ADDR_ONLY			__BIT(1)
#define	 AUX_EN				__BIT(0)
#define	ANXDP_BUF_DATA(x)	(0x7c0 + 4 * (x))
#define	ANXDP_SOC_GENERAL_CTL	0x800
#define	 AUDIO_MODE_SPDIF_MODE		__BIT(8)
#define	 VIDEO_MODE_SLAVE_MODE		__BIT(1)
#define	ANXDP_CRC_CON		0x890
#define	ANXDP_PLL_REG_2		0x9e4
#define	ANXDP_PLL_REG_3		0x9e8
#define	ANXDP_PLL_REG_4		0x9ec
#define	ANXDP_PLL_REG_5		0xa00




struct anxdp_softc {
	struct mtx mtx;
	device_t	iicbus;
	struct device*	sc_dev;

	struct resource *res[2];
	u_int		sc_flags;
#define	ANXDP_FLAG_ROCKCHIP	__BIT(0)
	struct drm_connector 	sc_connector;
	struct drm_encoder      sc_encoder;
	struct drm_dp_aux       sc_dpaux;
	struct drm_panel *	sc_panel;
	uint8_t			sc_dpcd[DP_RECEIVER_CAP_SIZE];
	struct drm_bridge	sc_bridge;
	struct drm_display_mode	sc_curmode;
};

#define	ANXDP_LOCK(sc)	mtx_lock(&(sc)->mtx)
#define	ANXDP_UNLOCK(sc) mtx_unlock(&(sc)->mtx)
#define	ANXDP_WRITE(sc, reg, val) bus_write_4((sc)->res[0], (reg), (val))
#define	ANXDP_READ(sc, reg) bus_read_4((sc)->res[0], (reg))

#define	to_edp_connector(x)	container_of(x, struct anxdp_connector, base)
int anxdp_attach(struct anxdp_softc *sc);
void anxdp_add_bridge(struct anxdp_softc *sc,struct drm_encoder *encoder);

DECLARE_CLASS(anxdp_driver);
#endif /* ANXEDP_H */

