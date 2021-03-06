/*-
 * Copyright (c) 2019 Emmanuel Vadot <manu@FreeBSD.org>
 * Copyright (c) 2015 Oleksandr Tymoshenko <gonzo@freebsd.org>
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

#ifndef _DW_HDMI_H_
#define _DW_HDMI_H_

#define	DW_HDMI_DESIGN_ID	0x00
#define	DW_HDMI_REVISION_ID	0x01
#define	DW_HDMI_PRODUCT_ID0	0x02
#define	DW_HDMI_PRODUCT_ID1	0x03
#define	DW_HDMI_CONFIG0_ID	0x04
#define	DW_HDMI_CONFIG1_ID	0x05
#define	DW_HDMI_CONFIG2_ID	0x06
#define	DW_HDMI_CONFIG3_ID	0x07

/* Interrupt registers. */
#define	DW_HDMI_IH_FC_STAT0	0x0100
#define	DW_HDMI_IH_FC_STAT1	0x0101
#define	DW_HDMI_IH_FC_STAT2	0x0102
#define	DW_HDMI_IH_AS_STAT0	0x0103
#define	DW_HDMI_IH_PHY_STAT0	0x0104

#define	DW_HDMI_IH_I2CM_STAT0	0x0105
#define	 DW_HDMI_IH_I2CM_STAT0_ERROR	(1 << 0)
#define	 DW_HDMI_IH_I2CM_STAT0_DONE	(1 << 1)

#define	DW_HDMI_IH_CEC_STAT0	0x0106
#define	DW_HDMI_IH_VP_STAT0	0x0107
#define	DW_HDMI_IH_I2CMPHY_STAT0	0x0108
#define	 DW_HDMI_IH_I2CMPHY_STAT0_ERROR	(1 << 0)
#define	 DW_HDMI_IH_I2CMPHY_STAT0_DONE	(1 << 1)
#define	DW_HDMI_IH_AHBDMAAUD_STAT0	0x0109

#define	DW_HDMI_IH_MUTE_FC_STAT0	0x0180
#define	DW_HDMI_IH_MUTE_FC_STAT1	0x0181
#define	DW_HDMI_IH_MUTE_FC_STAT2	0x0182
#define	DW_HDMI_IH_MUTE_AS_STAT0	0x0183
#define	DW_HDMI_IH_MUTE_PHY_STAT0	0x0184

#define	DW_HDMI_IH_MUTE_I2CM_STAT0	0x0185
#define	 DW_HDMI_IH_MUTE_I2CM_STAT0_ERROR	(1 << 0)
#define	 DW_HDMI_IH_MUTE_I2CM_STAT0_DONE	(1 << 1)

#define	DW_HDMI_IH_MUTE_CEC_STAT0	0x0186
#define	DW_HDMI_IH_MUTE_VP_STAT0	0x0187
#define	DW_HDMI_IH_MUTE_I2CMPHY_STAT0	0x0188
#define	DW_HDMI_IH_MUTE_AHBDMAAUD_STAT0	0x0189

#define	DW_HDMI_IH_MUTE		0x01FF
#define	 DW_HDMI_IH_MUTE_ALL	(1 << 0)
#define	 DW_HDMI_IH_MUTE_WAKEUP	(1 << 1)

/* Video Packetizer */
#define	DW_HDMI_VP_STATUS	0x0800
#define	DW_HDMI_VP_PR_CD	0x0801
#define	DW_HDMI_VP_STUFF	0x0802
#define	DW_HDMI_VP_REMAP	0x0803
#define	DW_HDMI_VP_CONF		0x0804
#define	DW_HDMI_VP_MASK		0x0807

/* Frame Composer */
#define	DW_HDMI_FC_INVIDCONF	0x1000
#define	 DW_HDMI_FC_INVIDCONF_HSYNC_POL_HIGH	(1 << 5)
#define	 DW_HDMI_FC_INVIDCONF_VSYNC_POL_HIGH	(1 << 6)
#define	 DW_HDMI_FC_INVIDCONF_DATA_POL_HIGH	(1 << 4)
#define	 DW_HDMI_FC_INVIDCONF_HDMI_MODE		(1 << 3)
#define	 DW_HDMI_FC_INVIDCONF_INTERLACED_MODE	(1 << 0)
#define	DW_HDMI_FC_INHACTIV0	0x1001
#define	DW_HDMI_FC_INHACTIV1	0x1002
#define	DW_HDMI_FC_INHBLANK0	0x1003
#define	DW_HDMI_FC_INHBLANK1	0x1004
#define	DW_HDMI_FC_INVACTIV0	0x1005
#define	DW_HDMI_FC_INVACTIV1	0x1006
#define	DW_HDMI_FC_INVBLANK	0x1007
#define	DW_HDMI_FC_HSYNCINDELAY0	0x1008
#define	DW_HDMI_FC_HSYNCINDELAY1	0x1009
#define	DW_HDMI_FC_HSYNCINWIDTH0	0x100A
#define	DW_HDMI_FC_HSYNCINWIDTH1	0x100B
#define	DW_HDMI_FC_VSYNCINDELAY	0x100C
#define	DW_HDMI_FC_VSYNCINWIDTH	0x100D
#define	DW_HDMI_FC_CTRLDUR	0x1011
#define	DW_HDMI_FC_EXCTRLDUR	0x1012
#define	DW_HDMI_FC_EXCTRLSPAC	0x1013
#define	DW_HDMI_FC_CH0PREAM	0x1014
#define	DW_HDMI_FC_CH1PREAM	0x1015
#define	DW_HDMI_FC_CH2PREAM	0x1016
#define	DW_HDMI_FC_AVICONF3	0x1017
#define	DW_HDMI_FC_GCP		0x1018
#define	DW_HDMI_FC_AVICONF0	0x1019
#define	DW_HDMI_FC_AVICONF1	0x101A
#define	DW_HDMI_FC_AVICONF2	0x101B
#define	DW_HDMI_FC_AVIDVID	0x101C
#define	DW_HDMI_FC_AVIDETB0	0x101D
#define	DW_HDMI_FC_AVIDETB1	0x101E
#define	DW_HDMI_FC_AVISBB0	0x101F
#define	DW_HDMI_FC_AVISBB1	0x1020
#define	DW_HDMI_FC_AVIELB0	0x1021
#define	DW_HDMI_FC_AVIELB1	0x1022
#define	DW_HDMI_FC_AVISRB0	0x1023
#define	DW_HDMI_FC_AVISRB1	0x1024
#define	DW_HDMI_FC_AUDICONF0	0x1025
#define	DW_HDMI_FC_AUDICONF1	0x1026
#define	DW_HDMI_FC_AUDICONF2	0x1027
#define	DW_HDMI_FC_AUDICONF3	0x1028
#define	DW_HDMI_FC_VSDIEEEID0	0x1029
#define	DW_HDMI_FC_VSDSIZE	0x102A

/* Phy registers */
#define	DW_HDMI_PHY_CONF0			0x3000
#define	  HDMI_PHY_CONF0_PDZ_MASK		0x80
#define	  HDMI_PHY_CONF0_PDZ_OFFSET		7
#define	  HDMI_PHY_CONF0_ENTMDS_MASK		0x40
#define	  HDMI_PHY_CONF0_ENTMDS_OFFSET		6
#define	  HDMI_PHY_CONF0_SPARECTRL_MASK		0x20
#define	  HDMI_PHY_CONF0_SPARECTRL_OFFSET	5
#define	  HDMI_PHY_CONF0_GEN2_PDDQ_MASK		0x10
#define	  HDMI_PHY_CONF0_GEN2_PDDQ_OFFSET	4
#define	  HDMI_PHY_CONF0_GEN2_TXPWRON_MASK	0x8
#define	  HDMI_PHY_CONF0_GEN2_TXPWRON_OFFSET	3
#define	  HDMI_PHY_CONF0_GEN2_ENHPDRXSENSE_MASK	0x4
#define	  HDMI_PHY_CONF0_GEN2_ENHPDRXSENSE_OFFSET	2
#define	  HDMI_PHY_CONF0_SELDATAENPOL_MASK	0x2
#define	  HDMI_PHY_CONF0_SELDATAENPOL_OFFSET	1
#define	  HDMI_PHY_CONF0_SELDIPIF_MASK		0x1
#define	  HDMI_PHY_CONF0_SELDIPIF_OFFSET	0
#define	DW_HDMI_PHY_TST0			0x3001
#define	  HDMI_PHY_TST0_TSTCLR_MASK		0x20
#define	  HDMI_PHY_TST0_TSTCLR_OFFSET		5
#define	  HDMI_PHY_TST0_TSTEN_MASK		0x10
#define	  HDMI_PHY_TST0_TSTEN_OFFSET		4
#define	  HDMI_PHY_TST0_TSTCLK_MASK		0x1
#define	  HDMI_PHY_TST0_TSTCLK_OFFSET		0
#define	DW_HDMI_PHY_TST1			0x3002
#define	DW_HDMI_PHY_TST2			0x3003
#define	DW_HDMI_PHY_STAT0			0x3004
#define	  HDMI_PHY_STAT0_RX_SENSE3		0x80
#define	  HDMI_PHY_STAT0_RX_SENSE2		0x40
#define	  HDMI_PHY_STAT0_RX_SENSE1		0x20
#define	  HDMI_PHY_STAT0_RX_SENSE0		0x10
#define	  HDMI_PHY_STAT0_RX_SENSE		0xf0
#define	  HDMI_PHY_STAT0_HPD			0x02
#define	  HDMI_PHY_TX_PHY_LOCK			0x01
#define	DW_HDMI_PHY_INT0			0x3005
#define	DW_HDMI_PHY_MASK0			0x3006
#define	DW_HDMI_PHY_POL0			0x3007
#define	  HDMI_PHY_POL0_HPD			0x02

/* HDMI Master PHY Registers */
#define	DW_HDMI_PHY_I2CM_SLAVE_ADDR		0x3020
#define	  HDMI_PHY_I2CM_SLAVE_ADDR_PHY_GEN2	0x69
#define	  HDMI_PHY_I2CM_SLAVE_ADDR_HEAC_PHY	0x49
#define	DW_HDMI_PHY_I2CM_ADDRESS_ADDR		0x3021
#define	DW_HDMI_PHY_I2CM_DATAO_1_ADDR		0x3022
#define	DW_HDMI_PHY_I2CM_DATAO_0_ADDR		0x3023
#define	DW_HDMI_PHY_I2CM_DATAI_1_ADDR		0x3024
#define	DW_HDMI_PHY_I2CM_DATAI_0_ADDR		0x3025
#define	DW_HDMI_PHY_I2CM_OPERATION_ADDR		0x3026
#define	  HDMI_PHY_I2CM_OPERATION_ADDR_WRITE	0x10
#define	  HDMI_PHY_I2CM_OPERATION_ADDR_READ	0x1
#define	DW_HDMI_PHY_I2CM_INT_ADDR		0x3027
#define	DW_HDMI_PHY_I2CM_CTLINT_ADDR		0x3028
#define	DW_HDMI_PHY_I2CM_DIV_ADDR		0x3029
#define	DW_HDMI_PHY_I2CM_SOFTRSTZ_ADDR		0x302a
#define	DW_HDMI_PHY_I2CM_SS_SCL_HCNT_1_ADDR	0x302b
#define	DW_HDMI_PHY_I2CM_SS_SCL_HCNT_0_ADDR	0x302c
#define	DW_HDMI_PHY_I2CM_SS_SCL_LCNT_1_ADDR	0x302d
#define	DW_HDMI_PHY_I2CM_SS_SCL_LCNT_0_ADDR	0x302e
#define	DW_HDMI_PHY_I2CM_FS_SCL_HCNT_1_ADDR	0x302f
#define	DW_HDMI_PHY_I2CM_FS_SCL_HCNT_0_ADDR	0x3030
#define	DW_HDMI_PHY_I2CM_FS_SCL_LCNT_1_ADDR	0x3031
#define	DW_HDMI_PHY_I2CM_FS_SCL_LCNT_0_ADDR	0x3032

/* Main Controller */
#define	DW_HDMI_MC_CLKDIS			0x4001
#define	 DW_HDMI_MC_CLKDIS_PIXELCLK		(1 << 0)
#define	 DW_HDMI_MC_CLKDIS_TMDSCLK		(1 << 1)
#define	 DW_HDMI_MC_CLKDIS_PREPCLK		(1 << 2)
#define	 DW_HDMI_MC_CLKDIS_AUDCLK		(1 << 3)
#define	 DW_HDMI_MC_CLKDIS_CSCCLK		(1 << 4)
#define	 DW_HDMI_MC_CLKDIS_CECCLK		(1 << 5)
#define	 DW_HDMI_MC_CLKDIS_HDCPCLK		(1 << 6)
#define	DW_HDMI_MC_SWRSTZREQ			0x4002
#define	DW_HDMI_MC_FLOWCTRL			0x4004
#define	  HDMI_MC_FLOWCTRL_FEED_THROUGH_OFF_MASK	0x1
#define	  HDMI_MC_FLOWCTRL_FEED_THROUGH_OFF_CSC_IN_PATH	0x1
#define	  HDMI_MC_FLOWCTRL_FEED_THROUGH_OFF_CSC_BYPASS	0x0
#define	DW_HDMI_MC_PHYRSTZ			0x4005
#define	  HDMI_MC_PHYRSTZ_ASSERT		0x0
#define	  HDMI_MC_PHYRSTZ_DEASSERT		0x1
#define	DW_HDMI_MC_LOCKONCLOCK			0x4006
#define	DW_HDMI_MC_HEACPHY_RST			0x4007
#define	  HDMI_MC_HEACPHY_RST_ASSERT		0x1
#define	  HDMI_MC_HEACPHY_RST_DEASSERT		0x0

/* HDCP Encryption Engine Registers */
#define	DW_HDMI_A_HDCPCFG0				0x5000
#define	  HDMI_A_HDCPCFG0_RXDETECT_MASK			0x4
#define	  HDMI_A_HDCPCFG0_RXDETECT_ENABLE		0x4
#define	  HDMI_A_HDCPCFG0_RXDETECT_DISABLE		0x0
#define	DW_HDMI_A_HDCPCFG1				0x5001
#define	  HDMI_A_HDCPCFG1_ENCRYPTIONDISABLE_MASK	0x2
#define	  HDMI_A_HDCPCFG1_ENCRYPTIONDISABLE_DISABLE	0x2
#define	  HDMI_A_HDCPCFG1_ENCRYPTIONDISABLE_ENABLE	0x0

/* PHY I2CM */
#define	DW_HDMI_PHY_I2CM_INT_ADDR	0x3027
#define	DW_HDMI_PHY_I2CM_CTLINT_ADDR	0x3028

/* I2CM */
#define	DW_HDMI_I2CM_SLAVE	0x7E00
#define	DW_HDMI_I2CM_ADDRESS	0x7E01
#define	DW_HDMI_I2CM_DATAO	0x7E02
#define	DW_HDMI_I2CM_DATAI	0x7E03
#define	DW_HDMI_I2CM_OP		0x7E04
#define	 DW_HDMI_I2CM_OP_RD	(1 << 0)
#define	 DW_HDMI_I2CM_OP_RDEXT	(1 << 1)
#define	 DW_HDMI_I2CM_OP_WR	(1 << 4)

#define	DW_HDMI_I2CM_INT		0x7E05
#define	 DW_HDMI_I2CM_INT_DONE_STATUS	(1 << 0)
#define	 DW_HDMI_I2CM_INT_DONE_INT	(1 << 1)
#define	 DW_HDMI_I2CM_INT_DONE_MASK	(1 << 2)
#define	 DW_HDMI_I2CM_INT_DONE_POL	(1 << 3)

#define	DW_HDMI_I2CM_CLINT		0x7E06
#define	DW_HDMI_I2CM_CLINT_ARB_STATUS	(1 << 0)
#define	DW_HDMI_I2CM_CLINT_ARB_INT	(1 << 1)
#define	DW_HDMI_I2CM_CLINT_ARB_MASK	(1 << 2)
#define	DW_HDMI_I2CM_CLINT_ARB_POL	(1 << 3)
#define	DW_HDMI_I2CM_CLINT_NACK_STATUS	(1 << 4)
#define	DW_HDMI_I2CM_CLINT_NACK_INT	(1 << 5)
#define	DW_HDMI_I2CM_CLINT_NACK_MASK	(1 << 6)
#define	DW_HDMI_I2CM_CLINT_NACK_POL	(1 << 7)

#define	DW_HDMI_I2CM_DIV		0x7E07
#define	 DW_HDMI_I2CM_DIV_FAST_MODE	(1 << 3)

#define	DW_HDMI_I2CM_SEGADDR		0x7E08

#define	DW_HDMI_I2CM_SOFTRSTZ		0x7E09
#define	 DW_HDMI_I2CM_SOFTRSTZ_RST	(1 << 0)

#define	DW_HDMI_I2CM_SEGPTR		0x7E0A

/* HDMI PHY register with access through I2C */
#define	DW_HDMI_PHY_I2C_CKCALCTRL	0x5
#define	  CKCALCTRL_OVERRIDE		(1 << 15)
#define	DW_HDMI_PHY_I2C_CPCE_CTRL	0x6
#define	  CPCE_CTRL_45_25		((3 << 7) | (3 << 5))
#define	  CPCE_CTRL_92_50		((2 << 7) | (2 << 5))
#define	  CPCE_CTRL_185			((1 << 7) | (1 << 5))
#define	  CPCE_CTRL_370			((0 << 7) | (0 << 5))
#define	DW_HDMI_PHY_I2C_CKSYMTXCTRL	0x9
#define	  CKSYMTXCTRL_OVERRIDE		(1 << 15)
#define	  CKSYMTXCTRL_TX_SYMON		(1 << 3)
#define	  CKSYMTXCTRL_TX_TRAON		(1 << 2)
#define	  CKSYMTXCTRL_TX_TRBON		(1 << 1)
#define	  CKSYMTXCTRL_TX_CK_SYMON	(1 << 0)
#define	DW_HDMI_PHY_I2C_VLEVCTRL	0x0E
#define	DW_HDMI_PHY_I2C_CURRCTRL	0x10
#define	DW_HDMI_PHY_I2C_PLLPHBYCTRL	0x13
#define	  VLEVCTRL_TX_LVL(x)		((x) << 5)
#define	  VLEVCTRL_CK_LVL(x)		(x)
#define	DW_HDMI_PHY_I2C_GMPCTRL		0x15
#define	  GMPCTRL_45_25			0x00
#define	  GMPCTRL_92_50			0x05
#define	  GMPCTRL_185			0x0a
#define	  GMPCTRL_370			0x0f
#define	DW_HDMI_PHY_I2C_MSM_CTRL	0x17
#define	  MSM_CTRL_FB_CLK		(0x3 << 1)
#define	DW_HDMI_PHY_I2C_TXTERM		0x19
#define	  TXTERM_133			0x5

#define	GRF_SOC_CON20	0x6250
#define	 CON20_DSI0_VOP_SEL_S	0
#define	 CON20_DSI0_VOP_SEL_M	(1 << CON20_DSI0_VOP_SEL_S)
#define	 CON20_DSI0_VOP_SEL_B	(0 << CON20_DSI0_VOP_SEL_S)
#define	 CON20_DSI0_VOP_SEL_L	(1 << CON20_DSI0_VOP_SEL_S)
#define	 CON20_HDMI_VOP_SEL_S	6
#define	 CON20_HDMI_VOP_SEL_M	(1 << CON20_HDMI_VOP_SEL_S)
#define	 CON20_HDMI_VOP_SEL_B	(0 << CON20_HDMI_VOP_SEL_S)
#define	 CON20_HDMI_VOP_SEL_L	(1 << CON20_HDMI_VOP_SEL_S)

#define	GRF_SOC_CON22	0x6258
#define	 CON22_DPHY_TX0_RXMODE_S	0
#define	 CON22_DPHY_TX0_RXMODE_M	(0xf << CON22_DPHY_TX0_RXMODE_S)
#define	 CON22_DPHY_TX0_RXMODE_EN	(0xb << CON22_DPHY_TX0_RXMODE_S)
#define	 CON22_DPHY_TX0_RXMODE_DIS	(0x0 << CON22_DPHY_TX0_RXMODE_S)
#define	 CON22_DPHY_TX0_TXSTOPMODE_S	4
#define	 CON22_DPHY_TX0_TXSTOPMODE_M	(0xf << CON22_DPHY_TX0_TXSTOPMODE_S)
#define	 CON22_DPHY_TX0_TXSTOPMODE_EN	(0xc << CON22_DPHY_TX0_TXSTOPMODE_S)
#define	 CON22_DPHY_TX0_TXSTOPMODE_DIS	(0x0 << CON22_DPHY_TX0_TXSTOPMODE_S)
#define	 CON22_DPHY_TX0_TURNREQUEST_S	12
#define	 CON22_DPHY_TX0_TURNREQUEST_M	(0xf << CON22_DPHY_TX0_TURNREQUEST_S)
#define	 CON22_DPHY_TX0_TURNREQUEST_EN	(0x1 << CON22_DPHY_TX0_TURNREQUEST_S)
#define	 CON22_DPHY_TX0_TURNREQUEST_DIS	(0x0 << CON22_DPHY_TX0_TURNREQUEST_S)

#endif /* _DW_HDMI_H_ */
