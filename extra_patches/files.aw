# Allwinner Video Controller
dev/drm/allwinner/aw_de2.c		optional drm fdt aw_de2_drm
dev/drm/allwinner/aw_de2_drm.c		optional drm fdt aw_de2_drm compile-with "${DRM_C}"
dev/drm/allwinner/aw_de2_ui_plane.c	optional drm fdt aw_de2_drm compile-with "${DRM_C}"
dev/drm/allwinner/aw_de2_vi_plane.c	optional drm fdt aw_de2_drm compile-with "${DRM_C}"
dev/drm/allwinner/aw_de2_mixer.c	optional drm fdt aw_de2_drm compile-with "${DRM_C}"
dev/drm/allwinner/aw_de2_tcon.c		optional drm fdt aw_de2_drm compile-with "${DRM_C}"
dev/drm/allwinner/aw_de2_hdmi_phy.c	optional drm fdt aw_de2_drm compile-with "${DRM_C}"
dev/drm/allwinner/aw_de2_mixer_if.m	optional drm fdt aw_de2_drm
dev/drm/allwinner/aw_de2_tcon_if.m	optional drm fdt aw_de2_drm

# Synopsis DesignWare HDMI Controller
dev/drm/bridges/dw_hdmi/aw_de2_dw_hdmi.c	optional drm fdt dw_hdmi aw_de2_drm compile-with "${DRM_C}"
dev/drm/bridges/dw_hdmi/dw_hdmi_if.m		optional drm fdt dw_hdmi aw_de2_drm
dev/drm/bridges/dw_hdmi/dw_hdmi_phy_if.m	optional drm fdt dw_hdmi aw_de2_drm

# ANX6345 RGB to eDP bridge
dev/drm/bridges/anx6345/anx6345.c		optional fdt anx6345 compile-with "${DRM_C}"
