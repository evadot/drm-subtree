/* Public domain */

#ifndef __DRMKPI_LINUX_FB_H_
#define __DRMKPI_LINUX_FB_H_
#include <sys/fbio.h>

#include <linux/fs.h>
#include <linux/init.h>
#include <linux/workqueue.h>
#include <linux/list.h>

/* screen: unblanked, hsync: on,  vsync: on */
#define	FB_BLANK_UNBLANK	0

/* screen: blanked,   hsync: on,  vsync: on */
#define	FB_BLANK_NORMAL		1

/* screen: blanked,   hsync: on,  vsync: off */
#define	FB_BLANK_VSYNC_SUSPEND	2

/* screen: blanked,   hsync: off, vsync: on */
#define	FB_BLANK_HSYNC_SUSPEND 3

/* screen: blanked,   hsync: off, vsync: off */
#define	FB_BLANK_POWERDOWN	4

struct fb_cmap;
struct fb_var_screeninfo;

/* FBSD implementation in drm_fb_helper.c */
int fb_get_options(const char *name, char **option);
struct fb_info *framebuffer_alloc(size_t size, struct device *dev);
void framebuffer_release(struct fb_info *info);

struct fb_fillrect;
struct fb_copyarea;
struct fb_image;

struct vt_kms_softc {
	struct drm_fb_helper    *fb_helper;
	struct task              fb_mode_task;
};

#endif /* __DRMKPI_LINUX_FB_H_ */
