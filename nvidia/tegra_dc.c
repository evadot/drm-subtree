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
#include <sys/gpio.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/rman.h>
#include <sys/sysctl.h>

#include <machine/bus.h>

#include <dev/extres/clk/clk.h>
#include <dev/extres/hwreset/hwreset.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fixed.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_gem.h>
#include <drm/drm_vblank.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/drm/nvidia/tegra_dc_reg.h>
#include <dev/drm/nvidia/tegra_drm.h>
#include <arm/nvidia/tegra_pmc.h>

#include "tegra_drm_if.h"
#include "tegra_dc_if.h"

#define	WR4(_sc, _r, _v)	bus_write_4((_sc)->mem_res, 4 * (_r), (_v))
#define	RD4(_sc, _r)		bus_read_4((_sc)->mem_res, 4 * (_r))

#define	LOCK(_sc)		mtx_lock(&(_sc)->mtx)
#define	UNLOCK(_sc)		mtx_unlock(&(_sc)->mtx)
#define	SLEEP(_sc, timeout)						\
	mtx_sleep(sc, &sc->mtx, 0, "tegra_dc_wait", timeout);
#define	LOCK_INIT(_sc)							\
	mtx_init(&_sc->mtx, device_get_nameunit(_sc->dev), "tegra_dc", MTX_DEF)
#define	LOCK_DESTROY(_sc)	mtx_destroy(&_sc->mtx)
#define	ASSERT_LOCKED(_sc)	mtx_assert(&_sc->mtx, MA_OWNED)
#define	ASSERT_UNLOCKED(_sc)	mtx_assert(&_sc->mtx, MA_NOTOWNED)


#define	SYNCPT_VBLANK0 26
#define	SYNCPT_VBLANK1 27

#define	DC_MAX_PLANES 2		/* Maximum planes */

/* DRM Formats supported by DC */
static uint32_t dc_primary_plane_formats[] = {
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_RGB565,
};

static uint32_t dc_cursor_plane_formats[] = {
	DRM_FORMAT_RGBA8888,
};


static uint32_t dc_overlay_plane_formats[] = {
	DRM_FORMAT_ARGB4444,
	DRM_FORMAT_ARGB1555,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_RGBA5551,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_ABGR4444,
	DRM_FORMAT_ABGR1555,
	DRM_FORMAT_BGRA5551,
	DRM_FORMAT_XRGB1555,
	DRM_FORMAT_RGBX5551,
	DRM_FORMAT_XBGR1555,
	DRM_FORMAT_BGRX5551,
	DRM_FORMAT_BGR565,
	DRM_FORMAT_BGRA8888,
	DRM_FORMAT_RGBA8888,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_RGBX8888,
	DRM_FORMAT_BGRX8888,
	DRM_FORMAT_UYVY,
	DRM_FORMAT_YUYV,
	DRM_FORMAT_YUV420,
	DRM_FORMAT_YUV422,
};

static const uint64_t plane_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_NVIDIA_16BX2_BLOCK(0),
	DRM_FORMAT_MOD_NVIDIA_16BX2_BLOCK(1),
	DRM_FORMAT_MOD_NVIDIA_16BX2_BLOCK(2),
	DRM_FORMAT_MOD_NVIDIA_16BX2_BLOCK(3),
	DRM_FORMAT_MOD_NVIDIA_16BX2_BLOCK(4),
	DRM_FORMAT_MOD_NVIDIA_16BX2_BLOCK(5),
	DRM_FORMAT_MOD_INVALID
};


/* Complete description of one window (plane) */
struct dc_window {
	/* Source (in framebuffer) rectangle, in pixels */
	u_int			src_x;
	u_int			src_y;
	u_int			src_w;
	u_int			src_h;

	/* Destination (on display) rectangle, in pixels */
	u_int			dst_x;
	u_int			dst_y;
	u_int			dst_w;
	u_int			dst_h;

	/* Parsed pixel format */
	u_int			bits_per_pixel;
	bool			is_yuv;		/* any YUV mode */
	bool			is_yuv_planar;	/* planar YUV mode */
	uint32_t 		color_mode;	/* DC_WIN_COLOR_DEPTH */
	uint32_t		swap;		/* DC_WIN_BYTE_SWAP */
	uint32_t		surface_kind;	/* DC_WINBUF_SURFACE_KIND */
	uint32_t		block_height;	/* DC_WINBUF_SURFACE_KIND */

	/* Parsed flipping, rotation is not supported for pitched modes */
	bool			flip_x;		/* inverted X-axis */
	bool			flip_y;		/* inverted Y-axis */
	bool			transpose_xy;	/* swap X and Y-axis */

	/* Color planes base addresses and strides */
	bus_size_t		base[3];
	uint32_t		stride[3];	/* stride[2] isn't used by HW */
};

struct tegra_dc_state {
	struct drm_crtc_state	drm_state;
	clk_t			*clk;
	uint64_t		pclk;
	uint64_t		div;
	uint32_t		planes_mask;
};

struct dc_plane_state {
	struct drm_plane_state drm_plane_state;
};

struct dc_softc {
	device_t		dev;
	struct resource		*mem_res;
	struct resource		*irq_res;
	void			*irq_ih;
	struct mtx		mtx;

	clk_t			clk_parent;
	clk_t			clk_dc;
	hwreset_t		hwreset_dc;

	int			pitch_align;

	struct tegra_crtc 	tegra_crtc;
	struct drm_pending_vblank_event *event;
};


static struct ofw_compat_data compat_data[] = {
	{"nvidia,tegra124-dc",	1},
	{NULL,			0},
};

/* Convert standard drm pixel format to tegra windows parameters. */
static int
dc_parse_drm_format(struct tegra_fb *fb, struct dc_window *win)
{
	struct tegra_bo *bo;
	uint32_t cm;
	uint32_t sw;
	bool is_yuv, is_yuv_planar;
	int i;

	DRM_TRACE();
	switch (fb->drm_fb.format->format) {
	case DRM_FORMAT_XBGR8888:
		sw = BYTE_SWAP(NOSWAP);
		cm = WIN_COLOR_DEPTH_R8G8B8A8;
		is_yuv = false;
		is_yuv_planar = false;
		break;

	case DRM_FORMAT_XRGB8888:
		sw = BYTE_SWAP(NOSWAP);
		cm = WIN_COLOR_DEPTH_B8G8R8A8;
		is_yuv = false;
		is_yuv_planar = false;
		break;

	case DRM_FORMAT_RGB565:
		sw = BYTE_SWAP(NOSWAP);
		cm = WIN_COLOR_DEPTH_B5G6R5;
		is_yuv = false;
		is_yuv_planar = false;
		break;

	case DRM_FORMAT_UYVY:
		sw = BYTE_SWAP(NOSWAP);
		cm = WIN_COLOR_DEPTH_YCbCr422;
		is_yuv = true;
		is_yuv_planar = false;
		break;

	case DRM_FORMAT_YUYV:
		sw = BYTE_SWAP(SWAP2);
		cm = WIN_COLOR_DEPTH_YCbCr422;
		is_yuv = true;
		is_yuv_planar = false;
		break;

	case DRM_FORMAT_YUV420:
		sw = BYTE_SWAP(NOSWAP);
		cm = WIN_COLOR_DEPTH_YCbCr420P;
		is_yuv = true;
		is_yuv_planar = true;
		break;

	case DRM_FORMAT_YUV422:
		sw = BYTE_SWAP(NOSWAP);
		cm = WIN_COLOR_DEPTH_YCbCr422P;
		is_yuv = true;
		is_yuv_planar = true;
		break;

	default:
		/* Unsupported format */
		return (-EINVAL);
	}

	/* Basic check of arguments. */
	switch (fb->rotation) {
	case 0:
	case 180:
		break;

	case 90: 		/* Rotation is supported only */
	case 270:		/*  for block linear surfaces */
		if (!fb->block_linear)
			return (-EINVAL);
		break;

	default:
		return (-EINVAL);
	}
	/* XXX Add more checks (sizes, scaling...) */

	if (win == NULL)
		return (0);

	win->surface_kind =
	    fb->block_linear ? SURFACE_KIND_BL_16B2: SURFACE_KIND_PITCH;
	win->block_height = fb->block_height;
	switch (fb->rotation) {
	case 0:					/* (0,0,0) */
		win->transpose_xy = false;
		win->flip_x = false;
		win->flip_y = false;
		break;

	case 90:				/* (1,0,1) */
		win->transpose_xy = true;
		win->flip_x = false;
		win->flip_y = true;
		break;

	case 180:				/* (0,1,1) */
		win->transpose_xy = false;
		win->flip_x = true;
		win->flip_y = true;
		break;

	case 270:				/* (1,1,0) */
		win->transpose_xy = true;
		win->flip_x = true;
		win->flip_y = false;
		break;
	}
	win->flip_x ^= fb->flip_x;
	win->flip_y ^= fb->flip_y;

	win->color_mode = cm;
	win->swap = sw;
	win->bits_per_pixel = fb->drm_fb.format->cpp[0] * 8;
	win->is_yuv = is_yuv;
	win->is_yuv_planar = is_yuv_planar;

	for (i = 0; i < fb->drm_fb.format->num_planes; i++) {
		bo = fb->planes[i];
		win->base[i] = bo->pbase + fb->drm_fb.offsets[i];
		win->stride[i] = fb->drm_fb.pitches[i];
	}
	return (0);
}

/*
 * Scaling functions.
 *
 * It's unclear if we want/must program the fractional portion
 * (aka bias) of init_dda registers, mainly when mirrored axis
 * modes are used.
 * For now, we use 1.0 as recommended by TRM.
 */
static inline uint32_t
dc_scaling_init(uint32_t start)
{

	return (1 << 12);
}

static inline uint32_t
dc_scaling_incr(uint32_t src, uint32_t dst, uint32_t maxscale)
{
	uint32_t val;

	val = (src - 1) << 12 ; /* 4.12 fixed float */
	val /= (dst - 1);
	if (val  > (maxscale << 12))
		val = maxscale << 12;
	return val;
}

/* -------------------------------------------------------------------
 *
 *    HW Access.
 *
 */

/*
 * Setup pixel clock.
 * Minimal frequency is pixel clock, but output is free to select
 * any higher.
 */
static int
dc_setup_clk(struct dc_softc *sc, struct drm_crtc *crtc,
    struct drm_display_mode *mode, uint32_t *div)
{
	uint64_t pclk, freq;
	struct tegra_drm_encoder *output;
	struct drm_encoder *encoder;
	long rv;

	DRM_TRACE();
	pclk = mode->clock * 1000;

	/* Find attached encoder */
	output = NULL;
	list_for_each_entry(encoder, &crtc->dev->mode_config.encoder_list,
	    head) {
		if (encoder->crtc == crtc) {
			output = container_of(encoder, struct tegra_drm_encoder,
			    encoder);
			break;
		}
	}
	if (output == NULL)
		return (-ENODEV);

	if (output->setup_clock == NULL)
		panic("Output have not setup_clock function.\n");
	rv = output->setup_clock(output, sc->clk_dc, pclk);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot setup pixel clock: %llu\n",
		    pclk);
		return (rv);
	}

	rv = clk_get_freq(sc->clk_dc, &freq);
	*div = (freq * 2 / pclk) - 2;

	DRM_DEBUG_KMS("frequency: %llu, DC divider: %u\n", freq, *div);

	return 0;
}

static void
dc_setup_window(struct dc_softc *sc, unsigned int index, struct dc_window *win)
{
	uint32_t h_offset, v_offset, h_size, v_size, bpp;
	uint32_t h_init_dda, v_init_dda, h_incr_dda, v_incr_dda;
	uint32_t val;

	DRM_TRACE();
#ifdef DMR_DEBUG_WINDOW
	printf("%s window: %d\n", __func__, index);
	printf("  src: x: %d, y: %d, w: %d, h: %d\n",
	   win->src_x, win->src_y, win->src_w, win->src_h);
	printf("  dst: x: %d, y: %d, w: %d, h: %d\n",
	   win->dst_x, win->dst_y, win->dst_w, win->dst_h);
	printf("  bpp: %d, color_mode: %d, swap: %d\n",
	   win->bits_per_pixel, win->color_mode, win->swap);
#endif

	if (win->is_yuv)
		bpp = win->is_yuv_planar ? 1 : 2;
	else
		bpp = (win->bits_per_pixel + 7) / 8;

	if (!win->transpose_xy) {
		h_size = win->src_w * bpp;
		v_size = win->src_h;
	} else {
		h_size = win->src_h * bpp;
		v_size = win->src_w;
	}

	h_offset = win->src_x * bpp;;
	v_offset = win->src_y;
	if (win->flip_x) {
		h_offset += win->src_w * bpp - 1;
	}
	if (win->flip_y)
		v_offset += win->src_h - 1;

	/* Adjust offsets for planar yuv modes */
	if (win->is_yuv_planar) {
		h_offset &= ~1;
		if (win->flip_x )
			h_offset |= 1;
		v_offset &= ~1;
		if (win->flip_y )
			v_offset |= 1;
	}

	/* Setup scaling. */
	if (!win->transpose_xy) {
		h_init_dda = dc_scaling_init(win->src_x);
		v_init_dda = dc_scaling_init(win->src_y);
		h_incr_dda = dc_scaling_incr(win->src_w, win->dst_w, 4);
		v_incr_dda = dc_scaling_incr(win->src_h, win->dst_h, 15);
	} else {
		h_init_dda =  dc_scaling_init(win->src_y);
		v_init_dda =  dc_scaling_init(win->src_x);
		h_incr_dda = dc_scaling_incr(win->src_h, win->dst_h, 4);
		v_incr_dda = dc_scaling_incr(win->src_w, win->dst_w, 15);
	}
#ifdef DMR_DEBUG_WINDOW
	printf("\n");
	printf("  bpp: %d, size: h: %d v: %d, offset: h:%d v: %d\n",
	   bpp, h_size, v_size, h_offset, v_offset);
	printf("  init_dda: h: %d v: %d, incr_dda: h: %d v: %d\n",
	   h_init_dda, v_init_dda, h_incr_dda, v_incr_dda);
#endif

	LOCK(sc);

	/* Select target window  */
	val = WINDOW_A_SELECT << index;
	WR4(sc, DC_CMD_DISPLAY_WINDOW_HEADER, val);

	/* Sizes */
	WR4(sc, DC_WIN_POSITION, WIN_POSITION(win->dst_x, win->dst_y));
	WR4(sc, DC_WIN_SIZE, WIN_SIZE(win->dst_w, win->dst_h));
	WR4(sc, DC_WIN_PRESCALED_SIZE, WIN_PRESCALED_SIZE(h_size, v_size));

	/* DDA */
	WR4(sc, DC_WIN_DDA_INCREMENT,
	    WIN_DDA_INCREMENT(h_incr_dda, v_incr_dda));
	WR4(sc, DC_WIN_H_INITIAL_DDA, h_init_dda);
	WR4(sc, DC_WIN_V_INITIAL_DDA, v_init_dda);

	/* Color planes base addresses and strides */
	WR4(sc, DC_WINBUF_START_ADDR, win->base[0]);
	if (win->is_yuv_planar) {
		WR4(sc, DC_WINBUF_START_ADDR_U, win->base[1]);
		WR4(sc, DC_WINBUF_START_ADDR_V, win->base[2]);
		WR4(sc, DC_WIN_LINE_STRIDE,
		     win->stride[1] << 16 | win->stride[0]);
	} else {
		WR4(sc, DC_WIN_LINE_STRIDE, win->stride[0]);
	}

	/* Offsets for rotation and axis flip */
	WR4(sc, DC_WINBUF_ADDR_H_OFFSET, h_offset);
	WR4(sc, DC_WINBUF_ADDR_V_OFFSET, v_offset);

	/* Color format */
	WR4(sc, DC_WIN_COLOR_DEPTH, win->color_mode);
	WR4(sc, DC_WIN_BYTE_SWAP, win->swap);

	/* Tiling */
	val = win->surface_kind;
	if (win->surface_kind == SURFACE_KIND_BL_16B2)
		val |= SURFACE_KIND_BLOCK_HEIGHT(win->block_height);
	WR4(sc, DC_WINBUF_SURFACE_KIND, val);

	/* Color space coefs for YUV modes */
	if (win->is_yuv) {
		WR4(sc, DC_WINC_CSC_YOF,   0x00f0);
		WR4(sc, DC_WINC_CSC_KYRGB, 0x012a);
		WR4(sc, DC_WINC_CSC_KUR,   0x0000);
		WR4(sc, DC_WINC_CSC_KVR,   0x0198);
		WR4(sc, DC_WINC_CSC_KUG,   0x039b);
		WR4(sc, DC_WINC_CSC_KVG,   0x032f);
		WR4(sc, DC_WINC_CSC_KUB,   0x0204);
		WR4(sc, DC_WINC_CSC_KVB,   0x0000);
	}

	val = WIN_ENABLE;
	if (win->is_yuv)
		val |= CSC_ENABLE;
	else if (win->bits_per_pixel < 24)
		val |= COLOR_EXPAND;
	if (win->flip_y)
		val |= V_DIRECTION;
	if (win->flip_x)
		val |= H_DIRECTION;
	if (win->transpose_xy)
		val |= SCAN_COLUMN;
	WR4(sc, DC_WINC_WIN_OPTIONS, val);

#ifdef DMR_DEBUG_WINDOW
	/* Set underflow debug mode -> highlight missing pixels. */
	WR4(sc, DC_WINBUF_UFLOW_CTRL, UFLOW_CTR_ENABLE);
	WR4(sc, DC_WINBUF_UFLOW_DBG_PIXEL, 0xFFFF0000);
#endif

	UNLOCK(sc);
}

/* -------------------------------------------------------------------
 *
 *    Plane functions.
 *
 */
static int
tegra_plane_state_add(struct tegra_plane *plane,
			  struct drm_plane_state *drm_plane_state)
{
	struct drm_crtc_state *crtc_state;
	struct tegra_dc_state *state;
	int rv;

	DRM_TRACE();
	crtc_state = drm_atomic_get_crtc_state(drm_plane_state->state,
	    drm_plane_state->crtc);
	if (IS_ERR(crtc_state))
		return (PTR_ERR(crtc_state));

	/* Check plane state for visibility and calculate clipping bounds */
	rv = drm_atomic_helper_check_plane_state(drm_plane_state, crtc_state,
	    0, INT_MAX, true, true);
	if (rv < 0)
		return (rv);

	state = container_of(crtc_state, struct tegra_dc_state, drm_state);
	state->planes_mask |= 1 << plane->index;

	return (0);
}

static void
dc_plane_destroy(struct drm_plane *drm_plane)
{
	struct tegra_plane *plane;

	DRM_TRACE();
	plane = container_of(drm_plane, struct tegra_plane, drm_plane);
	drm_plane_cleanup(drm_plane);
	free(plane, DRM_MEM_KMS);
}

static void
dc_plane_reset(struct drm_plane *drm_plane)
{
	struct tegra_plane *plane;
	struct dc_plane_state *state;

	DRM_TRACE();
	plane = container_of(drm_plane, struct tegra_plane, drm_plane);
	if (drm_plane->state) {
		__drm_atomic_helper_plane_destroy_state(drm_plane->state);
		free(plane, DRM_MEM_KMS);
	}

	state = malloc(sizeof(*state), DRM_MEM_KMS,M_WAITOK | M_ZERO);
	drm_plane->state = &state->drm_plane_state;
	drm_plane->state->plane = drm_plane;
	drm_plane->state->zpos = plane->index;
	drm_plane->state->normalized_zpos = plane->index;
}

static struct drm_plane_state *
dc_plane_atomic_duplicate_state(struct drm_plane *drm_plane)
{
	struct dc_plane_state *dst;
	struct dc_plane_state *state;

	DRM_TRACE();
	dst = malloc(sizeof(*state), DRM_MEM_KMS,M_WAITOK | M_ZERO);

	__drm_atomic_helper_plane_duplicate_state(drm_plane,
	    &dst->drm_plane_state);

	/* XXXX TODO copy local fields */
	return (&dst->drm_plane_state);
}

static bool
dc_plane_format_mod_supported(struct drm_plane *plane,
	uint32_t format, uint64_t modifier)
{
	const struct drm_format_info *info;

	DRM_TRACE();
	info = drm_format_info(format);
	if (modifier == DRM_FORMAT_MOD_LINEAR)
		return true;
	if (info->num_planes == 1)
		return true;

	return false;
}


static void
dc_plane_atomic_destroy_state(struct drm_plane *drm_plane,
    struct drm_plane_state *drm_plane_state)
{
	struct dc_plane_state *state;

	state = container_of(drm_plane_state, struct dc_plane_state,
	    drm_plane_state);
	__drm_atomic_helper_plane_destroy_state(drm_plane_state);
	kfree(state);
}

const struct drm_plane_funcs dc_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = dc_plane_destroy,
	.reset = dc_plane_reset,
	.atomic_duplicate_state = dc_plane_atomic_duplicate_state,
	.atomic_destroy_state = dc_plane_atomic_destroy_state,
	.format_mod_supported = dc_plane_format_mod_supported,
};

static int
dc_plane_atomic_check(struct drm_plane *drm_plane,
    struct drm_plane_state *drm_plane_state)
{
	struct tegra_plane *plane;
	struct tegra_crtc *crtc;
	struct tegra_fb *fb;
	struct dc_softc *sc;
	struct dc_window win;
	int rv;

	DRM_TRACE();
	if (drm_plane_state->crtc == NULL)
		return (0);

	plane = container_of(drm_plane, struct tegra_plane, drm_plane);
	fb = container_of(drm_plane_state->fb, struct tegra_fb, drm_fb);
	crtc = container_of(drm_plane_state->crtc, struct tegra_crtc, drm_crtc);
	sc = device_get_softc(crtc->dev);

	memset(&win, 0, sizeof(win));
	win.src_x = drm_plane->state->src.x1 >> 16;
	win.src_y = drm_plane->state->src.y1 >> 16;
	win.src_w = drm_rect_width(&drm_plane->state->src) >> 16;
	win.src_h = drm_rect_height(&drm_plane->state->src) >> 16;
	win.dst_x = drm_plane->state->dst.x1;
	win.dst_y = drm_plane->state->dst.y1;
	win.dst_w = drm_rect_width(&drm_plane->state->dst);
	win.dst_h = drm_rect_height(&drm_plane->state->dst);
printf("%s: fb: %p(%p), win: %p, idx: %d\n", __func__, fb, drm_plane_state->fb, &win, plane->index);
	rv = dc_parse_drm_format(fb, &win);
	if (rv != 0) {
		DRM_WARN("unsupported pixel format %d\n",
		    fb->drm_fb.format->format);
		return (rv);
	}

	rv = tegra_plane_state_add(plane, drm_plane_state);
	if (rv != 0)
		return (rv);

	return (0);
}

static void
dc_plane_atomic_disable(struct drm_plane *drm_plane,
    struct drm_plane_state *old_state)
{
	struct tegra_plane *plane;
	struct tegra_crtc *crtc;
	struct dc_softc *sc;
	uint32_t val, idx;

	DRM_TRACE();
	if (old_state == NULL || old_state->crtc == NULL)
		return;

	plane = container_of(drm_plane, struct tegra_plane, drm_plane);
	crtc = container_of(old_state->crtc, struct tegra_crtc, drm_crtc);

	sc = device_get_softc(crtc->dev);
	idx = plane->index;

	LOCK(sc);

	WR4(sc, DC_CMD_DISPLAY_WINDOW_HEADER, WINDOW_A_SELECT << idx);

	val = RD4(sc, DC_WINC_WIN_OPTIONS);
	val &= ~WIN_ENABLE;
	WR4(sc, DC_WINC_WIN_OPTIONS, val);

	UNLOCK(sc);
}

static void
dc_plane_atomic_update(struct drm_plane *drm_plane,
    struct drm_plane_state *old_state)
{
	struct tegra_plane *plane;
	struct tegra_crtc *crtc;
	struct tegra_fb *fb;
	struct dc_softc *sc;
	struct dc_window win;
	int rv;

	DRM_TRACE();
	if (drm_plane->state->crtc == NULL || drm_plane->state->fb == NULL)
		return;

	if (!drm_plane->state->visible) {
		dc_plane_atomic_disable(drm_plane, old_state);
		return;
	}

	plane = container_of(drm_plane, struct tegra_plane, drm_plane);
	fb = container_of(drm_plane->state->fb, struct tegra_fb, drm_fb);
	crtc = container_of(drm_plane->state->crtc,
	    struct tegra_crtc, drm_crtc);
	sc = device_get_softc(crtc->dev);

	memset(&win, 0, sizeof(win));
	win.src_x = drm_plane->state->src.x1 >> 16;
	win.src_y = drm_plane->state->src.y1 >> 16;
	win.src_w = drm_rect_width(&drm_plane->state->src) >> 16;
	win.src_h = drm_rect_height(&drm_plane->state->src) >> 16;
	win.dst_x = drm_plane->state->dst.x1;
	win.dst_y = drm_plane->state->dst.y1;
	win.dst_w = drm_rect_width(&drm_plane->state->dst);
	win.dst_h = drm_rect_height(&drm_plane->state->dst);
	rv = dc_parse_drm_format(fb, &win);
	if (rv != 0) {
		DRM_WARN("unsupported pixel format %d\n",
		    fb->drm_fb.format->format);
	}

	dc_setup_window(sc, plane->index, &win);
}

static const struct drm_plane_helper_funcs dc_plane_helper_funcs = {
	.atomic_check = dc_plane_atomic_check,
	.atomic_disable = dc_plane_atomic_disable,
	.atomic_update = dc_plane_atomic_update,
};

static int
dc_primary_plane_create(struct dc_softc *sc, struct tegra_drm *drm,
    struct drm_plane **out_plane)
{
	int rv;
	struct tegra_plane *plane;

	DRM_TRACE();
	plane = malloc(sizeof(*plane), DRM_MEM_KMS, M_WAITOK | M_ZERO);
	plane->index = 0;
	rv = drm_universal_plane_init(&drm->drm_dev, &plane->drm_plane,
	    1 << sc->tegra_crtc.nvidia_head, &dc_plane_funcs,
	    dc_primary_plane_formats, nitems(dc_primary_plane_formats),
	    plane_modifiers, DRM_PLANE_TYPE_PRIMARY, NULL);
	if (rv != 0) {
		free(plane, DRM_MEM_KMS);
		return (rv);
	}

	drm_plane_helper_add(&plane->drm_plane, &dc_plane_helper_funcs);
	drm_plane_create_zpos_property(&plane->drm_plane, plane->index, 0, 255);

	rv = drm_plane_create_rotation_property(&plane->drm_plane,
	    DRM_MODE_ROTATE_0, DRM_MODE_ROTATE_0 | DRM_MODE_REFLECT_Y);

	*out_plane = &plane->drm_plane;
	return (0);
}


static int
dc_overlay_plane_create(struct dc_softc *sc, struct tegra_drm *drm, int idx)
{
	int rv;
	struct tegra_plane *plane;

	DRM_TRACE();
	plane = malloc(sizeof(*plane), DRM_MEM_KMS, M_WAITOK | M_ZERO);
	plane->index = idx;
	rv = drm_universal_plane_init(&drm->drm_dev, &plane->drm_plane,
	    1 << sc->tegra_crtc.nvidia_head, &dc_plane_funcs,
	    dc_overlay_plane_formats, nitems(dc_overlay_plane_formats),
	    NULL, DRM_PLANE_TYPE_OVERLAY, NULL);
	if (rv != 0) {
		free(plane, DRM_MEM_KMS);
		return (rv);
	}
	drm_plane_helper_add(&plane->drm_plane, &dc_plane_helper_funcs);
	drm_plane_create_zpos_property(&plane->drm_plane, plane->index, 0, 255);

	rv = drm_plane_create_rotation_property(&plane->drm_plane,
	    DRM_MODE_ROTATE_0, DRM_MODE_ROTATE_0 | DRM_MODE_REFLECT_Y);

	return (0);
}

static int
dc_cursor_atomic_check(struct drm_plane *drm_plane,
    struct drm_plane_state *drm_plane_state)
{
	struct tegra_plane *plane;
	int rv;

	DRM_TRACE();
	if (drm_plane_state->crtc == NULL)
		return (0);

	plane = container_of(drm_plane, struct tegra_plane, drm_plane);

	if (drm_plane_state->src_w != drm_plane_state->src_h)
		return (-EINVAL);
	if ((drm_plane_state->src_w >> 16 != drm_plane_state->crtc_w) ||
	    (drm_plane_state->src_h >> 16 != drm_plane_state->crtc_h))
		return (-EINVAL);


	switch (drm_plane_state->crtc_w) {
	case 32:
	case 64:
	case 128:
	case 256:
		break;
	default:
		return (-EINVAL);
	}

	rv = tegra_plane_state_add(plane, drm_plane_state);
	if (rv != 0)
		return (rv);

	return (0);
}

static void
dc_cursor_atomic_update(struct drm_plane *drm_plane,
    struct drm_plane_state *old_state)
{
	struct tegra_plane *plane;
	struct tegra_crtc *crtc;
	struct tegra_fb *fb;
	struct dc_softc *sc;
	struct tegra_bo *bo;
	uint32_t val;

	DRM_TRACE();
	if (drm_plane->state->crtc == NULL || drm_plane->state->fb == NULL)
		return;

	plane = container_of(drm_plane, struct tegra_plane, drm_plane);
	crtc = container_of(drm_plane->state->crtc, struct tegra_crtc, drm_crtc);
	fb = container_of(drm_plane->state->fb, struct tegra_fb, drm_fb);
	sc = device_get_softc(crtc->dev);

	bo = tegra_fb_get_plane(fb, 0);


	switch (drm_plane->state->crtc_w) {
	case 32:
		val = CURSOR_SIZE(C32x32);
		break;
	case 64:
		val = CURSOR_SIZE(C64x64);
		break;
	case 128:
		val = CURSOR_SIZE(C128x128);
		break;
	case 256:
		val = CURSOR_SIZE(C256x256);
		break;
	default:
		panic("Invalid cursor size");;
	}

	val |= CURSOR_CLIP(CC_DISPLAY);
	val |= CURSOR_START_ADDR(bo->pbase);
	WR4(sc, DC_DISP_CURSOR_START_ADDR, val);

	val = RD4(sc, DC_DISP_BLEND_CURSOR_CONTROL);
	val &= ~CURSOR_DST_BLEND_FACTOR_SELECT(~0);
	val &= ~CURSOR_SRC_BLEND_FACTOR_SELECT(~0);
	val |= CURSOR_MODE_SELECT;
	val |= CURSOR_DST_BLEND_FACTOR_SELECT(DST_NEG_K1_TIMES_SRC);
	val |= CURSOR_SRC_BLEND_FACTOR_SELECT(SRC_BLEND_K1_TIMES_SRC);
	val |= CURSOR_ALPHA(~0);
	WR4(sc, DC_DISP_BLEND_CURSOR_CONTROL, val);

	val = RD4(sc, DC_DISP_DISP_WIN_OPTIONS);
	val |= CURSOR_ENABLE;
	WR4(sc, DC_DISP_DISP_WIN_OPTIONS, val);

   	WR4(sc, DC_DISP_CURSOR_POSITION,
   	    CURSOR_POSITION(drm_plane->state->crtc_x, drm_plane->state->crtc_y));

	/* XXX This fixes cursor underflow issues, but why ?  */
	WR4(sc, DC_DISP_CURSOR_UNDERFLOW_CTRL, CURSOR_UFLOW_CYA);
}

static void
dc_cursor_atomic_disable(struct drm_plane *drm_plane,
    struct drm_plane_state *old_state)
{
	struct tegra_plane *plane;
	struct tegra_crtc *crtc;
	struct dc_softc *sc;
	uint32_t val;

	DRM_TRACE();
	if (old_state == NULL || old_state->crtc == NULL)
		return;

	plane = container_of(drm_plane, struct tegra_plane, drm_plane);
	crtc = container_of(old_state->crtc, struct tegra_crtc, drm_crtc);

	sc = device_get_softc(crtc->dev);

	LOCK(sc);
	val = RD4(sc, DC_DISP_DISP_WIN_OPTIONS);
	val &= ~CURSOR_ENABLE;
	WR4(sc, DC_DISP_DISP_WIN_OPTIONS, val);
	UNLOCK(sc);
}

static const struct drm_plane_helper_funcs dc_cursor_plane_helper_funcs = {
	.atomic_check = dc_cursor_atomic_check,
	.atomic_update = dc_cursor_atomic_update,
	.atomic_disable = dc_cursor_atomic_disable,
};


static int
dc_cursor_plane_create(struct dc_softc *sc, struct tegra_drm *drm,
    struct drm_plane **out_plane)
{
	int rv;
	struct tegra_plane *plane;

	DRM_TRACE();
	plane = malloc(sizeof(*plane), DRM_MEM_KMS, M_WAITOK | M_ZERO);
	plane->index = 6; 	/* for WINDOW_ENABLE index */
	rv = drm_universal_plane_init(&drm->drm_dev, &plane->drm_plane,
	    1 << sc->tegra_crtc.nvidia_head, &dc_plane_funcs,
	    dc_cursor_plane_formats, nitems(dc_cursor_plane_formats),
	    NULL,  DRM_PLANE_TYPE_CURSOR, NULL);
	if (rv != 0) {
		free(plane, DRM_MEM_KMS);
		return (rv);
	}

	drm_plane_helper_add(&plane->drm_plane, &dc_cursor_plane_helper_funcs);
	*out_plane = &plane->drm_plane;
	return (0);
}



/* -------------------------------------------------------------------
 *
 *    CRTC helper functions.
 *
 */
static int
dc_crtc_mode_set(struct drm_crtc *drm_crtc, struct drm_display_mode *mode)
{
	struct dc_softc *sc;
	struct tegra_crtc *crtc;
	struct tegra_fb *fb;
	uint32_t div, h_ref_to_sync, v_ref_to_sync;
	int rv;

	DRM_TRACE();
	crtc = container_of(drm_crtc, struct tegra_crtc, drm_crtc);
	sc = device_get_softc(crtc->dev);
	fb = container_of(drm_crtc->primary->fb, struct tegra_fb, drm_fb);

	h_ref_to_sync = 1;
	v_ref_to_sync = 1;

	/* Setup clocks */
	rv = dc_setup_clk(sc, drm_crtc, mode, &div);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot set pixel clock\n");
		return (rv);
	}

	/* Timing */
	WR4(sc, DC_DISP_DISP_TIMING_OPTIONS, 0);

	WR4(sc, DC_DISP_REF_TO_SYNC,
	    (v_ref_to_sync << 16) |
	     h_ref_to_sync);

	WR4(sc, DC_DISP_SYNC_WIDTH,
	    ((mode->vsync_end - mode->vsync_start) << 16) |
	    ((mode->hsync_end - mode->hsync_start) <<  0));

	WR4(sc, DC_DISP_BACK_PORCH,
	    ((mode->vtotal - mode->vsync_end) << 16) |
	    ((mode->htotal - mode->hsync_end) <<  0));

	WR4(sc, DC_DISP_FRONT_PORCH,
	    ((mode->vsync_start - mode->vdisplay) << 16) |
	    ((mode->hsync_start - mode->hdisplay) <<  0));

	WR4(sc, DC_DISP_DISP_ACTIVE,
	    (mode->vdisplay << 16) | mode->hdisplay);

	WR4(sc, DC_DISP_DISP_INTERFACE_CONTROL, DISP_DATA_FORMAT(DF1P1C));

	WR4(sc,DC_DISP_DISP_CLOCK_CONTROL,
	    SHIFT_CLK_DIVIDER(div) | PIXEL_CLK_DIVIDER(PCD1));

	return (0);

}

static void
dc_crtc_prepare(struct drm_crtc *drm_crtc)
{
	struct dc_softc *sc;
	struct tegra_crtc *crtc;
	uint32_t val;

	DRM_TRACE();
	crtc = container_of(drm_crtc, struct tegra_crtc, drm_crtc);
	sc = device_get_softc(crtc->dev);

	WR4(sc, DC_CMD_GENERAL_INCR_SYNCPT_CNTRL, SYNCPT_CNTRL_NO_STALL);
	/* XXX allocate syncpoint from host1x */
	WR4(sc, DC_CMD_CONT_SYNCPT_VSYNC, SYNCPT_VSYNC_ENABLE |
	    (sc->tegra_crtc.nvidia_head == 0 ? SYNCPT_VBLANK0: SYNCPT_VBLANK1));

	WR4(sc, DC_CMD_DISPLAY_POWER_CONTROL,
	    PW0_ENABLE | PW1_ENABLE | PW2_ENABLE | PW3_ENABLE |
	    PW4_ENABLE | PM0_ENABLE | PM1_ENABLE);

	val = RD4(sc, DC_CMD_DISPLAY_COMMAND);
	val |= DISPLAY_CTRL_MODE(CTRL_MODE_C_DISPLAY);
	WR4(sc, DC_CMD_DISPLAY_COMMAND, val);

	WR4(sc, DC_CMD_INT_MASK,
	    WIN_A_UF_INT | WIN_B_UF_INT | WIN_C_UF_INT |
	    WIN_A_OF_INT | WIN_B_OF_INT | WIN_C_OF_INT);

	WR4(sc, DC_CMD_INT_ENABLE,
	    VBLANK_INT | WIN_A_UF_INT | WIN_B_UF_INT | WIN_C_UF_INT |
	    WIN_A_OF_INT | WIN_B_OF_INT | WIN_C_OF_INT);
}

static void
dc_atomic_begin(struct drm_crtc *drm_crtc,
    struct drm_crtc_state *old_crtc_state)
{
	uint32_t irqflags;

	DRM_TRACE();
	if (drm_crtc->state->event == NULL)
		return;

	spin_lock_irqsave(&drm_crtc->dev->event_lock, irqflags);

	if (drm_crtc_vblank_get(drm_crtc) != 0)
		drm_crtc_send_vblank_event(drm_crtc, drm_crtc->state->event);
	else
		drm_crtc_arm_vblank_event(drm_crtc, drm_crtc->state->event);

	drm_crtc->state->event = NULL;
	spin_unlock_irqrestore(&drm_crtc->dev->event_lock, irqflags);
}

static void
dc_atomic_flush(struct drm_crtc *drm_crtc,
    struct drm_crtc_state *old_crtc_state)
{
	struct dc_softc *sc;
	struct tegra_crtc *crtc;
	struct tegra_dc_state *state;
	uint32_t val;

	DRM_TRACE();
	crtc = container_of(drm_crtc, struct tegra_crtc, drm_crtc);
	state = container_of(drm_crtc->state, struct tegra_dc_state, drm_state);
	sc = device_get_softc(crtc->dev);

printf("%s: Planes mask: 0x%08X\n", __func__, state->planes_mask);
	val = GENERAL_UPDATE | state->planes_mask << 9;
	WR4(sc, DC_CMD_STATE_CONTROL, val);

	val = RD4(sc, DC_CMD_INT_MASK);
	val |= FRAME_END_INT;
	WR4(sc, DC_CMD_INT_MASK, val);

	val = RD4(sc, DC_CMD_INT_ENABLE);
	val |= FRAME_END_INT;
	WR4(sc, DC_CMD_INT_ENABLE, val);

	val = GENERAL_ACT_REQ | state->planes_mask << 1;
	WR4(sc, DC_CMD_STATE_CONTROL, val);
}

static void
dc_atomic_enable(struct drm_crtc *drm_crtc, struct drm_crtc_state *old_state)
{
	struct dc_softc *sc;
	struct tegra_crtc *crtc;
	struct drm_display_mode *mode;

	DRM_TRACE();
	crtc = container_of(drm_crtc, struct tegra_crtc, drm_crtc);
	mode = &drm_crtc->state->adjusted_mode;
	sc = device_get_softc(crtc->dev);

	dc_crtc_prepare(drm_crtc);
	dc_crtc_mode_set(drm_crtc, mode);

	/* Commit all */
	WR4(sc, DC_CMD_STATE_CONTROL, GENERAL_UPDATE );
	WR4(sc, DC_CMD_STATE_CONTROL, GENERAL_ACT_REQ);

	drm_crtc_vblank_on(drm_crtc);
}

static void
dc_atomic_disable(struct drm_crtc *drm_crtc, struct drm_crtc_state *old_state)
{
	struct dc_softc *sc;
	struct tegra_crtc *crtc;
	uint32_t val, irqflags;

	DRM_TRACE();
	crtc = container_of(drm_crtc, struct tegra_crtc, drm_crtc);
	sc = device_get_softc(crtc->dev);

	/* XXX TODO: Wait until DC is idle */

	/* Stop display */
	val = RD4(sc, DC_CMD_DISPLAY_COMMAND);
	val &= ~DISPLAY_CTRL_MODE(~0);
	val |= DISPLAY_CTRL_MODE(CTRL_MODE_STOP);
	WR4(sc, DC_CMD_DISPLAY_COMMAND, val);

	/* Power down display */
	val = RD4(sc, DC_CMD_DISPLAY_POWER_CONTROL);
	val &= PW0_ENABLE | PW1_ENABLE | PW2_ENABLE | PW3_ENABLE | PW4_ENABLE |
	    PM0_ENABLE | PM1_ENABLE;
	WR4(sc, DC_CMD_DISPLAY_POWER_CONTROL, val);

	drm_crtc_vblank_off(drm_crtc);

	spin_lock_irqsave(&drm_crtc->dev->event_lock, irqflags);

	if (drm_crtc->state->event) {
		drm_crtc_send_vblank_event(drm_crtc, drm_crtc->state->event);
		drm_crtc->state->event = NULL;
	}

	spin_unlock_irqrestore(&drm_crtc->dev->event_lock, irqflags);

}

static const struct drm_crtc_helper_funcs dc_crtc_helper_funcs = {
	.atomic_begin = dc_atomic_begin,
	.atomic_flush = dc_atomic_flush,
	.atomic_enable = dc_atomic_enable,
	.atomic_disable = dc_atomic_disable,

};

/* -------------------------------------------------------------------
 *
 *   Exported functions (mainly vsync related).
 *
 */
void
tegra_dc_cancel_page_flip(struct drm_crtc *drm_crtc, struct drm_file *file)
{
	struct dc_softc *sc;
	struct tegra_crtc *crtc;
	struct drm_device *drm;
	unsigned long flags;

	DRM_TRACE();
	crtc = container_of(drm_crtc, struct tegra_crtc, drm_crtc);
	sc = device_get_softc(crtc->dev);
	drm = drm_crtc->dev;
	spin_lock_irqsave(&drm->event_lock, flags);

	if ((sc->event != NULL) && (sc->event->base.file_priv == file)) {
//		drm_event_cancel_free(crtc->dev, &e->base);
//		sc->event->base.destroy(&sc->event->base);
		drm_crtc_vblank_put(drm_crtc);
		sc->event = NULL;
	}
	spin_unlock_irqrestore(&drm->event_lock, flags);
}

/* -------------------------------------------------------------------
 *
 *    CRTC functions.
 *
 */

static void
dc_destroy(struct drm_crtc *crtc)
{

	DRM_TRACE();
	drm_crtc_cleanup(crtc);
	memset(crtc, 0, sizeof(*crtc));
}

static void
dc_reset(struct drm_crtc *drm_crtc)
{
	struct tegra_dc_state *state;

	DRM_TRACE();
	if (drm_crtc->state != NULL) {
		state = container_of(drm_crtc->state, struct tegra_dc_state,
		    drm_state);
		__drm_atomic_helper_crtc_destroy_state(drm_crtc->state);
		free(state, DRM_MEM_KMS);
	}

	state =  malloc(sizeof(*state), DRM_MEM_DRIVER, M_WAITOK | M_ZERO);
	drm_crtc->state = &state->drm_state;
	drm_crtc->state->crtc = drm_crtc;
	drm_crtc_vblank_reset(drm_crtc);
}

static struct drm_crtc_state *
dc_atomic_duplicate_state(struct drm_crtc *drm_crtc)
{
	struct tegra_dc_state *src, *dst;

	DRM_TRACE();
	src = container_of(drm_crtc->state, struct tegra_dc_state, drm_state);

	dst = malloc(sizeof(*dst), DRM_MEM_DRIVER, M_WAITOK | M_ZERO);

	__drm_atomic_helper_crtc_duplicate_state(drm_crtc, &dst->drm_state);

	dst->clk = src->clk;
	dst->pclk = src->pclk;
	dst->div = src->div;
	dst->planes_mask = src->planes_mask;

	return (&dst->drm_state);
}

static void
dc_atomic_destroy_state(struct drm_crtc *drm_crtc,
    struct drm_crtc_state *drm_state)
{
	struct tegra_dc_state *state;

	DRM_TRACE();
	state = container_of(drm_state, struct tegra_dc_state, drm_state);
	__drm_atomic_helper_crtc_destroy_state(drm_state);
	free(state, DRM_MEM_KMS);
}

static uint32_t
dc_get_vblank_counter(struct drm_crtc *drm_crtc)
{
	struct tegra_crtc *crtc;

	crtc = container_of(drm_crtc, struct tegra_crtc, drm_crtc);
	/* XXXX read counter from syncpt engine */
	return (drm_crtc_vblank_count(drm_crtc));
}

static int
dc_enable_vblank(struct drm_crtc *drm_crtc)
{
	struct dc_softc *sc;
	struct tegra_crtc *crtc;
	uint32_t val;

	DRM_TRACE();
	crtc = container_of(drm_crtc, struct tegra_crtc, drm_crtc);
	sc = device_get_softc(crtc->dev);

	LOCK(sc);
	val = RD4(sc, DC_CMD_INT_MASK);
	val |= VBLANK_INT;
	WR4(sc, DC_CMD_INT_MASK, val);
	UNLOCK(sc);

	return(0);
}

static void
dc_disable_vblank(struct drm_crtc *drm_crtc)
{
	struct dc_softc *sc;
	struct tegra_crtc *crtc;
	uint32_t val;

	DRM_TRACE();
	crtc = container_of(drm_crtc, struct tegra_crtc, drm_crtc);
	sc = device_get_softc(crtc->dev);

	LOCK(sc);
	val = RD4(sc, DC_CMD_INT_MASK);
	val &= ~VBLANK_INT;
	WR4(sc, DC_CMD_INT_MASK, val);
	UNLOCK(sc);
}

static const struct drm_crtc_funcs dc_crtc_funcs = {
	.page_flip = drm_atomic_helper_page_flip,
	.set_config = drm_atomic_helper_set_config,
	.destroy = dc_destroy,
	.reset = dc_reset,

	.atomic_duplicate_state = dc_atomic_duplicate_state,
	.atomic_destroy_state = dc_atomic_destroy_state,

	.get_vblank_counter = dc_get_vblank_counter,
	.enable_vblank = dc_enable_vblank,
	.disable_vblank = dc_disable_vblank,
};

/* -------------------------------------------------------------------
 *
 *    Bus and infrastructure.
 *
 */

static void
dc_display_enable(device_t dev, bool enable)
{
	struct dc_softc *sc;
	uint32_t val;

	DRM_TRACE();
	sc = device_get_softc(dev);

	/* Set display mode */
	val = enable ? CTRL_MODE_C_DISPLAY: CTRL_MODE_STOP;
	WR4(sc, DC_CMD_DISPLAY_COMMAND, DISPLAY_CTRL_MODE(val));

	/* and commit it*/
	WR4(sc, DC_CMD_STATE_CONTROL, GENERAL_UPDATE);
	WR4(sc, DC_CMD_STATE_CONTROL, GENERAL_ACT_REQ);
}

static void
dc_hdmi_enable(device_t dev, bool enable)
{
	struct dc_softc *sc;
	uint32_t val;

	DRM_TRACE();
	sc = device_get_softc(dev);

	val = RD4(sc, DC_DISP_DISP_WIN_OPTIONS);
	if (enable)
		val |= HDMI_ENABLE;
	else
		val &= ~HDMI_ENABLE;
	WR4(sc, DC_DISP_DISP_WIN_OPTIONS, val);

}

static void
dc_setup_timing(device_t dev, int h_pulse_start)
{
	struct dc_softc *sc;

	DRM_TRACE();
	sc = device_get_softc(dev);

	/* Setup display timing */
	WR4(sc, DC_DISP_DISP_TIMING_OPTIONS, VSYNC_H_POSITION(1));
	WR4(sc, DC_DISP_DISP_COLOR_CONTROL,
	    DITHER_CONTROL(DITHER_DISABLE) | BASE_COLOR_SIZE(SIZE_BASE888));

	WR4(sc, DC_DISP_DISP_SIGNAL_OPTIONS0, H_PULSE2_ENABLE);
	WR4(sc, DC_DISP_H_PULSE2_CONTROL,
	    PULSE_CONTROL_QUAL(QUAL_VACTIVE) | PULSE_CONTROL_LAST(LAST_END_A));

	WR4(sc, DC_DISP_H_PULSE2_POSITION_A,
	    PULSE_START(h_pulse_start) | PULSE_END(h_pulse_start + 8));
}

static void
dc_intr(void *arg)
{
	struct dc_softc *sc;
	uint32_t status;

	sc = arg;

	/* Confirm interrupt */
	status = RD4(sc, DC_CMD_INT_STATUS);
	WR4(sc, DC_CMD_INT_STATUS, status);
	if (status & VBLANK_INT) {
		drm_crtc_handle_vblank(&sc->tegra_crtc.drm_crtc);
	}
}

static int
dc_init_client(device_t dev, device_t host1x, struct tegra_drm *drm)
{
	struct dc_softc *sc;
	struct drm_plane *primary, *cursor;
	int i, rv;

	DRM_TRACE();
	sc = device_get_softc(dev);

	if (drm->pitch_align < sc->pitch_align)
		drm->pitch_align = sc->pitch_align;

	rv = dc_primary_plane_create(sc, drm, &primary);
	if (rv!= 0){
		device_printf(dev, "Cannot create primary plane\n");
		return (rv);
	}

	rv = dc_cursor_plane_create(sc, drm, &cursor);
	if (rv!= 0){
		device_printf(dev, "Cannot create cursor plane\n");
		return (rv);
	}

	for (i = 0; i < DC_MAX_PLANES; i++) {
		dc_overlay_plane_create(sc, drm, i + 1);
		if (rv != 0) {
			device_printf(dev, "Cannot create overlay plane\n");
		return (rv);
		}
	}

	rv = drm_crtc_init_with_planes(&drm->drm_dev, &sc->tegra_crtc.drm_crtc,
	    primary, cursor, &dc_crtc_funcs, NULL);
	if (rv!= 0){
		device_printf(dev, "Cannot init drm_crtc\n");
		return (rv);
	}

	drm_mode_crtc_set_gamma_size(&sc->tegra_crtc.drm_crtc, 256);
	drm_crtc_helper_add(&sc->tegra_crtc.drm_crtc, &dc_crtc_helper_funcs);


	WR4(sc, DC_CMD_INT_TYPE,
	    WIN_A_UF_INT | WIN_B_UF_INT | WIN_C_UF_INT |
	    WIN_A_OF_INT | WIN_B_OF_INT | WIN_C_OF_INT);

	WR4(sc, DC_CMD_INT_POLARITY,
	    WIN_A_UF_INT | WIN_B_UF_INT | WIN_C_UF_INT |
	    WIN_A_OF_INT | WIN_B_OF_INT | WIN_C_OF_INT);

	WR4(sc, DC_CMD_INT_ENABLE, 0);
	WR4(sc, DC_CMD_INT_MASK, 0);

	rv = bus_setup_intr(dev, sc->irq_res, INTR_TYPE_MISC | INTR_MPSAFE,
	    NULL, dc_intr, sc, &sc->irq_ih);
	if (rv != 0) {
		device_printf(dev, "Cannot register interrupt handler\n");
		return (rv);
	}

	return (0);
}

static int
dc_exit_client(device_t dev, device_t host1x, struct tegra_drm *drm)
{
	struct dc_softc *sc;

	sc = device_get_softc(dev);

	if (sc->irq_ih != NULL)
		bus_teardown_intr(dev, sc->irq_res, sc->irq_ih);
	sc->irq_ih = NULL;

	return (0);
}

static int
get_fdt_resources(struct dc_softc *sc, phandle_t node)
{
	int rv;

	rv = hwreset_get_by_ofw_name(sc->dev, 0, "dc", &sc->hwreset_dc);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot get 'dc' reset\n");
		return (rv);
	}
	rv = clk_get_by_ofw_name(sc->dev, 0, "parent", &sc->clk_parent);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot get 'parent' clock\n");
		return (rv);
	}
	rv = clk_get_by_ofw_name(sc->dev, 0, "dc", &sc->clk_dc);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot get 'dc' clock\n");
		return (rv);
	}

	rv = OF_getencprop(node, "nvidia,head", &sc->tegra_crtc.nvidia_head,
	    sizeof(sc->tegra_crtc.nvidia_head));
	if (rv <= 0) {
		device_printf(sc->dev,
		    "Cannot get 'nvidia,head' property\n");
		return (rv);
	}
	return (0);
}

static int
enable_fdt_resources(struct dc_softc *sc)
{
	int id, rv;

	rv = clk_set_parent_by_clk(sc->clk_dc, sc->clk_parent);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot set parent for 'dc' clock\n");
		return (rv);
	}

	id = (sc->tegra_crtc.nvidia_head == 0) ?
	    TEGRA_POWERGATE_DIS: TEGRA_POWERGATE_DISB;
	rv = tegra_powergate_sequence_power_up(id, sc->clk_dc, sc->hwreset_dc);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot enable 'DIS' powergate\n");
		return (rv);
	}

	return (0);
}

static int
dc_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Tegra Display Controller");
	return (BUS_PROBE_DEFAULT);
}

static int
dc_attach(device_t dev)
{
	struct dc_softc *sc;
	phandle_t node;
	int rid, rv;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->tegra_crtc.dev = dev;

	node = ofw_bus_get_node(sc->dev);
	LOCK_INIT(sc);

	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "Cannot allocate memory resources\n");
		goto fail;
	}

	rid = 0;
	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid, RF_ACTIVE);
	if (sc->irq_res == NULL) {
		device_printf(dev, "Cannot allocate IRQ resources\n");
		goto fail;
	}

	rv = get_fdt_resources(sc, node);
	if (rv != 0) {
		device_printf(dev, "Cannot parse FDT resources\n");
		goto fail;
	}
	rv = enable_fdt_resources(sc);
	if (rv != 0) {
		device_printf(dev, "Cannot enable FDT resources\n");
		goto fail;
	}

	/*
	 * Tegra124
	 *  -  64 for RGB modes
	 *  - 128 for YUV planar modes
	 *  - 256 for block linear modes
	 */
	sc->pitch_align = 256;

	rv = TEGRA_DRM_REGISTER_CLIENT(device_get_parent(sc->dev), sc->dev);
	if (rv != 0) {
		device_printf(dev, "Cannot register DRM device\n");
		goto fail;
	}

	return (bus_generic_attach(dev));

fail:
	TEGRA_DRM_DEREGISTER_CLIENT(device_get_parent(sc->dev), sc->dev);
	if (sc->irq_ih != NULL)
		bus_teardown_intr(dev, sc->irq_res, sc->irq_ih);
	if (sc->clk_parent != NULL)
		clk_release(sc->clk_parent);
	if (sc->clk_dc != NULL)
		clk_release(sc->clk_dc);
	if (sc->hwreset_dc != NULL)
		hwreset_release(sc->hwreset_dc);
	if (sc->irq_res != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq_res);
	if (sc->mem_res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);
	LOCK_DESTROY(sc);

	return (ENXIO);
}

static int
dc_detach(device_t dev)
{
	struct dc_softc *sc;

	sc = device_get_softc(dev);

	TEGRA_DRM_DEREGISTER_CLIENT(device_get_parent(sc->dev), sc->dev);

	if (sc->irq_ih != NULL)
		bus_teardown_intr(dev, sc->irq_res, sc->irq_ih);
	if (sc->clk_parent != NULL)
		clk_release(sc->clk_parent);
	if (sc->clk_dc != NULL)
		clk_release(sc->clk_dc);
	if (sc->hwreset_dc != NULL)
		hwreset_release(sc->hwreset_dc);
	if (sc->irq_res != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq_res);
	if (sc->mem_res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);
	LOCK_DESTROY(sc);

	return (bus_generic_detach(dev));
}

static device_method_t tegra_dc_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			dc_probe),
	DEVMETHOD(device_attach,		dc_attach),
	DEVMETHOD(device_detach,		dc_detach),

	/* tegra drm interface */
	DEVMETHOD(tegra_drm_init_client,	dc_init_client),
	DEVMETHOD(tegra_drm_exit_client,	dc_exit_client),

	/* tegra dc interface */
	DEVMETHOD(tegra_dc_display_enable,	dc_display_enable),
	DEVMETHOD(tegra_dc_hdmi_enable,		dc_hdmi_enable),
	DEVMETHOD(tegra_dc_setup_timing,	dc_setup_timing),

	DEVMETHOD_END
};

static devclass_t tegra_dc_devclass;
DEFINE_CLASS_0(tegra_dc, tegra_dc_driver, tegra_dc_methods,
    sizeof(struct dc_softc));
DRIVER_MODULE(tegra_dc, host1x, tegra_dc_driver, tegra_dc_devclass, NULL, NULL);
