/*
 * CDDL HEADER START
 *
 * This file and its contents are supplied under the terms of the
 * Common Development and Distribution License ("CDDL"), version 1.0.
 * You may only use this file in accordance with the terms of version
 * 1.0 of the CDDL.
 *
 * A full copy of the text of the CDDL should have accompanied this
 * source.  A copy of the CDDL is also available via the Internet at
 * http://www.illumos.org/license/CDDL.
 *
 * CDDL HEADER END
*/
/*
 * Copyright 2018 Saso Kiselkov. All rights reserved.
 */

#include <string.h>
#include <stdlib.h>

#include <GL/glew.h>
#include <cglm/cglm.h>

#include <XPLMGraphics.h>
#include <XPLMDisplay.h>

#include <acfutils/assert.h>
#include <acfutils/dr.h>
#include <acfutils/geom.h>
#include <acfutils/glutils.h>
#include <acfutils/helpers.h>
#include <acfutils/math.h>
#include <acfutils/perf.h>
#include <acfutils/png.h>
#include <acfutils/safe_alloc.h>
#include <acfutils/shader.h>
#include <acfutils/thread.h>
#include <acfutils/time.h>

#include "atmo_xp11.h"
#include "xplane.h"

#define	UPD_INTVAL	100000		/* us */

#define	EFIS_WIDTH	194
#define	EFIS_LAT_PIX	(EFIS_WIDTH / 2)
#define	EFIS_LON_AFT	134
#define	EFIS_LON_FWD	134
#define	EFIS_HEIGHT	(EFIS_LON_FWD + EFIS_LON_AFT)
#define	WX_SMOOTH_RNG	300		/* meters */

static void atmo_xp11_set_range(double range);
static void atmo_xp11_probe(scan_line_t *sl);

static bool_t inited = B_FALSE;
static atmo_t atmo = {
	.set_range = atmo_xp11_set_range,
	.probe = atmo_xp11_probe
};
static XPLMCommandRef debug_cmd = NULL;

enum {
	XPLANE_RENDER_GAUGES_2D = 0,
	XPLANE_RENDER_GAUGES_3D_UNLIT,
	XPLANE_RENDER_GAUGES_3D_LIT
};

typedef enum {
	EFIS_MAP_RANGE_2_5NM,
	EFIS_MAP_RANGE_5NM,
	EFIS_MAP_RANGE_10NM,
	EFIS_MAP_RANGE_20NM,
	EFIS_MAP_RANGE_40NM,
	EFIS_MAP_RANGE_80NM,
	EFIS_MAP_RANGE_160NM,
	EFIS_MAP_NUM_RANGES
} efis_map_range_t;

/* Must follow order of efis_map_range_t */
static double efis_map_ranges[EFIS_MAP_NUM_RANGES] = {
	NM2MET(2.5),
	NM2MET(5),
	NM2MET(10),
	NM2MET(20),
	NM2MET(40),
	NM2MET(80),
	NM2MET(160)
};

static struct {
	mutex_t		lock;

	/* protected by lock */
	uint32_t	*pixels;
	double		range;
	unsigned	range_i;
	vect2_t		precip_nodes[5];

	/* only accessed by foreground drawing thread */
	uint64_t	last_update;
	unsigned	efis_x;
	unsigned	efis_y;
	unsigned	efis_w;
	unsigned	efis_h;
	mat4		efis_pvm;
	glutils_quads_t	efis_quads;
	GLuint		pbo;
	GLuint		tmp_tex[3];
	GLuint		tmp_fbo[3];
	GLsync		xfer_sync;
	GLuint		smooth_prog;
	GLuint		cleanup_prog;
} xp11_atmo;

typedef enum {
	XP11_CLOUD_CLEAR = 0,
	XP11_CLOUD_HIGH_CIRRUS = 1,
	XP11_CLOUD_SCATTERED = 2,
	XP11_CLOUD_BROKEN = 3,
	XP11_CLOUD_OVERCAST = 4,
	XP11_CLOUD_STRATUS = 5
} xp11_cloud_type_t;

static struct {
	dr_t	cloud_type[3];		/* xp11_cloud_type_t */
	dr_t	cloud_cover[3];		/* enum, 0..6 */
	dr_t	cloud_base[3];		/* meters MSL */
	dr_t	cloud_tops[3];		/* meters MSL */
	dr_t	wind_alt[3];		/* meters MSL */
	dr_t	wind_dir[3];		/* degrees true */
	dr_t	wind_spd[3];		/* knots */
	dr_t	wind_turb[3];		/* ratio 0..10 */
	dr_t	shear_dir[3];		/* degrees relative */
	dr_t	shear_spd[3];		/* knots */
	dr_t	turb;			/* ratio 0..1 */
	dr_t	render_type;

	dr_t	temp_sl;
	dr_t	temp_tropo;
	dr_t	alt_tropo;

	struct {
		dr_t	instr_brt;
		dr_t	mode;
		dr_t	submode;
		dr_t	range;
		dr_t	shows_wx;
		dr_t	wx_alpha;
		dr_t	shows_tcas;
		dr_t	shows_arpts;
		dr_t	shows_wpts;
		dr_t	shows_VORs;
		dr_t	shows_NDBs;
		dr_t	kill_map_fms_line;
	} EFIS;
} drs;

static int
debug_cmd_handler(XPLMCommandRef ref, XPLMCommandPhase phase, void *refcon)
{
	uint8_t buf[EFIS_WIDTH * EFIS_HEIGHT * 4];
	GLint old_read_fbo;

	UNUSED(ref);
	UNUSED(refcon);

	if (phase != xplm_CommandBegin)
		return (1);

	glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &old_read_fbo);
	for (int i = 0; i < 3; i++) {
		char filename[64];

		if (xp11_atmo.tmp_fbo[i] == 0)
			continue;

		glBindFramebuffer(GL_READ_FRAMEBUFFER, xp11_atmo.tmp_fbo[i]);
		glReadBuffer(GL_COLOR_ATTACHMENT0);

		glReadPixels(0, 0, EFIS_WIDTH, EFIS_HEIGHT,
		    GL_RGBA, GL_UNSIGNED_BYTE, buf);
		snprintf(filename, sizeof (filename), "xp11_atmo_fbo%d.png",
		    i);
		png_write_to_file_rgba(filename, EFIS_WIDTH, EFIS_HEIGHT, buf);
	}
	glBindFramebuffer(GL_READ_FRAMEBUFFER, old_read_fbo);

	return (1);
}

static void
atmo_xp11_set_range(double range)
{
	mutex_enter(&xp11_atmo.lock);
	/* Set the fallback value first */
	xp11_atmo.range = efis_map_ranges[EFIS_MAP_NUM_RANGES - 1];
	xp11_atmo.range_i = EFIS_MAP_NUM_RANGES - 1;

	for (int i = 0; i < EFIS_MAP_NUM_RANGES; i++) {
		if (range <= efis_map_ranges[i]) {
			xp11_atmo.range = efis_map_ranges[i];
			xp11_atmo.range_i = i;
			break;
		}
	}
	mutex_exit(&xp11_atmo.lock);
}

static void
atmo_xp11_probe(scan_line_t *sl)
{
#define	COST_PER_1KM	0.07
	double range;
	double dir_rand1 = (sin(DEG2RAD(sl->dir.x) * 6.7768) *
	    sin(DEG2RAD(sl->dir.x) * 18.06) *
	    sin(DEG2RAD(sl->dir.x) * 31.415)) / 15.0;
	double dir_rand2 = (sin(DEG2RAD(sl->dir.x) * 3.1767) *
	    sin(DEG2RAD(sl->dir.x) * 14.459) *
	    sin(DEG2RAD(sl->dir.x) * 34.252)) / 15.0;
	double sin_rhdg = sin(DEG2RAD(sl->ant_rhdg));
	double sin_rhdg_left = sl->vert_scan ? sin(DEG2RAD(sl->ant_rhdg -
	    sl->shape.x)) : 0;
	double sin_rhdg_right = sl->vert_scan ? sin(DEG2RAD(sl->ant_rhdg +
	    sl->shape.x)) : 0;
	double cos_rhdg = cos(DEG2RAD(sl->ant_rhdg));
	double cos_rhdg_left = sl->vert_scan ? cos(DEG2RAD(sl->ant_rhdg -
	    sl->shape.x)) : 0;
	double cos_rhdg_right = sl->vert_scan ? cos(DEG2RAD(sl->ant_rhdg +
	    sl->shape.x)) : 0;
	double sin_pitch = sin(DEG2RAD(sl->dir.y));
	double sin_pitch_up = !sl->vert_scan ? sin(DEG2RAD(sl->dir.y +
	    sl->shape.y * (0.5 + dir_rand1))) : 0;
	double sin_pitch_dn = !sl->vert_scan ? sin(DEG2RAD(sl->dir.y -
	    sl->shape.y * (0.5 + dir_rand2))) : 0;
	vect2_t precip_nodes[5];
	double energy = sl->energy;
	double sample_sz = sl->range / sl->num_samples;
	double sample_sz_rat = sample_sz / 1000.0;
	double cost_per_sample = COST_PER_1KM * sample_sz_rat;

	mutex_enter(&xp11_atmo.lock);
	range = xp11_atmo.range;
	memcpy(precip_nodes, xp11_atmo.precip_nodes, sizeof (precip_nodes));
	mutex_exit(&xp11_atmo.lock);

	for (int i = 0; i < sl->num_samples; i++) {
		int x = (((double)i + 1) / sl->num_samples) *
		    (sl->range / range) * EFIS_LON_FWD * sin_rhdg;
		int x_left = sl->vert_scan ? (((double)i + 1) /
		    sl->num_samples) * (sl->range / range) * EFIS_LON_FWD *
		    sin_rhdg_left : 0;
		int x_right = sl->vert_scan ? (((double)i + 1) /
		    sl->num_samples) * (sl->range / range) * EFIS_LON_FWD *
		    sin_rhdg_right : 0;
		int y = (((double)i + 1) / sl->num_samples) *
		    (sl->range / range) * EFIS_LON_FWD * cos_rhdg;
		int y_left = sl->vert_scan ? (((double)i + 1) /
		    sl->num_samples) * (sl->range / range) * EFIS_LON_FWD *
		    cos_rhdg_left : 0;
		int y_right = sl->vert_scan ? (((double)i + 1) /
		    sl->num_samples) * (sl->range / range) * EFIS_LON_FWD *
		    cos_rhdg_right : 0;
		double d = (((double)i + 1) / sl->num_samples) * sl->range;
		double z_up = sl->origin.elev + d * sin_pitch_up;
		double z = sl->origin.elev + d * sin_pitch;
		double z_dn = sl->origin.elev + d * sin_pitch_dn;
		double precip_intens_pt;
		double precip_intens[3];
		double energy_cost = 0;

		UNUSED(z);

		x += EFIS_LAT_PIX;
		y += EFIS_LON_AFT;

		/*
		 * No doppler radar support yet.
		 */
		sl->doppler_out[i] = 0;

		if (x < 0 || x >= EFIS_WIDTH || y < 0 || y >= EFIS_HEIGHT) {
			sl->energy_out[i] = 0;
			continue;
		}

		if (xp11_atmo.pixels != NULL) {
			precip_intens_pt = (xp11_atmo.pixels[y *
			    EFIS_WIDTH + x] & 0xff) / 255.0;
			if (sl->vert_scan) {
				precip_intens_pt += (xp11_atmo.pixels[y_left *
				    EFIS_WIDTH + x_left] & 0xff) / 255.0;
				precip_intens_pt += (xp11_atmo.pixels[y_right *
				    EFIS_WIDTH + x_right] & 0xff) / 255.0;
				precip_intens_pt /= 3.0;
			}
		} else {
			precip_intens_pt = 0.0;
		}

		/*
		 * Compute precip intensity while taking the precip modulation
		 * nodes into account.
		 */
		if (!sl->vert_scan) {
			precip_intens[0] = precip_intens_pt *
			    fx_lin_multi(z_up, precip_nodes, B_FALSE);
			if (isnan(precip_intens[0]))
				precip_intens[0] = 0;
		} else {
			precip_intens[0] = 0;
		}

		precip_intens[1] = precip_intens_pt *
		    fx_lin_multi(z, precip_nodes, B_FALSE);
		if (isnan(precip_intens[1]))
			precip_intens[1] = 0;

		if (!sl->vert_scan) {
			precip_intens[2] = precip_intens_pt *
			    fx_lin_multi(z_dn, precip_nodes, B_FALSE);
			if (isnan(precip_intens[2]))
				precip_intens[2] = 0;
		} else {
			precip_intens[2] = 0;
		}

		energy_cost = MAX(energy_cost, cost_per_sample *
		    precip_intens[0] * (energy / sl->energy));
		energy_cost = MAX(energy_cost, cost_per_sample *
		    precip_intens[1] * (energy / sl->energy));
		energy_cost = MAX(energy_cost, cost_per_sample *
		    precip_intens[2] * (energy / sl->energy));

		sl->energy_out[i] = energy_cost;
		energy = MAX(0, energy - energy_cost);
	}
}

static void
update_efis(void)
{
	enum {
	    EFIS_MODE_NORM = 1,
	    EFIS_SUBMODE_MAP = 2,
	    EFIS_SUBMODE_NAV = 3,
	    EFIS_SUBMODE_PLANE = 4,
	    EFIS_SUBMODE_GOOD_MAP = 5
	};
	/*
	 * IMPORTANT: the EFIS map brightness is tied to
	 * instrument_brightness_ratio[0], so make sure it's full intensity
	 * all the time so we can read the map.
	 */
	if (dr_getf(&drs.EFIS.instr_brt) != 1.0)
		dr_setf(&drs.EFIS.instr_brt, 1.0);
	if (dr_geti(&drs.EFIS.mode) != EFIS_MODE_NORM)
		dr_seti(&drs.EFIS.mode, EFIS_MODE_NORM);
	if (dr_geti(&drs.EFIS.submode) != EFIS_SUBMODE_GOOD_MAP)
		dr_seti(&drs.EFIS.submode, EFIS_SUBMODE_GOOD_MAP);
	if (dr_geti(&drs.EFIS.range) != (int)xp11_atmo.range_i)
		dr_seti(&drs.EFIS.range, xp11_atmo.range_i);
	if (dr_geti(&drs.EFIS.shows_wx) != 1)
		dr_seti(&drs.EFIS.shows_wx, 1);
	if (dr_getf(&drs.EFIS.wx_alpha) != 1.0)
		dr_seti(&drs.EFIS.wx_alpha, 1);
	if (dr_geti(&drs.EFIS.shows_tcas) != 0)
		dr_seti(&drs.EFIS.shows_tcas, 0);
	if (dr_geti(&drs.EFIS.shows_arpts) != 0)
		dr_seti(&drs.EFIS.shows_arpts, 0);
	if (dr_geti(&drs.EFIS.shows_wpts) != 0)
		dr_seti(&drs.EFIS.shows_wpts, 0);
	if (dr_geti(&drs.EFIS.shows_VORs) != 0)
		dr_seti(&drs.EFIS.shows_VORs, 0);
	if (dr_geti(&drs.EFIS.shows_NDBs) != 0)
		dr_seti(&drs.EFIS.shows_NDBs, 0);
	if (dr_geti(&drs.EFIS.kill_map_fms_line) == 0)
		dr_seti(&drs.EFIS.kill_map_fms_line, 1);
}

static void
update_precip(void)
{
	double cloud_z[2] = { 0, 0 };
	double tmp_0_alt, tmp_minus_20_alt;
	enum { CLOUD_TOP_MARGIN = 50, RAIN_EVAP_MARGIN = 5000 };

	/*
	 * To compute the location of the freezing level, we use the
	 * sea-level temperature and tropopause temperature & altitude
	 * to construct a linear temperature ramp. This is more-or-less
	 * how temperature decreases with altitude.
	 */
	tmp_0_alt = fx_lin(0, dr_getf(&drs.temp_sl), 0,
	    dr_getf(&drs.temp_tropo), dr_getf(&drs.alt_tropo));
	tmp_minus_20_alt = fx_lin(-20.0, dr_getf(&drs.temp_sl), 0,
	    dr_getf(&drs.temp_tropo), dr_getf(&drs.alt_tropo));

	/*
	 * If the temperature is inverted, force the algorithm below to at
	 * least not crash.
	 */
	if (tmp_0_alt >= tmp_minus_20_alt - 100)
		tmp_0_alt = tmp_minus_20_alt - 1000;

	for (int i = 0; i < 3; i++) {
		/* clear skies or cirrus clouds don't generate precip */
		if (dr_geti(&drs.cloud_type[i]) <= XP11_CLOUD_HIGH_CIRRUS)
			continue;
		cloud_z[0] = MIN(cloud_z[0], dr_getf(&drs.cloud_base[i]));
		cloud_z[1] = MAX(cloud_z[0], dr_getf(&drs.cloud_tops[i]));
	}

	/*
	 * The top of the precip ramp is just above the cloud top.
	 * The bottom is either at the cloud top minus the margin,
	 * or in the middle between the cloud top & bottom, whichever
	 * is higher.
	 */
	if (cloud_z[0] == cloud_z[1]) {
		for (int i = 0; i < 4; i++)
			xp11_atmo.precip_nodes[i] = VECT2(i, 0);
	} else {
		xp11_atmo.precip_nodes[0] =
		    VECT2(cloud_z[0] - RAIN_EVAP_MARGIN, 1.0);
		xp11_atmo.precip_nodes[1] = VECT2(cloud_z[0], 1.0);
		xp11_atmo.precip_nodes[2] =
		    VECT2(MAX(cloud_z[1] - CLOUD_TOP_MARGIN,
		    (cloud_z[0] + cloud_z[1] / 2)), 1);
		xp11_atmo.precip_nodes[3] =
		    VECT2(cloud_z[1] + CLOUD_TOP_MARGIN, 0);

		for (int i = 0; i < 4; i++) {
			/*
			 * Clamp the modulation curve so as not to extend
			 * above the freezing level. Even if the cloud
			 * reaches higher, its contents will be completely
			 * frozen, so WXR won't see them.
			 */
			xp11_atmo.precip_nodes[i].y *=
			    (1 - iter_fract(xp11_atmo.precip_nodes[i].x,
			    tmp_0_alt, tmp_minus_20_alt, B_TRUE));
		}
	}

	fx_lin_multi(0, xp11_atmo.precip_nodes, B_FALSE);
}

static void
setup_opengl(void)
{
	if (xp11_atmo.pbo == 0) {
		glGenBuffers(1, &xp11_atmo.pbo);
		glBindBuffer(GL_PIXEL_PACK_BUFFER, xp11_atmo.pbo);
		glBufferData(GL_PIXEL_PACK_BUFFER, EFIS_WIDTH * EFIS_HEIGHT *
		    sizeof (*xp11_atmo.pixels), 0, GL_STREAM_READ);
		glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
	}

	if (xp11_atmo.tmp_tex[0] == 0) {
		glGenTextures(3, xp11_atmo.tmp_tex);
		for (int i = 0; i < 3; i++) {
			XPLMBindTexture2d(xp11_atmo.tmp_tex[i], GL_TEXTURE_2D);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, EFIS_WIDTH,
			    EFIS_HEIGHT, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
			    GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
			    GL_LINEAR);
		}
	}

	if (xp11_atmo.tmp_fbo[0] == 0) {
		GLint old_fbo;

		glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &old_fbo);
		glGenFramebuffers(3, xp11_atmo.tmp_fbo);
		for (int i = 0; i < 3; i++) {
			glBindFramebuffer(GL_FRAMEBUFFER, xp11_atmo.tmp_fbo[i]);
			glFramebufferTexture2D(GL_FRAMEBUFFER,
			    GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
			    xp11_atmo.tmp_tex[i], 0);
			VERIFY3U(glCheckFramebufferStatus(GL_FRAMEBUFFER), ==,
			    GL_FRAMEBUFFER_COMPLETE);
		}
		glBindFramebuffer(GL_DRAW_FRAMEBUFFER, old_fbo);
	}

	if (xp11_atmo.efis_quads.vbo == 0) {
		vect2_t vtx[4] = {
		    VECT2(0, 0), VECT2(0, EFIS_HEIGHT),
		    VECT2(EFIS_WIDTH, EFIS_HEIGHT), VECT2(EFIS_WIDTH, 0)
		};
		glutils_init_2D_quads(&xp11_atmo.efis_quads, vtx, NULL, 4);
		glm_ortho(0, EFIS_WIDTH, 0, EFIS_HEIGHT, 0, 1,
		    xp11_atmo.efis_pvm);
	}
}

static void
transfer_new_efis_frame(void)
{
	double range = efis_map_ranges[MIN((unsigned)dr_geti(&drs.EFIS.range),
	    EFIS_MAP_NUM_RANGES - 1)];
	GLint old_read_fbo, old_draw_fbo;

	XPLMSetGraphicsState(0, 1, 0, 1, 1, 1, 1);
	glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &old_read_fbo);
	glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &old_draw_fbo);

	/*
	 * Step 1: transfer the EFIS screen FBO into the input FBO.
	 */
	glBindFramebuffer(GL_READ_FRAMEBUFFER, old_read_fbo);
	glReadBuffer(GL_COLOR_ATTACHMENT0);
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, xp11_atmo.tmp_fbo[0]);
	glClear(GL_COLOR_BUFFER_BIT);
	glDrawBuffer(GL_COLOR_ATTACHMENT0);
	glBlitFramebuffer(xp11_atmo.efis_x, xp11_atmo.efis_y,
	    xp11_atmo.efis_x + EFIS_WIDTH, xp11_atmo.efis_y + EFIS_HEIGHT,
	    0, 0, EFIS_WIDTH, EFIS_HEIGHT, GL_COLOR_BUFFER_BIT, GL_NEAREST);

	/*
	 * Step 2: pass the EFIS output through a cleanup shader to
	 * get rid of the EFIS symbology.
	 */
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, xp11_atmo.tmp_fbo[1]);
	glClear(GL_COLOR_BUFFER_BIT);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, xp11_atmo.tmp_tex[0]);

	glUseProgram(xp11_atmo.cleanup_prog);
	glUniformMatrix4fv(glGetUniformLocation(xp11_atmo.cleanup_prog, "pvm"),
	    1, GL_FALSE, (GLfloat *)xp11_atmo.efis_pvm);
	glUniform1i(glGetUniformLocation(xp11_atmo.cleanup_prog, "tex"), 0);
	glUniform2f(glGetUniformLocation(xp11_atmo.cleanup_prog, "tex_sz"),
	    EFIS_WIDTH, EFIS_HEIGHT);
	glutils_draw_quads(&xp11_atmo.efis_quads, xp11_atmo.cleanup_prog);

	/*
	 * Step 3: smooth the EFIS output to get a more sensible
	 * representation of precip intensity (rather than just
	 * using the pre-rendered colors as a fixed value.
	 */
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, xp11_atmo.tmp_fbo[2]);
	glClear(GL_COLOR_BUFFER_BIT);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, xp11_atmo.tmp_tex[1]);

	glUseProgram(xp11_atmo.smooth_prog);
	glUniformMatrix4fv(glGetUniformLocation(xp11_atmo.smooth_prog, "pvm"),
	    1, GL_FALSE, (GLfloat *)xp11_atmo.efis_pvm);
	glUniform1i(glGetUniformLocation(xp11_atmo.smooth_prog, "tex"), 0);
	glUniform2f(glGetUniformLocation(xp11_atmo.smooth_prog, "tex_sz"),
	    EFIS_WIDTH, EFIS_HEIGHT);
	glUniform1f(glGetUniformLocation(xp11_atmo.smooth_prog, "smooth_val"),
	    WX_SMOOTH_RNG / range);
	glutils_draw_quads(&xp11_atmo.efis_quads, xp11_atmo.smooth_prog);

	glUseProgram(0);

	/*
	 * Step 4: set up transfer of the output FBO back to the CPU.
	 */
	glBindFramebuffer(GL_READ_FRAMEBUFFER, xp11_atmo.tmp_fbo[2]);
	glBindBuffer(GL_PIXEL_PACK_BUFFER, xp11_atmo.pbo);
	glReadPixels(0, 0, EFIS_WIDTH, EFIS_HEIGHT, GL_RGBA, GL_UNSIGNED_BYTE,
	    NULL);
	glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);

	/*
	 * Step 5: restore the FBO state of X-Plane.
	 */
	glBindFramebuffer(GL_READ_FRAMEBUFFER, old_read_fbo);
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, old_draw_fbo);
}

static int
update_cb(XPLMDrawingPhase phase, int before, void *refcon)
{
	uint64_t now;
	UNUSED(phase);
	UNUSED(before);
	UNUSED(refcon);

	/*
	 * Careful, don't read the FBO from the other phases, you'll get junk.
	 */
	if (dr_geti(&drs.render_type) != XPLANE_RENDER_GAUGES_3D_LIT)
		return (1);

	glutils_disable_all_client_state();

	mutex_enter(&xp11_atmo.lock);

	update_efis();
	update_precip();

	if (xp11_atmo.pixels == NULL) {
		if (xp11_atmo.efis_w == 0 || xp11_atmo.efis_h == 0)
			goto out;
		xp11_atmo.pixels = safe_calloc(EFIS_WIDTH * EFIS_HEIGHT,
		    sizeof (*xp11_atmo.pixels));
	}

	setup_opengl();

	now = microclock();
	if (xp11_atmo.xfer_sync != 0) {
		if (glClientWaitSync(xp11_atmo.xfer_sync, 0, 0) !=
		    GL_TIMEOUT_EXPIRED) {
			void *ptr;

			/* Latest WXR image transfer is complete, fetch it */
			glBindBuffer(GL_PIXEL_PACK_BUFFER, xp11_atmo.pbo);
			ptr = glMapBuffer(GL_PIXEL_PACK_BUFFER, GL_READ_ONLY);
			if (ptr != NULL) {
				memcpy(xp11_atmo.pixels, ptr, EFIS_WIDTH *
				    EFIS_HEIGHT * sizeof (*xp11_atmo.pixels));
				glUnmapBuffer(GL_PIXEL_PACK_BUFFER);
			}
			glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
			xp11_atmo.xfer_sync = 0;
		}
	} else if (xp11_atmo.last_update + UPD_INTVAL <= now) {
		transfer_new_efis_frame();
		xp11_atmo.xfer_sync =
		    glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
		xp11_atmo.last_update = now;
	}

out:
	mutex_exit(&xp11_atmo.lock);
	return (1);
}

atmo_t *
atmo_xp11_init(const char *xpdir, const char *plugindir)
{
	char *path, *path2;

	ASSERT(!inited);
	inited = B_TRUE;

	memset(&xp11_atmo, 0, sizeof (xp11_atmo));

	debug_cmd = XPLMCreateCommand("openwxr/debug_atmo_xp11",
	    "Dump XP11 screenshot into X-Plane folder");
	ASSERT(debug_cmd != NULL);
	XPLMRegisterCommandHandler(debug_cmd, debug_cmd_handler, 0, NULL);

	for (int i = 0; i < 3; i++) {
		fdr_find(&drs.cloud_type[i],
		    "sim/weather/cloud_type[%d]", i);
		fdr_find(&drs.cloud_cover[i],
		    "sim/weather/cloud_coverage[%d]", i);
		fdr_find(&drs.cloud_base[i],
		    "sim/weather/cloud_base_msl_m[%d]", i);
		fdr_find(&drs.cloud_tops[i],
		    "sim/weather/cloud_tops_msl_m[%d]", i);
		fdr_find(&drs.wind_alt[i],
		    "sim/weather/wind_altitude_msl_m[%d]", i);
		fdr_find(&drs.wind_dir[i],
		    "sim/weather/wind_direction_degt[%d]", i);
		fdr_find(&drs.wind_spd[i],
		    "sim/weather/wind_speed_kt[%d]", i);
		fdr_find(&drs.wind_turb[i],
		    "sim/weather/turbulence[%d]", i);
		fdr_find(&drs.shear_dir[i],
		    "sim/weather/shear_direction_degt[%d]", i);
		fdr_find(&drs.shear_spd[i],
		    "sim/weather/shear_speed_kt[%d]", i);
	}
	fdr_find(&drs.turb, "sim/weather/wind_turbulence_percent");

	fdr_find(&drs.EFIS.instr_brt,
	    "sim/cockpit2/switches/instrument_brightness_ratio");
	fdr_find(&drs.EFIS.mode, "sim/cockpit2/EFIS/map_mode");
	fdr_find(&drs.EFIS.submode, "sim/cockpit/switches/EFIS_map_submode");
	fdr_find(&drs.EFIS.range,
	    "sim/cockpit/switches/EFIS_map_range_selector");
	fdr_find(&drs.EFIS.shows_wx, "sim/cockpit/switches/EFIS_shows_weather");
	fdr_find(&drs.EFIS.wx_alpha, "sim/cockpit/switches/EFIS_weather_alpha");
	fdr_find(&drs.EFIS.shows_tcas, "sim/cockpit/switches/EFIS_shows_tcas");
	fdr_find(&drs.EFIS.shows_arpts,
	    "sim/cockpit/switches/EFIS_shows_airports");
	fdr_find(&drs.EFIS.shows_wpts,
	    "sim/cockpit/switches/EFIS_shows_waypoints");
	fdr_find(&drs.EFIS.shows_VORs, "sim/cockpit/switches/EFIS_shows_VORs");
	fdr_find(&drs.EFIS.shows_NDBs, "sim/cockpit/switches/EFIS_shows_NDBs");
	fdr_find(&drs.EFIS.kill_map_fms_line, "sim/graphics/misc/kill_map_fms_line");

	fdr_find(&drs.render_type, "sim/graphics/view/panel_render_type");

	fdr_find(&drs.temp_sl, "sim/weather/temperature_sealevel_c");
	fdr_find(&drs.temp_tropo, "sim/weather/temperature_tropo_c");
	fdr_find(&drs.alt_tropo, "sim/weather/tropo_alt_mtr");

	for (int i = 0; i < 4; i++)
		xp11_atmo.precip_nodes[i] = VECT2(i, 0);
	xp11_atmo.precip_nodes[4] = NULL_VECT2;

	path = mkpathname(xpdir, plugindir, "data", "generic.vert", NULL);
	path2 = mkpathname(xpdir, plugindir, "data", "cleanup.frag", NULL);
	xp11_atmo.cleanup_prog = shader_prog_from_file("cleanup", path, path2,
	    DEFAULT_VTX_ATTRIB_BINDINGS, NULL);
	lacf_free(path);
	lacf_free(path2);
	if (xp11_atmo.cleanup_prog)
		goto errout;

	path = mkpathname(xpdir, plugindir, "data", "generic.vert", NULL);
	path2 = mkpathname(xpdir, plugindir, "data", "smooth.frag", NULL);
	xp11_atmo.smooth_prog = shader_prog_from_file("smooth", path, path2,
	    DEFAULT_VTX_ATTRIB_BINDINGS, NULL);
	lacf_free(path);
	lacf_free(path2);
	if (xp11_atmo.smooth_prog == 0)
		goto errout;

	mutex_init(&xp11_atmo.lock);

	XPLMRegisterDrawCallback(update_cb, xplm_Phase_Gauges, 0, NULL);

	return (&atmo);
errout:
	atmo_xp11_fini();
	return (NULL);
}

void
atmo_xp11_fini(void)
{
	if (!inited)
		return;
	inited = B_FALSE;

	XPLMUnregisterCommandHandler(debug_cmd, debug_cmd_handler, 0, NULL);

	if (xp11_atmo.pbo != 0)
		glDeleteBuffers(1, &xp11_atmo.pbo);
	if (xp11_atmo.tmp_fbo[0] != 0)
		glDeleteFramebuffers(3, xp11_atmo.tmp_fbo);
	if (xp11_atmo.tmp_tex[0] != 0)
		glDeleteTextures(3, xp11_atmo.tmp_tex);
	if (xp11_atmo.cleanup_prog != 0)
		glDeleteProgram(xp11_atmo.cleanup_prog);
	if (xp11_atmo.smooth_prog != 0)
		glDeleteProgram(xp11_atmo.smooth_prog);
	glutils_destroy_quads(&xp11_atmo.efis_quads);

	mutex_destroy(&xp11_atmo.lock);
	XPLMUnregisterDrawCallback(update_cb, xplm_Phase_Gauges, 0, NULL);
}

void
atmo_xp11_set_efis_pos(unsigned x, unsigned y, unsigned w, unsigned h)
{
	mutex_enter(&xp11_atmo.lock);

	xp11_atmo.efis_x = x;
	xp11_atmo.efis_y = y;
	xp11_atmo.efis_w = w;
	xp11_atmo.efis_h = h;

	free(xp11_atmo.pixels);
	xp11_atmo.pixels = NULL;

	mutex_exit(&xp11_atmo.lock);
}
