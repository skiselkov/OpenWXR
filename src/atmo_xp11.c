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

#include <XPLMDisplay.h>

#include <acfutils/assert.h>
#include <acfutils/dr.h>
#include <acfutils/geom.h>
#include <acfutils/helpers.h>
#include <acfutils/thread.h>
#include <acfutils/time.h>

#include "atmo_xp11.h"

#define	UPD_INTVAL	200000		/* us */

static void atmo_xp11_set_range(double range);
static void atmo_xp11_probe(scan_line_t *sl);

static bool_t inited = B_FALSE;
static atmo_t atmo = {
	.set_range = atmo_xp11_set_range,
	.probe = atmo_xp11_probe
};

typedef enum {
	EFIS_MAP_RANGE_10NM,
	EFIS_MAP_RANGE_20NM,
	EFIS_MAP_RANGE_40NM,
	EFIS_MAP_RANGE_80NM,
	EFIS_MAP_RANGE_160NM,
	EFIS_MAP_RANGE_320NM,
	EFIS_MAP_RANGE_640NM,
	EFIS_MAP_NUM_RANGES
} efis_map_range_t;

/* Must follow order of efis_map_range_t */
static double efis_map_ranges[EFIS_MAP_NUM_RANGES] = {
	NM2MET(10),
	NM2MET(20),
	NM2MET(40),
	NM2MET(80),
	NM2MET(160),
	NM2MET(320),
	NM2MET(640)
};

static struct {
	mutex_t		lock;

	/* protected by lock */
	uint32_t	*pixels;
	double		range;

	/* only accessed by foreground drawing thread */
	uint64_t	last_update;
	unsigned	efis_x;
	unsigned	efis_y;
	unsigned	efis_w;
	unsigned	efis_h;
	GLuint		pbo;
	GLsync		xfer_sync;
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

	struct {
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
	} EFIS;
} drs;

static void
atmo_xp11_set_range(double range)
{
	mutex_enter(&xp11_atmo.lock);
	xp11_atmo.range = range;
	mutex_exit(&xp11_atmo.lock);
}

static void
atmo_xp11_probe(scan_line_t *sl)
{
	UNUSED(sl);
}

static efis_map_range_t
efis_map_range_select(void)
{
	for (int i = 0; i < EFIS_MAP_NUM_RANGES; i++) {
		if (xp11_atmo.range <= efis_map_ranges[i])
			return (i);
	}
	return (EFIS_MAP_RANGE_640NM);
}

static void
update_efis(void)
{
	enum {
	    EFIS_MODE_NORM = 1,
	    EFIS_SUBMODE_MAP = 2
	};
	if (dr_geti(&drs.EFIS.mode) != EFIS_MODE_NORM)
		dr_seti(&drs.EFIS.mode, EFIS_MODE_NORM);
	if (dr_geti(&drs.EFIS.submode) != EFIS_SUBMODE_MAP)
		dr_seti(&drs.EFIS.submode, EFIS_SUBMODE_MAP);
	if (dr_geti(&drs.EFIS.range) == efis_map_range_select())
		dr_seti(&drs.EFIS.range, efis_map_range_select());
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
}

static int
update_cb(XPLMDrawingPhase phase, int before, void *refcon)
{
	uint64_t now = microclock();

	UNUSED(phase);
	UNUSED(before);
	UNUSED(refcon);

	mutex_enter(&xp11_atmo.lock);

	if (xp11_atmo.pixels == NULL) {
		if (xp11_atmo.efis_w == 0 || xp11_atmo.efis_h == 0)
			goto out;
		xp11_atmo.pixels = calloc(xp11_atmo.efis_w * xp11_atmo.efis_h,
		    sizeof (*xp11_atmo.pixels));
	}

	if (xp11_atmo.pbo == 0)
		glGenBuffers(1, &xp11_atmo.pbo);

	if (xp11_atmo.xfer_sync != 0) {
		if (glClientWaitSync(xp11_atmo.xfer_sync, 0, 0) !=
		    GL_TIMEOUT_EXPIRED) {
			void* ptr;
			/* Latest WXR image transfer is complete */

			glBindBuffer(GL_PIXEL_PACK_BUFFER, xp11_atmo.pbo);
			ptr = glMapBuffer(GL_PIXEL_PACK_BUFFER, GL_READ_ONLY);
			if (ptr != NULL) {
				memcpy(xp11_atmo.pixels, ptr,
				    xp11_atmo.efis_w * xp11_atmo.efis_h *
				    sizeof (*xp11_atmo.pixels));
				glUnmapBuffer(GL_PIXEL_PACK_BUFFER);
			}
			glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
			xp11_atmo.xfer_sync = 0;
		}
	} else if (xp11_atmo.last_update + UPD_INTVAL <= now) {
		glReadBuffer(GL_FRONT);
		glBindBuffer(GL_PIXEL_PACK_BUFFER, xp11_atmo.pbo);
		glReadPixels(xp11_atmo.efis_x, xp11_atmo.efis_y,
		    xp11_atmo.efis_w, xp11_atmo.efis_h, GL_RGBA,
		    GL_UNSIGNED_BYTE, NULL);
		glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
		xp11_atmo.xfer_sync =
		    glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
		xp11_atmo.last_update = now;
	}

	update_efis();

out:
	mutex_exit(&xp11_atmo.lock);
	return (1);
}

atmo_t *
atmo_xp11_init(void)
{
	ASSERT(!inited);
	inited = B_TRUE;

	memset(&xp11_atmo, 0, sizeof (xp11_atmo));

	mutex_init(&xp11_atmo.lock);

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

	fdr_find(&drs.EFIS.mode, "sim/cockpit/switches/EFIS_map_mode");
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

	XPLMRegisterDrawCallback(update_cb, xplm_Phase_Gauges, 0, NULL);

	return (&atmo);
}

void
atmo_xp11_fini(void)
{
	if (!inited)
		return;
	inited = B_FALSE;

	if (xp11_atmo.pbo != 0)
		glDeleteBuffers(1, &xp11_atmo.pbo);

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
