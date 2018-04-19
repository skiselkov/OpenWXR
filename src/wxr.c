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

#include <stdlib.h>
#include <string.h>

#include <GL/glew.h>
#include <cglm/cglm.h>

#include <XPLMGraphics.h>
#include <XPLMPlugin.h>

#include <opengpws/xplane_api.h>

#include <acfutils/assert.h>
#include <acfutils/crc64.h>
#include <acfutils/glutils.h>
#include <acfutils/helpers.h>
#include <acfutils/math.h>
#include <acfutils/perf.h>
#include <acfutils/safe_alloc.h>
#include <acfutils/shader.h>
#include <acfutils/thread.h>
#include <acfutils/time.h>
#include <acfutils/worker.h>

#include "wxr.h"
#include "xplane.h"

#define	TEX_UPD_INTVAL	40000			/* us, 25 fps */
#define	WORKER_INTVAL	33333			/* us, 30 fps */
#define	MAX_BEAM_ENERGY	1			/* dBmW */
#define	EARTH_CIRC	(2 * EARTH_MSL * M_PI)	/* meters */
#define	MAX_TERR_LAT	79
#define	GROUND_RETURN_MULT	0.2		/* energy multiplier */
#define	SHADOW_ENERGY_THRESH	0.57
#define	ENERGY_SCALE_FACT	0.04
#define	PANEL_TEX_SZ		2048		/* pixels */

struct wxr_s {
	const wxr_conf_t	*conf;
	const atmo_t		*atmo;

	/* only accessed by foreground thread */
	unsigned		cur_tex;
	GLuint			tex[2];
	GLuint			pbo;
	GLuint			shadow_tex[2];
	GLuint			shadow_pbo;
	GLsync			upload_sync;
	uint64_t		last_upload;
	GLuint			wxr_prog;
	glutils_quads_t		wxr_scr_quads;
	mat4			wxr_scr_pvm;
	vect2_t			draw_pos;
	vect2_t			draw_size;
	bool_t			draw_vert;

	mutex_t			lock;
	/* protected by lock above */
	bool_t			standby;
	bool_t			vert_mode;
	geo_pos3_t		acf_pos;
	vect3_t			acf_orient;
	unsigned		cur_range;
	double			gain;
	double			ant_pitch_req;
	unsigned		azi_lim_left;
	unsigned		azi_lim_right;
	double			pitch_stab;
	double			roll_stab;
	const wxr_color_t *	colors;
	size_t			num_colors;

	/* only accessed from worker thread */
	unsigned		ant_pos;
	unsigned		ant_pos_vert;
	bool_t			scan_right;
	scan_line_t		sl;
	egpws_terr_probe_t	tp;

	/* unstructured, always safe to read & write */
	uint32_t		*samples;
	uint32_t		*shadow_samples;
	bool_t			beam_shadow;

	XPLMPluginID		opengpws;
	const egpws_intf_t	*terr;

	worker_t		wk;
};

static void
wxr_ant_return2neutral(wxr_t *wxr)
{
	const wxr_conf_t *conf = wxr->conf;

	/*
	 * Since advance_ant_pos always first increments or decrements the
	 * variable before checking for reversal, we want to stay just one
	 * step away from the eddge to allow for that.
	 */
	wxr->ant_pos = clampi((conf->res_x / conf->scan_angle) *
	    (conf->parked_azi + conf->scan_angle / 2), 1, conf->res_x - 2);
}

static vect3_t
randomize_normal(vect3_t norm)
{
	double rx = 0.9 + ((double)crc64_rand() / UINT64_MAX) / 5;
	double ry = 0.9 + ((double)crc64_rand() / UINT64_MAX) / 5;
	double rz = 0.9 + ((double)crc64_rand() / UINT64_MAX) / 5;
	return (VECT3(norm.x * rx, norm.y * ry, norm.z * rz));
}

static void
advance_ant_pos(wxr_t *wxr)
{
	/*
	 * Move the antenna by one notch left/right (or up/down
	 * when in vertical mode).
	 */
	if (wxr->scan_right) {
		if (!wxr->vert_mode)
			wxr->ant_pos++;
		else
			wxr->ant_pos_vert++;
		if ((!wxr->vert_mode && (wxr->ant_pos == wxr->conf->res_x - 1 ||
		    wxr->ant_pos >= wxr->azi_lim_right)) || (wxr->vert_mode &&
		    wxr->ant_pos_vert == wxr->conf->res_x - 1))
			wxr->scan_right = B_FALSE;
	} else {
		if (!wxr->vert_mode)
			wxr->ant_pos--;
		else
			wxr->ant_pos_vert--;
		if ((!wxr->vert_mode &&
		    (wxr->ant_pos == 0 || wxr->ant_pos <= wxr->azi_lim_left)) ||
		    (wxr->vert_mode && wxr->ant_pos_vert == 0))
			wxr->scan_right = B_TRUE;
	}
}

static void
prep_terr_probe_coords(wxr_t *wxr, vect2_t ant_dir, vect2_t degree_sz)
{
	for (unsigned i = 0; i < wxr->conf->res_y; i++) {
		double d = ((double)i / wxr->conf->res_y) * wxr->sl.range;
		vect2_t disp_m = vect2_scmul(ant_dir, d);
		vect2_t disp_deg = VECT2(disp_m.x / degree_sz.x,
		    disp_m.y / degree_sz.y);
		geo_pos2_t p = GEO_POS2(wxr->sl.origin.lat + disp_deg.y,
		    wxr->sl.origin.lon + disp_deg.x);
		/*
		 * Handle geo coordinate wrapping.
		 */
		p.lat = clamp(p.lat, -MAX_TERR_LAT, MAX_TERR_LAT);
		if (p.lon <= -180.0)
			p.lon += 360;
		else if (p.lon >= 180.0)
			p.lon -= 360.0;
		ASSERT(is_valid_lat(p.lat));
		ASSERT(is_valid_lon(p.lon));
		wxr->tp.in_pts[i] = p;
	}
}

static bool_t
wxr_worker(void *userinfo)
{
	wxr_t *wxr = userinfo;
	double ant_pitch_base, acf_hdg, acf_pitch;
	vect2_t degree_sz;
	double scan_time;
	double sample_sz = wxr->sl.range / wxr->sl.num_samples;
	double sample_sz_rat = sample_sz / 1000.0;
	double extra_pitch = 0, extra_roll = 0;
	size_t num_colors;
	wxr_color_t *colors;

#ifdef	WXR_PROFILE
	uint64_t now = microclock();
	static uint64_t last_report_time = 0;
	static uint64_t total_time = 0;
	uint64_t start, end;

	start = now;
#endif	/* WXR_PROFILE */

	mutex_enter(&wxr->lock);

	wxr->sl.origin = wxr->acf_pos;
	wxr->sl.shape = wxr->conf->beam_shape;
	wxr->sl.range = wxr->conf->ranges[wxr->cur_range];
	wxr->sl.energy = MAX_BEAM_ENERGY;
	wxr->sl.max_range = wxr->conf->ranges[wxr->conf->num_ranges - 1];
	wxr->sl.num_samples = wxr->conf->res_y;
	ant_pitch_base = wxr->ant_pitch_req;
	acf_hdg = wxr->acf_orient.y;
	acf_pitch = wxr->acf_orient.x;
	if (acf_pitch > wxr->pitch_stab)
		extra_pitch = acf_pitch - wxr->pitch_stab;
	else if (acf_pitch < -wxr->pitch_stab)
		extra_pitch = wxr->pitch_stab + acf_pitch;
	if (wxr->acf_orient.z > wxr->roll_stab)
		extra_roll = wxr->acf_orient.z - wxr->roll_stab;
	else if (wxr->acf_orient.z < -wxr->roll_stab)
		extra_roll = wxr->acf_orient.z + wxr->roll_stab;

	num_colors = wxr->num_colors;
	colors = safe_calloc(sizeof (*colors), num_colors);
	memcpy(colors, wxr->colors, sizeof (*colors) * num_colors);

	mutex_exit(&wxr->lock);

	degree_sz = VECT2(
	    (EARTH_CIRC / 360.0) * cos(DEG2RAD(wxr->sl.origin.lat)),
	    (EARTH_CIRC / 360.0));

	/*
	 * A word on terrain drawing.
	 *
	 * We need to pass LATxLON points to OpenGPWS to give us terrain
	 * elevations, but since doing proper FPP-to-GEO transformations
	 * for each point would be pretty expensive (tons of trig), we
	 * fudge it by instead projecting lines at a fixed LATxLON
	 * increment using our heading. Essentially, we are projecting
	 * rhumb lines instead of true radials, but for the short terrain
	 * distances that we care about (at most around 100km), that is
	 * "close enough" that we don't need to care.
	 */
	if (wxr->tp.in_pts == NULL) {
		wxr->tp.num_pts = wxr->conf->res_y;
		wxr->tp.in_pts = safe_calloc(wxr->tp.num_pts,
		    sizeof (*wxr->tp.in_pts));
		wxr->tp.out_elev = safe_calloc(wxr->tp.num_pts,
		    sizeof (*wxr->tp.out_elev));
		wxr->tp.out_norm = safe_calloc(wxr->tp.num_pts,
		    sizeof (*wxr->tp.out_norm));
		wxr->tp.out_water = safe_calloc(wxr->tp.num_pts,
		    sizeof (*wxr->tp.out_water));
	}

	/*
	 * We want to maintain a constant scan rate, but in vertical mode
	 * we often scan a different sector size, so adjust the scan time
	 * so that we scan a constant degrees/second rate.
	 */
	if (!wxr->vert_mode) {
		scan_time = wxr->conf->scan_time;
	} else {
		scan_time = (wxr->conf->scan_angle_vert /
		    wxr->conf->scan_angle) * wxr->conf->scan_time;
	}

	for (unsigned i = 0; i < (unsigned)(wxr->conf->res_x *
	    (USEC2SEC(WORKER_INTVAL) / scan_time)); i++) {
		enum { NUM_VERT_SECTORS = 10 };
		double ant_hdg, ant_pitch_up_down;
		int off;
		double energy_spent[NUM_VERT_SECTORS];
		vect2_t ant_dir, ant_dir_neg;
		double sin_ant_pitch[NUM_VERT_SECTORS + 1];
		double ant_pitch = ant_pitch_base;
		double cos_ant_pitch;

		CTASSERT(NUM_VERT_SECTORS > 1);
		memset(energy_spent, 0, sizeof (energy_spent));

		advance_ant_pos(wxr);
		if (!wxr->vert_mode)
			off = wxr->ant_pos * wxr->conf->res_y;
		else
			off = wxr->ant_pos_vert * wxr->conf->res_y;

		wxr->sl.ant_rhdg = (wxr->conf->scan_angle *
		    ((wxr->ant_pos / (double)wxr->conf->res_x) - 0.5)) *
		    cos(DEG2RAD(extra_roll));
		ant_hdg = acf_hdg + wxr->sl.ant_rhdg;
		if (wxr->vert_mode) {
			UNUSED(acf_pitch);
			ant_pitch =
			    -(wxr->conf->scan_angle_vert *
			    ((wxr->ant_pos_vert / (double)wxr->conf->res_x) -
			    0.5));
			ant_pitch = clamp(ant_pitch, -90, 90);
		}
		ant_pitch += extra_pitch;
		wxr->sl.dir = VECT2(ant_hdg, ant_pitch);
		ant_pitch_up_down = ant_pitch;
		cos_ant_pitch = cos(DEG2RAD(ant_pitch));
		wxr->sl.vert_scan = wxr->vert_mode;

		wxr->atmo->probe(&wxr->sl);

		ant_dir = hdg2dir(ant_hdg);
		ant_dir_neg = vect2_neg(ant_dir);
		for (int j = 0; j < NUM_VERT_SECTORS + 1; j++) {
			double angle = ant_pitch_up_down -
			    wxr->conf->beam_shape.y / 2 +
			    (wxr->conf->beam_shape.y / NUM_VERT_SECTORS) * j;
			sin_ant_pitch[j] = sin(DEG2RAD(angle));
		}
		prep_terr_probe_coords(wxr, ant_dir, degree_sz);
		wxr->terr->terr_probe(&wxr->tp);

		/*
		 * No need to lock the samples, worst case is we will
		 * draw a partially updated scan line - no big deal.
		 */
		for (unsigned j = 0; j < wxr->conf->res_y; j++) {
			double energy[NUM_VERT_SECTORS];
			double abs_energy = 0;
			double energy_spent_total = 0;
			/* Distance of point along scan line from antenna. */
			double d = ((double)j / wxr->conf->res_y) *
			    wxr->sl.range * cos_ant_pitch;
			int64_t elev_rand_lim = iter_fract(d, 0, 100000,
			    B_TRUE) * 3000 + 10;
			int64_t elev_rand = (crc64_rand() % elev_rand_lim) -
			    (elev_rand_lim / 2);
			double terr_elev = wxr->tp.out_elev[j] + elev_rand;
			vect2_t ant_dir_neg_m = vect2_scmul(ant_dir_neg, d);
			/* Reverse vector from ground point to the antenna. */
			vect3_t back_v = vect3_unit(VECT3(ant_dir_neg_m.x,
			    ant_dir_neg_m.y, wxr->sl.origin.elev - terr_elev),
			    NULL);
			double ground_absorb[NUM_VERT_SECTORS];
			double ground_return[NUM_VERT_SECTORS];
			double ground_return_total = 0;
			vect3_t norm;
			double fract_dir;

			for (int k = 0; k < NUM_VERT_SECTORS; k++) {
				energy[k] = wxr->sl.energy_out[j] /
				    NUM_VERT_SECTORS;
			}

			norm = randomize_normal(wxr->tp.out_norm[j]);
			fract_dir = vect3_dotprod(back_v, norm);
			fract_dir = clamp(fract_dir, 0, 1);

			for (int k = 0; k < NUM_VERT_SECTORS; k++) {
				/* How perpendicular is the ground to us */
				double elev_min;
				double elev_max;
				/*
				 * Fraction of how much of the beam is below
				 * ground.
				 */
				double fract_hit;

				elev_min = wxr->sl.origin.elev +
				    sin_ant_pitch[k] * d;
				elev_max = wxr->sl.origin.elev +
				    sin_ant_pitch[k + 1] * d;
				/*
				 * At extreme antenna angles, the top/bottom
				 * distinction can break, so to avoid that, we
				 * manually flip the coordinates in this case
				 * and add 0.1m to elev_max to guarantee that
				 * it cannot be <= elev_min.
				 */
				if (elev_min > elev_max) {
					double tmp = elev_max;
					elev_max = elev_min;
					elev_min = tmp;
				}
				elev_max += 0.1;
				fract_hit = iter_fract(terr_elev, elev_min,
				    elev_max, B_FALSE) / 5;
				fract_hit = clamp(fract_hit, 0, 1);
				ground_absorb[k] = ((1 - energy_spent[k]) *
				    fract_hit) * sample_sz_rat;
				ground_return[k] = ((1 - energy_spent[k]) *
				    fract_hit * (fract_dir + 0.8) /
				    NUM_VERT_SECTORS) * GROUND_RETURN_MULT *
				    (1 - wxr->tp.out_water[j] * 0.89);
			}

			for (int k = 0; k < NUM_VERT_SECTORS; k++) {
				abs_energy += energy[k];
				ground_return_total += ground_return[k];
				energy_spent[k] += energy[k] + ground_absorb[k];
				energy_spent_total += energy_spent[k];
			}
			abs_energy = ((abs_energy / sample_sz_rat) +
			    ground_return_total) * wxr->gain;

			if (energy_spent_total / NUM_VERT_SECTORS >
			    SHADOW_ENERGY_THRESH && wxr->beam_shadow) {
				wxr->shadow_samples[off + j] =
				    BE32(0x70707070u);
			} else {
				wxr->shadow_samples[off + j] = 0x00u;
			}
			wxr->samples[off + j] = 0x00u;
			for (size_t k = 0; k < num_colors; k++) {
				if (abs_energy / ENERGY_SCALE_FACT >=
				    colors[k].min_val) {
					wxr->samples[off + j] = colors[k].rgba;
					break;
				}
			}
		}
	}

	free(colors);

#ifdef	WXR_PROFILE
	end = microclock();
	total_time += (end - start);
	if (now - last_report_time > 1000000) {
		printf("load: %.3f%%\n", (100.0 * total_time) /
		    (now - last_report_time));
		last_report_time = now;
		total_time = 0;
	}
#endif	/* WXR_PROFILE */

	return (B_TRUE);
}

wxr_t *
wxr_init(const wxr_conf_t *conf, const atmo_t *atmo)
{
	wxr_t *wxr = safe_calloc(1, sizeof (*wxr));

	ASSERT(conf->num_ranges != 0);
	ASSERT3U(conf->num_ranges, <, WXR_MAX_RANGES);
	ASSERT3U(conf->res_x, >=, WXR_MIN_RES);
	ASSERT3U(conf->res_y, >=, WXR_MIN_RES);
	ASSERT3F(conf->beam_shape.x, >, 0);
	ASSERT3F(conf->beam_shape.y, >, 0);
	ASSERT3F(conf->scan_time, >, 0);
	ASSERT3F(conf->scan_angle, >, 0);
	ASSERT3F(conf->scan_angle_vert, >, 0);
	ASSERT3F(ABS(conf->parked_azi), <=, conf->scan_angle / 2);
	ASSERT(atmo->probe != NULL);

	mutex_init(&wxr->lock);

	wxr->conf = conf;
	wxr->atmo = atmo;
	wxr->gain = 1.0;
	/*
	 * 4 vertices per quad, 2 coords per vertex
	 */
	wxr->samples = safe_calloc(conf->res_x * conf->res_y,
	    sizeof (*wxr->samples));
	wxr->shadow_samples = safe_calloc(conf->res_x * conf->res_y,
	    sizeof (*wxr->samples));
	wxr_ant_return2neutral(wxr);
	wxr->azi_lim_right = conf->res_x - 1;
	wxr->sl.energy_out = safe_calloc(wxr->conf->res_y, sizeof (double));
	wxr->sl.doppler_out = safe_calloc(wxr->conf->res_y, sizeof (double));
	wxr->atmo->set_range(wxr->conf->ranges[0]);

	(void)wxr_reload_gl_progs(wxr);
	glm_ortho(0, PANEL_TEX_SZ, 0, PANEL_TEX_SZ, 0, 1, wxr->wxr_scr_pvm);

	wxr->opengpws = XPLMFindPluginBySignature(OPENGPWS_PLUGIN_SIG);
	if (wxr->opengpws != XPLM_NO_PLUGIN_ID) {
		XPLMSendMessageToPlugin(wxr->opengpws, EGPWS_GET_INTF,
		    &wxr->terr);
	}

	worker_init(&wxr->wk, wxr_worker, WORKER_INTVAL, wxr, "OpenWXR-worker");

	return (wxr);
}

void
wxr_fini(wxr_t *wxr)
{
	if (!wxr->standby)
		worker_fini(&wxr->wk);

	if (wxr->tex[0] != 0)
		glDeleteTextures(2, wxr->tex);
	if (wxr->pbo != 0)
		glDeleteBuffers(1, &wxr->pbo);
	if (wxr->shadow_tex[0] != 0)
		glDeleteTextures(2, wxr->shadow_tex);
	if (wxr->shadow_pbo != 0)
		glDeleteBuffers(1, &wxr->shadow_pbo);

	glutils_destroy_quads(&wxr->wxr_scr_quads);

	free(wxr->samples);
	free(wxr->shadow_samples);
	free(wxr->sl.energy_out);
	free(wxr->sl.doppler_out);
	free(wxr->tp.in_pts);
	free(wxr->tp.out_elev);
	free(wxr->tp.out_norm);
	free(wxr->tp.out_water);

	if (wxr->wxr_prog != 0)
		glDeleteProgram(wxr->wxr_prog);

	mutex_destroy(&wxr->lock);

	free(wxr);
}

void
wxr_set_acf_pos(wxr_t *wxr, geo_pos3_t pos, vect3_t orient)
{
	ASSERT(!IS_NULL_GEO_POS(pos));
	ASSERT(!IS_NULL_VECT(orient));

	mutex_enter(&wxr->lock);
	wxr->acf_pos = pos;
	wxr->acf_orient = orient;
	mutex_exit(&wxr->lock);
}

void
wxr_set_scale(wxr_t *wxr, unsigned range_idx)
{
	double range;

	ASSERT3U(range_idx, <, wxr->conf->num_ranges);

	mutex_enter(&wxr->lock);
	wxr->cur_range = range_idx;
	range = wxr->conf->ranges[wxr->cur_range];
	mutex_exit(&wxr->lock);

	wxr->atmo->set_range(range);
}

unsigned
wxr_get_scale(const wxr_t *wxr)
{
	return (wxr->cur_range);
}

/*
 * `left' and `right' are in degrees from 0 (straight head).
 */
void
wxr_set_azimuth_limits(wxr_t *wxr, double left, double right)
{
	ASSERT3F(left, >=, -wxr->conf->scan_angle / 2);
	ASSERT3F(right, <=, wxr->conf->scan_angle / 2);

	mutex_enter(&wxr->lock);
	wxr->azi_lim_left = MAX(((left + wxr->conf->scan_angle / 2) /
	    wxr->conf->scan_angle) * wxr->conf->res_x, 0);
	wxr->azi_lim_right = MIN(((right + wxr->conf->scan_angle / 2) /
	    wxr->conf->scan_angle) * wxr->conf->res_x, wxr->conf->res_x - 1);
	mutex_exit(&wxr->lock);
}

double
wxr_get_ant_azimuth(const wxr_t *wxr)
{
	return ((((double)wxr->ant_pos / wxr->conf->res_x) - 0.5) *
	    wxr->conf->scan_angle);
}

void
wxr_set_ant_pitch(wxr_t *wxr, double angle)
{
	ASSERT3F(angle, <=, 90);
	ASSERT3F(angle, >=, -90);

	mutex_enter(&wxr->lock);
	wxr->ant_pitch_req = angle;
	mutex_exit(&wxr->lock);
}

double
wxr_get_ant_pitch(const wxr_t *wxr)
{
	/* TODO: this is broken */
	if (!wxr->vert_mode) {
		return (wxr->ant_pitch_req);
	} else {
		return (-((wxr->ant_pos_vert / (double)wxr->conf->res_x) -
		    0.5) * wxr->conf->scan_angle_vert);
	}
}

void
wxr_set_gain(wxr_t *wxr, double gain)
{
	ASSERT3F(gain, >=, 0.0);

	mutex_enter(&wxr->lock);
	wxr->gain = gain;
	mutex_exit(&wxr->lock);
}

double
wxr_get_gain(const wxr_t *wxr)
{
	return (wxr->gain);
}

/*
 * Sets how many degrees the radar auto-compensates for pitching and rolling
 * of the aircraft by counter-pitching & tilting the radar antenna to
 * maintain constant absolute antenna pitch & scanning across the horizon.
 * Pass 0 for either value for no stabilization.
 */
void
wxr_set_stab(wxr_t *wxr, double pitch, double roll)
{
	ASSERT3F(pitch, >=, 0);
	ASSERT3F(pitch, <=, 90);
	ASSERT3F(roll, >=, 0);
	ASSERT3F(roll, <=, 90);

	mutex_enter(&wxr->lock);
	wxr->pitch_stab = pitch;
	wxr->roll_stab = roll;
	mutex_exit(&wxr->lock);
}

void
wxr_get_stab(const wxr_t *wxr, bool_t *pitch, bool_t *roll)
{
	*pitch = wxr->pitch_stab;
	*roll = wxr->roll_stab;
}

static void
apply_pbo_tex(GLuint pbo, GLuint tex, GLuint res_x, GLuint res_y)
{
	XPLMBindTexture2d(tex, GL_TEXTURE_2D);
	glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbo);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, res_x, res_y, 0, GL_RGBA,
	    GL_UNSIGNED_BYTE, NULL);
}

static void
async_xfer_setup(GLuint pbo, void *buf, size_t sz)
{
	void *ptr;

	glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbo);
	glBufferData(GL_PIXEL_UNPACK_BUFFER, sz, 0, GL_STREAM_DRAW);
	ptr = glMapBuffer(GL_PIXEL_UNPACK_BUFFER, GL_WRITE_ONLY);
	if (ptr != NULL) {
		memcpy(ptr, buf, sz);
		glUnmapBuffer(GL_PIXEL_UNPACK_BUFFER);
	} else {
		logMsg("Error uploading WXR texture: "
		    "glMapBuffer returned NULL");
	}
}

GLuint
wxr_get_cur_tex(wxr_t *wxr, bool_t shadow_tex)
{
	uint64_t now = microclock();

	if (wxr->last_upload + TEX_UPD_INTVAL > now && wxr->upload_sync == 0)
		/* Previous upload still valid & nothing in flight */
		goto out;

	if (wxr->upload_sync != 0) {
		if (glClientWaitSync(wxr->upload_sync, 0, 0) !=
		    GL_TIMEOUT_EXPIRED) {
			/* Texture upload complete, apply the texture */
			wxr->upload_sync = 0;
			wxr->cur_tex = !wxr->cur_tex;

			apply_pbo_tex(wxr->pbo, wxr->tex[wxr->cur_tex],
			    wxr->conf->res_x, wxr->conf->res_y);
			apply_pbo_tex(wxr->shadow_pbo,
			    wxr->shadow_tex[wxr->cur_tex],
			    wxr->conf->res_x, wxr->conf->res_y);

			glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
		}
	} else {
		/*
		 * Set up a new async upload, it will most likely complete
		 * when we come through here again. But we memorize the
		 * current time as the time of the upload, so that we are
		 * not slipping frame timing.
		 */
		size_t sz = wxr->conf->res_x * wxr->conf->res_y *
		    sizeof (*wxr->samples);

		async_xfer_setup(wxr->pbo, wxr->samples, sz);
		async_xfer_setup(wxr->shadow_pbo, wxr->shadow_samples, sz);
		wxr->upload_sync =
		    glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
		wxr->last_upload = now;
	}

out:
	if (!shadow_tex)
		return (wxr->tex[wxr->cur_tex]);
	else
		return (wxr->shadow_tex[wxr->cur_tex]);
}

static void
wxr_bind_tex(wxr_t *wxr, bool_t shadow_tex)
{
	if (wxr->pbo == 0) {
		glGenBuffers(1, &wxr->pbo);
		glGenBuffers(1, &wxr->shadow_pbo);
	}

	if (wxr->tex[0] != 0) {
		GLuint tex = wxr_get_cur_tex(wxr, shadow_tex);
		ASSERT(tex != 0);
		glActiveTexture(GL_TEXTURE0);
		XPLMBindTexture2d(tex, GL_TEXTURE_2D);
	} else {
		/* initial texture upload, do a sync upload */
		ASSERT(wxr->cur_tex == 0);

		glGenTextures(2, wxr->tex);
		glGenTextures(2, wxr->shadow_tex);

		for (int i = 0; i < 2; i++) {
			XPLMBindTexture2d(wxr->tex[i], GL_TEXTURE_2D);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
			    GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
			    GL_LINEAR);

			XPLMBindTexture2d(wxr->shadow_tex[i], GL_TEXTURE_2D);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
			    GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
			    GL_LINEAR);
		}

		glActiveTexture(GL_TEXTURE0);

		XPLMBindTexture2d(wxr->tex[0], GL_TEXTURE_2D);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA,
		    wxr->conf->res_x, wxr->conf->res_y, 0,
		    GL_RGBA, GL_UNSIGNED_BYTE, wxr->samples);

		XPLMBindTexture2d(wxr->shadow_tex[0], GL_TEXTURE_2D);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA,
		    wxr->conf->res_x, wxr->conf->res_y, 0,
		    GL_RGBA, GL_UNSIGNED_BYTE, wxr->shadow_samples);

		if (!shadow_tex) {
			XPLMBindTexture2d(wxr->tex[0], GL_TEXTURE_2D);
		} else {
			XPLMBindTexture2d(wxr->shadow_tex[0], GL_TEXTURE_2D);
		}
	}
}

static void
wxr_draw_arc_recache(wxr_t *wxr, vect2_t pos, vect2_t size, bool_t vert)
{
	double scan_angle = !vert ? wxr->conf->scan_angle :
	    wxr->conf->scan_angle_vert;
	unsigned num_coords = ceil(scan_angle) * 4;
	vect2_t vtx[num_coords], tex[num_coords];

	for (unsigned i = 0, j = 0; i < num_coords; i += 4, j++) {
		/* Draw 1-degree increments */
		double fract1 = (double)j / scan_angle;
		double fract2 = (double)(j + 1) / scan_angle;
		double angle1 = DEG2RAD((fract1 * scan_angle) -
		    (scan_angle / 2));
		double angle2 = DEG2RAD((fract1 * scan_angle) -
		    (scan_angle / 2) + 1);

		/*
		 * Draw the quad in clockwise vertex order.
		 */

		/* lower-left */
		if (!vert)
			vtx[i] = VECT2(pos.x + size.x / 2, pos.y);
		else
			vtx[i] = VECT2(pos.x, pos.y + size.y / 2);
		tex[i] = VECT2(0, fract1);

		/* upper-left */
		if (!vert) {
			vtx[i + 1].x = (pos.x + size.x / 2) +
			    (sin(angle1) * (size.x / 2));
			vtx[i + 1].y = pos.y + (cos(angle1) * size.y);
		} else {
			vtx[i + 1].x = pos.x + (cos(angle1) * size.x);
			vtx[i + 1].y = (pos.y + size.y / 2) -
			    (sin(angle1) * (size.y / 2));
		}
		tex[i + 1] = VECT2(1, fract1);

		/* upper-right */
		if (!vert) {
			vtx[i + 2].x = (pos.x + size.x / 2) +
			    (sin(angle2) * (size.x / 2));
			vtx[i + 2].y = pos.y + (cos(angle2) * size.y);
		} else {
			vtx[i + 2].x = pos.x + (cos(angle2) * size.x);
			vtx[i + 2].y = (pos.y + size.y / 2) -
			    (sin(angle2) * (size.y / 2));
		}
		tex[i + 2] = VECT2(1, fract2);

		/* lower-right */
		if (!vert)
			vtx[i + 3] = VECT2(pos.x + size.x / 2, pos.y);
		else
			vtx[i + 3] = VECT2(pos.x, pos.y + size.y / 2);
		tex[i + 3] = VECT2(0, fract2);
	}

	glutils_destroy_quads(&wxr->wxr_scr_quads);
	glutils_init_2D_quads(&wxr->wxr_scr_quads, vtx, tex, num_coords);
}

static void
wxr_draw_arc(wxr_t *wxr, vect2_t pos, vect2_t size)
{
	if (!VECT2_EQ(pos, wxr->draw_pos) || !VECT2_EQ(size, wxr->draw_size) ||
	    wxr->draw_vert != wxr->vert_mode) {
		wxr_draw_arc_recache(wxr, pos, size, wxr->vert_mode);
		wxr->draw_pos = pos;
		wxr->draw_size = size;
		wxr->draw_vert = wxr->vert_mode;
	}

	glUseProgram(wxr->wxr_prog);

	glUniformMatrix4fv(glGetUniformLocation(wxr->wxr_prog, "pvm"),
	    1, GL_FALSE, (GLfloat *)wxr->wxr_scr_pvm);
	glUniform1i(glGetUniformLocation(wxr->wxr_prog, "tex"), 0);
	glUniform2f(glGetUniformLocation(wxr->wxr_prog, "tex_size"),
	    wxr->conf->res_x, wxr->conf->res_y);
	glUniform1f(glGetUniformLocation(wxr->wxr_prog, "smear_mult"),
	    wxr->vert_mode ? wxr->conf->smear.y : wxr->conf->smear.x);

	glutils_draw_quads(&wxr->wxr_scr_quads);

	glUseProgram(0);
}

static void
wxr_draw_square(wxr_t *wxr, vect2_t pos, vect2_t size)
{
	glUseProgram(wxr->wxr_prog);

	glUniformMatrix4fv(glGetUniformLocation(wxr->wxr_prog, "pvm"),
	    1, GL_FALSE, (GLfloat *)wxr->wxr_scr_pvm);
	glUniform1i(glGetUniformLocation(wxr->wxr_prog, "tex"), 0);
	glUniform2f(glGetUniformLocation(wxr->wxr_prog, "tex_size"),
	    wxr->conf->res_x, wxr->conf->res_y);

	if (!VECT2_EQ(pos, wxr->draw_pos) || !VECT2_EQ(size, wxr->draw_size) ||
	    wxr->draw_vert != wxr->vert_mode) {
		vect2_t vtx[4];
		vect2_t tex[4] = {
		    VECT2(0, 0), VECT2(1, 0), VECT2(1, 1), VECT2(0, 1)
		};

		if (!wxr->vert_mode) {
			vtx[0] = VECT2(pos.x, pos.y);
			vtx[1] = VECT2(pos.x, pos.y + size.y);
			vtx[2] = VECT2(pos.x + size.x, pos.y + size.y);
			vtx[3] = VECT2(pos.x + size.x, pos.y);
		} else {
			vtx[0] = VECT2(pos.x, pos.y + size.y);
			vtx[1] = VECT2(pos.x + size.x, pos.y + size.y);
			vtx[2] = VECT2(pos.x + size.x, pos.y);
			vtx[3] = VECT2(pos.x, pos.y);
		}
		glutils_destroy_quads(&wxr->wxr_scr_quads);
		glutils_init_2D_quads(&wxr->wxr_scr_quads, vtx, tex, 4);

		wxr->draw_pos = pos;
		wxr->draw_size = size;
		wxr->draw_vert = wxr->vert_mode;
	}

	glutils_draw_quads(&wxr->wxr_scr_quads);

	glUseProgram(0);
}

void
wxr_draw(wxr_t *wxr, vect2_t pos, vect2_t size)
{
	glGetError();
	XPLMSetGraphicsState(0, 1, 0, 1, 1, 1, 1);
	wxr_bind_tex(wxr, B_FALSE);
	if (wxr->conf->disp_type == WXR_DISP_ARC) {
		wxr_draw_arc(wxr, pos, size);
	} else {
		ASSERT3U(wxr->conf->disp_type, ==, WXR_DISP_SQUARE);
		wxr_draw_square(wxr, pos, size);
	}
	wxr_bind_tex(wxr, B_TRUE);
	if (wxr->conf->disp_type == WXR_DISP_ARC)
		wxr_draw_arc(wxr, pos, size);
	else
		wxr_draw_square(wxr, pos, size);
}

void
wxr_set_beam_shadow(wxr_t *wxr, bool_t flag)
{
	wxr->beam_shadow = flag;
}

bool_t
wxr_get_beam_shadow(const wxr_t *wxr)
{
	return (wxr->beam_shadow);
}

void
wxr_set_standby(wxr_t *wxr, bool_t flag)
{
	if (wxr->standby != flag) {
		wxr->standby = flag;
		wxr->vert_mode = B_FALSE;
		if (flag) {
			worker_fini(&wxr->wk);
			wxr_ant_return2neutral(wxr);
			memset(wxr->samples, 0, sizeof (*wxr->samples) *
			    wxr->conf->res_x * wxr->conf->res_y);
			memset(wxr->shadow_samples, 0, sizeof (*wxr->samples) *
			    wxr->conf->res_x * wxr->conf->res_y);
		} else {
			worker_init(&wxr->wk, wxr_worker, WORKER_INTVAL, wxr,
			    "OpenWXR-worker");
		}
	}
}

bool_t
wxr_get_standby(const wxr_t *wxr)
{
	return (wxr->standby);
}

void
wxr_clear_screen(wxr_t *wxr)
{
	if (!wxr->standby)
		mutex_enter(&wxr->wk.lock);

	mutex_enter(&wxr->lock);
	memset(wxr->samples, 0,
	    sizeof (*wxr->samples) * wxr->conf->res_x * wxr->conf->res_y);
	memset(wxr->shadow_samples, 0,
	    sizeof (*wxr->samples) * wxr->conf->res_x * wxr->conf->res_y);
	mutex_exit(&wxr->lock);

	if (!wxr->standby)
		mutex_exit(&wxr->wk.lock);
}

void
wxr_set_vert_mode(wxr_t *wxr, bool_t flag, double azimuth)
{
	if (!wxr->standby)
		mutex_enter(&wxr->wk.lock);

	mutex_enter(&wxr->lock);

	ASSERT3F(ABS(azimuth), <=, wxr->conf->scan_angle / 2);

	if (flag) {
		wxr->ant_pos = clampi(((azimuth + wxr->conf->scan_angle / 2) /
		    wxr->conf->scan_angle) * wxr->conf->res_x, 0,
		    wxr->conf->res_x - 1);
	}
	if (flag && !wxr->vert_mode) {
		wxr->vert_mode = B_TRUE;
		wxr->ant_pos_vert = clampi(((wxr->ant_pitch_req +
		    wxr->conf->scan_angle_vert / 2) /
		    wxr->conf->scan_angle_vert) * wxr->conf->res_x, 0,
		    wxr->conf->res_x - 1);
		memset(wxr->samples, 0, sizeof (*wxr->samples) *
		    wxr->conf->res_x * wxr->conf->res_y);
	} else if (!flag && wxr->vert_mode) {
		wxr->vert_mode = B_FALSE;
		memset(wxr->samples, 0, sizeof (*wxr->samples) *
		    wxr->conf->res_x * wxr->conf->res_y);
		memset(wxr->shadow_samples, 0, sizeof (*wxr->samples) *
		    wxr->conf->res_x * wxr->conf->res_y);
	}

	mutex_exit(&wxr->lock);

	if (!wxr->standby)
		mutex_exit(&wxr->wk.lock);
}

bool_t
wxr_get_vert_mode(const wxr_t *wxr)
{
	return (wxr->vert_mode);
}

/*
 * Colors should be in big-endian RGBA ('R' in top bits, 'A' in bottom bits).
 */
void
wxr_set_colors(wxr_t *wxr, const wxr_color_t *colors, size_t num)
{
	if (colors != wxr->colors) {
		mutex_enter(&wxr->lock);
		wxr->colors = colors;
		wxr->num_colors = num;
		mutex_exit(&wxr->lock);
	}
}

bool_t
wxr_reload_gl_progs(wxr_t *wxr)
{
	char *vtx = mkpathname(get_xpdir(), get_plugindir(), "data",
	    "smear.vert", NULL);
	char *frag = mkpathname(get_xpdir(), get_plugindir(), "data",
	    "smear.frag", NULL);
	GLuint prog = shader_prog_from_file("smear", vtx, frag,
	    DEFAULT_VTX_ATTRIB_BINDINGS, NULL);
	lacf_free(vtx);
	lacf_free(frag);

	if (prog == 0)
		return (B_FALSE);

	if (wxr->wxr_prog != 0)
		glDeleteProgram(wxr->wxr_prog);
	wxr->wxr_prog = prog;

	return (B_TRUE);
}
