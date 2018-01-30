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

#include <XPLMGraphics.h>

#include <acfutils/assert.h>
#include <acfutils/helpers.h>
#include <acfutils/shader.h>
#include <acfutils/thread.h>
#include <acfutils/time.h>
#include <acfutils/worker.h>

#include "wxr.h"
#include "xplane.h"

#define	TEX_UPD_INTVAL	40000		/* us, 25 fps */
#define	WORKER_INTVAL	33333		/* us, 30 fps */
#define	MAX_BEAM_ENERGY	1		/* dBmW */

struct wxr_s {
	const wxr_conf_t	*conf;
	const atmo_t		*atmo;

	/* only accessed by foreground thread */
	unsigned		cur_tex;
	GLuint			tex[2];
	GLuint			pbo;
	GLsync			upload_sync;
	uint64_t		last_upload;
	GLuint			wxr_prog;

	vect2_t			draw_pos;
	vect2_t			draw_size;
	size_t			draw_num_coords;
	GLfloat			*draw_vtx_coords;
	GLfloat			*draw_tex_coords;

	mutex_t			lock;
	/* protected by lock above */
	bool_t			standby;
	geo_pos3_t		acf_pos;
	vect3_t			acf_orient;
	unsigned		cur_range;
	double			gain;
	double			ant_pitch_req;
	unsigned		azi_lim_left;
	unsigned		azi_lim_right;
	double			pitch_stab;
	double			roll_stab;

	/* only accessed from worker thread */
	unsigned		ant_pos;
	bool_t			scan_right;
	scan_line_t		sl;

	/* unstructured, always safe to read & write */
	uint32_t		*samples;
	bool_t			beam_shadow;

	worker_t		wk;
};

static bool_t
wxr_worker(void *userinfo)
{
	wxr_t *wxr = userinfo;
	double ant_pitch, acf_hdg;

	mutex_enter(&wxr->lock);

	wxr->sl.origin = wxr->acf_pos;
	wxr->sl.shape = wxr->conf->beam_shape;
	wxr->sl.range = wxr->conf->ranges[wxr->cur_range];
	wxr->sl.energy = MAX_BEAM_ENERGY;
	wxr->sl.max_range = wxr->conf->ranges[wxr->conf->num_ranges - 1];
	wxr->sl.num_samples = wxr->conf->res_y;
	ant_pitch = wxr->ant_pitch_req;
	acf_hdg = wxr->acf_orient.y;

	mutex_exit(&wxr->lock);

	for (unsigned i = 0;
	    i < (unsigned)(wxr->conf->res_x * (USEC2SEC(WORKER_INTVAL) /
	    wxr->conf->scan_time)); i++) {
		double ant_hdg;
		int off;
		double energy_spent = 0;

		/* Move the antenna by one notch left/right */
		if (wxr->scan_right) {
			wxr->ant_pos++;
			if (wxr->ant_pos == wxr->conf->res_x - 1 ||
			    wxr->ant_pos == wxr->azi_lim_right)
				wxr->scan_right = B_FALSE;
		} else {
			wxr->ant_pos--;
			if (wxr->ant_pos == 0 ||
			    wxr->ant_pos == wxr->azi_lim_left)
				wxr->scan_right = B_TRUE;
		}

		off = wxr->ant_pos * wxr->conf->res_y;

		wxr->sl.ant_rhdg = (wxr->conf->scan_angle *
		    ((wxr->ant_pos / (double)wxr->conf->res_x) - 0.5));
		ant_hdg = acf_hdg + wxr->sl.ant_rhdg;
		wxr->sl.dir = VECT2(ant_hdg, ant_pitch);

		wxr->atmo->probe(&wxr->sl);

		/*
		 * No need to lock the samples, worst case is we will
		 * draw a partially updated scan line - no big deal.
		 */
		for (unsigned j = 0; j < wxr->conf->res_y; j++) {
			double energy = wxr->sl.energy_out[j];
			double sample_sz = wxr->sl.range / wxr->sl.num_samples;
			double sample_sz_rat = sample_sz / 1000.0;
			double abs_energy = energy / sample_sz_rat;

			energy_spent += energy;
			if (energy_spent > 0.8 && wxr->beam_shadow) {
				wxr->samples[off + j] = 0xff505050u;
			} else if (wxr->gain * abs_energy >= 0.02 * 0.6)
				wxr->samples[off + j] = 0xffa564d8u;
			else if (wxr->gain * abs_energy >= 0.03 / 2 * 0.6)
				wxr->samples[off + j] = 0xff2420edu;
			else if (wxr->gain * abs_energy >= 0.03 / 3 * 0.6)
				wxr->samples[off + j] = 0xff00f2ffu;
			else if (wxr->gain * abs_energy >= 0.03 / 4 * 0.6)
				wxr->samples[off + j] = 0xff55c278u;
			else
				wxr->samples[off + j] = 0xff000000u;
		}
	}

	return (B_TRUE);
}

wxr_t *
wxr_init(const wxr_conf_t *conf, const atmo_t *atmo)
{
	char *vtx, *frag;
	wxr_t *wxr = calloc(1, sizeof (*wxr));

	ASSERT(conf->num_ranges != 0);
	ASSERT3U(conf->num_ranges, <, WXR_MAX_RANGES);
	ASSERT3U(conf->res_x, >=, WXR_MIN_RES);
	ASSERT3U(conf->res_y, >=, WXR_MIN_RES);
	ASSERT3F(conf->beam_shape.x, >, 0);
	ASSERT3F(conf->beam_shape.y, >, 0);
	ASSERT3F(conf->scan_time, >, 0);
	ASSERT3F(conf->scan_angle, >, 0);
	ASSERT(atmo->probe != NULL);

	wxr->conf = conf;
	wxr->atmo = atmo;
	wxr->gain = 1.0;
	/*
	 * 4 vertices per quad, 2 coords per vertex
	 */
	wxr->draw_num_coords = 4 * 2 * ceil(conf->scan_angle);
	wxr->samples = calloc(conf->res_x * conf->res_y,
	    sizeof (*wxr->samples));
	wxr->ant_pos = conf->res_x / 2;
	wxr->azi_lim_right = conf->res_x - 1;
	wxr->sl.energy_out = calloc(wxr->conf->res_y, sizeof (double));
	wxr->sl.doppler_out = calloc(wxr->conf->res_y, sizeof (double));
	wxr->atmo->set_range(wxr->conf->ranges[0]);

	vtx = mkpathname(get_xpdir(), get_plugindir(), "data",
	    "smear_vtx.glsl", NULL);
	frag = mkpathname(get_xpdir(), get_plugindir(), "data",
	    "smear_frag.glsl", NULL);
	wxr->wxr_prog = shader_prog_from_file("smear", vtx, frag);
	lacf_free(vtx);
	lacf_free(frag);

	worker_init(&wxr->wk, wxr_worker, WORKER_INTVAL, wxr);

	return (wxr);
}

void
wxr_fini(wxr_t *wxr)
{
	if (wxr->tex != 0)
		glDeleteTextures(2, wxr->tex);
	if (wxr->pbo != 0)
		glDeleteBuffers(1, &wxr->pbo);

	free(wxr->draw_vtx_coords);
	free(wxr->draw_tex_coords);

	free(wxr->samples);
	free(wxr->sl.energy_out);
	free(wxr->sl.doppler_out);
	if (!wxr->standby)
		worker_fini(&wxr->wk);

	if (wxr->wxr_prog != 0)
		glDeleteProgram(wxr->wxr_prog);

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
	return (wxr->ant_pitch_req);
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

GLuint
wxr_get_cur_tex(wxr_t *wxr)
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

			XPLMBindTexture2d(wxr->tex[wxr->cur_tex],
			    GL_TEXTURE_2D);
			glBindBuffer(GL_PIXEL_UNPACK_BUFFER, wxr->pbo);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA,
			    wxr->conf->res_x, wxr->conf->res_y, 0, GL_RGBA,
			    GL_UNSIGNED_BYTE, NULL);
			glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
		}
	} else {
		/*
		 * Set up a new async upload, it will most likely complete
		 * when we come through here again. But we memorize the
		 * current time as the time of the upload, so that we are
		 * not slipping frame timing.
		 */
		void *ptr;
		size_t sz = wxr->conf->res_x * wxr->conf->res_y *
		    sizeof (*wxr->samples);

		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, wxr->pbo);
		glBufferData(GL_PIXEL_UNPACK_BUFFER, sz, 0, GL_STREAM_DRAW);
		ptr = glMapBuffer(GL_PIXEL_UNPACK_BUFFER, GL_WRITE_ONLY);
		if (ptr != NULL) {
			memcpy(ptr, wxr->samples, sz);
			glUnmapBuffer(GL_PIXEL_UNPACK_BUFFER);
			wxr->upload_sync =
			    glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
		} else {
			logMsg("Error uploading WXR instance %p: "
			    "glMapBuffer returned NULL", wxr);
		}
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
		wxr->last_upload = now;
	}

out:
	return (wxr->tex[wxr->cur_tex]);
}

static void
wxr_bind_tex(wxr_t *wxr)
{
	if (wxr->pbo == 0)
		glGenBuffers(1, &wxr->pbo);

	if (wxr->tex[0] != 0) {
		GLuint tex = wxr_get_cur_tex(wxr);
		ASSERT(tex != 0);
		glActiveTexture(GL_TEXTURE0);
		XPLMBindTexture2d(tex, GL_TEXTURE_2D);
	} else {
		/* initial texture upload, do a sync upload */
		ASSERT(wxr->cur_tex == 0);

		glGenTextures(2, wxr->tex);
		for (int i = 0; i < 2; i++) {
			XPLMBindTexture2d(wxr->tex[i], GL_TEXTURE_2D);
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
	}
}

static void
wxr_draw_arc_recache(wxr_t *wxr, vect2_t pos, vect2_t size)
{
	if (wxr->draw_vtx_coords == NULL) {
		wxr->draw_vtx_coords = calloc(wxr->draw_num_coords,
		    sizeof (*wxr->draw_vtx_coords));
		wxr->draw_tex_coords = calloc(wxr->draw_num_coords,
		    sizeof (*wxr->draw_tex_coords));
	}

	for (unsigned i = 0, j = 0; i < wxr->draw_num_coords; i += 4 * 2, j++) {
		/* Draw 1-degree increments */
		double fract1 = (double)j / wxr->conf->scan_angle;
		double fract2 = (double)(j + 1) / wxr->conf->scan_angle;
		double angle1 = DEG2RAD((fract1 * wxr->conf->scan_angle) -
		    (wxr->conf->scan_angle / 2));
		double angle2 = DEG2RAD((fract1 * wxr->conf->scan_angle) -
		    (wxr->conf->scan_angle / 2) + 1);

		/*
		 * Draw the quad in clockwise vertex order.
		 */

		/* lower-left */
		wxr->draw_vtx_coords[i] = pos.x + size.x / 2;
		wxr->draw_tex_coords[i] = 0;

		wxr->draw_vtx_coords[i + 1] = pos.y;
		wxr->draw_tex_coords[i + 1] = fract1;

		/* upper-left */
		wxr->draw_vtx_coords[i + 2] = (pos.x + size.x / 2) +
		    (sin(angle1) * (size.x / 2));
		wxr->draw_tex_coords[i + 2] = 1;

		wxr->draw_vtx_coords[i + 3] = pos.y + (cos(angle1) * size.y);
		wxr->draw_tex_coords[i + 3] = fract1;

		/* upper-right */
		wxr->draw_vtx_coords[i + 4] = (pos.x + size.x / 2) +
		    (sin(angle2) * (size.x / 2));
		wxr->draw_tex_coords[i + 4] = 1;

		wxr->draw_vtx_coords[i + 5] = pos.y + (cos(angle2) * size.y);
		wxr->draw_tex_coords[i + 5] = fract2;

		/* lower-right */
		wxr->draw_vtx_coords[i + 6] = pos.x + size.x / 2;
		wxr->draw_tex_coords[i + 6] = 0;

		wxr->draw_vtx_coords[i + 7] = pos.y;
		wxr->draw_tex_coords[i + 7] = fract2;
	}
}

static void
wxr_draw_arc(wxr_t *wxr, vect2_t pos, vect2_t size)
{
	if (!VECT2_EQ(pos, wxr->draw_pos) || !VECT2_EQ(size, wxr->draw_size)) {
		wxr_draw_arc_recache(wxr, pos, size);
		wxr->draw_pos = pos;
		wxr->draw_size = size;
	}

	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);

	glUseProgram(wxr->wxr_prog);
	glUniform1i(glGetUniformLocation(wxr->wxr_prog, "tex"), 0);
	glUniform2f(glGetUniformLocation(wxr->wxr_prog, "tex_size"),
	    wxr->conf->res_x, wxr->conf->res_y);

	glVertexPointer(2, GL_FLOAT, 0, wxr->draw_vtx_coords);
	glTexCoordPointer(2, GL_FLOAT, 0, wxr->draw_tex_coords);
	glDrawArrays(GL_QUADS, 0, wxr->draw_num_coords / 2);

	glUseProgram(0);

	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
}

static void
wxr_draw_square(wxr_t *wxr, vect2_t pos, vect2_t size)
{
	glUseProgram(wxr->wxr_prog);
	glUniform1i(glGetUniformLocation(wxr->wxr_prog, "tex"), 0);
	glUniform2f(glGetUniformLocation(wxr->wxr_prog, "tex_size"),
	    wxr->conf->res_x, wxr->conf->res_y);

	glBegin(GL_QUADS);
	glTexCoord2f(0, 0);
	glVertex2f(pos.x, pos.y);
	glTexCoord2f(1, 0);
	glVertex2f(pos.x, pos.y + size.y);
	glTexCoord2f(1, 1);
	glVertex2f(pos.x + size.x, pos.y + size.y);
	glTexCoord2f(0, 1);
	glVertex2f(pos.x + size.x, pos.y);
	glEnd();

	glUseProgram(0);
}

void
wxr_draw(wxr_t *wxr, vect2_t pos, vect2_t size)
{
	glGetError();
	XPLMSetGraphicsState(0, 1, 0, 1, 1, 1, 1);
	wxr_bind_tex(wxr);
	if (wxr->conf->disp_type == WXR_DISP_ARC) {
		wxr_draw_arc(wxr, pos, size);
	} else {
		ASSERT3U(wxr->conf->disp_type, ==, WXR_DISP_SQUARE);
		wxr_draw_square(wxr, pos, size);
	}
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
		if (flag) {
			worker_fini(&wxr->wk);
			wxr->ant_pos = wxr->conf->res_x / 2;
			memset(wxr->samples, 0, sizeof (*wxr->samples) *
			    wxr->conf->res_x * wxr->conf->res_y);
		} else {
			worker_init(&wxr->wk, wxr_worker, WORKER_INTVAL, wxr);
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
	mutex_enter(&wxr->wk.lock);
	mutex_enter(&wxr->lock);
	memset(wxr->samples, 0, sizeof (*wxr->samples) * wxr->conf->res_x *
	    wxr->conf->res_y);
	mutex_exit(&wxr->lock);
	mutex_exit(&wxr->wk.lock);
}
