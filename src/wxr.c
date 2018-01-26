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
#include <acfutils/thread.h>
#include <acfutils/time.h>
#include <acfutils/worker.h>

#include "wxr.h"

#define	TEX_UPD_INTVAL	40000		/* us, 25 fps */
#define	WORKER_INTVAL	33333		/* us, 30 fps */
#define	MAX_BEAM_ENERGY	100		/* dBmW */

struct wxr_s {
	const wxr_conf_t	*conf;
	const atmo_t		*atmo;

	/* only accessed by foreground thread */
	unsigned		cur_tex;
	GLuint			tex[2];
	GLuint			pbo;
	GLsync			upload_sync;
	uint64_t		last_upload;

	mutex_t			lock;
	/* protected by lock above */
	geo_pos3_t		acf_pos;
	vect3_t			acf_orient;
	unsigned		cur_range;
	double			gain;
	double			ant_pitch_req;

	/* only accessed from worker thread */
	unsigned		ant_pos;
	bool_t			scan_right;
	scan_line_t		sl;

	/* unstructured, always safe to read & write */
	uint8_t			*samples;

	worker_t		wk;
};

static bool_t
wxr_worker(void *userinfo)
{
	wxr_t *wxr = userinfo;
	scan_line_t sl;
	double ant_pitch, acf_hdg;

	mutex_enter(&wxr->lock);

	sl.origin = wxr->acf_pos;
	sl.shape = wxr->conf->beam_shape;
	sl.energy = MAX_BEAM_ENERGY * wxr->gain;
	sl.range = wxr->conf->ranges[wxr->cur_range];
	sl.num_samples = wxr->conf->res_y;
	ant_pitch = wxr->acf_orient.x + wxr->ant_pitch_req;
	acf_hdg = wxr->acf_orient.y;

	mutex_exit(&wxr->lock);

	for (unsigned i = 0;
	    i < (unsigned)(wxr->conf->res_x * USEC2SEC(WORKER_INTVAL)); i++) {
		double ant_hdg;

		/* Move the antenna by one notch left/right */
		if (wxr->scan_right) {
			wxr->ant_pos++;
			if (wxr->ant_pos == wxr->conf->res_x - 1)
				wxr->scan_right = B_FALSE;
		} else {
			wxr->ant_pos--;
			if (wxr->ant_pos == 0)
				wxr->scan_right = B_TRUE;
		}

		ant_hdg = acf_hdg + (wxr->conf->scan_angle *
		    ((wxr->ant_pos / (double)wxr->conf->res_x) - 0.5));
		sl.dir = VECT2(ant_hdg, ant_pitch);

		wxr->atmo->probe(&sl);

		/*
		 * No need to lock the samples, worst case is we will
		 * draw a partially updated scan line - no big deal.
		 */
		for (unsigned j = 0; j < wxr->conf->res_y; j++) {
			wxr->samples[j] =
			    (sl.energy_out[j] / MAX_BEAM_ENERGY) * UINT8_MAX;
		}
	}

	free(sl.energy_out);
	free(sl.doppler_out);

	return (B_TRUE);
}

wxr_t *
wxr_init(const wxr_conf_t *conf, const atmo_t *atmo)
{
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
	wxr->samples = calloc(conf->res_x * conf->res_y, 1);
	wxr->ant_pos = conf->res_x / 2;
	wxr->sl.energy_out = calloc(wxr->conf->res_y, sizeof (double));
	wxr->sl.doppler_out = calloc(wxr->conf->res_y, sizeof (double));
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

	free(wxr->samples);
	free(wxr->sl.energy_out);
	free(wxr->sl.doppler_out);
	worker_fini(&wxr->wk);

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
	ASSERT3U(range_idx, <, wxr->conf->num_ranges);

	mutex_enter(&wxr->lock);
	wxr->cur_range = range_idx;
	mutex_exit(&wxr->lock);
}

void
wxr_set_pitch(wxr_t *wxr, double angle)
{
	ASSERT3F(angle, <=, 90);
	ASSERT3F(angle, >=, -90);

	mutex_enter(&wxr->lock);
	wxr->ant_pitch_req = angle;
	mutex_exit(&wxr->lock);
}

void
wxr_set_gain(wxr_t *wxr, double gain)
{
	ASSERT3F(gain, >=, 0.0);
	ASSERT3F(gain, <=, 1.0);

	mutex_enter(&wxr->lock);
	wxr->gain = gain;
	mutex_exit(&wxr->lock);
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
			glTexImage2D(GL_TEXTURE_2D, 0, GL_R, wxr->conf->res_x,
			    wxr->conf->res_y, 0, GL_R, GL_UNSIGNED_BYTE, NULL);
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
		size_t sz = wxr->conf->res_x * wxr->conf->res_y;

		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, wxr->pbo);
		glBufferData(GL_PIXEL_UNPACK_BUFFER, sz, 0, GL_STREAM_DRAW);
		ptr = glMapBuffer(GL_PIXEL_UNPACK_BUFFER, GL_WRITE_ONLY);
		if (ptr != NULL) {
			memcpy(ptr, wxr->samples, sz);
			glUnmapBuffer(GL_PIXEL_UNPACK_BUFFER);
			wxr->upload_sync = glFenceSync(
			    GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
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
		XPLMBindTexture2d(wxr->tex[0], GL_TEXTURE_2D);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_R,
		    wxr->conf->res_x, wxr->conf->res_y, 0,
		    GL_R, GL_UNSIGNED_BYTE, wxr->samples);
	}
}

static void
wxr_draw_arc(wxr_t *wxr, vect2_t pos, vect2_t size)
{
	UNUSED(wxr);
	UNUSED(pos);
	UNUSED(size);
}

static void
wxr_draw_square(vect2_t pos, vect2_t size)
{
	glBegin(GL_QUADS);
	glTexCoord2f(0, 1);
	glVertex2f(pos.x, pos.y);
	glTexCoord2f(0, 0);
	glVertex2f(pos.x, pos.y + size.y);
	glTexCoord2f(1, 0);
	glVertex2f(pos.x + size.x, pos.y + size.y);
	glTexCoord2f(1, 1);
	glVertex2f(pos.x + size.x, pos.y);
	glEnd();
}

void
wxr_draw(wxr_t *wxr, vect2_t pos, vect2_t size)
{
	wxr_bind_tex(wxr);
	if (wxr->conf->disp_type == WXR_DISP_ARC) {
		wxr_draw_arc(wxr, pos, size);
	} else {
		ASSERT3U(wxr->conf->disp_type, ==, WXR_DISP_SQUARE);
		wxr_draw_square(pos, size);
	}
}
