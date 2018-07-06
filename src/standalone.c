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

#include <XPLMDisplay.h>
#include <XPLMPlugin.h>
#include <XPLMProcessing.h>
#include <XPLMUtilities.h>

#include <acfutils/dr.h>
#include <acfutils/helpers.h>
#include <acfutils/math.h>
#include <acfutils/mt_cairo_render.h>
#include <acfutils/perf.h>

#include <opengpws/xplane_api.h>

#include "../api/openwxr/wxr_intf.h"
#include "../api/openwxr/xplane_api.h"

#include "standalone.h"

#define	MAX_SCREENS	4
#define	MAX_MODES	16
#define	MAX_COLORS	8

#define	EFIS_OFF_X	16
#define	EFIS_OFF_Y	15
#define	EFIS_WIDTH	194
#define	EFIS_HEIGHT	268
#define	MAX_DR_NAME	128

#define	COLOR(c, scr)	((c) * ((pow(4.0, (scr)->brt / 0.75)) / 4.0))
#define	CYAN_RGB(scr)	COLOR(0, (scr)), COLOR(0.66, (scr)), COLOR(0.66, (scr))

static bool_t inited = B_FALSE;

typedef struct {
	char		name[MAX_DR_NAME];
	bool_t		have_dr;
	dr_t		dr;
} delayed_dr_t;

typedef enum {
	PANEL_RENDER_TYPE_2D = 0,
	PANEL_RENDER_TYPE_3D_UNLIT = 1,
	PANEL_RENDER_TYPE_3D_LIT = 2
} panel_render_type_t;

typedef struct wxr_sys_s wxr_sys_t;

typedef struct {
	double			x, y;
	double			w, h;
	struct wxr_sys_s	*sys;
	mt_cairo_render_t	*mtcr;

	double			power;
	double			power_on_rate;
	double			power_off_rate;
	double			brt;

	delayed_dr_t		power_dr;
	delayed_dr_t		brt_dr;
	double			scr_temp;
} wxr_scr_t;

typedef struct {
	vect2_t			stab_lim;
	unsigned		num_colors;
	wxr_color_t		colors[MAX_COLORS];
	wxr_color_t		base_colors[MAX_COLORS];
} mode_aux_info_t;

struct wxr_sys_s {
	double			power_on_time;
	double			power_on_delay;

	unsigned		num_modes;
	unsigned		cur_mode;
	wxr_conf_t		modes[MAX_MODES];
	mode_aux_info_t		aux[MAX_MODES];

	unsigned		num_screens;
	wxr_scr_t		screens[MAX_SCREENS];

	unsigned		efis_xywh[4];

	delayed_dr_t		power_dr;
	delayed_dr_t		mode_dr;
	delayed_dr_t		tilt_dr;
	delayed_dr_t		range_dr;
	delayed_dr_t		gain_dr;

	double			gain_auto_pos;
	bool_t			shared_egpws;
	const egpws_intf_t	*terr;
};

static wxr_sys_t sys = { 0 };
static XPLMWindowID debug_win = NULL;
static XPLMCommandRef open_debug_cmd = NULL;

static struct {
	dr_t		sim_time;
	dr_t		panel_render_type;
} drs;

static XPLMPluginID openwxr = XPLM_NO_PLUGIN_ID;
static void *atmo = NULL;
static void *wxr = NULL;
static openwxr_intf_t *intf = NULL;

static const egpws_conf_t egpws_conf = { .type = EGPWS_DB_ONLY };
static const egpws_range_t egpws_ranges[] = {
    { NM2MET(10), 100 },
    { NM2MET(25), 175 },
    { NM2MET(50), 250 },
    { NM2MET(100), 500 },
    { NM2MET(200), 1000 },
    { NAN, NAN }
};

#define	WXR_RES_X	320
#define	WXR_RES_Y	240

#define	MIN_GAIN	0.5	/* gain argument to WXR */
#define	MAX_GAIN	1.5	/* gain argument to WXR */
#define	DFL_GAIN	1.0	/* gain argument to WXR */

static void draw_debug_win(XPLMWindowID win, void *refcon);

#define	DELAYED_DR_OP(delayed_dr, op) \
	do { \
		delayed_dr_t *dd = (delayed_dr); \
		if (*dd->name != 0) { \
			if (!dd->have_dr) { \
				dd->have_dr = dr_find(&dd->dr, "%s", \
				    dd->name); \
			} \
			if (dd->have_dr) { \
				op; \
			} \
		} \
	} while (0)

static int
open_debug_win(XPLMCommandRef ref, XPLMCommandPhase phase, void *refcon)
{
	UNUSED(ref);
	UNUSED(phase);
	UNUSED(refcon);

	if (debug_win == NULL) {
		XPLMCreateWindow_t cr = {
		    .structSize = sizeof (cr), .visible = B_TRUE,
		    .left = 100, .top = 400, .right = 400, .bottom = 100,
		    .drawWindowFunc = draw_debug_win,
		    .decorateAsFloatingWindow =
		    xplm_WindowDecorationRoundRectangle,
		    .layer = xplm_WindowLayerFloatingWindows
		};
		debug_win = XPLMCreateWindowEx(&cr);
	}
	XPLMSetWindowIsVisible(debug_win, B_TRUE);
	XPLMBringWindowToFront(debug_win);

	return (1);
}

static float
floop_cb(float d_t, float elapsed, int counter, void *refcon)
{
	const wxr_conf_t *mode;
	mode_aux_info_t *aux;

	UNUSED(elapsed);
	UNUSED(counter);
	UNUSED(refcon);

	if (sys.shared_egpws && !sys.terr->is_inited())
		return (-1);

	/*
	 * Set up OpenGPWS as we need it:
	 * 1) DB-ONLY mode (no active EGPWS callouts)
	 * 2) enable sound playback (for our PWS callouts)
	 * 3) position known to run the terrain DB
	 * 4) nav systems on
	 */
	if (!sys.shared_egpws) {
		sys.terr->set_state(&egpws_conf);
		sys.terr->set_sound_inh(B_FALSE);
		sys.terr->set_ranges(egpws_ranges);
		sys.terr->set_pos_ok(B_TRUE);
		sys.terr->set_nav_on(B_TRUE, B_TRUE);
	}

	DELAYED_DR_OP(&sys.mode_dr, sys.cur_mode = dr_geti(&sys.mode_dr.dr));
	sys.cur_mode = MIN(sys.cur_mode, sys.num_modes - 1);

	mode = &sys.modes[sys.cur_mode];
	aux = &sys.aux[sys.cur_mode];
	if (wxr != NULL && mode->num_ranges == 0) {
		intf->fini(wxr);
		wxr = NULL;
	}
	if (wxr == NULL && mode->num_ranges != 0) {
		wxr = intf->init(mode, atmo);
		ASSERT(wxr != NULL);
	}
	if (wxr != NULL) {
		int range = 0;
		double tilt = 0, gain_ctl = 0.5;
		double gain = 0;
		bool_t power_on = B_TRUE, stby = B_FALSE;

		DELAYED_DR_OP(&sys.power_dr,
		    power_on = (dr_geti(&sys.power_dr.dr) != 0));

		if (power_on) {
			double now = dr_getf(&drs.sim_time);
			if (sys.power_on_time == 0)
				sys.power_on_time = now;
			stby = (now - sys.power_on_time < sys.power_on_delay);
		} else {
			stby = B_TRUE;
			sys.power_on_time = 0;
		}

		intf->set_standby(wxr, stby);
		intf->set_stab(wxr, aux->stab_lim.x, aux->stab_lim.y);

		if (sys.num_screens > 0)
			intf->set_brightness(wxr, sys.screens[0].brt / 0.75);

		intf->set_colors(wxr, aux->colors, aux->num_colors);

		DELAYED_DR_OP(&sys.range_dr,
		    range = dr_geti(&sys.range_dr.dr));
		range = clampi(range, 0, mode->num_ranges - 1);
		intf->set_scale(wxr, range);

		DELAYED_DR_OP(&sys.tilt_dr, tilt = dr_getf(&sys.tilt_dr.dr));
		intf->set_ant_pitch(wxr, tilt);

		DELAYED_DR_OP(&sys.gain_dr,
		    gain_ctl = dr_getf(&sys.gain_dr.dr));
		if (gain_ctl == sys.gain_auto_pos) {
			gain = DFL_GAIN;
		} else {
			gain = wavg(MIN_GAIN, MAX_GAIN, clamp(gain_ctl, 0, 1));
		}
		intf->set_gain(wxr, gain);
	}

	for (unsigned i = 0; wxr != NULL && i < sys.num_screens; i++) {
		bool_t power = B_TRUE;
		wxr_scr_t *scr = &sys.screens[i];
		double brt = 0.75;

		DELAYED_DR_OP(&scr->power_dr,
		    power = (dr_geti(&scr->power_dr.dr) != 0));
		if (power) {
			double rate = (scr->power_on_rate /
			    (1 + 50 * POW3(scr->scr_temp)));

			FILTER_IN(scr->scr_temp, 1, d_t, 10);
			FILTER_IN(scr->power, 1, d_t, rate);
		} else {
			FILTER_IN(scr->scr_temp, 0, d_t, 600);
			FILTER_IN(scr->power, 0, d_t, scr->power_off_rate);
		}

		DELAYED_DR_OP(&scr->brt_dr, brt = dr_getf(&scr->brt_dr.dr));
		if (brt > scr->brt)
			FILTER_IN(scr->brt, brt, d_t, 1);
		else
			FILTER_IN(scr->brt, brt, d_t, 0.2);
	}

	return (-1);
}

static int
draw_cb(XPLMDrawingPhase phase, int before, void *refcon)
{
	UNUSED(phase);
	UNUSED(before);
	UNUSED(refcon);

	if (dr_geti(&drs.panel_render_type) != PANEL_RENDER_TYPE_3D_LIT)
		return (1);

	for (unsigned i = 0; wxr != NULL && i < sys.num_screens; i++) {
		wxr_scr_t *scr = &sys.screens[i];
		double center_x = scr->x + scr->w / 2;

		intf->draw(wxr, VECT2(center_x - scr->h, scr->y),
		    VECT2(2 * scr->h, scr->h));
		mt_cairo_render_draw(scr->mtcr, VECT2(scr->x, scr->y),
		    VECT2(scr->w, scr->h));
	}

	return (1);
}

static void
draw_debug_win(XPLMWindowID win, void *refcon)
{
	int left, top, right, bottom, width, height;
	unsigned scr_id = (intptr_t)refcon;
	wxr_scr_t *scr;

	ASSERT3U(scr_id, <, sys.num_screens);
	scr = &sys.screens[scr_id];

	XPLMGetWindowGeometry(win, &left, &top, &right, &bottom);
	width = right - left;
	height = top - bottom;

	if (wxr != NULL)
		intf->draw(wxr, VECT2(left, bottom), VECT2(width, height));
	mt_cairo_render_draw(scr->mtcr, VECT2(left, bottom),
	    VECT2(width, height));
}

static void
render_cb(cairo_t *cr, unsigned w, unsigned h, void *userinfo)
{
	wxr_scr_t *scr = userinfo;
	double dashes[] = { 5, 5 };

	cairo_set_operator(cr, CAIRO_OPERATOR_CLEAR);
	cairo_paint(cr);
	cairo_set_operator(cr, CAIRO_OPERATOR_OVER);

	cairo_save(cr);

	cairo_scale(cr, w / (double)WXR_RES_X, h / (double)WXR_RES_Y);
	cairo_translate(cr, WXR_RES_X / 2, WXR_RES_Y);

	cairo_set_source_rgb(cr, CYAN_RGB(scr));
	cairo_set_line_width(cr, 1);
	for (int angle = -90; angle <= 90; angle += 30) {
		cairo_save(cr);
		cairo_rotate(cr, DEG2RAD(angle));
		cairo_move_to(cr, 0, 0);
		cairo_rel_line_to(cr, 0, -WXR_RES_Y);
		cairo_stroke(cr);
		cairo_restore(cr);
	}

	cairo_set_dash(cr, dashes, 2, 0);
	for (int i = 0; i < 4; i++) {
		cairo_arc(cr, 0, 0, (WXR_RES_Y / 4) * (i + 1), DEG2RAD(180),
		    DEG2RAD(360));
		cairo_stroke(cr);
	}
	cairo_set_dash(cr, NULL, 0, 0);

	cairo_restore(cr);

	if (scr->power < 0.99) {
		cairo_set_source_rgba(cr, 0, 0, 0, 1.0 - scr->power);
		cairo_paint(cr);
	}
}

static void
parse_conf_file(const conf_t *conf)
{
	const char *str;

	conf_get_i(conf, "efis/x", (int *)&sys.efis_xywh[0]);
	conf_get_i(conf, "efis/y", (int *)&sys.efis_xywh[1]);
	sys.efis_xywh[0] = clampi(sys.efis_xywh[0] + EFIS_OFF_X, 0, 2048);
	sys.efis_xywh[1] = clampi(sys.efis_xywh[1] + EFIS_OFF_Y, 0, 2048);
	sys.efis_xywh[2] = EFIS_WIDTH;
	sys.efis_xywh[3] = EFIS_HEIGHT;

	conf_get_i(conf, "num_modes", (int *)&sys.num_modes);
	sys.num_modes = clampi(sys.num_modes, 0, MAX_MODES);

	for (unsigned i = 0; i < sys.num_modes; i++) {
		wxr_conf_t *mode = &sys.modes[i];
		mode_aux_info_t *aux = &sys.aux[i];

		conf_get_i(conf, "res/x", (int *)&mode->res_x);
		conf_get_i(conf, "res/y", (int *)&mode->res_y);
		mode->res_x = clampi(mode->res_x, 64, 512);
		mode->res_y = clampi(mode->res_y, 64, 512);

		conf_get_d_v(conf, "mode/%d/beam_shape/x",
		    &mode->beam_shape.x, i);
		conf_get_d_v(conf, "mode/%d/beam_shape/y",
		    &mode->beam_shape.y, i);
		mode->beam_shape.x = clamp(mode->beam_shape.x, 1, 90);
		mode->beam_shape.y = clamp(mode->beam_shape.y, 1, 90);

		conf_get_d_v(conf, "mode/%d/scan_time",
		    &mode->scan_time, i);
		mode->scan_time = clamp(mode->scan_time, 0.1, 100);

		conf_get_d_v(conf, "mode/%d/scan_angle",
		    &mode->scan_angle, i);
		mode->scan_angle = clamp(mode->scan_angle, 1, 180);

		conf_get_d_v(conf, "mode/%d/smear/x", &mode->smear.x, i);
		conf_get_d_v(conf, "mode/%d/smear/y", &mode->smear.y, i);
		mode->smear.x = clamp(mode->smear.x, 0, 100);
		mode->smear.y = clamp(mode->smear.y, 0, 100);

		conf_get_d_v(conf, "mode/%d/parked_azi",
		    &mode->parked_azi, i);
		mode->parked_azi = clamp(mode->parked_azi,
		    -mode->scan_angle / 2, mode->scan_angle / 2);

		conf_get_i(conf, "num_ranges", (int *)&mode->num_ranges);
		mode->num_ranges = clampi(mode->num_ranges, 0, WXR_MAX_RANGES);
		for (unsigned j = 0; j < mode->num_ranges; j++)
			conf_get_d_v(conf, "range/%d", &mode->ranges[j], j);

		conf_get_d_v(conf, "mode/%d/stab_lim/x", &aux->stab_lim.x,
		    i);
		conf_get_d_v(conf, "mode/%d/stab_lim/y", &aux->stab_lim.y,
		    i);

		conf_get_i_v(conf, "mode/%d/num_colors",
		    (int *)&aux->num_colors, i);
		aux->num_colors = MIN(aux->num_colors, MAX_COLORS);
		for (unsigned j = 0; j < aux->num_colors; j++) {
			conf_get_d_v(conf, "mode/%d/colors/%d/thresh",
			    &aux->colors[j].min_val, i, j);
			if (conf_get_str_v(conf, "mode/%d/colors/%d/rgba",
			    &str, i, j)) {
				(void) sscanf(str, "%x", &aux->colors[j].rgba);
				aux->colors[j].rgba = BE32(aux->colors[j].rgba);
			}
			/* back up the master color palette */
			memcpy(aux->base_colors, aux->colors,
			    sizeof (aux->base_colors));
		}
	}

	if (conf_get_str(conf, "power_dr", &str))
		strlcpy(sys.power_dr.name, str, sizeof (sys.power_dr.name));
	conf_get_d(conf, "power_on_delay", &sys.power_on_delay);
		strlcpy(sys.power_dr.name, str, sizeof (sys.power_dr.name));
	if (conf_get_str(conf, "range_dr", &str))
		strlcpy(sys.range_dr.name, str, sizeof (sys.range_dr.name));
	if (conf_get_str(conf, "tilt_dr", &str))
		strlcpy(sys.tilt_dr.name, str, sizeof (sys.tilt_dr.name));
	if (conf_get_str(conf, "mode_dr", &str))
		strlcpy(sys.mode_dr.name, str, sizeof (sys.mode_dr.name));
	if (conf_get_str(conf, "gain_dr", &str))
		strlcpy(sys.gain_dr.name, str, sizeof (sys.gain_dr.name));
	conf_get_d(conf, "gain_auto_pos", &sys.gain_auto_pos);

	conf_get_i(conf, "num_screens", (int *)&sys.num_screens);
	sys.num_screens = clampi(sys.num_screens, 0, MAX_SCREENS);
	for (unsigned i = 0; i < sys.num_screens; i++) {
		wxr_scr_t *scr = &sys.screens[i];
		double fps = 15;
		const char *str;

		conf_get_d_v(conf, "scr/%d/x", &scr->x, i);
		conf_get_d_v(conf, "scr/%d/y", &scr->y, i);
		conf_get_d_v(conf, "scr/%d/w", &scr->w, i);
		conf_get_d_v(conf, "scr/%d/h", &scr->h, i);
		conf_get_d_v(conf, "scr/%d/fps", &fps, i);

		if (conf_get_str_v(conf, "scr/%d/brt_dr", &str, i)) {
			strlcpy(scr->brt_dr.name, str,
			    sizeof (scr->brt_dr.name));
		}

		if (conf_get_str_v(conf, "scr/%d/power_dr", &str, i)) {
			strlcpy(scr->power_dr.name, str,
			    sizeof (scr->power_dr.name));
		}
		conf_get_d_v(conf, "scr/%d/power_on_rate",
		    &scr->power_on_rate, i);
		conf_get_d_v(conf, "scr/%d/power_off_rate",
		    &scr->power_off_rate, i);
		scr->power_on_rate = MAX(scr->power_on_rate, 0.05);
		scr->power_off_rate = MAX(scr->power_off_rate, 0.05);

		scr->mtcr = mt_cairo_render_init(scr->w, scr->h, fps, NULL,
		    render_cb, NULL, scr);
		scr->sys = &sys;
	}
}

bool_t
sa_init(const conf_t *conf)
{
	XPLMPluginID opengpws;

	ASSERT(!inited);
	inited = B_TRUE;

	memset(&sys, 0, sizeof (sys));
	parse_conf_file(conf);

	open_debug_cmd = XPLMCreateCommand("openwxr/standalone_window",
	    "Open OpenWXR standalone mode debug window");
	ASSERT(open_debug_cmd != NULL);
	XPLMRegisterCommandHandler(open_debug_cmd, open_debug_win, 0, NULL);

	opengpws = XPLMFindPluginBySignature(OPENGPWS_PLUGIN_SIG);
	if (opengpws == XPLM_NO_PLUGIN_ID) {
		logMsg("WXR init failure: OpenGPWS plugin not found. "
		    "Is it installed?");
		goto errout;
	}
	XPLMSendMessageToPlugin(opengpws, EGPWS_GET_INTF, &sys.terr);

	openwxr = XPLMFindPluginBySignature(OPENWXR_PLUGIN_SIG);
	ASSERT(openwxr != XPLM_NO_PLUGIN_ID);

	XPLMSendMessageToPlugin(openwxr, OPENWXR_INTF_GET, &intf);
	ASSERT(intf != NULL);
	XPLMSendMessageToPlugin(openwxr, OPENWXR_ATMO_GET, &atmo);
	ASSERT(atmo != NULL);
	XPLMSendMessageToPlugin(openwxr, OPENWXR_ATMO_XP11_SET_EFIS,
	    sys.efis_xywh);

	fdr_find(&drs.sim_time, "sim/time/total_running_time_sec");
	fdr_find(&drs.panel_render_type, "sim/graphics/view/panel_render_type");

	XPLMRegisterFlightLoopCallback(floop_cb, -1, NULL);
	XPLMRegisterDrawCallback(draw_cb, xplm_Phase_Gauges, 0, NULL);

	return (B_TRUE);
errout:
	sa_fini();
	return (B_FALSE);
}

void
sa_fini(void)
{
	if (!inited)
		return;
	inited = B_FALSE;

	XPLMUnregisterCommandHandler(open_debug_cmd, open_debug_win, 0, NULL);

	for (unsigned i = 0; i < sys.num_screens; i++) {
		if (sys.screens[i].mtcr != NULL)
			mt_cairo_render_fini(sys.screens[i].mtcr);
	}

	if (wxr != NULL) {
		intf->fini(wxr);
		wxr = NULL;
	}
	intf = NULL;
	atmo = NULL;

	if (debug_win != NULL) {
		XPLMDestroyWindow(debug_win);
		debug_win = NULL;
	}

	XPLMUnregisterFlightLoopCallback(floop_cb, NULL);
	XPLMUnregisterDrawCallback(draw_cb, xplm_Phase_Gauges, 0, NULL);
}
