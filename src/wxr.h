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

#ifndef	_WXR_H_
#define	_WXR_H_

#include <acfutils/geom.h>

#include "atmo.h"
#include <openwxr/wxr_intf.h>
#include <openwxr/xplane_api.h>

#ifdef __cplusplus
extern "C" {
#endif

wxr_t *wxr_init(const wxr_conf_t *conf, const atmo_t *atmo);
void wxr_fini(wxr_t *wxr);

void wxr_set_acf_pos(wxr_t *wxr, geo_pos3_t pos, vect3_t orient);
void wxr_draw(wxr_t *wxr, vect2_t pos, vect2_t size);

void wxr_set_scale(wxr_t *wxr, unsigned range_idx);
unsigned wxr_get_scale(const wxr_t *wxr);

void wxr_set_azimuth_limits(wxr_t *wxr, double left, double right);
double wxr_get_ant_azimuth(const wxr_t *wxr);

void wxr_set_ant_pitch(wxr_t *wxr, double angle);
double wxr_get_ant_pitch(const wxr_t *wxr);

void wxr_set_gain(wxr_t *wxr, double gain);
double wxr_get_gain(const wxr_t *wxr);

void wxr_set_stab(wxr_t *wxr, double pitch, double roll);
void wxr_get_stab(const wxr_t *wxr, bool_t *pitch, bool_t *roll);

void wxr_set_beam_shadow(wxr_t *wxr, bool_t flag);
bool_t wxr_get_beam_shadow(const wxr_t *wxr);

void wxr_set_standby(wxr_t *wxr, bool_t flag);
bool_t wxr_get_standby(const wxr_t *wxr);

void wxr_set_vert_mode(wxr_t *wxr, bool_t flag, double azimuth);
bool_t wxr_get_vert_mode(const wxr_t *wxr);

void wxr_set_gnd_sense(wxr_t *wxr, bool_t flag);
bool_t wxr_get_gnd_sense(const wxr_t *wxr);

void wxr_clear_screen(wxr_t *wxr);

void wxr_set_colors(wxr_t *wxr, const wxr_color_t *colors, size_t num);

#ifdef __cplusplus
}
#endif

#endif	/* _WXR_H_ */
