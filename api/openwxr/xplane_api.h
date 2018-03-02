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

#ifndef	_OPENWXR_XPLANE_API_H_
#define	_OPENWXR_XPLANE_API_H_

#include "wxr_intf.h"

#ifdef __cplusplus
extern "C" {
#endif

#define	OPENWXR_PLUGIN_SIG	"skiselkov.openwxr"

typedef struct atmo_s atmo_t;
typedef struct wxr_s wxr_t;

typedef struct {
	double		min_val;
	double		max_val;
	uint32_t	rgba;		/* Big-endian RGBA */
} wxr_color_t;

typedef struct {
	wxr_t *(*init)(const wxr_conf_t *conf, const atmo_t *atmo);
	void (*fini)(wxr_t *wxr);

	void (*set_acf_pos)(wxr_t *wxr, geo_pos3_t pos, vect3_t orient);
	void (*set_scale)(wxr_t *wxr, unsigned range_idx);
	unsigned (*get_scale)(const wxr_t *wxr);
	void (*set_azimuth_limits)(wxr_t *wxr, double left, double right);
	double (*get_ant_azimuth)(const wxr_t *wxr);
	void (*set_ant_pitch)(wxr_t *wxr, double angle);
	double (*get_ant_pitch)(const wxr_t *wxr);
	void (*set_gain)(wxr_t *wxr, double gain);
	double (*get_gain)(const wxr_t *wxr);
	void (*set_stab)(wxr_t *wxr, double angle, double roll);
	void (*get_stab)(const wxr_t *wxr, bool_t *pitch, bool_t *roll);
	void (*set_beam_shadow)(wxr_t *wxr, bool_t flag);
	bool_t (*get_beam_shadow)(const wxr_t *wxr);
	void (*set_standby)(wxr_t *wxr, bool_t flag);
	bool_t (*get_standby)(const wxr_t *wxr);
	void (*draw)(wxr_t *wxr, vect2_t pos, vect2_t size);
	void (*clear_screen)(wxr_t *wxr);
	void (*set_vert_mode)(wxr_t *wxr, bool_t flag, double azimuth);
	bool_t (*get_vert_mode)(const wxr_t *wxr);
	void (*set_colors)(wxr_t *wxr, const wxr_color_t *colors, size_t num);
} openwxr_intf_t;

typedef enum {
	OPENWXR_ATMO_XP11_SET_EFIS = 0x20000,	/* int coords[4] arg */
	OPENWXR_INTF_GET,			/* openwxr_intf_t ** arg */
	OPENWXR_ATMO_GET			/* atmo_t ** arg */
} openwxr_msg_t;

#ifdef __cplusplus
}
#endif

#endif	/* _OPENWXR_XPLANE_API_H_ */
