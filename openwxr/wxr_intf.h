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

#ifndef	_OPENWXR_WXR_INTF_H_
#define	_OPENWXR_WXR_INTF_H_

#include <acfutils/geom.h>

#ifdef __cplusplus
extern "C" {
#endif

#define	WXR_MIN_RES	32
#define	WXR_MAX_RANGES	32

typedef enum {
	WXR_DISP_ARC,
	WXR_DISP_SQUARE
} wxr_disp_t;

/*
 * The main WXR configuration structure. This must be passed by te aircraft
 * to set up a WXR instance of OpenWXR.
 */
typedef struct {
	/* Number of range scales contained in `ranges' */
	size_t		num_ranges;
	/*
	 * The range scales at which the screen of the WXR can draw. This
	 * determines the sampling interval of the underlying weather so
	 * as to avoid excessive sample density and the associated
	 * performance cost.
	 */
	double		ranges[WXR_MAX_RANGES];
	/*
	 * WXR weather sampling resolution.
	 *
	 * res_x - the number of radial samples sent out by the antenna as it
	 *	scans left-to-right. This is NOT screen resolution, although
	 *	in general you will probably want your radial resolution to
	 *	be close to the actual screen resolution, so that the WXR
	 *	samples don't appear too blobby and large on the screen.
	 * res_y - the number of samples along a radar scan line. This should
	 *	be close to the screen resolution to avoid "blobby" looks, but
	 *	is again not its physical meaning. This means at what
	 *	resolution the WXR talks to the atmosphere to determine
	 *	reflected energy along a scan line.
	 */
	unsigned	res_x;
	unsigned	res_y;
	/*
	 * The beam cone shape:
	 * X - side-to-side angle of the radar beam
	 * Y - up-down angle of the radar beam
	 * These do NOT represent the radar's scan limit (i.e. how far the
	 * antenna can swing). They represent the spacial size of one radar
	 * pulse (once sent out).
	 */
	vect2_t		beam_shape;
	
	wxr_disp_t	disp_type;	/* The of radar display to draw. */
	double		scan_time;	/* Secs for one full swing side2side */
	double		scan_angle;	/* Degrees btwn full lateral deflect */
	double		scan_angle_vert;/* Degrees btwn full vertical deflect */
} wxr_conf_t;

#ifdef __cplusplus
}
#endif

#endif	/* _OPENWXR_WXR_INTF_H_ */
