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

#ifndef	_ATMO_H_
#define	_ATMO_H_

#include <acfutils/geom.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	geo_pos3_t	origin;		/* beam origin point */
	vect2_t		dir;		/* X-hdg degrees, Y-pitch degrees up */
	vect2_t		shape;		/* X-horiz degrees, Y-vert degrees */
	double		energy;		/* beam energy (no units), log scale */
	double		range;		/* scan line sampling range */
	int		num_samples;	/* number of samples to return */
	double		*energy_out;	/* energy return samples, log scale */
	double		*doppler_out;	/* freq shift, relative motion, m/s */
} scan_line_t;

typedef struct atmo_s {
	void		(*set_range)(double rng);
	void		(*probe)(scan_line_t *sl);
} atmo_t;

#ifdef __cplusplus
}
#endif

#endif	/* _ATMO_H_ */
