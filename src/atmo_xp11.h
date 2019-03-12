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

#ifndef	_ATMO_XP11_H_
#define	_ATMO_XP11_H_

#include "atmo.h"

#ifdef __cplusplus
extern "C" {
#endif

atmo_t *atmo_xp11_init(void);
void atmo_xp11_fini(void);

void atmo_xp11_set_efis_pos(unsigned x, unsigned y, unsigned w, unsigned h);

#ifdef __cplusplus
}
#endif

#endif	/* _ATMO_H_ */
