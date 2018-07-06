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

#ifndef	_FONTMGR_H_
#define	_FONTMGR_H_

#include <acfutils/types.h>
#include <cairo.h>
#include <cairo-ft.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	FONTMGR_EFIS_FONT,
	NUM_FONTMGR_FONTS
} fontmgr_font_t;

bool_t fontmgr_init(const char *xpdir, const char *plugindir);
void fontmgr_fini(void);

cairo_font_face_t *fontmgr_get(fontmgr_font_t f);

#ifdef __cplusplus
}
#endif

#endif	/* _FONTMGR_H_ */
