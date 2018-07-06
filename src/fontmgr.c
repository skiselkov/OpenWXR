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

#include <ft2build.h>
#include FT_FREETYPE_H

#include <acfutils/assert.h>
#include <acfutils/helpers.h>
#include <acfutils/mt_cairo_render.h>

#include "fontmgr.h"

typedef struct {
	const char		*file;
	FT_Face			ft_font;
	cairo_font_face_t	*cr_font;
} font_info_t;

static bool_t inited = B_FALSE;
static FT_Library ft = NULL;
static font_info_t fonts[NUM_FONTMGR_FONTS] = {
    { "Inconsolata/Inconsolata-Regular.ttf" },	/* FONTMGR_EFIS_FONT */
};

bool_t
fontmgr_init(const char *xpdir, const char *plugindir)
{
	char *fontdir = mkpathname(xpdir, plugindir, "fonts", NULL);
	FT_Error err;

	ASSERT(!inited);
	inited = B_TRUE;

	if ((err = FT_Init_FreeType(&ft)) != 0) {
		logMsg("Error initializing FreeType library: %s",
		    ft_err2str(err));
		goto errout;
	}

	for (int i = 0; i < NUM_FONTMGR_FONTS; i++) {
		font_info_t *f = &fonts[i];

		ASSERT(f->file != NULL);
		if (!try_load_font(fontdir, f->file, ft, &f->ft_font,
		    &f->cr_font))
			goto errout;
	}

	lacf_free(fontdir);

	return (B_TRUE);
errout:
	lacf_free(fontdir);
	fontmgr_fini();
	return (B_FALSE);
}

void
fontmgr_fini(void)
{
	if (!inited)
		return;
	inited = B_FALSE;

	for (int i = 0; i < NUM_FONTMGR_FONTS; i++) {
		if (fonts[i].cr_font != NULL) {
			cairo_font_face_destroy(fonts[i].cr_font);
			fonts[i].cr_font = NULL;
		}
		if (fonts[i].ft_font != NULL) {
			FT_Done_Face(fonts[i].ft_font);
			fonts[i].ft_font = NULL;
		}
	}
	if (ft != NULL) {
		FT_Done_FreeType(ft);
		ft = NULL;
	}
}

cairo_font_face_t *
fontmgr_get(fontmgr_font_t f)
{
	ASSERT(fonts[f].cr_font != NULL);
	return (fonts[f].cr_font);
}
