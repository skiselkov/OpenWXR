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
 * Copyright 2019 Saso Kiselkov. All rights reserved.
 */

#ifndef	_GLPRIV_H_
#define	_GLPRIV_H_

#include <acfutils/glew.h>
#include <acfutils/shader.h>

#include "xplane.h"

#ifdef	__cplusplus
extern "C" {
#endif

static bool_t
reload_gl_prog(GLint *prog, const shader_prog_info_t *info)
{
	GLint new_prog;
	char *path = mkpathname(get_xpdir(), get_plugindir(), "data", "bin",
	    NULL);

	ASSERT(path != NULL);
	new_prog = shader_prog_from_info(path, info);
	lacf_free(path);
	if (new_prog == 0)
		return (B_FALSE);
	if (*prog != 0 && new_prog != *prog)
		glDeleteProgram(*prog);
	*prog = new_prog;

	return (B_TRUE);
}

#ifdef	__cplusplus
}
#endif

#endif	/* _GLPRIV_H_ */
