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
 * Copyright 2017 Saso Kiselkov. All rights reserved.
 */

#include <string.h>
#include <stdio.h>

#include <acfutils/safe_alloc.h>

#include "dbg_log.h"

dbg_level_t dbg_level = { .all = -1 };

void
dbg_log_init(const conf_t *conf)
{
	memset(&dbg_level, 0, sizeof (dbg_level));

	conf_get_i(conf, "debug_all", &dbg_level.all);
}

void
dbg_log_fini(void)
{
}

void
dbg_log_impl(const char *dbg_class, int level, const char *fmt, ...)
{
	va_list ap;
	int len;
	char *msg;

	va_start(ap, fmt);
	len = vsnprintf(NULL, 0, fmt, ap);
	va_end(ap);
	msg = safe_malloc(len + 1);
	va_start(ap, fmt);
	vsnprintf(msg, len + 1, fmt, ap);
	va_end(ap);
	logMsg("[%s/%d]: %s", dbg_class, level, msg);
	free(msg);
}
