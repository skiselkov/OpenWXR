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

#ifndef	_DBG_LOG_H_
#define	_DBG_LOG_H_

#include <acfutils/assert.h>
#include <acfutils/conf.h>
#include <acfutils/helpers.h>
#include <acfutils/log.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	int	all;
} dbg_level_t;

extern dbg_level_t dbg_level;

#define	dbg_log(dbg_class, level, ...) \
	do { \
		ASSERT(dbg_level.all != -1); \
		if (dbg_level.dbg_class >= level || dbg_level.all >= level) \
			dbg_log_impl(#dbg_class, level, __VA_ARGS__); \
	} while (0)

void dbg_log_init(const conf_t *conf);
void dbg_log_fini(void);
void dbg_log_impl(const char *dbg_class, int level, const char *fmt, ...)
    PRINTF_ATTR(3);

#ifdef __cplusplus
}
#endif

#endif	/* _DBG_LOG_H_ */
