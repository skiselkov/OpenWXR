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

#include <errno.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include <GL/glew.h>

#include <XPLMPlugin.h>
#include <XPLMProcessing.h>
#include <XPLMUtilities.h>

#include <acfutils/assert.h>
#include <acfutils/core.h>
#include <acfutils/crc64.h>
#include <acfutils/dr.h>
#include <acfutils/helpers.h>
#include <acfutils/log.h>
#include <acfutils/time.h>
#include <acfutils/thread.h>

#include "dbg_log.h"

#define	PLUGIN_NAME		"OpenWXR by Saso Kiselkov"
#define	OPENWXR_PLUGIN_SIG	"skiselkov.openwxr"
#define	PLUGIN_DESCRIPTION \
	"An open-source generic weather radar simulation"

char			xpdir[512];
char			plugindir[512];

int			xp_ver, xplm_ver;
XPLMHostApplicationID	host_id;

PLUGIN_API int
XPluginStart(char *name, char *sig, char *desc)
{
	char *p;
	char *confpath;
	conf_t *conf = NULL;
	GLenum err;

	log_init(XPLMDebugString, "OpenWXR");
	crc64_init();
	crc64_srand(microclock());
	logMsg("This is OpenWXR (" PLUGIN_VERSION ") libacfutils-%s",
	    libacfutils_version);

	/* Always use Unix-native paths on the Mac! */
	XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);

	XPLMGetSystemPath(xpdir);
	XPLMGetPluginInfo(XPLMGetMyID(), NULL, plugindir, NULL, NULL);

#if	IBM
	fix_pathsep(xpdir);
	fix_pathsep(plugindir);
#endif	/* IBM */

	/* cut off the trailing path component (our filename) */
	if ((p = strrchr(plugindir, DIRSEP)) != NULL)
		*p = '\0';
	/*
	 * Cut off an optional '32' or '64' trailing component. Please note
	 * that XPLM 3.0 now supports OS-specific suffixes, so clamp those
	 * away as well.
	 */
	if ((p = strrchr(plugindir, DIRSEP)) != NULL) {
		if (strcmp(p + 1, "64") == 0 || strcmp(p + 1, "32") == 0 ||
		    strcmp(p + 1, "win_x64") == 0 ||
		    strcmp(p + 1, "mac_x64") == 0 ||
		    strcmp(p + 1, "lin_x64") == 0)
			*p = '\0';
	}

	/*
	 * Now we strip a leading xpdir from plugindir, so that now plugindir
	 * will be relative to X-Plane's root directory.
	 */
	if (strstr(plugindir, xpdir) == plugindir) {
		int xpdir_len = strlen(xpdir);
		int plugindir_len = strlen(plugindir);
		memmove(plugindir, &plugindir[xpdir_len],
		    plugindir_len - xpdir_len + 1);
	}

	strcpy(name, PLUGIN_NAME);
	strcpy(sig, OPENWXR_PLUGIN_SIG);
	strcpy(desc, PLUGIN_DESCRIPTION);

	XPLMGetVersions(&xp_ver, &xplm_ver, &host_id);

	err = glewInit();
	if (err != GLEW_OK) {
		/* Problem: glewInit failed, something is seriously wrong. */
		logMsg("FATAL ERROR: cannot initialize libGLEW: %s",
		    glewGetErrorString(err));
		return (0);
	}
	if (!GLEW_VERSION_2_1) {
		logMsg("FATAL ERROR: your system doesn't support OpenGL 2.1");
		return (0);
	}

	confpath = mkpathname(xpdir, plugindir, "OpenWXR.cfg", NULL);
	if (file_exists(confpath, NULL)) {
		int errline;

		conf = conf_read_file(confpath, &errline);
		if (conf == NULL) {
			if (errline < 0) {
				logMsg("Error reading configuration %s: cannot "
				    "open configuration file.", confpath);
			} else {
				logMsg("Error reading configuration %s: syntax "
				    "error on line %d.", confpath, errline);
			}
		}
	}
	if (conf == NULL)
		conf = conf_create_empty();
	dbg_log_init(conf);
	conf_free(conf);
	free(confpath);

	return (1);
}

PLUGIN_API void
XPluginStop(void)
{
}

PLUGIN_API int
XPluginEnable(void)
{
	return (1);
}

PLUGIN_API void
XPluginDisable(void)
{
}

PLUGIN_API void
XPluginReceiveMessage(XPLMPluginID from, int msg, void *param)
{
	UNUSED(from);
	UNUSED(msg);
	UNUSED(param);
}

const char *
get_xpdir(void)
{
	return (xpdir);
}

const char *get_plugindir(void)
{
	return (plugindir);
}
