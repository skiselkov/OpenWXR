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
 * Copyright 2022 Saso Kiselkov. All rights reserved.
 */

#include <errno.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include <XPLMPlugin.h>
#include <XPLMProcessing.h>
#include <XPLMUtilities.h>

#include <acfutils/assert.h>
#include <acfutils/core.h>
#include <acfutils/crc64.h>
#include <acfutils/dr.h>
#include <acfutils/glew.h>
#include <acfutils/helpers.h>
#include <acfutils/log.h>
#include <acfutils/time.h>
#include <acfutils/thread.h>

#include "atmo_xp11.h"
#include "dbg_log.h"
#include "fontmgr.h"
#include <openwxr/xplane_api.h>
#include "standalone.h"
#include "wxr.h"
#include "xplane.h"

#define	PLUGIN_NAME		"OpenWXR by Saso Kiselkov"
#define	PLUGIN_DESCRIPTION \
	"An open-source generic weather radar simulation"

char			xpdir[512];
char			plugindir[512];

static int		xp_ver, xplm_ver;
XPLMHostApplicationID	host_id;
static atmo_t		*atmo = NULL;

static openwxr_intf_t openwxr_intf = {
	.init = wxr_init,
	.fini = wxr_fini,
	.set_acf_pos = wxr_set_acf_pos,
	.set_scale = wxr_set_scale,
	.get_scale = wxr_get_scale,
	.set_azimuth_limits = wxr_set_azimuth_limits,
	.get_ant_azimuth = wxr_get_ant_azimuth,
	.set_ant_pitch = wxr_set_ant_pitch,
	.get_ant_pitch = wxr_get_ant_pitch,
	.set_gain = wxr_set_gain,
	.get_gain = wxr_get_gain,
	.set_stab = wxr_set_stab,
	.get_stab = wxr_get_stab,
	.set_beam_shadow = wxr_set_beam_shadow,
	.get_beam_shadow = wxr_get_beam_shadow,
	.set_standby = wxr_set_standby,
	.get_standby = wxr_get_standby,
	.draw = wxr_draw,
	.clear_screen = wxr_clear_screen,
	.set_vert_mode = wxr_set_vert_mode,
	.get_vert_mode = wxr_get_vert_mode,
	.set_colors = wxr_set_colors,
	.get_brightness = wxr_get_brightness,
	.set_brightness = wxr_set_brightness,
	.reload_gl_progs = wxr_reload_gl_progs
};

static conf_t *
load_config_file(void)
{
	char *confpath = mkpathname(xpdir, plugindir, "OpenWXR.cfg", NULL);
	conf_t *conf = NULL;

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
	lacf_free(confpath);

	return (conf);
}

PLUGIN_API int
XPluginStart(char *name, char *sig, char *desc)
{
	char *p;
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

	conf = load_config_file();
	if (conf == NULL)
		conf = conf_create_empty();
	dbg_log_init(conf);
	conf_free(conf);

	/*
	 * Must go ahead of XPluginEnable to always have an atmosphere
	 * ready for when external avionics start creating wxr_t instances.
	 */
	atmo = atmo_xp11_init();
	if (atmo == NULL)
		return (0);

	return (1);
}

PLUGIN_API void
XPluginStop(void)
{
	/*
	 * Must wait with shutdown until all instances of wxr_t have
	 * been shut down by external avionics, so we can't do this
	 * in XPluginDisable.
	 */
	atmo_xp11_fini();
}

PLUGIN_API int
XPluginEnable(void)
{
	conf_t *conf = load_config_file();
	bool_t standalone = B_FALSE;

	if (conf == NULL)
		return (1);

	conf_get_b(conf, "standalone", &standalone);
	if (standalone) {
		if (!fontmgr_init(xpdir, plugindir))
			goto errout;
		if (!sa_init(conf))
			goto errout;
	}

	conf_free(conf);
	return (1);
errout:
	conf_free(conf);
	XPluginDisable();
	return (0);
}

PLUGIN_API void
XPluginDisable(void)
{
	sa_fini();
	fontmgr_fini();
}

PLUGIN_API void
XPluginReceiveMessage(XPLMPluginID from, int msg, void *param)
{
	UNUSED(from);

	switch (msg) {
	case OPENWXR_INTF_GET:
		ASSERT(param != NULL);
		*(openwxr_intf_t **)param = &openwxr_intf;
		break;
	case OPENWXR_ATMO_GET:
		ASSERT(param != NULL);
		ASSERT(atmo != NULL);
		*(atmo_t **)param = atmo;
		break;
	case OPENWXR_ATMO_XP11_SET_EFIS: {
		unsigned *coords = param;
		atmo_xp11_set_efis_pos(coords[0], coords[1],
		    coords[2], coords[3]);
		break;
	}
	}
}

const char *
get_xpdir(void)
{
	return (xpdir);
}

const char *
get_plugindir(void)
{
	return (plugindir);
}

int
get_xpver(void)
{
	return (xp_ver);
}

#if	IBM
BOOL WINAPI
DllMain(HINSTANCE hinst, DWORD reason, LPVOID resvd)
{
	UNUSED(hinst);
	UNUSED(resvd);
	lacf_glew_dllmain_hook(reason);
	return (TRUE);
}
#endif	/* IBM */
