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

#ifndef	_OPENWXR_XPLANE_H_
#define	_OPENWXR_XPLANE_H_

#include <XPLMDefs.h>
#include <XPLMUtilities.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * X-Plane-specific plugin hooks.
 */
PLUGIN_API int XPluginStart(char *name, char *sig, char *desc);
PLUGIN_API void XPluginStop(void);
PLUGIN_API int XPluginEnable(void);
PLUGIN_API void XPluginDisable(void);
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID from, int msg, void *param);

const char *get_xpdir(void);
const char *get_plugindir(void);

int get_xpver(void);

#ifdef __cplusplus
}
#endif

#endif	/* _OPENWXR_XPLANE_H_ */
