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

#version 120

uniform sampler2D	tex;
uniform vec2		tex_sz;
uniform float		smooth;

void
main(void)
{
	vec2 pos = gl_FragCoord.xy / tex_sz;
	vec4 intens;

	for (int i = -2; i <= 2; i += 1) {
		for (int j = -2; j <= 2; j += 1) {
			intens += texture2D(tex, vec2(pos.x + i * smooth,
			    pos.y + j * smooth));
		}
	}

	gl_FragColor = intens / 25.0;
}
