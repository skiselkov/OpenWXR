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

#version 460

layout(location = 10) uniform sampler2D	tex;
layout(location = 11) uniform vec2	tex_sz;

layout(location = 0) out vec4		color_out;

float
compute_intens(vec2 pos)
{
	vec4 color = texture(tex, pos);

	if (color.b > 0.1) {
		if (color.a < 0.95)
			return (0.0);
		else if (color.r > 0.9 && color.g > 0.9)
			return (0.75);
		else if (color.r > 0.9)
			return (1.0);
		else if (color.g > 0.9)
			return (0.25);
		else if (color.g > 0.7)
			return (0.5);
		else
			return (0.0);
	} else {
		if (color.a < 0.95)
			return (0.0);
		else if (color.r > 0.9 && color.g > 0.9)
			return (0.75);
		else if (color.r > 0.9)
			return (1.0);
		else if (color.g > 0.8)
			return (0.25);
		else if (color.g > 0.4)
			return (0.5);
		else
			return (0.0);
	}
}

void
main(void)
{
	vec2 pos = gl_FragCoord.xy / tex_sz;
	color_out = vec4(compute_intens(pos), 0, 0, 1);
}
