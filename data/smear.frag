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

#ifdef GL_EXT_gpu_shader4
#extension GL_EXT_gpu_shader4: enable
#endif

layout(location = 10) uniform sampler2D	tex;
layout(location = 11) uniform float	smear_mult;
layout(location = 12) uniform float	brt;

layout(location = 0) in vec2		tex_coord;

layout(location = 0) out vec4		color_out;

vec3
brt_adjust(vec3 c)
{
	float f = pow(4.0, brt) / 4.0;
	return (c * vec3(f));
}

void
main()
{
	vec4 pixel = texture(tex, tex_coord);
	vec2 tex_size = textureSize(tex, 0);

	if (pixel.r == pixel.g && pixel.r == pixel.b) {
		color_out = pixel;
	} else {
		float s1 = sin(tex_coord.s * 16.1803);
		float s2 = sin(tex_coord.s * 95.828);
		float s3 = sin(tex_coord.s * 181.959);
		float s4 = sin(tex_coord.s * 314.159);
		float s5 = sin(tex_coord.s * 547.363);
		float smear_s = (2.5 * smear_mult * s1 * s2 * s3 * s4 * s5) /
		    tex_size.x;

		float t1 = sin(tex_coord.t * 16.1803);
		float t2 = sin(tex_coord.t * 95.828);
		float smear_t = (smear_mult * t1) / tex_size.y;

		/*
		 * Yes the smear components are reversed here, the 't'
		 * component is smeared using smear_s and vice versa.
		 * Gives us the jaggedy-edged look we want.
		 */
		color_out = texture(tex, vec2(tex_coord.s,
		    clamp(tex_coord.t + smear_s, 0.0, 1.0)));
	}

	color_out = vec4(brt_adjust(color_out.rgb), color_out.a);
}
