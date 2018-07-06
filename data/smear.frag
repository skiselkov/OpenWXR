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
#extension GL_EXT_gpu_shader4: enable

uniform	sampler2D	tex;
uniform	float		smear_mult;
uniform float		brt;

varying	vec2		tex_coord;

float
color(float c)
{
	return (c * (pow(4.0, brt) / 4.0));
}

void
main()
{
	vec4 pixel = texture2D(tex, tex_coord);
	vec2 tex_size = textureSize2D(tex, 0);
	vec4 out_color;

	if (pixel.r == pixel.g && pixel.r == pixel.b) {
		out_color = pixel;
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
		vec4 sample;

		/*
		 * Yes the smear components are reversed here, the 't'
		 * component is smeared using smear_s and vice versa.
		 * Gives us the look we jaggedy-edged look we want.
		 */
		out_color = texture2D(tex, vec2(
		    clamp(tex_coord.s + smear_t, 0.0, 1.0),
		    clamp(tex_coord.t + smear_s, 0.0, 1.0)));
	}
	gl_FragColor = vec4(color(out_color.r), color(out_color.g),
	    color(out_color.b), out_color.a);
}
