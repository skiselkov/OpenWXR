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

uniform sampler2D tex;
uniform vec2 tex_size;

void
main()
{
	vec4 pixel = texture2D(tex, gl_TexCoord[0].st);

	if (pixel.r == pixel.g && pixel.r == pixel.b) {
		gl_FragColor = pixel;
	} else {
		float s1 = sin(gl_TexCoord[0].s * 16.1803);
		float s2 = sin(gl_TexCoord[0].s * 95.828);
		float s3 = sin(gl_TexCoord[0].s * 181.959);
		float s4 = sin(gl_TexCoord[0].s * 314.159);
		float s5 = sin(gl_TexCoord[0].s * 547.363);
		float smear_s = (10.0 * s1 * s2 * s3 * s4 * s5) / tex_size.x;

		float t1 = sin(gl_TexCoord[0].t * 16.1803);
		float t2 = sin(gl_TexCoord[0].t * 95.828);
		float smear_t = (4.0 * t1) / tex_size.y;
		vec4 sample;

		/*
		 * Yes the smear components are reversed here, the 't'
		 * component is smeared using smear_s and vice versa.
		 * Gives us the look we jaggedy-edged look we want.
		 */
		gl_FragColor = texture2D(tex, vec2(
		    clamp(gl_TexCoord[0].s + smear_t, 0.0, 1.0),
		    clamp(gl_TexCoord[0].t + smear_s, 0.0, 1.0)));
	}
}
