#version 420

#include shaders/light_header.inc
#include shaders/light_header_frag.inc

uniform sampler2D texture0;
uniform sampler2DArray caustic_texture;
uniform int caustic_texture_index;
uniform sampler2DShadow caustic_depth_texture;

uniform float transparency;

in vec4 sun_tex_coordinates;
in vec2 texCoord0;
in float blendFactor;
in vec4 worldPos;

out vec4 outColor;

void main(void) {
	
	if (texture(texture0, texCoord0.st).a == 0.0 || transparency == 1.0)
	{
		discard;
	}

	outColor = vec4(0, 0, 0, 1);
	
	#include shaders/light_body_frag.inc

	// Handle the caustic values.
	float sun_shadow = 1;
	
	if (sun_tex_coordinates.x >=0 && sun_tex_coordinates.x < 1 &&
	    sun_tex_coordinates.y >=0 && sun_tex_coordinates.y < 1)
	{
		if (dot(normalVector, vec4(0, -1, 0, 0)) > 0)
			sun_shadow = 0;
		else
			sun_shadow = textureProj(caustic_depth_texture, sun_tex_coordinates);
	}
	vec4 a = sun_shadow * texture(caustic_texture, vec3(worldPos.xz / 10, caustic_texture_index));
	outColor = mix(a, outColor, 0.7);

	outColor *= texture(texture0, texCoord0.st);
	outColor = mix(vec4(0.2, 0.2, 0.4, 1.0), outColor, blendFactor);
}
