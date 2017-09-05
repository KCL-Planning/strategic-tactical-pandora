#version 420

#include shaders/light_header.inc
#include shaders/light_header_frag.inc

uniform mat3 normal_matrix;

uniform sampler2D texture0;
uniform sampler1D texture1;

uniform sampler2DArray caustic_texture;
uniform int caustic_texture_index;
uniform sampler2DShadow caustic_depth_texture;

in vec4 sun_tex_coordinates;
in vec2 texCoord0;
in float blendFactor;
in float height;
in vec4 worldPos;

out vec4 outColor;

void main(void) {
	outColor = vec4(0.0, 0.0, 0.0, 1.0);
	
	#include shaders/light_body_frag.inc
	outColor *= mix( texture(texture0, texCoord0.st), texture(texture1, height), 0.5);


	float sun_shadow = 1;
	
	if (sun_tex_coordinates.x >=0 && sun_tex_coordinates.x < 1 &&
	    sun_tex_coordinates.y >=0 && sun_tex_coordinates.y < 1)
	{
		sun_shadow = textureProj(caustic_depth_texture, sun_tex_coordinates);
	}

	vec4 a = sun_shadow * texture(caustic_texture, vec3(worldPos.xz / 10, caustic_texture_index));
	//outColor += a * 0.3;
	outColor = mix(a, outColor, 0.7);

	// Lets add some fog.
	outColor = mix(vec4(0.2, 0.2, 0.4, 1.0), outColor, blendFactor);
}
