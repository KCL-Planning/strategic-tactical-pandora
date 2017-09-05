#version 420

#include shaders/light_header.inc
#include shaders/light_header_frag.inc

uniform mat3 normal_matrix;

uniform sampler2D texture0;
uniform sampler1D texture1;

in vec2 texCoord0;
in float blendFactor;
in float height;
in vec4 worldPos;

out vec4 outColor;

void main(void) {
	outColor = vec4(0.0, 0.0, 0.0, 1.0);
	
	#include shaders/light_body_frag.inc
	outColor *= mix( texture(texture0, texCoord0.st), texture(texture1, height), 0.5);

	// Lets add some fog.
	outColor = mix(vec4(0.2, 0.2, 0.4, 1.0), outColor, blendFactor);
}
