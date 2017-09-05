#version 420

#include shaders/light_header.inc
#include shaders/light_header_frag.inc

uniform sampler2D texture0;
uniform float transparency;

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

	outColor *= texture(texture0, texCoord0.st);
	outColor = mix(vec4(0.2, 0.2, 0.4, 1.0), outColor, blendFactor);
}
