#version 420

uniform sampler2D texture0;

#include shaders/light_header.inc
#include shaders/light_header_frag.inc

in vec2 texCoord0;
in float blendFactor;
in vec4 worldPos;

out vec4 outColor;

void main(void) {
	
	if (texture(texture0, texCoord0.st).a == 0.0)
	{
		discard;
	}
	
	outColor = vec4(0.0, 0.0, 0.0, 1.0);
	#include shaders/light_body_frag.inc
	outColor *= texture(texture0, texCoord0.st);
	
	// Lets add some fog.
	outColor.a = 0.4;// = mix(vec4(0.1, 0.1, 0.2, 0.1), outColor, blendFactor);
}
