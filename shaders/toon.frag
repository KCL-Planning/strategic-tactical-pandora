#version 420

#include shaders/light_header.inc
#include shaders/light_header_frag.inc

in vec2 texCoord0;
in float blendFactor;
in vec4 worldPos;

out vec4 outColor;

void main(void) {
	
	outColor = vec4(0.0, 0.0, 0.0, 1.0);
	
	#include shaders/light_body_frag.inc
	if (texCoord0.s < 0.05 || texCoord0.x > 0.95 || texCoord0.t < 0.05 || texCoord0.t > 0.95)
	{
		outColor = vec4(0, 0, 0, 1);
	}
	else
	{
		outColor *= material_emissive;
	}
	
	// Lets add some fog.
	//outColor = mix(vec4(0.1, 0.1, 0.2, 0.1), outColor, blendFactor);
	outColor = mix(vec4(0.2, 0.2, 0.4, 1.0), outColor, blendFactor);
}
