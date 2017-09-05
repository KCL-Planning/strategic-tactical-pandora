#version 420

uniform sampler2D texture0;

in vec3 gvelocity;
in float glifetime;
in vec2 texCoord0;
in float blendFactor;

out vec4 outColor;

const float max_lifetime = 12;

void main(void)
{
	if (glifetime < 0 || texture(texture0, texCoord0.st).a < 0.5)
	{
		discard;
	}
	outColor = vec4(glifetime / max_lifetime, glifetime / max_lifetime, glifetime / max_lifetime, 1.0);
	outColor *= texture(texture0, texCoord0.st);
	outColor = mix(vec4(0.2, 0.2, 0.4, 1.0), outColor, blendFactor);
}
