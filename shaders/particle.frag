#version 420

uniform sampler2D texture0;

in vec3 gvelocity;
in float glifetime;
in float gsize;
in vec2 texCoord0;

out vec4 outColor;

void main(void)
{
	if (glifetime < 0 || texture(texture0, texCoord0.st).a < 0.1)
	{
		discard;
	}
	outColor = vec4(0.0, 0.5 * glifetime, 1.0 * glifetime, 1.0);
	//outColor = vec4(1.0 * glifetime, 1.0 * glifetime, 1.0 * glifetime, 1.0);
	outColor *= texture(texture0, texCoord0.st);
}
