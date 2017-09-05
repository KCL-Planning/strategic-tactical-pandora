#version 420 core

layout (location = 0) out vec4 colour;

uniform sampler2D texture0;

in vec2 texCoord0;

void main(void)
{
/*
	if (texture(texture0, texCoord0.st).a == 0.0)
	{
		discard;
	}
*/
	colour = vec4(0.0, 0.0, 0.0, 1.0);
}
