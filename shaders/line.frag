#version 420

uniform vec4 colour;

//in vec3 colour;
out vec4 outColor;

void main()
{
	outColor = colour;
}
