#version 420 core

uniform mat4 modelviewprojection_matrix;

layout (location = 0) in vec4 a_Vertex;
layout (location = 1) in vec2 a_TexCoord0;

out vec2 texCoord0;

void main(void)
{
	texCoord0 = a_TexCoord0;
	gl_Position = modelviewprojection_matrix * a_Vertex;
}
