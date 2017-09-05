#version 420

#include shaders/light_header.inc
#include shaders/light_header_vert.inc

uniform mat4 projection_matrix;
uniform mat4 view_matrix;

layout (location = 0) in vec3 a_Vertex;
layout (location = 1) in vec2 a_TexCoord0;
layout (location = 2) in vec3 a_Normal;
layout (location = 3) in mat4 model_matrix;

out vec2 texCoord0;
out float blendFactor;
out float height;
out vec4 worldPos;

void main(void) 
{
	texCoord0 = a_TexCoord0;
	vec4 pos = view_matrix * model_matrix * vec4(a_Vertex, 1.0);
	worldPos = model_matrix * vec4(a_Vertex, 1.0);
	
	mat4 modelview_matrix = view_matrix * model_matrix;
	
	#include shaders/light_body_vert.inc
	
	height = a_Vertex[1] / 10.0;

	blendFactor = exp2(-.1 * length(pos));
	gl_Position = projection_matrix * pos;
}
