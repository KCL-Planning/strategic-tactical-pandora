#version 420

#include shaders/light_header.inc
#include shaders/light_header_vert.inc

uniform mat4 projection_matrix;
uniform mat4 modelview_matrix;
uniform mat4 model_matrix;
uniform mat4 view_matrix;

in vec3 a_Vertex;
in vec2 a_TexCoord0;
in vec3 a_Normal;

out vec2 texCoord0;
out float blendFactor;
out float height;
out vec4 worldPos;

void main(void) 
{
	texCoord0 = a_TexCoord0;
	vec4 pos = modelview_matrix * vec4(a_Vertex, 1.0);
	worldPos = model_matrix * vec4(a_Vertex, 1.0);
	
	#include shaders/light_body_vert.inc
	height = (a_Vertex[1] + 2.0) / 4.0;

	//blendFactor = exp2(-.01 * length(pos));
	blendFactor = 1 - length(pos) / 300;
	gl_Position = projection_matrix * pos;
}
