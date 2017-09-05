#version 420

uniform mat4 projection_matrix;
uniform mat4 modelview_matrix;

in vec3 a_Vertex;

void main(void)
{
	gl_Position = projection_matrix * modelview_matrix * vec4(a_Vertex, 1.0);
}
