#version 420

uniform mat4 modelview_matrix;
uniform mat4 projection_matrix;

attribute vec2 a_Vertex;
attribute vec2 a_TexCoord0;

out vec2 texCoord0;
 
void main(void) {
	gl_Position = projection_matrix * modelview_matrix * vec4(a_Vertex, 0.0, 1.0);
	texCoord0 = a_TexCoord0;
}
