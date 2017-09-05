#version 420

uniform mat4 modelview_matrix;
uniform mat4 projection_matrix;

attribute vec2 a_Vertex;
attribute vec2 a_TexCoord0;
uniform sampler2D fbo_texture;
uniform vec4 light_position;

out vec2 light_position_2d;
out vec2 texCoord0;
 
void main(void) {
	gl_Position = projection_matrix * modelview_matrix * vec4(a_Vertex, 0.0, 1.0);
	texCoord0 = a_TexCoord0;
//	light_position_2d = projection_matrix_light * modelview_matrix_light * light_position;// * 0.5 + 0.5;
}
