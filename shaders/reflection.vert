#version 420

uniform mat4 projection_matrix;
uniform mat4 modelview_matrix;
uniform mat4 model_matrix;
uniform vec3 camPosition;

in vec3 a_Vertex;
in vec3 a_Normal;

out vec3 modelPos;
out vec3 modelNormal;
out vec3 nor;

void main(void) 
{
	vec4 pos = modelview_matrix * vec4(a_Vertex, 1.0);

	modelPos = vec3(model_matrix * vec4(a_Vertex, 1.0));
	modelNormal = mat3x3(model_matrix) * a_Normal;

	gl_Position = projection_matrix * pos;
}
