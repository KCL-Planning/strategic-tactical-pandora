#version 420

uniform sampler2D texture0;

uniform mat4 modelview_matrix;
uniform mat4 projection_matrix;

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 velocity;
layout (location = 2) in float lifetime;
layout (location = 3) in float type;

out vec3 vposition;
out vec3 vvelocity;
out float vlifetime;
out float vtype;

void main(void)
{
	vposition = position;
	vvelocity = velocity;
	vlifetime = lifetime;
	vtype = type;

	gl_Position = vec4(position, 1.0);
}
