#version 430

uniform float delta;

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 velocity;
layout (location = 2) in float lifetime;
layout (location = 3) in float type;

out vec3 tf_position;
out vec3 tf_velocity;
out float tf_lifetime;
out float tf_type;

void main(void)
{
	tf_position = velocity * delta + position;
	tf_velocity = velocity;
	tf_lifetime = lifetime - delta;
	tf_type = type;
}
