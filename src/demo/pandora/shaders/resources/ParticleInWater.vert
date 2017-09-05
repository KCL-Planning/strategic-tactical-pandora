#version 440

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
	// Update the velocity.
	tf_velocity = velocity + vec3(0, delta, 0);
	
	tf_position = velocity * delta + position;
	tf_lifetime = lifetime - delta;
	if (type != 0)
	{
		tf_type = type + delta * 4;
	}
	else
	{
		tf_type = type;
	}
}
