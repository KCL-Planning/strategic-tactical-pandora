#version 420

uniform float delta;

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 velocity;
layout (location = 2) in float lifetime;
layout (location = 3) in float size;

out vec3 tf_position;
out vec3 tf_velocity;
out float tf_lifetime;
out float tf_size;

void main(void)
{
	//if (lifetime > 0)
	{
		tf_position = velocity * delta + position;
		if (velocity.y < 0.25)
		  tf_velocity = velocity + vec3(0, delta * 0.1, 0);
		else
		  tf_velocity = velocity;
		tf_lifetime = lifetime - delta;
		tf_size = tf_size - delta;
	}
}
