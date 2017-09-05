#version 420

uniform float delta;

const int min_x = -50;
const int max_x = 50;
const int min_y = -50;
const int max_y = 50;
const int dimensions = 5;

const vec3 vector_field[25] = vec3[]
(
	// Row 0
	vec3(-0.8, 0, 0.2),
	vec3(-0.6, 0, 0.4),
	vec3(-1, 0, -0.1),
	vec3(-1, 0, 0),
	vec3(-1, 0, 0),
	
	// Row 1
	vec3(0.2, 0, 0.9),
	vec3(0.15, 0, 1.2),
	vec3(-0.4, 0, 1),
	vec3(-0.5, 0, 0.8),
	vec3(-0.9, 0, 0.1),
	
	// Row 2
	vec3(0.4, 0, 0.8),
	vec3(0.4, 0, 0.7),
	vec3(0.1, 0, 1),
	vec3(-0.3, 0, 0.8),
	vec3(-0.6, 0, 0.5),
	
	// Row 3
	vec3(0.9, 0, 0.1),
	vec3(0.8, 0, 0.2),
	vec3(0.5, 0, 0.6),
	vec3(0.0, 0, 0.1),
	vec3(-0.2, 0, 1.2),
	
	// Row 4
	vec3(0.8, 0, 0.2),
	vec3(0.8, 0, 0.3),
	vec3(0.5, 0, 0.6),
	vec3(0.3, 0, 0.1),
	vec3(-0.2, 0, 1.2)
);

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
	// Check in which part of the vector field the particle falls.
	int x = int(dimensions * ((position.x - min_x) / (max_x - min_x)));
	int y = int(dimensions * ((position.y - min_y) / (max_y - min_y)));
	
	// Update the velocity.
	tf_velocity = velocity + vector_field[x + y * dimensions] * delta * 0.1;
	
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
