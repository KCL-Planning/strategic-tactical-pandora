#version 430

uniform sampler1D random_texture;
uniform bool spawn_enabled;
uniform float total_time;

layout(points) in;
layout(points, max_vertices = 30) out;

in vec3 tf_position[];
in vec3 tf_velocity[];
in float tf_lifetime[];
in float tf_type[];

out vec3 gposition;
out vec3 gvelocity;
out float glifetime;
out float gtype;

const float PARTICLE_TYPE_LAUNCHER = 0.0f;
const float PARTICLE_TYPE_SHELL = 1.0f;
const float PARTICLE_TYPE_SECONDARY_SHELL = 2.0f;

void main()
{
	gposition = tf_position[0];
	if (tf_type[0] == PARTICLE_TYPE_LAUNCHER)
	{
		// Always make sure we retain our launcher.
		gvelocity = vec3(0, 0, 0);
		if (!spawn_enabled)
			glifetime = 0;
		else if (tf_lifetime[0] < 0)
			glifetime = 0.25;
		else
			glifetime = tf_lifetime[0];
		gtype = PARTICLE_TYPE_LAUNCHER;
		EmitVertex();
		EndPrimitive();

		if (spawn_enabled && tf_lifetime[0] < 0)
		{
			vec4 random = texture(random_texture, total_time);

			gvelocity = tf_velocity[0] + vec3(random.r, random.g, random.b);
			glifetime = 10.0;
			gtype = PARTICLE_TYPE_SHELL;
			EmitVertex();
			EndPrimitive();

			glifetime = 0.25;
		}
	}
	else if (tf_type[0] == PARTICLE_TYPE_SHELL)
	{
		// Create a number of secondary shells if the first shell runs out.
		if (tf_lifetime[0] < 0)
		{
			gvelocity = tf_velocity[0] + vec3(1, 0, 0);
			glifetime = 5.0;
			gtype = PARTICLE_TYPE_SECONDARY_SHELL;
			EmitVertex();
			EndPrimitive();

			gvelocity = tf_velocity[0] + vec3(-1, 0, 0);
			glifetime = 5.0;
			gtype = PARTICLE_TYPE_SECONDARY_SHELL;
			EmitVertex();
			EndPrimitive();

			gvelocity = tf_velocity[0] + vec3(0, 0, 1);
			glifetime = 5.0;
			gtype = PARTICLE_TYPE_SECONDARY_SHELL;
			EmitVertex();
			EndPrimitive();

			gvelocity = tf_velocity[0] + vec3(0, 0, -1);
			glifetime = 5.0;
			gtype = PARTICLE_TYPE_SECONDARY_SHELL;
			EmitVertex();
			EndPrimitive();
		}
		// If the shell is still intact we leave it.
		else
		{
			gvelocity = tf_velocity[0];
			glifetime = tf_lifetime[0];
			gtype = PARTICLE_TYPE_SHELL;
			EmitVertex();
			EndPrimitive();
		}
	}
	// Must be a secondary shell otherwise.
	else if (tf_lifetime[0] > 0)
	{
		gvelocity = tf_velocity[0];
		glifetime = tf_lifetime[0];
		gtype = PARTICLE_TYPE_SECONDARY_SHELL;
		EmitVertex();
		EndPrimitive();
	}
};
