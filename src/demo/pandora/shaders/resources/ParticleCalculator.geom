#version 430

uniform sampler1D random_texture;
uniform bool spawn_enabled;
uniform float spawn_rate;
uniform float spawn_lifetime;

layout(points) in;
layout(points, max_vertices = 2) out;

in vec3 tf_position[];
in vec3 tf_velocity[];
in float tf_lifetime[];
in float tf_type[];

out vec3 gposition;
out vec3 gvelocity;
out float glifetime;
out float gtype;

void main()
{
	gposition = tf_position[0];
	if (tf_type[0] == 0)
	{
		// Always make sure we retain our launcher.
		if (!spawn_enabled)
		{
			glifetime = 0;
		}
		else
		{
			glifetime = tf_lifetime[0];
		}
		gvelocity = vec3(0, 0, 0);
		gtype = 0;

		// Create a new particle everytime the timer runs out.
		if (glifetime < 0)
		{
			// Reset the spawner's time.
			glifetime += spawn_rate;
			EmitVertex();
			EndPrimitive();


			vec4 random = texture(random_texture, tf_lifetime[0] * 1024 * 1024);
			gvelocity = tf_velocity[0] + vec3(random.r, random.g, random.b) * 1.0;
			gtype = 1.0;
			glifetime = spawn_lifetime;
			EmitVertex();
			EndPrimitive();
		}
		else
		{
			EmitVertex();
			EndPrimitive();
		}
	}
	// We leave non-launchers as normal.
	else if (tf_lifetime[0] > 0)
	{
		gvelocity = tf_velocity[0];
		glifetime = tf_lifetime[0];
		gtype = tf_type[0];
		EmitVertex();
		EndPrimitive();
	}
};
