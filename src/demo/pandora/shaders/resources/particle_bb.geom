#version 430

uniform sampler2D texture0;

uniform mat4 modelview_matrix;
uniform mat4 projection_matrix;
uniform vec3 camera_location;

layout(points) in;
layout(triangle_strip, max_vertices = 4) out;

in vec3 location[];
in vec3 vvelocity[];
in float vlifetime[];
in float vtype[];

out float glifetime;
out vec2 texCoord0;
out float blendFactor;

void main()
{
	vec4 pos = gl_in[0].gl_Position - vec4(camera_location, 0);

	// Determine the locations of the corners so it faces the camera.
	vec4 axis_y = vec4(normalize(vec3(-pos.y / pos.x, 1, 0)), 0) * vtype[0] / 180.0;
	vec4 axis_z = vec4(normalize(cross(vec3(pos), vec3(axis_y))), 0) * vtype[0] / 180.0;

	// CCW.
	gl_Position = projection_matrix * modelview_matrix * (gl_in[0].gl_Position + axis_y + axis_z);
	glifetime = vlifetime[0];
	texCoord0 = vec2(1, 1);
	blendFactor = exp2(-.1 * length(gl_Position));
	EmitVertex();

	gl_Position = projection_matrix * modelview_matrix * (gl_in[0].gl_Position - axis_y + axis_z);
	glifetime = vlifetime[0];
	texCoord0 = vec2(0, 1);
	blendFactor = exp2(-.1 * length(gl_Position));
	EmitVertex();

	gl_Position = projection_matrix * modelview_matrix * (gl_in[0].gl_Position + axis_y - axis_z);
	glifetime = vlifetime[0];
	texCoord0 = vec2(1, 0);
	blendFactor = exp2(-.1 * length(gl_Position));
	EmitVertex();

	gl_Position = projection_matrix * modelview_matrix * (gl_in[0].gl_Position - axis_y - axis_z);
	glifetime = vlifetime[0];
	texCoord0 = vec2(0, 0);
	blendFactor = exp2(-.1 * length(gl_Position));
	EmitVertex();
	EndPrimitive();
};
