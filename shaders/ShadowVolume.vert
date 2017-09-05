#version 420

uniform sampler2D depth_texture;

uniform mat4 projection_matrix;
uniform mat4 modelview_matrix;
uniform mat4 view_matrix;
uniform mat4 model_matrix;
uniform mat4 light_shadow_matrix;

uniform float z_near;
uniform float z_far;

in vec3 a_Vertex;
in float a_cos;

out vec4 temp_pos;
out vec4 worldPos;
out float depth;

void main(void) 
{
	// TODO Make these values in uniforms and calculate them on the CPU.
	float a = z_far / (z_far - z_near);
	float b = z_far * z_near / (z_near - z_far);
	
	vec4 pos;
	if (gl_VertexID != 0)
	{
		vec4 light_pos = light_shadow_matrix * model_matrix * vec4(a_Vertex, 1.0);
		vec4 depth_value = texture2DProj(depth_texture, light_pos);

		// from: http://hub.jmonkeyengine.org/forum/topic/volumetric-lighting-filter-wip/
		float actual_depth = b / (depth_value.r - a);
		actual_depth /= a_cos;

		// Change the location of this vertex by multiplying it with the depth value.
		pos = modelview_matrix * vec4((normalize(a_Vertex) * actual_depth), 1.0);
	}
	else
	{
		pos = modelview_matrix * vec4(a_Vertex, 1.0);
	}

	temp_pos = pos;

	worldPos = projection_matrix * pos;
	gl_Position = projection_matrix * pos;
}
