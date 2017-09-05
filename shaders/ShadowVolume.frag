#version 420

uniform sampler2D camera_depth_texture; 
uniform mat4 shadow_matrix;
uniform mat4 view_matrix;

uniform float camera_near;
uniform float camera_far;
uniform vec3 light_pos;
uniform vec3 light_colour;

in vec4 worldPos;
in vec4 temp_pos;

out vec4 outColor;

float ScatteringIntegral(vec3 cameraPos, vec3 lightPos, vec3 direction, float thickness)
{
    vec3 lightToCam = cameraPos - lightPos;

    // coefficients
    float direct = dot(direction, lightToCam);
    float scattered = dot(lightToCam, lightToCam);

    // evaluate integral
    float scattering = 1.0 / sqrt(scattered - direct*direct);
    return scattering*(atan( (thickness+direct)*scattering) - atan(direct*scattering));
}

void main(void)
{
	float a = camera_far / (camera_far - camera_near);
	float b = camera_far * camera_near / (camera_near - camera_far);

	// Calculate the volume of space that is occupied.
	vec4 direction = worldPos;
	float length = length(worldPos.xyz);
	vec3 normalized_direction = direction.xyz / length;

	vec4 projected_cam_pos = shadow_matrix * worldPos;

	// Clip the length to the depth buffer.
	vec4 depth_value = texture2DProj(camera_depth_texture, projected_cam_pos);
	float actual_depth = b / (depth_value.r - a);
	
	length = min(length, actual_depth);

	//vec3 light_pos = vec3(0.0, 14.2, -15.25);
	vec3 cam_pos = vec3(view_matrix[0][0], view_matrix[1][0], view_matrix[2][0]);

	//float light_influence = ScatteringIntegral(cam_pos, light_pos, normalized_direction, length);
	float light_influence = length / (camera_far - camera_near);

	// If the fragment is facing away from the camera we ADD the light influence, otherwise we remove it.
	if (gl_FrontFacing)
	{
		light_influence = -light_influence;
	}

	light_influence *= 10;
	
	// Now we assign a 'light value' for this fragment.
	outColor = vec4(light_colour.r * light_influence, light_colour.g * light_influence, light_colour.b * light_influence, light_influence);
}
