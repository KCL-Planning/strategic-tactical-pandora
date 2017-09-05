uniform vec4 material_ambient;
uniform vec4 material_diffuse;
uniform vec4 material_specular;
uniform vec4 material_emissive;

const int nr_lights = 5;

/**
 * Lights can have multiple types, these are:
 * 0: Directional light (e.g. sun light).
 * 1: Spot light (e.g. a torch).
 * 2: Point light (e.g. a lamp).
 */
struct light {
	int type;
	bool enabled;
	vec4 position;
	vec4 diffuse;
	vec4 ambient;
	vec4 specular;

	mat4 shadow_matrix;

	float constant_attenuation;
	float linear_attenuation;
	float quadratic_attenuation;

	vec3 direction;
	float light_angle;
};

uniform sampler2DShadow depth_texture[nr_lights];

uniform light lights[nr_lights];
