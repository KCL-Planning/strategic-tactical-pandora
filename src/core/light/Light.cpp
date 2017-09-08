#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp> 
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include "dpengine/light/Light.h"

namespace DreadedPE
{

Light::Light(LIGHT_TYPE type, const glm::vec3& ambient, const glm::vec3& diffuse, const glm::vec3& specular, float constant_attenuation, float linear_attenuation, float quadratic_attenuation, float light_angle, float close_plane, float far_plane, GLuint shadow_map_dimension)
	: type_(type), ambient_(ambient), diffuse_(diffuse), specular_(specular), constant_attenuation_(constant_attenuation), linear_attenuation_(linear_attenuation), quadratic_attenuation_(quadratic_attenuation), light_angle_(light_angle), close_plane_(close_plane), far_plane_(far_plane), shadow_map_dimension_(shadow_map_dimension)
{

}

void Light::preRender(const glm::mat4& transition)
{
	// Extract the angles.
	glm::fquat quaternion = glm::quat_cast(transition);

	location_ = glm::vec3(transition[3][0], transition[3][1], transition[3][2]);

	yaw_ = glm::yaw(quaternion);
	pitch_ = glm::pitch(quaternion);
	roll_ = glm::roll(quaternion);

	direction_ = glm::rotate(quaternion, glm::vec3(0, 0, -1));
}

void Light::prepare(float dt)
{

}

glm::mat4 Light::getViewMatrix() const
{
	glm::vec3 camera_location(-location_);
	glm::mat4 view_matrix = glm::rotate(glm::mat4(1.0f), -pitch_, glm::vec3(1.0f, 0.0f, 0.0f));
	view_matrix = glm::rotate(view_matrix, -yaw_, glm::vec3(0.0f, 1.0f, 0.0f));
	view_matrix = glm::rotate(view_matrix, -roll_, glm::vec3(0.0f, 0.0f, 1.0f));
	view_matrix = glm::translate(view_matrix, camera_location);
	return view_matrix;
}

};
