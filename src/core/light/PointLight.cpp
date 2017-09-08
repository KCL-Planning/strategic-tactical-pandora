#define _USE_MATH_DEFINES
#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp> 
#include <glm/gtc/quaternion.hpp>

#include <fstream>
#include <iostream>

#include "dpengine/renderer/ShadowRenderer.h"
#include "dpengine/light/PointLight.h"
#include "dpengine/entities/camera/Camera.h"

namespace DreadedPE
{

PointLight::PointLight(SceneManager& scene_manager, const glm::vec3& ambient, const glm::vec3& diffuse, const glm::vec3& specular, float constant_attenuation, float linear_attenuation, float quadratic_attenuation)
	: Light(POINTLIGHT, ambient, diffuse, specular, constant_attenuation, linear_attenuation, quadratic_attenuation, 90.0f, 0.1f, 120.0f)
{
	
}

PointLight::~PointLight()
{
	
}

glm::mat4 PointLight::getPerspectiveMatrix() const
{
	return glm::perspective(glm::radians(light_angle_ * 2), 1.0f, close_plane_, far_plane_);
}

void PointLight::preRender(const glm::mat4& transition)
{
	Light::preRender(transition);
}

};
