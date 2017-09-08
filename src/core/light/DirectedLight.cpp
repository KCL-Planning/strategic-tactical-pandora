#include "dpengine/light/DirectedLight.h"

#include "dpengine/scene/SceneManager.h"
#include "dpengine/renderer/ShadowRenderer.h"
#include "dpengine/entities/camera/Camera.h"
#include "dpengine/texture/Texture.h"

namespace DreadedPE
{

DirectedLight::DirectedLight(SceneManager& scene_manager, const glm::vec3& ambient, const glm::vec3& diffuse, const glm::vec3& specular, float constant_attenuation, float linear_attenuation, float quadratic_attenuation, GLuint shadow_map_dimension)
	: Light(DIRECTIONAL, ambient, diffuse, specular, constant_attenuation, linear_attenuation, quadratic_attenuation, 0.1f, 10000.0f, shadow_map_dimension)
{
	shadow_renderer_ = new ShadowRenderer(scene_manager, shadow_map_dimension);
}

DirectedLight::~DirectedLight()
{
	delete shadow_renderer_;
}

void DirectedLight::preRender(const glm::mat4& transition)
{
	Light::preRender(transition);
	shadow_renderer_->render(*this);

	glm::mat4 shadow_perspective_matrix = getPerspectiveMatrix();
	glm::mat4 shadow_scale_matrix = glm::translate(glm::mat4(1.0), glm::vec3(0.5f, 0.5f, 0.5f));
	shadow_scale_matrix = glm::scale(shadow_scale_matrix, glm::vec3(0.5f, 0.5f, 0.5f));

	shadow_matrix_ = shadow_scale_matrix * shadow_perspective_matrix * getViewMatrix();
}

glm::mat4 DirectedLight::getPerspectiveMatrix() const
{
	//return glm::perspective(90.0f, 1.0f, 0.1f, 60.0f);
	return glm::ortho(-25.0f, 25.0f, -25.0f, 25.0f, 2.0f, 50.0f);
}

GLuint DirectedLight::getShadowRendererTextureID() const
{
	return shadow_renderer_->getTexture().getActiveTextureId();
}

};
