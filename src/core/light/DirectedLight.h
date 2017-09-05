#ifndef CORE_LIGHT_SUN_H
#define CORE_LIGHT_SUN_H

#include <glm/glm.hpp>
#include "Light.h"

class SceneManager;
class ShadowRenderer;

/**
 * Create a light point that is an infinite distance away from the camera, i.e. a sun.
 * This light point projects light from a certain angle that is constant accross the 
 * entire scene.
 */

class DirectedLight : public Light
{
public:
	DirectedLight(SceneManager& scene_manager, const glm::vec3& ambient, const glm::vec3& diffuse, const glm::vec3& specular, float constant_attenuation, float linear_attenuation, float quadratic_attenuation, GLuint shadow_map_dimension = 2048);
	~DirectedLight();
	void preRender(const glm::mat4& transition);

	glm::mat4 getPerspectiveMatrix() const;

	ShadowRenderer& getShadowRenderer() const { return *shadow_renderer_; }
private:
	ShadowRenderer* shadow_renderer_;
};

#endif
