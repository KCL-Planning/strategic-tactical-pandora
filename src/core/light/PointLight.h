#ifndef DIRECTED_LIGHT_H
#define DIRECTED_LIGHT_H

#include "../shaders/LightShader.h"
#include "Light.h"

class Entity;
class Example;
class GLSLProgram;
class ShadowRenderer;
class SceneManager;

class PointLight : public Light
{
public:
	PointLight(SceneManager& scene_manager, float light_angle, const glm::vec3& ambient, const glm::vec3& diffuse, const glm::vec3& specular, float constant_attenuation, float linear_attenuation, float quadratic_attenuation, float close_plane, float far_plane, GLuint shadow_map_dimension = 2048, GLenum cull_mode = GL_FRONT, GLint texture_compare_mode = GL_COMPARE_REF_TO_TEXTURE, GLint texture_compare_function = GL_LEQUAL);
	~PointLight();
	void preRender(const glm::mat4& transition);

	glm::mat4 getPerspectiveMatrix() const;

	ShadowRenderer& getShadowRenderer() const { return *shadow_renderer_; }
	
	// Debug.
	void outputDepthBufferToFile(const std::string& file_name) const;
private:
	ShadowRenderer* shadow_renderer_;
};

#endif
