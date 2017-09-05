#ifndef CORE_SHADERS_ANIMATED_SHADOW_SHADER_H
#define CORE_SHADERS_ANIMATED_SHADOW_SHADER_H

#include "BasicShadowShader.h"

class AnimatedShadowShader : public BasicShadowShader
{
public:
	/**
	 * We do not render scene leaf lights...
	 */
	virtual void initialise(const SceneLeafLight& light_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights)
	{

	}

	virtual void initialise(const SceneLeafModel& model_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights);

	static AnimatedShadowShader& getShader();
private:

	static const unsigned int MAX_BONES_ = 100;

	AnimatedShadowShader(const std::string& vertex_shader, const std::string& fragment_shader);
	static AnimatedShadowShader* shader_;
	static GLuint bone_matrix_loc_[AnimatedShadowShader::MAX_BONES_];
};

#endif
