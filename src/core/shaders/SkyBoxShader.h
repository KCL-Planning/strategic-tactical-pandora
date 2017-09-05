#ifndef CORE_SHADERS_SKY_BOX_SHADER_H
#define CORE_SHADERS_SKY_BOX_SHADER_H

#include "ShaderInterface.h"
#include "glslshader.h"

class SkyBoxShader : public GLSLProgram, public ShaderInterface
{
public:
	SkyBoxShader(const std::string& vertex_shader, const std::string& fragment_shader);

	void initialise(const SceneLeafLight& light_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights);

	void initialise(const SceneLeafModel& model_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights);

	static SkyBoxShader& getShader();

private:
	static SkyBoxShader* shader_;
	GLuint modelview_matrix_loc_, projection_matrix_loc_, texture0_loc_;
};

#endif
