#ifndef CORE_SHADERS_LINE_SHADER_H
#define CORE_SHADERS_LINE_SHADER_H

#include <glm/glm.hpp>
#include <vector>

#include "glslshader.h"
#include "ShaderInterface.h"

class SceneLeafLight;

class LineShader : public ShaderInterface, GLSLProgram
{
public:
	//void initialise(const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix);

	void initialise(const SceneLeafLight& light_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights);

	void initialise(const SceneLeafModel& model_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights);
	
	void initialise(const glm::vec4& colour, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, GLuint vertex_buffer_id);

	static LineShader& getShader();

protected:
	static GLuint modelview_matrix_loc_, projection_matrix_loc_, colour_loc_;
	LineShader(const std::string& vertex_shader, const std::string& fragment_shader);
	
private:
	static LineShader* shader_;
};

#endif
