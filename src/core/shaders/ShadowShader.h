#ifndef CORE_SHADERS_SHADOW_SHADER_H
#define CORE_SHADERS_SHADOW_SHADER_H

#include <GL/glew.h>
#include <string>

#include "glslshader.h"
#include "ShaderInterface.h"

class ShadowShader : public GLSLProgram, public ShaderInterface
{
public:

	/**
	 * Initialise all the uniform and attributes of this shader.
	 * @param light_node The scene node which represents a light.
	 * @param view_matrix The view matrix.
	 * @param model_matrix The model matrix.
	 * @param projection_matrix The projectiong matrix.
	 */
	virtual void initialise(const SceneLeafLight& light_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights)
	{
		
	}

	/**
	 * Initialise all the uniform and attributes of this shader.
	 * @param model_node The scene node which represents a model.
	 * @param view_matrix The view matrix.
	 * @param model_matrix The model matrix.
	 * @param projection_matrix The projectiong matrix.
	 */
	virtual void initialise(const SceneLeafModel& model_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights);

	static ShadowShader& getShader();
protected:
	GLuint projectiomodelview_matrix_loc_;
	GLuint texture0_loc_;
	ShadowShader(const std::string& vertex_shader, const std::string& fragment_shader);
	
private:
	static ShadowShader* shader_;
};

#endif

