#ifndef CORE_SHADERS_TOON_SHADER_H
#define CORE_SHADERS_TOON_SHADER_H

#include "LightShader.h"

class ToonShader : public LightShader
{
public:
	/**
	 * We do not render scene leaf lights...
	 */
	virtual void initialise(const SceneLeafLight& light_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights)
	{

	}

	virtual void initialise(const SceneLeafModel& model_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights);

	static ToonShader& getShader();
	
	void setColour(const glm::vec4& colour) { colour_ = colour; }
protected:
	GLuint modelview_matrix_loc_, model_matrix_loc_, projection_matrix_loc_, view_matrix_loc_, texture0_loc_, material_ambient_loc_, material_diffuse_loc_, material_specular_loc_, material_emissive_loc_;
	ToonShader(const std::string& vertex_shader, const std::string& fragment_shader);
	
private:
	static ToonShader* shader_;
	glm::vec4 colour_;
};

#endif
