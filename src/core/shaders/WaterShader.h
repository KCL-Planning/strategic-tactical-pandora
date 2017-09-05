#ifndef WATER_SHADER_H
#define WATER_SHADER_H

#include "LightShader.h"

//class WaterShader : public GLSLProgram, public ShaderInterface
class WaterShader : public LightShader
{
public:
	/**
	 * We do not render scene leaf lights...
	 */
	void initialise(const SceneLeafLight& light_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights)
	{

	}

	void initialise(const SceneLeafModel& model_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights);

	static WaterShader& getShader();
private:
	WaterShader(const std::string& vertex_shader, const std::string& fragment_shader);
	static WaterShader* shader_;
	static GLuint modelview_matrix_loc_, model_matrix_loc_, projection_matrix_loc_, view_matrix_loc_, texture0_loc_, material_ambient_loc_, material_diffuse_loc_, material_specular_loc_, material_emissive_loc_;
};

#endif
