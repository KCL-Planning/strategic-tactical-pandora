#ifndef CORE_SHADERS_TERRAIN_SHADER_H
#define CORE_SHADERS_TERRAIN_SHADER_H

#include "LightShader.h"

class LightManager;
class Terrain;

class TerrainShader : public LightShader
{
public:

	/**
	 * We do not render scene leaf lights...
	 */
	void initialise(const SceneLeafLight& light_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights)
	{

	}

	void initialise(const SceneLeafModel& model_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights);

	static TerrainShader& getShader();
private:
	TerrainShader(const std::string& vertex_shader, const std::string& fragment_shader);
	static TerrainShader* terrain_shader_;
	GLuint modelview_matrix_loc_, model_matrix_loc_, projection_matrix_loc_, view_matrix_loc_, texture0_loc_, texture1_loc_, material_ambient_loc_, material_diffuse_loc_, material_specular_loc_, material_emissive_loc_;

};

#endif
