#ifndef DEMO_PANDORA_SHADERS_CAUSTIC_TERRAIN_SHADER_H
#define DEMO_PANDORA_SHADERS_CAUSTIC_TERRAIN_SHADER_H

#include "dpengine/shaders/LightShader.h"

namespace DreadedPE
{
	class DirectedLight;
	class LightManager;
	class Terrain;
	class Texture;
};
class CausticTexture;

class CausticTerrainShader : public DreadedPE::LightShader
{
public:

	/**
	 * We do not render scene leaf lights...
	 */
	void prepareToRender(const DreadedPE::SceneLeafLight& light_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const DreadedPE::SceneLeafLight*>& lights)
	{

	}

	void prepareToRender(const DreadedPE::SceneLeafModel& model_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const DreadedPE::SceneLeafLight*>& lights);

	void render()
	{
		
	}
	
	static void initialiseSun(DreadedPE::DirectedLight& sun, CausticTexture& caustic_texture);

	static CausticTerrainShader& getShader();
private:
	CausticTerrainShader(const std::string& vertex_shader, const std::string& fragment_shader);
	static CausticTerrainShader* terrain_shader_;
	GLuint modelview_matrix_loc_, model_matrix_loc_, projection_matrix_loc_, view_matrix_loc_, texture0_loc_, texture1_loc_, material_ambient_loc_, material_diffuse_loc_, material_specular_loc_, material_emissive_loc_, sun_shadow_matrix_loc_, caustic_texture_loc_, caustic_texture_index_loc_, caustic_depth_texture_loc_;

	static DreadedPE::DirectedLight* sun_;
	static CausticTexture* caustic_texture_;
};

#endif
