#ifndef DEMO_PANDORA_SHADERS_CAUSIC_SHADER_H
#define DEMO_PANDORA_SHADERS_CAUSIC_SHADER_H

#include <string>

#include "../../../core/shaders/LightShader.h"

class DirectedLight;
class Texture;
class SceneManager;
class SceneNode;
class CausticTexture;

class CausticShader : public LightShader
{
public:
	/**
	 * We do not render scene leaf lights...
	 */
	virtual void initialise(const SceneLeafLight& light_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights)
	{

	}

	virtual void initialise(const SceneLeafModel& model_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights);

	static void initialiseSun(DirectedLight& sun, CausticTexture& caustic_texture);

	static CausticShader& getShader();
protected:
	GLuint modelview_matrix_loc_, model_matrix_loc_, projection_matrix_loc_, view_matrix_loc_, texture0_loc_, material_ambient_loc_, material_diffuse_loc_, material_specular_loc_, material_emissive_loc_, transparency_loc_, sun_shadow_matrix_loc_, caustic_texture_loc_, caustic_texture_index_loc_, caustic_depth_texture_loc_;
	CausticShader(const std::string& vertex_shader, const std::string& fragment_shader);
	
private:
	static CausticShader* shader_;
	static DirectedLight* sun_;
	static CausticTexture* caustic_texture_;
};

#endif
