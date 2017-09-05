#ifndef DEMO_VOLLUMETRIC_SHADOW_VOLUME_SHADER_H
#define DEMO_VOLLUMETRIC_SHADOW_VOLUME_SHADER_H

#include "../../core/shaders/ShaderInterface.h"
#include "../../core/shaders/glslshader.h"

class SceneLeafModel;
class LightVolumeShape;
class Light;
class Texture;

class ShadowVolumeShader : public GLSLProgram, public ShaderInterface
{
public:
	static ShadowVolumeShader& getShader();
	
	void initialise(const SceneLeafLight& light_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights);

	void initialise(const SceneLeafModel& model_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights);
	
	void initialise(const SceneLeafModel& model_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const SceneLeafLight& light, float camera_near_plane, float camera_far_plane, Texture& camera_depth_texture);
protected:
	GLuint model_matrix_loc_, modelview_matrix_loc_, projection_matrix_loc_, z_near_loc_, z_far_loc_, camera_near_plane_loc_, camera_far_plane_loc_, view_matrix_loc_, depth_texture_loc_, light_shadow_matrix_loc_, camera_depth_texture_loc_, shadow_matrix_loc_, light_pos_loc_, light_colour_loc_;
	ShadowVolumeShader(const std::string& vertex_shader, const std::string& fragment_shader);
	
private:
	static ShadowVolumeShader* shader_;
	bool depth_not_initialised_;
	GLuint m_depth_buffer_;
	GLuint m_cos_buffer_;
	std::vector<float> m_cos_;

	void loadDepth(const Light& light);
};

#endif
