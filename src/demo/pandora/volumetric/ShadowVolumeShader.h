#ifndef DEMO_VOLLUMETRIC_SHADOW_VOLUME_SHADER_H
#define DEMO_VOLLUMETRIC_SHADOW_VOLUME_SHADER_H

#include <dpengine/shaders/ShaderInterface.h>
#include <dpengine/shaders/glslshader.h>

namespace DreadedPE
{
class SceneLeafModel;
class LightVolumeShape;
class Light;
class Texture;
};

class ShadowVolumeShader : public DreadedPE::GLSLProgram, public DreadedPE::ShaderInterface
{
public:
	static ShadowVolumeShader& getShader();
	
	void prepareToRender(const DreadedPE::SceneLeafModel& model_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const DreadedPE::SceneLeafLight*>& lights);
	
	void render()
	{

	}

	void initialise(const DreadedPE::SceneLeafModel& model_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const DreadedPE::SceneLeafLight& light, float camera_near_plane, float camera_far_plane, DreadedPE::Texture& camera_depth_texture);
protected:
	GLuint model_matrix_loc_, modelview_matrix_loc_, projection_matrix_loc_, z_near_loc_, z_far_loc_, camera_near_plane_loc_, camera_far_plane_loc_, view_matrix_loc_, depth_texture_loc_, light_shadow_matrix_loc_, camera_depth_texture_loc_, shadow_matrix_loc_, light_pos_loc_, light_colour_loc_;
	ShadowVolumeShader(const std::string& vertex_shader, const std::string& fragment_shader);
	
private:
	static ShadowVolumeShader* shader_;
	bool depth_not_initialised_;
	GLuint m_depth_buffer_;
	GLuint m_cos_buffer_;
	std::vector<float> m_cos_;

	void loadDepth(const DreadedPE::Light& light);
};

#endif
