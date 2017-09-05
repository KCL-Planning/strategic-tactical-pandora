#ifndef UNDER_WATER_SHADER_H
#define UNDER_WATER_SHADER_H

#include "glslshader.h"
#include "ShaderInterface.h"

class Texture;
//class WaterShader : public GLSLProgram, public ShaderInterface
class UnderWaterShader : public GLSLProgram
{
public:
	void postProcess(float dt);

	static UnderWaterShader& getShader();

	GLuint getFrameBufferId() const { return fbo_id_; }
private:
	UnderWaterShader(const std::string& vertex_shader, const std::string& fragment_shader);
	void initialise();
	static UnderWaterShader* shader_;
	static GLuint modelview_matrix_loc_, projection_matrix_loc_, time_loc_, fbo_texture_loc_;

	unsigned int width_, height_;
	//GLuint fbo_id_, texture_id_, depth_id_;
	GLuint fbo_id_;//, texture_id2_;
	Texture* texture_;

	GLuint buffer_index_, elements_index_, texture_index_;

	float time_;

	// Data structures to render a quad.
	std::vector<GLuint> m_indices_;
	std::vector<glm::vec2> m_textures_;
	std::vector<glm::vec3> m_vertices_;
};

#endif
