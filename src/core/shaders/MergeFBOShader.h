#ifndef CORE_SHADER_MERGE_FBO_SHADER_H
#define CORE_SHADER_MERGE_FBO_SHADER_H

#include "glslshader.h"
#include "ShaderInterface.h"

class Texture;

class MergeFBOShader : public GLSLProgram
{
public:
	void postProcess(const Texture& texture0, const Texture& texture1, float dt);

	static MergeFBOShader& getShader();

	GLuint getFrameBufferId() const { return fbo_id_; }

	void onResize(int width, int height);
private:
	MergeFBOShader(const std::string& vertex_shader, const std::string& fragment_shader);
	void initialise();
	static MergeFBOShader* shader_;
	static GLuint modelview_matrix_loc_, projection_matrix_loc_, fbo_texture_loc_, screen_texture_loc_;

	unsigned int width_, height_;
	GLuint fbo_id_;//, texture_id_;
	Texture* texture_;
	
	GLuint buffer_index_, elements_index_, texture_index_;

	// Data structures to render a quad.
	std::vector<GLuint> m_indices_;
	std::vector<glm::vec2> m_textures_;
	std::vector<glm::vec3> m_vertices_;
};

#endif
