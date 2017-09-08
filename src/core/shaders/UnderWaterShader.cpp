#include "dpengine/shaders/UnderWaterShader.h"

#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "dpengine/light/Light.h"
#include "dpengine/scene/Material.h"
#include "dpengine/shapes/Water.h"
#include "dpengine/scene/SceneLeafModel.h"
#include "dpengine/texture/Texture.h"
#include "dpengine/renderer/Window.h"

namespace DreadedPE
{

UnderWaterShader* UnderWaterShader::shader_ = NULL;
GLuint UnderWaterShader::modelview_matrix_loc_ = 0;
GLuint UnderWaterShader::projection_matrix_loc_ = 0;
GLuint UnderWaterShader::time_loc_ = 0;
GLuint UnderWaterShader::fbo_texture_loc_ = 0;

UnderWaterShader::UnderWaterShader(const std::string& vertex_shader, const std::string& fragment_shader)
	: GLSLProgram(vertex_shader, fragment_shader)
{
	Window::getActiveWindow()->getSize(width_, height_);
	
	// TODO: Change to the new texture system.
	assert(false);
}

void UnderWaterShader::initialise()
{
	texture_ = new Texture(GL_TEXTURE_2D);
	
	glBindTexture(GL_TEXTURE_2D, texture_->getTextureId());
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width_, height_, 0, GL_RGB, GL_FLOAT, NULL);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	
	glGenFramebuffers(1, &fbo_id_);
	glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);

	// Attach the rgb texture to it.
	glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, texture_->getTextureId(), 0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	// Create a 2D coordinate system.
	m_vertices_.push_back(glm::vec3(width_, height_, 0));
	m_vertices_.push_back(glm::vec3(0, height_, 0));
	m_vertices_.push_back(glm::vec3(0, 0, 0));
	m_vertices_.push_back(glm::vec3(width_, 0, 0));
	
	m_textures_.push_back(glm::vec2(1, 0));
	m_textures_.push_back(glm::vec2(0, 0));
	m_textures_.push_back(glm::vec2(0, 1));
	m_textures_.push_back(glm::vec2(1, 1));

	m_indices_.push_back(0);
	m_indices_.push_back(3);
	m_indices_.push_back(2);
	
	m_indices_.push_back(0);
	m_indices_.push_back(2);
	m_indices_.push_back(1);

	glGenBuffers(1, &buffer_index_);
	glBindBuffer(GL_ARRAY_BUFFER, buffer_index_); //Bind the vertex buffer
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * m_vertices_.size(), &m_vertices_[0], GL_STATIC_DRAW); //Send the data to OpenGL
	
	glGenBuffers(1, &elements_index_);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elements_index_); //Bind the vertex buffer
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * m_indices_.size(), &m_indices_[0], GL_STATIC_DRAW); //Send the data to OpenGL
	
	// Generate the buffer for the texture coordinates.
	glGenBuffers(1, &texture_index_);
	glBindBuffer(GL_ARRAY_BUFFER, texture_index_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 2 * m_textures_.size(), &m_textures_[0], GL_STATIC_DRAW);
	time_ = 0;
	getShader();
}

void UnderWaterShader::postProcess(float dt)
{
	bindShader();

	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glDisableVertexAttribArray(2);
	glDisableVertexAttribArray(3);
	glDisableVertexAttribArray(4);
	glDisableVertexAttribArray(5);
	glDisableVertexAttribArray(6);
	glDisableVertexAttribArray(7);

	float simpleModelviewMatrix[16];
	float simpleProjecetionMatrix[16];
	glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);
	glClear(GL_COLOR_BUFFER_BIT);
	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, width_, height_, 0, -1, 1);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	
	glGetFloatv(GL_MODELVIEW_MATRIX, simpleModelviewMatrix);
	glGetFloatv(GL_PROJECTION_MATRIX, simpleProjecetionMatrix);
	
	glUniformMatrix4fv(modelview_matrix_loc_, 1, false, simpleModelviewMatrix);
	glUniformMatrix4fv(projection_matrix_loc_, 1, false, simpleProjecetionMatrix);
	
	time_ += dt;
	glUniform1i(fbo_texture_loc_, texture_->getActiveTextureId());
	glUniform1f(time_loc_, time_);
	
	glBindBuffer(GL_ARRAY_BUFFER, buffer_index_);
	glVertexAttribPointer((GLint)0, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, texture_index_);
	glVertexAttribPointer((GLint)1, 2, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elements_index_);
	glDrawElements(GL_TRIANGLES, m_indices_.size(), GL_UNSIGNED_INT, 0);

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

UnderWaterShader& UnderWaterShader::getShader()
{
	if (shader_ == NULL)
	{
		shader_ =  new UnderWaterShader("shaders/underwater.vert", "shaders/underwater.frag");

		// Load the shader.
		if (!shader_->initialize())
		{
			std::cerr << "Failed to initialise the under water shader." << std::endl;
			exit(1);
		}

		shader_->initialize();
		shader_->bindAttrib(0, "a_Vertex");
		shader_->bindAttrib(1, "a_TexCoord0");
		shader_->linkProgram();
		shader_->bindShader();

		//Re link the program
		shader_->linkProgram();
		shader_->bindShader();

		// Cache the locations of all the uniform variables.
		modelview_matrix_loc_ = shader_->getUniformLocation("modelview_matrix");
		projection_matrix_loc_ = shader_->getUniformLocation("projection_matrix");
		time_loc_ = shader_->getUniformLocation("time");
		fbo_texture_loc_ = shader_->getUniformLocation("fbo_texture");
		shader_->initialise();
	}
	return *shader_;
}

};
