#include "GodRays.h"
#include <limits>
#include "GL/glew.h"

#include "../example.h"
#include "../core/Camera.h"
#include "../core/light/PointLight.h"
#include "../core/shaders/glslshader.h"

GLSLProgram* GodRays::shader_ = NULL;
GLuint GodRays::attribute_v_coord_postproc_ = 0;

GodRays::GodRays(Example& example, const PointLight& light, unsigned int width, unsigned int height)
	: example_(&example), light_(&light), width_(width), height_(height)
{
	
}

GLSLProgram* GodRays::getShader()
{
	if (shader_ == NULL)
	{
		shader_ =  new GLSLProgram("shaders/godrays.vert", "shaders/godrays.frag");
		shader_->initialize();
		shader_->bindAttrib(0, "a_Vertex");
		shader_->bindAttrib(1, "a_TexCoord0");
		shader_->linkProgram();
		shader_->bindShader();

		attribute_v_coord_postproc_ = shader_->getAttribLocation("a_Vertex");
	}
	return shader_;
}

void GodRays::initialise()
{
	glGenTextures(1, &texture_id_);
	glBindTexture(GL_TEXTURE_2D, texture_id_);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width_, height_, 0, GL_RGB, GL_FLOAT, NULL);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	
	glGenTextures(1, &depth_id_);
	glBindTexture(GL_TEXTURE_2D, depth_id_);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32, width_, height_, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
	
	glGenFramebuffers(1, &fbo_id_);
	glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);

	// Attach the rgb texture to it.
	glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, texture_id_, 0);
	glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, depth_id_, 0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	glActiveTexture(GL_TEXTURE13);
	glBindTexture(GL_TEXTURE_2D, texture_id_);
	glActiveTexture(GL_TEXTURE30);


	glGenTextures(1, &texture_id2_);
	glBindTexture(GL_TEXTURE_2D, texture_id2_);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width_, height_, 0, GL_RGB, GL_FLOAT, NULL);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	
	glGenFramebuffers(1, &fbo_id2_);
	glBindFramebuffer(GL_FRAMEBUFFER, fbo_id2_);

	// Attach the rgb texture to it.
	glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, texture_id2_, 0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	glActiveTexture(GL_TEXTURE17);
	glBindTexture(GL_TEXTURE_2D, texture_id2_);
	glActiveTexture(GL_TEXTURE30);

	// Create a 2D coordinate system.
	m_vertices_.push_back(Vertex(width_, height_, 0));
	m_vertices_.push_back(Vertex(0, height_, 0));
	m_vertices_.push_back(Vertex(0, 0, 0));
	m_vertices_.push_back(Vertex(width_, 0, 0));
	
	m_textures_.push_back(TexCoord(1, 0));
	m_textures_.push_back(TexCoord(0, 0));
	m_textures_.push_back(TexCoord(0, 1));
	m_textures_.push_back(TexCoord(1, 1));

	m_indices_.push_back(0);
	m_indices_.push_back(3);
	m_indices_.push_back(2);
	
	m_indices_.push_back(0);
	m_indices_.push_back(2);
	m_indices_.push_back(1);

	glGenBuffers(1, &buffer_index_);
    glBindBuffer(GL_ARRAY_BUFFER, buffer_index_); //Bind the vertex buffer
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * m_vertices_.size(), &m_vertices_[0], GL_STATIC_DRAW); //Send the data to OpenGL
	
	// To draw the upper part of the piramid.
	glGenBuffers(1, &elements_index_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elements_index_); //Bind the vertex buffer
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * m_indices_.size(), &m_indices_[0], GL_STATIC_DRAW); //Send the data to OpenGL
	
	// Generate the buffer for the texture coordinates.
	glGenBuffers(1, &texture_index_);
	glBindBuffer(GL_ARRAY_BUFFER, texture_index_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 2 * m_textures_.size(), &m_textures_[0], GL_STATIC_DRAW);

	getShader();
}

void GodRays::prepare(const Camera& camera)
{
	// Check if the light is facing our direction.
	Vertex light_direction = light_->getCurrentDirection();
	light_direction.normalise();

	Vertex light_to_cam(camera.getPosition().x - light_->getCurrentLocation().x, camera.getPosition().y - light_->getCurrentLocation().y, camera.getPosition().z - light_->getCurrentLocation().z);
	light_to_cam.normalise();

	float dot_product = light_direction.x * light_to_cam.x + light_direction.y * light_to_cam.y + light_direction.z * light_to_cam.z;
	
	if (dot_product < 0.0)
	{
		light_location_.x = -1;
		light_location_.y = -1;
	}
	else
	{
		// Render the scene such that only the lightpoints are visible. The rest of the scene is drawn black.
		glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);
		glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	
		example_->render(camera, true, false, false, false, true);//, PointLight::getShader());
	
		glBindFramebuffer(GL_FRAMEBUFFER, 0);

		// Calculate the light's position.
		glm::detail::tmat4x4<float> perspective_matrix = glm::perspective(45.0f, float(1024) / float(768), 0.1f, 100.0f);
		glm::vec4 light_location(light_->getCurrentLocation().x, light_->getCurrentLocation().y, light_->getCurrentLocation().z, 1.0f);

		glm::vec3 camera_location(-camera.getPosition().x, -camera.getPosition().y, -camera.getPosition().z);
		glm::mat4 view_matrix = glm::rotate(glm::mat4(1.0f), -camera.getPitch(), glm::vec3(1.0f, 0.0f, 0.0f));
		view_matrix = glm::rotate(view_matrix, -camera.getYaw(), glm::vec3(0.0f, 1.0f, 0.0f));
		view_matrix = glm::rotate(view_matrix, -camera.getRoll(), glm::vec3(0.0f, 0.0f, 1.0f));
		view_matrix = glm::translate(view_matrix, camera_location);
	
		light_location_ = perspective_matrix * view_matrix * light_location;
		light_location_ /= light_location_.w;

		if (light_location_.x < 1.0 && light_location_.x > -1.0 && light_location_.y < 1.0 && light_location_.y > -1.0 && light_location_.z < 1.0 && light_location_.z > 0.0)
		{
			light_location_.x = (light_location_.x + 1.0f) /* 1024*/ / 2.0f;
			light_location_.y = (light_location_.y + 1.0f) /* 768*/ / 2.0f;
		}
		else
		{
			//std::numeric_limits<float>::max();
			light_location_.x = -1;
			light_location_.y = -1;
		}
	}
}

void GodRays::postProcess()
{
	float simpleModelviewMatrix[16];
	float simpleProjecetionMatrix[16];
	glBindFramebuffer(GL_FRAMEBUFFER, fbo_id2_);
	glClear(GL_COLOR_BUFFER_BIT);
	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, width_, height_, 0, -1, 1);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	
	glGetFloatv(GL_MODELVIEW_MATRIX, simpleModelviewMatrix);
	glGetFloatv(GL_PROJECTION_MATRIX, simpleProjecetionMatrix);
	
	shader_->bindShader();
	shader_->sendUniform("fbo_texture", 13);
	shader_->sendUniform("screen_texture", 14);
	shader_->sendUniform("exposure", 1.0f);
	shader_->sendUniform("decay", 0.99f);
	shader_->sendUniform("density", 1.0f);
	shader_->sendUniform("weight", 1.0f);

	shader_->sendUniform("light_position", light_location_.x, light_location_.y, light_location_.z, light_location_.w);
	shader_->sendUniform4x4("modelview_matrix", simpleModelviewMatrix);
	shader_->sendUniform4x4("projection_matrix", simpleProjecetionMatrix);
	
	glBindBuffer(GL_ARRAY_BUFFER, buffer_index_);
	glVertexAttribPointer((GLint)0, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, texture_index_);
	glVertexAttribPointer((GLint)1, 2, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elements_index_);
	glDrawElements(GL_TRIANGLES, m_indices_.size(), GL_UNSIGNED_INT, 0);

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}
