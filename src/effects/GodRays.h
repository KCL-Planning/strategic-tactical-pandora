#ifndef GOD_RAYS_H
#define GOD_RAYS_H

#include "GL/glew.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp> 

class Camera;
class Example;
class PointLight;
class GLSLProgram;

class GodRays
{
public:
	GodRays(Example& example, const PointLight& light, unsigned int width, unsigned int height);
	void initialise();
	void prepare(const Camera& camera);
	void postProcess();
	glm::vec4 glToScreen(const glm::vec4& v);
	const glm::vec4& getLightLocation() const { return light_location_; }

	unsigned int getFrameBufferId() const { return fbo_id2_; }
private:

	static GLSLProgram* shader_;
	static GLSLProgram* getShader();
	static GLuint attribute_v_coord_postproc_;


	Example* example_;
	const PointLight* light_; 
	unsigned int width_, height_;
	unsigned int fbo_id_, texture_id_, depth_id_;
	unsigned int fbo_id2_, texture_id2_;
	unsigned int vbo_fbo_vertices_;

	//float modelviewMatrix_[16];
	//float projectionMatrix_[16];

	// Data structures to render a quad.
	std::vector<unsigned int> m_indices_;
	std::vector<glm::vec2> m_textures_;
	std::vector<glm::vec3> m_vertices_;

	glm::vec4 light_location_;

	unsigned int buffer_index_, elements_index_, texture_index_;
};

#endif
