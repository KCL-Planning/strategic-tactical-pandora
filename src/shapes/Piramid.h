#ifndef PIRAMID_H
#define PIRAMID_H

#include "Shape.h"

class Camera;
class LightManager;
class LightShader;

class Piramid: public Shape
{
public:
	Piramid(float height, float width, float length);
	//Piramid(LightShader& m_GLSLProgram, float x, float y, float z);
	//void initialise();
	//void prepare(float dt);
	//void render(const Camera& cam, bool render_shadow, const LightManager& light_manager, GLSLProgram* shader);

private:
	//unsigned int buffer_index, colours_index, elements_index, elements_index2, texture_index, texture_index2, normals_index, normals_index2;
	//std::vector<float> m_vertices, m_colours, m_normals, m_normals2;
	//std::vector<glm::vec2> m_textures, m_textures2;
	//std::vector<unsigned int> m_indices, m_indices2;
};

#endif
