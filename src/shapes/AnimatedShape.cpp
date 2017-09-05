#include <vector>
#include <string>
#include <glm/glm.hpp>

#include "AnimatedShape.h"

AnimatedShape::AnimatedShape(const std::vector<const std::vector<glm::vec3>* >& m_vertices,
                             std::vector<Animation>& animations,
                             const std::vector<glm::vec2>& m_tex_coords,
                             const std::vector<GLuint>& m_indices,
                             const std::vector<const std::vector<glm::vec3>* >& m_normals)
							 : Shape(std::vector<glm::vec3>(), m_tex_coords, m_indices, std::vector<glm::vec3>()), m_vertices_(m_vertices), m_normals_(m_normals), animations_(animations)
{

}

void AnimatedShape::addAnimation(const std::string& name, float time, unsigned int start_keyframe, unsigned int end_keyframe)
{
	animations_.push_back(Animation(name, time, start_keyframe, end_keyframe));
}

