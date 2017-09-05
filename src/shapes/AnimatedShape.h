#ifndef SHAPES_ANIMATED_SHAPE_H
#define SHAPES_ANIMATED_SHAPE_H

#include "Shape.h"

struct Animation
{
	Animation(const std::string& name, float time, unsigned int start_keyframe, unsigned int end_keyframe)
		: name_(name), time_(time), start_keyframe_(start_keyframe), end_keyframe_(end_keyframe)
	{

	}

	std::string name_;
	float time_;
	unsigned int start_keyframe_, end_keyframe_;
};

/**
 * A shape that has multiple keypoints that can be used to be animated.
 */
class AnimatedShape : public Shape
{
public:
	AnimatedShape(const std::vector<const std::vector<glm::vec3>* >& m_vertices,
	              std::vector<Animation>& animations,
	              const std::vector<glm::vec2>& m_tex_coords,
	              const std::vector<GLuint>& m_indices,
	              const std::vector<const std::vector<glm::vec3>* >& m_normals);

	void addAnimation(const std::string& name, float time, unsigned int start_keyframe, unsigned int end_keyframe);

	const std::vector<const std::vector<glm::vec3>* >& getVertices() const { return m_vertices_; }
	const std::vector<const std::vector<glm::vec3>* >& getNormals() const { return m_normals_; }
	const std::vector<Animation>& getAnimations() const { return animations_; }

private:
	const std::vector<const std::vector<glm::vec3>* > m_vertices_;
	const std::vector<const std::vector<glm::vec3>* > m_normals_;
	std::vector<Animation> animations_;
};

#endif
