#ifndef CORE_MODELS_ANIMATED_MODEL_H
#define CORE_MODELS_ANIMATED_MODEL_H

#include <GL/glew.h>

#include "../../shapes/Shape.h"

class Animation;
class Node;
class Shape;
class BoneNode;
class Bone;

class AnimatedModel : public Shape
{
public:
	AnimatedModel();
	
	AnimatedModel(const std::vector<glm::vec3>& m_vertices,
	              const std::vector<glm::vec2>& m_tex_coords,
	              const std::vector<GLuint>& m_indices,
	              const std::vector<glm::vec3>& m_normals, 
	              BoneNode& root_node,
	              const std::vector<Animation*>& animations,
	              const std::vector<Bone*>& bones);

	virtual void prepare(float dt);

	/**
	 * Set the active animation. This will update the translations, scaling, and rotations of the bones.
	 * @param animation The animation that will become the active animation.
	 */
	void setAnimation(Animation& animation);
	
	void setRootNode(BoneNode& bone_node) { root_node_ = &bone_node; }

	const std::vector<Animation*>& getAnimations() const { return animations_; }
	std::vector<Animation*>& getAnimations() { return animations_; }

	float getAnimationTime() const { return animation_time_; }
	float getAnimationDuration() const { return animation_duration_; }
	const Animation* getCurrentAnimation() const { return current_animation_; }
	
	void finalise();

private:

	//void updateNodeHierarchy(float animation_duration, const BoneNode& root_node_, const glm::mat4& transform);

	BoneNode* root_node_;
	std::vector<Animation*> animations_;
	//std::vector<Bone*> bones_;

	Animation* current_animation_;
	float animation_time_, animation_duration_;

	//glm::mat4 inverse_root_transform_;
};

#endif
