#ifndef CORE_MODELS_NODE_H
#define CORE_MODELS_NODE_H

#include <vector>
#include <glm/glm.hpp>
#include <glm/ext.hpp>

#include "../scene/SceneNode.h"

class Bone;
class AnimationNode;
class SceneManager;

class BoneNode : public SceneNode
{
public:

	BoneNode(SceneManager& scene_manager, const std::string& name, const glm::mat4& transformation, BoneNode* parent, Bone* bone);

	const std::string& getName() const { return name_; }

	virtual void prepare(float dt);

	//const glm::mat4& getTransformation() const { return transformation_; }

	//BoneNode* getParent() const { return parent_; }

	Bone* getBone() const { return bone_; }

	//const std::vector<BoneNode*>& getChildren() const { return children_; }

	void setActiveAnimationNode(AnimationNode* active_animation_node);

	AnimationNode* getAnimationNode() const { return active_animation_node_; }

private:
	std::string name_;
	//glm::mat4 transformation_;
	//BoneNode* parent_;
	Bone* bone_;

	SceneNode* root_node_;
	bool inverse_root_initialised_;
	glm::mat4 inverse_root_;
	bool default_translate_initialised_, default_rotate_initialised_, default_scale_initialised_;
	glm::mat4 default_translate_, default_rotate_, default_scale_;

	//std::vector<BoneNode*> children_;

	AnimationNode* active_animation_node_;
};

#endif
