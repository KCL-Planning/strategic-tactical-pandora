#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include "BoneNode.h"
#include "../scene/SceneNode.h"
#include "Bone.h"
#include "AnimationNode.h"
#include "Animation.h"

#ifdef _WIN32
#include <windows.h>
#endif

BoneNode::BoneNode(SceneManager& scene_manager, const std::string& name, const glm::mat4& transformation, BoneNode* parent, Bone* bone)
	: SceneNode(scene_manager, parent, transformation, false), name_(name), bone_(bone), root_node_(NULL), active_animation_node_(NULL)
{
	if (parent != NULL)
	{
		parent->children_.push_back(this);
		root_node_ = parent;
		while (root_node_->getParent() != NULL)
		{
			root_node_ = root_node_->getParent();
		}
	}
	inverse_root_initialised_ = false;
	default_translate_initialised_ = false;
	default_rotate_initialised_ = false;
	default_scale_initialised_ = false;
}

void BoneNode::prepare(float animation_duration)
{
	// We first calculate the local transformation of the bone.
	glm::mat4 node_transformation = getLocalTransformation();
	AnimationNode* animation_node = getAnimationNode();

	// Update the transformation of this bone if an animation node is attached to this bone.
	if (animation_node != NULL)
	{
		glm::mat4 scaling_matrix(1.0f);
		if (animation_node->getScalings().size() > 1)
		{
			for (std::vector<std::pair<glm::vec3, float> >::const_iterator ci = animation_node->getScalings().begin(); ci != animation_node->getScalings().end(); ++ci)
			{
				const std::pair<glm::vec3, float>& current_scaling = *ci;
				std::pair<glm::vec3, float> next_scaling = ci + 1 == animation_node->getScalings().end() ? std::make_pair((*animation_node->getScalings().begin()).first, active_animation_node_->getAnimation().getDuration()) : *(ci + 1);
				
				if (animation_duration < next_scaling.second && animation_duration >= current_scaling.second)
				{
					float delta = (animation_duration - current_scaling.second) / (next_scaling.second - current_scaling.second);
					scaling_matrix = glm::scale(scaling_matrix, current_scaling.first * (1.0f - delta) + next_scaling.first * delta);
					break;
				}
			}
		}
		else if (animation_node->getScalings().size() == 1)
		{
			if (!default_scale_initialised_)
			{
				default_scale_ = glm::scale(scaling_matrix, animation_node->getScalings()[0].first);
				default_scale_initialised_ = true;
			}
			scaling_matrix = default_scale_;
		}

		glm::mat4 rotation_matrix(1.0f);
		if (animation_node->getRotations().size() > 1)
		{
			for (std::vector<std::pair<glm::fquat, float> >::const_iterator ci = animation_node->getRotations().begin(); ci != animation_node->getRotations().end(); ++ci)
			{
				const std::pair<glm::fquat, float>& current_rotation = *ci;
				//std::pair<glm::fquat, float> next_rotation = ci + 1 == animation_node->getRotations().end() ? std::make_pair((*animation_node->getRotations().begin()).first, active_animation_node_->getAnimation().getDuration()) : *(ci + 1);
				float rot_time = ci + 1 == animation_node->getRotations().end() ? active_animation_node_->getAnimation().getDuration() : (*(ci + 1)).second;

				//if (animation_duration < next_rotation.second && animation_duration >= current_rotation.second)
				if (animation_duration < rot_time && animation_duration >= current_rotation.second)
				{
					//float delta = (animation_duration - current_rotation.second) / (next_rotation.second - current_rotation.second);
					float delta = (animation_duration - current_rotation.second) / (rot_time - current_rotation.second);
					//glm::fquat quat_rot = static_cast<glm::fquat(*)(const glm::fquat&, const glm::fquat&, const float&)>(&glm::slerp)(current_rotation.first, next_rotation.first, delta);
					
					//glm::fquat quat_rot = static_cast<glm::fquat(*)(const glm::fquat&, const glm::fquat&, const float&)>(&glm::slerp)(current_rotation.first, rot, delta);
					//rotation_matrix = glm::mat4_cast(quat_rot);

					glm::fquat rot = ci + 1 == animation_node->getRotations().end() ? (*animation_node->getRotations().begin()).first : (*(ci + 1)).first;
					rotation_matrix = glm::mat4_cast(static_cast<glm::fquat(*)(const glm::fquat&, const glm::fquat&, const float&)>(&glm::slerp)(current_rotation.first, rot, delta));
					break;
				}
			}
		}
		else if (animation_node->getRotations().size() == 1)
		{
			if (!default_rotate_initialised_)
			{
				default_rotate_ = glm::mat4_cast(animation_node->getRotations()[0].first);
				default_rotate_initialised_ = true;
			}
			rotation_matrix = default_rotate_;
		}

		glm::mat4 translate_matrix(1.0f);
		if (animation_node->getPositions().size() > 1)
		{
			for (std::vector<std::pair<glm::vec3, float> >::const_iterator ci = animation_node->getPositions().begin(); ci != animation_node->getPositions().end(); ++ci)
			{

				const std::pair<glm::vec3, float>& current_position = *ci;
				std::pair<glm::vec3, float> next_position = ci + 1 == animation_node->getPositions().end() ? std::make_pair((*animation_node->getPositions().begin()).first, active_animation_node_->getAnimation().getDuration()) : *(ci + 1);
				
				if (animation_duration < next_position.second && animation_duration >= current_position.second)
				{
					float delta = (animation_duration - current_position.second) / (next_position.second - current_position.second);
					translate_matrix = glm::translate(translate_matrix, current_position.first * (1.0f - delta) + next_position.first * delta);
					break;
				}
			}
		}
		else if (animation_node->getPositions().size() == 1)
		{
			if (!default_translate_initialised_)
			{
				default_translate_ = glm::translate(translate_matrix, animation_node->getPositions()[0].first);
				default_translate_initialised_ = true;
			}
			translate_matrix = default_translate_;
		}

		node_transformation = translate_matrix * rotation_matrix * scaling_matrix;
	}

	if (parent_ != NULL)
	{
		complete_transformation_ = parent_->getCompleteTransformation() * node_transformation;
	}
	else
	{
		complete_transformation_ = node_transformation;
	}

	// If this node contains a bone, then we need to update the position of the bone as follows.
	// We first calculate the bone's offset relative to either the root bone, or the offset 
	// relative to the parent bone (if any). Then we transform this local transformation
	// to the 'global bone space' relative to the root. We then finaly transform from the 
	// 'global bone space' to the 'model space'.
	if (getBone() != NULL)
	{
		if (!inverse_root_initialised_)
		{
			inverse_root_ = glm::inverse(root_node_->getLocalTransformation());
			inverse_root_initialised_ = true;
		}
		getBone()->setFinalTransformation(inverse_root_ * complete_transformation_ * getBone()->getOffsetMatrix());
		//getBone()->setFinalTransformation(glm::inverse(root_node_->getLocalTransformation()) * complete_transformation_ * getBone()->getOffsetMatrix());
	}

	// Update all the children of this node in the same fashion.
	for (std::vector<SceneNode*>::const_iterator ci = getChildren().begin(); ci != getChildren().end(); ++ci)
	{
		static_cast<BoneNode*>((*ci))->prepare(animation_duration);
	}
}

void BoneNode::setActiveAnimationNode(AnimationNode* animation_node)
{
	active_animation_node_ = animation_node;
}
