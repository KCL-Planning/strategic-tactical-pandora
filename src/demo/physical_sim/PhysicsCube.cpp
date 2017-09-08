#include "PhysicsCube.h"

#include <vector>
#include <limits>
 
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtc/matrix_transform.hpp> 
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include "../../core/scene/SceneNode.h"
#include "../../core/scene/SceneLeafModel.h"
#include "../../core/scene/SceneManager.h"
#include "../../core/collision/CollisionInfo.h"
#include "../../core/collision/ConvexPolygon.h"
#include "../../core/scene/portal/Region.h"
#include "../../core/scene/Material.h"
#include "../../core/shaders/BasicShadowShader.h"
#include "../../shapes/Cube.h"
#include "../../core/texture/TargaTexture.h"

PhysicsCube::PhysicsCube(SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transformation)
	: Entity(scene_manager, parent, transformation, OBSTACLE, "Physics Cube")
{
	// We sneak in a special scene node that takes care of the rotation correction between this node and the parent node.
	
	// Set up the physical properties.
	y_velocity_ = 0.0f;
	ConvexPolygon* bc = new ConvexPolygon(*this, 1.0f, 1.0f, 1.0f);
	addCollision(*bc);
	height_ = 1.0f;
	
	Texture* grass_texture = TargaTexture::loadTexture("data/textures/grass.tga");
	Texture* height_texture = TargaTexture::loadTexture("data/textures/height.tga");
	
	MaterialLightProperty* bright_ambient = new MaterialLightProperty(0.0, 0.0, 0.0, 1.0);
	MaterialLightProperty* bright_diffuse = new MaterialLightProperty(0.0, 0.0, 0.0, 1.0);
	MaterialLightProperty* bright_specular = new MaterialLightProperty(0.0, 0.0, 0.0, 1.0);
	MaterialLightProperty* bright_emmisive = new MaterialLightProperty(1.0, 0.2, 1.0, 1.0);

	Material* material = new Material(*bright_ambient, *bright_diffuse, *bright_specular, *bright_emmisive);
	material->add2DTexture(*grass_texture);
	
	Cube* cube = new Cube(1, 1, 1);
	SceneLeafModel* cube_slm = new SceneLeafModel(*this, NULL, *cube, *material, BasicShadowShader::getShader(), false, false);
	
	local_transformation_sans_rotation_ = local_transformation_;
	rotation_bias_ = glm::mat4(1.0f);
}
	
void PhysicsCube::prepare(float dt)
{
	//std::cout << "Location! (" << getGlobalLocation().x << ", " << getGlobalLocation().y << ", " << getGlobalLocation().z << ")" << std::endl;
	glm::mat4 current_transformation = local_transformation_sans_rotation_;
	
	// Lets subject the player to some gravity if we are not on a surface.
	y_velocity_ += dt;
	if (y_velocity_ > 0.5f) y_velocity_ = 0.5f;
	
	local_transformation_sans_rotation_ = glm::translate(local_transformation_sans_rotation_, glm::vec3(0, -y_velocity_, 0));
	local_transformation_ = local_transformation_sans_rotation_ * rotation_bias_;
	updateTransformations();

	bool collided_on_top = false;

	std::vector<CollisionInfo> collision_infos;
	glm::fquat parent_rotation = parent_->getGlobalRotation();

	// Localise the player.
	if (current_region_ == NULL)
	{
		current_region_ = Region::findRegionGlobal(getGlobalLocation());
	}
	bool found_collision = false;

	if (current_region_ != NULL)
	{
		found_collision = current_region_->getCollisions(*this, collision_infos);
	}
	else
	{
		found_collision = scene_manager_->getRoot().getCollisions(*this, collision_infos);
	}
	
	if (found_collision)
	{
		CollisionInfo highest_found_collision;
		{
			float highest_collision = -std::numeric_limits<float>::max();
			glm::vec3 collision_point;

			for (std::vector<CollisionInfo>::const_iterator ci = collision_infos.begin(); ci != collision_infos.end(); ++ci)
			{
				const CollisionInfo& collision_info = *ci;
				for (std::vector<glm::vec3>::const_iterator ci = collision_info.collision_loc_.begin(); ci != collision_info.collision_loc_.end(); ++ci)
				{
					Entity* colliding_entity = collision_info.colliding_entity_;
					if (colliding_entity == this)
					{
						colliding_entity = collision_info.other_colliding_entity_;
					}

					// Transform the collision location so it corresponds to the local location.
					glm::vec3 local_collision = *ci;

					///local_collision = glm::rotate(glm::inverse(glm::quat_cast(colliding_entity->getCompleteTransformation())), local_collision);
				
					if (local_collision.y > highest_collision)
					{
						highest_collision = local_collision.y;
						collision_point = *ci;
						highest_found_collision = collision_info;
					}
				}
			}
		}

		const CollisionInfo& collision_info = highest_found_collision;//collision_infos[collision_infos.size() - 1];
		Entity* colliding_entity = collision_info.colliding_entity_;
		if (colliding_entity == this)
		{
			colliding_entity = collision_info.other_colliding_entity_;
		}

		float highest_collision = -std::numeric_limits<float>::max();
		glm::vec3 collision_point;

		for (std::vector<glm::vec3>::const_iterator ci = collision_info.collision_loc_.begin(); ci != collision_info.collision_loc_.end(); ++ci)
		{
			//std::cout << "Collision point: (" << (*ci).x << ", " << (*ci).y << ", " << (*ci).z << ")" << std::endl;
				
				
			if ((*ci).y > highest_collision)
			{
				highest_collision = (*ci).y;
				collision_point = *ci;
			}
		}

		float height_difference = std::abs(complete_transformation_[3][1] - (collision_point.y + height_ / 2.0f));

		CollisionInfo higher_collision_info;
		Region* tmp_region = NULL;
		if (current_region_ != NULL)
		{
			tmp_region = current_region_->findRegion(getGlobalLocation());
		}

		if (height_difference > 0.35f)// || 
		   //(tmp_region == NULL && scene_manager_->getRoot().doesCollide(*this, higher_collision_info)) ||
		   //(tmp_region != NULL && tmp_region->doesCollide(*this, higher_collision_info)))
		{
			local_transformation_sans_rotation_ = current_transformation;
			local_transformation_ = local_transformation_sans_rotation_ * rotation_bias_;
		}
		else
		{
			// If the colliding entity is not the one we are colliding with at the moment then we need to change it!
			glm::vec4 transformed_collision_point = glm::inverse(parent_->getCompleteTransformation()) * glm::vec4(collision_point, 1.0f);
			
			///std::cout << "Transformed collision: (" << transformed_collision_point.x << ", " << transformed_collision_point.y << ", " << transformed_collision_point.z << ")" << std::endl;
			local_transformation_sans_rotation_[3][1] = transformed_collision_point.y + height_ / 2.0f;
			local_transformation_ = local_transformation_sans_rotation_ * rotation_bias_;
			updateTransformations();
			//std::cout << "Updated the global location to: (" << getGlobalLocation().x << ", " << getGlobalLocation().y << ", " << getGlobalLocation().z << ")" << std::endl;
			transition(colliding_entity);
			updateTransformations();
			collided_on_top = true;
		}
		y_velocity_ = 0.0f;
	}
	//SceneNode::prepare(dt);
	Entity::prepare(dt);

	if (current_region_ != NULL)
	{
		current_region_ = current_region_->findRegion(getGlobalLocation());
	}
}
/*
void PhysicsCube::setParent(SceneNode* new_parent)
{
	transition(new_parent);
	return;
	if (parent_ != NULL)
	{
		parent_->removeChild(*this);
	}
	
	parent_ = new_parent;
	if (parent_ != NULL)
	{
		parent_->addChild(*this);
		rotation_bias_ = glm::mat4_cast(glm::inverse(new_parent->getGlobalRotation()));
		//rotation_bias_->setTransformation(glm::mat4_cast(glm::inverse(new_parent->getGlobalRotation())));
	}
	
	mark(true, true);
	updateTransformations();
	//complete_transformation_ = new_parent->getCompleteTransformation() * rotation_bias_->getLocalTransformation() * getLocalTransformation();
	//std::cout << "Updated the local location to: (" << getLocalLocation().x << ", " << getLocalLocation().y << ", " << getLocalLocation().z << ")" << std::endl;
	//std::cout << "Updated the global location to: (" << getGlobalLocation().x << ", " << getGlobalLocation().y << ", " << getGlobalLocation().z << ")" << std::endl;
}
*/
void PhysicsCube::transition(SceneNode* scene_node)
{
	// If nothing changes then we are done!
	if (parent_ == scene_node)
	{
		return;
	}
	
	parent_->removeChild(*this);
	std::cout << "set parent: " << scene_node << ". Old: " << parent_ << std::endl;
	
	std::cout << "Old transition! (" << parent_->getGlobalLocation().x << ", " << parent_->getGlobalLocation().y << ", " << parent_->getGlobalLocation().z << ")" << std::endl;
	std::cout << "New transition! (" << scene_node->getGlobalLocation().x << ", " << scene_node->getGlobalLocation().y << ", " << scene_node->getGlobalLocation().z << ")" << std::endl;
	
	// Get the global location of the player.
	glm::vec4 global_location = glm::vec4(getGlobalLocation(), 1.0f);
	std::cout << "Global locaion: (" << global_location.x << ", " << global_location.y << ", " << global_location.z << ")" << std::endl;
	
	// Calculate the local location of the player relative to its new parent.
	global_location = glm::inverse(scene_node->getCompleteTransformation()) * global_location;
	local_transformation_sans_rotation_[3][0] = global_location.x;
	local_transformation_sans_rotation_[3][1] = global_location.y;
	local_transformation_sans_rotation_[3][2] = global_location.z;
	
	// We keep part of the rotation that happened while the cube was on a rotating object.
	local_transformation_sans_rotation_ = local_transformation_sans_rotation_ * (glm::mat4_cast(parent_->getGlobalRotation()) * rotation_bias_);
	
	std::cout << "Transition! (" << global_location.x << ", " << global_location.y << ", " << global_location.z << ")" << std::endl;
	global_location = scene_node->getCompleteTransformation() * global_location;
	std::cout << "Correct location(?) (" << global_location.x << ", " << global_location.y << ", " << global_location.z << ")" << std::endl;
	
	// Undo the rotation of this node to preserve the cube's orientation.
	rotation_bias_ = glm::mat4_cast(glm::inverse(scene_node->getGlobalRotation()));
	local_transformation_ = local_transformation_sans_rotation_ * rotation_bias_;
	
	// Store the scene node we are on at the moment.
	parent_ = scene_node;
	parent_->addChild(*this);
	//rotation_bias_->setParent(scene_node);
	
	//rotation_bias_->updateTransformations();
	updateTransformations();
	std::cout << "Final transition! (" << getGlobalLocation().x << ", " << getGlobalLocation().y << ", " << getGlobalLocation().z << ")" << std::endl;
}
