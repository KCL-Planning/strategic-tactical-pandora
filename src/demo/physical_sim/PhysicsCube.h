#ifndef DEMO_PHYSICAL_SIM_PHYSICS_CUBE_H
#define DEMO_PHYSICAL_SIM_PHYSICS_CUBE_H

#include <glm/glm.hpp>

#include "../../core/entities/Entity.h"

class BoxCollision;
class SceneNode;
class SceneManager;

class PhysicsCube : public Entity
{
public:
	PhysicsCube(SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transformation);
	
	void prepare(float dt);
	void transition(SceneNode* scene_node);
private:
	float y_velocity_;  // The velocity of the object downards (gravity).
	float height_;  // The 'height' of the object, used to offset collisions such that the colliding
	                // surfaces match.
	
	glm::mat4 rotation_bias_;  // When we become part of a new parent, we ignore the rotation that 
	                           // parent already has. When we become of another parent, we undo this
	                           // rotation.
	glm::mat4 local_transformation_sans_rotation_; // This is the local transformation of this entity
	                                               // where the rotation that is part of rotation_bias_
	                                               // is ignored. We do incur rotations that happen while
	                                               // this entity is part of its parent.
};

#endif
