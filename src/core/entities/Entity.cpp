#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <algorithm>

#include "Entity.h"

#include "GL/glew.h"
#include "../entities/camera/Camera.h"
#include "../collision/BoxCollision.h"
#include "../scene/SceneNode.h"
#include"../../shapes/Cube.h"
#include "../../shapes/Line.h"
#include "../scene/SceneLeafModel.h"
#include "behaviours/Behaviour.h"
#include "../collision/CollisionInfo.h"
#include "../scene/portal/Region.h"
#include "../scene/portal/Portal.h"
#include "../scene/SceneManager.h"
#include "../shaders/LineShader.h"

Entity::Entity(SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transformation, ENTITY_TYPE type, const std::string& name, bool init_children)
	: SceneNode(scene_manager, parent, transformation, init_children), is_alive_(true), type_(type), name_(name)//, region_(NULL)
{
	
}

Entity::~Entity()
{
	for (std::vector<BoxCollision*>::const_iterator ci = collisions_.begin(); ci != collisions_.end(); ++ci)
	{
		delete *ci;
	}
}

void Entity::getBoundedCollisionBox(const glm::mat4& transform_matrix, float& min_x, float& max_x, float& min_y, float& max_y, float& min_z, float& max_z) const
{
	for (std::vector<BoxCollision*>::const_iterator ci = getCollisions().begin(); ci != getCollisions().end(); ++ci)
	{
		const BoxCollision* box = *ci;
		for (unsigned int i = 0; i < 8; ++i)
		{
			glm::vec4 point = transform_matrix * glm::vec4(box->getPoints()[i], 1.0);
#ifndef _WIN32
			min_x = std::min(min_x, point.x);
			max_x = std::max(max_x, point.x);
			min_y = std::min(min_y, point.y);
			max_y = std::max(max_y, point.y);
			min_z = std::min(min_z, point.z);
			max_z = std::max(max_z, point.z);
#else
			min_x = min(min_x, point.x);
			max_x = max(max_x, point.x);
			min_y = min(min_y, point.y);
			max_y = max(max_y, point.y);
			min_z = min(min_z, point.z);
			max_z = max(max_z, point.z);
#endif
		}
	}
}

void Entity::prepare(float dt)
{
	if (!is_alive_)
	{
		scene_manager_->removeUpdateableEntity(*this);
		parent_->removeChild(*this);
		return;
	}

	// Update the collision detection in regions.
	updateTransformations();
	if (transformationIsUpdated())
	{
		if (current_region_ != NULL)
		{
			current_region_ = current_region_->findRegion(getGlobalLocation());
		}
		else
		{
			current_region_ = Region::findRegionGlobal(getGlobalLocation());
		}
#ifdef _WIN32
		std::stringstream ss;
		ss << "Update the region stuff for " << getName() << std::endl;
		OutputDebugString(ss.str().c_str());
#endif
		Region::updateCollidableEntity(*this);
	}

	SceneNode::prepare(dt);
}

void Entity::addCollision(BoxCollision& collision)
{
	collisions_.push_back(&collision);
}

void Entity::addCollision(BoxCollision& collision, Material& material, ShaderInterface& shader)
{
	collisions_.push_back(&collision);
	
	// To visualise the bounding box, we add new leafs to this node.
	glm::vec3 centre_point(0.0f, 0.0f, 0.0f);
	//std::vector<glm::vec3> points;
	//collision.getPoints(points);

	for (unsigned int i = 0; i < 8; ++i)
	//for (std::vector<glm::vec3>::const_iterator ci = points.begin(); ci != points.end(); ++ci)
	{
		centre_point += collision.getPoints()[i];
		//centre_point += *ci;
	}
	//centre_point /= points.size();
	centre_point /= 8;

	SceneNode* parent = this;
	if (centre_point != glm::vec3(0.0f, 0.0f, 0.0f))
	{
		parent = new SceneNode(*scene_manager_, this, glm::translate(glm::mat4(1.0), centre_point));
	}

#ifndef _WIN32
	float min_x = std::numeric_limits<float>::max();
	float max_x = -std::numeric_limits<float>::max();
	float min_y = std::numeric_limits<float>::max();
	float max_y = -std::numeric_limits<float>::max();
	float min_z = std::numeric_limits<float>::max();
	float max_z = -std::numeric_limits<float>::max();
#else
	float min_x = 100000;
	float max_x = -100000;
	float min_y = 100000;
	float max_y = -100000;
	float min_z = 100000;
	float max_z = -100000;
#endif

	for (unsigned int i = 0; i < 8; ++i)
	{
		if (collision.getPoints()[i].x < min_x) min_x = collision.getPoints()[i].x;
		if (collision.getPoints()[i].x > max_x) max_x = collision.getPoints()[i].x;
		if (collision.getPoints()[i].y < min_y) min_y = collision.getPoints()[i].y;
		if (collision.getPoints()[i].y > max_y) max_y = collision.getPoints()[i].y;
		if (collision.getPoints()[i].z < min_z) min_z = collision.getPoints()[i].z;
		if (collision.getPoints()[i].z > max_z) max_z = collision.getPoints()[i].z;
	}

	float box_width = max_x - min_x;
	float box_height = max_y - min_y;
	float box_depth = max_z - min_z;

	//float box_width = collision.getPoints()[0].x - collision.getPoints()[1].x;
	//float box_height = collision.getPoints()[0].y - collision.getPoints()[2].y;
	//float box_depth = collision.getPoints()[0].z - collision.getPoints()[4].z;
	//Cube* collision_cube = new Cube(std::abs(box_width), std::abs(box_height), std::abs(box_depth));
	
	///SceneLeafModel* collision_box_leaf = new SceneLeafModel(*parent, NULL, *collision_cube, material, shader, false, true, MODEL_TYPE::COLLISION);
	
	Cube* collision_cube = new Cube(collision.getPoints()[0], collision.getPoints()[1], 
	                                collision.getPoints()[2], collision.getPoints()[3], 
	                                collision.getPoints()[4], collision.getPoints()[5], 
	                                collision.getPoints()[6], collision.getPoints()[7]);
	SceneLeafModel* collision_box_leaf = new SceneLeafModel(*this, NULL, *collision_cube, material, shader, false, false, COLLISION);
	
	// Add the normals too.
	Line* normals = new Line(false);
	std::vector<glm::vec3> normal_lines;
	// Top.
	glm::vec3 top_centre = (collision.getPoints()[2] + collision.getPoints()[3] + collision.getPoints()[6] + collision.getPoints()[7]) / 4.0f;
	normal_lines.push_back(top_centre);
	normal_lines.push_back((top_centre - centre_point) + top_centre);
	
	// Bottom.
	glm::vec3 bottom_centre = (collision.getPoints()[0] + collision.getPoints()[1] + collision.getPoints()[4] + collision.getPoints()[5]) / 4.0f;
	normal_lines.push_back(bottom_centre);
	normal_lines.push_back((bottom_centre - centre_point) + bottom_centre);
	
	// Left.
	glm::vec3 left_centre = (collision.getPoints()[0] + collision.getPoints()[2] + collision.getPoints()[4] + collision.getPoints()[6]) / 4.0f;
	normal_lines.push_back(left_centre);
	normal_lines.push_back((left_centre - centre_point) + left_centre);
	
	// Right.
	glm::vec3 right_centre = (collision.getPoints()[1] + collision.getPoints()[3] + collision.getPoints()[5] + collision.getPoints()[7]) / 4.0f;
	normal_lines.push_back(right_centre);
	normal_lines.push_back((right_centre - centre_point) + right_centre);
	
	// Away.
	glm::vec3 away_centre = (collision.getPoints()[0] + collision.getPoints()[1] + collision.getPoints()[2] + collision.getPoints()[3]) / 4.0f;
	normal_lines.push_back(away_centre);
	normal_lines.push_back((away_centre - centre_point) + away_centre);
	
	// Close.
	glm::vec3 close_centre = (collision.getPoints()[4] + collision.getPoints()[5] + collision.getPoints()[6] + collision.getPoints()[7]) / 4.0f;
	normal_lines.push_back(close_centre);
	normal_lines.push_back((close_centre - centre_point) + close_centre);
	
	normals->setVertexBuffer(normal_lines);
	new SceneLeafModel(*this, NULL, *normals, material, LineShader::getShader(), false, false, COLLISION);
}

bool Entity::getCollisions(Entity& entity, std::vector<CollisionInfo>& infos) const
{
	//ss << name_ << "-";
	bool process_children = bounded_collision_box_ == NULL;
	if (!process_children)
	{
		// Check if the entity is within this node.
		for (std::vector<BoxCollision*>::const_iterator ci = entity.getCollisions().begin(); ci != entity.getCollisions().end(); ++ci)
		{
			BoxCollision* collision_box = *ci;

			//ss << "Collision box: (" << bounded_collision_box_->getPoints()[0].x << ", " << bounded_collision_box_->getPoints()[0].y << ", " << bounded_collision_box_->getPoints()[0].z << ")";
			//ss << " - (" << bounded_collision_box_->getPoints()[2].x << ", " << bounded_collision_box_->getPoints()[2].y << ", " << bounded_collision_box_->getPoints()[2].z << ")";
			//ss << " - (" << bounded_collision_box_->getPoints()[4].x << ", " << bounded_collision_box_->getPoints()[4].y << ", " << bounded_collision_box_->getPoints()[4].z << ")|";

			if (collision_box->isInside(*bounded_collision_box_))
			{
				//ss << "Is inside!|";
				process_children = true;
				break;
			}
		}
	}
	if (!process_children)
	{
		return false;
	}

	bool collision_detected = false;
	if (this != &entity && getType() == OBSTACLE)
	{
		CollisionInfo info;
		for (std::vector<BoxCollision*>::const_iterator ci = collisions_.begin(); ci != collisions_.end(); ++ci)
		{
			BoxCollision* lhs_c = *ci;
			for (std::vector<BoxCollision*>::const_iterator ci = entity.collisions_.begin(); ci != entity.collisions_.end(); ++ci)
			{
				BoxCollision* rhs_c = *ci;

				if (lhs_c->doesCollide(*rhs_c, info))
				{
					infos.push_back(info);
					collision_detected = true;
				}

				else if (rhs_c->doesCollide(*lhs_c, info))
				{
					infos.push_back(info);
					collision_detected = true;
				}
			}
		}
	}

	for (std::vector<SceneNode*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
	{
		if ((*ci)->getCollisions(entity, infos))
		{
			collision_detected = true;
		}
	}

	return collision_detected;
}

bool Entity::getCollisions(Entity& entity, const glm::vec3& begin, const glm::vec3& end, std::vector<CollisionInfo>& infos) const
{
	bool collision_detected = false;
	if (this != &entity)
	{
		for (std::vector<BoxCollision*>::const_iterator ci = collisions_.begin(); ci != collisions_.end(); ++ci)
		{
			CollisionInfo info;
			if ((*ci)->doesCollide(entity, begin, end, info))
			{
				infos.push_back(info);
				collision_detected = true;
			}
		}
	}

	for (std::vector<SceneNode*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
	{
		if ((*ci)->getCollisions(entity, begin, end, infos))
		{
			collision_detected = true;
		}
	}

	return collision_detected;
}

bool Entity::doesCollide(Entity& entity, CollisionInfo& info) const
{
	bool process_children = bounded_collision_box_ == NULL;
	if (!process_children)
	{
		// Check if the entity is within this node.
		for (std::vector<BoxCollision*>::const_iterator ci = entity.getCollisions().begin(); ci != entity.getCollisions().end(); ++ci)
		{
			BoxCollision* collision_box = *ci;

			if (collision_box->isInside(*bounded_collision_box_))
			{
				//ss << "Is inside!|";
				process_children = true;
				break;
			}
		}
	}
	if (!process_children)
	{
		return false;
	}

	bool does_collide = false;

	if (this != &entity && getType() == OBSTACLE)
	{
		for (std::vector<BoxCollision*>::const_iterator ci = collisions_.begin(); ci != collisions_.end(); ++ci)
		{
			BoxCollision* lhs_c = *ci;
			for (std::vector<BoxCollision*>::const_iterator ci = entity.collisions_.begin(); ci != entity.collisions_.end(); ++ci)
			{
				BoxCollision* rhs_c = *ci;
				
				if (lhs_c->doesCollide(*rhs_c, info))
				{
					return true;
				}
				
				if (rhs_c->doesCollide(*lhs_c, info))
				{
					return true;
				}
			}
		}
	}

	for (std::vector<SceneNode*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
	{
		if ((*ci)->doesCollide(entity, info))
		{
			return true;
		}
	}

	return false;
}

bool Entity::doesCollide(Entity& entity, const glm::vec3& begin, const glm::vec3& end, CollisionInfo& info) const
{
	if (this != &entity)
	{
		for (std::vector<BoxCollision*>::const_iterator ci = collisions_.begin(); ci != collisions_.end(); ++ci)
		{
			if ((*ci)->doesCollide(entity, begin, end, info))
			{
				return true;
			}
		}
	}

	for (std::vector<SceneNode*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
	{
		if ((*ci)->doesCollide(entity, begin, end, info))
		{
			return true;
		}
	}

	return false;
}

bool Entity::doesCollide(Entity* entity, const glm::vec3& begin, const glm::vec3& end, float effective_width) const
{
	if (this != entity)
	{
		for (std::vector<BoxCollision*>::const_iterator ci = collisions_.begin(); ci != collisions_.end(); ++ci)
		{
			if ((*ci)->doesCollide(begin, end, effective_width))
			{
				return true;
			}
		}
	}

	for (std::vector<SceneNode*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
	{
		if ((*ci)->doesCollide(entity, begin, end, effective_width))
		{
			return true;
		}
	}

	return false;
}

bool Entity::doesCollide(Entity* entity, const glm::vec3& point) const
{
	if (this != entity)
	{
		for (std::vector<BoxCollision*>::const_iterator ci = collisions_.begin(); ci != collisions_.end(); ++ci)
		{
			if ((*ci)->isInside(point))
			{
				return true;
			}
		}
	}

	for (std::vector<SceneNode*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
	{
		if ((*ci)->doesCollide(entity, point))
		{
			return true;
		}
	}

	return false;
}

/*
bool Entity::checkCollision(const Entity& other, CollisionInfo& info, std::stringstream& ss) const
{
	bool does_collide = false;
	for (std::vector<BoxCollision*>::const_iterator ci = collisions_.begin(); ci != collisions_.end(); ++ci)
	{
		BoxCollision* lhs_c = *ci;
		for (std::vector<BoxCollision*>::const_iterator ci = other.collisions_.begin(); ci != other.collisions_.end(); ++ci)
		{
			BoxCollision* rhs_c = *ci;

			if (lhs_c->doesCollide(*rhs_c, info, ss))
			{
				does_collide = true;
			}
		}
	}
	return does_collide;
}
*/

void Entity::onCollision(const std::vector<CollisionInfo>& collision_info)
{

}

void Entity::destroy()
{
	is_alive_ = false;
	mark(true, true);
}

bool Entity::activate(Entity& activator)
{
	for (std::vector<Behaviour*>::const_iterator ci = behaviours_.begin(); ci != behaviours_.end(); ++ci)
	{
		(*ci)->activate(activator);
	}
	scene_manager_->addUpdateableEntity(*this);
	return true;
}
