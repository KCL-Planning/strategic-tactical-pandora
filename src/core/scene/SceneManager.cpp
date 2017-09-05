#ifdef _WIN32
#include <Windows.h>
#endif

#include <algorithm>

#include <glm/gtx/quaternion.hpp>

#include "SceneManager.h"
#include "SceneLeaf.h"
#include "SceneNode.h"
#include "../renderer/Renderer.h"
#include "../entities/Entity.h"
#include "../collision/CollisionInfo.h"
#include "../entities/camera/Camera.h"
#include "Material.h"

// Debug.
#include "SceneLeafModel.h"
#include "../shaders/LineShader.h"
#include "../../shapes/Line.h"

SceneManager::SceneManager()
{
	glm::mat4 local_transformation(1.0f);
	root_ = new SceneNode(*this, NULL, local_transformation, false);
	
	dummy_node_ = new SceneNode(*this, NULL, glm::mat4(1.0f), false);
	dummy_entity_ = new Entity(*this, dummy_node_, glm::mat4(1.0f), PASSABLE, "dummy");
	
	line_ = new Line();
	
	MaterialLightProperty* ambient = new MaterialLightProperty(0, 0, 0, 0);
	MaterialLightProperty* diffuse = new MaterialLightProperty(0, 0, 0, 0);
	MaterialLightProperty* specular = new MaterialLightProperty(0, 0, 0, 0);
	MaterialLightProperty* emmisive = new MaterialLightProperty(1, 1, 0, 0.8f);
	Material* material = new Material(*ambient, *diffuse, *specular, *emmisive);
	
	SceneLeafModel* path_ = new SceneLeafModel(*root_, NULL, *line_, *material, LineShader::getShader(), true, true);
}

void SceneManager::visitLeafs(Renderer& renderer) const
{
	std::vector<SceneNode*> open_list;
	open_list.push_back(root_);

	while (open_list.size() > 0)
	{
		SceneNode* node = open_list[open_list.size() - 1];
		open_list.erase(open_list.end() - 1);

		open_list.insert(open_list.end(), node->getChildren().begin(), node->getChildren().end());
		for (std::vector<SceneLeaf*>::const_iterator ci = node->getLeafs().begin(); ci != node->getLeafs().end(); ++ci)
		{
			(*ci)->accept(renderer);
		}
/*
		for (SceneLeaf* leaf : node->getLeafs())
		{
			leaf->accept(renderer);
		}
*/
	}
}

void SceneManager::addUpdateableEntity(Entity& entity)
{
	entities_to_add_or_remove_.push_back(std::make_pair(&entity, true));
}

void SceneManager::removeUpdateableEntity(Entity& entity)
{
	entities_to_add_or_remove_.push_back(std::make_pair(&entity, false));
}

Entity* SceneManager::pickEntity(const Camera& camera, float mouse_x, float mouse_y) const
{
	// Transform the viewport coordinates to the normalised devise coordinates.
	float ndc_mouse_x = (2 * mouse_x) / camera.getWidth() - 1.0f;
	float ndc_mouse_y = 1.0f - (2 * mouse_y) / camera.getHeight();
	
	// Transform the normalised device coordinates into homogeneous clip coordinates.
	glm::vec4 hcc_ray(ndc_mouse_x, ndc_mouse_y, -1.0f, 1.0f);
	
	// Transform these into eye coordinates.
	glm::vec4 eye_ray = glm::inverse(camera.getPerspectiveMatrix()) * hcc_ray;
	eye_ray.z = -1.0f;
	eye_ray.w = 0.0f;
	
	// Transform these into world coordinates.
	glm::vec3 world_coordinates = glm::vec3(glm::inverse(camera.getViewMatrix()) * eye_ray);
	glm::vec3 direction = glm::normalize(world_coordinates);
	
	/// Debug.
	std::vector<glm::vec3> line_points;
	line_points.push_back(camera.getGlobalLocation());
	line_points.push_back(camera.getGlobalLocation() + glm::vec3(direction) * 60.0f);
	line_->setVertexBuffer(line_points);
	
	// Get the collisions.
	std::vector<CollisionInfo> collisions;
	if (!root_->getCollisions(*dummy_entity_, camera.getGlobalLocation(), camera.getGlobalLocation() + glm::vec3(direction) * camera.getFarPlane(), collisions))
	{
		return NULL;
	}
	
	Entity* closest_entity = NULL;
	float min_distance = std::numeric_limits<float>::max();
	for (std::vector<CollisionInfo>::const_iterator ci = collisions.begin(); ci != collisions.end(); ++ci)
	{
		const CollisionInfo& collision_info = *ci;
		Entity* e = collision_info.colliding_entity_ == NULL ? collision_info.other_colliding_entity_ : collision_info.colliding_entity_;
		for (std::vector<glm::vec3>::const_iterator ci = collision_info.collision_loc_.begin(); ci != collision_info.collision_loc_.end(); ++ci)
		{
			const glm::vec3& l = *ci;
			float d = glm::distance(l, camera.getGlobalLocation());
			if (d < min_distance)
			{
				min_distance = d;
				closest_entity = e;
			}
		}
	}
	return closest_entity;
}

void SceneManager::prepare(float dt)
{
	// Add / remove entities.
	for (std::vector<std::pair<Entity*, bool> >::const_iterator ci = entities_to_add_or_remove_.begin(); ci != entities_to_add_or_remove_.end(); ++ci)
	{
		Entity* entity = (*ci).first;
		std::vector<Entity*>::iterator entity_i = std::find(entities_to_update_.begin(), entities_to_update_.end(), entity);

		// Add an entity.
		if ((*ci).second)
		{
			if (entity_i == entities_to_update_.end())
			{
				entities_to_update_.push_back(entity);
			}
		}

		// Remove an entity.
		else
		{
			if (entity_i != entities_to_update_.end())
			{
				entities_to_update_.erase(entity_i);
			}
		}
	}
	entities_to_add_or_remove_.clear();

	for (std::vector<Entity*>::iterator i = entities_to_update_.begin(); i != entities_to_update_.end(); ++i)
	{
		(*i)->mark(true, true);
	}

	root_->prepare(dt);
}

void SceneManager::addPlayer(SceneNode& player)
{
	players_.push_back(&player);
}
