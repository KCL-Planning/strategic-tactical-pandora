#ifndef SCENE_MANAGER_H
#define SCENE_MANAGER_H

#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp> 

#include "../shaders/ShaderInterface.h"

class Camera;
class Light;
class LightManager;
class Material;
class SceneNode;
class GLSLProgram;
class Renderer;
class SceneLeafLight;
class Shape;
class Entity;
class Line;

/**
 * In order to render a scene we use a scene manager, each node can represent a rotation / transformation and the leaves
 * represent the entities that must be rendered.
 */
class SceneManager
{
public:
	SceneManager();
	SceneNode& getRoot() const { return *root_; }
	void setRoot(SceneNode& root);

	void visitLeafs(Renderer& renderer) const;

	void addPlayer(SceneNode& player);

	void addUpdateableEntity(Entity& entity);
	void removeUpdateableEntity(Entity& entity);

	/**
	 * Find an entity that we click on on the screen. We use the collision boxes to detect the closest entity.
	 */
	Entity* pickEntity(const Camera& camera, float mouse_x, float mouse_y) const;

	/**
	 * Update all the entities that need to be updated and traverse the tree graph and update any scene nodes 
	 * that need to be updated as a consequence of that.
	 * @param dt The time that has elapsed since the last call.
	 */
	void prepare(float dt);

	// Debug, for now.
	const std::vector<SceneNode*>& getPlayers() const { return players_; }
private:
	SceneNode* root_;

	// Debug node so we can add a dummy entity for the pick entity function.
	// Note: We might have to change the function so you can pass NULL as the entity parameter.
	SceneNode* dummy_node_;
	Entity* dummy_entity_;

	// Instead of pre processing all the entities that need to be upgraded.
	std::vector<Entity*> entities_to_update_;
	
	// Update the entities -- in order. True means add, false means remove.
	std::vector<std::pair<Entity*, bool> > entities_to_add_or_remove_;
	
	// Debug, for now.
	std::vector<SceneNode*> players_;
	
	Line* line_;
};

#endif
