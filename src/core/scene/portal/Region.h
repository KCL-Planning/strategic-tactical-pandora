#ifndef CORE_SCENE_PORTAL_REGION_H
#define CORE_SCENE_PORTAL_REGION_H

#include <glm/glm.hpp>

#include <map>
#include <vector>
#include <sstream>
#include <string>

class Portal;
class BoundedBox;
class SceneNode;
class Renderer;
class Frustum;
class Entity;
class CollisionInfo;
class SceneManager;
class Region;
class Portal;

struct RegionPortalDebug
{
	RegionPortalDebug(const Region* region, const Portal* portal, std::string debug_str)
		: region_(region), portal_(portal), debug_str_(debug_str)
	{

	}

	const Region* region_;
	const Portal* portal_;
	std::string debug_str_;
};

/**
 * A region is an optional property for scene nodes. It is used to speed up the rendering process by culling objects that are not 
 * visible and reduce the number of objects we need to test collision agains.
 */
class Region
{
public:
	Region(SceneNode& scene_node);
	Region(SceneNode& scene_node, const std::string& name);

	~Region();

	const std::string& getName() const { return name_; }

	Portal& addPortalToOtherRegion(Region& other, const std::vector<glm::vec3>& points);

	/**
	 * Instead of frustum culling, we use the portal system to restrict the areas we need to check.
	 */
	virtual void preRender(const Frustum& frustum, const glm::vec3& camera_position, Renderer& renderer, bool process_lights, unsigned int& nr_calls, unsigned int portal_depth, std::vector<const Portal*>& processed_portals, std::stringstream& ss);
	
	// Debug function.
	//void getRegionsToRender(const Frustum& frustum, const glm::vec3& camera_position, unsigned int portal_depth, const Portal* accessed_from, std::vector<const Portal*>& processed_portals, std::vector<RegionPortalDebug>& processed) const;

	SceneNode& getSceneNode() const { return *scene_node_; }

	Region* findRegion(const glm::vec3& location);

	bool isInRegion(const glm::vec3& location) const;
	
	bool isVisibleIn(const SceneNode& node) const;

	bool getCollisions(Entity& entity, std::vector<CollisionInfo>& info) const;
	bool getCollisions(Entity& entity, const glm::vec3& begin, const glm::vec3& end, std::vector<CollisionInfo>& info) const;
	
	/**
	 * Same as getCollisions, but we stop once we found the first collision.
	 */
	bool doesCollide(Entity& entity, CollisionInfo& info) const;
	bool doesCollide(Entity& entity, const glm::vec3& begin, const glm::vec3& end, CollisionInfo& info) const;
	bool doesCollide(Entity* entity, const glm::vec3& begin, const glm::vec3& end, float effective_width) const;
	bool doesCollide(Entity* entity, const glm::vec3& point) const;

	/**
	 * Update the visibility of @ref{scene_node}, the region this node is in should already 
	 * been established. If the node is in no region then this function does nothing. If 
	 * the node is in a region then a breath-first search is executed to find all regions
	 * where the node is visible from.
	 * @param scene_node The node of which we want to update the visibility of.
	 */
	static void updateVisibility(SceneNode& scene_node);
	static std::vector<Region*>* getRegionsVisibleIn(SceneNode& scene_node);

	/**
	 * Update the set of regions where @ref{entity} could collide, the region this entity 
	 * is in should already been established. If the entity is in no region then this function 
	 * does nothing. If the entity is in a region then a breath-first search is executed to 
	 * find all regions where the entity could collide.
	 * @param entity The entity of which we want to update the regions where it can collide in.
	 */
	static void updateCollidableEntity(Entity& entity);
	static std::vector<Region*>* getRegionsCollidableIn(Entity& entity);

	static Region* findRegionGlobal(const glm::vec3& location);
	
	static const std::vector<Region*>& getAllRegions();
	
	const std::vector<SceneNode*>& getVisibleEntities() const { return entities_visible_from_different_regions_; }
	
	const std::vector<Portal*>& getPortals() const { return portals_; }
	
	// DEBUG
	void debugCreateBoundedBox(SceneManager& scene_manager);
	void getRenderingPortals(const Frustum& frustum, const glm::vec3& camera_position, std::vector<glm::vec3>& portals, std::vector<const Portal*>& processed_portals) const;

private:

	std::string name_;
	
	/**
	 * For every region we keep track of every entity that has a collision box in this region.
	 */
	void removeCollidableEntity(Entity& entity);
	void addCollidableEntity(Entity& entity);
	
	/**
	 * For every region we keep track of all the scene nodes that are visible from this region.
	 * Following two functions add / remove them.
	 */
	void removeVisibleEntityFromOtherRegion(SceneNode& node);
	void addVisibleEntityFromOtherRegion(SceneNode& node);
	/*
	bool getCollisions(Entity& entity, std::vector<CollisionInfo>& info, std::vector<const Region*>& closed_list) const;
	bool getCollisions(Entity& entity, const glm::vec3& begin, const glm::vec3& end, std::vector<CollisionInfo>& info, std::vector<const Region*>& closed_list) const;

	bool doesCollide(Entity& entity, CollisionInfo& info, std::vector<const Region*>& closed_list) const;
	bool doesCollide(Entity& entity, const glm::vec3& begin, const glm::vec3& end, CollisionInfo& info, std::vector<const Region*>& closed_list) const;
	bool doesCollide(const glm::vec3& begin, const glm::vec3& end, float effective_width, std::vector<const Region*>& closed_list) const;
	bool doesCollide(const glm::vec3& point, std::vector<const Region*>& closed_list) const;
	*/
	Region* findRegion(const glm::vec3& location, std::vector<const Region*>& closed_list);

	// The list of scene nodes that are visible in this region.
	std::vector<SceneNode*> entities_visible_from_different_regions_;

	// Mapping form the scene nodes to the set of regions that they are visible in.
	static std::map<SceneNode*, std::vector<Region*>* > scene_node_regions_;

	// The list of entities that are collidable in this region.
	std::vector<Entity*> collidable_entities_;

	// Mapping form the entities to the set of regions that they are collidable in.
	static std::map<Entity*, std::vector<Region*>* > entity_collision_regions_;

	static std::vector<Region*> all_regions_;

	SceneNode* scene_node_;
	std::vector<Portal*> portals_;
};

#endif
