#ifndef CORE_SCENE_SCENE_NODE_H
#define CORE_SCENE_SCENE_NODE_H

#include <vector>
#include <sstream>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class Entity;
class BoxCollision;
class Behaviour;
class Renderer;
class SceneLeaf;
class Frustum;
class InFrustumCheck;
class Material;
class CollisionInfo;
class Plane;
class BoundedBox;
class Material;
class Region;
class SceneManager;

/**
 * The scene graph is constructed of scene nodes which contain the transformation graph which is applied to
 * all its children which can either be other scene nodes or scene leafs. The tree structure allows for 
 * efficient computations of collision detection and frustum culling because once we prove that a scene 
 * node cannot collide or does not intersect with the frustum we can ignore all its children too.
 */
class SceneNode
{
public:
	SceneNode(SceneManager& scene_manager, SceneNode* parent, const glm::mat4& local_transformation, bool init_children = true);
	virtual ~SceneNode();

	// Disable copy constructor.
	SceneNode(const SceneNode& other);
	void operator=(const SceneNode& other);
	
	/**
	 * Update the transitions of the scene node and all its children.
	 * @param dt The time that has elapsed since the last call.
	 */
	virtual void prepare(float dt);

	/**
	 * Every tick we mark scene nodes for updates. Only those that are marked will
	 * be updated, the rest remains static.
	 */
	void mark(bool update, bool visit);

	/**
	 * Prepare the textures and other entities before actually rendering them. All the entities that can
	 * be rendered will be passed to the renderer.
	 * @param frustum The frustum which we use to cull against.
	 * @param view_matrix The view matrix.
	 * @param renderer The renderer which will render the scene.
	 * @param process_lights Process light leafs if true, ignore when false.
	 * @param nr_calls The number of times the pre render is called.
	 */
	virtual void preRender(const Frustum& frustum, const glm::vec3& camera_position, Renderer& renderer, bool process_lights, unsigned int& nr_calls);

	//virtual Region* findRegion(const glm::vec3& point);
	
	virtual void addLeaf(SceneLeaf& leaf);
	virtual void removeLeaf(SceneLeaf& leaf);

	const std::vector<SceneNode*>& getChildren() const { return children_; }
	const std::vector<SceneLeaf*>& getLeafs() const { return leafs_; }

	const glm::mat4& getLocalTransformation() const { return local_transformation_; }
	void setTransformation(const glm::mat4& local_transformation, bool updateBoundaries = false);

	const glm::mat4& getCompleteTransformation() const { return complete_transformation_; }
	glm::mat4& getCompleteTransformation() { return complete_transformation_; }

	glm::vec3 getGlobalLocation() const { return glm::vec3(complete_transformation_[3][0], complete_transformation_[3][1], complete_transformation_[3][2]); }
	glm::vec3 getLocalLocation() const { return glm::vec3(local_transformation_[3][0], local_transformation_[3][1], local_transformation_[3][2]); }

	glm::fquat getGlobalRotation() const { return glm::quat_cast(complete_transformation_); }
	glm::fquat getLocalRotation() const { return glm::quat_cast(local_transformation_); }

	virtual void destroy();

	/**
	 * Create a bounded box such that all the points where a collision can occur amongs its children
	 * are inside this box.
	 */
	void initialiseBoundedBoxes(const std::vector<const SceneNode*>& excluded_nodes);
	void setCollisionBoundedBox(BoundedBox& box);

	void compress();

	/**
	 * Check if the given entity resides within this scene node. If so then the collision information
	 * is collected in info. The stringstream is there for debug purposes. The scene nodes them-
	 * selves will never collide, its default behaviour is to check if the given entity is within
	 * the bounding box that contains all the children of this scene node. If that is the case then
	 * we check the entity against its children, if not we cull the collision checking.
	 *
	 * If no collision boxes are added then we assume that the entity does collide and all its
	 * children will be checked for collisions.
	 *
	 * @entity The entity we check collision against.
	 * @info A list of entities that captures any collision detected and the intersecting points.
	 * @ss Debug string.
	 * @return True if a collision was detected, false otherwise.
	 */
	virtual bool getCollisions(Entity& entity, std::vector<CollisionInfo>& info) const;
	virtual bool getCollisions(Entity& entity, const glm::vec3& begin, const glm::vec3& end, std::vector<CollisionInfo>& info) const;
	
	/**
	 * Same as getCollisions, but we stop once we found the first collision.
	 */
	virtual bool doesCollide(Entity& entity, CollisionInfo& info) const;
	virtual bool doesCollide(Entity& entity, const glm::vec3& begin, const glm::vec3& end, CollisionInfo& info) const;
	virtual bool doesCollide(Entity* entity, const glm::vec3& point) const;
	virtual bool doesCollide(Entity* entity, const glm::vec3& begin, const glm::vec3& end, float effective_width) const;

	BoundedBox& getFrustumChecker() const { return *frustum_checker_; }
	BoundedBox& getCollisionChecker() const { return *bounded_collision_box_; }

	SceneNode* getParent() const { return parent_; }
	virtual void setParent(SceneNode* new_parent);
	
	Region* getRegion() const { return current_region_; }
	void setRegion(Region& region) { current_region_ = &region; }

	void addChild(SceneNode& child);
	void removeChild(const SceneNode& child);

	// Debug rendering.
	static Material* bright_material_;
	
	/**
	 * Behaviour describes how an entity animates. They can respond to on activate triggers.
	 */
	void addBehaviour(Behaviour& behaviour);
	void deleteBehaviour(const Behaviour& behaviour);

	void ignoreRotations(bool enable) { ignore_rotations_ = enable; }

	void updateTransformations();
protected:
	
	inline bool transformationIsUpdated() const { return previous_transform_ != getCompleteTransformation(); }
	
	SceneManager* scene_manager_;
	glm::mat4 local_transformation_;
	glm::mat4 complete_transformation_;
	SceneNode* parent_;

	BoundedBox* bounded_collision_box_;
	BoundedBox* frustum_checker_;

	std::vector<SceneNode*> children_;
	std::vector<Behaviour*> behaviours_;

	Region* current_region_;
	
	/**
	 * Called whenever the containing visual box is updated. Either becaues of a transformation or due to the 
	 * transformation of one if its child nodes. This updates the @ref{frustum_checker} bounding box such that
	 * the updated child node still fits in it.
	 * @param recursive If true then all the children are checked recursively, otherwise only the leafs of this
	 * scene node are checked.
	 */
	virtual void updateBoundingBoxesBoundaries(bool recursive);
private:
	
	/**
	 * Update from which region this node is part and visible from.
	 */
	//void updateVisibility();
	
	/**
	 * Administrative stuff, remove / add children to this node as applicable.
	 */
	void updateChildren();

	std::vector<SceneLeaf*> leafs_;
	
	std::vector<SceneNode*> children_to_add_;
	std::vector<const SceneNode*> children_to_remove_;
	std::vector<const SceneLeaf*> leafs_to_remove_;

	// These flags determine how scene nodes are updated. Those that are 
	// marked for visits will be traversed, those that are marked for updates
	// will actually be updated.
	bool marked_for_update_;
	bool marked_for_visit_;
	
	glm::mat4 previous_transform_;
	
	bool ignore_rotations_;
	
	/**
	 * Keep track of all the regions this entity is visible from. We also keep the scene node
	 * that is added to that region to offset this entity.
	 */
	std::vector<Region*> regions_visible_from_;
};

#endif
