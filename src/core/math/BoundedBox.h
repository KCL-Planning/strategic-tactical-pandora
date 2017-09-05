/**
 * A polygon is a 3d shape that consists of a a set of planes which define the circumvence 
 * of the 3d volume. The polygon is always convex due to the inifinite planes which bound it.
 */
#ifndef CORE_MATH_BOUNDED_BOX_H
#define CORE_MATH_BOUNDED_BOX_H

#include <glm/glm.hpp>
#include <vector>

#include "../scene/frustum/InFrustumCheck.h"

class Entity;
class Plane;
class SceneLeafLight;
class SceneLeafModel;

class BoundedBox : public InFrustumCheck
{
public:
	/**
	 * Create a bounded box that encompases all the leafs that are children of the given scene node
	 * as close as possible. This box will subsequently be used to determine whether the node should 
	 * be renderered.
	 * @param The scene node for which the polygon should be constructed.
	 */
	BoundedBox(const SceneNode& scene_node, bool check_visibility, const std::vector<const SceneNode*>& excluded_nodes);

	/**
	 * Create a box for a scene leaf model. This is always done to check visibility. Collision detecting
	 * is only every performed on scene nodes.
	 */
	BoundedBox(const SceneLeafModel& scene_leaf_model);

	/**
	 * Construct a axis aligned box with the given centre point, width, height, and depth.
	 */
	BoundedBox(const SceneNode& scene_node, float width, float height, float depth);

	/**
	 * Construct a box that encloses the given scene node.
	 */
	BoundedBox(const SceneNode& scene_node, 
               const glm::vec3& bottom_left_away, 
               const glm::vec3& bottom_right_away, 
               const glm::vec3& top_left_away, 
               const glm::vec3& top_right_away, 
               const glm::vec3& bottom_left_close, 
               const glm::vec3& bottom_right_close, 
               const glm::vec3& top_left_close, 
               const glm::vec3& top_right_close);
			   
	//void getPoints(std::vector<glm::vec3>& points) const;
	const glm::vec3* getPoints() const { return &points_[0]; }

	const glm::vec3& getCentrePoint() const { return centre_point_; }

	Plane* const* getPlanes() const { return &sides_[0]; }

	/**
	 * Check if the polygon is (partially) visible from the given frustum.
	 */
	bool isInsideFrustum(const Frustum& frustrum) const;

	/**
	 * Check if this polygon is (partially) inside another polygon.
	 */
	bool isInside(const BoundedBox& other) const;

	/**
	 * Check if a point is inside the box.
	 */
	bool isInside(const glm::vec3& point) const;

	bool doesCollide(const glm::vec3& begin_point, const glm::vec3& end_point) const;
	
	/**
	 * Check if this bounded box collides with the line segments, such that no part of the line gets
	 * closer than @param effective_width to the bounded box.
	 * @param begin_point The begin point of the line segment.
	 * @param end_point The end point of the line segment.
	 * @param effective_width How far away the linesegment needs to stay away from any aspect of the bounded box.
	 * @return True of the line segment gets closer than @param effective_width.
	 */
	bool doesCollide(const glm::vec3& begin_point, const glm::vec3& end_point, float effective_width) const;

	//void getDebug(const SceneNode& leaf, const Frustum& frustrum) const;

	/**
	 * Update the bounded box either for visibility or collision purposes.
	 * @param recursive Whether we should check the entire tree or only the direct children.
	 * @param update_visibility If true then we update the bounding box based on the visibility boxes of its children, otherwise
	 * we update the bounding box according to its children's collision boxes.
	 * @return True if the bounding box's dimensions were updated, false otherwise.
	 */
	bool update(bool recursive, bool update_visiblity);

private:

	void processSceneNode(const SceneNode& scene_node, const glm::mat4& transform_matrix, float &min_x, float &max_x, float &min_y, float &max_y, float &min_z, float &max_z, bool check_visibility, const std::vector<const SceneNode*>& excluded_nodes, const std::string& debug);

	/**
	 * Set all the relevant variables of this bounding box based on the extremities.
	 * @param min_x The minimum x over all the processed children so far.
	 * @param max_x The maximum x over all the processed children so far.
	 * @param min_y The minimum y over all the processed children so far.
	 * @param max_y The maximum y over all the processed children so far.
	 * @param min_z The minimum z over all the processed children so far.
	 * @param max_z The maximum z over all the processed children so far.
	 */
	void initialise(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z);

	/**
	 * Recursive call (only if @ref{update_visibility} is true) to update the bounds on this box.
	 * @param min_x The minimum x over all the processed children so far.
	 * @param max_x The maximum x over all the processed children so far.
	 * @param min_y The minimum y over all the processed children so far.
	 * @param max_y The maximum y over all the processed children so far.
	 * @param min_z The minimum z over all the processed children so far.
	 * @param max_z The maximum z over all the processed children so far.
	 * @param recursive Whether we should check the entire tree or only the direct children.
	 * @param update_visibility If true then we update the bounding box based on the visibility boxes of its children, otherwise
	 */
	void update(float &min_x, float &max_x, float &min_y, float &max_y, float &min_z, float &max_z, bool recursive, bool update_visibility);

protected:
	const SceneNode* scene_node_;
	glm::vec3 points_[8];
	Plane* sides_[6];
	glm::vec3 centre_point_;
	glm::vec4 principal_axis_[3];
};

#endif
