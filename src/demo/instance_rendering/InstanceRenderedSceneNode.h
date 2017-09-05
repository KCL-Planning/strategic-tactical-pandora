#ifndef DEMO_INSTANCE_RENDERING_INSTANCE_RENDERED_SCENE_NODE_H
#define DEMO_INSTANCE_RENDERING_INSTANCE_RENDERED_SCENE_NODE_H

#include <glm/glm.hpp>
#include <GL/glew.h>
#include <vector>

#include "../../core/scene/SceneNode.h"

class SceneManager;
class SceneNode;
class InstanceRenderedShape;
class SceneLeafInstanced;

/**
 * This node in the tree contains all the scene nodes that render a specific shape. All the scene nodes that are children of this
 * scene node are considered to be locations where another instance of the shape should be rendered. The only information that is 
 * relevant is the translation of these scene nodes.
 */
class InstanceRenderedSceneNode : public SceneNode
{
public:
	InstanceRenderedSceneNode(SceneManager& scene_manager, SceneNode* parent, const glm::mat4& local_transformation);
	
	/**
	 * Update the model matrixes of the instanced shape.
	 * @param dt Time passed since last call.
	 */
	void prepare(float dt);
	
	void addChild(SceneNode& child);
	void removeChild(SceneNode& child);
	
	/**
	 * Check if this scene node should be rendered -- this overloads the prerenders of all its children.
	 * @param frustum The frustum which we use to determine whether this scene node should be rendered.
	 * @param camera_position The position of the camera.
	 * @param renderer The renderer we should return this node to if it needs to be rendered.
	 * @param process_lights A flag to determine whether lights should be processed (ignored).
	 * @param nr_calls Debug flag.
	 */
	void preRender(const Frustum& frustum, const glm::vec3& camera_position, Renderer& renderer, bool process_lights, unsigned int& nr_calls);
	
	/**
	 * Set the instanced leaf to be rendered.
	 * @param leaf The leaf that is rendered multiple times.
	 */
	void setInstancedLeaf(SceneLeafInstanced& leaf) { instanced_scene_leaf_ = &leaf; }
	
	/**
	 * Called whenever one of the children has moved, prompting us to update the model matrixes of the children.
	 * @param recursive Ignored.
	 */
	virtual void updateBoundingBoxesBoundaries(bool recursive);
private:
	void updateMatrixes();
	
	SceneLeafInstanced* instanced_scene_leaf_;  // The shape that is getting rendered more than once.
	bool update_matrixes_;
};

#endif
