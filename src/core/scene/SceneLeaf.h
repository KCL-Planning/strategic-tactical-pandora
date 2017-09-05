#ifndef CORE_SCENE_SCENE_LEAF_H
#define CORE_SCENE_SCENE_LEAF_H

#include <vector>
#include <glm/glm.hpp>

class ShaderInterface;
class SceneLeafLight;
class Renderer;
class SceneNode;
class Frustum;
class InFrustumCheck;
class SceneVisitor;

class SceneLeaf
{
public:
	SceneLeaf(SceneNode& scene_node, InFrustumCheck* frustum_checker = NULL);

	/**
	 * Draw the object to the screen, given the view and projection matrix. The model matrix is already part 
	 * of the scene leaf. The renderer can opt to overwrite the default shader used by specifying another 
	 * shader that must be used for the rendering process. This is usefull if we want to render shadows and 
	 * want to use a shader which renders all opaque meshes black and ignores any transparent meshes. Or if
	 * we want to render all opaque meshes black but leave the transparent meshes in their original colour.
	 * @param view_matrix
	 * @param projection_matrix
	 * @param shader (Optional) The shader that should be used to render the leaf. If this is NULL then we
	 * use the default shader.
	 */
	virtual void draw(const glm::mat4& view_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights, ShaderInterface* shader = NULL) const = 0;

	virtual void prepare(float dt) = 0;
	virtual void preRender(const Frustum& frustum, const glm::vec3& camera_position, Renderer& renderer, bool process_lights) = 0;

	virtual void accept(SceneVisitor& visitory) const = 0;

	const SceneNode* getParent() const { return parent_; }

	void setParent(SceneNode* parent) { parent_ = parent; }

	virtual void initialiseFrustrumChecker() = 0;
	bool isInFrustum(const Frustum& frustum) const;
	
	bool isVisible() const { return is_visible_; }
	void setVisible(bool is_visible) { is_visible_ = is_visible; }

	/**
	 * Delete this scene leaf.
	 */
	void remove();
protected:
	SceneNode* parent_;
	InFrustumCheck* frustum_checker_;
	bool is_visible_;
};

#endif
