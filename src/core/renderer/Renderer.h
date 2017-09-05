#ifndef CORE_RENDERER_RENDERER_H
#define CORE_RENDERER_RENDERER_H

#include <glm/glm.hpp>
#include "../scene/SceneVisitor.h"

/**
 * In order to render a scene, we use the visitor pattern and deal with geometry and lights 
 * in different ways.
 */
class SceneLeafLight;
class SceneLeafModel;
class RenderableSceneLeaf;
class Region;
class SceneManager;
class FrustumCaster;

class Renderer : public SceneVisitor
{
public:
	Renderer();

	virtual void visit(const SceneLeafLight& light) = 0;
	virtual void visit(const RenderableSceneLeaf& model) = 0;

	virtual void render(const FrustumCaster& frustum) = 0;

	virtual void onResize(int width, int height) = 0;
protected:
	// Optimise rendering by remembering the last region we rendered from before. Instead of 
	// searching the region we are in from the root node we perform a local search to find
	// the region we need to render from.
	Region* getRegionToRenderFrom(const glm::vec3& render_location, const SceneManager& scene_manager);

private:
	Region* last_region_;
};

#endif
