#ifndef CORE_RENDERER_SIMPLE_RENDERER_H
#define CORE_RENDERER_SIMPLE_RENDERER_H

#include <vector>

#include "Renderer.h"

/**
 * In order to render a scene, we use the visitor pattern and deal with geometry and lights 
 * in different ways.
 */
class FrustumCaster;
class Entity;
class Light;
class PointLight;
class LightManager;
class SceneLeafLight;
class SceneLeafModel;
class RenderableSceneLeaf;
class SceneManager;
class Frustum;
class Texture;

class SimpleRenderer : public Renderer
{
public:
	SimpleRenderer(SceneManager& scene_manager);

	~SimpleRenderer();

	void render(const FrustumCaster& cam);

	unsigned int getFramebufferId() const { return fbo_id_; }
	Texture& getTexture() const { return *color_texture_; }
	Texture& getDepth() const { return *depth_texture_; }

	void visit(const SceneLeafLight& light);
	void visit(const RenderableSceneLeaf& model);

	// Debug.
	unsigned int getCulledObjects() const { return culled_objects_; }
	unsigned int getRenderedObjects() const { return rendered_objects_; }
	unsigned int getPreRenderedObjects() const { return pre_rendered_objects_; }
	const Frustum& getFrustum() const { return *frustum_; }

	void setShowCollisionBoxes(bool show_collision_boxes) { show_collision_boxes_ = show_collision_boxes; }

	const std::stringstream& getDebugString() const { return ss_; }

	void setRenderMode(GLenum render_mode) { render_mode_ = render_mode; glPolygonMode( GL_FRONT_AND_BACK, render_mode_); }

	unsigned int getDoubleRenderedModels() const { return double_models_; }
	unsigned int getDoubleRenderedLights() const { return double_lights_; }

	void onResize(int width, int height);

private:

	void initialise();

	const RenderableSceneLeaf* sky_box_;
	std::vector<const RenderableSceneLeaf*> active_entities_;
	std::vector<const SceneLeafLight*> active_lights_;
	//LightManager* light_manager_;

	SceneManager* scene_manager_;

	// Off screen buffer.
	unsigned int fbo_id_;
	//unsigned int texture_id_, depth_id_;
	Texture* color_texture_;
	Texture* depth_texture_;

	// Debug.
	unsigned int culled_objects_, rendered_objects_, pre_rendered_objects_;

	Frustum* frustum_;

	bool show_collision_boxes_;
	std::stringstream ss_;

	GLenum render_mode_;

	unsigned int double_models_, double_lights_;
};

#endif
