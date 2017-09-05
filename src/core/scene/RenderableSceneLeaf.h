#ifndef CORE_SCENE_RENDERABLE_SCENE_LEAF_H
#define CORE_SCENE_RENDERABLE_SCENE_LEAF_H

#include "../renderer/ShadowRenderer.h"
#include "SceneLeaf.h"

enum MODEL_TYPE { OBJECT, SKYBOX, COLLISION };

/**
 * This interface declares a class that can be rendered by a renderer.
 */
class RenderableSceneLeaf : public SceneLeaf
{
public:
	RenderableSceneLeaf(SceneNode& scene_node, bool is_transparent, bool is_double_sided, MODEL_TYPE type, ShadowRenderer::SHADOW_TYPE shadow_type, InFrustumCheck* frustum_checker = NULL)
		: SceneLeaf(scene_node, frustum_checker), is_transparent_(is_transparent), is_double_sided_(is_double_sided), type_(type), shadow_type_(shadow_type)
	{
		
	}
	
	bool isTransparent() const { return is_transparent_; }
	bool isDoubleSided() const { return is_double_sided_; }
	MODEL_TYPE getType() const { return type_; }

	void setShadowType(ShadowRenderer::SHADOW_TYPE shadow_type) { shadow_type_ = shadow_type; }
	ShadowRenderer::SHADOW_TYPE getShadowType() const { return shadow_type_; }
protected:
	//ShaderInterface* shader_;//, *shadow_shader_;
	bool is_transparent_;
	bool is_double_sided_;

	// The rendering priority.
	MODEL_TYPE type_;

	// Shadow type.
	ShadowRenderer::SHADOW_TYPE shadow_type_;
};

#endif
