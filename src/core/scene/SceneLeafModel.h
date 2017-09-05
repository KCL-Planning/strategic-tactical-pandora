#ifndef CORE_SCENE_SCENE_LEAF_MODEL_H
#define CORE_SCENE_SCENE_LEAF_MODEL_H

//enum MODEL_TYPE { OBJECT, SKYBOX, COLLISION };

#include <vector>
#include <glm/glm.hpp>

#include "RenderableSceneLeaf.h"
#include "../renderer/ShadowRenderer.h"

class Shape;
class Material;
class ShaderInterface;
class Frustum;
class Renderer;
class SceneLeafLight;
class InFrustumCheck;
class SceneVisitor;

class SceneLeafModel : public RenderableSceneLeaf
{
public:
	SceneLeafModel(SceneNode& parent, InFrustumCheck* frustum_checker, Shape& shape, const Material& material, ShaderInterface& shader, bool is_transparent, bool is_double_sided, MODEL_TYPE type = OBJECT, ShadowRenderer::SHADOW_TYPE shadow_type = ShadowRenderer::STATIC_SHADOW);
	virtual ~SceneLeafModel();
	void prepare(float dt);
	void preRender(const Frustum& frustum, const glm::vec3& camera_position, Renderer& renderer, bool process_lights);
	
	void accept(SceneVisitor& visitor) const;

	Shape& getModel() const { return *shape_; }
	const Material& getMaterial() const { return *material_; }
	void setMaterial(const Material& material) { material_ = &material; }
	ShaderInterface& getShader() const { return *shader_; }
	//ShaderInterface* getShadowShader() const { return shadow_shader_; }

	//bool isTransparent() const { return is_transparent_; }
	//bool isDoubleSided() const { return is_double_sided_; }
	//MODEL_TYPE getType() const { return type_; }
	Shape& getShape() const { return *shape_; }

	virtual void draw(const glm::mat4& view_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights, ShaderInterface* shader = NULL) const;

	void initialiseFrustrumChecker();

	//void setShadowType(ShadowRenderer::SHADOW_TYPE shadow_type) { shadow_type_ = shadow_type; }
	//ShadowRenderer::SHADOW_TYPE getShadowType() const { return shadow_type_; }

protected:
	Shape* shape_;
	const Material* material_;
	ShaderInterface* shader_;//, *shadow_shader_;
	//bool is_transparent_;
	//bool is_double_sided_;

	// The rendering priority.
	//MODEL_TYPE type_;

	// Shadow type.
	//ShadowRenderer::SHADOW_TYPE shadow_type_;
};

#endif
