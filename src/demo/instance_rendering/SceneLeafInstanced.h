#ifndef DEMO_INSTANCE_RENDERING_SCENE_LEAF_INSTANCED_H
#define DEMO_INSTANCE_RENDERING_SCENE_LEAF_INSTANCED_H

#include "../../core/scene/RenderableSceneLeaf.h"
#include "../../core/scene/SceneLeafModel.h"

class InstanceRenderedSceneNode;
class InstanceShader;
class InstanceRenderedShape;
class Material;

class SceneLeafInstanced : public SceneLeafModel
{
public:
	SceneLeafInstanced(InstanceRenderedSceneNode& parent, InstanceShader& shader, InstanceRenderedShape& shape, Material& material);

	void draw(const glm::mat4& view_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights, ShaderInterface* shader = NULL) const;

	//void prepare(float dt);
	void preRender(const Frustum& frustum, const glm::vec3& camera_position, Renderer& renderer, bool process_lights);

	void accept(SceneVisitor& visitory) const;

	InstanceRenderedShape& getInstancedShape() const { return *instanced_shape_; }

	void initialiseFrustrumChecker();

private:
	InstanceShader* instance_shader_;
	InstanceRenderedShape* instanced_shape_;
};

#endif
