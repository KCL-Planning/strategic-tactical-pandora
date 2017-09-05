#include "SceneLeafModel.h"

#include "SceneNode.h"
#include "../renderer/Renderer.h"
#include "../shaders/ShaderInterface.h"
#include "../../shapes/Shape.h"
#include "../math/BoundedBox.h"
//#include "frustum/BoxCheck.h"

SceneLeafModel::SceneLeafModel(SceneNode& parent, InFrustumCheck* frustum_checker, Shape& shape, const Material& material, ShaderInterface& shader, bool is_transparent, bool is_double_sided, MODEL_TYPE type, ShadowRenderer::SHADOW_TYPE shadow_type)
	: RenderableSceneLeaf(parent, is_transparent, is_double_sided, type, shadow_type, frustum_checker), shape_(&shape), material_(&material), shader_(&shader)
{
	
}

SceneLeafModel::~SceneLeafModel()
{

}

void SceneLeafModel::prepare(float dt)
{
	shape_->prepare(dt);
}

void SceneLeafModel::preRender(const Frustum& frustum, const glm::vec3& camera_position, Renderer& renderer, bool process_lights)
{
	if (frustum_checker_ != NULL && !frustum_checker_->isInsideFrustum(frustum))
	{
		return;
	}

	// The entities are prerendered by the game world. This class solely deals with rendering the objects.
	renderer.visit(*this);
}

void SceneLeafModel::accept(SceneVisitor& visitor) const
{
	visitor.visit(*this);
}

void SceneLeafModel::draw(const glm::mat4& view_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights, ShaderInterface* shader) const
{
	ShaderInterface* shader_to_use = shader == NULL ? shader_ : shader;

	shader_to_use->initialise(*this, view_matrix, parent_->getCompleteTransformation(), projection_matrix, lights);

	//shape_->render();
}

void SceneLeafModel::initialiseFrustrumChecker()
{
	if (frustum_checker_ != NULL)
	{
		delete frustum_checker_;
	}
	frustum_checker_ = new BoundedBox(*this);
}
