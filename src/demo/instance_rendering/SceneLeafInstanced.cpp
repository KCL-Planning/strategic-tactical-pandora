#include "SceneLeafInstanced.h"

#include <cstdlib>

#include "../../core/renderer/ShadowRenderer.h"
#include "../../core/scene/frustum/InFrustumCheck.h"
#include "../../core/scene/SceneNode.h"

#include "InstanceShader.h"
#include "InstanceRenderedShape.h"
#include "InstanceRenderedSceneNode.h"

SceneLeafInstanced::SceneLeafInstanced(InstanceRenderedSceneNode& parent, InstanceShader& shader, InstanceRenderedShape& shape, Material& material)
	: SceneLeafModel(parent, NULL, shape, material, shader, false, false), instanced_shape_(&shape), instance_shader_(&shader)
{
	parent.setInstancedLeaf(*this);
}

void SceneLeafInstanced::draw(const glm::mat4& view_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights, ShaderInterface* shader) const
{
	instance_shader_->initialise(*this, view_matrix, parent_->getCompleteTransformation(), projection_matrix, lights);
}
/*
void SceneLeafInstanced::prepare(float dt)
{
	
	if (instanced_shape_->isUpdated())
	{
		// Update the model matrices of the shapes.
		std::vector<glm::mat4> new_model_matrixes;
		new_model_matrixes.reserve(instanced_shape_->getModelMatrixes().size());
		
		//for (int i = 0; i < shape_->getModelMatrixes().size(); ++i)
		for (std::vector<SceneNode*>::const_iterator ci = instanced_shape_->getNodes().begin(); ci != instanced_shape_->getNodes().end(); ++ci)
		{
			SceneNode* scene_node = *ci;
			scene_node->prepare(dt);
			new_model_matrixes.push_back(scene_node->getCompleteTransformation());
		}
		instanced_shape_->setModelMatrixes(new_model_matrixes);
		instanced_shape_->finaliseBuffers();
		instanced_shape_->setUpdated(false);
	}
	
	instanced_shape_->prepare(dt);
}
*/
void SceneLeafInstanced::preRender(const Frustum& frustum, const glm::vec3& camera_position, Renderer& renderer, bool process_lights)
{
	/*
	if (frustum_checker_ != NULL && !frustum_checker_->isInsideFrustum(frustum))
	{
		return;
	}
	*/
	renderer.visit(*this);
}

void SceneLeafInstanced::initialiseFrustrumChecker()
{
	
}

void SceneLeafInstanced::accept(SceneVisitor& visitory) const
{
	visitory.visit(*this);
}
