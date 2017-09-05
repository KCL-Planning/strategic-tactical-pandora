#ifdef _WIN32
#include <windows.h>
#endif

#include "SkyBoxLeaf.h"

#include "../../shapes/Cube.h"
#include "SceneNode.h"
#include "../loaders/targa.h"
#include "../shaders/ShaderInterface.h"
#include "../renderer/Renderer.h"

SkyBoxLeaf::SkyBoxLeaf(SceneNode& parent, Shape& shape, ShaderInterface& shader, Material& material)//, const std::string& left, const std::string& right, const std::string& top, const std::string& bottom, const std::string& back, const std::string& front)
	: SceneLeafModel(parent, NULL, shape, material, shader, false, false, SKYBOX)
{
	
}

void SkyBoxLeaf::draw(const glm::mat4& view_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights, ShaderInterface* shader) const
{
	// Ignore any rotations.
	glm::mat4 no_rotation_transformation = glm::translate(glm::mat4(1.0f), parent_->getGlobalLocation());
	getShader().initialise(*this, view_matrix, no_rotation_transformation, projection_matrix, lights);

	//shape_->render();
}

void SkyBoxLeaf::prepare(float dt)
{

}

void SkyBoxLeaf::preRender(const Frustum& frustum, Renderer& renderer)
{
	renderer.visit(*this);
}

void SkyBoxLeaf::accept(Renderer& renderer) const
{
	renderer.visit(*this);
}
