#include "ChainLink.h"
#include "../../../core/scene/SceneLeafModel.h"
#include "../../../core/scene/Material.h"
#include "../../../core/texture/TargaTexture.h"
#include "../../../core/shaders/BasicShadowShader.h"
#include "../../../core/shaders/ToonShader.h"
#include "../../../shapes/sphere.h"

ChainLink::ChainLink(SceneManager& scene_manager, SceneNode& parent, const glm::mat4& local_transformation)
	: Entity(scene_manager, &parent, local_transformation, OBSTACLE, "Chain Link"), been_found_(false)
{
	Texture* grass_texture = TargaTexture::loadTexture("data/textures/grass.tga");
	
	// Initialise a terrain to render.
	MaterialLightProperty* chain_link_ambient = new MaterialLightProperty(0.7f, 0.7f, 0.7f, 1.0f);
	MaterialLightProperty* chain_link_diffuse = new MaterialLightProperty(0.8f, 0.8f, 0.8f, 1.0f);
	MaterialLightProperty* chain_link_specular = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);
	MaterialLightProperty* chain_link_emmisive = new MaterialLightProperty(0.0f, 1.0f, 0.5f, 0.5f);

	Material* chain_link_material_ = new Material(*chain_link_ambient, *chain_link_diffuse, *chain_link_specular, *chain_link_emmisive);
	chain_link_material_->add2DTexture(*grass_texture);
	/*
	sphere_ = new Sphere(8, 8, 0.4f);
	status_node_ = new SceneLeafModel(*this, NULL, *sphere_, *chain_link_material_, ToonShader::getShader(), true, false);
	
	status_node_->setVisible(false);
	*/
}

bool ChainLink::activate(Entity& activator)
{
	//status_node_->setVisible(true);
}
