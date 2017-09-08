#include "ChainLink.h"
#include "dpengine/scene/SceneLeafModel.h"
#include "dpengine/scene/Material.h"
#include "dpengine/texture/TargaTexture.h"
#include "dpengine/shaders/BasicShadowShader.h"
#include "dpengine/shaders/ToonShader.h"
#include "dpengine/shapes/sphere.h"

ChainLink::ChainLink(DreadedPE::SceneManager& scene_manager, DreadedPE::SceneNode& parent, const glm::mat4& local_transformation)
	: DreadedPE::Entity(scene_manager, &parent, local_transformation, DreadedPE::OBSTACLE, "Chain Link"), been_found_(false)
{
	DreadedPE::Texture* grass_texture = DreadedPE::TargaTexture::loadTexture("data/textures/grass.tga");
	
	// Initialise a terrain to render.
	DreadedPE::MaterialLightProperty* chain_link_ambient = new DreadedPE::MaterialLightProperty(0.7f, 0.7f, 0.7f, 1.0f);
	DreadedPE::MaterialLightProperty* chain_link_diffuse = new DreadedPE::MaterialLightProperty(0.8f, 0.8f, 0.8f, 1.0f);
	DreadedPE::MaterialLightProperty* chain_link_specular = new DreadedPE::MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);
	DreadedPE::MaterialLightProperty* chain_link_emmisive = new DreadedPE::MaterialLightProperty(0.0f, 1.0f, 0.5f, 0.5f);

	DreadedPE::Material* chain_link_material_ = new DreadedPE::Material(*chain_link_ambient, *chain_link_diffuse, *chain_link_specular, *chain_link_emmisive);
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
	return true;
}
