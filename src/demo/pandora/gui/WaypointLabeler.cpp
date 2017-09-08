#include <sstream>
#include <glm/gtc/matrix_transform.hpp>
#include "WaypointLabeler.h"
#include "../RRT.h"

#include "dpengine/shapes/TextBanner.h"
#include "dpengine/scene/SceneLeafModel.h"
#include "dpengine/scene/Material.h"
#include "dpengine/shaders/BasicShadowShader.h"
#include "dpengine/texture/Texture.h"
#include "dpengine/texture/TargaTexture.h"

WaypointLabeler::WaypointLabeler(DreadedPE::SceneManager& scene_manager, RRT& rrt, DreadedPE::SceneNode& parent)
	: SceneNode(scene_manager, &parent, glm::mat4(1.0f)), rrt_(&rrt)
{
	font_texture_ = DreadedPE::TargaTexture::loadTexture("data/textures/fonts/hand_drawn.tga");
	
	DreadedPE::MaterialLightProperty terrain_ambient(0.2f, 0.2f, 0.2f, 1.0f);
	DreadedPE::MaterialLightProperty terrain_diffuse(0.8f, 0.8f, 0.8f, 1.0f);
	DreadedPE::MaterialLightProperty terrain_specular(0.0f, 0.0f, 0.0f, 1.0f);
	DreadedPE::MaterialLightProperty terrain_emmisive(0.0f, 0.0f, 0.0f, 1.0f);

	material_ = std::make_shared<DreadedPE::Material>(terrain_ambient, terrain_diffuse, terrain_specular, terrain_emmisive);
	material_->add2DTexture(*font_texture_);
	rrt.addListener(*this);
}

void WaypointLabeler::rrtInvalidated()
{
	for (std::vector<DreadedPE::SceneLeafModel*>::const_iterator ci = models_.begin(); ci != models_.end(); ++ci)
	{
		//removeLeaf(**ci);
		//delete ci;
	}
	models_.clear();
}

void WaypointLabeler::rrtUpdated()
{
	unsigned int i = 0;
	for (std::vector<Waypoint*>::const_iterator ci = rrt_->getPoints().begin(); ci != rrt_->getPoints().end(); ++ci)
	{
		DreadedPE::SceneNode* scene_node = new DreadedPE::SceneNode(*scene_manager_, this, glm::translate(glm::mat4(1.0f), (*ci)->position_));
		std::stringstream ss;
		ss << "" << i;
		std::shared_ptr<DreadedPE::TextBanner> banner(std::make_shared<DreadedPE::TextBanner>(ss.str(), *font_texture_, 0.5f));
		
		DreadedPE::SceneLeafModel* leaf = new DreadedPE::SceneLeafModel(*scene_node, NULL, banner, material_, DreadedPE::BasicShadowShader::getShader(), true, true);
		models_.push_back(leaf);
		++i;
	}
}
