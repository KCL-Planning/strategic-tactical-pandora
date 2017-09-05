#include <sstream>

#include "WaypointLabeler.h"
#include "../RRT.h"

#include "../../../shapes/TextBanner.h"
#include "../../../core/scene/SceneLeafModel.h"
#include "../../../core/scene/Material.h"
#include "../../../core/shaders/BasicShadowShader.h"
#include "../../../core/texture/Texture.h"
#include "../../../core/texture/TargaTexture.h"

WaypointLabeler::WaypointLabeler(SceneManager& scene_manager, RRT& rrt, SceneNode& parent)
	: SceneNode(scene_manager, &parent, glm::mat4(1.0f)), rrt_(&rrt)
{
	font_texture_ = TargaTexture::loadTexture("data/textures/fonts/hand_drawn.tga");
	
	MaterialLightProperty* terrain_ambient = new MaterialLightProperty(0.2f, 0.2f, 0.2f, 1.0f);
	MaterialLightProperty* terrain_diffuse = new MaterialLightProperty(0.8f, 0.8f, 0.8f, 1.0f);
	MaterialLightProperty* terrain_specular = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);
	MaterialLightProperty* terrain_emmisive = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);

	material_ = new Material(*terrain_ambient, *terrain_diffuse, *terrain_specular, *terrain_emmisive);
	material_->add2DTexture(*font_texture_);
	rrt.addListener(*this);
}

void WaypointLabeler::rrtInvalidated()
{
	for (std::vector<SceneLeafModel*>::const_iterator ci = models_.begin(); ci != models_.end(); ++ci)
	{
		//removeLeaf(**ci);
		(*ci)->remove();
	}
	models_.clear();
	/*for (std::vector<TextBanner*>::const_iterator ci = shapes_.begin(); ci != shapes_.end(); ++ci)
	{
		delete *ci;
	}*/
	shapes_.clear();
}

void WaypointLabeler::rrtUpdated()
{
	unsigned int i = 0;
	for (std::vector<Waypoint*>::const_iterator ci = rrt_->getPoints().begin(); ci != rrt_->getPoints().end(); ++ci)
	{
		SceneNode* scene_node = new SceneNode(*scene_manager_, this, glm::translate(glm::mat4(1.0f), (*ci)->position_));
		std::stringstream ss;
		ss << "" << i;
		TextBanner* banner = new TextBanner(ss.str(), *font_texture_, 0.5f);
		
		SceneLeafModel* leaf = new SceneLeafModel(*scene_node, NULL, *banner, *material_, BasicShadowShader::getShader(), true, true);
		models_.push_back(leaf);
		shapes_.push_back(banner);
		++i;
	}
}
