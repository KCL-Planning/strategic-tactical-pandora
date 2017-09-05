#include <sstream>

#include "RechargeStation.h"

#include "../../../core/loaders/PortalLevelFormatLoader.h"
#include "../../../core/scene/SceneManager.h"
#include "../../../core/scene/Material.h"
#include "../../../core/shaders/ToonShader.h"
#include "../../../core/shaders/BasicShadowShader.h"
#include "../../../core/texture/FreeImageLoader.h"
#include "../../../core/texture/TargaTexture.h"

#include "../Waypoint.h"
#include "../shaders/CausticShader.h"
#include "../structures/Structure.h"

RechargeStation::RechargeStation(SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transformation)
	: MissionSite(scene_manager, parent, transformation)
{
	start_waypoint_->position_ = getLocalLocation() + glm::vec3(0, -2, 0);
	can_recharge_ = true;
	
	Texture* texture = TargaTexture::loadTexture("data/models/Pandora/misc/atlas/ColourAtlas.tga");
	MaterialLightProperty* wfl_ambient = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);
	MaterialLightProperty* wfl_diffuse = new MaterialLightProperty(0.8f, 0.8f, 0.8f, 1.0f);
	MaterialLightProperty* wfl_specular = new MaterialLightProperty(0.2f, 0.2f, 0.2f, 1.0f);
	MaterialLightProperty* wfl_emmisive = new MaterialLightProperty(0.6f, 0.6f, 0.6f, 1.0f);

	Material* wfl_material_ = new Material(*wfl_ambient, *wfl_diffuse, *wfl_specular, *wfl_emmisive);
	wfl_material_->add2DTexture(*texture);
	
	PortalLevelFormatLoader* level_loader = new PortalLevelFormatLoader();
	SceneNode* level_node_ = level_loader->importLevel("data/models/Pandora/misc/recharge_station.plf", *wfl_material_, CausticShader::getShader(), *scene_manager_, *this, false);
	if (level_node_ == NULL)
	{
#ifdef _WIN32
		MessageBox(NULL, "Could not load the level!", "Error", MB_OK);
#else
		std::cerr << "Could not load the level!" << std::endl;
#endif
		exit(1);
	}
}
