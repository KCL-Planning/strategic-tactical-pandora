#include <sstream>

#include "RechargeStation.h"

#include "dpengine/loaders/PortalLevelFormatLoader.h"
#include "dpengine/scene/SceneManager.h"
#include "dpengine/scene/Material.h"
#include "dpengine/shaders/ToonShader.h"
#include "dpengine/shaders/BasicShadowShader.h"
#include "dpengine/texture/FreeImageLoader.h"
#include "dpengine/texture/TargaTexture.h"

#include "../Waypoint.h"
#include "../shaders/CausticShader.h"
#include "../structures/Structure.h"

RechargeStation::RechargeStation(DreadedPE::SceneManager& scene_manager, DreadedPE::SceneNode* parent, const glm::mat4& transformation)
	: MissionSite(scene_manager, parent, transformation)
{
	start_waypoint_->position_ = getLocalLocation() + glm::vec3(0, -2, 0);
	can_recharge_ = true;
	
	DreadedPE::Texture* texture = DreadedPE::TargaTexture::loadTexture("data/models/Pandora/misc/atlas/ColourAtlas.tga");
	DreadedPE::MaterialLightProperty* wfl_ambient = new DreadedPE::MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);
	DreadedPE::MaterialLightProperty* wfl_diffuse = new DreadedPE::MaterialLightProperty(0.8f, 0.8f, 0.8f, 1.0f);
	DreadedPE::MaterialLightProperty* wfl_specular = new DreadedPE::MaterialLightProperty(0.2f, 0.2f, 0.2f, 1.0f);
	DreadedPE::MaterialLightProperty* wfl_emmisive = new DreadedPE::MaterialLightProperty(0.6f, 0.6f, 0.6f, 1.0f);

	DreadedPE::Material* wfl_material_ = new DreadedPE::Material(*wfl_ambient, *wfl_diffuse, *wfl_specular, *wfl_emmisive);
	wfl_material_->add2DTexture(*texture);
	
	DreadedPE::PortalLevelFormatLoader* level_loader = new DreadedPE::PortalLevelFormatLoader();
	DreadedPE::SceneNode* level_node_ = level_loader->importLevel("data/models/Pandora/misc/recharge_station.plf", *wfl_material_, CausticShader::getShader(), *scene_manager_, *this, false);
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
