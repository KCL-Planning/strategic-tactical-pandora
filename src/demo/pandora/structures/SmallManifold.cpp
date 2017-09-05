#include "SmallManifold.h"

#include "../../../core/loaders/PortalLevelFormatLoader.h"
#include "../../../core/scene/SceneManager.h"
#include "../../../core/scene/Material.h"
#include "../../../core/shaders/ToonShader.h"
#include "../../../core/shaders/BasicShadowShader.h"
#include "../../../core/texture/FreeImageLoader.h"
#include "../../../core/texture/TargaTexture.h"

#include "../Waypoint.h"
#include "../shaders/CausticShader.h"
#include "../ontology/Pose.h"
#include "../ontology/InspectionPoint.h"
#include "../level/MissionSite.h"

SmallManifold::SmallManifold(SceneManager& scene_manager, SceneNode* parent, MissionSite& mission_site, const glm::mat4& transformation)
	: Entity(scene_manager, parent, transformation, OBSTACLE, "Small manifold")
{
	// Initialise the texture to use.
	Texture* texture = TargaTexture::loadTexture("data/models/Pandora/misc/atlas/ColourAtlas.tga");
	MaterialLightProperty* wfl_ambient = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);
	MaterialLightProperty* wfl_diffuse = new MaterialLightProperty(0.8f, 0.8f, 0.8f, 1.0f);
	MaterialLightProperty* wfl_specular = new MaterialLightProperty(0.2f, 0.2f, 0.2f, 1.0f);
	MaterialLightProperty* wfl_emmisive = new MaterialLightProperty(0.6f, 0.6f, 0.6f, 1.0f);

	Material* wfl_material_ = new Material(*wfl_ambient, *wfl_diffuse, *wfl_specular, *wfl_emmisive);
	wfl_material_->add2DTexture(*texture);
	
	PortalLevelFormatLoader* level_loader = new PortalLevelFormatLoader();
	SceneNode* level_node_ = level_loader->importLevel("data/models/Pandora/misc/small_manifold.plf", *wfl_material_, CausticShader::getShader(), *scene_manager_, *this, false);
	if (level_node_ == NULL)
	{
#ifdef _WIN32
		MessageBox(NULL, "Could not load the level!", "Error", MB_OK);
#else
		std::cerr << "Could not load the level!" << std::endl;
#endif
		exit(1);
	}
	
	// Initialise the inspection points.
	InspectionPoint* ip1 = new InspectionPoint(Pose(getLocalLocation().x + 4, getLocalLocation().y + 2, getLocalLocation().z, 0, -90), NULL);
	InspectionPoint* ip2 = new InspectionPoint(Pose(getLocalLocation().x + 4, getLocalLocation().y + 2, getLocalLocation().z - 2, 0, -90), NULL);
	InspectionPoint* ip3 = new InspectionPoint(Pose(getLocalLocation().x + 4, getLocalLocation().y + 2, getLocalLocation().z + 2, 0, -90), NULL);
	InspectionPoint* ip4 = new InspectionPoint(Pose(getLocalLocation().x - 4, getLocalLocation().y + 2, getLocalLocation().z + 2, 0, 90), NULL);
	InspectionPoint* ip5 = new InspectionPoint(Pose(getLocalLocation().x - 4, getLocalLocation().y + 2, getLocalLocation().z, 0, 90), NULL);
	InspectionPoint* ip6 = new InspectionPoint(Pose(getLocalLocation().x - 4, getLocalLocation().y + 2, getLocalLocation().z - 2, 0, 90), NULL);
	InspectionPoint* ip7 = new InspectionPoint(Pose(getLocalLocation().x - 0.5f, getLocalLocation().y + 2, getLocalLocation().z + 5, 0, 180), NULL);
	InspectionPoint* ip8 = new InspectionPoint(Pose(getLocalLocation().x - 0.5f, getLocalLocation().y + 2, getLocalLocation().z - 5, 0, 0), NULL);
	
	inspection_points_.push_back(ip1);
	inspection_points_.push_back(ip2);
	inspection_points_.push_back(ip3);
	inspection_points_.push_back(ip4);
	inspection_points_.push_back(ip5);
	inspection_points_.push_back(ip6);
	inspection_points_.push_back(ip7);
	inspection_points_.push_back(ip8);
	
	InspectionGoal* inspection_goal = new InspectionGoal(mission_site);
	inspection_goals_.push_back(inspection_goal);
	for (std::vector<InspectionPoint*>::const_iterator ci = getInspectionPoints().begin(); ci != getInspectionPoints().end(); ++ci)
	{
		InspectionPoint* ip = *ci;
		inspection_goal->addInspectionPoint(*ip);
	}
}
