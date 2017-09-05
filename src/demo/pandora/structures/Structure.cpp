#include "Structure.h"

#include "../ontology/InspectionGoal.h"
#include "../ontology/InspectionPoint.h"

#include "../shaders/CausticShader.h"

#include "../../../core/loaders/PortalLevelFormatLoader.h"
#include "../../../core/scene/SceneManager.h"
#include "../../../core/scene/SceneLeafModel.h"
#include "../../../core/scene/Material.h"
#include "../../../core/shaders/ToonShader.h"
#include "../../../core/shaders/BasicShadowShader.h"
#include "../../../core/texture/FreeImageLoader.h"
#include "../../../core/texture/TargaTexture.h"
#include "../ontology/OntologyInterface.h"
#include "../level/MissionSite.h"

Structure::Structure(const std::string& name, const std::string& plf_file_name, const std::string& texture_file_name, SceneManager& scene_manager, SceneNode* parent, MissionSite& mission_site, const glm::mat4& transformation, const std::vector<InspectionPoint*>& inspection_points, bool separate_inspection_goals)
	: Entity(scene_manager, parent, transformation, OBSTACLE, name), mission_site_(&mission_site), inspection_points_(inspection_points), can_recharge_(false)
{
	std::cout << "CREATE A STRUCTURE WITH: " << inspection_points.size() << std::endl;
	
	//if (separate_inspection_goals)
	{
		for (std::vector<InspectionPoint*>::const_iterator ci = inspection_points.begin(); ci != inspection_points.end(); ++ci)
		{
			InspectionGoal* inspection_goal = new InspectionGoal(*this);
			inspection_goals_.push_back(inspection_goal);
			InspectionPoint* ip = *ci;
			inspection_goal->addInspectionPoint(*ip);
		}
	}/*
	else
	{
		InspectionGoal* inspection_goal = new InspectionGoal(*this);
		inspection_goals_.push_back(inspection_goal);
		for (std::vector<InspectionPoint*>::const_iterator ci = inspection_points.begin(); ci != inspection_points.end(); ++ci)
		{
			InspectionPoint* ip = *ci;
			inspection_goal->addInspectionPoint(*ip);
		}
	}*/
	
	interact_location_ = glm::vec3(getCompleteTransformation() * glm::vec4(0, 0, 4, 1));
	
	// Create the meshes.
	Texture* texture = TargaTexture::loadTexture(texture_file_name);
	MaterialLightProperty* wfl_ambient = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);
	MaterialLightProperty* wfl_diffuse = new MaterialLightProperty(0.8f, 0.8f, 0.8f, 1.0f);
	MaterialLightProperty* wfl_specular = new MaterialLightProperty(0.2f, 0.2f, 0.2f, 1.0f);
	MaterialLightProperty* wfl_emmisive = new MaterialLightProperty(0.6f, 0.6f, 0.6f, 1.0f);

	Material* wfl_material_ = new Material(*wfl_ambient, *wfl_diffuse, *wfl_specular, *wfl_emmisive);
	wfl_material_->add2DTexture(*texture);
	
	PortalLevelFormatLoader* level_loader = new PortalLevelFormatLoader();
	SceneNode* level_node_ = level_loader->importLevel(plf_file_name, *wfl_material_, CausticShader::getShader(), *scene_manager_, *this, false);
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

Structure::Structure(const std::string& name, SceneManager& scene_manager, SceneNode* parent, MissionSite& mission_site, const glm::mat4& transformation)
	: Entity(scene_manager, parent, transformation, OBSTACLE, name), mission_site_(&mission_site), can_recharge_(false)
{
	interact_location_ = glm::vec3(getCompleteTransformation() * glm::vec4(0, 0, 4, 1));
}

void Structure::addValve(Valve& valve)
{
	valves_.push_back(&valve);
}

void Structure::makeBright(SceneNode& scene_node)
{
	for (std::vector<SceneLeaf*>::const_iterator ci = scene_node.getLeafs().begin(); ci != scene_node.getLeafs().end(); ++ci)
	{
		SceneLeaf* scene_leaf = *ci;
		SceneLeafModel* model = dynamic_cast<SceneLeafModel*>(scene_leaf);
		if (model != NULL)
		{
			model->setMaterial(*SceneNode::bright_material_);
		}
	}

	for (std::vector<SceneNode*>::const_iterator ci = scene_node.getChildren().begin(); ci != scene_node.getChildren().end(); ++ci)
	{
		makeBright(**ci);
	}
}

void Structure::destroy()
{
	std::cout << "DESTROY STRUCTURE!" << std::endl;
	mission_site_->removeStructure(*this);
	Entity::destroy();
}
