#include "Structure.h"

#include "../ontology/InspectionGoal.h"
#include "../ontology/InspectionPoint.h"

#include "../shaders/CausticShader.h"

#include "dpengine/loaders/PortalLevelFormatLoader.h"
#include "dpengine/scene/SceneManager.h"
#include "dpengine/scene/SceneLeafModel.h"
#include "dpengine/scene/Material.h"
#include "dpengine/shaders/ToonShader.h"
#include "dpengine/shaders/BasicShadowShader.h"
#include "dpengine/texture/FreeImageLoader.h"
#include "dpengine/texture/TargaTexture.h"
#include "../ontology/OntologyInterface.h"
#include "../level/MissionSite.h"

Structure::Structure(const std::string& name, const std::string& plf_file_name, const std::string& texture_file_name, DreadedPE::SceneManager& scene_manager, DreadedPE::SceneNode* parent, MissionSite& mission_site, const glm::mat4& transformation, const std::vector<InspectionPoint*>& inspection_points, bool separate_inspection_goals)
	: DreadedPE::Entity(scene_manager, parent, transformation, DreadedPE::OBSTACLE, name), mission_site_(&mission_site), inspection_points_(inspection_points), can_recharge_(false)
{
	std::cout << "CREATE A STRUCTURE WITH: " << inspection_points.size() << std::endl;
	
	if (separate_inspection_goals)
	{
		for (std::vector<InspectionPoint*>::const_iterator ci = inspection_points.begin(); ci != inspection_points.end(); ++ci)
		{
			InspectionGoal* inspection_goal = new InspectionGoal(*this);
			inspection_goals_.push_back(inspection_goal);
			InspectionPoint* ip = *ci;
			inspection_goal->addInspectionPoint(*ip);
		}
	}
	else
	{
		InspectionGoal* inspection_goal = new InspectionGoal(*this);
		inspection_goals_.push_back(inspection_goal);
		for (std::vector<InspectionPoint*>::const_iterator ci = inspection_points.begin(); ci != inspection_points.end(); ++ci)
		{
			InspectionPoint* ip = *ci;
			inspection_goal->addInspectionPoint(*ip);
		}
	}
	
	interact_location_ = glm::vec3(getCompleteTransformation() * glm::vec4(0, 0, 4, 1));
	
	// Create the meshes.
	DreadedPE::Texture* texture = DreadedPE::TargaTexture::loadTexture(texture_file_name);
	DreadedPE::MaterialLightProperty wfl_ambient(0.0f, 0.0f, 0.0f, 1.0f);
	DreadedPE::MaterialLightProperty wfl_diffuse(0.8f, 0.8f, 0.8f, 1.0f);
	DreadedPE::MaterialLightProperty wfl_specular(0.2f, 0.2f, 0.2f, 1.0f);
	DreadedPE::MaterialLightProperty wfl_emmisive(0.6f, 0.6f, 0.6f, 1.0f);

	std::shared_ptr<DreadedPE::Material> wfl_material(std::make_shared<DreadedPE::Material>(wfl_ambient, wfl_diffuse, wfl_specular, wfl_emmisive));
	wfl_material->add2DTexture(*texture);
	
	DreadedPE::PortalLevelFormatLoader* level_loader = new DreadedPE::PortalLevelFormatLoader();
	DreadedPE::SceneNode* level_node_ = level_loader->importLevel(plf_file_name, wfl_material, &CausticShader::getShader(), *scene_manager_, *this, false);
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

Structure::Structure(const std::string& name, DreadedPE::SceneManager& scene_manager, DreadedPE::SceneNode* parent, MissionSite& mission_site, const glm::mat4& transformation)
	: DreadedPE::Entity(scene_manager, parent, transformation, DreadedPE::OBSTACLE, name), mission_site_(&mission_site), can_recharge_(false)
{
	interact_location_ = glm::vec3(getCompleteTransformation() * glm::vec4(0, 0, 4, 1));
}

void Structure::addValve(Valve& valve)
{
	valves_.push_back(&valve);
}

void Structure::makeBright(SceneNode& scene_node)
{
	/*
	for (std::vector<DreadedPE::SceneLeaf*>::const_iterator ci = scene_node.getLeafs().begin(); ci != scene_node.getLeafs().end(); ++ci)
	{
		DreadedPE::SceneLeaf* scene_leaf = *ci;
		DreadedPE::SceneLeafModel* model = dynamic_cast<DreadedPE::SceneLeafModel*>(scene_leaf);
		if (model != NULL)
		{
			model->setMaterial(*DreadedPE::SceneNode::bright_material_);
		}
	}

	for (std::vector<DreadedPE::SceneNode*>::const_iterator ci = scene_node.getChildren().begin(); ci != scene_node.getChildren().end(); ++ci)
	{
		makeBright(**ci);
	}
	*/
}

void Structure::destroy()
{
	std::cout << "DESTROY STRUCTURE!" << std::endl;
	mission_site_->removeStructure(*this);
	DreadedPE::Entity::destroy();
}
