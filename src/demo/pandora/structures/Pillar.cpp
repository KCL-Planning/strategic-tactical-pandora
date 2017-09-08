#include "Pillar.h"

#include <stdlib.h>

#include "dpengine/loaders/PortalLevelFormatLoader.h"
#include "dpengine/scene/SceneManager.h"
#include "dpengine/scene/SceneLeaf.h"
#include "dpengine/scene/SceneLeafModel.h"
#include "dpengine/scene/SceneNode.h"
#include "dpengine/scene/Material.h"
#include "dpengine/texture/TargaTexture.h"
#include "dpengine/shaders/BasicShadowShader.h"

#include "../Waypoint.h"
#include "../level/MissionSite.h"
#include "../shaders/CausticShader.h"

Pillar::Pillar(const std::string& name, MissionSite& mission_site, DreadedPE::SceneManager& scene_manager, DreadedPE::SceneNode* parent, const glm::mat4& transformation, const std::string& level_file_name, DreadedPE::Texture& texture)
	: DreadedPE::Entity(scene_manager, parent, transformation, DreadedPE::OBSTACLE, name), shinny_timer_(0), has_been_observed_(false), notification_sent_(false), mission_site_(&mission_site)
{
	//Texture* wfl_texture = TargaTexture::loadTexture("data/models/levels/modular/atlas.tga");
	
	// Initialise the texture to use.
	DreadedPE::MaterialLightProperty wfl_ambient(0.4f, 0.4f, 0.4f, 1.0f);
	DreadedPE::MaterialLightProperty wfl_diffuse(0.8f, 0.8f, 0.8f, 1.0f);
	DreadedPE::MaterialLightProperty wfl_specular(0.3f, 0.3f, 0.3f, 1.0f);
	DreadedPE::MaterialLightProperty wfl_emmisive(0.6f, 0.6f, 0.6f, 1.0f);

	wfl_material_ = std::make_shared<DreadedPE::Material>(wfl_ambient, wfl_diffuse, wfl_specular, wfl_emmisive);
	wfl_material_->add2DTexture(texture);
	
	DreadedPE::PortalLevelFormatLoader* level_loader = new DreadedPE::PortalLevelFormatLoader();
	//level_node_ = level_loader->importLevel("data/models/levels/pandora/structures.plf", *scene_manager_, scene_manager_->getRoot());
	DreadedPE::SceneNode* level_node_ = level_loader->importLevel(level_file_name, wfl_material_, &CausticShader::getShader(), *scene_manager_, *this, false);
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

void Pillar::setObserved()
{
	if (!has_been_observed_)
	{
		has_been_observed_ = true;
		shinny_timer_ = 10;
		scene_manager_->addUpdateableEntity(*this);
	}
}

void Pillar::prepare(float dt)
{
	DreadedPE::Entity::prepare(dt);
	DreadedPE::MaterialLightProperty& mwp = wfl_material_->getEmissive();
	if (shinny_timer_ > 0)
	{
		shinny_timer_ -= dt;
		
		int i = shinny_timer_;
		float diff = shinny_timer_ - i;
		if (diff > 0.5) diff = 1.0 - diff;
		
		mwp.alpha_ = 1.0f;
		mwp.blue_ = 0.6f;
		mwp.green_ = 0.5f + diff;
		mwp.red_ = 0.6f;
	}
	else	
	{
		mwp.alpha_ = 1.0f;
		mwp.blue_ = 0.6f;
		mwp.green_ = 0.6f;
		mwp.red_ = 0.6f;
		scene_manager_->removeUpdateableEntity(*this);
	}
}

void Pillar::makeBright(DreadedPE::SceneNode& scene_node)
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
