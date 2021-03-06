#include "Chain.h"

#include <stdlib.h>

#include "dpengine/loaders/AssimpLoader.h"
#include "dpengine/loaders/PortalLevelFormatLoader.h"
#include "dpengine/texture/TargaTexture.h"
#include "dpengine/scene/SceneManager.h"
#include "dpengine/scene/SceneLeaf.h"
#include "dpengine/scene/SceneLeafModel.h"
#include "dpengine/scene/SceneNode.h"
#include "dpengine/scene/Material.h"
#include "dpengine/shaders/BasicShadowShader.h"
#include "dpengine/models/AnimatedModel.h"
#include "dpengine/texture/Texture.h"
#include "dpengine/entities/Entity.h"

#include "dpengine/collision/ConvexPolygon.h"

#include "dpengine/shapes/Cube.h"

#include "../models/HeightMap.h"

#include "../Waypoint.h"
#include "../level/MissionSite.h"

#include "../ontology/ChainGoal.h"
#include "ChainLink.h"

int Chain::global_chain_id_ = 0;

Chain::Chain(MissionSite& mission_site, DreadedPE::SceneManager& scene_manager, DreadedPE::SceneNode* parent, HeightMap& height_map, const glm::mat4& transformation, unsigned int nr_chain_links)
	: Structure("chain", scene_manager, parent, mission_site, transformation), has_been_observed_(false), notification_sent_(false), mission_site_(&mission_site)
{
	std::stringstream ss;
	ss << "chain_" << global_chain_id_;
	++global_chain_id_;
	id_ = ss.str();
	
	chain_goal_ = new ChainGoal(*this);
	
	DreadedPE::Texture* grass_texture = DreadedPE::TargaTexture::loadTexture("data/models/Pandora/misc/atlas/ColourAtlas.tga");
	
	// Initialise the materials for the chain.
	DreadedPE::MaterialLightProperty chain_ambient(0.7f, 0.7f, 0.7f, 1.0f);
	DreadedPE::MaterialLightProperty chain_diffuse(0.8f, 0.8f, 0.8f, 1.0f);
	DreadedPE::MaterialLightProperty chain_specular(0.0f, 0.0f, 0.0f, 1.0f);
	DreadedPE::MaterialLightProperty chain_emmisive(0.5f, 0.5f, 0.5f, 1.0f);

	std::shared_ptr<DreadedPE::Material> material(std::make_shared<DreadedPE::Material>(chain_ambient, chain_diffuse, chain_specular, chain_emmisive));
	material->add2DTexture(*grass_texture);
	
	// Load the chain model.
	std::pair<std::shared_ptr<DreadedPE::AnimatedModel>, std::map<aiTextureType, std::vector<DreadedPE::Texture*>* >* > chain_obj = DreadedPE::AssimpLoader::LoadModel(scene_manager, "data/models/Pandora/misc/small_chain.obj");
	/*
	InstanceRenderedShape* single_instance = new InstanceRenderedShape(*chain_obj.first);
	InstanceRenderedSceneNode* chain = new InstanceRenderedSceneNode(scene_manager, this, glm::mat4(1.0f));
	SceneLeafInstanced* leaf = new SceneLeafInstanced(*chain, InstanceShader::getShader(), *single_instance, *material);
	chain->setInstancedLeaf(*leaf);
	*/
	// Do a random walk to determine where the chain will go.
	glm::mat4 current_direction = glm::mat4(1.0f);
	glm::vec3 current_location(0, 0, 0);
	float rotation = 0;
	float pitch = 10;
	float yaw = 0;
	float total_pitch = 10;
	bool on_surface = true;
	unsigned int last_links_since_change = 0;
	
	updateTransformations();
	std::cout << "Chain is at: " << getGlobalLocation().x << ", " << getGlobalLocation().y << ", " << getGlobalLocation().z << ")" << std::endl;
	
	//Cube* collision_box = new Cube(0.2f, 0.1f, 0.8f);
	for (unsigned int i = 0; i < nr_chain_links; ++i)
	{
		std::cout << "Pitch: " << pitch << ". Current direction: (" << current_location.x << ", " << current_location.y << ", " << current_location.z << ")" << std::endl;
		
		// Rotate the direction.
		current_direction = glm::mat4(1.0f);
		current_direction = glm::rotate(current_direction, glm::radians(total_pitch), glm::vec3(0, 1, 0));
		current_direction = glm::rotate(current_direction, glm::radians(yaw), glm::vec3(1, 0, 0));
		
		glm::vec4 current_location2 = current_direction * glm::vec4(0.0f, 0.0f, -0.4f, 1.0f);
		current_location += glm::vec3(current_location2);
		
		rotation += 55;
		
		glm::mat4 chain_location = glm::translate(glm::mat4(1.0f), current_location);
		chain_location = glm::rotate(chain_location, glm::radians(total_pitch), glm::vec3(0, 1, 0));
		chain_location = glm::rotate(chain_location, glm::radians(yaw), glm::vec3(1, 0, 0));
		chain_location = glm::rotate(chain_location, glm::radians(rotation), glm::vec3(0, 0, 1));
		
		//ChainLink* chain_link = new ChainLink(scene_manager, *this, chain_location);
		DreadedPE::Entity* chain_link = new Entity(scene_manager, this, chain_location, DreadedPE::OBSTACLE, "chainlink");
		//chain->addChild(*chain_link);
		//chain_links_.push_back(chain_link);
		
		DreadedPE::SceneNode* node = new DreadedPE::SceneNode(*scene_manager_, this, chain_location);
		DreadedPE::SceneLeafModel* col_slm = new DreadedPE::SceneLeafModel(*node, NULL, chain_obj.first, material, DreadedPE::BasicShadowShader::getShader(), false, false);
		
		node->updateTransformations();
		std::cout << "(" << node->getGlobalLocation().x << ", " << node->getGlobalLocation().y << node->getGlobalLocation().z << ")" << std::endl;
		
		// Create a bounding box per chain.
		DreadedPE::ConvexPolygon* bc = new DreadedPE::ConvexPolygon(*chain_link, 0.2f, 0.1f, 0.8f);
		chain_link->addCollision(*bc);
		
		if (rand() < RAND_MAX / 16 || ((on_surface && last_links_since_change > 16) || (!on_surface && last_links_since_change > 12)))
		{
			on_surface = !on_surface;
			last_links_since_change = 0;
			
			if (on_surface) yaw = 15;
			else yaw = -15;
		}
		++last_links_since_change;
		
		if (on_surface && current_location.y < height_map.getHeight(current_location.x, current_location.z))
		{
			yaw = 25;
		}
		else if (on_surface && current_location.y > 0.3f)
		{
			yaw = 0;
		}
		else if (!on_surface && current_location.y > height_map.getHeight(current_location.x, current_location.z) - 0.1f)
		{
			yaw = -10;
		}
		else if (!on_surface)
		{
			yaw = 0;
		}
		else
		{
			yaw = (float)rand() / (float)RAND_MAX - 0.5f;
		}
		
		total_pitch += 30 * (((float)rand() / (float)RAND_MAX) - 0.5f);
	}
	//single_instance->finaliseBuffers();
	Waypoint& start = mission_site.getStartWaypoint();
	std::cout << "Start point for chain: " << start << std::endl;
	start.position_.y = 3.0f;
}

void Chain::setObserved()
{
	has_been_observed_ = true;
}
