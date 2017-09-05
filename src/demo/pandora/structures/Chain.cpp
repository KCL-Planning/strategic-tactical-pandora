#include "Chain.h"

#include <stdlib.h>

#include "../../../core/loaders/AssimpLoader.h"
#include "../../../core/loaders/PortalLevelFormatLoader.h"
#include "../../../core/texture/TargaTexture.h"
#include "../../../core/scene/SceneManager.h"
#include "../../../core/scene/SceneLeaf.h"
#include "../../../core/scene/SceneLeafModel.h"
#include "../../../core/scene/SceneNode.h"
#include "../../../core/scene/Material.h"
#include "../../../core/shaders/BasicShadowShader.h"
#include "../../../core/models/AnimatedModel.h"
#include "../../../core/texture/Texture.h"
#include "../../../core/entities/Entity.h"

#include "../../../core/collision/BoxCollision.h"

#include "../../../shapes/Cube.h"

#include "../../instance_rendering/InstanceShader.h"
#include "../../instance_rendering/SceneLeafInstanced.h"
#include "../../instance_rendering/InstanceRenderedShape.h"
#include "../../instance_rendering/InstanceRenderedSceneNode.h"

#include "../../../core/entities/HeightMap.h"

#include "../Waypoint.h"
#include "../level/MissionSite.h"

#include "../ontology/ChainGoal.h"
#include "ChainLink.h"

int Chain::global_chain_id_ = 0;

Chain::Chain(MissionSite& mission_site, SceneManager& scene_manager, SceneNode* parent, HeightMap& height_map, const glm::mat4& transformation, unsigned int nr_chain_links)
	: Structure("chain", scene_manager, parent, mission_site, transformation), has_been_observed_(false), notification_sent_(false), mission_site_(&mission_site)
{
	std::stringstream ss;
	ss << "chain_" << global_chain_id_;
	++global_chain_id_;
	id_ = ss.str();
	
	chain_goal_ = new ChainGoal(*this);
	
	Texture* grass_texture = TargaTexture::loadTexture("data/models/Pandora/misc/atlas/ColourAtlas.tga");
	
	// Initialise the materials for the chain.
	MaterialLightProperty* chain_ambient = new MaterialLightProperty(0.7f, 0.7f, 0.7f, 1.0f);
	MaterialLightProperty* chain_diffuse = new MaterialLightProperty(0.8f, 0.8f, 0.8f, 1.0f);
	MaterialLightProperty* chain_specular = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);
	MaterialLightProperty* chain_emmisive = new MaterialLightProperty(0.5f, 0.5f, 0.5f, 1.0f);

	Material* material = new Material(*chain_ambient, *chain_diffuse, *chain_specular, *chain_emmisive);
	material->add2DTexture(*grass_texture);
	
	// Load the chain model.
	std::pair<AnimatedModel*, std::map<aiTextureType, std::vector<Texture*>* >* > chain_obj = AssimpLoader::LoadModel(scene_manager, "data/models/Pandora/misc/small_chain.obj");
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
		current_direction = glm::rotate(current_direction, total_pitch, glm::vec3(0, 1, 0));
		current_direction = glm::rotate(current_direction, yaw, glm::vec3(1, 0, 0));
		
		glm::vec4 current_location2 = current_direction * glm::vec4(0.0f, 0.0f, -0.4f, 1.0f);
		current_location += glm::vec3(current_location2);
		
		rotation += 55;
		
		glm::mat4 chain_location = glm::translate(glm::mat4(1.0f), current_location);
		chain_location = glm::rotate(chain_location, total_pitch, glm::vec3(0, 1, 0));
		chain_location = glm::rotate(chain_location, yaw, glm::vec3(1, 0, 0));
		chain_location = glm::rotate(chain_location, rotation, glm::vec3(0, 0, 1));
		
		//ChainLink* chain_link = new ChainLink(scene_manager, *this, chain_location);
		Entity* chain_link = new Entity(scene_manager, this, chain_location, OBSTACLE, "chainlink");
		//chain->addChild(*chain_link);
		//chain_links_.push_back(chain_link);
		
		SceneNode* node = new SceneNode(*scene_manager_, this, chain_location);
		SceneLeafModel* col_slm = new SceneLeafModel(*node, NULL, *chain_obj.first, *material, BasicShadowShader::getShader(), false, false);
		
		node->updateTransformations();
		std::cout << "(" << node->getGlobalLocation().x << ", " << node->getGlobalLocation().y << node->getGlobalLocation().z << ")" << std::endl;
		
		// Create a bounding box per chain.
		BoxCollision* bc = new BoxCollision(*chain_link, 0.2f, 0.1f, 0.8f);
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
