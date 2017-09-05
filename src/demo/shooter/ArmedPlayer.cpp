#ifdef _WIN32
#include <windows.h>
#endif

#include "GL/glew.h"
#include "GL/glfw.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/vector_angle.hpp>

#include "ArmedPlayer.h"
#include "../../core/scene/SceneManager.h"
#include "../../core/scene/SceneLeafModel.h"
#include "../../core/entities/camera/Camera.h"
#include "../../core/entities/behaviours/MoveBehaviour.h"
#include "../../shapes/Cube.h"
#include "../../core/scene/Material.h"
#include "../../core/shaders/BasicShadowShader.h"
#include "../../core/shaders/ShadowShader.h"
#include "../../core/shaders/CreateAnimatedShadowMapShader.h"
#include "../../core/scene/portal/Region.h"
#include "../../core/math/BoundedBox.h"
#include "../../core/texture/Texture.h"
#include "Bullet.h"

ArmedPlayer::ArmedPlayer(SceneNode* parent, const glm::mat4& transformation, float height, SceneNode& scene_node, const Terrain& terrain, SceneManager& scene_manager, Texture& bullet_texture)
	: Player(parent, transformation, height, scene_node, terrain, scene_manager), last_fired_bullet_(NULL), last_fired_time_(0.0f), bullet_texture_(&bullet_texture)
{
	
}

void ArmedPlayer::prepare(float dt)
{
	Player::prepare(dt);
	last_fired_time_ += dt;
	// Check if we shot anything :)).
	if (glfwGetKey('Z') == GLFW_PRESS && last_fired_time_ > 0.3f)
	{
		last_fired_time_ = 0.0f;
		glm::vec3 direction(0.0f, 0.0f, -1.0f);

		Region* region = NULL;
		if (current_region_ == NULL)
		{
			region = Region::findRegionGlobal(getGlobalLocation());
		}
		else
		{
			region = current_region_;
		}

		if (region == NULL)
		{
#ifdef _WIN32
			MessageBox(NULL, "Unable to find the region to fire the bullet in!", "An error occurred", MB_ICONERROR | MB_OK);
#endif
			last_fired_bullet_ = new Bullet(*scene_manager_, NULL, glm::translate(glm::mat4(1.0f), getGlobalLocation()), OBSTACLE, "Bullet", false);
			//scene_manager_->getRoot().addChild(*last_fired_bullet_);
		}
		else
		{
			last_fired_bullet_ = new Bullet(*scene_manager_, &region->getSceneNode(), glm::translate(glm::mat4(1.0f), getGlobalLocation() - region->getSceneNode().getGlobalLocation()), OBSTACLE, "Bullet", true);
			//region->getSceneNode().addChild(*last_fired_bullet_);
		}

		if (bullet_shape_ == NULL)
		{
			bullet_shape_ = new Cube(0.2f, 0.2f, 0.3f);
		
			MaterialLightProperty* bright_ambient = new MaterialLightProperty(1.0, 1.0, 1.0, 1.0);
			MaterialLightProperty* bright_diffuse = new MaterialLightProperty(0.0, 0.0, 0.0, 1.0);
			MaterialLightProperty* bright_specular = new MaterialLightProperty(0.0, 0.0, 0.0, 1.0);
			MaterialLightProperty* bright_emmisive = new MaterialLightProperty(0.0, 1.0, 1.0, 1.0);

			bullet_material_ = new Material(*bright_ambient, *bright_diffuse, *bright_specular, *bright_emmisive);
			bullet_material_->add2DTexture(*bullet_texture_);
		}

		BoundedBox* visibility_bounded_box = new BoundedBox(*last_fired_bullet_, 0.2f, 0.2f, 0.3f);
		BoundedBox* collision_bounded_box = new BoundedBox(*last_fired_bullet_, 0.2f, 0.2f, 0.3f);
		last_fired_bullet_->setCollisionBoundedBox(*collision_bounded_box);
		//last_fired_bullet_->frustum_checker_ = visibility_bounded_box;
		//SceneLeafModel* stair_step_leaf_node = new SceneLeafModel(*last_fired_bullet_, visibility_bounded_box, *bullet_shape_, *bullet_material_, BasicShadowShader::getShader(), &ShadowShader::getShader(), false, false);
		SceneLeafModel* stair_step_leaf_node = new SceneLeafModel(*last_fired_bullet_, visibility_bounded_box, *bullet_shape_, *bullet_material_, BasicShadowShader::getShader(), false, false);
		
		std::vector<const SceneNode*> excluded_nodes;
		last_fired_bullet_->initialiseBoundedBoxes(excluded_nodes);

		glm::vec3 bullet_velocity(0.0f, 0.0f, -1.0f);
		bullet_velocity = glm::vec3(getCompleteTransformation() * glm::vec4(bullet_velocity, 0.0f));
		//bullet_velocity = glm::rotate(bullet_velocity, camera_->getRoll(), glm::vec3(0.0f, 0.0f, 1.0f));
		//bullet_velocity = glm::rotate(bullet_velocity, camera_->getYaw(), glm::vec3(0.0f, 1.0f, 0.0f));
		//bullet_velocity = glm::rotate(bullet_velocity, camera_->getPitch(), glm::vec3(1.0f, 0.0f, 0.0f));

		MoveBehaviour* bullet_move_behaviour = new MoveBehaviour(*last_fired_bullet_, bullet_velocity);
		last_fired_bullet_->addBehaviour(*bullet_move_behaviour);
		scene_manager_->addUpdateableEntity(*last_fired_bullet_);


		//scene_manager_->getRoot().addChild(*last_fired_bullet_);
	}
}

Shape* ArmedPlayer::bullet_shape_ = NULL;
Material* ArmedPlayer::bullet_material_ = NULL;
