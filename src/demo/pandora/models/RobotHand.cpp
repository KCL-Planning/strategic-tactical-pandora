#include "RobotHand.h"
#include "../shaders/CausticShader.h"

#include <glm/gtc/matrix_transform.hpp>

#include "dpengine/shapes/Cube.h"
#include "dpengine/shapes/Cylinder.h"
#include "dpengine/scene/SceneLeafModel.h"
#include "dpengine/scene/Material.h"
#include "dpengine/texture/TargaTexture.h"

RobotHand::RobotHand(DreadedPE::SceneManager& scene_manager, DreadedPE::SceneNode* parent, const glm::mat4& transformation)
	: DreadedPE::Entity(scene_manager, parent, transformation, DreadedPE::OBSTACLE, "RobotHand"), fold_(0), unfold_arm_(false), hand_angle_(0), arm_is_unfolded_(false)
{
	//Cube* arm_cube = new Cube(0.1f, 0.1f, 0.5f);
	std::shared_ptr<DreadedPE::Cylinder> arm_cube(std::make_shared<DreadedPE::Cylinder>(0.5f, 0.1f, 16));
	std::shared_ptr<DreadedPE::Cube> hand_cube(std::make_shared<DreadedPE::Cube>(0.05f, 0.01f, 0.25f));
	
	first_half_ = new DreadedPE::SceneNode(scene_manager, this, glm::mat4(1.0f));
	DreadedPE::SceneNode* first_arm_anchor = new DreadedPE::SceneNode(scene_manager, first_half_, glm::rotate(glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, -0.25f)), glm::radians(90.0f), glm::vec3(1, 0, 0)));
	DreadedPE::SceneNode* second_half_anchor = new DreadedPE::SceneNode(scene_manager, first_half_, glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, -0.0f)));
	second_half_ = new DreadedPE::SceneNode(scene_manager, second_half_anchor, glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, -0.5f)));
	DreadedPE::SceneNode* second_arm_anchor = new DreadedPE::SceneNode(scene_manager, second_half_, glm::rotate(glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, -0.25f)), glm::radians(90.0f), glm::vec3(1, 0, 0)));
	hand_ = new DreadedPE::SceneNode(scene_manager, second_half_, glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, -0.5f)));
	
	DreadedPE::SceneNode* upper_hand_node = new DreadedPE::SceneNode(scene_manager, hand_, glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.05f, 0.0f)));
	DreadedPE::SceneNode* lower_hand_node = new DreadedPE::SceneNode(scene_manager, hand_, glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, -0.05f, 0.0f)));
	
	DreadedPE::MaterialLightProperty ambient(0.8, 0.8, 0.8, 1.0);
	DreadedPE::MaterialLightProperty diffuse(0.6, 0.6, 0.6, 1.0);
	DreadedPE::MaterialLightProperty specular(0.3, 0.3, 0.3, 1.0);
	DreadedPE::MaterialLightProperty emmisive(0.5, 0.5, 0.5, 1.0);

	DreadedPE::Texture* texture = DreadedPE::TargaTexture::loadTexture("data/textures/grass.tga");
	
	std::shared_ptr<DreadedPE::Material> material(std::make_shared<DreadedPE::Material>(ambient, diffuse, specular, emmisive));
	material->add2DTexture(*texture);
	
	DreadedPE::SceneLeafModel* first_arm_model = new DreadedPE::SceneLeafModel(*first_arm_anchor, NULL, arm_cube, material, CausticShader::getShader(), false, false, DreadedPE::OBJECT, DreadedPE::ShadowRenderer::STATIC_SHADOW);
	DreadedPE::SceneLeafModel* second_arm_model = new DreadedPE::SceneLeafModel(*second_arm_anchor, NULL, arm_cube, material, CausticShader::getShader(), false, false, DreadedPE::OBJECT, DreadedPE::ShadowRenderer::STATIC_SHADOW);
	DreadedPE::SceneLeafModel* upper_hand_model = new DreadedPE::SceneLeafModel(*upper_hand_node, NULL, hand_cube, material, CausticShader::getShader(), false, false, DreadedPE::OBJECT, DreadedPE::ShadowRenderer::STATIC_SHADOW);
	DreadedPE::SceneLeafModel* lower_hand_model = new DreadedPE::SceneLeafModel(*lower_hand_node, NULL, hand_cube, material, CausticShader::getShader(), false, false, DreadedPE::OBJECT, DreadedPE::ShadowRenderer::STATIC_SHADOW);
}

void RobotHand::unfoldArm(bool unfold_arm)
{
	unfold_arm_ = unfold_arm;
}

void RobotHand::prepare(float dt)
{
	if (!unfold_arm_ && fold_ <= 70)
	{
		fold_ += 5 * dt;
		first_half_->setTransformation(glm::rotate(first_half_->getLocalTransformation(), glm::radians(dt * 5), glm::vec3(0, 1, 0)));
		second_half_->setTransformation(glm::rotate(second_half_->getLocalTransformation(), glm::radians(dt * 10), glm::vec3(0, -1, 0)));
		arm_is_unfolded_ = false;
	}
	else if (fold_ > 0 && unfold_arm_)
	{
		fold_ -= 5 * dt;
		first_half_->setTransformation(glm::rotate(first_half_->getLocalTransformation(), glm::radians(-dt * 5), glm::vec3(0, 1, 0)));
		second_half_->setTransformation(glm::rotate(second_half_->getLocalTransformation(), glm::radians(-dt * 10), glm::vec3(0, -1, 0)));
		arm_is_unfolded_ = false;
	}
	else if (unfold_arm_)
	{
		arm_is_unfolded_ = true;
	}
	
	if (hand_angle_ != 0)
	{
		hand_->setTransformation(glm::rotate(hand_->getLocalTransformation(), glm::radians(10 * dt), glm::vec3(0, 0, 1)));
	}
	
	DreadedPE::Entity::prepare(dt);
}
