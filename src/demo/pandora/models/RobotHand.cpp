#include "RobotHand.h"
#include "../shaders/CausticShader.h"

#include "../../../shapes/Cube.h"
#include "../../../shapes/Cylinder.h"
#include "../../../core/scene/SceneLeafModel.h"
#include "../../../core/scene/Material.h"
#include "../../../core/texture/TargaTexture.h"

RobotHand::RobotHand(SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transformation)
	: Entity(scene_manager, parent, transformation, OBSTACLE, "RobotHand"), fold_(0), unfold_arm_(false), hand_angle_(0), arm_is_unfolded_(false)
{
	//Cube* arm_cube = new Cube(0.1f, 0.1f, 0.5f);
	Cylinder* arm_cube = new Cylinder(0.5f, 0.1f, 16);
	Cube* hand_cube = new Cube(0.05f, 0.01f, 0.25f);
	
	first_half_ = new SceneNode(scene_manager, this, glm::mat4(1.0f));
	SceneNode* first_arm_anchor = new SceneNode(scene_manager, first_half_, glm::rotate(glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, -0.25f)), 90.0f, glm::vec3(1, 0, 0)));
	SceneNode* second_half_anchor = new SceneNode(scene_manager, first_half_, glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, -0.0f)));
	second_half_ = new SceneNode(scene_manager, second_half_anchor, glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, -0.5f)));
	SceneNode* second_arm_anchor = new SceneNode(scene_manager, second_half_, glm::rotate(glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, -0.25f)), 90.0f, glm::vec3(1, 0, 0)));
	hand_ = new SceneNode(scene_manager, second_half_, glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, -0.5f)));
	
	SceneNode* upper_hand_node = new SceneNode(scene_manager, hand_, glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.05f, 0.0f)));
	SceneNode* lower_hand_node = new SceneNode(scene_manager, hand_, glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, -0.05f, 0.0f)));
	
	MaterialLightProperty* ambient = new MaterialLightProperty(0.8, 0.8, 0.8, 1.0);
	MaterialLightProperty* diffuse = new MaterialLightProperty(0.6, 0.6, 0.6, 1.0);
	MaterialLightProperty* specular = new MaterialLightProperty(0.3, 0.3, 0.3, 1.0);
	MaterialLightProperty* emmisive = new MaterialLightProperty(0.5, 0.5, 0.5, 1.0);

	Texture* texture = TargaTexture::loadTexture("data/textures/grass.tga");
	
	Material* material = new Material(*ambient, *diffuse, *specular, *emmisive);
	material->add2DTexture(*texture);
	
	SceneLeafModel* first_arm_model = new SceneLeafModel(*first_arm_anchor, NULL, *arm_cube, *material, CausticShader::getShader(), false, false, OBJECT, ShadowRenderer::STATIC_SHADOW);
	SceneLeafModel* second_arm_model = new SceneLeafModel(*second_arm_anchor, NULL, *arm_cube, *material, CausticShader::getShader(), false, false, OBJECT, ShadowRenderer::STATIC_SHADOW);
	SceneLeafModel* upper_hand_model = new SceneLeafModel(*upper_hand_node, NULL, *hand_cube, *material, CausticShader::getShader(), false, false, OBJECT, ShadowRenderer::STATIC_SHADOW);
	SceneLeafModel* lower_hand_model = new SceneLeafModel(*lower_hand_node, NULL, *hand_cube, *material, CausticShader::getShader(), false, false, OBJECT, ShadowRenderer::STATIC_SHADOW);
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
		first_half_->setTransformation(glm::rotate(first_half_->getLocalTransformation(), dt * 5, glm::vec3(0, 1, 0)));
		second_half_->setTransformation(glm::rotate(second_half_->getLocalTransformation(), dt * 10, glm::vec3(0, -1, 0)));
		arm_is_unfolded_ = false;
	}
	else if (fold_ > 0 && unfold_arm_)
	{
		fold_ -= 5 * dt;
		first_half_->setTransformation(glm::rotate(first_half_->getLocalTransformation(), -dt * 5, glm::vec3(0, 1, 0)));
		second_half_->setTransformation(glm::rotate(second_half_->getLocalTransformation(), -dt * 10, glm::vec3(0, -1, 0)));
		arm_is_unfolded_ = false;
	}
	else if (unfold_arm_)
	{
		arm_is_unfolded_ = true;
	}
	
	if (hand_angle_ != 0)
	{
		hand_->setTransformation(glm::rotate(hand_->getLocalTransformation(), 10 * dt, glm::vec3(0, 0, 1)));
	}
	
	Entity::prepare(dt);
}
