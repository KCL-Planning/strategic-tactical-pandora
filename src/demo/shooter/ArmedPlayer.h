#ifndef DEMO_SHOOTER_ARMED_PLAYER_H
#define DEMO_SHOOTER_ARMED_PLAYER_H

#include "../../core/entities/Player.h"

class Shape;
class Material;
class Texture;

class ArmedPlayer : public Player
{
public:
	ArmedPlayer(SceneNode* parent, const glm::mat4& transformation, float height, SceneNode& scene_node, const Terrain& terrain, SceneManager& scene_manager, Texture& bullet_texture);

	void prepare(float dt);

	Entity* getLastBullet() const { return last_fired_bullet_; }
private:
	Entity* last_fired_bullet_;
	float last_fired_time_;
	Texture* bullet_texture_;
	
	static Shape* bullet_shape_;
	static Material* bullet_material_;
};

#endif
