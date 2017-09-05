#ifndef DEMO_REGION_TEST_FRUSTUM_ENTITY_H
#define DEMO_REGION_TEST_FRUSTUM_ENTITY_H

#include <glm/glm.hpp>

#include "../../core/entities/Entity.h"

class Camera;
class Line;
class SceneManager;
class Frustum;

class FrustumEntity : public Entity
{
public:
	FrustumEntity(SceneManager& scene_manager, Camera& camera);

	void prepare(float dt);
private:
	Camera* camera_;
	Line* line_;
	Frustum* frustum_;
};

#endif
