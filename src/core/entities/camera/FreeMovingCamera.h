#ifndef CORE_ENTITIES_CAMERA_FREE_MOVING_CAMERA_H
#define CORE_ENTITIES_CAMERA_FREE_MOVING_CAMERA_H
#define _USE_MATH_DEFINES
#include <cmath>

#include <glm/glm.hpp>

#include "Camera.h"

// Set of functions that control the camera.

class FreeMovingCamera : public Camera
{
public:
	//Camera();
	FreeMovingCamera(SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transformation, float fov, float width ,float height, float near_plane, float far_plane);
	
	// Handle the user input and update where the camera is looking.
	virtual void prepare(float dt);

private:
	float yaw_, pitch_, roll_;
};

#endif
