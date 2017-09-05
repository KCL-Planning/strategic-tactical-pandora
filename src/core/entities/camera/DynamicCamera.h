#ifndef CORE_SCENE_CAMERA_DYNAMIC_CAMERA_H
#define CORE_SCENE_CAMERA_DYNAMIC_CAMERA_H

#include <glm/glm.hpp>
//#include "../Entity.h"
#include "Camera.h"

//class Camera;
class SceneNode;
class SceneManager;

/**
 * Have a camera that is attached to some entity which we want to watch. But make sure
 * it does not collide with any of the surrounding area.
 */
class DynamicCamera : public Camera
{
public:
	DynamicCamera(SceneNode& to_follow, SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transformation, float fov, float width ,float height, float near_plane, float far_plane);
	~DynamicCamera();
	virtual void prepare(float dt);
	//Camera& getCamera() const { return *camera_; }

	void setTargetToFollow(SceneNode& to_follow, SceneNode* parent) { to_follow_ = &to_follow; parent_ = parent; }
private:
	SceneNode* to_follow_;
	//Camera* camera_;
	glm::vec3 prefered_transformation_;

	float yaw_, roll_, pitch_;

	bool moved_camera_last_frame_;
	void mouseScrollCallBack(double xoffset, double yoffset);
};

#endif
