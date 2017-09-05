#ifndef CAMERA_H
#define CAMERA_H
#define _USE_MATH_DEFINES
#include <cmath>

#include <glm/glm.hpp>

#include "../Entity.h"
#include "../../renderer/FrustumCaster.h"
#include "../../renderer/ShadowRenderer.h"

// Set of functions that control the camera.

enum DIRECTION { NONE, FORWARD, BACKWARD, LEFT, RIGHT };

class Camera : public Entity, public FrustumCaster
{
public:
	/**
	 * Create a camera.
	 * @param scene_manager The scene manager.
	 * @param parent The parent this camera is part of.
	 * @param transformation The transformation applied to this camera relative to its @ref{parent}.
	 * @param fov The field of view.
	 * @param aspect_ratio The aspect ratio.
	 * @param near_plane The distance of the near plane to the camera.
	 * @param far_plane The distance  to the far plane of the camera.
	 */
	Camera(SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transformation, float fov, float width, float height, float near_plane, float far_plane);
	
	virtual glm::mat4 getViewMatrix() const;

	// Make sure the parameters don't exceed our camera limits.
	static void checkLimits(float& pitch, float& yaw, float& roll);

	virtual glm::vec3 getLocation() const;
	virtual glm::mat4 getPerspectiveMatrix() const;

	const glm::mat4& getShadowMatrix() const { return shadow_matrix_; }

	float getFOV() const { return fov_; }
	float getWidth() const { return width_; }
	float getHeight() const { return height_; }
	float getNearPlane() const { return near_plane_; }
	float getFarPlane() const { return far_plane_; }

	void onResize(int width, int height);

private:
	glm::mat4 perspective_matrix_;
	glm::mat4 shadow_matrix_;
	float fov_, width_, height_, near_plane_, far_plane_;
};

#endif
