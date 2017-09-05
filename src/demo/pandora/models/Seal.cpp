#include "Seal.h"
#define GLX_GLXEXT_LEGACY //Must be declared so that our local glxext.h is picked up, rather than the system one
#define GLFW_NO_GLU // Do not allow GL/glfw to include the gl.h header

#include "GL/glew.h"
#include "GL/glfw.h"

#define _USE_MATH_DEFINES
#ifdef _WIN32
#include <math.h>
#else
#include <cmath>
#endif

#include <glm/glm.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtc/matrix_transform.hpp> 
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include "../../../core/entities/camera/Camera.h"
#include "../../../core/scene/SceneNode.h"
#include "../../../core/scene/SceneManager.h"
#include "../../../shapes/terrain.h"
#include "../../../shapes/Line.h"
#include "../../../core/collision/BoxCollision.h"
#include "../../../core/collision/CollisionInfo.h"
#include "../../../core/scene/portal/Region.h"
#include "../../../core/scene/SceneLeafModel.h"
#include "../../../core/shaders/LineShader.h"
#include "../../../core/shaders/BasicShadowShader.h"

Seal::Seal(SceneNode* parent, const glm::mat4& transformation, SceneManager& scene_manager)
	: Entity(scene_manager, parent, transformation, OBSTACLE, "shark")
{
	
}

void Seal::init(Material& material, ShaderInterface& shader)
{
	
}

void Seal::prepare(float dt)
{
	SceneNode::prepare(dt);
}

void Seal::onCollision(const CollisionInfo& collision_info)
{
	
}
