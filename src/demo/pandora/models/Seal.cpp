#include "Seal.h"
#define GLX_GLXEXT_LEGACY //Must be declared so that our local glxext.h is picked up, rather than the system one
#define GLFW_NO_GLU // Do not allow GL/glfw to include the gl.h header

#include "GL/glew.h"

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

#include "dpengine/entities/camera/Camera.h"
#include "dpengine/scene/SceneNode.h"
#include "dpengine/scene/SceneManager.h"
#include "dpengine/shapes/terrain.h"
#include "dpengine/shapes/Line.h"
#include "dpengine/collision/ConvexPolygon.h"
#include "dpengine/collision/CollisionInfo.h"
#include "dpengine/scene/portal/Region.h"
#include "dpengine/scene/SceneLeafModel.h"
#include "dpengine/shaders/LineShader.h"
#include "dpengine/shaders/BasicShadowShader.h"

Seal::Seal(DreadedPE::SceneNode* parent, const glm::mat4& transformation, DreadedPE::SceneManager& scene_manager)
	: DreadedPE::Entity(scene_manager, parent, transformation, DreadedPE::OBSTACLE, "shark")
{
	
}

void Seal::init(DreadedPE::Material& material, DreadedPE::ShaderInterface& shader)
{
	
}

void Seal::prepare(float dt)
{
	SceneNode::prepare(dt);
}

void Seal::onCollision(const DreadedPE::CollisionInfo& collision_info)
{
	
}
