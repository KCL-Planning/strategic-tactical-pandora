#include "FrustumEntity.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <GL/glew.h>
#include <GL/glfw.h>

#include "../../core/scene/Material.h"
#include "../../core/scene/SceneLeafModel.h"
#include "../../core/scene/SceneNode.h"
#include "../../core/scene/portal/Portal.h"
#include "../../core/scene/portal/Region.h"
#include "../../core/scene/frustum/Frustum.h"
#include "../../core/entities/camera/Camera.h"
#include "../../core/shaders/LineShader.h"
#include "../../core/shaders/BasicShadowShader.h"

#include "../../shapes/Line.h"

FrustumEntity::FrustumEntity(SceneManager& scene_manager, Camera& camera)
	: Entity(scene_manager, NULL, glm::mat4(1.0f), OBSTACLE, "Frustum Entity"), camera_(&camera)
{
	line_ = new Line(false);
	
	MaterialLightProperty* ambient = new MaterialLightProperty(0, 0, 0, 0);
	MaterialLightProperty* diffuse = new MaterialLightProperty(0, 0, 0, 0);
	MaterialLightProperty* specular = new MaterialLightProperty(0, 0, 0, 0);
	MaterialLightProperty* emmisive = new MaterialLightProperty(1, 1, 0, 0.8f);
	Material* material = new Material(*ambient, *diffuse, *specular, *emmisive);
	
	SceneLeafModel* path_ = new SceneLeafModel(*this, NULL, *line_, *material, LineShader::getShader(), true, true);

	frustum_ = new Frustum(glm::mat4(1.0f));
}

void FrustumEntity::prepare(float dt)
{
	if (glfwGetKey('L') != GLFW_PRESS)
	{
		return;
	}
	// Update the lines that represent the frustums.
	std::vector<glm::vec3> line_segments;
	std::vector<glm::vec3> portal_segments;

	glm::mat4 camera_frustum(1.0f);
	//camera_frustum = camera_->getPerspectiveMatrix() * glm::inverse(camera_->getCompleteTransformation());
	camera_frustum = camera_->getPerspectiveMatrix() * glm::inverse(camera_->getCompleteTransformation());
	frustum_->setFrustum(camera_frustum);

	Region* region = Region::findRegionGlobal(getGlobalLocation());
	if (region == NULL)
	{
		return;
	}

	std::vector<const Portal*> processed_portals;
	//region->getRenderingFrustum(*frustum_, camera_->getGlobalLocation(), portal_segments);
	region->getRenderingPortals(*frustum_, camera_->getGlobalLocation(), portal_segments, processed_portals);

	{
	//std::stringstream ss;
	//ss << "Camera location: " << camera_->getGlobalLocation().x << ", " << camera_->getGlobalLocation().y << ", " << camera_->getGlobalLocation().z << ")" << std::endl;
	//OutputDebugString(ss.str().c_str());
	}

	for (std::vector<Region*>::const_iterator ci = Region::getAllRegions().begin(); ci != Region::getAllRegions().end(); ++ci)
	{
		const Region* region = *ci;
		for (std::vector<Portal*>::const_iterator ci = region->getPortals().begin(); ci != region->getPortals().end(); ++ci)
		{
			Portal* portal = *ci;
			Frustum frustum = portal->getRenderingFrustum(*frustum_, camera_->getGlobalLocation(), portal_segments);

			// Calculate the 8 points that make up the frustums.

			// 0 = near
			// 1 = far
			// 2 = right
			// 3 = left
			// 4 = top
			// 5 = bottom

			glm::mat3 m(1.0f);

			// left_bottom_near
			m[0][0] = frustum.getPlanes()[3].x;
			m[1][0] = frustum.getPlanes()[3].y;
			m[2][0] = frustum.getPlanes()[3].z;

			m[0][1] = frustum.getPlanes()[5].x;
			m[1][1] = frustum.getPlanes()[5].y;
			m[2][1] = frustum.getPlanes()[5].z;

			m[0][2] = frustum.getPlanes()[0].x;
			m[1][2] = frustum.getPlanes()[0].y;
			m[2][2] = frustum.getPlanes()[0].z;
			
			glm::vec3 left_bottom_near = glm::inverse(m) * -glm::vec3(frustum.getPlanes()[3].w, frustum.getPlanes()[5].w, frustum.getPlanes()[0].w);

			// right_bottom_near
			m[0][0] = frustum.getPlanes()[2].x;
			m[1][0] = frustum.getPlanes()[2].y;
			m[2][0] = frustum.getPlanes()[2].z;

			m[0][1] = frustum.getPlanes()[5].x;
			m[1][1] = frustum.getPlanes()[5].y;
			m[2][1] = frustum.getPlanes()[5].z;

			m[0][2] = frustum.getPlanes()[0].x;
			m[1][2] = frustum.getPlanes()[0].y;
			m[2][2] = frustum.getPlanes()[0].z;
			
			glm::vec3 right_bottom_near = glm::inverse(m) * -glm::vec3(frustum.getPlanes()[2].w, frustum.getPlanes()[5].w, frustum.getPlanes()[0].w);

			// right_top_near
			m[0][0] = frustum.getPlanes()[2].x;
			m[1][0] = frustum.getPlanes()[2].y;
			m[2][0] = frustum.getPlanes()[2].z;

			m[0][1] = frustum.getPlanes()[4].x;
			m[1][1] = frustum.getPlanes()[4].y;
			m[2][1] = frustum.getPlanes()[4].z;

			m[0][2] = frustum.getPlanes()[0].x;
			m[1][2] = frustum.getPlanes()[0].y;
			m[2][2] = frustum.getPlanes()[0].z;
			
			glm::vec3 right_top_near = glm::inverse(m) * -glm::vec3(frustum.getPlanes()[2].w, frustum.getPlanes()[4].w, frustum.getPlanes()[0].w);

			// left_top_near
			m[0][0] = frustum.getPlanes()[3].x;
			m[1][0] = frustum.getPlanes()[3].y;
			m[2][0] = frustum.getPlanes()[3].z;

			m[0][1] = frustum.getPlanes()[4].x;
			m[1][1] = frustum.getPlanes()[4].y;
			m[2][1] = frustum.getPlanes()[4].z;

			m[0][2] = frustum.getPlanes()[0].x;
			m[1][2] = frustum.getPlanes()[0].y;
			m[2][2] = frustum.getPlanes()[0].z;
			
			glm::vec3 left_top_near = glm::inverse(m) * -glm::vec3(frustum.getPlanes()[3].w, frustum.getPlanes()[4].w, frustum.getPlanes()[0].w);


			// left_bottom_far
			m[0][0] = frustum.getPlanes()[3].x;
			m[1][0] = frustum.getPlanes()[3].y;
			m[2][0] = frustum.getPlanes()[3].z;

			m[0][1] = frustum.getPlanes()[5].x;
			m[1][1] = frustum.getPlanes()[5].y;
			m[2][1] = frustum.getPlanes()[5].z;

			m[0][2] = frustum.getPlanes()[1].x;
			m[1][2] = frustum.getPlanes()[1].y;
			m[2][2] = frustum.getPlanes()[1].z;
			
			glm::vec3 left_bottom_far = glm::inverse(m) * -glm::vec3(frustum.getPlanes()[3].w, frustum.getPlanes()[5].w, frustum.getPlanes()[1].w);

			// right_bottom_far
			m[0][0] = frustum.getPlanes()[2].x;
			m[1][0] = frustum.getPlanes()[2].y;
			m[2][0] = frustum.getPlanes()[2].z;

			m[0][1] = frustum.getPlanes()[5].x;
			m[1][1] = frustum.getPlanes()[5].y;
			m[2][1] = frustum.getPlanes()[5].z;

			m[0][2] = frustum.getPlanes()[1].x;
			m[1][2] = frustum.getPlanes()[1].y;
			m[2][2] = frustum.getPlanes()[1].z;
			
			glm::vec3 right_bottom_far = glm::inverse(m) * -glm::vec3(frustum.getPlanes()[2].w, frustum.getPlanes()[5].w, frustum.getPlanes()[1].w);

			// right_top_far
			m[0][0] = frustum.getPlanes()[2].x;
			m[1][0] = frustum.getPlanes()[2].y;
			m[2][0] = frustum.getPlanes()[2].z;

			m[0][1] = frustum.getPlanes()[4].x;
			m[1][1] = frustum.getPlanes()[4].y;
			m[2][1] = frustum.getPlanes()[4].z;

			m[0][2] = frustum.getPlanes()[1].x;
			m[1][2] = frustum.getPlanes()[1].y;
			m[2][2] = frustum.getPlanes()[1].z;
			
			glm::vec3 right_top_far = glm::inverse(m) * -glm::vec3(frustum.getPlanes()[2].w, frustum.getPlanes()[4].w, frustum.getPlanes()[1].w);

			// left_top_far
			m[0][0] = frustum.getPlanes()[3].x;
			m[1][0] = frustum.getPlanes()[3].y;
			m[2][0] = frustum.getPlanes()[3].z;

			m[0][1] = frustum.getPlanes()[4].x;
			m[1][1] = frustum.getPlanes()[4].y;
			m[2][1] = frustum.getPlanes()[4].z;

			m[0][2] = frustum.getPlanes()[1].x;
			m[1][2] = frustum.getPlanes()[1].y;
			m[2][2] = frustum.getPlanes()[1].z;
			
			glm::vec3 left_top_far = glm::inverse(m) * -glm::vec3(frustum.getPlanes()[3].w, frustum.getPlanes()[4].w, frustum.getPlanes()[1].w);
			/*
			std::stringstream ss;
			
			ss << "LBN: " << left_bottom_near.x << ", " << left_bottom_near.y << ", " << left_bottom_near.z << ")" << std::endl;
			ss << "RBN: " << right_bottom_near.x << ", " << right_bottom_near.y << ", " << right_bottom_near.z << ")" << std::endl;
			ss << "RTN: " << right_top_near.x << ", " << right_top_near.y << ", " << right_top_near.z << ")" << std::endl;
			ss << "LTN: " << left_top_near.x << ", " << left_top_near.y << ", " << left_top_near.z << ")" << std::endl;

			ss << "LBF: " << left_bottom_far.x << ", " << left_bottom_far.y << ", " << left_bottom_far.z << ")" << std::endl;
			ss << "RBF: " << right_bottom_far.x << ", " << right_bottom_far.y << ", " << right_bottom_far.z << ")" << std::endl;
			ss << "RTF: " << right_top_far.x << ", " << right_top_far.y << ", " << right_top_far.z << ")" << std::endl;
			ss << "LTF: " << left_top_far.x << ", " << left_top_far.y << ", " << left_top_far.z << ")" << std::endl;

			OutputDebugString(ss.str().c_str());
			*/
			// Near plane lines.
			line_segments.push_back(left_bottom_near);
			line_segments.push_back(right_bottom_near);

			line_segments.push_back(right_bottom_near);
			line_segments.push_back(right_top_near);

			line_segments.push_back(right_top_near);
			line_segments.push_back(left_top_near);

			line_segments.push_back(left_top_near);
			line_segments.push_back(left_bottom_near);
			
			// Far plane lines.
			line_segments.push_back(left_bottom_far);
			line_segments.push_back(right_bottom_far);
			
			line_segments.push_back(right_bottom_far);
			line_segments.push_back(right_top_far);
			
			line_segments.push_back(right_top_far);
			line_segments.push_back(left_top_far);

			line_segments.push_back(left_top_far);
			line_segments.push_back(left_bottom_far);

			// Other lines.
			line_segments.push_back(left_bottom_near);
			line_segments.push_back(left_bottom_far);

			line_segments.push_back(right_bottom_near);
			line_segments.push_back(right_bottom_far);

			line_segments.push_back(right_top_near);
			line_segments.push_back(right_top_far);

			line_segments.push_back(left_top_near);
			line_segments.push_back(left_top_far);
		}
	}

	//line_->setVertexBuffer(line_segments);
	line_->setVertexBuffer(portal_segments);
}
