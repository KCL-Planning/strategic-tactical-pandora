#ifdef _WIN32
#include <windows.h>
#endif
#include <iostream>
#include <algorithm>

#include <glm/gtx/quaternion.hpp>

#include <math.h>

#include "BoundedBox.h"

#include "../entities/Entity.h"
#include "../scene/SceneNode.h"
#include "../scene/SceneLeaf.h"
#include "../scene/SceneLeafModel.h"
#include "../scene/SceneLeafLight.h"
#include "../../shapes/Shape.h"
#include "../math/Plane.h"
#include "../scene/frustum/Frustum.h"
#include "../collision/BoxCollision.h"

BoundedBox::BoundedBox(const SceneNode& scene_node, bool check_visibility, const std::vector<const SceneNode*>& excluded_nodes)
	: scene_node_(&scene_node)
{
	// We calculate the box relative the the scene node's location.
	glm::mat4 transformation_graph(1.0f);

	float min_x = std::numeric_limits<float>::max();
	float max_x = -std::numeric_limits<float>::max();
	float min_y = std::numeric_limits<float>::max();
	float max_y = -std::numeric_limits<float>::max();
	float min_z = std::numeric_limits<float>::max();
	float max_z = -std::numeric_limits<float>::max();
	 
	std::string s;
	processSceneNode(scene_node, transformation_graph, min_x, max_x, min_y, max_y, min_z, max_z, check_visibility, excluded_nodes, s);
#ifdef HORROR_GAME_ENABLE_DEBUG
	std::stringstream ss;
	ss << "FINAL RESULT: ";
	const Entity* entity = dynamic_cast<const Entity*>(&scene_node);
	if (entity != NULL)
	{
		ss << entity->getName() << ": ";
	}
	else
	{
		ss << "SCENE_NODE: ";
	}
	ss << min_x << "-" << max_x << ", " << min_y << "-" << max_y << ", " << min_z << "-" << max_z;
#ifdef _WIN32
	MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#endif
#endif
	initialise(min_x, max_x, min_y, max_y, min_z, max_z);
}

BoundedBox::BoundedBox(const SceneLeafModel& scene_leaf_model)
	: scene_node_(scene_leaf_model.getParent())
{
	float min_x = std::numeric_limits<float>::max();
	float max_x = -std::numeric_limits<float>::max();
	float min_y = std::numeric_limits<float>::max();
	float max_y = -std::numeric_limits<float>::max();
	float min_z = std::numeric_limits<float>::max();
	float max_z = -std::numeric_limits<float>::max();

	for (std::vector<GLuint>::const_iterator ci = scene_leaf_model.getModel().getIndices().begin(); ci != scene_leaf_model.getModel().getIndices().end(); ++ci)
	{
		const glm::vec3& point = scene_leaf_model.getModel().getVertices()[*ci];
		min_x = std::min(min_x, point.x);
		max_x = std::max(max_x, point.x);
		min_y = std::min(min_y, point.y);
		max_y = std::max(max_y, point.y);
		min_z = std::min(min_z, point.z);
		max_z = std::max(max_z, point.z);
	}
	/*
	for (std::vector<glm::vec3>::const_iterator ci = scene_leaf_model.getModel().getVertices().begin(); ci != scene_leaf_model.getModel().getVertices().end(); ++ci)
	{
		const glm::vec3& point = *ci;
		min_x = std::min(min_x, point.x);
		max_x = std::max(max_x, point.x);
		min_y = std::min(min_y, point.y);
		max_y = std::max(max_y, point.y);
		min_z = std::min(min_z, point.z);
		max_z = std::max(max_z, point.z);
	}
	*/
	initialise(min_x, max_x, min_y, max_y, min_z, max_z);
}

BoundedBox::BoundedBox(const SceneNode& scene_node, float width, float height, float depth)
	: scene_node_(&scene_node)
{
	initialise(-width / 2.0f, width / 2.0f, -height / 2.0f, height / 2.0f, -depth / 2.0f, depth / 2.0f);
}

BoundedBox::BoundedBox(const SceneNode& scene_node, 
                       const glm::vec3& bottom_left_away, 
                       const glm::vec3& bottom_right_away, 
                       const glm::vec3& top_left_away, 
                       const glm::vec3& top_right_away, 
                       const glm::vec3& bottom_left_close, 
                       const glm::vec3& bottom_right_close, 
                       const glm::vec3& top_left_close, 
                       const glm::vec3& top_right_close)
	: scene_node_(&scene_node)
{
	points_[0] = bottom_left_away;
	points_[1] = bottom_right_away;
	points_[2] = top_left_away;
	points_[3] = top_right_away;
	points_[4] = bottom_left_close;
	points_[5] = bottom_right_close;
	points_[6] = top_left_close;
	points_[7] = top_right_close;
	
	// Calculate the min / max.
	float min_x = std::numeric_limits<float>::max();
	float max_x = -std::numeric_limits<float>::max();
	float min_y = std::numeric_limits<float>::max();
	float max_y = -std::numeric_limits<float>::max();
	float min_z = std::numeric_limits<float>::max();
	float max_z = -std::numeric_limits<float>::max();

	for (unsigned int i = 0; i < 8; ++i)
	{
		min_x = std::min(min_x, points_[i].x);
		max_x = std::max(min_x, points_[i].x);
		min_y = std::min(min_y, points_[i].y);
		max_y = std::max(min_y, points_[i].y);
		min_z = std::min(min_z, points_[i].z);
		max_z = std::max(min_z, points_[i].z);
	}
	initialise(min_x, max_x, min_y, max_y, min_z, max_z);
}

void BoundedBox::initialise(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z)
{
	float width = max_x - min_x;
	float height = max_y - min_y;
	float depth = max_z - min_z;

	centre_point_ = glm::vec3((max_x + min_x) / 2.0f, (max_y + min_y) / 2.0f, (max_z + min_z) / 2.0f);

	points_[0] = glm::vec3(-width / 2.0f, -height / 2.0f, -depth / 2.0f) + centre_point_;
	points_[1] = glm::vec3(width / 2.0f, -height / 2.0f, -depth / 2.0f) + centre_point_;
	points_[2] = glm::vec3(-width / 2.0f, height / 2.0f, -depth / 2.0f) + centre_point_;
	points_[3] = glm::vec3(width / 2.0f, height / 2.0f, -depth / 2.0f) + centre_point_;
	points_[4] = glm::vec3(-width / 2.0f, -height / 2.0f, depth / 2.0f) + centre_point_;
	points_[5] = glm::vec3(width / 2.0f, -height / 2.0f, depth / 2.0f) + centre_point_;
	points_[6] = glm::vec3(-width / 2.0f, height / 2.0f, depth / 2.0f) + centre_point_;
	points_[7] = glm::vec3(width / 2.0f, height / 2.0f, depth / 2.0f) + centre_point_;

	Plane* p0 = new Plane(points_[0], points_[1], points_[2]);
	Plane* p1 = new Plane(points_[4], points_[6], points_[5]);
	Plane* p2 = new Plane(points_[6], points_[2], points_[7]);
	Plane* p3 = new Plane(points_[4], points_[5], points_[0]);
	Plane* p4 = new Plane(points_[4], points_[0], points_[6]);
	Plane* p5 = new Plane(points_[5], points_[7], points_[1]);
	
	sides_[0] = p0;
	sides_[1] = p1;
	sides_[2] = p2;
	sides_[3] = p3;
	sides_[4] = p4;
	sides_[5] = p5;

	principal_axis_[0] = glm::vec4(width, 0.0f, 0.0f, 0.0f);
	principal_axis_[1] = glm::vec4(0.0f, height, 0.0f, 0.0f);
	principal_axis_[2] = glm::vec4(0.0f, 0.0f, depth, 0.0f);
}

//bool BoundedBox::isInsideFrustum(const SceneNode& node, const Frustum& frustrum) const
bool BoundedBox::isInsideFrustum(const Frustum& frustum) const
{
	glm::vec4 global_centre_point = scene_node_->getCompleteTransformation() * glm::vec4(centre_point_, 1.0f);

	glm::vec4 principal_axis_x = scene_node_->getCompleteTransformation() * principal_axis_[0];
	glm::vec4 principal_axis_y = scene_node_->getCompleteTransformation() * principal_axis_[1];
	glm::vec4 principal_axis_z = scene_node_->getCompleteTransformation() * principal_axis_[2];

	// Check if the middle point of the box is inside the extended frustrum.
	for (std::vector<glm::vec4>::const_iterator ci = frustum.getPlanes().begin(); ci != frustum.getPlanes().end(); ++ci)
	{
		const glm::vec4& plane = *ci;

		// Calculate the effective radius of this box.
		float effective_radius = 0.5f * (glm::abs(glm::dot(plane, principal_axis_x)) + glm::abs(glm::dot(plane, principal_axis_y)) + glm::abs(glm::dot(plane, principal_axis_z)));

		// Check if the centre point of the box is inside the frustrum.
		if (-effective_radius >= glm::dot(global_centre_point, plane))
		{
			return false;
		}
	}
	return true;
}

bool BoundedBox::isInside(const BoundedBox& other) const
{
	std::vector<std::pair<int, int> > line_segments;
	line_segments.push_back(std::make_pair(0, 1));
	line_segments.push_back(std::make_pair(2, 3));
	line_segments.push_back(std::make_pair(0, 2));
	line_segments.push_back(std::make_pair(1, 3));
	
	line_segments.push_back(std::make_pair(4, 5));
	line_segments.push_back(std::make_pair(6, 7));
	line_segments.push_back(std::make_pair(4, 6));
	line_segments.push_back(std::make_pair(5, 7));
	
	line_segments.push_back(std::make_pair(0, 4));
	line_segments.push_back(std::make_pair(1, 5));
	line_segments.push_back(std::make_pair(2, 6));
	line_segments.push_back(std::make_pair(3, 7));
	
	// Transform all the points.
	glm::vec3 transformed_points[8];
	glm::vec3 other_transformed_points[8];
	
	for (unsigned int i = 0; i < 8; ++i)
	{
		transformed_points[i] = glm::vec3(scene_node_->getCompleteTransformation() * glm::vec4(points_[i], 1.0f));
		other_transformed_points[i] = glm::vec3(other.scene_node_->getCompleteTransformation() * glm::vec4(other.points_[i], 1.0f));
	}
	
	for (unsigned int i = 0; i < 8; ++i)
	{
		if (isInside(other_transformed_points[i]) || other.isInside(transformed_points[i]))
		{
			return true;
		}
	}
	
	for (std::vector<std::pair<int, int> >::const_iterator ci = line_segments.begin(); ci != line_segments.end(); ++ci)
	{
		if (doesCollide(other_transformed_points[(*ci).first], other_transformed_points[(*ci).second]))
		{
			return true;
		}
		if (other.doesCollide(transformed_points[(*ci).first], transformed_points[(*ci).second]))
		{
			return true;
		}
	}
	
	return false;
	
	/*
	glm::vec4 global_centre_point = scene_node_->getCompleteTransformation() * glm::vec4(centre_point_, 1.0f);
	
	glm::vec4 principal_axis_x = principal_axis_[0];
	glm::vec4 principal_axis_y = principal_axis_[1];
	glm::vec4 principal_axis_z = principal_axis_[2];
	float max_length = sqrt(glm::length(principal_axis_x) * glm::length(principal_axis_x) + glm::length(principal_axis_y) * glm::length(principal_axis_y) + glm::length(principal_axis_z) * glm::length(principal_axis_z)) / 2.0f;

	glm::vec4 other_global_centre_point = other.scene_node_->getCompleteTransformation() * glm::vec4(other.centre_point_, 1.0f);

	glm::vec4 other_principal_axis_x = other.principal_axis_[0];
	glm::vec4 other_principal_axis_y = other.principal_axis_[1];
	glm::vec4 other_principal_axis_z = other.principal_axis_[2];
	float other_max_length = sqrt(glm::length(other_principal_axis_x) * glm::length(other_principal_axis_x) + glm::length(other_principal_axis_y) * glm::length(other_principal_axis_y) + glm::length(other_principal_axis_z) * glm::length(other_principal_axis_z)) / 2.0f;

	//ss << "Centres: (" << global_centre_point.x << ", " << global_centre_point.y << ", " << global_centre_point.z << ")|";
	//ss << "(" << other_global_centre_point.x << ", " << other_global_centre_point.y << ", " << other_global_centre_point.z << "). Lengths: " << max_length << " , " << other_max_length << "|";
	//ss << glm::distance(global_centre_point, other_global_centre_point) << " <= " << max_length + other_max_length << "|";
	std::string name, other_name;
	const Entity* entity = dynamic_cast<const Entity*>(scene_node_);
	if (entity != NULL)
	{
		name = entity->getName();
	}
	entity = dynamic_cast<const Entity*>(other.scene_node_);
	if (entity != NULL)
	{
		other_name = entity->getName();
	}
	//ss << name << " - " << other_name << "|";
	return glm::distance(global_centre_point, other_global_centre_point) <= max_length + other_max_length;
	*/
}

bool BoundedBox::isInside(const glm::vec3& point) const
{
	bool debug = point == glm::vec3(0, 0, 1);
	
	if (debug)
	{
		//std::cout << "[BoundedBox::isInside] (" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
		//std::cout << "Transformation: (" << scene_node_->getGlobalLocation().x << ", " << scene_node_->getGlobalLocation().y << ", " << scene_node_->getGlobalLocation().z << ")"  << std::endl;
	}
	for (unsigned int i = 0; i < 6; ++i)
	{
		Plane* plane = sides_[i];
		glm::vec4 transformed_plane = glm::transpose(glm::inverse(scene_node_->getCompleteTransformation())) * glm::vec4(plane->getNormal(), plane->getD());
		float distance_to_plane = glm::dot(transformed_plane, glm::vec4(point, 1.0f));
		
		if (debug)
		{
			//std::cout << *plane << std::endl;
			//std::cout << "(" << transformed_plane.x << ", " << transformed_plane.y << ", " << transformed_plane.z << ", " << transformed_plane.w << ") distance to plane: " << distance_to_plane << std::endl;
		}
		if (distance_to_plane < 0)
		{
			//std::cout << "FALSE!" << std::endl;
			return false;
		}
	}
	//std::cout << "TRUE!" << std::endl;
/*
	std::cout << "[BoundedBox::isInside] (" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
	std::cout << "Transformation: (" << scene_node_->getGlobalLocation().x << ", " << scene_node_->getGlobalLocation().y << ", " << scene_node_->getGlobalLocation().z << ")"  << std::endl;
	for (unsigned int i = 0; i < 6; ++i)
	{
		Plane* plane = sides_[i];
		glm::vec4 transformed_plane = glm::transpose(scene_node_->getCompleteTransformation()) * glm::vec4(plane->getNormal(), plane->getD());
		float distance_to_plane = glm::dot(transformed_plane, glm::vec4(point, 1.0f));
		std::cout << *plane << std::endl;
		std::cout << "(" << transformed_plane.x << ", " << transformed_plane.y << ", " << transformed_plane.z << ", " << transformed_plane.w << ") distance to plane: " << distance_to_plane << std::endl;
	}
*/
	return true;
}
/*
bool BoundedBox::isInside(const glm::vec3& point, std::stringstream& ss) const
{
	bool is_inside = true;
	for (unsigned int i = 0; i < 6; ++i)
	{
		Plane* plane = sides_[i];
		glm::vec4 transformed_plane = scene_node_->getCompleteTransformation() * glm::vec4(plane->getNormal(), plane->getD());
		float distance_to_plane = glm::dot(transformed_plane, glm::vec4(point, 1.0f));

		ss << "<" << transformed_plane.x << ", " <<  transformed_plane.y << ", " << transformed_plane.z << ", " << transformed_plane.w << ">" << i << ": " << distance_to_plane << " - ";
		ss << "<" << plane->getNormal().x << ", " <<  plane->getNormal().y << ", " << plane->getNormal().z << ", " << plane->getD() << ">" << i << ": " << distance_to_plane << ".|";

		if (distance_to_plane < 0)
		{
			is_inside = false;
		}
	}

	return is_inside;
}
*/
bool BoundedBox::doesCollide(const glm::vec3& begin_point, const glm::vec3& end_point) const
{
	glm::vec3 updated_begin_point = begin_point;
	updated_begin_point = glm::vec3(glm::inverse(scene_node_->getCompleteTransformation()) * glm::vec4(updated_begin_point, 1.0f));

	glm::vec3 updated_end_point = end_point;
	updated_end_point = glm::vec3(glm::inverse(scene_node_->getCompleteTransformation()) * glm::vec4(updated_end_point, 1.0f));

	/*
	// Absolute values.
	glm::fquat inverse_matrix = glm::inverse(scene_node_->getGlobalRotation());
	glm::vec3 updated_begin_point = begin_point;
	updated_begin_point -= scene_node_->getGlobalLocation();
	updated_begin_point = glm::rotate(inverse_matrix, updated_begin_point);

	glm::vec3 updated_end_point = end_point;
	updated_end_point -= scene_node_->getGlobalLocation();
	updated_end_point = glm::rotate(inverse_matrix, updated_end_point);
	*/
	// Transform the lines such that the box we compare against is aligned.
	glm::vec3 intersection_point;
	for (unsigned int i = 0; i < 6; ++i)
	{
		if (sides_[i]->intersectsWith(updated_begin_point, updated_end_point, intersection_point))
		{
			return true;
		}
	}
	return false;
}

bool BoundedBox::doesCollide(const glm::vec3& begin_point, const glm::vec3& end_point, float effective_width) const
{
	glm::vec3 updated_begin_point = begin_point;
	updated_begin_point = glm::vec3(glm::inverse(scene_node_->getCompleteTransformation()) * glm::vec4(updated_begin_point, 1.0f));

	glm::vec3 updated_end_point = end_point;
	updated_end_point = glm::vec3(glm::inverse(scene_node_->getCompleteTransformation()) * glm::vec4(updated_end_point, 1.0f));
	/*
	// Absolute values.
	glm::fquat inverse_matrix = glm::inverse(scene_node_->getGlobalRotation());
	glm::vec3 updated_begin_point = begin_point;
	updated_begin_point -= scene_node_->getGlobalLocation();
	updated_begin_point = glm::rotate(inverse_matrix, updated_begin_point);

	glm::vec3 updated_end_point = end_point;
	updated_end_point -= scene_node_->getGlobalLocation();
	updated_end_point = glm::rotate(inverse_matrix, updated_end_point);
	*/
	// Transform the lines such that the box we compare against is aligned.
	glm::vec3 intersection_point;
	for (unsigned int i = 0; i < 6; ++i)
	{
		if (sides_[i]->getDistance(updated_begin_point, updated_end_point) < effective_width)
		{
			return true;
		}
	}
	return false;
}

/*
void BoundedBox::getDebug(const SceneNode& node, const Frustum& frustum, std::stringstream& ss) const
{
	glm::vec4 global_centre_point = node.getCompleteTransformation() * glm::vec4(centre_point_, 1.0f);
	ss << "Centre point: " << global_centre_point.x << ", " << global_centre_point.y << ", " << global_centre_point.z << "| ";
	ss << "Principal axes X: " << principal_axis_[0].x << ", " << principal_axis_[0].y << ", " << principal_axis_[0].z << "| ";
	ss << "Principal axes Y: " << principal_axis_[1].x << ", " << principal_axis_[1].y << ", " << principal_axis_[1].z << "| ";
	ss << "Principal axes Z: " << principal_axis_[2].x << ", " << principal_axis_[2].y << ", " << principal_axis_[2].z << "| ";

	// Check if the middle point of the box is inside the extended frustrum.
	for (std::vector<glm::vec4>::const_iterator ci = frustum.getPlanes().begin(); ci != frustum.getPlanes().end(); ++ci)
	{
		const glm::vec4& plane = *ci;
		ss << "Plane: " << plane.x << ", " << plane.y << ", " << plane.z << ", " << plane.w << "|";

		// Calculate the effective radius of this box.
		float effective_radius = 0.5 * (glm::abs(glm::dot(plane, principal_axis_[0])) + glm::abs(glm::dot(plane, principal_axis_[1])) + glm::abs(glm::dot(plane, principal_axis_[2])));
		ss << "   Effective radius: " << effective_radius << ".  Distance from plane: " << glm::dot(global_centre_point, plane) << "|";

		// Check if the centre point of the box is inside the frustrum.
		if (-effective_radius >= glm::dot(global_centre_point, plane))
		{
			ss << "   Outside!";
		}
		else
		{
			ss << "   Inside!";
		}
		ss << "|";
	}
}
*/

bool BoundedBox::update(bool recursive, bool update_visiblity)
{
	float min_x = std::numeric_limits<float>::max();
	float max_x = -std::numeric_limits<float>::max();
	float min_y = std::numeric_limits<float>::max();
	float max_y = -std::numeric_limits<float>::max();
	float min_z = std::numeric_limits<float>::max();
	float max_z = -std::numeric_limits<float>::max();

	update(min_x, max_x, min_y, max_y, min_z, max_z, recursive, update_visiblity);

	// TODO: Check if the bounds actually have been altered.
	/// ...

	initialise(min_x, max_x, min_y, max_y, min_z, max_z);

	return true;
}

void BoundedBox::update(float &min_x, float &max_x, float &min_y, float &max_y, float &min_z, float &max_z, bool recursive, bool update_visiblity)
{
	for (std::vector<SceneNode*>::const_iterator ci = scene_node_->getChildren().begin(); ci != scene_node_->getChildren().end(); ++ci)
	{
		SceneNode* scene_node = *ci;
		BoundedBox* bounding_box = NULL;
		if (update_visiblity)
		{
			bounding_box = &scene_node->getFrustumChecker();
		}
		else
		{
			bounding_box = &scene_node->getCollisionChecker();
		}
		min_x = std::min(min_x, bounding_box->centre_point_.x - bounding_box->principal_axis_[0].x / 2.0f);
		max_x = std::max(max_x, bounding_box->centre_point_.x + bounding_box->principal_axis_[0].x / 2.0f);

		min_y = std::min(min_y, bounding_box->centre_point_.y - bounding_box->principal_axis_[1].y / 2.0f);
		max_y = std::max(max_y, bounding_box->centre_point_.y + bounding_box->principal_axis_[1].y / 2.0f);

		min_z = std::min(min_z, bounding_box->centre_point_.z - bounding_box->principal_axis_[2].z / 2.0f);
		max_z = std::max(max_z, bounding_box->centre_point_.z + bounding_box->principal_axis_[2].z / 2.0f);

		if (recursive)
		{
			bounding_box->update(min_x, max_x, min_y, max_y, min_z, max_z, recursive, update_visiblity);
		}
	}
}

void BoundedBox::processSceneNode(const SceneNode& scene_node, const glm::mat4& transform_matrix, float &min_x, float &max_x, float &min_y, float &max_y, float &min_z, float &max_z, bool check_visibility, const std::vector<const SceneNode*>& excluded_nodes, const std::string& debug)
{
	if (check_visibility)
	{
		for (std::vector<SceneLeaf*>::const_iterator ci = scene_node.getLeafs().begin(); ci != scene_node.getLeafs().end(); ++ci)
		{
			SceneLeafModel* model = dynamic_cast<SceneLeafModel*>(*ci);

			if (model == NULL)
			{
				continue;
			}
			
			for (std::vector<GLuint>::const_iterator ci = model->getModel().getIndices().begin(); ci != model->getModel().getIndices().end(); ++ci)
			{
				const glm::vec4& point = transform_matrix * glm::vec4(model->getModel().getVertices()[*ci], 1.0f);
				min_x = std::min(min_x, point.x);
				max_x = std::max(max_x, point.x);
				min_y = std::min(min_y, point.y);
				max_y = std::max(max_y, point.y);
				min_z = std::min(min_z, point.z);
				max_z = std::max(max_z, point.z);
			}
		
			/*
			// Check the geometry and fit a box around it.
			for (std::vector<glm::vec3>::const_iterator ci = model->getModel().getVertices().begin(); ci != model->getModel().getVertices().end(); ++ci)
			{
				glm::vec4 point = transform_matrix * glm::vec4(*ci, 1.0f);
				min_x = std::min(min_x, point.x);
				max_x = std::max(max_x, point.x);
				min_y = std::min(min_y, point.y);
				max_y = std::max(max_y, point.y);
				min_z = std::min(min_z, point.z);
				max_z = std::max(max_z, point.z);
			}
			*/
		}
	}
	else
	{
		// Only entities have collision boxes.
		const Entity* entity = dynamic_cast<const Entity*>(&scene_node);
		if (entity != NULL)
		{
#ifdef HORROR_GAME_ENABLE_DEBUG
			std::stringstream ss;
			ss << "Process: " << debug << " -> " << entity->getName() << std::endl;
			ss << transform_matrix[3][0] << ", " << transform_matrix[3][1] << ", " << transform_matrix[3][2] << ")" << std::endl;
#endif
			entity->getBoundedCollisionBox(transform_matrix, min_x, max_x, min_y, max_y, min_z, max_z);
			/*
			for (std::vector<BoxCollision*>::const_iterator ci = entity->getCollisions().begin(); ci != entity->getCollisions().end(); ++ci)
			{
				const BoxCollision* box = *ci;
				for (unsigned int i = 0; i < 8; ++i)
				{
					glm::vec4 point = transform_matrix * glm::vec4(box->getPoints()[i], 1.0);
					//glm::vec4 point = glm::vec4(box->getPoints()[i], 1.0);
					min_x = std::min(min_x, point.x);
					max_x = std::max(max_x, point.x);
					min_y = std::min(min_y, point.y);
					max_y = std::max(max_y, point.y);
					min_z = std::min(min_z, point.z);
					max_z = std::max(max_z, point.z);
#ifdef HORROR_GAME_ENABLE_DEBUG
					ss << point.x << ", " << point.y << ", " << point.z << std::endl;
#endif
				}
			}*/

			// Some entities might have defined their own bounded box. We need to take this into account
			// as not all entities use box collision objects (e.g. height maps).
			//if (entity->getCollisionChecker()
			//if (entity->getCollisionChecker().getPoints()
#ifdef _WIN32
			//MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#endif
		}
	}

	for (std::vector<SceneNode*>::const_iterator ci = scene_node.getChildren().begin(); ci != scene_node.getChildren().end(); ++ci)
	{
		if (std::find(excluded_nodes.begin(), excluded_nodes.end(), *ci) != excluded_nodes.end())
		{
			continue;
		}
#ifdef HORROR_GAME_ENABLE_DEBUG
		std::stringstream ss;
		ss << debug;
		const Entity* this_entity = dynamic_cast<const Entity*>(&scene_node);
		if (this_entity != NULL)
		{
			ss << this_entity->getName() << " -> ";
		}
		else
		{
			ss << "SCENE_NODE -> ";
		}

		const Entity* entity = dynamic_cast<Entity*>(*ci);
		if (entity != NULL)
		{
			ss << " -> " << entity->getName();
		}
		processSceneNode(**ci, transform_matrix * (*ci)->getLocalTransformation(), min_x, max_x, min_y, max_y, min_z, max_z, check_visibility, excluded_nodes, ss.str().c_str());
#endif
		processSceneNode(**ci, transform_matrix * (*ci)->getLocalTransformation(), min_x, max_x, min_y, max_y, min_z, max_z, check_visibility, excluded_nodes, debug);
	}
}
