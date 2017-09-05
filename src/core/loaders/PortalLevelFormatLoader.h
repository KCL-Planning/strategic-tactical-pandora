#ifndef CORE_LOADERS_PORTAL_LEVEL_FORMAT_LOADER_H
#define CORE_LOADERS_PORTAL_LEVEL_FORMAT_LOADER_H

#ifdef _WIN32
#include <windows.h>
#endif

#include <iostream>
#include <string>
#include <vector>
#include <sstream>

#include <glm/glm.hpp>
#include <GL/glew.h>

class ShaderInterface;
class Entity;
class Level;
class Region;
class SceneNode;
class SceneManager;
class NavigationMesh;
class Material;

#include "../math/Plane.h"
#include "PLFCube.h"

struct PLF_Triangle
{
	PLF_Triangle(unsigned int vertex0, unsigned int vertex1, unsigned int vertex2, const glm::vec3& normal, const glm::vec2& uv0, const glm::vec2& uv1, const glm::vec2& uv2)
		: vertex_(glm::ivec3(vertex0, vertex1, vertex2)), normal_(normal)
	{
		uv_[0] = uv0;
		uv_[1] = uv1;
		uv_[2] = uv2;
	}

	PLF_Triangle()
		: vertex_(glm::ivec3(std::numeric_limits<unsigned int>::max(), std::numeric_limits<unsigned int>::max(), std::numeric_limits<unsigned int>::max())), normal_(glm::vec3(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()))
	{
		uv_[0] = glm::vec2(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
		uv_[1] = glm::vec2(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
		uv_[2] = glm::vec2(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
	}

	bool sanityCheck()
	{
		for (unsigned int i = 0; i < 3; ++i)
		{
			if (vertex_[i] == std::numeric_limits<unsigned int>::max())
				return false;
			else if (normal_[i] == std::numeric_limits<unsigned int>::max())
				return false;
			for (unsigned int j = 0; j < 2; ++j)
			{
				if (uv_[i][j] == std::numeric_limits<unsigned int>::max())
					return false;
			}
		}
		return true;
	}

	glm::ivec3 vertex_;
	glm::vec3 normal_;
	glm::vec2 uv_[3];
};

class Section
{
public:
	Section(unsigned int section_id)
		: section_id_(section_id)
	{

	}

	void addIndex(GLuint index)
	{
		indices_.push_back(index);
	}

	void calculateCentre(const std::vector<glm::vec3>& vertices)
	{
		centre_ = glm::vec3(0, 0, 0);
		for (std::vector<GLuint>::const_iterator ci = indices_.begin(); ci != indices_.end(); ++ci)
		{
			centre_ = centre_ + vertices[*ci];
		}
		centre_ = glm::vec3(centre_.x / indices_.size(), centre_.y / indices_.size(), centre_.z / indices_.size());
	}

	glm::vec3 getCentre() const
	{
		return centre_;
	}

	const std::vector<PLF_Cube*>& getCollisionBoxes() const { return collision_boxes_; }

	/**
	 * Construct a cube out of 8 points. Needs to construct a set of 6 planes
	 * that do not collide. Their normals will dictate which part of the cube 
	 * they represent.
	 */
	void transformToStandardAxis(std::vector<glm::vec3>& transformed_points, const std::vector<glm::vec3>& bb)
	{
/*
#ifdef _WIN32
		{
		std::stringstream ss;
		ss << "transformToStandardAxis: ";
		for (unsigned int i = 0; i < bb.size(); ++i)
		{
			ss << "(" << bb[i].x << "," << bb[i].y << "," << bb[i].z << ")" << std::endl;
		}
		OutputDebugString(ss.str().c_str());
		}
#endif
*/
		// Construct a local coordinate system and use that to construct the bounding box.
		const glm::vec3& origin = bb[0];

		float min_distance = std::numeric_limits<float>::max();
		const glm::vec3* px = NULL;
		for (std::vector<glm::vec3>::const_iterator ci = bb.begin(); ci != bb.end(); ++ci)
		{
			const glm::vec3& point = *ci;
			if (&point == &origin) continue;
			float distance = glm::distance(point, origin);
			if (distance < min_distance)
			{
				px = &point;
				min_distance = distance;
			}
		}

		min_distance = std::numeric_limits<float>::max();
		const glm::vec3* py = NULL;
		for (std::vector<glm::vec3>::const_iterator ci = bb.begin(); ci != bb.end(); ++ci)
		{
			const glm::vec3& point = *ci;
			if (&point == &origin || &point == px) continue;
			float distance = glm::distance(point, origin);
			if (distance < min_distance)
			{
				py = &point;
				min_distance = distance;
			}
		}

		// Next create the 3rd point by taking the cross product, this will form the local coordinate system.
		glm::vec3 axis_x = *px - origin;
		glm::vec3 axis_y = *py - origin;
		glm::vec3 axis_z = glm::cross(axis_x, axis_y);
/*
#ifdef _WIN32
		{
		std::stringstream ss;
		ss << "Axis system: ";
		ss << "X = (" << axis_x.x << "," << axis_x.y << "," << axis_x.z << ")" << std::endl;
		ss << "Y = (" << axis_y.x << "," << axis_y.y << "," << axis_y.z << ")" << std::endl;
		ss << "Z = (" << axis_z.x << "," << axis_z.y << "," << axis_z.z << ")" << std::endl;
		OutputDebugString(ss.str().c_str());
		}
#endif
*/
		// Now we are going to transform this coordinate system to the identity matrix. This can be done by taking
		// the inverse. Then using M^T we can calculate the positions of the point in the 'normalized' coordinate system
		// this will help us to determine the locations of all the points (Note: only works with cubes!).
		glm::mat3 m(glm::normalize(axis_x), glm::normalize(axis_y), glm::normalize(axis_z));
/*	
#ifdef _WIN32
		{
		std::stringstream ss;
		ss << "Matrix: ";
		for (int y = 0; y < 3; ++y)
		{
			ss << "[";
			for (int x = 0; x < 3; ++x)
			{
				ss << m[x][y] << " ";
			}
			ss << "]" << std::endl;
		}
		OutputDebugString(ss.str().c_str());
		}
#endif
*/	
		m = glm::inverse(m);
/*
#ifdef _WIN32
		{
		std::stringstream ss;
		ss << "Inversed matrix: ";
		for (int y = 0; y < 3; ++y)
		{
			ss << "[";
			for (int x = 0; x < 3; ++x)
			{
				ss << m[x][y] << " ";
			}
			ss << "]" << std::endl;
		}
		OutputDebugString(ss.str().c_str());
		}
#endif
*/
		for (std::vector<glm::vec3>::const_iterator ci = bb.begin(); ci != bb.end(); ++ci)
		{
			glm::vec3 tmp = m * *ci;
/*
#ifdef _WIN32
			{
			std::stringstream ss;
			ss << "(" << (*ci).x << "," << (*ci).y << "," << (*ci).z << ") -> (" << tmp.x << "," << tmp.y << "," << tmp.z << ")" << std::endl;
			OutputDebugString(ss.str().c_str());
			}
#endif
*/
			transformed_points.push_back(tmp);
		}
	}

	
	void addCollisionBox(const std::vector<glm::vec3>& points)
	{
		std::vector<glm::vec3> transformed_points;
		transformToStandardAxis(transformed_points, points);
/*
#ifdef _WIN32
		{
		std::stringstream ss;
		for (unsigned int i = 0; i < points.size(); ++i)
		{
			ss << "(" << points[i].x << "," << points[i].y << "," << points[i].z << ") -> (" << transformed_points[i].x << "," << transformed_points[i].y << "," << transformed_points[i].z << ")" << std::endl;
		}
		OutputDebugString(ss.str().c_str());
		}
#else
		for (unsigned int i = 0; i < points.size(); ++i)
		{
			std::cout << "(" << points[i].x << "," << points[i].y << "," << points[i].z << ") -> (" << transformed_points[i].x << "," << transformed_points[i].y << "," << transformed_points[i].z << ")" << std::endl;
		}
#endif
*/
		// Order the points (we now assume that the box is axis aligned; will deal
		// with more complex cases later. Blender has an axis system such that 
		// X is the close / away axis; Y is the left / right axis; Z is the up / down axis.
		glm::vec3 bottom_left_away(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
		glm::vec3 bottom_right_away(-std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
		glm::vec3 top_left_away(std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
		glm::vec3 top_right_away(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
		glm::vec3 bottom_left_close(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
		glm::vec3 bottom_right_close(-std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
		glm::vec3 top_left_close(std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
		glm::vec3 top_right_close(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), std::numeric_limits<float>::max());

		float EPSILON = 0.1f;

		unsigned int bottom_left_away_i, bottom_right_away_i, top_left_away_i, top_right_away_i, bottom_left_close_i, bottom_right_close_i, top_left_close_i, top_right_close_i;
		for (unsigned int i = 0; i < transformed_points.size(); ++i)
		{
/*
#ifdef _WIN32
			std::stringstream ss;
			ss << i << ": Process (" << transformed_points[i].x << "," << transformed_points[i].y << "," << transformed_points[i].z << ") -=- ";
#else
			std::cout << i << ": Process (" << transformed_points[i].x << "," << transformed_points[i].y << "," << transformed_points[i].z << ") -=- ";
#endif
*/
			if (transformed_points[i].x - EPSILON <= bottom_left_away.x && transformed_points[i].y - EPSILON <= bottom_left_away.y && transformed_points[i].z + EPSILON >= bottom_left_away.z) { //001
				//std::cout << "bottom_left_away::(" << bottom_left_away.x << "," << bottom_left_away.y << "," << bottom_left_away.z << ") -> (" << transformed_points[i].x << "," << transformed_points[i].y << "," << transformed_points[i].z << ")" << std::endl;
				bottom_left_away = transformed_points[i];
				bottom_left_away_i = i;
			}


			if (transformed_points[i].x + EPSILON >= bottom_right_away.x && transformed_points[i].y - EPSILON <= bottom_right_away.y && transformed_points[i].z + EPSILON >= bottom_right_away.z) { //101
				//std::cout << "bottom_right_away::(" << bottom_right_away.x << "," << bottom_right_away.y << "," << bottom_right_away.z << ") -> (" << transformed_points[i].x << "," << transformed_points[i].y << "," << transformed_points[i].z << ")" << std::endl;
				bottom_right_away = transformed_points[i];
				bottom_right_away_i = i;
			}

			if (transformed_points[i].x - EPSILON <= top_left_away.x && transformed_points[i].y + EPSILON >= top_left_away.y && transformed_points[i].z + EPSILON >= top_left_away.z) { // 011
				//std::cout << "top_left_away::(" << top_left_away.x << "," << top_left_away.y << "," << top_left_away.z << ") -> (" << transformed_points[i].x << "," << transformed_points[i].y << "," << transformed_points[i].z << ")" << std::endl;
				top_left_away = transformed_points[i];
				top_left_away_i = i;
			}

			if (transformed_points[i].x + EPSILON >= top_right_away.x && transformed_points[i].y + EPSILON >= top_right_away.y && transformed_points[i].z + EPSILON >= top_right_away.z) { // 111
				//std::cout << "top_right_away::(" << top_right_away.x << "," << top_right_away.y << "," << top_right_away.z << ") -> (" << transformed_points[i].x << "," << transformed_points[i].y << "," << transformed_points[i].z << ")" << std::endl;
				top_right_away = transformed_points[i];
				top_right_away_i = i;
			}

			if (transformed_points[i].x - EPSILON <= bottom_left_close.x && transformed_points[i].y - EPSILON <= bottom_left_close.y && transformed_points[i].z - EPSILON <= bottom_left_close.z) { // 000
				//std::cout << "bottom_left_close::(" << bottom_left_close.x << "," << bottom_left_close.y << "," << bottom_left_close.z << ") -> (" << transformed_points[i].x << "," << transformed_points[i].y << "," << transformed_points[i].z << ")" << std::endl;
				bottom_left_close = transformed_points[i];
				bottom_left_close_i = i;
			}	

			if (transformed_points[i].x + EPSILON >= bottom_right_close.x && transformed_points[i].y - EPSILON <= bottom_right_close.y && transformed_points[i].z - EPSILON <= bottom_right_close.z) { //100
				//std::cout << "bottom_right_close::(" << bottom_right_close.x << "," << bottom_right_close.y << "," << bottom_right_close.z << ") -> (" << transformed_points[i].x << "," << transformed_points[i].y << "," << transformed_points[i].z << ")" << std::endl;
				bottom_right_close = transformed_points[i];
				bottom_right_close_i = i;
			}

			if (transformed_points[i].x - EPSILON <= top_left_close.x && transformed_points[i].y + EPSILON >= top_left_close.y && transformed_points[i].z - EPSILON <= top_left_close.z) { // 010
				//std::cout << "top_left_close::(" << top_left_close.x << "," << top_left_close.y << "," << top_left_close.z << ") -> (" << transformed_points[i].x << "," << transformed_points[i].y << "," << transformed_points[i].z << ")" << std::endl;
				top_left_close = transformed_points[i];
				top_left_close_i = i;
			}

			if (transformed_points[i].x + EPSILON >= top_right_close.x && transformed_points[i].y + EPSILON >= top_right_close.y && transformed_points[i].z - EPSILON <= top_right_close.z) { // 110
				//std::cout << "top_right_close::(" << top_right_close.x << "," << top_right_close.y << "," << top_right_close.z << ") -> (" << transformed_points[i].x << "," << transformed_points[i].y << "," << transformed_points[i].z << ")" << std::endl;
				top_right_close = transformed_points[i];
				top_right_close_i = i;
			}
/*
#ifdef _WIN32
			ss << std::endl;
			ss << "Result so far: " << std::endl;
			ss << "bottom_left_away::(" << bottom_left_away.x << "," << bottom_left_away.y << "," << bottom_left_away.z << ")" << std::endl;
			ss << "bottom_right_away::(" << bottom_right_away.x << "," << bottom_right_away.y << "," << bottom_right_away.z << ")" << std::endl;
			ss << "top_left_away::(" << top_left_away.x << "," << top_left_away.y << "," << top_left_away.z << ")" << std::endl;
			ss << "top_right_away::(" << top_right_away.x << "," << top_right_away.y << "," << top_right_away.z << ")" << std::endl;
			ss << "bottom_left_close::(" << bottom_left_close.x << "," << bottom_left_close.y << "," << bottom_left_close.z << ")" << std::endl;
			ss << "bottom_right_close::(" << bottom_right_close.x << "," << bottom_right_close.y << "," << bottom_right_close.z << ")" << std::endl;
			ss << "top_left_close::(" << top_left_close.x << "," << top_left_close.y << "," << top_left_close.z << ")" << std::endl;
			ss << "top_right_close::(" << top_right_close.x << "," << top_right_close.y << "," << top_right_close.z << ")" << std::endl;
			ss << std::endl;
			OutputDebugString(ss.str().c_str());
#else
			std::cout << std::endl;
			std::cout << "Result so far: " << std::endl;
			std::cout << "bottom_left_away::(" << bottom_left_away.x << "," << bottom_left_away.y << "," << bottom_left_away.z << ")" << std::endl;
			std::cout << "bottom_right_away::(" << bottom_right_away.x << "," << bottom_right_away.y << "," << bottom_right_away.z << ")" << std::endl;
			std::cout << "top_left_away::(" << top_left_away.x << "," << top_left_away.y << "," << top_left_away.z << ")" << std::endl;
			std::cout << "top_right_away::(" << top_right_away.x << "," << top_right_away.y << "," << top_right_away.z << ")" << std::endl;
			std::cout << "bottom_left_close::(" << bottom_left_close.x << "," << bottom_left_close.y << "," << bottom_left_close.z << ")" << std::endl;
			std::cout << "bottom_right_close::(" << bottom_right_close.x << "," << bottom_right_close.y << "," << bottom_right_close.z << ")" << std::endl;
			std::cout << "top_left_close::(" << top_left_close.x << "," << top_left_close.y << "," << top_left_close.z << ")" << std::endl;
			std::cout << "top_right_close::(" << top_right_close.x << "," << top_right_close.y << "," << top_right_close.z << ")" << std::endl;
			std::cout << std::endl;
#endif
*/
		}
		
		collision_boxes_.push_back(new PLF_Cube(points[bottom_left_away_i], points[bottom_right_away_i], points[top_left_away_i], points[top_right_away_i],
		                              points[bottom_left_close_i], points[bottom_right_close_i], points[top_left_close_i], points[top_right_close_i]));

		/*
		for (std::vector<glm::vec3>::const_iterator ci = bb.begin(); ci != bb.end(); ++ci)
		{
			if ((*ci).x >= bottom_left_away.x && (*ci).y <= bottom_left_away.y && (*ci).z <= bottom_left_away.z) {//100
				bottom_left_away = *ci;
			}
		}

		glm::vec3 bottom_right_away(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
		for (std::vector<glm::vec3>::const_iterator ci = bb.begin(); ci != bb.end(); ++ci)
		{
			if ((*ci).x >= bottom_right_away.x && (*ci).y >= bottom_right_away.y && (*ci).z <= bottom_right_away.z) //110
				bottom_right_away = *ci;
		}

		glm::vec3 top_left_away(-std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
		for (std::vector<glm::vec3>::const_iterator ci = bb.begin(); ci != bb.end(); ++ci)
		{
			if ((*ci).x >= top_left_away.x && (*ci).y <= top_left_away.y && (*ci).z >= top_left_away.z) // 101
				top_left_away = *ci;
		}

		glm::vec3 top_right_away(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
		for (std::vector<glm::vec3>::const_iterator ci = bb.begin(); ci != bb.end(); ++ci)
		{
			if ((*ci).x >= top_right_away.x && (*ci).y >= top_right_away.y && (*ci).z >= top_right_away.z) // 111
				top_right_away = *ci;
		}

		glm::vec3 bottom_left_close(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
		for (std::vector<glm::vec3>::const_iterator ci = bb.begin(); ci != bb.end(); ++ci)
		{
			if ((*ci).x <= bottom_left_close.x && (*ci).y <= bottom_left_close.y && (*ci).z <= bottom_left_close.z) // 000
				bottom_left_close = *ci;
		}

		glm::vec3 bottom_right_close(std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
		for (std::vector<glm::vec3>::const_iterator ci = bb.begin(); ci != bb.end(); ++ci)
		{
			if ((*ci).x <= bottom_right_close.x && (*ci).y >= bottom_right_close.y && (*ci).z <= bottom_right_close.z) //010
				bottom_right_close = *ci;
		}

		glm::vec3 top_left_close(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
		for (std::vector<glm::vec3>::const_iterator ci = bb.begin(); ci != bb.end(); ++ci)
		{
			if ((*ci).x <= top_left_close.x && (*ci).y <= top_left_close.y && (*ci).z >= top_left_close.z) // 001
				top_left_close = *ci;
		}

		glm::vec3 top_right_close(std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
		for (std::vector<glm::vec3>::const_iterator ci = bb.begin(); ci != bb.end(); ++ci)
		{
			if ((*ci).x <= top_right_close.x && (*ci).y >= top_right_close.y && (*ci).z >= top_right_close.z) // 011
				top_right_close = *ci;
		}

		collision_boxes_.push_back(PLF_Cube(bottom_left_away, bottom_right_away, top_left_away, top_right_away,
		                              bottom_left_close, bottom_right_close, top_left_close, top_right_close));
		*/
	}
	
	const ::std::vector<GLuint>& getIndices() const { return indices_; }

	unsigned int getSectionId() const { return section_id_; }

private:
	unsigned int section_id_;
	std::vector<GLuint> indices_;
	glm::vec3 centre_;
	std::vector<PLF_Cube*> collision_boxes_;
};

struct PortalDef
{
	PortalDef(unsigned int s1, unsigned int s2, const std::vector<glm::vec3>& points)
		: section_id_1_(s1), section_id_2_(s2)//, points_(points)
	{
		// Sort the points such that they form a convex polygon, e.g.
		// 1-2
		// |/|
		// 3-4
		// is not acceptable because it forms a concave shape.
		
		std::vector<glm::vec3> open_list(points);
		std::vector<glm::vec3> work_list;

		points_ = findConvexShape(open_list, work_list, points);
	}

	std::vector<glm::vec3> findConvexShape(std::vector<glm::vec3> open_list, const std::vector<glm::vec3>& w, const std::vector<glm::vec3>& points);

	unsigned int section_id_1_;
	unsigned int section_id_2_;
	std::vector<glm::vec3> points_;
};

class PortalLevelFormatLoader
{
public:
	SceneNode* importLevel(const std::string& filename, Material& material, ShaderInterface& shader, SceneManager& scene_manager, SceneNode& terrain_node, bool create_regions = true);

	/**
	 * Create the navigation mesh. We create debug meshes if scene_manager is not NULL. Must be called 
	 * after succesfully loading a level using importLevel.
	 */
	NavigationMesh* createNavigationMesh(SceneManager* scene_manager = NULL);
	
private:
	std::vector<glm::vec3> vertices_;
	std::vector<glm::vec3> normals_;
	std::vector<glm::vec2> texture_coordinates_;
	std::vector<Section*> sections_;
	std::vector<Region*> regions_;
	std::vector<PortalDef> portals_;
	Entity* root_node_;
};

#endif
