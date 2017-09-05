#ifdef _WIN32
#include <Windows.h>
#endif

#include "HeightMap.h"

#include <complex>
#include <math.h>

#include "../math/Math.h"
#include "../math/Plane.h"
#include "../collision/CollisionInfo.h"
#include "../collision/BoxCollision.h"

HeightMap::HeightMap(unsigned int width, unsigned int height, float cell_size, const std::vector<float>& height_map, SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transformation, ENTITY_TYPE type, const std::string& name, bool init_children)
	: Entity(scene_manager, parent, transformation, type, name, init_children), width_(width), height_(height), cell_size_(cell_size), height_map_(&height_map)
{

}

bool HeightMap::getCollisions(Entity& entity, std::vector<CollisionInfo>& info) const
{
	bool found_collision = false;
	for (std::vector<SceneNode*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
	{
		if ((*ci)->getCollisions(entity, info))
		{
			found_collision = true;
		}
	}
	return found_collision;
}

void HeightMap::getBoundedCollisionBox(const glm::mat4& transform_matrix, float& min_x, float& max_x, float& min_y, float& max_y, float& min_z, float& max_z) const
{
	for (unsigned int x = 0; x < width_; ++x)
	{
		for (unsigned int z = 0; z < height_; ++z)
		{
			glm::vec4 point(x * cell_size_ - (width_ / 2) * cell_size_, (*height_map_)[x + z * width_], z * cell_size_ - (height_ / 2) * cell_size_, 1.0f);
			point = transform_matrix * point;
			min_x = std::min(min_x, point.x);
			max_x = std::max(max_x, point.x);
			min_y = std::min(min_y, point.y);
			max_y = std::max(max_y, point.y);
			min_z = std::min(min_z, point.z);
			max_z = std::max(max_z, point.z);
		}
	}
	/*
	// In order to speed up collision detection, we need to put a box around the entire terrain.
	float min_height = std::numeric_limits<float>::max();
	float max_height = -std::numeric_limits<float>::max();

	for (std::vector<float>::const_iterator ci = height_map_->begin(); ci != height_map_->end(); ++ci)
	{
		min_height = std::min(min_height, *ci);
		max_height = std::max(max_height, *ci);
	}
	float half_width = cell_size_ * (width_ / 2);
	float half_depth = cell_size_ * (height_ / 2);
	
	min_x = -half_width;
	max_x = half_width;
	min_y = min_height;
	max_y = max_height;
	min_z = -half_depth;
	max_z = half_depth;
	
	BoundedBox* bb = new BoundedBox(*this, 
		glm::vec3(-half_width, min_height, half_depth), glm::vec3(half_width, min_height, half_depth),
		glm::vec3(-half_width, max_height, half_depth), glm::vec3(half_width, max_height, half_depth),
		glm::vec3(-half_width, min_height, -half_depth), glm::vec3(half_width, min_height, -half_depth),
		glm::vec3(-half_width, max_height, -half_depth), glm::vec3(half_width, max_height, -half_depth));
	setCollisionBoundedBox(*bb);
	*/
}

bool HeightMap::getCollisions(Entity& entity, const glm::vec3& begin, const glm::vec3& end, std::vector<CollisionInfo>& info) const
{
	bool found_collision = false;
	if (&entity != this)
	{
		glm::mat4 m = glm::inverse(getCompleteTransformation());

		glm::vec3 p1 = glm::vec3(m * glm::vec4(begin, 1.0f));
		glm::vec3 p2 = glm::vec3(m * glm::vec4(end, 1.0f));
	
		// We use a very simple method, by just checking all the triangles that fall within
		// the square that is made up between the begin and end points of the lines.
		int p1_x = (p1.x / cell_size_) + (width_ / 2);
		int p1_y = (p1.z / cell_size_) + (height_ / 2);

		int p2_x = (p2.x / cell_size_) + (width_ / 2);
		int p2_y = (p2.z / cell_size_) + (height_ / 2);

		int min_x = std::min(p1_x, p2_x);
		if (min_x < 0) min_x = 0;
		if (min_x >= width_) return false;

		int max_x = std::max(p1_x, p2_x) + 1;
		if (max_x < 0) return false;
		if (max_x >= width_ - 1) max_x = width_ - 1;
	
		int min_y = std::min(p1_y, p2_y);
		if (min_y < 0) min_y = 0;
		if (min_y >= height_) return false;

		int max_y = std::max(p1_y, p2_y) + 1;
		if (max_y < 0) return false;
		if (max_y >= height_ - 1) max_y = height_ - 1;
	/*
	#ifdef _WIN32
		std::stringstream ss;
		ss << p1.x << " -> " << p1_x << "; " << p1.z << " -> " << p1_y << "; " << p2.x << " -> " << p2_x << "; " << p2.z << " -> " << p2_y << "; " << std::endl;
		ss << "(" << min_x << ", " << min_y << ") -- (" << max_x << ", " << max_y << ")" << std::endl;
		OutputDebugString(ss.str().c_str());
	#endif
	*/
		
		for (int x = min_x; x < max_x; ++x)
		{
			for (int y = min_y; y < max_y; ++y)
			{
				glm::vec3 top_left(x * cell_size_ - (cell_size_ / 2.0f) * (width_ - 1), (*height_map_)[x + y * width_], y * cell_size_ - (cell_size_ / 2.0f) * (height_ - 1));
				glm::vec3 top_right((x + 1) * cell_size_ - (cell_size_ / 2.0f) * (width_ - 1), (*height_map_)[x + 1 + y * width_], y * cell_size_ - (cell_size_ / 2.0f) * (height_ - 1));
				glm::vec3 bottom_left(x * cell_size_ - (cell_size_ / 2.0f) * (width_ - 1), (*height_map_)[x + (y + 1) * width_], (y + 1) * cell_size_ - (cell_size_ / 2.0f) * (height_ - 1));
				glm::vec3 bottom_right((x + 1) * cell_size_ - (cell_size_ / 2.0f) * (width_ - 1), (*height_map_)[x + 1 + (y + 1) * width_], (y + 1) * cell_size_ - (cell_size_ / 2.0f) * (height_ - 1));
			
				std::vector<glm::vec3> upper_triangle;
				upper_triangle.push_back(top_left);
				upper_triangle.push_back(top_right);
				upper_triangle.push_back(bottom_left);

				std::vector<glm::vec3> bottom_triangle;
				bottom_triangle.push_back(bottom_right);
				bottom_triangle.push_back(top_right);
				bottom_triangle.push_back(bottom_left);

				// Check the 2 triangles that make up this square.
				glm::vec3 plane_intersection;
				if (Plane::intersectsWithPolygon(upper_triangle, p1, p2, plane_intersection))
				{
					CollisionInfo ci;
					ci.colliding_entity_ = const_cast<HeightMap*>(this);
					ci.other_colliding_entity_ = &entity;
					ci.collision_loc_.push_back(plane_intersection);
					info.push_back(ci);
					found_collision = true;
				}
				if (Plane::intersectsWithPolygon(bottom_triangle, p1, p2, plane_intersection))
				{
					CollisionInfo ci;
					ci.colliding_entity_ = const_cast<HeightMap*>(this);
					ci.other_colliding_entity_ = &entity;
					ci.collision_loc_.push_back(plane_intersection);
					info.push_back(ci);
					found_collision = true;
				}
			}
		}
	}

	for (std::vector<SceneNode*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
	{
		if ((*ci)->getCollisions(entity, begin, end, info))
		{
			found_collision = true;
		}
	}
	return found_collision;
	/*
	glm::vec3 direction = glm::normalize(p2 - p1);

	while (true)
	{
		float distance_to_left_border = fmod(std::abs(p1.x), cell_size_);
		float distance_to_right_border = cell_size_ - distance_to_left_border;
		float distance_to_bottom_border = fmod(std::abs(p1.z), cell_size_);
		float distance_to_top_border = cell_size_ - distance_to_bottom_border;

		glm::vec2 top_left = glm::vec2(p1.x, p1.z) + distance_to_top_border - distance_to_left_border;
		glm::vec2 top_right = glm::vec2(p1.x, p1.z) + distance_to_top_border + distance_to_right_border;
		glm::vec2 bottom_left = glm::vec2(p1.x, p1.z) - distance_to_bottom_border - distance_to_left_border;
		glm::vec2 bottom_right = glm::vec2(p1.x, p1.z) - distance_to_bottom_border + distance_to_right_border;

		///   --- x -->
		/// +----------+  ^
		/// |top     / |  |
		/// |      /   |  y (z in 3D)
		/// |    /     |  |
		/// |  /   bot |  |
		/// +/---------+
		// 'top' side of the triangle.
		glm::vec2 closest_intersection(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
		glm::vec3 plane_p1;
		glm::vec3 plane_p2;
		glm::vec3 plane_p3;

		if (fmod(std::abs(p1.x), cell_size_) < fmod(std::abs(p1.z), cell_size_))
		{
			plane_p1 = glm::vec3(top_left.x, getHeight(top_left.x, top_left.y), top_left.y);
			plane_p2 = glm::vec3(top_right.x, getHeight(top_right.x, top_right.y), top_right.y);
			plane_p3 = glm::vec3(bottom_left.x, getHeight(bottom_left.x, bottom_left.y), bottom_left.y);

			glm::vec2 tmp_intersection;
			// Check against the left border.
			if (direction.x < 0 && Math::getIntersectionSegments(glm::vec2(p1.x, p1.z), glm::vec2(p2.x, p2.z), bottom_left, top_left, tmp_intersection))
			{
				if (glm::distance(glm::vec2(p1.x, p1.z), tmp_intersection) < glm::distance(glm::vec2(p1.x, p1.z), closest_intersection))
				{
					closest_intersection = tmp_intersection;
				}
			}

			// Check against the diagonal border.
			if (std::abs(direction.x) < (direction.x < 0 ? -direction.z : direction.z) &&
			    direction.x > direction.z && Math::getIntersectionSegments(glm::vec2(p1.x, p1.z), glm::vec2(p2.x, p2.z), bottom_left, top_right, tmp_intersection))
			{
				if (glm::distance(glm::vec2(p1.x, p1.z), tmp_intersection) < glm::distance(glm::vec2(p1.x, p1.z), closest_intersection))
				{
					closest_intersection = tmp_intersection;
				}
			}

			// Check against the top border.
			if (direction.z > 0 && Math::getIntersectionSegments(glm::vec2(p1.x, p1.z), glm::vec2(p2.x, p2.z), top_left, top_right, tmp_intersection))
			{
				if (glm::distance(glm::vec2(p1.x, p1.z), tmp_intersection) < glm::distance(glm::vec2(p1.x, p1.z), closest_intersection))
				{
					closest_intersection = tmp_intersection;
				}
			}
		}
		// 'bottom' side of the triangle.
		else
		{
			plane_p1 = glm::vec3(bottom_right.x, getHeight(bottom_right.x, bottom_right.y), bottom_right.y);
			plane_p2 = glm::vec3(top_right.x, getHeight(top_right.x, top_right.y), top_right.y);
			plane_p3 = glm::vec3(bottom_left.x, getHeight(bottom_left.x, bottom_left.y), bottom_left.y);

			glm::vec2 tmp_intersection;
			// Check against the right border.
			if (direction.x > 0 && Math::getIntersectionSegments(glm::vec2(p1.x, p1.z), glm::vec2(p2.x, p2.z), bottom_right, top_right, tmp_intersection))
			{
				if (glm::distance(glm::vec2(p1.x, p1.z), tmp_intersection) < glm::distance(glm::vec2(p1.x, p1.z), closest_intersection))
				{
					closest_intersection = tmp_intersection;
				}
			}

			// Check against the diagonal border.
			if (std::abs(direction.x) > (direction.x < 0 ? -direction.z : direction.z) &&
			    Math::getIntersectionSegments(glm::vec2(p1.x, p1.z), glm::vec2(p2.x, p2.z), bottom_left, top_right, tmp_intersection))
			{
				if (glm::distance(glm::vec2(p1.x, p1.z), tmp_intersection) < glm::distance(glm::vec2(p1.x, p1.z), closest_intersection))
				{
					closest_intersection = tmp_intersection;
				}
			}

			// Check against the bottom border.
			if (direction.z < 0 && Math::getIntersectionSegments(glm::vec2(p1.x, p1.z), glm::vec2(p2.x, p2.z), bottom_left, bottom_right, tmp_intersection))
			{
				if (glm::distance(glm::vec2(p1.x, p1.z), tmp_intersection) < glm::distance(glm::vec2(p1.x, p1.z), closest_intersection))
				{
					closest_intersection = tmp_intersection;
				}
			}
		}

		if (closest_intersection.x != std::numeric_limits<float>::max())
		{
			float cos_alpha = (closest_intersection.x * direction.x + closest_intersection.y * direction.z) / (glm::length(closest_intersection) * glm::length(direction));
			float length = glm::length(closest_intersection) / cos_alpha;

			//glm::vec3 pn = p1 + direction * glm::length(closest_intersection);
			glm::vec3 pn = p1 + direction * (length + 0.1f);
			float height_at_p = getHeight(p1.x, p1.z);
			float height_at_pn = getHeight(pn.x, pn.z);

			if ((height_at_p < 0 && height_at_pn > 0) || (height_at_p > 0 && height_at_pn < 0))
			{
				// Found a collision!
				glm::vec3 plane_intersection;
				if (Plane::intersectsWithInfPlane(plane_p1, plane_p2, plane_p3, p1, direction, plane_intersection))
				{
					CollisionInfo ci;
					ci.colliding_entity_ = const_cast<HeightMap*>(this);
					ci.other_colliding_entity_ = &entity;
					ci.collision_loc_.push_back(plane_intersection);
					return true;
				}
				else
				{
					std::cerr << "This should not happen, a collision should have been detected!" << std::endl;
				}
			}

			p1 = pn;
		}
		else
		{
			break;
		}
	}
	return false;
	*/
}
	
bool HeightMap::doesCollide(Entity& entity, CollisionInfo& info) const
{
	if (&entity != this)
	{
		for (unsigned int i = 0; i < 8; ++i)
		{
			if (doesCollide(&entity, entity.getCollisionChecker().getPoints()[i]))
			{
				return true;
			}
		}
	}

	for (std::vector<SceneNode*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
	{
		if ((*ci)->doesCollide(entity, info))
		{
			return true;
		}
	}
	return false;
}

bool HeightMap::doesCollide(Entity& entity, const glm::vec3& begin, const glm::vec3& end, CollisionInfo& info) const
{
	if (this != &entity)
	{
		std::vector<CollisionInfo> collisions;
		if (getCollisions(entity, begin, end, collisions))
		{
			assert (collisions.size() == 1);
			info = collisions[0];
			return true;
		}
	}

	for (std::vector<SceneNode*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
	{
		if ((*ci)->doesCollide(entity, begin, end, info))
		{
			return true;
		}
	}
	return false;
}

bool HeightMap::doesCollide(Entity* entity, const glm::vec3& begin, const glm::vec3& end, float effective_width) const
{
	if (entity != this)
	{
		glm::mat4 m = glm::inverse(getCompleteTransformation());

		glm::vec3 p1 = glm::vec3(m * glm::vec4(begin, 1.0f));
		glm::vec3 p2 = glm::vec3(m * glm::vec4(end, 1.0f));
	
		// We use a very simple method, by just checking all the triangles that fall within
		// the square that is made up between the begin and end points of the lines.
		int p1_x = (p1.x / cell_size_) + (width_ / 2);
		int p1_y = (p1.z / cell_size_) + (height_ / 2);

		int p2_x = (p2.x / cell_size_) + (width_ / 2);
		int p2_y = (p2.z / cell_size_) + (height_ / 2);

		int min_x = std::min(p1_x, p2_x);
		if (min_x < 0) min_x = 0;
		if (min_x >= width_) return false;

		int max_x = std::max(p1_x, p2_x) + 1;
		if (max_x < 0) return false;
		if (max_x >= width_ - 1) max_x = width_ - 1;
	
		int min_y = std::min(p1_y, p2_y);
		if (min_y < 0) min_y = 0;
		if (min_y >= height_) return false;

		int max_y = std::max(p1_y, p2_y) + 1;
		if (max_y < 0) return false;
		if (max_y >= height_ - 1) max_y = height_ - 1;
/*
#ifdef _WIN32
		std::stringstream ss;
		ss << p1.x << " -> " << p1_x << "; " << p1.z << " -> " << p1_y << "; " << p2.x << " -> " << p2_x << "; " << p2.z << " -> " << p2_y << "; " << std::endl;
		ss << "(" << min_x << ", " << min_y << ") -- (" << max_x << ", " << max_y << ")" << std::endl;
		OutputDebugString(ss.str().c_str());
#endif
*/
		for (int x = min_x; x < max_x; ++x)
		{
			for (int y = min_y; y < max_y; ++y)
			{
				glm::vec3 top_left(x * cell_size_ - (cell_size_ / 2.0f) * (width_ - 1), (*height_map_)[x + y * width_], y * cell_size_ - (cell_size_ / 2.0f) * (height_ - 1));
				glm::vec3 top_right((x + 1) * cell_size_ - (cell_size_ / 2.0f) * (width_ - 1), (*height_map_)[x + 1 + y * width_], y * cell_size_ - (cell_size_ / 2.0f) * (height_ - 1));
				glm::vec3 bottom_left(x * cell_size_ - (cell_size_ / 2.0f) * (width_ - 1), (*height_map_)[x + (y + 1) * width_], (y + 1) * cell_size_ - (cell_size_ / 2.0f) * (height_ - 1));
				glm::vec3 bottom_right((x + 1) * cell_size_ - (cell_size_ / 2.0f) * (width_ - 1), (*height_map_)[x + 1 + (y + 1) * width_], (y + 1) * cell_size_ - (cell_size_ / 2.0f) * (height_ - 1));
			
				std::vector<glm::vec3> upper_triangle;
				upper_triangle.push_back(top_left);
				upper_triangle.push_back(top_right);
				upper_triangle.push_back(bottom_left);

				std::vector<glm::vec3> bottom_triangle;
				bottom_triangle.push_back(bottom_right);
				bottom_triangle.push_back(top_right);
				bottom_triangle.push_back(bottom_left);

				// Check the 2 triangles that make up this square.
				glm::vec3 plane_intersection;
				if (Plane::getDistanceFromPolygon(upper_triangle, p1, p2) <= effective_width)
				{
					return true;
				}
				if (Plane::getDistanceFromPolygon(bottom_triangle, p1, p2) <= effective_width)
				{
					return true;
				}
			}
		}
	}

	for (std::vector<SceneNode*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
	{
		if ((*ci)->doesCollide(entity, begin, end, effective_width))
		{
			return true;
		}
	}
	return false;
}

bool HeightMap::doesCollide(Entity* entity, const glm::vec3& point) const
{
	if (entity != this)
	{
		glm::mat4 m = glm::inverse(getCompleteTransformation());
		glm::vec4 p = m * glm::vec4(point, 1.0f);
		if (p.y <= getHeight(p.x, p.z))
		{
			return true;
		}
	}

	for (std::vector<SceneNode*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
	{
		if ((*ci)->doesCollide(entity, point))
		{
			return true;
		}
	}
	return false;
}


float HeightMap::getHeight(float x, float z) const
{
	float scaledX = x / cell_size_ + width_ / 2;
	float scaledZ = z / cell_size_ + height_ / 2;

/*
    Round down to get the x and z position near where we are
*/
    int x0 = (int)floor(scaledX);
    int z0 = (int)floor(scaledZ);

/*
    Get the 4 points surrounding the position passed in
*/
    int p0 = (z0 * width_ + x0);
    int p1 = ((z0 * width_ + x0) + 1);
    int p2 = ((z0 + 1) * width_ + x0);
    int p3 = ((z0 + 1) * width_ + x0 + 1);

    float fracX = scaledX - (float)x0;
    float fracZ = scaledZ - (float)z0;

/*
    If we are outside the bounds of the map, just return zero as the height
*/
    if (p0 >= (int)height_map_->size() ||
		p1 >= (int)height_map_->size() ||
		p2 >= (int)height_map_->size() ||
		p3 >= (int)height_map_->size() ||
		p0 < 0 || p1 < 0 || p2 < 0 || p3 < 0)
    {
        return 0.0f;
    }

/*
    Bilinearly interpolate the height values
*/
    float xInterp0 = (*height_map_)[p0] + fracX * ((*height_map_)[p1] - (*height_map_)[p0]);
    float xInterp1 = (*height_map_)[p2] + fracX * ((*height_map_)[p3] - (*height_map_)[p2]);
    return xInterp0 + fracZ * (xInterp1 - xInterp0);
}
