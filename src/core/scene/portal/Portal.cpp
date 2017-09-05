#ifdef _WIN32
#include <Windows.h>
#endif

#include <algorithm>
#include <iostream>

#include "Portal.h"

#include "Region.h"
#include "../frustum/Frustum.h"
#include "../../renderer/Renderer.h"
#include "../SceneNode.h"
#include "../../math/Plane.h"

//#define HORROR_GAME_ENABLE_DEBUG

float Portal::EPSILON = 0.001f;

Portal::Portal(const std::vector<glm::vec3>& points, Region& from, Region& to)
	: points_(points), from_(&from), to_(&to), mirror_portal_(NULL)
{
	if (points.size() > 2)
	{
		normal_ = glm::normalize(glm::cross(points[1] - points[0], points[points.size() - 1] - points[0]));
	}
	else
	{
		std::cerr << "Cannot create a portal with just two points!" << std::endl;
	}
}

bool Portal::preRender(const Frustum& frustum, const glm::vec3& camera_position, Renderer& renderer, bool process_lights, unsigned int& nr_calls, unsigned int portal_depth, std::vector<const Portal*>& processed_portals, std::stringstream& ss)
{
	if (std::find(processed_portals.begin(), processed_portals.end(), this) != processed_portals.end())
	{
		/*
		std::cout << "Ignore: ";
		for (std::vector<const Portal*>::const_iterator ci = processed_portals.begin(); ci != processed_portals.end(); ++ci)
		{
			std::cout << (*ci)->from_->getName() << " -> ";
		}
		std::cout << std::endl;
		*/
		return false;
	}
	/*
	std::stringstream ss2;
	ss2 << "Process: ";
	for (std::vector<const Portal*>::const_iterator ci = processed_portals.begin(); ci != processed_portals.end(); ++ci)
	{
		ss2 << (*ci)->from_->getName() << " -> " << (*ci)->to_->getName() << " *** ";
	}
	ss2 << from_->getName() << " -> " << to_->getName();
	ss2 << std::endl;
	*/
	// Check each of the planes of the frustum and cull them accordingly.
	std::vector<glm::vec4> culled_planes;

	std::vector<glm::vec4> transformed_points;
	for (std::vector<glm::vec3>::const_iterator ci = points_.begin(); ci != points_.end(); ++ci)
	{
		transformed_points.push_back(from_->getSceneNode().getCompleteTransformation() * glm::vec4(*ci, 1.0f));
	}

#ifdef HORROR_GAME_ENABLE_DEBUG
	std::cout << "Frustum:|";
	for (std::vector<glm::vec4>::const_iterator ci = frustum.getPlanes().begin(); ci != frustum.getPlanes().end(); ++ci)
	{
		float distance_to_cam = glm::dot(*ci, glm::vec4(camera_position, 1));
		std::cout << "(" << (*ci).x << ", " << (*ci).y << ", " << (*ci).z << ", " << (*ci).w << ") Distance: " << distance_to_cam << "|";
	}
	std::cout << std::endl;
#endif

	std::vector<glm::vec4> final_points;

	// For each plane check if they cull the portal.
	//for (unsigned int plane_index = 1; plane_index < frustum.getPlanes().size(); ++plane_index)
	for (unsigned int plane_index = 0; plane_index < frustum.getPlanes().size(); ++plane_index)
	//for (std::vector<glm::vec4>::const_iterator ci = frustum.getPlanes().begin(); ci != frustum.getPlanes().end(); ++ci)
	{
		
		// For the first pass we ignore the near plane, this helps with the problem that occurs when camera position is very close to
		// the portal such that the near plane is on one side of the portal and the camera on the other side. By ignoring the near plane
		// we do not run into the situation where we consider the portal to not be visible.
		if (portal_depth == 0 && plane_index == 0)
		{
			//final_points.insert(final_points.end(), transformed_points.begin(), transformed_points.end());
			continue;
		}
		
		final_points.clear();
		/*
		ss2 << "Process the " << plane_index << "th plane. " << "(" << transformed_points.size() << ")" << std::endl;
		for (unsigned int i = 0; i < transformed_points.size(); ++i)
		{
			const glm::vec4& from_point = transformed_points[i];
			ss2 << "- (" << from_point.x << ", " << from_point.y << ", " << from_point.z << ")" << std::endl;
		}
		*/
		//const glm::vec4& plane = *ci;
		const glm::vec4& plane = frustum.getPlanes()[plane_index];
		for (unsigned int i = 0; i < transformed_points.size(); ++i)
		{
			const glm::vec4& from_point = transformed_points[i];
			const glm::vec4& to_point = transformed_points[(i + 1) % transformed_points.size()];

			float distance_from_plane = glm::dot(from_point, plane);

			// Point lies on or inside the frustum, check if the connecting point lies outside.
			// If this is the point then we need to cull the point.
			float distance_to_plane = glm::dot(to_point, plane);
			/*
//#ifdef HORROR_GAME_ENABLE_DEBUG
			ss2 << "From: " << from_point.x << "," << from_point.y << "," << from_point.z << "(" << distance_from_plane << ") - ";
			ss2 << "To: " << to_point.x << "," << to_point.y << "," << to_point.z << " (" << distance_to_plane << ").";
			ss2 << std::endl;
//#endif
			*/
			// Solve the solution: (from_point + (to_point - from_point) * x) DOT plane = 0.
			// This is solved as: x = (from_point DOT plane) / ((from_point - to_point) DOT plane).

			// The from point is inside the plane and the to point is outside of the plane. This means that we store the 
			// point that intersects with the plane and move on to the next point.
			if (distance_from_plane >= 0 && distance_to_plane < -EPSILON)
			{
				float x = glm::dot(from_point, plane) / glm::dot((from_point - to_point), plane);
				glm::vec4 intermediate_point = from_point + x * (to_point - from_point);
				final_points.push_back(intermediate_point);
//#ifdef HORROR_GAME_ENABLE_DEBUG
				//ss2 << "Split1: (" << intermediate_point.x << ", " << intermediate_point.y << ", " << intermediate_point.z << ")";
//#endif
			}

			// The to point is inside the plane and the from point is outside. This means that we store the point that 
			// intersects with the plane and the to point to the list of new points and move on.
			else if (distance_from_plane < -EPSILON && distance_to_plane >= 0)
			{
				float x = glm::dot(from_point, plane) / glm::dot((from_point - to_point), plane);
				glm::vec4 intermediate_point = from_point + x * (to_point - from_point);
				final_points.push_back(intermediate_point);
				final_points.push_back(to_point);
//#ifdef HORROR_GAME_ENABLE_DEBUG
				//ss2 << "Split2: (" << intermediate_point.x << ", " << intermediate_point.y << ", " << intermediate_point.z << ")" << std::endl;
//#endif
			}
			// If both points lie on the possive side of the plane then we do not have to split the
			// line and we can leave the end point as it is.
			else if (distance_from_plane >= -EPSILON && distance_to_plane >= -EPSILON)
			{
//#ifdef HORROR_GAME_ENABLE_DEBUG
				//ss2 << "Keep: (" << to_point.x << ", " << to_point.y << ", " << to_point.z << ")" << std::endl;
//#endif
				final_points.push_back(to_point);
			}
//#ifdef HORROR_GAME_ENABLE_DEBUG2
			// If both points lie on the negative side of the plane then we can ignore the to point.
			// The from point might be part of a line that intersects with the plane (i.e. with the line
			// (i - 1)).
			else //if (distance_from_plane < -EPSILON && distance_to_plane < -EPSILON)
			{
				//ss2 << "Remove: (" << to_point.x << ", " << to_point.y << ", " << to_point.z << ")" << std::endl;
				// Ignore it.
			}
			//ss2 << "|";
//#endif
		}

		transformed_points.clear();
		
		if (final_points.empty())
		{
			//ss << "No go!";
			break;
		}
		transformed_points.insert(transformed_points.end(), final_points.begin(), final_points.end());
#ifdef HORROR_GAME_ENABLE_DEBUG
		ss << "|";
#endif
	}

	//OutputDebugString(ss2.str().c_str());

	// Create a new frustum and render the region on the other side of the portal -- if there is a frustum left!
	if (transformed_points.size() > 0)
	{
#ifdef HORROR_GAME_ENABLE_DEBUG
		ss2 << "Portal enabled!";
		for (std::vector<glm::vec4>::const_iterator ci = transformed_points.begin(); ci != transformed_points.end(); ++ci)
		{
			ss2 << (*ci).x << ", " << (*ci).y << ", " << (*ci).z << "|";
		}
#endif
		//ss << "Lets go!";
		std::vector<glm::vec4> planes;
		
		// The near plane will become the portal, although the normal needs to be flipped.
		glm::vec3 point_on_portal = glm::vec3(from_->getSceneNode().getCompleteTransformation() * glm::vec4(points_[0], 1.0f));
		planes.push_back(glm::vec4(-normal_, -glm::dot(-normal_, point_on_portal)));
		
		// Keep the near and far planes.
		//planes.push_back(frustum.getPlanes()[0]);
		planes.push_back(frustum.getPlanes()[1]);

		for (unsigned int i = 0; i < transformed_points.size(); ++i)
		{
			glm::vec3 cross_product = glm::cross(glm::vec3(transformed_points[(i + 1) % transformed_points.size()]) - camera_position, glm::vec3(transformed_points[i]) - camera_position);
			glm::vec3 N = glm::normalize(cross_product);
			planes.push_back(glm::vec4(N, -glm::dot(N, camera_position)));
#ifdef HORROR_GAME_ENABLE_DEBUG
			ss << N.x << ", " << N.y << ", " << N.z << ", " << -glm::dot(N, camera_position) << "|";
#endif
		}
		
		/*
		planes.push_back(frustum.getPlanes()[2]);
		planes.push_back(frustum.getPlanes()[3]);
		planes.push_back(frustum.getPlanes()[4]);
		planes.push_back(frustum.getPlanes()[5]);
		*/
		//std::cout << "*** BEGIN *** " << std::endl;
		//std::cout << *this << std::endl;
		/*
		for (std::vector<glm::vec4>::const_iterator ci = planes.begin(); ci != planes.end(); ++ci)
		{
			std::cout << "(" << (*ci).x << ", " << (*ci).y << ", " << (*ci).z << ", " << (*ci).w << ") -- ";
		}
		std::cout << "*** END *** " << std::endl;
		*/
		
		std::vector<const Portal*> processed_portals_copy(processed_portals);
		processed_portals_copy.push_back(this);
		if (mirror_portal_ != NULL)
		{
			processed_portals_copy.push_back(mirror_portal_);
		}
		/*
		std::stringstream ss3;
		ss3 << "Enable portal: " << from_->getName() << " -> " << to_->getName() << std::endl;
		OutputDebugString(ss3.str().c_str());
		*/
		Frustum culled_frustum(planes);
		to_->preRender(culled_frustum, camera_position, renderer, process_lights, nr_calls, portal_depth + 1, processed_portals_copy, ss);
		return true;
	}
	//std::cout << "Outside of the frustum!" << std::endl;
	return false;
}

Frustum Portal::getRenderingFrustum(const Frustum& frustum, const glm::vec3& camera_position, std::vector<glm::vec3>& portals) const
{
	// Check each of the planes of the frustum and cull them accordingly.
	std::vector<glm::vec4> culled_planes;

	std::vector<glm::vec4> transformed_points;
	for (std::vector<glm::vec3>::const_iterator ci = points_.begin(); ci != points_.end(); ++ci)
	{
		transformed_points.push_back(from_->getSceneNode().getCompleteTransformation() * glm::vec4(*ci, 1.0f));
	}

	std::vector<glm::vec4> final_points;

	// For each plane check if they cull the portal -- we ignore the near and far planes.
	//for (std::vector<glm::vec4>::const_iterator ci = frustum.getPlanes().begin(); ci != frustum.getPlanes().end(); ++ci)
	for (unsigned int plane_index = 2; plane_index < frustum.getPlanes().size(); ++plane_index)
	{
		/*
		// For the first pass we ignore the near plane, this helps with the problem that occurs when camera position is very close to
		// the portal such that the near plane is on one side of the portal and the camera on the other side. By ignoring the near plane
		// we do not run into the situation where we consider the portal to not be visible.
		if ((ci == frustum.getPlanes().begin()))// || ci == frustum.getPlanes().begin() + 1))
		{
			final_points.insert(final_points.end(), transformed_points.begin(), transformed_points.end());
			continue;
		}
		*/
		final_points.clear();

		//const glm::vec4& plane = *ci;
		const glm::vec4& plane = frustum.getPlanes()[plane_index];
		for (unsigned int i = 0; i < transformed_points.size(); ++i)
		{
			const glm::vec4& from_point = transformed_points[i];
			const glm::vec4& to_point = transformed_points[(i + 1) % transformed_points.size()];

			float distance_from_plane = glm::dot(from_point, plane);

			// Point lies on or inside the frustum, check if the connecting point lies outside.
			// If this is the point then we need to cull the point.
			float distance_to_plane = glm::dot(to_point, plane);

			// Solve the solution: (from_point + (to_point - from_point) * x) DOT plane = 0.
			// This is solved as: x = (from_point DOT plane) / ((from_point - to_point) DOT plane).
			
			// The from point is inside the plane and the to point is outside of the plane. This means that we store the 
			// point that intersects with the plane and move on to the next point.
			if (distance_from_plane >= 0 && distance_to_plane < -EPSILON)
			{
				float x = glm::dot(from_point, plane) / glm::dot((from_point - to_point), plane);
				glm::vec4 intermediate_point = from_point + x * (to_point - from_point);
				final_points.push_back(intermediate_point);
			}

			// The to point is inside the plane and the from point is outside. This means that we store the point that 
			// intersects with the plane and the to point to the list of new points and move on.
			else if (distance_from_plane < -EPSILON && distance_to_plane >= 0)
			{
				float x = glm::dot(from_point, plane) / glm::dot((from_point - to_point), plane);
				glm::vec4 intermediate_point = from_point + x * (to_point - from_point);
				final_points.push_back(intermediate_point);
				final_points.push_back(to_point);
			}
			// If both points lie on the possive side of the plane then we do not have to split the
			// line and we can leave the end point as it is.
			else if (distance_from_plane >= -EPSILON && distance_to_plane >= -EPSILON)
			{
				final_points.push_back(to_point);
			}
		}

		transformed_points.clear();
		
		if (final_points.empty())
		{
			break;
		}
		transformed_points.insert(transformed_points.end(), final_points.begin(), final_points.end());
	}

	// Create a new frustum and render the region on the other side of the portal -- if there is a frustum left!
	if (transformed_points.size() > 0)
	{
		std::vector<glm::vec4> planes;
		// The near plane will become the portal, although the normal needs to be flipped.
		glm::vec3 point_on_portal = glm::normalize(glm::vec3(from_->getSceneNode().getCompleteTransformation() * glm::vec4(points_[0], 1.0f)));
		planes.push_back(glm::vec4(-normal_, -glm::dot(-normal_, point_on_portal)));
		
		// Keep the near and far planes.
		//planes.push_back(frustum.getPlanes()[0]);
		planes.push_back(frustum.getPlanes()[1]);
		
		for (unsigned int i = 0; i < transformed_points.size(); ++i)
		{
			glm::vec3 cross_product = glm::cross(glm::vec3(transformed_points[(i + 1) % transformed_points.size()]) - camera_position, glm::vec3(transformed_points[i]) - camera_position);
			glm::vec3 N = glm::normalize(cross_product);
			planes.push_back(glm::vec4(N, -glm::dot(N, camera_position)));

			portals.push_back(glm::vec3(transformed_points[i]));
			portals.push_back(glm::vec3(transformed_points[(i + 1) % transformed_points.size()]));
		}

		Frustum culled_frustum(planes);
		return culled_frustum;
	}
	return frustum;
}

void Portal::getRenderingPortals(const Frustum& frustum, const glm::vec3& camera_position, std::vector<glm::vec3>& portals, std::vector<const Portal*>& processed_portals) const
{
	if (std::find(processed_portals.begin(), processed_portals.end(), this) != processed_portals.end())
	{
		return;
	}
	// Check each of the planes of the frustum and cull them accordingly.
	std::vector<glm::vec4> culled_planes;

	std::vector<glm::vec4> transformed_points;
	for (std::vector<glm::vec3>::const_iterator ci = points_.begin(); ci != points_.end(); ++ci)
	{
		transformed_points.push_back(from_->getSceneNode().getCompleteTransformation() * glm::vec4(*ci, 1.0f));
	}

	std::vector<glm::vec4> final_points;

	// For each plane check if they cull the portal.
	//for (unsigned int plane_index = 1; plane_index < frustum.getPlanes().size(); ++plane_index)
	for (unsigned int plane_index = 0; plane_index < frustum.getPlanes().size(); ++plane_index)
	{
		final_points.clear();
		
		const glm::vec4& plane = frustum.getPlanes()[plane_index];
		for (unsigned int i = 0; i < transformed_points.size(); ++i)
		{
			const glm::vec4& from_point = transformed_points[i];
			const glm::vec4& to_point = transformed_points[(i + 1) % transformed_points.size()];

			float distance_from_plane = glm::dot(from_point, plane);

			// Point lies on or inside the frustum, check if the connecting point lies outside.
			// If this is the point then we need to cull the point.
			float distance_to_plane = glm::dot(to_point, plane);
		
			// Solve the solution: (from_point + (to_point - from_point) * x) DOT plane = 0.
			// This is solved as: x = (from_point DOT plane) / ((from_point - to_point) DOT plane).

			// The from point is inside the plane and the to point is outside of the plane. This means that we store the 
			// point that intersects with the plane and move on to the next point.
			if (distance_from_plane >= 0 && distance_to_plane < -EPSILON)
			{
				float x = glm::dot(from_point, plane) / glm::dot((from_point - to_point), plane);
				glm::vec4 intermediate_point = from_point + x * (to_point - from_point);
				final_points.push_back(intermediate_point);
			}

			// The to point is inside the plane and the from point is outside. This means that we store the point that 
			// intersects with the plane and the to point to the list of new points and move on.
			else if (distance_from_plane < -EPSILON && distance_to_plane >= 0)
			{
				float x = glm::dot(from_point, plane) / glm::dot((from_point - to_point), plane);
				glm::vec4 intermediate_point = from_point + x * (to_point - from_point);
				final_points.push_back(intermediate_point);
				final_points.push_back(to_point);
			}
			// If both points lie on the possive side of the plane then we do not have to split the
			// line and we can leave the end point as it is.
			else if (distance_from_plane >= -EPSILON && distance_to_plane >= -EPSILON)
			{
				final_points.push_back(to_point);
			}
			// If both points lie on the negative side of the plane then we can ignore the to point.
			// The from point might be part of a line that intersects with the plane (i.e. with the line
			// (i - 1)).
			else //if (distance_from_plane < -EPSILON && distance_to_plane < -EPSILON)
			{
				// Ignore it.
			}
		}

		transformed_points.clear();
		
		if (final_points.empty())
		{
			break;
		}
		transformed_points.insert(transformed_points.end(), final_points.begin(), final_points.end());
	}

	// Create a new frustum and render the region on the other side of the portal -- if there is a frustum left!
	if (transformed_points.size() > 0)
	{
#ifdef HORROR_GAME_ENABLE_DEBUG
		ss2 << "Portal enabled!";
		for (std::vector<glm::vec4>::const_iterator ci = transformed_points.begin(); ci != transformed_points.end(); ++ci)
		{
			ss2 << (*ci).x << ", " << (*ci).y << ", " << (*ci).z << "|";
		}
#endif
		//ss << "Lets go!";
		std::vector<glm::vec4> planes;
		
		// The near plane will become the portal, although the normal needs to be flipped.
		glm::vec3 point_on_portal = glm::vec3(from_->getSceneNode().getCompleteTransformation() * glm::vec4(points_[0], 1.0f));
		planes.push_back(glm::vec4(-normal_, -glm::dot(-normal_, point_on_portal)));
		
		// Keep the near and far planes.
		//planes.push_back(frustum.getPlanes()[0]);
		planes.push_back(frustum.getPlanes()[1]);

		for (unsigned int i = 0; i < transformed_points.size(); ++i)
		{
			glm::vec3 cross_product = glm::cross(glm::vec3(transformed_points[(i + 1) % transformed_points.size()]) - camera_position, glm::vec3(transformed_points[i]) - camera_position);
			glm::vec3 N = glm::normalize(cross_product);
			planes.push_back(glm::vec4(N, -glm::dot(N, camera_position)));
		}
		std::vector<const Portal*> processed_portals_copy(processed_portals);
		processed_portals_copy.push_back(this);
		if (mirror_portal_ != NULL)
		{
			processed_portals_copy.push_back(mirror_portal_);
		}
		Frustum culled_frustum(planes);
		to_->getRenderingPortals(culled_frustum, camera_position, portals, processed_portals);
	}
}

bool Portal::intersectsWith(const glm::vec3& begin, const glm::vec3& end) const
{
	Plane plane(points_);
	glm::vec3 intersecting_point;
	return plane.intersectsWith(begin, end, intersecting_point);
}

std::ostream& operator<<(std::ostream& os, const Portal& portal)
{
	os << "Portal: " << portal.from_->getName() << "->" << portal.to_->getName() << ": ";
	for (std::vector<glm::vec3>::const_iterator ci = portal.points_.begin(); ci != portal.points_.end(); ++ci)
	{
		os << "(" << (*ci).x << ", " << (*ci).y << ", " << (*ci).z << ") -- ";
	}
	os << "; Normal: (" << portal.normal_.x << ", " << portal.normal_.y << ", " << portal.normal_.z << ")";
	return os;
}
