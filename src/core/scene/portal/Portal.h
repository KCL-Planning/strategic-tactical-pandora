#ifndef CORE_SCENE_PORTAL_PORTAL_H
#define CORE_SCENE_PORTAL_PORTAL_H

#include <vector>
#include <sstream>
#include <glm/glm.hpp>

class Renderer;
class Frustum;
class Region;
struct RegionPortalDebug;

class Portal
{
public:
	Portal(const std::vector<glm::vec3>& points, Region& from, Region&to);

	bool preRender(const Frustum& frustum, const glm::vec3& camera_position, Renderer& renderer, bool process_lights, unsigned int& nr_calls, unsigned int portal_depth, std::vector<const Portal*>& processed_portals, std::stringstream& ss);

	Region& getFromRegion() const { return *from_; }
	Region& getToRegion() const { return *to_; }

	void setMirrorPortal(Portal& portal) { mirror_portal_ = &portal; }

	const std::vector<glm::vec3>& getPoints() const { return points_; }
	
	Frustum getRenderingFrustum(const Frustum& frustum, const glm::vec3& camera_position, std::vector<glm::vec3>& portals) const;
	void getRenderingPortals(const Frustum& frustum, const glm::vec3& camera_position, std::vector<glm::vec3>& portals, std::vector<const Portal*>& processed_portals) const;

	bool intersectsWith(const glm::vec3& begin, const glm::vec3& end) const;

	//void getRegionsToRender(const Frustum& frustum, const glm::vec3& camera_position, unsigned int portal_depth, std::vector<const Portal*>& processed_portals, std::vector<RegionPortalDebug>& processed) const;

private:
	// The points of the portal relative to the region it is a part of.
	std::vector<glm::vec3> points_;
	glm::vec3 normal_;

	// A portal is a one-way window from one region to another.
	Region* from_, *to_;

	Portal* mirror_portal_;

	static float EPSILON;
	
	friend std::ostream& operator<<(std::ostream& os, const Portal& portal);
};

#endif

std::ostream& operator<<(std::ostream& os, const Portal& portal);
