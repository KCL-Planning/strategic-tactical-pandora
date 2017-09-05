#ifndef CORE_MATH_MATH
#define CORE_MATH_MATH

#include <glm/glm.hpp>
#include <glm/ext.hpp>

namespace Math
{

// dist3D_Segment_to_Segment(): get the 3D minimum distance between 2 segments
//    Input:  two 3D line segments S1 and S2
//    Return: the shortest distance between S1 and S2
float dist3D_Segment_to_Segment(const glm::vec3& p1_begin, const glm::vec3& p1_end, const glm::vec3& p2_begin, const glm::vec3& p2_end);

// Get the 3d minimum distance between a point and a line.
float dist3D_Segment_to_Point(const glm::vec3& p1_begin, const glm::vec3& p1_end, const glm::vec3& point);

// Check if a point projects on the line segment or falls outside of it.
bool projectsOnLine(const glm::vec3& p1_begin, const glm::vec3& p1_end, const glm::vec3& point);

// Get the intersection of two lines. These lines will most likely not intersect, but rather move relatively
// close to oneanother, so we get the closest point possible. If the lines are parallel then we return 
// std::numerical_limits<float>::max().
float getIntersection(const glm::vec3& p1_begin, const glm::vec3& p1_end, const glm::vec3& p2_begin, const glm::vec3& p2_end, glm::vec3& p1, glm::vec3& p2);

bool getIntersection(const glm::vec2& p1_begin, const glm::vec2& p1_end, const glm::vec2& p2_begin, glm::vec2& p2_end, glm::vec2& p1);

bool getIntersectionSegments(const glm::vec2& p1_begin, const glm::vec2& p1_end, const glm::vec2& p2_begin, glm::vec2& p2_end, glm::vec2& p1);

float signedAngle(const glm::vec2& v1, const glm::vec2& v2);

/* Build a unit quaternion representing the rotation
 * from u to v. The input vectors need not be normalised. */
glm::fquat getQuaternion(glm::vec3 u, glm::vec3 v);

};

#endif
