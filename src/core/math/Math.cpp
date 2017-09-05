#include <iostream>

#include <glm/glm.hpp>
#include "Math.h"

// dist3D_Segment_to_Segment(): get the 3D minimum distance between 2 segments
//    Input:  two 3D line segments S1 and S2
//    Return: the shortest distance between S1 and S2
float Math::dist3D_Segment_to_Segment(const glm::vec3& p1_begin, const glm::vec3& p1_end, const glm::vec3& p2_begin, const glm::vec3& p2_end)
{
	glm::vec3   u = p1_end - p1_begin;
	glm::vec3   v = p2_end - p2_begin;
	glm::vec3   w = p1_begin - p2_begin;
	float    a = glm::dot(u,u);         // always >= 0
	float    b = glm::dot(u,v);
	float    c = glm::dot(v,v);         // always >= 0
	float    d = glm::dot(u,w);
	float    e = glm::dot(v,w);
	float    D = a*c - b*b;        // always >= 0
	float    sc, sN, sD = D;       // sc = sN / sD, default sD = D >= 0
	float    tc, tN, tD = D;       // tc = tN / tD, default tD = D >= 0

	// compute the line parameters of the two closest points
	if (D < 0.01f) { // the lines are almost parallel
		float min_distance = std::numeric_limits<float>::max();
		min_distance = std::min(Math::dist3D_Segment_to_Point(p1_begin, p1_end, p2_begin), min_distance);
		min_distance = std::min(Math::dist3D_Segment_to_Point(p1_begin, p1_end, p2_end), min_distance);
		min_distance = std::min(Math::dist3D_Segment_to_Point(p2_begin, p2_end, p1_begin), min_distance);
		min_distance = std::min(Math::dist3D_Segment_to_Point(p2_begin, p2_end, p1_end), min_distance);
		return min_distance;
		
		sN = 0.0;         // force using point P0 on segment S1
		sD = 1.0;         // to prevent possible division by 0.0 later
		tN = e;
		tD = c;
		std::cout << "Almost parallel!" << std::endl;
	}
	else {                 // get the closest points on the infinite lines
		sN = (b*e - c*d);
		tN = (a*e - b*d);
		if (sN < 0.0) {        // sc < 0 => the s=0 edge is visible
			sN = 0.0;
			tN = e;
			tD = c;
		}
		else if (sN > sD) {  // sc > 1  => the s=1 edge is visible
			sN = sD;
			tN = e + b;
			tD = c;
		}
	}

	if (tN < 0.0) {            // tc < 0 => the t=0 edge is visible
		tN = 0.0;
		// recompute sc for this edge
		if (-d < 0.0)
			sN = 0.0;
		else if (-d > a)
			sN = sD;
		else {
			sN = -d;
			sD = a;
		}
	}
	else if (tN > tD) {      // tc > 1  => the t=1 edge is visible
		tN = tD;
		// recompute sc for this edge
		if ((-d + b) < 0.0)
			sN = 0;
		else if ((-d + b) > a)
			sN = sD;
		else {
			sN = (-d +  b);
			sD = a;
		}
	}
	// finally do the division to get sc and tc
	sc = (abs(sN) < 0.01f ? 0.0f : sN / sD);
	tc = (abs(tN) < 0.01f ? 0.0f : tN / tD);

	// get the difference of the two closest points
	glm::vec3   dP = w + (sc * u) - (tc * v);  // =  S1(sc) - S2(tc)

	//return sqrt(glm::dot(dP,dP));   // return the closest distance
	return glm::length(dP);
}

float Math::dist3D_Segment_to_Point(const glm::vec3& p1_begin, const glm::vec3& p1_end, const glm::vec3& point)
{
	glm::vec3 p = point - p1_begin;
	glm::vec3 line = p1_end - p1_begin;
	float line_length = glm::length(line);
	float dis_to_point = glm::length(p);

	float l = glm::dot(p, line) / line_length;
	float cos_angle = glm::dot(p, line) / (line_length * dis_to_point);

	// 1st case: the point is 'behind' the line.
	if (cos_angle < 0)
	{
		return dis_to_point;
	}
	// 2nd case: the point is 'beyond' the line.
	else if (line_length < l)
	{
		return glm::length(point - p1_end);
	}
	// The projection is on the line.
	else
	{
		// Due to floating point precision, we might end up with a negative number.
		return sqrt(std::abs(dis_to_point * dis_to_point - l * l));
	}
}

bool Math::projectsOnLine(const glm::vec3& p1_begin, const glm::vec3& p1_end, const glm::vec3& point)
{
	glm::vec3 p = point - p1_begin;
	glm::vec3 line = p1_end - p1_begin;
	float line_length = glm::length(line);
	float dis_to_point = glm::length(p);

	float l = glm::dot(p, line) / line_length;
	float cos_angle = glm::dot(p, line) / (line_length * dis_to_point);

	// 1st case: the point is 'behind' the line.
	if (cos_angle < 0)
	{
		return false;
	}
	// 2nd case: the point is 'beyond' the line.
	else if (line_length < l)
	{
		return false;
	}
	// The projection is on the line.
	else
	{
		// Due to floating point precision, we might end up with a negative number.
		return true;
	}
}


float Math::getIntersection(const glm::vec3& p1_begin, const glm::vec3& p1_end, const glm::vec3& p2_begin, const glm::vec3& p2_end, glm::vec3& p1, glm::vec3& p2)
{
	/*
	nA = dot(cross(B2-B1,A1-B1),cross(A2-A1,B2-B1));
	nB = dot(cross(A2-A1,A1-B1),cross(A2-A1,B2-B1));
	d = dot(cross(A2-A1,B2-B1),cross(A2-A1,B2-B1));
	A0 = A1 + (nA/d)*(A2-A1);
	B0 = B1 + (nB/d)*(B2-B1);
	*/

	// Check that the lines are parallel, so they will never intersect! We could calculate the distance from the point to a line
	// but for now we simply return this.
	if (glm::dot(glm::normalize(p1_end - p1_begin), glm::normalize(p2_end - p2_begin)) > 0.999f)
	{
		return std::numeric_limits<float>::max();
	}

	float nA = glm::dot(glm::cross(p2_end - p2_begin, p1_begin - p2_begin), glm::cross(p1_end - p1_begin, p2_end - p2_begin));
	float nB = glm::dot(glm::cross(p1_end - p1_begin, p1_begin - p2_begin), glm::cross(p1_end - p1_begin, p2_end - p2_begin));
	float d = glm::dot(glm::cross(p1_end - p1_begin, p2_end - p2_begin), glm::cross(p1_end - p1_begin, p2_end - p2_begin));
	p1 = p1_begin + (nA / d) * (p1_end - p1_begin);
	p2 = p2_begin + (nB / d) * (p2_end - p2_begin);
	return glm::distance(p1, p2);
}

bool Math::getIntersection(const glm::vec2& p1_begin, const glm::vec2& p1_end, const glm::vec2& p2_begin, glm::vec2& p2_end, glm::vec2& p1)
{
	// Ignore if the lines are parallel.
	if (glm::dot(p1_end - p1_begin, p2_end - p1_begin) < 0.001f) return false;
	float delta1 = (p1_end.y - p1_begin.y) / (p1_end.x - p1_begin.x);
	float delta2 = (p2_end.y - p2_begin.y) / (p2_end.x - p2_begin.x);

	float c1 = p1_begin.y - p1_begin.x * delta1;
	float c2 = p2_begin.y - p2_begin.x * delta2;
	float x = (c2 - c1) / (delta1 - delta2);
	float y = c1 + delta1 * x;
	p1.x = x;
	p1.y = y;
	return true;
}

bool Math::getIntersectionSegments(const glm::vec2& p1_begin, const glm::vec2& p1_end, const glm::vec2& p2_begin, glm::vec2& p2_end, glm::vec2& p1)
{
	glm::vec2 tmp_p;
	if (!getIntersection(p1_begin, p1_end, p2_begin, p2_end, tmp_p))
	{
		return false;
	}

	// Check if this point is on both lines.
	float min_x1 = p1_begin.x < p1_end.x ? p1_begin.x : p1_end.x;
	float min_x2 = p2_begin.x < p2_end.x ? p2_begin.x : p2_end.x;

	float max_x1 = p1_begin.x < p1_end.x ? p1_end.x : p1_begin.x;
	float max_x2 = p2_begin.x < p2_end.x ? p2_end.x : p2_begin.x;

	if (tmp_p.x >= min_x1 && tmp_p.x >= min_x2 && tmp_p.x <= max_x1 && tmp_p.x <= max_x2)
	{
		p1 = tmp_p;
		return true;
	}
	return false;
}

float Math::signedAngle(const glm::vec2& v1, const glm::vec2& v2)
{
      float perpDot = v1.x * v2.y - v1.y * v2.x;
 
      return (float)atan2(perpDot, glm::dot(v1, v2));
}

glm::fquat Math::getQuaternion(glm::vec3 u, glm::vec3 v)
{
	float norm_u_norm_v = sqrt(glm::dot(u, u) * glm::dot(v, v));
	float real_part = norm_u_norm_v + glm::dot(u, v);
	glm::vec3 w;

	if (real_part < 1.e-6f * norm_u_norm_v)
	{
		/* If u and v are exactly opposite, rotate 180 degrees
			* around an arbitrary orthogonal axis. Axis normalisation
			* can happen later, when we normalise the quaternion. */
		real_part = 0.0f;
		w = abs(u.x) > abs(u.z) ? glm::vec3(-u.y, u.x, 0.f)
								: glm::vec3(0.f, -u.z, u.y);
	}
	else
	{
		/* Otherwise, build quaternion the standard way. */
		w = glm::cross(u, v);
	}

	return glm::normalize(glm::fquat(real_part, w.x, w.y, w.z));
}
