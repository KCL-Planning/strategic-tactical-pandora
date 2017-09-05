#ifndef DEMO_FLAT_WALL_H
#define DEMO_FLAT_WALL_H

#include "../../shapes/Shape.h"

struct Hole
{
	Hole(float x, float y, float width, float height)
		: x_(x), y_(y), width_(width), height_(height)
	{

	}

	float x_, y_, width_, height_;
};

struct Quad
{
	Quad(const glm::vec2& top_left, const glm::vec2& top_right, const glm::vec2& bottom_right, const glm::vec2& bottom_left)
		: top_left_(top_left), top_right_(top_right), bottom_right_(bottom_right), bottom_left_(bottom_left)
	{

	}

	glm::vec2 top_left_, top_right_, bottom_right_, bottom_left_;
};

std::ostream& operator<<(std::ostream& os, const Quad& quad);

/**
 * A class used to generate walls. This generates the set of vertices and texture coordinates.
 * The top left side of the wall is (0, 0) and the bottom right point is (width, height).
 */
class Wall : public Shape
{
public:
	Wall(float width, float height, float depth, bool is_floor);

	/**
	 * Create a hole:
	 * @param x The x coordinate of the centre of the hole.
	 * @param y The y coordinate of the centre of the hole.
	 * @param width The width of the hole.
	 * @param height The height of the hole.
	 */
	void createHole(float x, float y, float width, float height);

	/**
	 * Create the vertex coordinates, texture coordinates, normals, etc.
	 */
	void finalise();

	const std::vector<Quad>& getQuads() const { return found_quads_; }

	const std::vector<Hole>& getHoles() const { return holes_; }

	float getWidth() const { return width_; }
	float getHeight() const { return height_; }
	float getDepth() const { return depth_; }

private:
	float width_, height_, depth_;
	bool is_floor_;
	std::vector<Hole> holes_;

	enum DIRECTION { LEFT, RIGHT, UP, DOWN };

	// The rectangles whichh dictate how the wall is split up into quads.
	std::vector<Quad> found_quads_;

	glm::vec2 getIntersection(const std::vector<std::pair<glm::vec2, glm::vec2> >& lines, const glm::vec2& start, DIRECTION direction) const;
};

#endif
