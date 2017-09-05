#ifndef SHAPES_LINE_H
#define SHAPES_LINE_H

#include "Shape.h"

/**
 * Draws a sequence of lines.
 */
class Line : public Shape
{
public:
	/**
	 * Create a line shape and give a hint of how the lines should be rendered. 
	 * @param use_line_strip If true we use the GL_LINE_STRIP rendering command, which means that the lines form one connected string.
	 * If this is false then every pair of lines form a line segment.
	 */
	Line(bool use_line_strip = true);
	void render();
private:
	bool use_line_strip_;
};

#endif
