#ifdef _WIN32
#include <windows.h>
#endif

#include <glm/glm.hpp>

#include <sstream>

#include "Wall.h"

std::ostream& operator<<(std::ostream& os, const Quad& quad)
{
	os << "Top left: " << quad.top_left_.x << ", " << quad.top_left_.y << std::endl;
	os << "Top right: " << quad.top_right_.x << ", " << quad.top_right_.y << std::endl;
	os << "Bottom right: " << quad.bottom_right_.x << ", " << quad.bottom_right_.y << std::endl;
	os << "Bottom left: " << quad.bottom_left_.x << ", " << quad.bottom_left_.y << std::endl;
	return os;
}

Wall::Wall(float width, float height, float depth, bool is_floor)
	: width_(width), height_(height), depth_(depth), is_floor_(is_floor)
{

}

void Wall::createHole(float x, float y, float width, float height)
{
	holes_.push_back(Hole(x, y, width, height));
}

void Wall::finalise()
{
	std::vector<std::pair<glm::vec2, glm::vec2> > lines;
	std::vector<bool> processed_lines;

	// Add the circumference of the square.
	lines.push_back(std::make_pair(glm::vec2(0, 0), glm::vec2(width_, 0)));
	lines.push_back(std::make_pair(glm::vec2(0, 0), glm::vec2(0, height_)));
	lines.push_back(std::make_pair(glm::vec2(width_, 0), glm::vec2(width_, height_)));
	lines.push_back(std::make_pair(glm::vec2(0, height_), glm::vec2(width_, height_)));
	for (unsigned int i = 0; i < 4; ++i)
	{
		processed_lines.push_back(true);
	}

	unsigned int hole_index = 0;
	for (std::vector<Hole>::const_iterator ci = holes_.begin(); ci != holes_.end(); ++ci)
	{
		const Hole& hole = *ci;
		lines.push_back(std::make_pair(glm::vec2(hole.x_ - hole.width_ / 2.0f, hole.y_ - hole.height_ / 2.0f), glm::vec2(hole.x_ + hole.width_ / 2.0f, hole.y_ - hole.height_ / 2.0f))); // Top left.
		lines.push_back(std::make_pair(glm::vec2(hole.x_ + hole.width_ / 2.0f, hole.y_ - hole.height_ / 2.0f), glm::vec2(hole.x_ + hole.width_ / 2.0f, hole.y_ + hole.height_ / 2.0f))); // Top right.
		lines.push_back(std::make_pair(glm::vec2(hole.x_ + hole.width_ / 2.0f, hole.y_ + hole.height_ / 2.0f), glm::vec2(hole.x_ - hole.width_ / 2.0f, hole.y_ + hole.height_ / 2.0f))); // Bottom right
		lines.push_back(std::make_pair(glm::vec2(hole.x_ - hole.width_ / 2.0f, hole.y_ + hole.height_ / 2.0f), glm::vec2(hole.x_ - hole.width_ / 2.0f, hole.y_ - hole.height_ / 2.0f))); // Bottom left.

		for (unsigned int i = 0; i < 4; ++i)
		{
			processed_lines.push_back(false);
		}

		Quad quad(glm::vec2(hole.x_ - hole.width_ / 2.0f, hole.y_ + hole.height_ / 2.0f), glm::vec2(hole.x_ + hole.width_ / 2.0f, hole.y_ + hole.height_ / 2.0f), glm::vec2(hole.x_ + hole.width_ / 2.0f, hole.y_ - hole.height_ / 2.0f), glm::vec2(hole.x_ - hole.width_ / 2.0f, hole.y_ - hole.height_ / 2.0f));

		// Connect the front end and back end of the wall.

		// Top bit.
		if (is_floor_)
		{	
			m_vertices_.push_back(glm::vec3(quad.top_left_.x, -depth_ / 2, quad.top_left_.y));
			m_vertices_.push_back(glm::vec3(quad.top_right_.x, -depth_ / 2, quad.top_right_.y));
			m_vertices_.push_back(glm::vec3(quad.top_left_.x, depth_ / 2, quad.top_left_.y));
			m_vertices_.push_back(glm::vec3(quad.top_right_.x, depth_ / 2, quad.top_right_.y));
			
		}
		else
		{
			m_vertices_.push_back(glm::vec3(quad.top_left_.x, quad.top_left_.y, depth_ / 2));
			m_vertices_.push_back(glm::vec3(quad.top_right_.x, quad.top_right_.y, depth_ / 2));
			m_vertices_.push_back(glm::vec3(quad.top_left_.x, quad.top_left_.y, -depth_ / 2));
			m_vertices_.push_back(glm::vec3(quad.top_right_.x, quad.top_right_.y, -depth_ / 2));
		}

		// Textures for front and back.
		m_tex_coords_.push_back(glm::vec2(quad.top_left_.x, quad.top_left_.y));
		m_tex_coords_.push_back(glm::vec2(quad.top_right_.x, quad.top_right_.y));
		m_tex_coords_.push_back(glm::vec2(quad.top_left_.x, quad.top_left_.y + depth_));
		m_tex_coords_.push_back(glm::vec2(quad.top_right_.x, quad.top_right_.y + depth_));
		
		// Indexes of the vertices to render -- top front to back of the wall.
		m_indices_.push_back(hole_index * 16 + 3);
		m_indices_.push_back(hole_index * 16 + 1);
		m_indices_.push_back(hole_index * 16 + 0);
		
		m_indices_.push_back(hole_index * 16 + 0);
		m_indices_.push_back(hole_index * 16 + 2);
		m_indices_.push_back(hole_index * 16 + 3);

		for (unsigned int i = 0; i < 4; ++i)
		{
			if (is_floor_)
			{
				m_normals_.push_back(glm::vec3(0.0f, 0.0f, -1.0f));
			}
			else
			{
				m_normals_.push_back(glm::vec3(0.0f, -1.0f, 0.0f));
			}
		}

		// Bottom bit.
		if (is_floor_)
		{
			m_vertices_.push_back(glm::vec3(quad.bottom_left_.x, depth_ / 2, quad.bottom_left_.y));
			m_vertices_.push_back(glm::vec3(quad.bottom_right_.x, depth_ / 2, quad.bottom_right_.y));
			m_vertices_.push_back(glm::vec3(quad.bottom_left_.x, -depth_ / 2, quad.bottom_left_.y));
			m_vertices_.push_back(glm::vec3(quad.bottom_right_.x, -depth_ / 2, quad.bottom_right_.y));
		}
		else
		{
			m_vertices_.push_back(glm::vec3(quad.bottom_left_.x, quad.bottom_left_.y, -depth_ / 2));
			m_vertices_.push_back(glm::vec3(quad.bottom_right_.x, quad.bottom_right_.y, -depth_ / 2));
			m_vertices_.push_back(glm::vec3(quad.bottom_left_.x, quad.bottom_left_.y, depth_ / 2));
			m_vertices_.push_back(glm::vec3(quad.bottom_right_.x, quad.bottom_right_.y, depth_ / 2));
		}

		m_tex_coords_.push_back(glm::vec2(quad.bottom_left_.x, quad.bottom_left_.y));
		m_tex_coords_.push_back(glm::vec2(quad.bottom_right_.x, quad.bottom_right_.y));
		m_tex_coords_.push_back(glm::vec2(quad.bottom_left_.x, quad.bottom_left_.y + depth_));
		m_tex_coords_.push_back(glm::vec2(quad.bottom_right_.x, quad.bottom_right_.y + depth_));
		
		// Indexes of the vertices to render -- top front to back of the wall.
		m_indices_.push_back(hole_index * 16 + 7);
		m_indices_.push_back(hole_index * 16 + 5);
		m_indices_.push_back(hole_index * 16 + 4);

		m_indices_.push_back(hole_index * 16 + 4);
		m_indices_.push_back(hole_index * 16 + 6);
		m_indices_.push_back(hole_index * 16 + 7);
		
		for (unsigned int i = 0; i < 4; ++i)
		{
			if (is_floor_)
			{
				m_normals_.push_back(glm::vec3(0.0f, 0.0f, 1.0f));
			}
			else
			{
				m_normals_.push_back(glm::vec3(0.0f, 1.0f, 0.0f));
			}
		}

		// Left bit.
		if (is_floor_)
		{
			m_vertices_.push_back(glm::vec3(quad.top_left_.x, -depth_ / 2, quad.top_left_.y));
			m_vertices_.push_back(glm::vec3(quad.top_left_.x, depth_ / 2, quad.top_left_.y));
			m_vertices_.push_back(glm::vec3(quad.bottom_left_.x, depth_ / 2, quad.bottom_left_.y));
			m_vertices_.push_back(glm::vec3(quad.bottom_left_.x, -depth_ / 2, quad.bottom_left_.y));
		}
		else
		{
			m_vertices_.push_back(glm::vec3(quad.bottom_left_.x, quad.bottom_left_.y, -depth_ / 2));
			m_vertices_.push_back(glm::vec3(quad.bottom_left_.x, quad.bottom_left_.y, depth_ / 2));
			m_vertices_.push_back(glm::vec3(quad.top_left_.x, quad.top_left_.y, depth_ / 2));
			m_vertices_.push_back(glm::vec3(quad.top_left_.x, quad.top_left_.y, -depth_ / 2));
		}

		m_tex_coords_.push_back(glm::vec2(quad.bottom_left_.x, quad.bottom_left_.y));
		m_tex_coords_.push_back(glm::vec2(quad.bottom_left_.x + depth_, quad.bottom_left_.y));
		m_tex_coords_.push_back(glm::vec2(quad.top_left_.x + depth_, quad.top_left_.y));
		m_tex_coords_.push_back(glm::vec2(quad.top_left_.x, quad.top_left_.y));
		
		// Indexes of the vertices to render -- top front to back of the wall.
		m_indices_.push_back(hole_index * 16 + 8);
		m_indices_.push_back(hole_index * 16 + 11);
		m_indices_.push_back(hole_index * 16 + 10);
		
		m_indices_.push_back(hole_index * 16 + 8);
		m_indices_.push_back(hole_index * 16 + 10);
		m_indices_.push_back(hole_index * 16 + 9);

		for (unsigned int i = 0; i < 4; ++i)
		{
			m_normals_.push_back(glm::vec3(1.0f, 0.0f, 0.0f));
		}

		// Right bit.
		if (is_floor_)
		{
			m_vertices_.push_back(glm::vec3(quad.bottom_right_.x, -depth_ / 2, quad.bottom_right_.y));
			m_vertices_.push_back(glm::vec3(quad.bottom_right_.x, depth_ / 2, quad.bottom_right_.y));
			m_vertices_.push_back(glm::vec3(quad.top_right_.x, depth_ / 2, quad.top_right_.y));
			m_vertices_.push_back(glm::vec3(quad.top_right_.x, -depth_ / 2, quad.top_right_.y));
		}
		else
		{
			m_vertices_.push_back(glm::vec3(quad.top_right_.x, quad.top_right_.y, -depth_ / 2));
			m_vertices_.push_back(glm::vec3(quad.top_right_.x, quad.top_right_.y, depth_ / 2));
			m_vertices_.push_back(glm::vec3(quad.bottom_right_.x, quad.bottom_right_.y, depth_ / 2));
			m_vertices_.push_back(glm::vec3(quad.bottom_right_.x, quad.bottom_right_.y, -depth_ / 2));
		}

		m_tex_coords_.push_back(glm::vec2(quad.top_right_.x, quad.top_right_.y));
		m_tex_coords_.push_back(glm::vec2(quad.top_right_.x + depth_, quad.top_right_.y));
		m_tex_coords_.push_back(glm::vec2(quad.bottom_right_.x + depth_, quad.bottom_right_.y));
		m_tex_coords_.push_back(glm::vec2(quad.bottom_right_.x, quad.bottom_right_.y));
		
		// Indexes of the vertices to render -- top front to back of the wall.
		m_indices_.push_back(hole_index * 16 + 12);
		m_indices_.push_back(hole_index * 16 + 15);
		m_indices_.push_back(hole_index * 16 + 14);

		m_indices_.push_back(hole_index * 16 + 12);
		m_indices_.push_back(hole_index * 16 + 14);
		m_indices_.push_back(hole_index * 16 + 13);
		
		for (unsigned int i = 0; i < 4; ++i)
		{
			m_normals_.push_back(glm::vec3(-1.0f, 0.0f, 0.0f));
		}

		++hole_index;
	}
	unsigned int total_nr_lines = lines.size();

	float min_height = 0.0f;

	// Process all the lines and create the quads. We process the lines starting at points firstly topmost and leftmost incase two points
	// have the same y-values.
	found_quads_.clear();
	unsigned int nr_processed_lines = 4;
	while (nr_processed_lines < total_nr_lines)
	{
		++nr_processed_lines;
		std::pair<glm::vec2, glm::vec2> selected_line_ = std::make_pair(glm::vec2(std::numeric_limits<float>::max(), std::numeric_limits<float>::max()), glm::vec2(std::numeric_limits<float>::max(), std::numeric_limits<float>::max()));

		unsigned int selected_line_index = std::numeric_limits<unsigned int>::max();
		for (unsigned int line_index = 4; line_index < total_nr_lines; ++line_index)
		{
			if (processed_lines[line_index])
			{
				continue;
			}
			const std::pair<glm::vec2, glm::vec2> current_line = lines[line_index];

			if (current_line.first.y < selected_line_.first.y || current_line.first.y == selected_line_.first.y && current_line.first.x < selected_line_.first.x)
			{
				selected_line_index = line_index;
				selected_line_ = current_line;
			}
		}
		processed_lines[selected_line_index] = true;
		min_height = std::max(min_height, selected_line_.first.y);
		min_height = std::max(min_height, selected_line_.second.y);

		std::stringstream ss;
		ss << "Selected line: (" << selected_line_.first.x << ", " << selected_line_.first.y << ") - (" << selected_line_.second.x << ", " << selected_line_.second.y << ")" << std::endl;
		for (std::vector<std::pair<glm::vec2, glm::vec2> >::const_iterator ci = lines.begin(); ci != lines.end(); ++ci)
		{
			const glm::vec2& lhs_point = (*ci).first;
			const glm::vec2& rhs_point = (*ci).second; 
			ss << "Line: (" << lhs_point.x << ", " << lhs_point.y << ") - (" << rhs_point.x << ", " << rhs_point.y << ")" << std::endl;
		}

		switch (selected_line_index % 4)
		{
		// Top left -> Top right.
		case 0:
			{
			// Go left.
			glm::vec2 left = getIntersection(lines, selected_line_.first, LEFT);
			lines.push_back(std::make_pair(left, selected_line_.first));

			// Go up.
			glm::vec2 left_up = getIntersection(lines, selected_line_.first, UP);
			lines.push_back(std::make_pair(left_up, selected_line_.first));

			Quad quad_up(glm::vec2(left.x, left_up.y), left_up, selected_line_.first, left);
			found_quads_.push_back(quad_up);

			glm::vec2 right_up = getIntersection(lines, selected_line_.second, UP);
			lines.push_back(std::make_pair(right_up, selected_line_.second));

			Quad quad_right_up(glm::vec2(selected_line_.first.x, right_up.y), right_up, selected_line_.second, selected_line_.first);
			found_quads_.push_back(quad_right_up);

			// Go right.
			glm::vec2 right = getIntersection(lines, selected_line_.second, RIGHT);
			lines.push_back(std::make_pair(selected_line_.second, right));

			Quad quad_right(right_up, glm::vec2(right.x, right_up.y), right, selected_line_.second);
			found_quads_.push_back(quad_right);

			ss << "CASE0:";
			ss << "Left: (" << left.x << ", " << left.y << ") ";
			ss << "Left up: (" << left_up.x << ", " << left_up.y << ")";
			ss << "Right up: (" << right_up.x << ", " << right_up.y << ")";
			ss << "Right: (" << right.x << ", " << right.y << ")";

			ss << "Left up: " << quad_up << std::endl;
			ss << "Right up: " << quad_right_up << std::endl;
			ss << "Right: " << quad_right << std::endl;
#ifdef _WIN32
			//MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#endif
			break;
			}
		// Top right -> Bottom right.
		case 1:
			break;
		// Bottom left -> Top left.
		case 3:
			break;

		// Bottom right -> Bottom left.
		case 2:
			{
			// Go left.
			glm::vec2 left = getIntersection(lines, selected_line_.second, LEFT);
			lines.push_back(std::make_pair(left, selected_line_.second));

			// Go up.
			glm::vec2 left_up = getIntersection(lines, selected_line_.second, UP);
			lines.push_back(std::make_pair(left_up, selected_line_.second));

			Quad quad_up(glm::vec2(left.x, left_up.y), left_up, selected_line_.second, left);
			found_quads_.push_back(quad_up);

			glm::vec2 right_up = getIntersection(lines, selected_line_.first, UP);
			lines.push_back(std::make_pair(right_up, selected_line_.first));

			//Quad quad_right_up(glm::vec2(selected_line_.first.x, right_up.y), right_up, selected_line_.first, selected_line_.second);
			//found_quads.push_back(quad_right_up);

			// Go right.
			glm::vec2 right = getIntersection(lines, selected_line_.first, RIGHT);
			lines.push_back(std::make_pair(selected_line_.first, right));

			Quad quad_right(right_up, glm::vec2(right.x, right_up.y), right, selected_line_.first);
			found_quads_.push_back(quad_right);

			ss << "CASE2:";
			ss << "Left: (" << left.x << ", " << left.y << ") ";
			ss << "Left up: (" << left_up.x << ", " << left_up.y << ")";
			ss << "Right up: (" << right_up.x << ", " << right_up.y << ")";
			ss << "Right: (" << right.x << ", " << right.y << ")";

			ss << "Left up: " << quad_up << std::endl;
//			ss << "Right up: " << quad_right_up << std::endl;
			ss << "Right: " << quad_right << std::endl;
#ifdef _WIN32
			//MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#endif
			break;
			}
		}
	}

	// Include a quad for the bottom half if necessary.
	if (min_height < height_)
	{
		found_quads_.push_back(Quad(glm::vec2(0.0f, min_height), glm::vec2(width_, min_height), glm::vec2(width_, height_), glm::vec2(0.0f, height_)));
	}

	///
	/// Given the quads, we can now create the vertices, texture coordinates, and normals.
	///
	unsigned int quad_index = 0;
	for (std::vector<Quad>::const_iterator ci = found_quads_.begin(); ci != found_quads_.end(); ++ci)
	{
		const Quad& quad = *ci;

/*
		std::stringstream ss;
		ss << "(" << quad.top_left_.x << ", " << quad.top_left_.y << ") ";
		ss << "(" << quad.top_right_.x << ", " << quad.top_right_.y << ")";
		ss << "(" << quad.bottom_right_.x << ", " << quad.bottom_right_.y << ")";
		ss << "(" << quad.bottom_left_.x << ", " << quad.bottom_left_.y << ")";
#ifdef _WIN32
		MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#endif
*/

		if (is_floor_)
		{
			// Front of the wall.
			m_vertices_.push_back(glm::vec3(quad.bottom_left_.x, depth_ / 2, quad.bottom_left_.y));
			m_vertices_.push_back(glm::vec3(quad.bottom_right_.x, depth_ / 2, quad.bottom_right_.y));
			m_vertices_.push_back(glm::vec3(quad.top_right_.x, depth_ / 2, quad.top_right_.y));
			m_vertices_.push_back(glm::vec3(quad.top_left_.x, depth_ / 2, quad.top_left_.y));

			// Back of the wall.
			m_vertices_.push_back(glm::vec3(quad.bottom_left_.x, -depth_ / 2, quad.bottom_left_.y));
			m_vertices_.push_back(glm::vec3(quad.bottom_right_.x, -depth_ / 2, quad.bottom_right_.y));
			m_vertices_.push_back(glm::vec3(quad.top_right_.x, -depth_ / 2, quad.top_right_.y));
			m_vertices_.push_back(glm::vec3(quad.top_left_.x, -depth_ / 2, quad.top_left_.y));
		}
		else
		{
			// Front of the wall.
			m_vertices_.push_back(glm::vec3(quad.top_left_.x, quad.top_left_.y, depth_ / 2));
			m_vertices_.push_back(glm::vec3(quad.top_right_.x, quad.top_right_.y, depth_ / 2));
			m_vertices_.push_back(glm::vec3(quad.bottom_right_.x, quad.bottom_right_.y, depth_ / 2));
			m_vertices_.push_back(glm::vec3(quad.bottom_left_.x, quad.bottom_left_.y, depth_ / 2));

			// Back of the wall.
			m_vertices_.push_back(glm::vec3(quad.top_left_.x, quad.top_left_.y, -depth_ / 2));
			m_vertices_.push_back(glm::vec3(quad.top_right_.x, quad.top_right_.y, -depth_ / 2));
			m_vertices_.push_back(glm::vec3(quad.bottom_right_.x, quad.bottom_right_.y, -depth_ / 2));
			m_vertices_.push_back(glm::vec3(quad.bottom_left_.x, quad.bottom_left_.y, -depth_ / 2));
		}

		// Textures for front and back.
		m_tex_coords_.push_back(glm::vec2(quad.top_left_.x, quad.top_left_.y));
		m_tex_coords_.push_back(glm::vec2(quad.top_right_.x, quad.top_right_.y));
		m_tex_coords_.push_back(glm::vec2(quad.bottom_right_.x, quad.bottom_right_.y));
		m_tex_coords_.push_back(glm::vec2(quad.bottom_left_.x, quad.bottom_left_.y));

		m_tex_coords_.push_back(glm::vec2(quad.top_left_.x, quad.top_left_.y));
		m_tex_coords_.push_back(glm::vec2(quad.top_right_.x, quad.top_right_.y));
		m_tex_coords_.push_back(glm::vec2(quad.bottom_right_.x, quad.bottom_right_.y));
		m_tex_coords_.push_back(glm::vec2(quad.bottom_left_.x, quad.bottom_left_.y));

		// Indexes of the vertices to render -- front of the wall.
		m_indices_.push_back(hole_index * 16 + quad_index * 8 + 2);
		m_indices_.push_back(hole_index * 16 + quad_index * 8 + 3);
		m_indices_.push_back(hole_index * 16 + quad_index * 8);

		m_indices_.push_back(hole_index * 16 + quad_index * 8 + 1);
		m_indices_.push_back(hole_index * 16 + quad_index * 8 + 2);
		m_indices_.push_back(hole_index * 16 + quad_index * 8);

		// Indexes of the vertices to render -- back of the wall.
		m_indices_.push_back(hole_index * 16 + quad_index * 8 + 4);
		m_indices_.push_back(hole_index * 16 + quad_index * 8 + 7);
		m_indices_.push_back(hole_index * 16 + quad_index * 8 + 6);

		m_indices_.push_back(hole_index * 16 + quad_index * 8 + 4);
		m_indices_.push_back(hole_index * 16 + quad_index * 8 + 6);
		m_indices_.push_back(hole_index * 16 + quad_index * 8 + 5);

		if (is_floor_)
		{
			for (unsigned int i = 0; i < 4; ++i)
			{
				m_normals_.push_back(glm::vec3(0.0f, 1.0f, 0.0f));
			}
			for (unsigned int i = 0; i < 4; ++i)
			{
				m_normals_.push_back(glm::vec3(0.0f, -1.0f, 0.0f));
			}
		}
		else
		{
			for (unsigned int i = 0; i < 4; ++i)
			{
				m_normals_.push_back(glm::vec3(0.0f, 0.0f, 1.0f));
			}
			for (unsigned int i = 0; i < 4; ++i)
			{
				m_normals_.push_back(glm::vec3(0.0f, 0.0f, -1.0f));
			}
		}

		++quad_index;
	}
	
	// Bind all this to OpenGL.
	// Generate buffers.
	glGenBuffers(1, &m_vertex_buffer_);
    glBindBuffer(GL_ARRAY_BUFFER, m_vertex_buffer_); //Bind the vertex buffer
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * m_vertices_.size(), &m_vertices_[0], GL_STATIC_DRAW); //Send the data to OpenGL

// 	glGenBuffers(1, &m_colour_buffer_);
//	glBindBuffer(GL_ARRAY_BUFFER, m_colour_buffer_); //Bind the vertex buffer
//	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * m_colours_.size(), &m_colours_[0], GL_STATIC_DRAW); //Send the data to OpenGL
	
	glGenBuffers(1, &m_index_buffer_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_index_buffer_); //Bind the vertex buffer
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * m_indices_.size(), &m_indices_[0], GL_STATIC_DRAW); //Send the data to OpenGL
	
	// Generate the buffer for the texture coordinates.
	glGenBuffers(1, &m_tex_coord_buffer_);
	glBindBuffer(GL_ARRAY_BUFFER, m_tex_coord_buffer_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 2 * m_tex_coords_.size(), &m_tex_coords_[0], GL_STATIC_DRAW);
	
	// Generate the buffers to store the normals.
	glGenBuffers(1, &m_normal_buffer_);
	glBindBuffer(GL_ARRAY_BUFFER, m_normal_buffer_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * m_normals_.size(), &m_normals_[0], GL_STATIC_DRAW);
}

/**
 * Lines only go up / down or left / right.
 */
glm::vec2 Wall::getIntersection(const std::vector<std::pair<glm::vec2, glm::vec2> >& lines, const glm::vec2& start, DIRECTION direction) const
{
	glm::vec2 intersecting_point;
	if (direction == DOWN || direction == RIGHT)
	{
		intersecting_point = glm::vec2(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
	}
	else
	{
		intersecting_point = glm::vec2(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
	}
	
	for (std::vector<std::pair<glm::vec2, glm::vec2> >::const_iterator ci = lines.begin(); ci != lines.end(); ++ci)
	{
		const glm::vec2& lhs_point = (*ci).first;
		const glm::vec2& rhs_point = (*ci).second;

		if (lhs_point == start || rhs_point == start)
		{
			continue;
		}

		switch (direction)
		{
		case DOWN:
			if (((start.x >= lhs_point.x && start.x <= rhs_point.x) ||
			    (start.x <= lhs_point.x && start.x >= rhs_point.x)) &&
			     start.y <= std::min(lhs_point.y, rhs_point.y))
			{
				glm::vec2 p(start.x, std::min(lhs_point.y, rhs_point.y));
				if (p.y <= intersecting_point.y)
				{
					intersecting_point = p;
				}
			}
			break;
		case UP:
			if (((start.x >= lhs_point.x && start.x <= rhs_point.x) ||
			    (start.x <= lhs_point.x && start.x >= rhs_point.x)) &&
			    start.y >= std::max(lhs_point.y, rhs_point.y))
			{
				glm::vec2 p(start.x, std::max(lhs_point.y, rhs_point.y));
				std::stringstream ss;
				ss << "Start: " << start.x << ", " << start.y << std::endl;
				ss << "Process: (" << lhs_point.x << ", " << lhs_point.y << ") - (" << rhs_point.x << ", " << rhs_point.y << ")";
				ss << "Intersecting point: (" << p.x << ", " << p.y << ").y (" << p.y << " >= " << intersecting_point.y << "?";
				
				if (p.y >= intersecting_point.y)
				{
					ss << "Update the intersection point!";
					intersecting_point = p;
				}
				else
				{
					ss << "No update :(" << p.y << " < " << intersecting_point.y << "!";
				}

#ifdef _WIN32
				//MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#endif
			}
			else
			{
				std::stringstream ss;
				ss << "Start: " << start.x << ", " << start.y << std::endl;
				ss << "Skip: (" << lhs_point.x << ", " << lhs_point.y << ") - (" << rhs_point.x << ", " << rhs_point.y << ")";
#ifdef _WIN32
				//MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#endif
			}
			break;
		case LEFT:
			if (((start.y >= lhs_point.y && start.y <= rhs_point.y) ||
			    (start.y <= lhs_point.y && start.y >= rhs_point.y)) &&
			    start.x >= std::max(lhs_point.x, rhs_point.x))
			{
				glm::vec2 p(std::max(lhs_point.x, rhs_point.x), start.y);
				if (p.x >= intersecting_point.x)
				{
					intersecting_point = p;
				}
			}
			break;
		case RIGHT:
			if (((start.y >= lhs_point.y && start.y <= rhs_point.y) ||
			    (start.y <= lhs_point.y && start.y >= rhs_point.y)) &&
			    start.x <= std::min(lhs_point.x, rhs_point.x))
			{
				glm::vec2 p(std::min(lhs_point.x, rhs_point.x), start.y);
				if (p.x <= intersecting_point.x)
				{
					intersecting_point = p;
				}
			}
			break;
		}
	}

	//if (intersecting_point.x == std::numeric_limits<float>::min() || intersecting_point.x == std::numeric_limits<float>::max())
	if (direction == UP)
	{
		std::stringstream ss;
		ss << "Get Intersection" << start.x << ", " << start.y << ") ";
		switch (direction)
		{
		case DOWN:
			ss << "DOWN. ";
			break;
		case UP:
			ss << "UP. ";
			break;
		case LEFT:
			ss << "LEFT. ";
			break;
		case RIGHT:
			ss << "RIGHT. ";
			break;
		}
		ss << std::endl;
		ss << "Intersection: " << intersecting_point.x << ", " << intersecting_point.y << std::endl;
		for (std::vector<std::pair<glm::vec2, glm::vec2> >::const_iterator ci = lines.begin(); ci != lines.end(); ++ci)
		{
			ss << "(" << (*ci).first.x << ", " << (*ci).first.y << ") - (" << (*ci).second.x << ", " << (*ci).second.y << ")";
		}

#ifdef _WIN32
		//MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#endif

	}

	return intersecting_point;
}
