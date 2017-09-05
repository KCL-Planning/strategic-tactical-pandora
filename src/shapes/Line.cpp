#include "Line.h"

Line::Line(bool use_line_strip)
	: use_line_strip_(use_line_strip)
{
	glGenBuffers(1, &m_vertex_buffer_);
	glBindBuffer(GL_ARRAY_BUFFER, m_vertex_buffer_); //Bind the vertex buffer
}

void Line::render()
{
	//Bind the index array
	glBindBuffer(GL_ARRAY_BUFFER, getVertexBufferId());

	//Draw the triangles
	glDrawArrays(use_line_strip_ ? GL_LINE_STRIP : GL_LINES, 0, m_vertices_.size());
}
