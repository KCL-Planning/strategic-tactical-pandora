#include "dpengine/loaders/PLFCube.h"

namespace DreadedPE
{

PLF_Cube::PLF_Cube(const glm::vec3& bottom_left_away,
			const glm::vec3& bottom_right_away,
			const glm::vec3& top_left_away,
			const glm::vec3& top_right_away,
			const glm::vec3& bottom_left_close,
			const glm::vec3& bottom_right_close,
			const glm::vec3& top_left_close,
			const glm::vec3& top_right_close)
	: bottom_left_away_(bottom_left_away), bottom_right_away_(bottom_right_away), top_left_away_(top_left_away), 
		top_right_away_(top_right_away), bottom_left_close_(bottom_left_close), bottom_right_close_(bottom_right_close),
		top_left_close_(top_left_close), top_right_close_(top_right_close)
{
	{
	std::vector<glm::vec3> points;
	points.push_back(bottom_left_away_); points.push_back(bottom_right_away_); points.push_back(bottom_right_close_); points.push_back(bottom_left_close_);
	planes_.push_back(Plane(points));
	}
	{
	std::vector<glm::vec3> points;
	points.push_back(top_right_close_); points.push_back(top_right_away_); points.push_back(top_left_away_); points.push_back(top_left_close_);
	planes_.push_back(Plane(points));
	}
	{
	std::vector<glm::vec3> points;
	points.push_back(top_left_away_); points.push_back(bottom_left_away_); points.push_back(bottom_left_close_); points.push_back(top_left_close_);
	planes_.push_back(Plane(points));
	}
	{
	std::vector<glm::vec3> points;
	points.push_back(bottom_right_close_); points.push_back(bottom_right_away_); points.push_back(top_right_away_); points.push_back(top_right_close_);
	planes_.push_back(Plane(points));
	}
	{
	std::vector<glm::vec3> points;
	points.push_back(top_left_away_); points.push_back(top_right_away_); points.push_back(bottom_right_away_); points.push_back(bottom_left_away_);
	planes_.push_back(Plane(points));
	}
	{
	std::vector<glm::vec3> points;
	points.push_back(bottom_right_close_); points.push_back(top_right_close_); points.push_back(top_left_close_); points.push_back(bottom_left_close_);
	planes_.push_back(Plane(points));
	}
	
	initialise();
}

bool PLF_Cube::operator==(const PLF_Cube& rhs) const
{
	return bottom_left_away_ == rhs.bottom_left_away_ &&
				bottom_right_away_ == rhs.bottom_right_away_ &&
				top_left_away_ == rhs.top_left_away_ &&
				top_right_away_ == rhs.top_right_away_ &&
				bottom_left_close_ == rhs.bottom_left_close_ &&
				bottom_right_close_ == rhs.bottom_right_close_ &&
				top_left_close_ == rhs.top_left_close_ &&
				top_right_close_ == rhs.top_right_close_;
}

void PLF_Cube::initialise() 
{
	m_vertices_.clear();
	m_vertices_.push_back(bottom_left_away_); m_vertices_.push_back(bottom_right_away_); m_vertices_.push_back(bottom_right_close_); m_vertices_.push_back(bottom_left_close_);
	m_tex_coords_.push_back(glm::vec2(0, 1)); m_tex_coords_.push_back(glm::vec2(1, 1)); m_tex_coords_.push_back(glm::vec2(1, 0)); m_tex_coords_.push_back(glm::vec2(0, 0));
	m_normals_.push_back(glm::vec3(0, -1, 0)); m_normals_.push_back(glm::vec3(0, -1, 0)); m_normals_.push_back(glm::vec3(0, -1, 0)); m_normals_.push_back(glm::vec3(0, -1, 0));
	m_indices_.push_back(m_vertices_.size() - 2);
	m_indices_.push_back(m_vertices_.size() - 3);
	m_indices_.push_back(m_vertices_.size() - 4);

	m_indices_.push_back(m_vertices_.size() - 1);
	m_indices_.push_back(m_vertices_.size() - 2);
	m_indices_.push_back(m_vertices_.size() - 4);

	m_vertices_.push_back(top_right_close_); m_vertices_.push_back(top_right_away_); m_vertices_.push_back(top_left_away_); m_vertices_.push_back(top_left_close_);
	m_tex_coords_.push_back(glm::vec2(0, 1)); m_tex_coords_.push_back(glm::vec2(1, 1)); m_tex_coords_.push_back(glm::vec2(1, 0)); m_tex_coords_.push_back(glm::vec2(0, 0));
	m_normals_.push_back(glm::vec3(0, 1, 0)); m_normals_.push_back(glm::vec3(0, 1, 0)); m_normals_.push_back(glm::vec3(0, 1, 0)); m_normals_.push_back(glm::vec3(0, 1, 0));
	m_indices_.push_back(m_vertices_.size() - 2);
	m_indices_.push_back(m_vertices_.size() - 3);
	m_indices_.push_back(m_vertices_.size() - 4);

	m_indices_.push_back(m_vertices_.size() - 1);
	m_indices_.push_back(m_vertices_.size() - 2);
	m_indices_.push_back(m_vertices_.size() - 4);

	m_vertices_.push_back(top_left_away_); m_vertices_.push_back(bottom_left_away_); m_vertices_.push_back(bottom_left_close_); m_vertices_.push_back(top_left_close_);
	m_tex_coords_.push_back(glm::vec2(0, 1)); m_tex_coords_.push_back(glm::vec2(1, 1)); m_tex_coords_.push_back(glm::vec2(1, 0)); m_tex_coords_.push_back(glm::vec2(0, 0));
	m_normals_.push_back(glm::vec3(1, 0, 0)); m_normals_.push_back(glm::vec3(1, 0, 0)); m_normals_.push_back(glm::vec3(1, 0, 0)); m_normals_.push_back(glm::vec3(1, 0, 0));
	m_indices_.push_back(m_vertices_.size() - 2);
	m_indices_.push_back(m_vertices_.size() - 3);
	m_indices_.push_back(m_vertices_.size() - 4);

	m_indices_.push_back(m_vertices_.size() - 1);
	m_indices_.push_back(m_vertices_.size() - 2);
	m_indices_.push_back(m_vertices_.size() - 4);

	m_vertices_.push_back(bottom_right_close_); m_vertices_.push_back(bottom_right_away_); m_vertices_.push_back(top_right_away_); m_vertices_.push_back(top_right_close_);
	m_tex_coords_.push_back(glm::vec2(0, 1)); m_tex_coords_.push_back(glm::vec2(1, 1)); m_tex_coords_.push_back(glm::vec2(1, 0)); m_tex_coords_.push_back(glm::vec2(0, 0));
	m_normals_.push_back(glm::vec3(-1, 0, 0)); m_normals_.push_back(glm::vec3(-1, 0, 0)); m_normals_.push_back(glm::vec3(-1, 0, 0)); m_normals_.push_back(glm::vec3(-1, 0, 0));
	m_indices_.push_back(m_vertices_.size() - 2);
	m_indices_.push_back(m_vertices_.size() - 3);
	m_indices_.push_back(m_vertices_.size() - 4);

	m_indices_.push_back(m_vertices_.size() - 1);
	m_indices_.push_back(m_vertices_.size() - 2);
	m_indices_.push_back(m_vertices_.size() - 4);

	m_vertices_.push_back(top_left_away_); m_vertices_.push_back(top_right_away_); m_vertices_.push_back(bottom_right_away_); m_vertices_.push_back(bottom_left_away_);
	m_tex_coords_.push_back(glm::vec2(0, 1)); m_tex_coords_.push_back(glm::vec2(1, 1)); m_tex_coords_.push_back(glm::vec2(1, 0)); m_tex_coords_.push_back(glm::vec2(0, 0));
	m_normals_.push_back(glm::vec3(0, 0, -1)); m_normals_.push_back(glm::vec3(0, 0, -1)); m_normals_.push_back(glm::vec3(0, 0, -1)); m_normals_.push_back(glm::vec3(0, 0, -1));
	m_indices_.push_back(m_vertices_.size() - 2);
	m_indices_.push_back(m_vertices_.size() - 3);
	m_indices_.push_back(m_vertices_.size() - 4);

	m_indices_.push_back(m_vertices_.size() - 1);
	m_indices_.push_back(m_vertices_.size() - 2);
	m_indices_.push_back(m_vertices_.size() - 4);

	m_vertices_.push_back(bottom_right_close_); m_vertices_.push_back(top_right_close_); m_vertices_.push_back(top_left_close_); m_vertices_.push_back(bottom_left_close_);
	m_tex_coords_.push_back(glm::vec2(0, 1)); m_tex_coords_.push_back(glm::vec2(1, 1)); m_tex_coords_.push_back(glm::vec2(1, 0)); m_tex_coords_.push_back(glm::vec2(0, 0));
	m_normals_.push_back(glm::vec3(0, 0, 1)); m_normals_.push_back(glm::vec3(0, 0, 1)); m_normals_.push_back(glm::vec3(0, 0, 1)); m_normals_.push_back(glm::vec3(0, 0, 1));
	m_indices_.push_back(m_vertices_.size() - 2);
	m_indices_.push_back(m_vertices_.size() - 3);
	m_indices_.push_back(m_vertices_.size() - 4);

	m_indices_.push_back(m_vertices_.size() - 1);
	m_indices_.push_back(m_vertices_.size() - 2);
	m_indices_.push_back(m_vertices_.size() - 4);
	
	glGenBuffers(1, &m_vertex_buffer_);
	glBindBuffer(GL_ARRAY_BUFFER, m_vertex_buffer_); //Bind the vertex buffer
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * m_vertices_.size(), &m_vertices_[0], GL_STATIC_DRAW); //Send the data to OpenGL

	// To draw the upper part of the piramid.
	glGenBuffers(1, &m_index_buffer_);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_index_buffer_); //Bind the vertex buffer
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * m_indices_.size(), &m_indices_[0], GL_STATIC_DRAW); //Send the data to OpenGL

	// Generate the buffer for the texture coordinates.
	glGenBuffers(1, &m_tex_coord_buffer_);
	glBindBuffer(GL_ARRAY_BUFFER, m_tex_coord_buffer_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 2 * m_tex_coords_.size(), &m_tex_coords_[0], GL_STATIC_DRAW);

	// Generate the buffers to store the normals.
	glGenBuffers(1, &m_normal_buffer_);
	glBindBuffer(GL_ARRAY_BUFFER, m_normal_buffer_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * m_normals_.size(), &m_normals_[0], GL_STATIC_DRAW);
}
	
bool PLF_Cube::isInside(const glm::vec3& point) const
{
	for (std::vector<Plane>::const_iterator ci = planes_.begin(); ci != planes_.end(); ++ci)
	{
		const Plane& plane = *ci;
		glm::vec4 transformed_plane = glm::vec4(plane.getNormal(), plane.getD());
		float distance_to_plane = glm::dot(transformed_plane, glm::vec4(point, 1.0f));
		if (distance_to_plane < 0.01f)
		{
			return false;
		}
	}
	return true;
}

std::ostream& operator<<(std::ostream& os, const PLF_Cube& cube)
{
	os << "(" << cube.bottom_left_away_.x << "," << cube.bottom_left_away_.y << "," << cube.bottom_left_away_.z << ")" << std::endl;
	os << "(" << cube.bottom_right_away_.x << "," << cube.bottom_right_away_.y << "," << cube.bottom_right_away_.z << ")" << std::endl;
	os << "(" << cube.top_left_away_.x << "," << cube.top_left_away_.y << "," << cube.top_left_away_.z << ")" << std::endl;
	os << "(" << cube.top_right_away_.x << "," << cube.top_right_away_.y << "," << cube.top_right_away_.z << ")" << std::endl;
	os << "(" << cube.bottom_left_close_.x << "," << cube.bottom_left_close_.y << "," << cube.bottom_left_close_.z << ")" << std::endl;
	os << "(" << cube.bottom_right_close_.x << "," << cube.bottom_right_close_.y << "," << cube.bottom_right_close_.z << ")" << std::endl;
	os << "(" << cube.top_left_close_.x << "," << cube.top_left_close_.y << "," << cube.top_left_close_.z << ")" << std::endl;
	os << "(" << cube.top_right_close_.x << "," << cube.top_right_close_.y << "," << cube.top_right_close_.z << ")" << std::endl;

	for (std::vector<Plane>::const_iterator ci = cube.getPlanes().begin(); ci != cube.getPlanes().end(); ++ci)
	{
		os << *ci << std::endl;
	}

	os << "Vertices: ";
	for (std::vector<glm::vec3>::const_iterator ci = cube.getVertices().begin(); ci != cube.getVertices().end(); ++ci)
	{		
		os << "(" << (*ci).x << ", " << (*ci).y << ", " << (*ci).z << "), ";
	}
	os << std::endl;
	os << "Indices: ";
	for (std::vector<GLuint>::const_iterator ci = cube.getIndices().begin(); ci != cube.getIndices().end(); ++ci)
	{
		os << *ci << ", ";
	}
	os << std::endl;
	
	return os;
}

};
