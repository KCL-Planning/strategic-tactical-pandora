#include "dpengine/loaders/WavefrontLoader.h"

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>

#include "dpengine/shapes/Shape.h"

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif

namespace DreadedPE
{

std::ostream& operator<<(std::ostream& os, const Triangle& triangle)
{
	for (std::vector<const Face*>::const_iterator ci = triangle.faces_.begin(); ci != triangle.faces_.end(); ++ci)
	{
		os << "Face: v=" << (*ci)->vertex_index_ << "n=" << (*ci)->normal_index_ << "vt=" << (*ci)->texture_coordinate_index_ << std::endl;
	}
	return os;
}

Face::Face(const std::string& string)
{
	vertex_index_ = -1;//std::numeric_limits<unsigned int>::max();
	texture_coordinate_index_ = -1;//std::numeric_limits<unsigned int>::max();
	normal_index_ = -1;//std::numeric_limits<unsigned int>::max();

	size_t string_index = 0;
	size_t next_string_index;
	
	unsigned int i = 0;
	do
	{
		next_string_index = string.find_first_of('/', string_index);
		std::string token = string.substr(string_index, next_string_index - string_index);

		if (i == 0 && token.size() > 0)
		{
			vertex_index_ = ::atoi(token.c_str()) - 1;
		}
		else if (i == 1 && token.size() > 0)
		{
			texture_coordinate_index_ = ::atoi(token.c_str()) - 1;
		}
		else if (i == 2 && token.size() > 0)
		{
			normal_index_ = ::atoi(token.c_str()) - 1;
		}

		string_index = next_string_index + 1;
		++i;
	} while (next_string_index != std::string::npos);
	
	if (vertex_index_ == -1 || texture_coordinate_index_ == -1 || normal_index_ == -1)
	{
#ifdef _WIN32
		std::stringstream ss;
		ss << string << "-> " << vertex_index_ << " " << texture_coordinate_index_ << " " << normal_index_ << std::endl;
		MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#endif
	}
}

std::shared_ptr<Shape> WavefrontLoader::importShape(const std::string& filename)
{
	std::ifstream mesh_file;
	mesh_file.open(filename.c_str(), std::ios::in);

	if (!mesh_file.is_open())
	{
		std::cerr << "Could not open: " << filename << std::endl;
		return NULL;
	}

	std::string line;

	std::vector<glm::vec3> vertices;
	std::vector<glm::vec3> normals;
	std::vector<glm::vec2> texture_coordinates;
	std::vector<const Triangle*> triangles;
	while (std::getline(mesh_file, line))
	{
		std::vector<std::string> tokens;
		std::istringstream iss(line);
		std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), std::back_inserter<std::vector<std::string> >(tokens));

		if (tokens.empty())
		{
			continue;
		}

		// Vertices.
		if ("v" == tokens[0] && tokens.size() == 4)
		{
			vertices.push_back(glm::vec3(::atof(tokens[1].c_str()), ::atof(tokens[2].c_str()), ::atof(tokens[3].c_str())));
		}
		// Texture coordinates.
		else if ("vt" == tokens[0] && tokens.size() == 3)
		{
			texture_coordinates.push_back(glm::vec2(::atof(tokens[1].c_str()), ::atof(tokens[2].c_str())));
		}
		// Normals.
		else if ("vn" == tokens[0] && tokens.size() == 4)
		{
			normals.push_back(glm::vec3(::atof(tokens[1].c_str()), ::atof(tokens[2].c_str()), ::atof(tokens[3].c_str())));
		}
		// Faces. These are structured as: vertex/texture coordinate/normal.
		else if ("f" == tokens[0] && tokens.size() == 4)
		{
			std::vector<const Face*> faces;

			for (std::vector<std::string>::const_iterator ci = tokens.begin() + 1; ci != tokens.end(); ++ci)
			{
				const std::string& face_str = *ci;
				faces.push_back(new Face(face_str));
			}

			triangles.push_back(new Triangle(*faces[0], *faces[1], *faces[2]));
		}
		/*
		else
		{
#ifdef _WIN32
			std::stringstream ss;
			ss << "Skip:" << line << std::endl;
			MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#endif
		}
		*/
	}
	mesh_file.close();

	std::ofstream of;
	of.open("test.obj");

	for (std::vector<glm::vec3>::const_iterator ci = vertices.begin(); ci != vertices.end(); ++ci)
	{
		of << "v " << (*ci).x << " " << (*ci).y << " " << (*ci).z << std::endl;
	}
	for (std::vector<glm::vec3>::const_iterator ci = normals.begin(); ci != normals.end(); ++ci)
	{
		of << "vn " << (*ci).x << " " << (*ci).y << " " << (*ci).z << std::endl;
	}
	for (std::vector<glm::vec2>::const_iterator ci = texture_coordinates.begin(); ci != texture_coordinates.end(); ++ci)
	{
		of << "vt " << (*ci).x << " " << (*ci).y << std::endl;
	}
	for (std::vector<const Triangle*>::const_iterator ci = triangles.begin(); ci != triangles.end(); ++ci)
	{
		const Triangle* t = *ci;
		for (std::vector<const Face*>::const_iterator ci = t->faces_.begin(); ci != t->faces_.end(); ++ci)
		{
			of << "f " << (*ci)->vertex_index_ << " " << (*ci)->texture_coordinate_index_ << " " << (*ci)->normal_index_ << std::endl;
		}
	}

	of.close();

	/*
	for (std::vector<const Triangle*>::const_iterator ci = triangles.begin(); ci != triangles.end(); ++ci)
	{
		const Triangle* triangle = *ci;
#ifdef _WIN32
		std::stringstream ss;
		ss << *triangle << std::endl;
		MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#endif
	}
	*/

	std::vector<glm::vec3> final_vertices(vertices);
	std::vector<glm::vec3> final_normals(vertices.size(), glm::vec3());
	std::vector<glm::vec2> final_texture_coordinates(vertices.size(), glm::vec2());

	//std::vector<glm::vec3> all_normals(normals);
	//std::vector<glm::vec2> all_texture_coordinates(texture_coordinates);

	std::vector<float> normal_count;
	for (unsigned int i = 0; i < vertices.size(); ++i)
	{
		normal_count.push_back(1.0f);
	}

	// Create a mesh.
	std::vector<GLuint> indices;
	for (std::vector<const Triangle*>::const_iterator ci = triangles.begin(); ci != triangles.end(); ++ci)
	{
		const Triangle* triangle = *ci;

		for (std::vector<const Face*>::const_iterator ci = triangle->faces_.begin(); ci != triangle->faces_.end(); ++ci)
		{
			GLuint vertex_index = (*ci)->vertex_index_;
			unsigned int normal_index = (*ci)->normal_index_;
			unsigned int texture_coordinate_index = (*ci)->texture_coordinate_index_;
			
			// Check if this vertex has already been processed.
			bool found_comparable_vertex = false;
			bool could_use_comparable_vertex = false;

			for (unsigned int i = 0; i < indices.size(); ++i)
			{
				// Check if the difference between the normals are not within the bound and texture coordinate match up.
				if (indices[i] == vertex_index)
				{
					found_comparable_vertex = true;
					if (glm::dot(final_normals[vertex_index], final_normals[normal_index]) > .66f && std::abs(glm::distance(final_texture_coordinates[vertex_index], final_texture_coordinates[texture_coordinate_index])) < 0.01f)
					{
						indices.push_back(vertex_index);
						final_normals[vertex_index] = final_normals[vertex_index] + normals[normal_index];
						normal_count[vertex_index] = normal_count[vertex_index] + 1;
						could_use_comparable_vertex = true;
						break;
					}
				}
			}

			// If we found a faces with the same vertex but with a different normal vector and / or different texture coordinate
			// then we create a new vertex.
			if (found_comparable_vertex && !could_use_comparable_vertex)
			{
				indices.push_back(final_vertices.size());
				final_vertices.push_back(vertices[vertex_index]);
				final_normals.push_back(normals[normal_index]);
				normal_count.push_back(1);
				final_texture_coordinates.push_back(texture_coordinates[texture_coordinate_index]);

				//all_normals.push_back(normals[normal_index]);
				//all_texture_coordinates.push_back(texture_coordinates[texture_coordinate_index]);
			}
			// Otherwise we reuse an existing one.
			else if (!could_use_comparable_vertex)
			{
				indices.push_back(vertex_index);
				final_normals[vertex_index] = normals[normal_index];
				final_texture_coordinates[vertex_index] = texture_coordinates[texture_coordinate_index];
			}
		}
	}

	// Post process the normals.
	for (unsigned int i = 0; i < normals.size(); ++i)
	{
		final_normals[i] = final_normals[i] / normal_count[i];
	}
/*
#ifdef _WIN32
	std::stringstream ss;
	ss << "Mesh -> " << vertices.size() << " " << texture_coordinates.size() << "(" << final_texture_coordinates.size() << ") " << normals.size() << "(" << final_normals.size() << ") " << triangles.size() << std::endl;
	MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#endif
*/
	for (std::vector<const Triangle*>::const_iterator ci = triangles.begin(); ci != triangles.end(); ++ci)
	{
		delete *ci;
	}

	// Todo: If all the triangles vertex is part of have the same texture coordinate and normals then we can compress the mesh.
	return std::make_shared<Shape>(final_vertices, final_texture_coordinates, indices, final_normals);
}

};
