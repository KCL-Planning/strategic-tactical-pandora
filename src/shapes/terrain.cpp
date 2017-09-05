#include <fstream>
#include <cmath>
#include <iostream>

#include <stdlib.h>
#include <time.h>

#include "terrain.h"
#include "../core/shaders/LightShader.h"
#include "../core/entities/camera/Camera.h"
#include "../core/light/Light.h"
#include "../core/loaders/targa.h"

Terrain::Terrain()
{
    m_vertex_buffer_ = m_index_buffer_ = 0;
}

void Terrain::generateVertices(const vector<float> heights, int width)
{
	float cell_size = 8.0f;
	//Generate the vertices
	int i = 0;
	for (float z = float(-width / 2); z <= (width/2); ++z) 
	{
		for (float x = float(-width / 2); x <= (width/2); ++x) 
		{
			m_vertices_.push_back(glm::vec3(x * cell_size, heights[i++], z * cell_size));
		}
	}

	// Create the 'flipside' so the shadows are cast correctly.
	i = 0;
	for (float z = float(-width / 2); z <= (width/2); ++z) 
	{
		for (float x = float(-width / 2); x <= (width/2); ++x) 
		{
			m_vertices_.push_back(glm::vec3(x * cell_size, heights[i++] - .1f, z * cell_size));
		}
	}



	glGenBuffers(1, &m_vertex_buffer_); //Generate a buffer for the vertices
	glBindBuffer(GL_ARRAY_BUFFER, m_vertex_buffer_); //Bind the vertex buffer
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * m_vertices_.size() * 3, &m_vertices_[0], GL_STATIC_DRAW); //Send the data to OpenGL
}

void Terrain::generateIndices(int width)
{
    /*
        We loop through building the triangles that
        make up each grid square in the heightmap

        (z*w+x) *----* (z*w+x+1)
                |   /| 
                |  / | 
                | /  |
     ((z+1)*w+x)*----* ((z+1)*w+x+1)
    */
    //Generate the triangle indices
    for (int z = 0; z < width - 1; ++z) //Go through the rows - 1
    {
        for (int x = 0; x < width - 1; ++x) //And the columns - 1
        {
            m_indices_.push_back((z * width) + x); //Current point        
            m_indices_.push_back(((z + 1) * width) + x); //Next row
            m_indices_.push_back((z * width) + x + 1); //Same row, but next column

            m_indices_.push_back(((z + 1) * width) + x); //Next row
            m_indices_.push_back(((z + 1) * width) + x + 1); //Next row, next column
            m_indices_.push_back((z * width) + x + 1); //Same row, but next column
        }
    }

	int offset = width * width;
	// Create the flipside, so shadows are cast correctly.
    for (int z = 0; z < width - 1; ++z) //Go through the rows - 1
    {
        for (int x = 0; x < width - 1; ++x) //And the columns - 1
        {
            m_indices_.push_back(offset + (z * width) + x + 1); //Same row, but next column
            m_indices_.push_back(offset + ((z + 1) * width) + x); //Next row
            m_indices_.push_back(offset + (z * width) + x); //Current point        

            m_indices_.push_back(offset + (z * width) + x + 1); //Same row, but next column
            m_indices_.push_back(offset + ((z + 1) * width) + x + 1); //Next row, next column
            m_indices_.push_back(offset + ((z + 1) * width) + x); //Next row
        }
    }

    glGenBuffers(1, &m_index_buffer_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_index_buffer_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * m_indices_.size(), &m_indices_[0], GL_STATIC_DRAW);
}
/*
glm::vec3* crossProduct(glm::vec3* out, glm::vec3* v1, glm::vec3* v2)
{
    glm::vec3 v;
    v.x = (v1->y * v2->z) - (v1->z * v2->y);
	v.y = (v1->z * v2->x) - (v1->x * v2->z);
	v.z = (v1->x * v2->y) - (v1->y * v2->x);

    out->x = v.x;
    out->y = v.y;
    out->z = v.z;

    return out;
}

Vertex* normalize(Vertex* in) 
{
    float l = sqrtf(in->x * in->x + in->y * in->y + in->z * in->z);
    in->x = in->x / l;
    in->y = in->y / l;
    in->z = in->z / l;
    return in;
}
*/
void Terrain::generateNormals() 
{
    //vector<glm::vec3> faceNormals; //Temporary array to store the face normals
    vector<int> shareCount;

    m_normals_.resize(m_vertices_.size()); //We want a normal for each vertex
    shareCount.resize(m_vertices_.size());

    for (unsigned int i = 0; i < shareCount.size(); ++i) 
    {
        shareCount[i] = 0;
    }

    unsigned int numTriangles = m_indices_.size() / 3;

    //faceNormals.resize(numTriangles); //One normal per triangle

    for (unsigned int i = 0; i < numTriangles; ++i)
    {
        glm::vec3* v1 = &m_vertices_[m_indices_[i*3]];
        glm::vec3* v2 = &m_vertices_[m_indices_[(i*3)+1]];
        glm::vec3* v3 = &m_vertices_[m_indices_[(i*3)+2]];

        glm::vec3 vec1, vec2;

        vec1.x = v2->x - v1->x;
        vec1.y = v2->y - v1->y;
        vec1.z = v2->z - v1->z;

        vec2.x = v3->x - v1->x;
        vec2.y = v3->y - v1->y;
        vec2.z = v3->z - v1->z;

		
        glm::vec3 normal = glm::cross(vec1, vec2);
		//glm::vec3* normal = &faceNormals[i];
		//crossProduct(normal, &vec1, &vec2); //Calculate the normal
        normal = glm::normalize(normal);

        for (int j = 0; j < 3; ++j) 
        {
            int index = m_indices_[(i*3)+j];
            m_normals_[index].x += normal.x;
            m_normals_[index].y += normal.y;
            m_normals_[index].z += normal.z;
            shareCount[index]++;
        }
    }

    for (unsigned int i = 0; i < m_vertices_.size(); ++i)
    {
        m_normals_[i].x = m_normals_[i].x / shareCount[i];
        m_normals_[i].y = m_normals_[i].y / shareCount[i];
        m_normals_[i].z = m_normals_[i].z / shareCount[i];
        m_normals_[i] = glm::normalize(m_normals_[i]);
    }

    glGenBuffers(1, &m_normal_buffer_);
    glBindBuffer(GL_ARRAY_BUFFER, m_normal_buffer_); //Bind the vertex buffer
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * m_normals_.size() * 3, &m_normals_[0], GL_STATIC_DRAW); //Send the data to OpenGL
}

void Terrain::generateTexCoords(int width)
{
    for (int z = 0; z < width; ++z)
    {
        for (int x = 0; x < width; ++x)
        {
            float s = (float(x) / float(width)) * 8.0f;
            float t = (float(z) / float(width)) * 8.0f;
            m_tex_coords_.push_back(glm::vec2(s, t));
        }
    }

	// For the flip side.
    for (int z = 0; z < width; ++z)
    {
        for (int x = 0; x < width; ++x)
        {
            float s = (float(x) / float(width)) * 8.0f;
            float t = (float(z) / float(width)) * 8.0f;
            m_tex_coords_.push_back(glm::vec2(s, t));
        }
    }
    glGenBuffers(1, &m_tex_coord_buffer_);
    glBindBuffer(GL_ARRAY_BUFFER, m_tex_coord_buffer_); //Bind the vertex buffer
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * m_tex_coords_.size() * 2, &m_tex_coords_[0], GL_STATIC_DRAW); //Send the data to OpenGL
}

bool Terrain::createRandomHeightmap(int width, float max_height, float min_height) 
{
	width_ = width;
	heights_.reserve(width * width); //Reserve some space (faster)

	srand (time(NULL));

	//Go through the string converting each character to a float and scale it
	for (int i = 0; i < (width * width); ++i) 
	{
		float height = ((float)rand() / (float)RAND_MAX) * (max_height - min_height) + min_height;
		
		//Convert to floating value, the unsigned char cast is importantant otherwise the values wrap at 128
		heights_.push_back(height);
	}
	
	std::vector<float> old_heights = heights_;
	
	// Smooth the terrain.
	for (int i = 0; i < 2; ++i)
	{
		for (int x = 0; x < width; ++x) 
		{
			for (int y = 0; y < width; ++y)
			{
				float average_height = old_heights[x + y * width];
				float samples = 1;
				for (int dx = std::max(x - 1, 0); dx < std::min(x + 1, width); ++dx)
				{
					for (int dy = std::max(y - 1, 0); dy < std::min(y + 1, width); ++dy)
					{
						++samples;
						average_height += old_heights[dx + dy * width];
					}
				}
				old_heights[x + y * width] = average_height/ samples;
			}
		}
		heights_ = old_heights;
	}

	generateVertices(heights_, width);
	generateIndices(width);
	generateTexCoords(width);
	generateNormals();

	return true;
}

bool Terrain::createHeightmap(int width, float height) 
{
	width_ = width;
	heights_.reserve(width * width); //Reserve some space (faster)

	//Go through the string converting each character to a float and scale it
	for (int i = 0; i < (width * width); ++i) 
	{
		//Convert to floating value, the unsigned char cast is importantant otherwise the values wrap at 128
		heights_.push_back(height);
	}

	generateVertices(heights_, width);
	generateIndices(width);
	generateTexCoords(width);
	generateNormals();

	return true;
}

bool Terrain::loadHeightmap(const string& rawFile, int width) 
{
    const float HEIGHT_SCALE = 2.0f; 
    std::ifstream fileIn(rawFile.c_str(), std::ios::binary);

    if (!fileIn.good()) 
    {
#ifdef _WIN32
		MessageBox(NULL, "Could not find the heightmap file", "Error", MB_OK);
#endif
        std::cout << "File does not exist" << std::endl;
        return false;
    }

    //This line reads in the whole file into a string
    string stringBuffer(std::istreambuf_iterator<char>(fileIn), (std::istreambuf_iterator<char>()));

    fileIn.close();

    if (stringBuffer.size() != (width * width)) 
    {
#ifdef _WIN32
		MessageBox(NULL, "Image size does not match passed width", "Error", MB_OK);
#endif
        std::cout << "Image size does not match passed width" << std::endl;
        return false;
    }

	width_ = width;
    heights_.reserve(width * width); //Reserve some space (faster)

    //Go through the string converting each character to a float and scale it
    for (int i = 0; i < (width * width); ++i) 
    {
        //Convert to floating value, the unsigned char cast is importantant otherwise the values wrap at 128
        float value = (float)(unsigned char)stringBuffer[i] / 256.0f; 
    
        heights_.push_back(value * HEIGHT_SCALE);
    }

    generateVertices(heights_, width);
    generateIndices(width);
    generateTexCoords(width);
    generateNormals();

    return true;
}

bool Terrain::loadHeightmap(const TargaImage& height_map)
{
	const float HEIGHT_SCALE = 1.0f; 
    
	if (height_map.getWidth() != height_map.getHeight())
	{
		return false;
	}

	width_ = height_map.getWidth();
    heights_.reserve(width_ * width_); //Reserve some space (faster)
	
    //Go through the string converting each character to a float and scale it
    for (unsigned int i = 0; i < (width_ * width_); ++i) 
    {
        //Convert to floating value, the unsigned char cast is importantant otherwise the values wrap at 128
		float value = (float)(unsigned char)height_map.getImageData()[i * 3] / 256.0f; 
		//float value = (float)(unsigned char)0.0f; 
    
        heights_.push_back(value * HEIGHT_SCALE);
    }
	width_--;

    generateVertices(heights_, width_);
    generateIndices(width_);
    generateTexCoords(width_);
    generateNormals();

    return true;
}

GLfloat Terrain::getHeight(float x, float z) const
{
    //float halfWidth = float(width_) * 0.5f;

    //float scaledX = x + halfWidth;
    //float scaledZ = z + halfWidth;
	float scaledX = x;
	float scaledZ = z;

/*
    Round down to get the x and z position near where we are
*/
    int x0 = (int)floor(scaledX);
    int z0 = (int)floor(scaledZ);

/*
    Get the 4 points surrounding the position passed in
*/
    int p0 = (z0 * width_ + x0);
    int p1 = ((z0 * width_ + x0) + 1);
    int p2 = ((z0 + 1) * width_ + x0);
    int p3 = ((z0 + 1) * width_ + x0 + 1);

    float fracX = scaledX - (float)x0;
    float fracZ = scaledZ - (float)z0;

/*
    If we are outside the bounds of the map, just return zero as the height
*/
    if (p0 >= (int)m_vertices_.size() ||
		p1 >= (int)m_vertices_.size() ||
		p2 >= (int)m_vertices_.size() ||
		p3 >= (int)m_vertices_.size() ||
		p0 < 0 || p1 < 0 || p2 < 0 || p3 < 0)
    {
        return 0.0f;
    }

/*
    Bilinearly interpolate the height values
*/
    float xInterp0 = m_vertices_[p0].y + fracX * (m_vertices_[p1].y - m_vertices_[p0].y);
    float xInterp1 = m_vertices_[p2].y + fracX * (m_vertices_[p3].y - m_vertices_[p2].y);
    return xInterp0 + fracZ * (xInterp1 - xInterp0);
}
