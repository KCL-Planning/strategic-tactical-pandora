#ifndef SHAPES_SHAPE_H
#define SHAPES_SHAPE_H

#include <vector>
#include <glm/glm.hpp>
#include "GL/glew.h"

class Camera;
class GLSLProgram;
class LightManager;
class Bone;

class Shape
{
public:

	Shape() {}

	Shape(const std::vector<glm::vec3>& m_vertices,
	     const std::vector<glm::vec2>& m_tex_coords,
	     const std::vector<GLuint>& m_indices,
	     const std::vector<glm::vec3>& m_normals);

	virtual void prepare(float dt);

	virtual void render();

	GLuint getVertexBufferId() const { return m_vertex_buffer_; }
	GLuint getIndexBufferId() const { return m_index_buffer_; }
	GLuint getTexCoordBufferId() const { return m_tex_coord_buffer_; }
	GLuint getNormalBufferId() const { return m_normal_buffer_; }
	GLuint getBoneIdsBufferId() const { return m_bone_ids_buffer_; }
	GLuint getBoneWeightsBufferId() const { return m_bone_weight_buffer_; }
	GLuint getBoneIds2BufferId() const { return m_bone_ids2_buffer_; }
	GLuint getBoneWeights2BufferId() const { return m_bone_weight2_buffer_; }

	const std::vector<glm::vec3>& getVertices() const { return m_vertices_; }
	const std::vector<glm::vec2>& getTexCoords() const { return m_tex_coords_; }
	const std::vector<GLuint>& getIndices() const { return m_indices_; }
	const std::vector<glm::vec3>& getNormals() const { return m_normals_; }
	const std::vector<glm::ivec4>& getBoneIds() const { return m_bone_ids_; }
	const std::vector<glm::vec4>& getBoneWeights() const { return m_bone_weights_; }
	const std::vector<glm::ivec4>& getBoneIds2() const { return m_bone_ids2_; }
	const std::vector<glm::vec4>& getBoneWeights2() const { return m_bone_weights2_; }
	
	std::vector<glm::vec3>& getVertices() { return m_vertices_; }
	std::vector<glm::vec2>& getTexCoords() { return m_tex_coords_; }
	std::vector<GLuint>& getIndices() { return m_indices_; }
	std::vector<glm::vec3>& getNormals() { return m_normals_; }
	std::vector<glm::ivec4>& getBoneIds() { return m_bone_ids_; }
	std::vector<glm::vec4>& getBoneWeights() { return m_bone_weights_; }
	std::vector<glm::ivec4>& getBoneIds2() { return m_bone_ids2_; }
	std::vector<glm::vec4>& getBoneWeights2() { return m_bone_weights2_; }

	const std::vector<Bone*>& getBones() const { return bones_; }
	std::vector<Bone*>& getBones() { return bones_; }

	void setVertexBuffer(const std::vector<glm::vec3>& v);
	void setTexCoords(const std::vector<glm::vec2>& c);
	
protected:
	GLuint m_vertex_buffer_;
	GLuint m_index_buffer_;
	GLuint m_tex_coord_buffer_;
	GLuint m_normal_buffer_;
	GLuint m_bone_ids_buffer_;
	GLuint m_bone_weight_buffer_;
	GLuint m_bone_ids2_buffer_;
	GLuint m_bone_weight2_buffer_;

	std::vector<glm::vec3> m_vertices_;
	std::vector<glm::vec2> m_tex_coords_;
	std::vector<GLuint> m_indices_;
	std::vector<glm::vec3> m_normals_;
	std::vector<glm::ivec4> m_bone_ids_;
	std::vector<glm::vec4> m_bone_weights_;
	std::vector<glm::ivec4> m_bone_ids2_;
	std::vector<glm::vec4> m_bone_weights2_;

	std::vector<Bone*> bones_;
};

#endif
