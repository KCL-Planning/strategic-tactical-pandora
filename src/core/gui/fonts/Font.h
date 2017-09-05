#ifndef CORE_GUI_FONTS_FONT_H
#define CORE_GUI_FONTS_FONT_H

#include <vector>
#include "GL/glew.h"

#include <glm/glm.hpp>

class Texture;

/**
 * Interface for all the fonts in the engine.
 */
class Font
{
public:
	Font(Texture& texture);

	/**
	 * Set the string that should be displayed by this font. 
	 * @param text The string that should be printed by the font. 
	 */
	virtual void setString(const std::string& text, float font_size) = 0;

	/**
	 * Append a string to the font. 
	 * @param text The text to append to the existing string.
	 * @param font_size The font size this text should be drawn at.
	 */
	virtual void appendString(const glm::mat4& model_translation, const std::string& text, float font_size) = 0;

	/**
	 * Append a string to the font.
	 * @param m_vertices The vertex coordinates.
	 * @param m_tex_coords The texture coordinates.
	 * @param m_indices The indices.
	 * @param m_normals The normals.
	 */
	void appendString(const std::vector<glm::vec3>& m_vertices,
	                  const std::vector<glm::vec2>& m_tex_coords,
	                  const std::vector<GLuint>& m_indices,
	                  const std::vector<glm::vec3>& m_normals);

	/**
	 * Create a clone of this font.
	 */
	virtual Font& clone() = 0;
	
	/**
	 * Draw the font using the given perspective and model matrix.
	 * @param perspective_matrix The perspective matrix.
	 * @param model_matrix The model matrix that specifies where the font should be printed.
	 */
	virtual void draw(const glm::mat4& perspective_matrix, const glm::mat4& model_matrix) = 0;

	const std::string& getText() const { return text_; }

	Texture& getTexture() const { return *font_texture_; }

	GLuint getVertexBufferId() const { return m_vertex_buffer_; }
	GLuint getIndexBufferId() const { return m_index_buffer_; }
	GLuint getTexCoordBufferId() const { return m_tex_coord_buffer_; }
	GLuint getNormalBufferId() const { return m_normal_buffer_; }

	const std::vector<glm::vec3>& getVertices() const { return m_vertices_; }
	const std::vector<glm::vec2>& getTexCoords() const { return m_tex_coords_; }
	const std::vector<GLuint>& getIndices() const { return m_indices_; }
	const std::vector<glm::vec3>& getNormals() const { return m_normals_; }

	void clearText();

	virtual void onResize(int width, int height) { }

protected:
	void finaliseBuffers();
	inline void markBuffersForUpdate() { buffers_need_updating_ = true; }

	bool buffers_need_updating_;

	std::string text_;
	Texture* font_texture_;
	
	GLuint m_vertex_buffer_;
	GLuint m_index_buffer_;
	GLuint m_tex_coord_buffer_;
	GLuint m_normal_buffer_;
	
	std::vector<glm::vec3> m_vertices_;
	std::vector<glm::vec2> m_tex_coords_;
	std::vector<GLuint> m_indices_;
	std::vector<glm::vec3> m_normals_;
};

#endif
