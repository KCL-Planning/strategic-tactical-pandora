#ifndef CORE_GUI_CONTAINER_H
#define CORE_GUI_CONTAINER_H

#include <vector>

#include "GUIElement.h"

class Font;
class Theme;
class FontRenderingGUIElement;

/**
 * A container is a GUI item that can have other GUI components as children.
 */
class Container : public GUIElement
{
public:
	Container(const Theme& theme, Font& font, float x, float y, float size_x, float size_y, bool render_outside);

	virtual void addElement(GUIElement& child, float x, float y);
	virtual void removeElement(GUIElement& child);
	
	virtual void addElement(Container& child, float x, float y);
	virtual void removeElement(Container& child);
	
	virtual void draw(const glm::mat4& perspective_matrix, int level) const;

	virtual void update(float dt);

	GLuint getVertexBufferId() const { return vertex_buffer_; }
	GLuint getTextureCoordinatesBufferId() const { return tex_coord_buffer_; }
	GLuint getIndexBufferId() const { return index_buffer_; }
	
	float getContentSizeX() const { return content_size_x_; }
	float getContentSizeY() const { return content_size_y_; }
	
	virtual GUIElement* processMousePressEvent(int mouse_x, int mouse_y);
	virtual void processMouseReleasedEvent(int mouse_x, int mouse_y);

	GLuint getCombinedVertexBufferId() const { return combined_vertex_buffer_; }
	GLuint getCombinedTextureCoordinatesBufferId() const { return combined_tex_coord_buffer_; }
	GLuint getCombinedIndexBufferId() const { return combined_index_buffer_; }

	const std::vector<GLuint>& getCombinedIndices() const { return combined_indices_; }
	
	void setTextureUVMapping(const std::vector<glm::vec2>& texture);
	
	void updateTransformations();

	/**
	 * Set the flag that dictates that the combined buffers needs to be updated because one of the
	 * child GUI elements of the container has updated its texture coordinate, vertex  or indices buffers.
	 */
	void updateBuffers() { need_to_update_buffers_ = true; }

	virtual void onResize(int width, int height);

protected:
	std::vector<GUIElement*> children_;
	std::vector<FontRenderingGUIElement*> font_rendering_children_;
	std::vector<Container*> container_children_;
	
//private:
	float content_size_x_, content_size_y_; // Keep track of the boundaries of all the components that are part of this container.
	bool render_outside_;

	bool need_to_update_buffers_;

	// The buffers used to only render the container itself -- this is used for filling the stencil buffer.
	GLuint vertex_buffer_;
	GLuint tex_coord_buffer_;
	GLuint index_buffer_;

	std::vector<glm::vec3> vertices_;
	std::vector<glm::vec2> tex_coords_;
	std::vector<GLuint> indices_;
	
	// To speed up the rendering of containers and their content we combine all the vertices and texture coordinates in a single 
	// buffer. Before rendering we check if these need to be updated.
	GLuint combined_vertex_buffer_;
	GLuint combined_tex_coord_buffer_;
	GLuint combined_index_buffer_;

	std::vector<glm::vec3> combined_vertices_;
	std::vector<glm::vec2> combined_tex_coords_;
	std::vector<GLuint> combined_indices_;

	Font* font_;
};

#endif
