#ifndef CORE_GUI_GUI_ELEMENT_H
#define CORE_GUI_GUI_ELEMENT_H

#include <vector>
#include <iostream>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <GL/glew.h>
#include "themes/Theme.h"

class Container;
class Texture;

class GUIElement
{
public:
	GUIElement(const Theme& theme, float x, float y, float size_x, float size_y);

	void setPosition(float x, float y)
	{
		x_ = x;
		y_ = y;
		local_transformation_ = glm::translate(glm::mat4(1.0f), glm::vec3(x, y, 0.0f));
	}
	
	void setDimensions(float size_x, float size_y)
	{
		size_x_ = size_x;
		size_y_ = size_y;
	}

	void setParent(Container* parent)
	{
		parent_ = parent;
	}
	
	void setVisible(bool visible) { is_visible_ = visible; }
	bool isVisible() const { return is_visible_; }

	float getLocalX() const { return x_; }
	float getLocalY() const { return y_; }

	float getGlobalX() const { return global_transformation_[3][0]; }
	float getGlobalY() const { return -global_transformation_[3][1]; }

	float getWidth() const { return size_x_; }
	float getHeight() const { return size_y_; }

	/**
	 * Update the GUI element, the return value signifies whether the vertex and / or the texture coordinates
	 * have been updated. If this happens then the container where this GUI element is a child of updates the
	 * combined vertex and texture coordinates.
	 */
	virtual void update(float dt) { updateTransformations(); }

	void updateTransformations();

	/**
	 * Get the indices that are used for drawing the GUI elements.
	 */
	const std::vector<GLuint>& getIndices() const { return m_indices_; }

	/**
	 * Get the vertices.
	 */
	const std::vector<glm::vec3>& getVertexes() const { return m_vertices_; }

	/**
	 * Get the texture coordinates.
	 */
	const std::vector<glm::vec2>& getTextureCoordinates() const { return m_tex_coords_; }

	const Theme& getTheme() const { return *theme_; }
	
	Texture& getTexture() const { return *texture_; }
	void setTexture(Texture& texture) { texture_ = &texture; }

	virtual GUIElement* processMousePressEvent(int x, int y) { return NULL; } 
	virtual void processMouseReleasedEvent(int x, int y) { } 
	virtual void processMouseEnteredEvent() { } 
	virtual void processMouseLeftEvent() { }

	/**
	 * Draw this GUI element.
	 * @param perspective_matrix The perspective matrix that should be applied to all transformations of the GUI elements.
	 * @param level The 'depth' of this GUI element. This is used to apply proper clipping for GUI elements such that they do
	 * not fall ourside their containers.
	 */
	//virtual void draw(const glm::mat4& perspective_matrix, int level) const = 0;
	
	Container* getParent() const { return parent_; }
	const glm::mat4& getLocalTransformation() const { return local_transformation_; }
	const glm::mat4& getGlobalTransformation() const { return global_transformation_; }

	virtual void onResize(int width, int height);
	
	virtual void setTextureUVMapping(const std::vector<glm::vec2>& texture);

protected:
	float x_, y_, size_x_, size_y_;
	Container* parent_;

	glm::mat4 local_transformation_;
	glm::mat4 global_transformation_;
	
	std::vector<glm::vec3> m_vertices_;
	std::vector<glm::vec2> m_tex_coords_;
	std::vector<GLuint> m_indices_;

	const Theme* theme_;
	Texture* texture_;
	
	bool is_visible_;
};

#endif
