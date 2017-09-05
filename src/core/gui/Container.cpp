#include "Container.h"
#include "themes/Theme.h"

#include <GL/glfw.h>

#include "../shaders/GUIShader.h"
#include "../shaders/LineShader.h"

#include "fonts/FontRenderingGUIElement.h"
#include "fonts/Font.h"

Container::Container(const Theme& theme, Font& font, float x, float y, float size_x, float size_y, bool render_outside)
	: GUIElement(theme, x, y, size_x, size_y), font_(&font), content_size_x_(-std::numeric_limits<float>::max()), content_size_y_(-std::numeric_limits<float>::max()), render_outside_(render_outside), need_to_update_buffers_(true)
{
	int width, height;
	glfwGetWindowSize(&width, &height);
	
	m_vertices_.push_back(glm::vec3(0, height, 0));
	m_vertices_.push_back(glm::vec3(size_x, height, 0));
	m_vertices_.push_back(glm::vec3(0, height - size_y, 0));
	m_vertices_.push_back(glm::vec3(size_x, height - size_y, 0));
	
	//m_tex_coords_ = theme.getFrameTexture();
	m_tex_coords_ = theme.getInvisibleTexture();
	//m_tex_coords_ = theme.getMaximiseTexture();

	m_indices_.push_back(1);
	m_indices_.push_back(0);
	m_indices_.push_back(2);

	m_indices_.push_back(1);
	m_indices_.push_back(2);
	m_indices_.push_back(3);

	// Generate the combined buffers.
	combined_vertices_ = m_vertices_;
	glGenBuffers(1, &combined_vertex_buffer_);
	glBindBuffer(GL_ARRAY_BUFFER, combined_vertex_buffer_); //Bind the vertex buffer
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * combined_vertices_.size(), &combined_vertices_[0], GL_DYNAMIC_DRAW); //Send the data to OpenGL

	// Generate the buffer for the texture coordinates.
	combined_tex_coords_ = m_tex_coords_;
	glGenBuffers(1, &combined_tex_coord_buffer_);
	glBindBuffer(GL_ARRAY_BUFFER, combined_tex_coord_buffer_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 2 * combined_tex_coords_.size(), &combined_tex_coords_[0], GL_DYNAMIC_DRAW);

	// Generate the buffer for the indices.
	combined_indices_ = m_indices_;
	glGenBuffers(1, &combined_index_buffer_);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, combined_index_buffer_);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * combined_indices_.size(), &combined_indices_[0], GL_DYNAMIC_DRAW);

	// Generate the buffers used for filling in the stencil buffer.
	glGenBuffers(1, &vertex_buffer_);
	glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_); //Bind the vertex buffer
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * combined_vertices_.size(), &combined_vertices_[0], GL_STATIC_DRAW); //Send the data to OpenGL

	// Generate the buffer for the texture coordinates.
	glGenBuffers(1, &tex_coord_buffer_);
	glBindBuffer(GL_ARRAY_BUFFER, tex_coord_buffer_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 2 * combined_tex_coords_.size(), &combined_tex_coords_[0], GL_STATIC_DRAW);

	// Generate the buffer for the indices.
	glGenBuffers(1, &index_buffer_);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer_);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * combined_indices_.size(), &combined_indices_[0], GL_STATIC_DRAW);
}

void Container::setTextureUVMapping(const std::vector<glm::vec2>& texture)
{
	m_tex_coords_ = texture;
	need_to_update_buffers_ = true;
}

void Container::updateTransformations()
{
	if (parent_ == NULL)
	{
		global_transformation_ = local_transformation_;
	}
	else
	{
		global_transformation_ = parent_->global_transformation_ * local_transformation_;
	}
}

void Container::addElement(GUIElement& child, float x, float y)
{
	child.setParent(this);
	children_.push_back(&child);
	child.setPosition(x, y);
	
	FontRenderingGUIElement* font_rendering_child = dynamic_cast<FontRenderingGUIElement*>(&child);
	if (font_rendering_child != NULL)
	{
		font_rendering_children_.push_back(font_rendering_child);
	}

	content_size_x_ = std::max(content_size_x_, x + child.getWidth());
	content_size_y_ = std::max(content_size_y_, -y + child.getHeight());
}

void Container::removeElement(GUIElement& child)
{
	for (std::vector<GUIElement*>::iterator i = children_.begin(); i != children_.end(); ++i)
	{
		if (*i == &child)
		{
			for (std::vector<FontRenderingGUIElement*>::iterator i = font_rendering_children_.begin(); i != font_rendering_children_.end(); ++i)
			{
				if (*i == &child)
				{
					font_rendering_children_.erase(i);
					break;
				}
			}
			child.setParent(NULL);
			children_.erase(i);
			return;
		}
	}
}

void Container::addElement(Container& child, float x, float y)
{
	child.setParent(this);
	container_children_.push_back(&child);
	child.setPosition(x, y);
	
	content_size_x_ = std::max(content_size_x_, x + child.getWidth());
	content_size_y_ = std::max(content_size_y_, -y + child.getHeight());
}

void Container::removeElement(Container& child)
{
	for (std::vector<Container*>::iterator i = container_children_.begin(); i != container_children_.end(); ++i)
	{
		if (*i == &child)
		{
			child.setParent(NULL);
			container_children_.erase(i);
			return;
		}
	}
}

void Container::update(float dt)
{
	//m_tex_coords_ = theme_->getMaximiseTexture();
	GUIElement::update(dt);
	for (std::vector<GUIElement*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
	{
		// Check if this element is clicked on by the mouse.
		(*ci)->update(dt);
	}

	// Update the combined buffers, if necessary.
	if (need_to_update_buffers_)
	{
		need_to_update_buffers_ = false;
		combined_tex_coords_.clear();
		combined_vertices_.clear();
		combined_indices_.clear();

		// First draw the container bit.
		combined_vertices_.insert(combined_vertices_.end(), m_vertices_.begin(), m_vertices_.end());
		combined_tex_coords_.insert(combined_tex_coords_.end(), m_tex_coords_.begin(), m_tex_coords_.end());
		combined_indices_.insert(combined_indices_.end(), m_indices_.begin(), m_indices_.end());
/*
#ifdef _WIN32
			std::stringstream ss2;
			ss2 << "[Vertices buffer] " << combined_vertex_buffer_ << "; [Texture coordinate buffer] " << combined_tex_coord_buffer_ << "; [Indicies buffer] " << combined_index_buffer_ << std::endl;
			OutputDebugString(ss2.str().c_str());
#endif
*/	
		// Then draw its children.
		GLuint indices_offset = combined_vertices_.size();
		for (std::vector<GUIElement*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
		{
			GUIElement* child = *ci;
			
			if (!child->isVisible())
			{
				continue;
			}
/*
#ifdef _WIN32
			std::stringstream ss;
			ss << "[" << this << "]" << std::endl;
#endif
*/
			for (std::vector<glm::vec3>::const_iterator ci = child->getVertexes().begin(); ci != child->getVertexes().end(); ++ci)
			{
				glm::vec4 v(*ci, 1.0f);
				v = child->getLocalTransformation() * v;
				combined_vertices_.push_back(glm::vec3(v));
/*
#ifdef _WIN32
				ss << "(" << (*ci).x << ", " << (*ci).y << ", " << (*ci).z << ") -=- (" << v.x << ", " << v.y << ", " << v.z << ")" << std::endl;
#endif
*/
			}
/*
#ifdef _WIN32
			OutputDebugString(ss.str().c_str());
#endif
*/
			combined_tex_coords_.insert(combined_tex_coords_.end(), child->getTextureCoordinates().begin(), child->getTextureCoordinates().end());
			//for (std::vector<GLuint>::const_iterator ci = child->getIndices().begin(); ci != child-getIndices().end(); ++ci)
			for (unsigned int i = 0; i < child->getIndices().size(); ++i)
			{
				combined_indices_.push_back(child->getIndices()[i] + indices_offset);
			}

			indices_offset += child->getVertexes().size();
		}

		glBindBuffer(GL_ARRAY_BUFFER, combined_vertex_buffer_);
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * combined_vertices_.size(), &combined_vertices_[0], GL_DYNAMIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, combined_tex_coord_buffer_);
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 2 * combined_tex_coords_.size(), &combined_tex_coords_[0], GL_DYNAMIC_DRAW);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, combined_index_buffer_);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLfloat) * combined_indices_.size(), &combined_indices_[0], GL_DYNAMIC_DRAW);
/*
#ifdef _WIN32
		std::stringstream ss;
		ss << "Updated the container with the following arrays: " << combined_vertices_.size() << " " << combined_tex_coords_.size() << " " << combined_indices_.size() << "." << std::endl;
		OutputDebugString(ss.str().c_str());
#endif
*/
	}
	
	// Check if we need to update the font.
	bool need_to_update_font = false;
	for (std::vector<FontRenderingGUIElement*>::const_iterator ci = font_rendering_children_.begin(); ci != font_rendering_children_.end(); ++ci)
	{
		FontRenderingGUIElement* child = *ci;
		if (child->fontIsUpdated())
		{
			font_->clearText();
			need_to_update_font = true;
			break;
		}
	}

	if (need_to_update_font)
	{
		//std::cout << "Update font" << std::endl;
		for (std::vector<FontRenderingGUIElement*>::const_iterator ci = font_rendering_children_.begin(); ci != font_rendering_children_.end(); ++ci)
		{
			FontRenderingGUIElement* child = *ci;
			if (child->isVisible())
			{
				font_->appendString(child->getLocalTransformation(), child->getText(), child->getFontSize());
				child->setFontNeedsUpdating(false);
			}
		}
	}

	for (std::vector<Container*>::const_iterator ci = container_children_.begin(); ci != container_children_.end(); ++ci)
	{
		(*ci)->update(dt);
	}
	
	GUIElement::update(dt);
}

GUIElement* Container::processMousePressEvent(int mouse_x, int mouse_y)
{
	if (mouse_x >= global_transformation_[3][0] && mouse_x <= global_transformation_[3][0] + size_x_ &&
		mouse_y >= -global_transformation_[3][1] && mouse_y <= -global_transformation_[3][1] + size_y_)
	{
		// Check if any of its children are 'clicked'.
		for (std::vector<GUIElement*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
		{
			// Check if this element is clicked on by the mouse.
			if (mouse_x >= (*ci)->getGlobalX() && mouse_x <= (*ci)->getGlobalX() + (*ci)->getWidth() &&
			    mouse_y >= (*ci)->getGlobalY() && mouse_y <= (*ci)->getGlobalY() + (*ci)->getHeight())
			{
				GUIElement* pressed_element = (*ci)->processMousePressEvent(mouse_x, mouse_y);
				if (pressed_element != NULL)
				{
					return pressed_element;
				}
			}
		}
		
		for (std::vector<Container*>::const_iterator ci = container_children_.begin(); ci != container_children_.end(); ++ci)
		{
			// Check if this element is clicked on by the mouse.
			if (mouse_x >= (*ci)->getGlobalX() && mouse_x <= (*ci)->getGlobalX() + (*ci)->getWidth() &&
			    mouse_y >= (*ci)->getGlobalY() && mouse_y <= (*ci)->getGlobalY() + (*ci)->getHeight())
			{
				GUIElement* pressed_element = (*ci)->processMousePressEvent(mouse_x, mouse_y);
				if (pressed_element != NULL)
				{
					return pressed_element;
				}
			}
		}
	}
	
	return NULL;
}

void Container::processMouseReleasedEvent(int mouse_x, int mouse_y)
{
	for (std::vector<GUIElement*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
	{
		// Check if this element is clicked on by the mouse.
		(*ci)->processMouseReleasedEvent(mouse_x, mouse_y);
	}
	
	for (std::vector<Container*>::const_iterator ci = container_children_.begin(); ci != container_children_.end(); ++ci)
	{
		(*ci)->processMouseReleasedEvent(mouse_x, mouse_y);
	}
}

void Container::draw(const glm::mat4& perspective_matrix, int level) const
{
	if (!is_visible_) return;

	if (!render_outside_)
	{
		glEnable(GL_STENCIL_BUFFER);
		// Use the stensil buffer to make sure we don't render anything outside the container.
		glClear(GL_STENCIL_BUFFER_BIT);
		
		glStencilMask(0xFF);
		glStencilFunc(GL_ALWAYS, 1, 0xFF);
		glStencilOp(GL_KEEP, GL_KEEP, GL_INCR);
		
		glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
		glDepthMask(GL_FALSE);

		// Draw a rectangle on screen that covers the entire container.
		GUIShader& shader = GUIShader::getShader();
		
		if (parent_ != NULL)
		{
			shader.renderOutline(*parent_, parent_->getGlobalTransformation(), perspective_matrix);
		}
		shader.renderOutline(*this, global_transformation_, perspective_matrix);
	
		// Disable the mask and only draw where we previously drawed the rectangle.
		glStencilFunc(GL_EQUAL, (parent_ != NULL ? 2 : 1), 0xFF);
		glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
		glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
		glDepthMask(GL_TRUE);
	}
	else
	{
		glDisable(GL_STENCIL_BUFFER);
	}

	// Draw the container and its contents.	
	GUIShader& shader = GUIShader::getShader();
	shader.renderContainer(*this, global_transformation_, perspective_matrix);

	// Draw the font of all children (except those nested in containers).
	font_->draw(perspective_matrix, global_transformation_);
	
	for (std::vector<Container*>::const_iterator ci = container_children_.begin(); ci != container_children_.end(); ++ci)
	{
		if ((*ci)->isVisible())
		{
			(*ci)->draw(perspective_matrix, level + 1);
		}
	}
}

void Container::onResize(int width, int height)
{
	m_vertices_.clear();
	m_vertices_.push_back(glm::vec3(0, height, 0));
	m_vertices_.push_back(glm::vec3(size_x_, height, 0));
	m_vertices_.push_back(glm::vec3(0, height - size_y_, 0));
	m_vertices_.push_back(glm::vec3(size_x_, height - size_y_, 0));
	
	glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_); //Bind the vertex buffer
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * m_vertices_.size(), &m_vertices_[0], GL_STATIC_DRAW); //Send the data to OpenGL

	for (std::vector<GUIElement*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
	{
		(*ci)->onResize(width, height);
	}
	
	for (std::vector<Container*>::const_iterator ci = container_children_.begin(); ci != container_children_.end(); ++ci)
	{
		(*ci)->onResize(width, height);
	}
	font_->onResize(width, height);
	updateBuffers();
}
