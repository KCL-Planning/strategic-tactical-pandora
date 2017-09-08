#include <GL/glew.h>

#include <glm/gtc/matrix_transform.hpp>

#include "ActionLabel.h"
#include "dpengine/shaders/LineShader.h"

#include "dpengine/gui/Label.h"
#include "dpengine/gui/events/ButtonPressedListener.h"
#include "dpengine/gui/themes/Theme.h"
#include "dpengine/gui/fonts/Font.h"
#include "dpengine/shapes/Shape.h"
#include <dpengine/renderer/Window.h>

ActionLabel::ActionLabel(const DreadedPE::Theme& theme, DreadedPE::Font& font, const glm::vec4& colour, float size_x, float size_y, const std::string& label, const planning_msgs::ActionDispatch& action)
	: DreadedPE::Container(theme, font, 0, 0, size_x, size_y, true), font_(&font), colour_(colour), label_(label), action_(action)
{
	DreadedPE::Window* window = DreadedPE::Window::getActiveWindow();
	int width, height;
	window->getSize(width, height);

	// Test data.
	m_vertices_.clear();
	m_vertices_.push_back(glm::vec3(0, height, 0));
	m_vertices_.push_back(glm::vec3(size_x_, height, 0));
	m_vertices_.push_back(glm::vec3(0, height - size_y_, 0));
	m_vertices_.push_back(glm::vec3(size_x_, height - size_y_, 0));
	
	shape_ = new DreadedPE::Shape();
	
	//font_->appendString(glm::translate(getLocalTransformation(), glm::vec3(0, -size_y / 2.0f, 0)), "TEST123", 16);
	font_->appendString(getLocalTransformation(), label, 12);
	
	start_time_ = action.dispatch_time;
	duration_ = action.duration;

/*
	glGenBuffers(1, &m_vertex_buffer_);
	
	glBindBuffer(GL_ARRAY_BUFFER, m_vertex_buffer_); //Bind the vertex buffer
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * m_vertices_.size(), &m_vertices_[0], GL_STATIC_DRAW); //Send the data to OpenGL
*/
	/*
	tool_hint_ = new Label(theme, 150, 75, label, 10);
	addElement(*tool_hint_, 10, -25);
	tool_hint_->setVisible(false);
	*/
}

void ActionLabel::draw(const glm::mat4& perspective_matrix, int level) const
{
	
	DreadedPE::Window* window = DreadedPE::Window::getActiveWindow();
	int width, height;
	window->getSize(width, height);
	
	DreadedPE::LineShader& shader = DreadedPE::LineShader::getShader();
	
	shader.initialise(colour_, glm::mat4(1.0f), global_transformation_, perspective_matrix, getCombinedVertexBufferId());
	
	//Bind the index array
	glBindBuffer(GL_ARRAY_BUFFER, getCombinedVertexBufferId());

	//Draw the triangles
	glDrawArrays(GL_TRIANGLE_STRIP, 0, m_vertices_.size());
	
	font_->draw(perspective_matrix, global_transformation_);
	
	Container::draw(perspective_matrix, level);
}

void ActionLabel::setDimensions(float width, float height)
{
	std::cout << "[ActionLabel::setDimensions] Set dimensions: " << width << ", " << height << std::endl;
	int screen_width, screen_height;
	DreadedPE::Window* window = DreadedPE::Window::getActiveWindow();
	window->getSize(screen_width, screen_height);
//	glfwGetWindowSize(&screen_width, &screen_height);
	
	size_x_ = width;
	size_y_ = height;
	
	m_vertices_.clear();
	m_vertices_.push_back(glm::vec3(0, screen_height, 0));
	m_vertices_.push_back(glm::vec3(size_x_, screen_height, 0));
	m_vertices_.push_back(glm::vec3(0, screen_height - size_y_, 0));
	m_vertices_.push_back(glm::vec3(size_x_, screen_height - size_y_, 0));
	
	updateBuffers();
}

DreadedPE::GUIElement* ActionLabel::processMousePressEvent(int x, int y)
{
	return this;
	// When the mouse is pressed, show a hint tool with all the relevant action information.
	tool_hint_->setPosition(x - getGlobalX(), -(y - getGlobalY()));
	tool_hint_->setVisible(true);
	updateBuffers();
	tool_hint_->setFontNeedsUpdating(true);
	return this;
}
	
void ActionLabel::processMouseReleasedEvent(int x, int y)
{
	return;
	// Hide the hint tool.
	tool_hint_->setVisible(false);
	updateBuffers();
	tool_hint_->setFontNeedsUpdating(true);
}

void ActionLabel::onResize(int width, int height)
{
	Container::onResize(width, height);
}
