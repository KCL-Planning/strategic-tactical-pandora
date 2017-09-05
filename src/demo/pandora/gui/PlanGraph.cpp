#include "PlanGraph.h"

#include <GL/glfw.h>
#include <sstream>
#include <math.h>

#include "../../../shapes/Line.h"
#include "../../../core/shaders/LineShader.h"
#include "../../../core/gui/Label.h"

#include "../../../core/gui/fonts/TexturedFont.h"

PlanGraph::PlanGraph(Theme& theme, Font& font, float x, float y, float size_x, float size_y, float pixels_per_seconds)
	: Container(theme, font, x, y, size_x, size_y, true), font_(&font), total_time_elapsed_(0), pixels_per_seconds_(pixels_per_seconds), screen_width_(0), screen_height_(0)
{
	int width, height;
	glfwGetWindowSize(&width, &height);
	screen_height_ = height;
	screen_width_ = width;
	
	line_ = new Line(false);
	
	// Time lines.
	for (float i = 0; i < 100; ++i)
	{
		std::stringstream ss;
		ss << i;
		Label* label = new Label(theme, 0, 0, ss.str(), 12);
		addElement(*label, 60.0f * pixels_per_seconds_ * i, -size_y + 30);
		
		minute_labels_.push_back(label);
	}
	
	setTextureUVMapping(theme.getInvisibleTexture());
	
	expected_energy_line_ = new Line(true);
	actual_energy_line_ = new Line(true);
	current_time_line_ = new Line(true);
}

void PlanGraph::draw(const glm::mat4& perspective_matrix, int level) const
{
	Container::draw(perspective_matrix, level);
	
	LineShader& ls = LineShader::getShader();
	ls.initialise(glm::vec4(1.0f, 1.0f, 1.0f, 1.0f), glm::mat4(1.0f), global_transformation_, perspective_matrix, line_->getVertexBufferId());
	line_->render();
	
	ls.initialise(glm::vec4(1.0f, 0.0f, 1.0f, 1.0f), glm::mat4(1.0f), global_transformation_, perspective_matrix, expected_energy_line_->getVertexBufferId());
	expected_energy_line_->render();
	
	ls.initialise(glm::vec4(0.0f, 1.0f, 0.0f, 1.0f), glm::mat4(1.0f), global_transformation_, perspective_matrix, actual_energy_line_->getVertexBufferId());
	actual_energy_line_->render();
	
	ls.initialise(glm::vec4(1.0f, 0.0f, 0.0f, 1.0f), glm::mat4(1.0f), global_transformation_, perspective_matrix, current_time_line_->getVertexBufferId());
	current_time_line_->render();
}

void PlanGraph::update(float dt)
{
	total_time_elapsed_ += dt;
	Container::update(dt);
	
	std::vector<glm::vec3> line_points;
	// Line for each minute.
	for (float i = 0; i < getContentSizeX(); ++i)
	{
		line_points.push_back(glm::vec3(60.0f * pixels_per_seconds_ * i, screen_height_, 0.0f));
		line_points.push_back(glm::vec3(60.0f * pixels_per_seconds_ * i, screen_height_ - size_y_, 0.0f));
	}
	//line_points.push_back(glm::vec3(100, screen_height_, 0));
	//line_points.push_back(glm::vec3(100, screen_height_ - 50, 0));
	
	line_->setVertexBuffer(line_points);

	// Draw the time line.
	line_points.clear();
	
	//line_points.push_back(glm::vec3(total_time_elapsed_ * pixels_per_seconds_, 1000, 0.0f));
	//line_points.push_back(glm::vec3(total_time_elapsed_ * pixels_per_seconds_, 800, 0.0f));
	
	line_points.push_back(glm::vec3(total_time_elapsed_ * pixels_per_seconds_, screen_height_, 0.0f));
	line_points.push_back(glm::vec3(total_time_elapsed_ * pixels_per_seconds_, screen_height_ - size_y_, 0.0f));
	
	current_time_line_->setVertexBuffer(line_points);
}

void PlanGraph::resetTimeLine()
{
	total_time_elapsed_ = 0;
}

void PlanGraph::onResize(float width, float height)
{
	screen_width_ = width;
	screen_height_ = height;
	font_->onResize(width, height);
	Container::onResize(width, height);
}
