#include "PlanGraph.h"
#include "dpengine/scene/SceneLeafModel.h"

#include <sstream>
#include <math.h>

#include "dpengine/shapes/Line.h"
#include "dpengine/shaders/LineShader.h"
#include "dpengine/gui/Label.h"

#include "dpengine/gui/fonts/TexturedFont.h"
#include <dpengine/renderer/Window.h>

PlanGraph::PlanGraph(DreadedPE::Theme& theme, DreadedPE::Font& font, float x, float y, float size_x, float size_y, float pixels_per_seconds)
	: DreadedPE::Container(theme, font, x, y, size_x, size_y, true), font_(&font), total_time_elapsed_(0), pixels_per_seconds_(pixels_per_seconds), screen_width_(0), screen_height_(0)
{
	DreadedPE::Window* window = DreadedPE::Window::getActiveWindow();
	int width, height;
	window->getSize(width, height);
	screen_height_ = height;
	screen_width_ = width;
	
	line_ = std::make_shared<DreadedPE::Line>(false);
	
	// Time lines.
	for (float i = 0; i < 100; ++i)
	{
		std::stringstream ss;
		ss << i;
		DreadedPE::Label* label = new DreadedPE::Label(theme, 0, 0, ss.str(), 12);
		addElement(*label, 60.0f * pixels_per_seconds_ * i, -size_y + 30);
		
		minute_labels_.push_back(label);
	}
	
	setTextureUVMapping(theme.getInvisibleTexture());
	
	expected_energy_line_ = std::make_shared<DreadedPE::Line>(true);
	actual_energy_line_ = std::make_shared<DreadedPE::Line>(true);
	current_time_line_ = std::make_shared<DreadedPE::Line>(true);
}

void PlanGraph::draw(const glm::mat4& perspective_matrix, int level) const
{
	DreadedPE::Container::draw(perspective_matrix, level);
	
	/** TODO Fix this, currently cannot render without a SceneLeafModel! */
	DreadedPE::LineShader& ls = DreadedPE::LineShader::getShader();
	ls.initialise(glm::vec4(1.0f, 1.0f, 1.0f, 1.0f), glm::mat4(1.0f), global_transformation_, perspective_matrix, line_->getVertexBufferId());
	glBindBuffer(GL_ARRAY_BUFFER, line_->getVertexBufferId());
	glDrawArrays(GL_LINES, 0, line_->getVertices().size());
	//line_->render();
	
	ls.initialise(glm::vec4(1.0f, 0.0f, 1.0f, 1.0f), glm::mat4(1.0f), global_transformation_, perspective_matrix, expected_energy_line_->getVertexBufferId());
	glBindBuffer(GL_ARRAY_BUFFER, expected_energy_line_->getVertexBufferId());
	glDrawArrays(GL_LINE_STRIP, 0, expected_energy_line_->getVertices().size());
	//expected_energy_line_->render();
	
	ls.initialise(glm::vec4(0.0f, 1.0f, 0.0f, 1.0f), glm::mat4(1.0f), global_transformation_, perspective_matrix, actual_energy_line_->getVertexBufferId());
	glBindBuffer(GL_ARRAY_BUFFER, actual_energy_line_->getVertexBufferId());
	glDrawArrays(GL_LINE_STRIP, 0, actual_energy_line_->getVertices().size());
	//actual_energy_line_->render();
	
	ls.initialise(glm::vec4(1.0f, 0.0f, 0.0f, 1.0f), glm::mat4(1.0f), global_transformation_, perspective_matrix, current_time_line_->getVertexBufferId());
	glBindBuffer(GL_ARRAY_BUFFER, current_time_line_->getVertexBufferId());
	glDrawArrays(GL_LINE_STRIP, 0, current_time_line_->getVertices().size());
	//current_time_line_->render();
}

void PlanGraph::update(float dt)
{
	total_time_elapsed_ += dt;
	
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
	DreadedPE::Container::update(dt);
	markForUpdate();
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
	DreadedPE::Container::onResize(width, height);
}
