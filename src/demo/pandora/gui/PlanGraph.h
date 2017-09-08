#ifndef DEMO_PANDORA_GUI_PLAN_GRAPH_H
#define DEMO_PANDORA_GUI_PLAN_GRAPH_H

#include <memory>
#include <vector>

#include "dpengine/gui/Container.h"

namespace DreadedPE
{
	class Line;
	class Theme;
	class Font;
	class Label;
};

class PlanGraph : public DreadedPE::Container
{
public:
	PlanGraph(DreadedPE::Theme& theme, DreadedPE::Font& font, float x, float y, float size_x, float size_y, float pixels_per_seconds);
	void draw(const glm::mat4& perspective_matrix, int level) const;
	void update(float dt);
	
	void setPixelsPerSecond(float pixels_per_second) { pixels_per_seconds_ = pixels_per_second; }
	
	void onResize(float width, float height);
	
	float getTime() const { return total_time_elapsed_; }
	
	void resetTimeLine();
private:
	std::shared_ptr<DreadedPE::Line> line_;
	std::shared_ptr<DreadedPE::Line> expected_energy_line_;
	std::shared_ptr<DreadedPE::Line> actual_energy_line_;
	std::shared_ptr<DreadedPE::Line> current_time_line_;
//	FreeTypeFont* font_;
	DreadedPE::Font* font_;
	
	std::vector<DreadedPE::Label*> minute_labels_;
	
	float total_time_elapsed_;
	float pixels_per_seconds_;
	
	int screen_width_, screen_height_;
};

#endif
