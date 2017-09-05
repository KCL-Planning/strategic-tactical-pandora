/**
 * Singleton class that handles all GUI related aspects.
 */
#ifndef CORE_GUI_GUI_MANAGER_H
#define CORE_GUI_GUI_MANAGER_H

#include <vector>
#include <glm/glm.hpp>

class Frame;
class Container;
class Theme;
class Font;
class GUIElement;

class GUIManager
{
public:
	static GUIManager& getInstance();

	GUIManager();

	Frame& createFrame(Theme& theme, Font& font, float x, float y, float size_x, float size_y);
	void deleteFrame(Container& frame);
	void addFrame(Container& frame);

	void draw(const glm::mat4& perspective_matrix) const;
	void update(float dt);

	void onResize(int width, int height);

private:
	static GUIManager* gui_manager_;

	GUIElement* picked_gui_element_;  // The GUI element that is currently selected.

	std::vector<Container*> active_frames_;    // The frames are ordered from back -> front. The active Frame (if picked_gui_element is not NULL)
	                                       // is the last one in this list.
	std::vector<Container*> inactive_frames_;
};

#endif
