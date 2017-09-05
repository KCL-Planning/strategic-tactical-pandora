#ifndef CORE_GUI_SCROLLBAR_UNIT_H
#define CORE_GUI_SCROLLBAR_UNIT_H

#include "GUIElement.h"

class Container;
class Scrollbar;

/**
 * The scroll unit of a scrollbar, by moving the unit we move the 
 * container that this scrollbar is a part of.
 */
class ScrollbarUnit : public GUIElement
{
public:
	/**
	 * Create a scroll bar unit given the scrollbar it is a part of. We assume that the
	 * height or with (for vertical and not vertical, respectively) is the portion of the 
	 * container that is visible. Thus, in the vertical case, if the scrollbar is all the way
	 * down then the bottom of the container touches the bottom of the scrollbar and if the 
	 * scrollbar is all the way up then the upper part of the container touches the upper part 
	 * of the scrollbar. The rest of the values are interpolated between these two extremes.
	 * 
	 * @param theme The theme used for this scrollbar.
	 * @param x The x location of the scrollbar, relative to its parent.
	 * @param y The y location of the scrollbar, relative to its parent.
	 * @param size_x The width of the scrollbar.
	 * @param size_y The height of the scrollbar.
	 * @param scrollbar The scrollbar this unit is a part of.
	 */
	ScrollbarUnit(Theme& theme, float x, float y, float size_x, float size_y, Scrollbar& scrollbar);
	
	/**
	 * Every tick we check if the user has grapped the scrollbar unit, and if it has where it is 
	 * moved to. This also triggers the container the scrollbar is part of to be moved accordingly.
	 * @param dt The time that has passed since the last update.
	 */
	void update(float dt);
	
	GUIElement* processMousePressEvent(int x, int y);
	void processMouseReleasedEvent(int x, int y);
	
	/**
	 * Update the location of the scrollbar unit based on the location of the container that we are
	 * scrolling.
	 */
	void setContentSize(float size_x, float size_y);
	
	/**
	 * Draw the scrollbar unit.
	 */
	//void draw(const glm::mat4& perspective_matrix, int level) const;

	void onResize(int width, int height);
private:
	Scrollbar* scrollbar_; // The container that is affected by this scrollbar.
	float scroll_unit_size_; // The width or hight of the scroll unit, depending whether it's vertical or horizontar, respectively.
	
	bool is_grapped_; // Flag to determine whether or not the unit is 'grabbed' by the mouse.
	int grap_location_; // The position of where the mouse grabbed the unit.
	int org_location_;  // The location of the unit prior to being moved by a mouse.
};

#endif
