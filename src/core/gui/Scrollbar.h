#ifndef CORE_GUI_SCROLLBAR_H
#define CORE_GUI_SCROLLBAR_H

#include "Container.h"
#include "events/ResizedEventListener.h"

class ScrollbarUnit;

/**
 * Displays a bar with a scroll unit on it, by moving the unit we move the 
 * container that this scrollbar is a part of.
 */
class Scrollbar : public Container, public ResizedEventListener
{
public:
	/**
	 * Create a scroll bar given the container the scrollbar can affect. We assume that the
	 * height or with (for vertical and not vertical, respectively) is the portion of the 
	 * container that is visible. Thus, in the vertical case, if the scrollbar is all the way
	 * down then the bottom of the container touches the bottom of the scrollbar and if the 
	 * scrollbar is all the way up then the upper part of the container touches the upper part 
	 * of the scrollbar. The rest of the values are interpolated between these two extremes.
	 * 
	 * @param theme The theme used for this scrollbar.
	 * @param font The font to be used.
	 * @param x The x location of the scrollbar, relative to its parent.
	 * @param y The y location of the scrollbar, relative to its parent.
	 * @param size_x The width of the scrollbar.
	 * @param size_y The height of the scrollbar.
	 * @param scrollee The container that this scrollbar affects.
	 * @param vertical If true then the scrollbar will be vertical, if false then the scrollbar 
	 * will be horizontal.
	 */
	Scrollbar(Theme& theme, Font& font, float x, float y, float size_x, float size_y, Container& scrollee, bool vertical);
	
	/**
	 * Draw the scrollbar.
	 */
	//void draw(const glm::mat4& perspective_matrix, int level) const;
	
	/**
	 * Check if the scrollbar is vertical or not.
	 * @return True if the scrollbar is vertical, false if it is horizontal.
	 */
	bool isVertical() const { return is_vertical_; }
	
	/**
	 * Return the container that is being scrolled.
	 */
	Container& getScrollee() const { return *scrollee_; }
	
	/**
	 * Callback function for when the scrollee has resized.
	 * @param container The container that has been resized.
	 */
	void resizedEvent(const Container& container);

//	void onResize(int width, int height);
private:
	Container* scrollee_; // The container that is affected by this scrollbar.
	bool is_vertical_;
	
	ScrollbarUnit* scrollbar_unit_; // The bit of the scrollbar that moves the container.
};

#endif
