#ifndef DEMO_PANDORA_RRT_UPDATE_LISTENER_H
#define DEMO_PANDORA_RRT_UPDATE_LISTENER_H

class RRTUpdateListener
{
public:
	/**
	 * A function call that is called when the current RRT is no longer valid.
	 */
	virtual void rrtInvalidated() = 0;
	
	/**
	 * A function call that is called when the new RRT is ready.
	 */
	virtual void rrtUpdated() = 0;
private:
	
};

#endif
