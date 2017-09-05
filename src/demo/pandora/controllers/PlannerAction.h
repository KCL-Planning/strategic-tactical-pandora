#ifndef DEMO_PANDORA_CONTROLLERS_PLANNING_ACTION_H
#define DEMO_PANDORA_CONTROLLERS_PLANNING_ACTION_H

#include <planning_msgs/ActionDispatch.h>
#include <planning_msgs/ActionFeedback.h>

/**
 * Interface for planning actions.
 */
class PlannerAction
{
public:
	
	enum PLANNER_ACTION_STATUS { EXECUTING, SUCCEEDED, FAILED };
	
	/**
	 * A function that will return whether an action has been finished, is still ongoing or has failed.
	 * @return The status of the action.
	 */
	virtual PLANNER_ACTION_STATUS getStatus() {return SUCCEEDED;}
	
	/**
	 * Update the action.
	 */
	virtual void update(float dt) {}
	
	/**
	 * When an action is completed (successful or otherwise) the action can amend the feedback
	 * message with custom information.
	 * @param feedback The message to be amended.
	 * @param status The return status of this action's outcome.
	 */
	virtual void amendFeedback(planning_msgs::ActionFeedback& feedback, PLANNER_ACTION_STATUS status) { }
	
	void setActionMsg(const planning_msgs::ActionDispatch& action_msg) { action_msg_ = action_msg; }
	const planning_msgs::ActionDispatch& getActionMsg() const { return action_msg_; }
protected:
	planning_msgs::ActionDispatch action_msg_;
};

#endif
