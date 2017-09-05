#ifndef DEMO_PANDORA_CONTROLLERS_ACTION_EXECUTION_LISTENER_H
#define DEMO_PANDORA_CONTROLLERS_ACTION_EXECUTION_LISTENER_H

class PlannerAction;

/**
 * Interface that has callback functions for the execution framework.
 */
class ActionExecutionListener
{
public:
	virtual void actionExecutionStarted(const PlannerAction& action) = 0;
	virtual void actionExecutionFailed(const PlannerAction& action) = 0;
	virtual void actionExecutionSucceeded(const PlannerAction& action) = 0;
};

#endif
