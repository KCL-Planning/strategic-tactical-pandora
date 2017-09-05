#ifndef CORE_ENTITIES_BEHAVIOURS_HOVER_BEHAVIOUR_H
#define CORE_ENTITIES_BEHAVIOURS_HOVER_BEHAVIOUR_H

#include "Behaviour.h"

class Entity;

class HoverBehaviour : public Behaviour
{
public:
	HoverBehaviour(SceneNode& entity);
	virtual void prepare(float dt);
	virtual void activate(const Entity& activator);

private:
	float time_elapsed_;
};

#endif



	