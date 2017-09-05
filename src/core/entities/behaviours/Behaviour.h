#ifndef CORE_ENTITIES_BEHAVIOURS_BEHAVIOUR_H
#define CORE_ENTITIES_BEHAVIOURS_BEHAVIOUR_H

class Entity;
class SceneNode;

/**
 * When an entity is interacted with or whenever time passes the behaviour will state how an entity
 * changes its state over time. This can be through movement or through changing the colour / textures / etc.
 *
 * This is the common interface for all behaviours.
 */
class Behaviour
{
public:
	Behaviour(SceneNode& entity);

	virtual void prepare(float dt) = 0;
	virtual void activate(const Entity& activator) = 0;
protected:
	SceneNode* entity_;
};

#endif
