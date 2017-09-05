#ifndef DEMO_PANDORA_ONTOLOGY_CHAIN_GOAL_H
#define DEMO_PANDORA_ONTOLOGY_CHAIN_GOAL_H
#include <vector>

#include "Goal.h"

class Chain;

class ChainGoal : public Goal
{
public:
	ChainGoal(Chain& chain);
	
	virtual ~ChainGoal();
	
	Chain& getChain() const { return *chain_; }
	
	bool isEnabled() const;
	
private:
	Chain* chain_;
};

#endif
