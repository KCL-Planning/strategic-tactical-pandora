#include "ChainGoal.h"
#include "../structures/Chain.h"

ChainGoal::ChainGoal(Chain& chain)
	: Goal(chain), chain_(&chain)
{
	
}

ChainGoal::~ChainGoal()
{
	
}

bool ChainGoal::isEnabled() const
{
	return chain_->hasBeenObserved();
}
