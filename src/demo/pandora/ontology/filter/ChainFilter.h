#ifndef DEMO_PANDORA_ONTOLOGY_FILTER_CHAIN_FILTER_H
#define DEMO_PANDORA_ONTOLOGY_FILTER_CHAIN_FILTER_H

#include "Filter.h"

class Chain;

/**
 * Filter that gets triggered when a pillar becomes observed for the first time.
 */
class ChainFilter : public Filter
{
public:
	ChainFilter(const knowledge_msgs::Filter& filter_msg, Chain& chain);
	
	bool checkFilter();
	
	knowledge_msgs::Notification prepareNotification();
	
private:
	Chain* chain_;
};

#endif
