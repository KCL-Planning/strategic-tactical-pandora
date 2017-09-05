#ifndef DEMO_PANDORA_ONTOLOGY_FILTER_PILLAR_FILTER_H
#define DEMO_PANDORA_ONTOLOGY_FILTER_PILLAR_FILTER_H

#include "Filter.h"

class Pillar;

/**
 * Filter that gets triggered when a pillar becomes observed for the first time.
 */
class PillarFilter : public Filter
{
public:
	PillarFilter(const knowledge_msgs::Filter& filter_msg, Pillar& pillar);
	
	bool checkFilter();
	
	knowledge_msgs::Notification prepareNotification();
	
private:
	Pillar* pillar_;
};

#endif
