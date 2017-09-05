#ifndef DEMO_PANDORA_ONTOLOGY_OCTOMAP_UPDATE_LISTENER_H
#define DEMO_PANDORA_ONTOLOGY_OCTOMAP_UPDATE_LISTENER_H

class OctomapUpdateListener
{
public:
	virtual void octomapUpdated() = 0;
};

#endif
