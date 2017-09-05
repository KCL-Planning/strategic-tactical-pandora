#ifndef DEMO_PANDORA_ONTOLOGY_FILTER_FILTER_H
#define DEMO_PANDORA_ONTOLOGY_FILTER_FILTER_H

#include <knowledge_msgs/Filter.h>
#include <knowledge_msgs/Notification.h>

#include <ostream>
#include <glm/glm.hpp>

/**
 * Interface for any filter that can be added to the ontology, everytime the environment changes
 * we check if any of the filters are violated and act accordinly.
 */
class Filter
{
public:
	Filter(const knowledge_msgs::Filter& filter_msg);
	
	virtual knowledge_msgs::Notification prepareNotification();
	
	/**
	 * This function is checked everytime the ontology is updated.
	 * @return False if the filter is violated, true otherwise.
	 */
	virtual bool checkFilter() = 0;
	
	friend std::ostream& operator<<(std::ostream& o, const Filter& filter);
	
	/**
	 * Return all the points that form a line (or point if it's only one) that should be
	 * drawn on the screen to visualise this filter.
	 */
	const std::vector<glm::vec3>& getPoints() const { return points_; }
	
protected:
	// DEBUG: Just for visualisation.
	std::vector<glm::vec3> points_;
	bool has_been_triggered_; /// DEBUG: True if this filter returned false to the 'check filter' at least once.
	
	knowledge_msgs::Filter filter_msg_; // The msg that was used to construct this filter.
};

std::ostream& operator<<(std::ostream& o, const Filter& filter);

#endif
