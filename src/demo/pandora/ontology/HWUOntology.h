#ifndef DEMO_PANDORA_ONTOLOGY_HWU_ONTOLOGY_H
#define DEMO_PANDORA_ONTOLOGY_HWU_ONTOLOGY_H

#include <ros/ros.h>

#include "OntologyInterface.h"
#include "OctomapUpdateListener.h"
#include "../RRTUpdateListener.h"

/**
 * The interface to the HWU ontology. All the data is stored and retreived from this ontology.
 */
class HWUOntology : public OntologyInterface
{
public:
	HWUOntology(ros::NodeHandle& ros_node, OctomapBuilder& octomap_builder);
	
	/**
	 * Called everytime when the RRT is invalidated. This means that we should drop the filters.
	 */
	void rrtInvalidated() { OntologyInterface::rrtInvalidated(); }
	
	/**
	 * Called everytime when the RRT is updated.
	 */
	void rrtUpdated();
	
	/**
	 * Get all the inspection points stored in the ontology.
	 * @param inspection_points The vector where all found inspection points will be added to.
	 * @return True if the data could be retreived from the ontology, false otherwise.
	 */
	bool getInspectionPoints(std::vector<InspectionPoint>& inspection_points);
	
	/**
	 * Auxillery function that resolved the inspection point instance given the name of it.
	 * @param inspection_point_name The name of the inspection point as it is stored in the ontology (W#, where # is its number).
	 * @return The inspection point associated with the given name, NULL is no such inspection point exists (this should never happen!).
	 */
	InspectionPoint& getInspectionPoint(const std::string& inspection_point_name);

private:
	
	/**
	 * Add an inspection point to the ontology.
	 */
	void addInspectionPoint(InspectionPoint& inspection_point);
	
	ros::NodeHandle* ros_node_; /// The ros node.
	ros::ServiceClient ontology_client_; /// Service which exposes information in the ontology.
	ros::Subscriber complete_plan_listener_; /// Listen to messages that contain the complete current plan.
};

#endif
