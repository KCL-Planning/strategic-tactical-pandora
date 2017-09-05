#ifndef DEMO_PANDORA_ONTOLOGY_ONTOLOGY_INTERFACE_H
#define DEMO_PANDORA_ONTOLOGY_ONTOLOGY_INTERFACE_H

#include <ros/ros.h>

#include <planning_msgs/GetSubTypes.h>
#include <planning_msgs/GetInstancesOfType.h>
#include <planning_msgs/GetAttributesOfType.h>
#include <planning_msgs/GetAttributesOfInstance.h>
#include <planning_msgs/CompletePlan.h>

#include <knowledge_msgs/Filter.h>
#include <knowledge_msgs/RoadmapRefresh.h>
#include <knowledge_msgs/KnowledgeInterface.h>

#include <vector>
#include <map>
#include <knowledge_msgs/KnowledgeInterface.h>

#include "InspectionPoint.h"
#include "OctomapUpdateListener.h"
#include "ValveGoal.h"
#include "../RRTUpdateListener.h"

class AUV;
class RRT;
class Filter;
class OctomapBuilder;
class Valve;
class ValvePanel;
class MissionSite;
class Goal;
class Waypoint;
class Mission;

class OntologyInterface : public OctomapUpdateListener, public RRTUpdateListener
{
public:
	OntologyInterface(ros::NodeHandle& ros_node, OctomapBuilder& octomap_builder);
	
	/**
	 * Finalise the initialisation of the Ontology.
	 * @param rrt The RRT creator, this contains all the waypoints the AUV can use.
	 */
	void initialise(RRT& rrt);
	
	/**
	 * Function that is called every time when the octomap is updated.
	 */
	void octomapUpdated();

	/**
	 * Return the list of all installed filters.
	 * @return The list of installed filters.
	 */
	const std::vector<Filter*>& getFilters() const { return filters_; };
	
	/**
	 * Get all the inspection points stored in the ontology.
	 * @param inspection_points The vector where all found inspection points will be added to.
	 * @return True if the data could be retreived from the ontology, false otherwise.
	 */
	virtual bool getInspectionPoints(std::vector<InspectionPoint*>& inspection_points) = 0;
	
	/**
	 * Called everytime when the RRT is invalidated. This means that we should drop the filters.
	 */
	virtual void rrtInvalidated() { filters_.clear(); }
	
	/**
	 * Called everytime when the RRT is updated.
	 */
	virtual void rrtUpdated();

	/**
	 * Auxillery function that resolved the waypoint instance given the name of it.
	 * @param waypoint_name The name of the waypoint as it is stored in the ontology (W#, where # is its number).
	 * @return The waypoint associated with the given name, NULL is no such waypoint exists (this should never happen!).
	 */
	virtual Waypoint* getWaypoint(const std::string& waypoint_name) const;
	
	/**
	 * Auxillery function that resolved the inspection point instance given the name of it.
	 * @param inspection_point_name The name of the inspection point as it is stored in the ontology (W#, where # is its number).
	 * @return The inspection point associated with the given name, NULL is no such inspection point exists (this should never happen!).
	 */
	virtual InspectionPoint& getInspectionPoint(const std::string& inspection_point_name) = 0;
	
	/**
	 * This function is called when an inspection point has been observed. For the HWU ontology this does nothing because this
	 * is handled elsewhere for them, but for our ontology we state that the pilar this inspection point is part of is now 
	 * observed.
	 * @param inspection_point The location of the inspection point; TODO pass the actuall name you monster...
	 */
	virtual void observedInspectionPoint(const glm::vec3& inspection_point) { }
	
	/**
	 * Add a mission site to the ontology.
	 * @param mission_site The mission site to be added.
	 */
	void addMissionSite(MissionSite& mission_site);
	
	/**
	 * Remove a mission site from the ontology.
	 * @param mission_site The mission site to be removed.
	 */
	void removeMissionSite(const MissionSite& mission_site);
	
	/**
	 * Get all the mission sites the ontology knows about.
	 * @return All the mission sites the ontology knows about.
	 */
	const std::vector<MissionSite*>& getMissionSites() const { return mission_sites_; }
	
	/**
	 * Get the mission site with id @ref{mission_site_id}.
	 * @param mission_site_id The ID of the mission site we require.
	 * @return The mission site with the given ID.
	 */
	virtual MissionSite* getMissionSite(const std::string& mission_site_id) const;
	
	/**
	 * Add an AUV to the ontology.
	 * @param auv The AUV to be added.
	 */
	void addAUV(const AUV& auv);
	
	/**
	 * Remove a structure from the ontology.
	 */
	//void removeStructure(const Structure& structure);
	
protected:
	
	OctomapBuilder* octomap_; /// The octomap used to check connectivity and visibility of waypoints and inspection points.
	RRT* rrt_; /// The module that constructs the RRT.
	
	std::vector<MissionSite*> mission_sites_;                       /// The set of mission sites.
	//std::map<std::string, Goal*> goal_mappings_;                    /// The set of goals.
	std::map<std::string, Mission*> missions_;                      /// Mapping from ids to the missions.
	std::vector<const AUV*> auvs_;                                  /// The set of AUVs.
	
private:
	
	/**
	 * This is the callback function whenever a new filter is requested. The message contains all the relevant information for the
	 * filter, check @ref{knowledge_msgs::Filter} for more information. If one of the filters is violated a notificaction is send out.
	 */
	void updateFilter(const knowledge_msgs::Filter::ConstPtr& msg);
	
	std::vector<Filter*> filters_; /// The set of filters that is checked everytime the ontology is updated.
	clock_t last_time_filters_checked_; /// The last time the filters were checked.
	
	ros::Subscriber filter_server_;   /// Service in which filters can be placed.
	ros::Publisher notification_server_; /// Topic on which notifications of when filters are triggered are published.
};

#endif
