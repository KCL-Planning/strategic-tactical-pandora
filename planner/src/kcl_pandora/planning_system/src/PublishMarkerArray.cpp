#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

/**
 * listener to update the current AUV position.
 * This is used as the initial position of the vehicle during PRM construction;
 * it will be later replaced by a call to the ontology.
 */
namespace PandoraKCL
{
	/* output all inspection points */
	void publishInspectionMarkerArray(ros::NodeHandle nh)
	{
		visualization_msgs::MarkerArray marker_array;
		
		size_t counter = 0;
		for (std::map<std::string,Point3D>::iterator iit=PandoraKCL::inspectionPoints.begin(); iit!=PandoraKCL::inspectionPoints.end(); ++iit) {

			visualization_msgs::Marker marker;

			marker.header.frame_id = "world";
			marker.header.stamp = ros::Time();
			marker.ns = "mission_inspection_point";
			marker.id = counter; counter++;
			marker.type = visualization_msgs::Marker::SPHERE;
			marker.action = visualization_msgs::Marker::MODIFY;
			marker.pose.position.x = iit->second.N;
			marker.pose.position.y = iit->second.E;
			marker.pose.position.z = iit->second.D;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = 0.0;
			marker.pose.orientation.w = 1.0;
			marker.scale.x = 0.3;
			marker.scale.y = 0.3;
			marker.scale.z = 0.3;
			marker.color.a = 1.0;

			marker.color.r = 0.4;
			marker.color.g = 0.0;
			marker.color.b = 1.0;

			marker.text = iit->first;

			// HWU simulation (flipped)
			if(PandoraKCL::use_octomap) {
				marker.header.frame_id = "map";
				marker.pose.position.y = -inspectionPoints[iit->first].E;
				marker.pose.position.z = -inspectionPoints[iit->first].D;
			}

			if(inspectionPointsInspected[iit->first]) {
				marker.color.r = 0.0;
				marker.color.g = 1.0;
			}

			marker_array.markers.push_back(marker);
		}
		ipsPublisher.publish( marker_array );
	}

	/* output all waypoints */
	void publishWaypointMarkerArray(ros::NodeHandle nh)
	{
		visualization_msgs::MarkerArray marker_array;
		
		size_t counter = 0;
		for (std::map<std::string,Point3D>::iterator iit=PandoraKCL::waypoints.begin(); iit!=PandoraKCL::waypoints.end(); ++iit) {

			visualization_msgs::Marker marker;

			marker.header.frame_id = "world";
			marker.header.stamp = ros::Time();
			marker.ns = "mission_waypoint";
			marker.id = counter; counter++;
			marker.type = visualization_msgs::Marker::SPHERE;
			marker.action = visualization_msgs::Marker::MODIFY;
			marker.pose.position.x = iit->second.N;
			marker.pose.position.y = iit->second.E;
			marker.pose.position.z = iit->second.D;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = 0.0;
			marker.pose.orientation.w = 1.0;
			marker.scale.x = 0.2;
			marker.scale.y = 0.2;
			marker.scale.z = 0.2;
			marker.color.a = 1.0;

			marker.color.r = 0.3;
			marker.color.g = 1.0;
			marker.color.b = 0.3;

			marker.text = iit->first;

			// HWU simulation (flipped)
			if(PandoraKCL::use_octomap) {
				marker.header.frame_id = "map";
				marker.pose.position.y = -waypoints[iit->first].E;
				marker.pose.position.z = -waypoints[iit->first].D;
			}

			marker_array.markers.push_back(marker);
		}
		wpsPublisher.publish( marker_array );
	}
}
