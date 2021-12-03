#include "ViconProcessor.h"

#include "visualization_msgs/MarkerArray.h"
#include "ros/ros.h"

ViconProcessor::ViconProcessor(const ros::Publisher &markerPublisher) : markerPublisher(markerPublisher), rvizMarkerBuilder() {}

void ViconProcessor::pushMarkers(vicon_bridge::MarkersPtr viconMarkers) const
{
	visualization_msgs::Marker rvizMarker = rvizMarkerBuilder.convertViconMarkersToRvizMarker(viconMarkers);
	markerPublisher.publish(rvizMarker);
}

void ViconProcessor::pushSegment(geometry_msgs::TransformStampedPtr pose_msg) const
{
	//visualization_msgs::Marker rvizMarker = rvizMarkerBuilder.convertViconPoseToRvizMarker(pose_msg);
	//markerPublisher.publish(rvizMarker);
}
