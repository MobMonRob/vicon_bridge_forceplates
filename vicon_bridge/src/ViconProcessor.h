#ifndef VICON_PROCESSOR_H
#define VICON_PROCESSOR_H

#include "ros/publisher.h"
#include "geometry_msgs/TransformStamped.h"
#include "RvizMarkerBuilder.h"
#include "vicon_bridge/Markers.h"

class ViconProcessor
{
public:
	ViconProcessor(const ros::Publisher &markerPublisher);
	void pushMarkers(vicon_bridge::MarkersPtr markers_msg) const;
	void pushSegment(geometry_msgs::TransformStampedPtr pose_msg) const;

private:
	const ros::Publisher &markerPublisher;
	const RvizMarkerBuilder rvizMarkerBuilder;
};

#endif
