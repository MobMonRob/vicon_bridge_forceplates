#ifndef RVIZ_MARKERBUILDER_H
#define RVIZ_MARKERBUILDER_H

#include "ros/node_handle.h"
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include "vicon_bridge/Markers.h"

namespace Markerproperty
{
enum Type : uint8_t;
enum Action : uint8_t;
} // namespace Markerproperty

//http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html
enum Markerproperty::Type : uint8_t
{
	ARROW = visualization_msgs::Marker::ARROW,
	CUBE = visualization_msgs::Marker::CUBE,
	SPHERE = visualization_msgs::Marker::SPHERE,
	CYLINDER = visualization_msgs::Marker::CYLINDER,
	LINE_STRIP = visualization_msgs::Marker::LINE_LIST,
	LINE_LIST = visualization_msgs::Marker::LINE_LIST,
	CUBE_LIST = visualization_msgs::Marker::CUBE_LIST,
	SPHERE_LIST = visualization_msgs::Marker::SPHERE_LIST,
	POINTS = visualization_msgs::Marker::POINTS,
	TEXT_VIEW_FACING = visualization_msgs::Marker::TEXT_VIEW_FACING,
	MESH_RESOURCE = visualization_msgs::Marker::MESH_RESOURCE,
	TRIANGLE_LIST = visualization_msgs::Marker::TRIANGLE_LIST,
};

//http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html
enum Markerproperty::Action : uint8_t
{
	ADD = visualization_msgs::Marker::ADD,
	MODIFY = visualization_msgs::Marker::MODIFY,
	DELETE = visualization_msgs::Marker::DELETE,
	DELETEALL = visualization_msgs::Marker::DELETEALL,
};

class RvizMarkerBuilder
{
public:
	// Auslagern in Klasse zur Konvertierung
	visualization_msgs::Marker convertViconMarkersToRvizMarker(vicon_bridge::MarkersPtr viconMarkers) const;
	visualization_msgs::Marker convertViconPoseToRvizMarker(geometry_msgs::TransformStampedPtr pose_msg) const;
	visualization_msgs::Marker getTestMarker() const;

private:
	ros::NodeHandle paramServer = ros::NodeHandle("~");
	visualization_msgs::Marker buildStandardMarker() const;
	std_msgs::Header buildHeader(ros::Time stamp, std::string frame_id) const;
	geometry_msgs::Pose buildPose(geometry_msgs::Point position, geometry_msgs::Quaternion orientation) const;
	geometry_msgs::Point viconPositionToRvizPosition(const geometry_msgs::Point &viconPosition) const;
	geometry_msgs::Point buildPosition(double x, double y, double z) const;
	geometry_msgs::Quaternion buildOrientation(double w, double x, double y, double z) const;
	geometry_msgs::Vector3 buildScaleAllEqual(double xyz) const;
	geometry_msgs::Vector3 buildScale(double x, double y, double z) const;
	std_msgs::ColorRGBA buildColorRGB(float r, float g, float b) const;
};

#endif
