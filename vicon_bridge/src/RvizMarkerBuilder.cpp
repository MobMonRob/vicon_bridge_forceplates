#include "RvizMarkerBuilder.h"

#include <cstdint>
#include "ros/node_handle.h"
#include "ros/macros.h"

using namespace visualization_msgs;
using namespace geometry_msgs;
using namespace std_msgs;
using namespace Markerproperty;

visualization_msgs::Marker RvizMarkerBuilder::convertViconMarkersToRvizMarker(vicon_bridge::MarkersPtr viconMarkers) const
{
	visualization_msgs::Marker marker;
	marker = buildStandardMarker();

	marker.type = Type::SPHERE_LIST;
	marker.header = viconMarkers->header;
	marker.header.frame_id = "/world";

	double scale;
	paramServer.param("scale", scale, 0.02); //name, value, default
	marker.scale = buildScaleAllEqual(scale);

	marker.color = buildColorRGB(1.0, 0.0, 1.0);

	for (auto currentViconMarker : viconMarkers->markers)
	{
		marker.points.push_back(viconPositionToRvizPosition(currentViconMarker.translation));
	}

	return marker;
}

visualization_msgs::Marker RvizMarkerBuilder::convertViconPoseToRvizMarker(geometry_msgs::TransformStampedPtr pose_msg) const
{
	visualization_msgs::Marker marker = buildStandardMarker();

	marker.type = Type::SPHERE;
	marker.header = pose_msg->header;
	marker.ns = pose_msg->child_frame_id;

	const geometry_msgs::Vector3 &poseTranslation = pose_msg->transform.translation;
	marker.pose.position = buildPosition(poseTranslation.x, poseTranslation.y, poseTranslation.z);
	marker.pose.orientation = pose_msg->transform.rotation;

	return marker;
}

visualization_msgs::Marker RvizMarkerBuilder::getTestMarker() const
{
	return buildStandardMarker();
}

visualization_msgs::Marker RvizMarkerBuilder::buildStandardMarker() const
{
	//http://wiki.ros.org/rviz/DisplayTypes/Marker
	Marker marker; //primitve 3D shape

	marker.header = buildHeader(ros::Time::now(), "/world");				//Header for time/frame information
	marker.ns = "namespace";												//namespace ->identifier
	marker.id = 0;															//object id
	marker.type = Type::SPHERE;												//object type
	marker.action = Action::ADD;											//object action
	marker.pose = buildPose(Point(), buildOrientation(1.0, 0.0, 0.0, 0.0)); //object pose
	marker.scale = buildScale(1.0, 1.0, 1.0);								//object scale
	marker.color = buildColorRGB(0.0, 1.0, 1.0);							//object color
	marker.lifetime = ros::Duration(0, 0);									//How long the object should last before being automatically deleted. 0 means forever ->(sec, nsec)
	marker.frame_locked = false;											//If this marker should be frame-locked, i.e. retransformed into its frame every timestep

	return marker;
}

std_msgs::Header RvizMarkerBuilder::buildHeader(ros::Time stamp, std::string frame_id) const
{
	Header header;

	header.stamp = stamp;
	header.frame_id = frame_id;

	return header;
}

geometry_msgs::Pose RvizMarkerBuilder::buildPose(Point position, Quaternion orientation) const
{
	Pose pose;

	pose.position = position;
	pose.orientation = orientation;

	return pose;
}

geometry_msgs::Point RvizMarkerBuilder::viconPositionToRvizPosition(const geometry_msgs::Point &viconPosition) const
{
	static const uint32_t viconMillimetreToRvizMetreFactor = 1000;
	static const uint32_t &f = viconMillimetreToRvizMetreFactor;

	return buildPosition(viconPosition.x / f, viconPosition.y / f, viconPosition.z / f);
}

geometry_msgs::Point RvizMarkerBuilder::buildPosition(double x, double y, double z) const
{
	Point position;

	position.x = x;
	position.y = y;
	position.z = z;

	return position;
}

geometry_msgs::Quaternion RvizMarkerBuilder::buildOrientation(double w, double x, double y, double z) const
{
	Quaternion orientation;

	orientation.w = w;
	orientation.x = x;
	orientation.y = y;
	orientation.z = z;

	return orientation;
}

geometry_msgs::Vector3 RvizMarkerBuilder::buildScaleAllEqual(double xyz) const
{
	return buildScale(xyz, xyz, xyz);
}

geometry_msgs::Vector3 RvizMarkerBuilder::buildScale(double x, double y, double z) const
{
	Vector3 scale;

	scale.x = x;
	scale.y = y;
	scale.z = z;

	return scale;
}

std_msgs::ColorRGBA RvizMarkerBuilder::buildColorRGB(float r, float g, float b) const
{
	ColorRGBA color;

	color.a = 1.0;
	color.r = r;
	color.g = g;
	color.b = b;

	return color;
}
