#include "ros/ros.h"
#include "std_msgs/String.h"

#include "RvizMarkerBuilder.h"
#include "vicon_bridge.h"
#include "ViconProcessor.h"
#include "visualization_msgs/Marker.h"

#include <sstream>

void testVicon(ros::NodeHandle nodeHandle);
void testMarkerBuilder(ros::NodeHandle nodeHandle);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ViconRvizNode");
	ros::NodeHandle nodeHandle;

	//testMarkerBuilder(nodeHandle);
	testVicon(nodeHandle);

	return 0;
}

void testVicon(ros::NodeHandle nodeHandle)
{
	ros::Publisher markerPublisher = nodeHandle.advertise<visualization_msgs::Marker>("vicon_rviz_marker", 1000);

	ViconProcessor viconProcessor(markerPublisher);


///////////////////
	ros::NodeHandle paramServer("~");
	double scale;
	paramServer.param("scale", scale, 1.0); //name, value, default
	ROS_INFO("Scale: %f", scale);
///////////////////


	ros::AsyncSpinner aspin(1);
	aspin.start();
	ViconReceiver vr(viconProcessor);
	aspin.stop();
}

void testMarkerBuilder(ros::NodeHandle nodeHandle)
{
	ros::Publisher markerPublisher = nodeHandle.advertise<visualization_msgs::Marker>("vicon_rviz_marker", 1000);

	ros::Rate loop_rate(10);

	int count = 0;
	while (ros::ok())
	{
		std_msgs::String msg;

		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();

		//ROS_INFO("%s", msg.data.c_str()); // Gibt Text in der Konsole aus.

		RvizMarkerBuilder markerBuilder;
		markerPublisher.publish(markerBuilder.getTestMarker());

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}
}
