#include "vicon_bridge.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vicon_bridgeNode");
	//  ViconReceiver vr;
	//  ros::spin();

	ros::AsyncSpinner aspin(1);
	aspin.start();
	ViconReceiver vr;
	aspin.stop();

	return 0;
}
