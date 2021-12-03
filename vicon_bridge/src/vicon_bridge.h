#ifndef VICON_BRIDGE_H
#define VICON_BRIDGE_H

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <vicon_bridge/viconGrabPose.h>

#include <map>
#include <vicon_bridge/viconCalibrateSegment.h>

#include "ViconProcessor.h"
#include "vicon_bridge/Markers.h"
#include "vicon_bridge/ForcePlates.h"

using std::string;

class SegmentPublisher
{
public:
	ros::Publisher pub;
	bool is_ready;
	tf::Transform calibration_pose;
	bool calibrated;
	SegmentPublisher() : is_ready(false), calibration_pose(tf::Pose::getIdentity()),
						 calibrated(false){};
};

typedef std::map<string, SegmentPublisher> SegmentMap;

class ViconReceiver
{
public:
	ViconReceiver(std::optional<ViconProcessor> viconProcessor = std::nullopt);
	~ViconReceiver();
	void startGrabbing();
	void stopGrabbing();

private:
	const std::optional<ViconProcessor> viconProcessor;

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv;
	// Diagnostic Updater
	diagnostic_updater::Updater diag_updater;
	double min_freq_;
	double max_freq_;
	diagnostic_updater::FrequencyStatus freq_status_;
	// Parameters:
	string stream_mode_;
	string host_name_;
	string tf_ref_frame_id_;
	string tracked_frame_suffix_;
	// Publisher
	ros::Publisher marker_pub_;
	ros::Publisher forcePlatePublisher;
	// TF Broadcaster
	tf::TransformBroadcaster tf_broadcaster_;
	//geometry_msgs::PoseStamped vicon_pose;
	tf::Transform flyer_transform;
	ros::Time now_time;
	// TODO: Make the following configurable:
	ros::ServiceServer m_grab_vicon_pose_service_server;
	ros::ServiceServer calibrate_segment_server_;
	//  ViconDataStreamSDK::CPP::Client MyClient;
	unsigned int lastFrameNumber;
	unsigned int frameCount;
	unsigned int droppedFrameCount;
	ros::Time time_datum;
	unsigned int frame_datum;
	unsigned int n_markers;
	unsigned int n_unlabeled_markers;
	bool segment_data_enabled;
	bool marker_data_enabled;
	bool unlabeled_marker_data_enabled;
	

	bool broadcast_tf_, publish_tf_, publish_markers_;

	bool grab_frames_;
	bool device_data_enabled;
	boost::thread grab_frames_thread_;
	SegmentMap segment_publishers_;
	boost::mutex segments_mutex_;
	std::vector<std::string> time_log_;

private:
	void diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
	bool init_vicon();
	void createSegmentThread(const string subject_name, const string segment_name);
	void createSegment(const string subject_name, const string segment_name);
	void grabThread();
	bool shutdown_vicon();
	bool process_frame();
	void process_subjects(const ros::Time &frame_time);
	vicon_bridge::MarkersPtr process_markers(const ros::Time &frame_time, unsigned int vicon_frame_num);
	vicon_bridge::ForcePlatePtr process_forcePlateData();
	//void process_forcePlateData();
	void forcePlateSpinner();
	bool grabPoseCallback(vicon_bridge::viconGrabPose::Request &req, vicon_bridge::viconGrabPose::Response &resp);
	bool calibrateSegmentCallback(vicon_bridge::viconCalibrateSegment::Request &req, vicon_bridge::viconCalibrateSegment::Response &resp);
	
};

#endif
