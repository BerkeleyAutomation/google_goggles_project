#pragma once
#include <ros/ros.h>
#include <string>
#include <tf/transform_listener.h>

#include "tracker.h"

class TabletopTrackerROS : public TabletopTracker {
public:
	tf::TransformListener listener;
	ros::Publisher points_pub;
	ros::Publisher cloud_pub;
	ros::Publisher cluster_pub;
	ros::Subscriber cloud_sub;
	ros::Publisher cyl_pub;
	ros::Publisher table_height_pub;
	bool hasPendingMessage;
	ros::Time latest_stamp;

	TabletopTrackerROS(ros::NodeHandle n,TabletopTracker::Mode mode = TabletopTracker::TRACK);
	void updateTransform();


	void callback(const sensor_msgs::PointCloud2& msg);
	void publish();
	
	std::string latest_cloud_frame;
	
	std::string base_frame;
	std::string map_frame;
};
