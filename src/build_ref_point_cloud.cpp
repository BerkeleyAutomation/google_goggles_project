#include "tracker_ros.h"
#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include "table_config.h"
using namespace std;

static ColorCloudPtr latest_cloud;
int main(int argc, char* argv[]) {
	ros::init(argc, argv, "build_ref_point_cloud");
	Parser parser;
	parser.addGroup(TableConfig());
	parser.read(argc, argv);

	ros::NodeHandle nh;
	TabletopTrackerROS tracker(nh);
	ROS_INFO("Tracker ready");

	ros::Duration(1).sleep();

	while (ros::ok()) {
		for (int i=0; i < 10; i++) { ros::spinOnce(); }
		if (tracker.hasPendingMessage) {
			ROS_INFO("tracker has pending message. updating");
			try {
				tracker.updateAll();
				tracker.publish();
				tracker.hasPendingMessage = false;
			} catch (std::runtime_error err) {
				ROS_ERROR_STREAM("error while updating tracker: " <<err.what());
				ros::Duration(.1).sleep();
			}
		} else {
			ros::Duration(0.01).sleep();
		}
	}
	
	stringstream ss;
	if (argc >= 2) {
		ss << argv[1];
	} else {
		ss << "points";
	}
	ss << "_" << (int)ros::Time::now().toSec() << ".pcd";
	tracker.write(ss.str());
}