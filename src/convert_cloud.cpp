#include <ros/ros.h>
#include <iostream>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/io/pcd_io.h>

#include "tracker.h"
#include "table.h"
#include "utils_pcl.h"
#include "cloud_ops.h"
#include <boost/foreach.hpp>
#include "dist_math.h"
#include <ros/console.h>
#include <sstream>
#include "table_config.h"
#include <pcl/common/centroid.h>
#include <math.h>
#include <sstream>

using namespace std;
using namespace Eigen;

static string map_frame = "/odom_combined";
static string base_frame = "/base_link";

static ColorCloudPtr latest_cloud;
static tf::TransformListener* listener;
static ros::Subscriber cloud_sub;
static ros::Publisher cloud_pub;

static int callback_counter = 0;

Eigen::Affine3f toEigenTransform(const btTransform& transform) {
	btVector3 transBullet = transform.getOrigin();
	btQuaternion quatBullet = transform.getRotation();
	Eigen::Translation3f transEig;
	transEig = Eigen::Translation3f(transBullet.x(), transBullet.y(), transBullet.z());
	Eigen::Matrix3f rotEig = Eigen::Quaternionf(quatBullet.w(),quatBullet.x(),quatBullet.y(),quatBullet.z()).toRotationMatrix();
	Eigen::Affine3f out = transEig*rotEig;
	return out;
}

void callback(const sensor_msgs::PointCloud2& msg) {
	callback_counter++;
	ROS_INFO("Got msg");
	ColorCloudPtr cloud(new ColorCloud());
	pcl::fromROSMsg(msg, *cloud);
	
	tf::StampedTransform stamped_transform;
	listener->lookupTransform(map_frame, "/openni_rgb_optical_frame", ros::Time(0), stamped_transform);
	Eigen::Affine3f map_transform = toEigenTransform(stamped_transform.asBt());
	
	//ColorCloudPtr map_transformed_cloud = transformPointCloud1(downsampleCloud(latest_cloud, TableConfig::VOXEL_SIZE), map_transform);
	ColorCloudPtr map_transformed_cloud = transformPointCloud1(cloud, map_transform);
	
	std::stringstream ss;
	ss << "cloud_" << callback_counter << ".pcd";
	pcl::io::savePCDFileBinary(ss.str(),*map_transformed_cloud);
	
	//latest_stamp = msg.header.stamp;
	//hasPendingMessage = true;
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "convert_cloud");
	
	listener = new tf::TransformListener();
	
	Parser parser;
	parser.read(argc, argv);

	ros::NodeHandle nh;
	
	cloud_sub = nh.subscribe("/camera/depth_registered/points",1,&callback);
	cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("google_goggles/odom_cloud",100);
	
	ros::Duration(1).sleep();

	while (ros::ok()) {
		ros::spin();
		/*for (int i=0; i < 10; i++) { ros::spinOnce(); }
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
		}*/
		
	}
}