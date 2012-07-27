#include "tracker_ros.h"
#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include "table_config.h"

#include "cloud_ops.h"
#include "table.h"

#include <pcl/io/pcd_io.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;



//TabletopTrackerROS* tracker;
static ColorCloudPtr latest_cloud(new ColorCloud());
static string latest_cloud_frame;

ros::Publisher pose_pub;
ros::Publisher grasp_pub;


void callback(const sensor_msgs::PointCloud2& msg) {
	ColorCloudPtr cloud(new ColorCloud());
	ColorCloudPtr table_hull;
	float xmin, xmax, ymin, ymax;
	float table_height;
	std::vector<pcl::Vertices> table_polygons;


	pcl::fromROSMsg(msg, *cloud);
	table_height = getTableHeight(cloud);

	ColorCloudPtr in_table = getTablePoints(cloud, table_height);
	in_table = getBiggestCluster(in_table, TableConfig::TABLE_CLUSTERING_TOLERANCE);
	table_hull = findConvexHull(in_table, table_polygons);
	getTableBounds(in_table, xmin, xmax, ymin, ymax);

	ColorCloudPtr on_table = getPointsOnTableHull(cloud, table_hull, table_polygons, table_height+TableConfig::ABOVE_TABLE_CUTOFF);

	//icp
	pcl::IterativeClosestPoint<ColorPoint, ColorPoint> icp;
	icp.setInputCloud(cloud);
	icp.setInputTarget(latest_cloud);
	ColorCloudPtr final(new ColorCloud());
	std::cout << "aligning..." << std::endl;
	icp.align(*final);
	
	double fitness = icp.getFitnessScore();
	std::cout << "has converged:" << icp.hasConverged() << " score: " << fitness << std::endl;
	
	Eigen::Affine3f tf;
	tf.matrix() = icp.getFinalTransformation();

	Eigen::Affine3d tfd;
	for (int i=0;i<tf.matrix().rows();i++) {
		for (int j=0;j<tf.matrix().cols();j++) {
			tfd.matrix()(i,j) = tf.matrix()(i,j);
		}
	}

	tf::Transform ros_tf;
	tf::TransformEigenToTF(tfd,ros_tf);

	geometry_msgs::PoseStamped pose;
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = latest_cloud_frame;

	tf::pointTFToMsg(ros_tf.getOrigin(),pose.pose.position);
	tf::quaternionTFToMsg(ros_tf.getRotation(),pose.pose.orientation);

	//publish pose
	pose_pub.publish(pose);
	
	geometry_msgs::PoseStamped grasp_pose;
	grasp_pose.header.stamp = ros::Time::now();
	grasp_pose.header.frame_id = latest_cloud_frame;
	
	float min_tf_angle = 5*M_PI;
	float best_angle = 0;
	for (int i=0;i<4;i++) {
		float angle = i * M_PI_2;
		tf::Transform this_pose = tf::Transform::getIdentity();
		tf::Quaternion rot;
		rot.setEuler(angle,0.,0.);
		this_pose.setRotation(rot);
		tf::Transform overall = ros_tf * this_pose;
		float tf_angle = fabs(tf::Transform::getIdentity().getRotation().angle(overall.getRotation()));
		if (tf_angle < min_tf_angle) {
			min_tf_angle = tf_angle;
			best_angle = angle;
		}
	}
	
	tf::Transform grasp_pose_tf = tf::Transform::getIdentity();
	tf::Quaternion rot;
	rot.setEuler(best_angle,0.,0.);
	grasp_pose_tf.setRotation(rot);
	
	tf::pointTFToMsg(grasp_pose_tf.getOrigin(),grasp_pose.pose.position);
	tf::quaternionTFToMsg(grasp_pose_tf.getRotation(),grasp_pose.pose.orientation);

	//publish pose
	grasp_pub.publish(grasp_pose);
}

void save_current_cloud_callback(const sensor_msgs::PointCloud2& msg) {
	latest_cloud_frame = msg.header.frame_id;
	//pcl::fromROSMsg(msg, *latest_cloud); return;

	ColorCloudPtr cloud(new ColorCloud());
	ColorCloudPtr table_hull;
	float xmin, xmax, ymin, ymax;
	float table_height;
	std::vector<pcl::Vertices> table_polygons;


	pcl::fromROSMsg(msg, *cloud);
	table_height = getTableHeight(cloud);

	ColorCloudPtr in_table = getTablePoints(cloud, table_height);
	in_table = getBiggestCluster(in_table, TableConfig::TABLE_CLUSTERING_TOLERANCE);
	table_hull = findConvexHull(in_table, table_polygons);
	getTableBounds(in_table, xmin, xmax, ymin, ymax);

	ColorCloudPtr on_table = getPointsOnTableHull(cloud, table_hull, table_polygons, table_height+TableConfig::ABOVE_TABLE_CUTOFF);
	
	*latest_cloud = *on_table;
	
	
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "point_cloud_aligner");
	Parser parser;
	parser.addGroup(TableConfig());
	parser.read(argc, argv);

	ros::NodeHandle nh;

	pose_pub = nh.advertise<geometry_msgs::PoseStamped>("google_goggles/object_pose",1);
	grasp_pub = nh.advertise<geometry_msgs::PoseStamped>("google_goggles/grasp_pose",1);


	ros::spin();
	/*
	tracker = new TabletopTrackerROS(nh);
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
	*/
}
