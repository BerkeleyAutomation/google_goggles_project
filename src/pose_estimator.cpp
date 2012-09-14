#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <math.h>

#include <pcl/common/common_headers.h>
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>

#include <pcl/registration/icp.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/ros/conversions.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "tracker/tracker_ros.h"
#include "tracker/table_config.h"

#include "tracker/cloud_ops.h"

#include <google_goggles_msgs/ObjectReferenceData.h>
#include <google_goggles_msgs/ObjectData.h>
#include <google_goggles_srvs/AlignObject.h>
#include <google_goggles_srvs/AlignCloud.h>

#include <graspit_srvs/TestGrasps.h>


using namespace std;
using namespace Eigen;

static TabletopTrackerROS* tracker;

static tf::TransformListener* listener;

ros::Subscriber ref_cloud_sub;
ros::Publisher ref_cloud_pub;

ros::Publisher aligned_cloud_pub;

ros::Publisher pose_pub;
ros::Publisher grasp_pub;

ros::Publisher object_data_pub;

static ros::ServiceClient* testGraspsService = 0;
static ros::ServiceServer* alignObjectService = 0;
static ros::ServiceServer* alignCloudService = 0;

std::vector<Affine3f> getGraspPoses() {
	std::vector<Affine3f> poses;
	int num_angles = 16;
	for (int i=0;i<num_angles;i++) {
		float angle = i * (2. * M_PI / num_angles);
		poses.push_back(Affine3f(AngleAxisf(angle,Vector3f(0,0,1))));
	}
	for (size_t i=0;i<poses.size();i++) {
		poses[i] = Translation3f(Vector3f(0,0,0.10)) * poses[i];
	}
	return poses;
}

bool align(const sensor_msgs::PointCloud2& pc,geometry_msgs::PoseStamped& pose_out) {
	if (!tracker->initialized) { return false; }
	ROS_INFO("*****************GOT REF CLOUD*********");
	CloudPtr ref_cloud(new Cloud());
	pcl::fromROSMsg(pc, *ref_cloud);
	
	downsampleCloudInPlace(ref_cloud,TableConfig::VOXEL_SIZE);
	
	CloudPtr latest_cloud(new Cloud());
	pcl::copyPointCloud(*(tracker->latest_cluster),*latest_cloud );
	
	CloudPtr aligned_cloud(new Cloud());
	
	Point min, max;
	pcl::getMinMax3D(*latest_cloud,min,max);
	
	Affine3f best_tf;
	
	{
		float best_score = 1000000;
		bool got_best = false;
		float best_angle;
		Vector3f best_axis;
		Quaternionf best_rotation;
		
		
		int num_transforms = 1000;
		
		int pub_tf_num = -1;
		
		float angle;
		Vector3f axis;
		for (int i=1;i<=num_transforms;i++) {
			std::cout << "testing " << i << "/" << num_transforms << "..." << std::endl;
			angle = rand() * 2.0 * M_PI / RAND_MAX;
			axis.setRandom();
			Quaternionf rot(AngleAxisf(angle,axis.normalized()));
			
			//float angle = i * (2. * M_PI / num_angles);
			//std::cout << "testing " << i+1 << "/" << num_angles << "..." << std::endl;
			//Quaternionf rot(AngleAxisf(angle,Vector3f(0,0,1)));
			
			CloudPtr ref_cloud_rot(new Cloud());
			pcl::transformPointCloud(*ref_cloud,*ref_cloud_rot,Vector3f(0,0,0),rot);
			
			Point ref_min, ref_max;
			pcl::getMinMax3D(*ref_cloud_rot,ref_min,ref_max);
			
			Translation3f shift(
					min.x - ref_min.x,
					((max.y+min.y) - (ref_max.y+ref_min.y)) / 2,
					max.z - ref_max.z);
			
			Affine3f guess = shift * rot;
			
			//std::cout << "angle: " << 180 * angle/M_PI << " shift: (" << shift.x() << ", " << shift.y() << ", " << shift.z() << ")" << std::endl;
			
			if (i == pub_tf_num) {
				ROS_INFO("Publishing ref cloud");
				CloudPtr ref_cloud_tranformed(new Cloud());
				pcl::transformPointCloud(*ref_cloud,*ref_cloud_tranformed,guess);
				sensor_msgs::PointCloud2 ref_cloud_msg;
				pcl::toROSMsg(*ref_cloud_tranformed,ref_cloud_msg);
				ref_cloud_msg.header.stamp = ros::Time::now();
				ref_cloud_msg.header.frame_id = tracker->map_frame;
				ref_cloud_pub.publish(ref_cloud_msg);
			}
			
			//icp
			pcl::IterativeClosestPoint<Point, Point> icp;
			icp.setInputCloud(ref_cloud);
			icp.setInputTarget(latest_cloud);
			
			icp.setMaxCorrespondenceDistance(0.01);
			
			//std::cout << "aligning..." << std::endl;
			icp.align(*aligned_cloud,guess.matrix());
			
			double fitness = icp.getFitnessScore();
			//std::cout << "has converged:" << icp.hasConverged() << " score: " << fitness << std::endl;
			
			Affine3f tf;
			tf.matrix() = icp.getFinalTransformation();
			
			if (icp.hasConverged() && fitness < best_score) {
				got_best = true;
				best_score = fitness;
				best_angle = angle;
				best_axis = axis;
				best_rotation = rot;
				best_tf = tf;
			}
		}
		
		if (!got_best) {
			ROS_ERROR("ICP failed!");
			return false;
		}
		
		ROS_INFO_STREAM("Best: axis (" << best_axis.x() << "," << best_axis.y() << "," << best_axis.z() << ")" << " angle: " << 180 * best_angle/M_PI << " with score: " << best_score);
	}
	
	pcl::transformPointCloud(*ref_cloud,*aligned_cloud,best_tf);
	
	sensor_msgs::PointCloud2 aligned_cloud_msg;
	pcl::toROSMsg(*aligned_cloud,aligned_cloud_msg);
	//aligned_cloud_msg.header = msg.header;
	aligned_cloud_msg.header.stamp = ros::Time::now();
	aligned_cloud_msg.header.frame_id = tracker->map_frame;
	ROS_INFO("Publishing aligned cloud");
	aligned_cloud_pub.publish(aligned_cloud_msg);
	
	/**************** Publishing **********************/

	Affine3d tfd;
	tfd.matrix() = best_tf.matrix().cast<double>();

	tf::Transform ros_tf;
	tf::TransformEigenToTF(tfd,ros_tf);

	geometry_msgs::PoseStamped pose;
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = tracker->map_frame;

	tf::pointTFToMsg(ros_tf.getOrigin(),pose.pose.position);
	tf::quaternionTFToMsg(ros_tf.getRotation(),pose.pose.orientation);

	//publish pose
	pose_pub.publish(pose);
	pose_out = pose;
	
	return true;
}

bool serviceCallbackObject(google_goggles_srvs::AlignObject::Request& request, google_goggles_srvs::AlignObject::Response& response) {
	ROS_INFO("AlignObject service called!");
	geometry_msgs::PoseStamped pose;
	pose.header.stamp = ros::Time(0);
	align(request.object.cloud,pose);
	if (pose.header.stamp == ros::Time(0)) {
		ROS_ERROR("Align call failed!");
		return false;
	}
	response.pose = pose;
	return true;
}

bool serviceCallbackCloud(google_goggles_srvs::AlignCloud::Request& request, google_goggles_srvs::AlignCloud::Response& response) {
	ROS_INFO("AlignObject service called!");
	geometry_msgs::PoseStamped pose;
	pose.header.stamp = ros::Time(0);
	align(request.cloud,pose);
	if (pose.header.stamp == ros::Time(0)) {
		ROS_ERROR("Align call failed!");
		return false;
	}
	response.pose = pose;
	return true;
}

void callback(const google_goggles_msgs::ObjectReferenceData& msg) {
	geometry_msgs::PoseStamped pose;
	align(msg.object.cloud,pose);
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "pose_estimator");
	Parser parser;
	parser.addGroup(TableConfig());
	parser.read(argc, argv);

	ros::NodeHandle nh;
	tracker = new TabletopTrackerROS(nh);
	ROS_INFO("Tracker created");
	
	listener = new tf::TransformListener();
	
	//ref_cloud_sub = nh.subscribe("/google_goggles/ref_cloud",1,&callback);
	ref_cloud_sub = nh.subscribe("object_ref_data",1,&callback);
	
	ref_cloud_pub     = nh.advertise<sensor_msgs::PointCloud2>("ref_object",1,true);
	aligned_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("aligned_object",1,true);
	
	pose_pub = nh.advertise<geometry_msgs::PoseStamped>("object_pose",1,true);
	grasp_pub = nh.advertise<geometry_msgs::PoseStamped>("grasp_pose",1,true);
	
	object_data_pub = nh.advertise<google_goggles_msgs::ObjectData>("aligned_object_data",1,true);
	
	testGraspsService = new ros::ServiceClient(nh.serviceClient<graspit_srvs::TestGrasps>("test_grasps"));
	
	alignObjectService = new ros::ServiceServer(nh.advertiseService("align_object", serviceCallbackObject));
	alignCloudService = new ros::ServiceServer(nh.advertiseService("align_cloud", serviceCallbackCloud));
	
	ROS_INFO("Pose estimator ready");

	ros::Duration(1).sleep();

	while (ros::ok()) {
		for (int i=0; i < 10; i++) { ros::spinOnce(); }
		if (tracker->hasPendingMessage) {
			ROS_INFO("tracker has pending message. updating");
			try {
				tracker->updateAll();
				tracker->publish();
				tracker->hasPendingMessage = false;
			} catch (std::runtime_error err) {
				ROS_ERROR_STREAM("error while updating tracker: " <<err.what());
				ros::Duration(.1).sleep();
			}
		} else {
			ros::Duration(0.01).sleep();
		}
	}
}