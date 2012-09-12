#include <ros/ros.h>
#include <iostream>
#include <sstream>

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

#include "google_goggles/ObjectReferenceData.h"

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

static ros::ServiceClient* testGraspsService = 0;

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

void callback(const google_goggles::ObjectReferenceData& msg) {
	if (!tracker->initialized) { return; }
	ROS_INFO("*****************GOT REF CLOUD*********");
	ColorCloudPtr ref_cloud(new ColorCloud());
	pcl::fromROSMsg(msg.object.cloud, *ref_cloud);
	
	ColorCloudPtr latest_cloud = tracker->latest_cluster;
	
	ColorCloudPtr aligned_cloud(new ColorCloud());
	
	ColorPoint min, max;
	pcl::getMinMax3D(*latest_cloud,min,max);
	
	Affine3f best_tf;
	
	{
		float best_score = 1000000;
		float best_angle = -1;
		
		int num_angles = 8;
		
		int pub_angle_num = 1;
		
		for (int i=0;i<num_angles;i++) {
			float angle = i * (2. * M_PI / num_angles);
			std::cout << "testing " << i+1 << "/" << num_angles << "..." << std::endl;
			Quaternionf rot(AngleAxisf(angle,Vector3f(0,0,1)));
			
			ColorCloudPtr ref_cloud_rot(new ColorCloud());
			pcl::transformPointCloud(*ref_cloud,*ref_cloud_rot,Vector3f(0,0,0),rot);
			
			ColorPoint ref_min, ref_max;
			pcl::getMinMax3D(*ref_cloud_rot,ref_min,ref_max);
			
			Translation3f shift(
					min.x - ref_min.x,
					((max.y+min.y) - (ref_max.y+ref_min.y)) / 2,
					max.z - ref_max.z);
			
			Affine3f guess = shift * rot;
			
			std::cout << "angle: " << 180 * angle/M_PI << " shift: (" << shift.x() << ", " << shift.y() << ", " << shift.z() << ")" << std::endl;
			
			if (i+1 == pub_angle_num) {
				ROS_INFO("Publishing ref cloud");
				ColorCloudPtr ref_cloud_tranformed(new ColorCloud());
				pcl::transformPointCloud(*ref_cloud,*ref_cloud_tranformed,guess);
				sensor_msgs::PointCloud2 ref_cloud_msg;
				pcl::toROSMsg(*ref_cloud_tranformed,ref_cloud_msg);
				ref_cloud_msg.header.stamp = ros::Time::now();
				ref_cloud_msg.header.frame_id = tracker->map_frame;
				ref_cloud_pub.publish(ref_cloud_msg);
			}
			
			//icp
			pcl::IterativeClosestPoint<ColorPoint, ColorPoint> icp;
			icp.setInputCloud(ref_cloud);
			icp.setInputTarget(latest_cloud);
			
			icp.setMaxCorrespondenceDistance(0.01);
			
			std::cout << "aligning..." << std::endl;
			icp.align(*aligned_cloud,guess.matrix());
			
			double fitness = icp.getFitnessScore();
			std::cout << "has converged:" << icp.hasConverged() << " score: " << fitness << std::endl;
			
			Affine3f tf;
			tf.matrix() = icp.getFinalTransformation();
			
			if (icp.hasConverged() && fitness < best_score) {
				best_score = fitness;
				best_angle = angle;
				best_tf = tf;
			}
		}
		
		ROS_INFO_STREAM("Best angle: " << 180 * best_angle/M_PI << " with score: " << best_score);
	
		if (best_angle == -1) {
			ROS_INFO("ICP failed!");
			return;
		}
	}
	
	pcl::transformPointCloud(*ref_cloud,*aligned_cloud,best_tf);
	
	sensor_msgs::PointCloud2 aligned_cloud_msg;
	pcl::toROSMsg(*aligned_cloud,aligned_cloud_msg);
	//aligned_cloud_msg.header = msg.header;
	aligned_cloud_msg.header.stamp = ros::Time::now();
	aligned_cloud_msg.header.frame_id = tracker->map_frame;
	ROS_INFO("Publishing aligned cloud");
	aligned_cloud_pub.publish(aligned_cloud_msg);
	
	graspit_srvs::TestGrasps::Request treq;
	treq.object = msg.object;
	treq.object.cloud = aligned_cloud_msg;
	treq.grasps = msg.grasps;
	
	//apply transformation to grasps
	for (size_t i=0;i<msg.grasps.size();i++) {
		Quaternionf rot(msg.grasps[i].grasp_pose.orientation.x,msg.grasps[i].grasp_pose.orientation.y,msg.grasps[i].grasp_pose.orientation.z,msg.grasps[i].grasp_pose.orientation.w);
		Translation3f shift(msg.grasps[i].grasp_pose.position.x,msg.grasps[i].grasp_pose.position.y,msg.grasps[i].grasp_pose.position.z);
		Affine3f tf = best_tf * shift * rot;
		Affine3d tfd;
		tfd.matrix() = tf.matrix().cast<double>();
		tf::Transform ros_tf;
		tf::TransformEigenToTF(tfd,ros_tf);
		tf::pointTFToMsg(ros_tf.getOrigin(),treq.grasps[i].grasp_pose.position);
		tf::quaternionTFToMsg(ros_tf.getRotation(),treq.grasps[i].grasp_pose.orientation);
	}
	
	graspit_srvs::TestGrasps::Response tres;
	
	ROS_INFO("Testing grasps...");
	testGraspsService->call(treq,tres);
	
	ROS_INFO("Got result!");
	double max_quality = -1;
	size_t max_ind = 0;
	for (size_t i=0;i<tres.qualities.size();i++) {
		std::cout << "Grasp #" << i << " quality: " << tres.qualities[i] << std::endl;
		if (tres.qualities[i] > max_quality) {
			max_quality = tres.qualities[i];
			max_ind = i;
		}
	}
	
	Quaternionf reference_pose(AngleAxisf(M_PI_2,Vector3f(0,0,1)));
	
	std::vector<Affine3f> grasp_poses = getGraspPoses();
	size_t min_ind = 0;
	float min_angle = 100000;
	for (size_t i=0;i<grasp_poses.size();i++) {
		Affine3f pose = best_tf * grasp_poses[i];
		Quaternionf rot(pose.rotation());
		if (rot.angularDistance(reference_pose) < min_angle) {
			min_angle = rot.angularDistance(reference_pose);
			min_ind = i;
		}
	}
	
	std::cout << "Best grasp is #" << min_ind << " with angle " << min_angle << std::endl;
	
	Affine3f grasp_tf = best_tf * grasp_poses[min_ind];
	
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
	
	geometry_msgs::PoseStamped grasp_pose;
	grasp_pose.header.stamp = ros::Time::now();
	grasp_pose.header.frame_id = tracker->map_frame;
	
	Affine3d grasp_tfd;
	grasp_tfd.matrix() = grasp_tf.matrix().cast<double>();
	
	tf::Transform grasp_pose_tf = tf::Transform::getIdentity();
	/*tf::Quaternion rot;
	rot.setEuler(best_angle,0.,0.);
	grasp_pose_tf.setRotation(rot);*/
	tf::TransformEigenToTF(grasp_tfd,grasp_pose_tf);
	
	tf::pointTFToMsg(grasp_pose_tf.getOrigin(),grasp_pose.pose.position);
	tf::quaternionTFToMsg(grasp_pose_tf.getRotation(),grasp_pose.pose.orientation);

	//publish pose
	grasp_pub.publish(grasp_pose);
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "pose_estimator");
	Parser parser;
	parser.addGroup(TableConfig());
	parser.read(argc, argv);

	ros::NodeHandle nh;
	tracker = new TabletopTrackerROS(nh);
	ROS_INFO("Tracker ready");
	
	//ref_cloud_sub = nh.subscribe("/google_goggles/ref_cloud",1,&callback);
	ref_cloud_sub = nh.subscribe("/google_goggles/ref_data",1,&callback);
	
	ref_cloud_pub     = nh.advertise<sensor_msgs::PointCloud2>("google_goggles/ref_object",1);
	aligned_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("google_goggles/aligned_object",1);
	
	pose_pub = nh.advertise<geometry_msgs::PoseStamped>("google_goggles/object_pose",1);
	grasp_pub = nh.advertise<geometry_msgs::PoseStamped>("google_goggles/grasp_pose",1);
	
	testGraspsService = new ros::ServiceClient(nh.serviceClient<graspit_srvs::TestGrasps>("graspit/test_grasps"));

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