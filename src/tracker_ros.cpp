#include "tracker_ros.h"
#include <string>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/io/pcd_io.h>
//#include <misc_msgs/TrackedCylinders.h>
using namespace std;
using namespace Eigen;

static string map_frame = "/odom_combined";
static string base_frame = "/base_link";

Eigen::Affine3f toEigenTransform(const btTransform& transform) {
	btVector3 transBullet = transform.getOrigin();
	btQuaternion quatBullet = transform.getRotation();
	Eigen::Translation3f transEig;
	transEig = Eigen::Translation3f(transBullet.x(), transBullet.y(), transBullet.z());
	Eigen::Matrix3f rotEig = Eigen::Quaternionf(quatBullet.w(),quatBullet.x(),quatBullet.y(),quatBullet.z()).toRotationMatrix();
	Eigen::Affine3f out = transEig*rotEig;
	return out;
}

TabletopTrackerROS::TabletopTrackerROS(ros::NodeHandle nh) : hasPendingMessage(false) {
	points_pub = nh.advertise<sensor_msgs::PointCloud2>("google_goggles/points",100);
	cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("google_goggles/cloud",100);
	cluster_pub = nh.advertise<sensor_msgs::PointCloud2>("google_goggles/clusters",100);
	//cyl_pub = nh.advertise<misc_msgs::TrackedCylinders>("spinning_tabletop/cylinders",100);
	//cloud_sub = nh.subscribe("input_cloud",1,&TabletopTrackerROS::callback, this);
	cloud_sub = nh.subscribe("/camera/depth_registered/points",1,&TabletopTrackerROS::callback, this);
}


void TabletopTrackerROS::callback(const sensor_msgs::PointCloud2& msg) {
	ROS_INFO("Got msg");
	ColorCloudPtr cloud(new ColorCloud());
	pcl::fromROSMsg(msg, *cloud);
	setLatest(cloud);
	latest_stamp = msg.header.stamp;
	hasPendingMessage = true;
}

void TabletopTrackerROS::updateTransform() {
	tf::StampedTransform stamped_transform;
	//listener.lookupTransform("/base_footprint", "/head_mount_kinect_rgb_optical_frame", ros::Time(0), stamped_transform);
	//listener.lookupTransform("/base_footprint", "/narrow_stereo_optical_frame", ros::Time(0), stamped_transform);
	//listener.lookupTransform("/base_footprint", "/camera_rgb_optical_frame", ros::Time(0), stamped_transform);
	//listener.lookupTransform("/base_footprint", "/openni_rgb_optical_frame", ros::Time(0), stamped_transform);
	listener.lookupTransform(map_frame, "/openni_rgb_optical_frame", ros::Time(0), stamped_transform);
	map_transform = toEigenTransform(stamped_transform.asBt());
	listener.lookupTransform(base_frame, map_frame, ros::Time(0), stamped_transform);
	map_to_base_transform = toEigenTransform(stamped_transform.asBt());
	
}

ColorCloudPtr addColor(ColorCloudPtr in, uint8_t r, uint8_t g, uint8_t b) {
	ColorCloudPtr out(new ColorCloud());
	BOOST_FOREACH(ColorPoint& pt, in->points) {
		pcl::PointXYZRGB outpt;
		outpt.x = pt.x;
		outpt.y = pt.y;
		outpt.z = pt.z;
		outpt.r = r;
		outpt.g = g;
		outpt.b = b;
		out->push_back(outpt);
	}
	out->width = in->width;
	out->height = in->height;
	out->is_dense = in->is_dense;
	return out;
}

ColorCloudPtr colorByID(ColorCloudPtr in, int id) {
	uint8_t r = (id * 53)%251;
	uint8_t g = (id * 139)%251;
	uint8_t b = (id * 223)%251;
	return addColor(in, r, g, b);
}


void TabletopTrackerROS::publish() {
	// turnpick_msgs::TabletopClusters tc;
	// turnpick_msgs::Table table;
	// vector<sensor_msgs::PointCloud2> objects;
	// vector<int> objects_id;
	
	// table.x_min = xmin;
	// table.x_max = xmax;
	// table.y_min = ymin;
	// table.y_max = ymax;
	//sensor_msgs::PointCloud2 pc;
	//pcl::toROSMsg(*table_hull, pc);
	
	if (combined_cluster) {
	sensor_msgs::PointCloud2 pc;
	pcl::toROSMsg(*combined_cluster, pc);
	pc.header.frame_id = map_frame;
	pc.header.stamp = ros::Time(0);
	cloud_pub.publish(pc);
	
// 	pcl::toROSMsg(*latest_cluster, pc);
// 	pc.header.frame_id = map_frame;
// 	pc.header.stamp = ros::Time(0);
// 	points_pub.publish(pc);
	}

	for (size_t i=0; i < clusters.size(); i++) {
		sensor_msgs::PointCloud2 pc;
		ColorCloudPtr c = clusters[i];
		int id = ids[i];
		ColorCloudPtr coloredCloud = colorByID(c,id);
		pcl::toROSMsg(*coloredCloud,pc);
		pc.header.frame_id = map_frame;
		pc.header.stamp = ros::Time(0);
		cluster_pub.publish(pc);

//		 misc_msgs::TrackedCylinders tc;
//		 tc.header.frame_id = map_frame;
//		 tc.header.stamp = latest_stamp;
//		 tc.ids = ids;
//		 BOOST_FOREACH(VectorXf& cyl, cylinder_params) {
//			 tc.xs.push_back(cyl(0));
//			 tc.ys.push_back(cyl(1));
//			 tc.zs.push_back(cyl(2));
//			 tc.rs.push_back(cyl(3));
//			 tc.hs.push_back(cyl(4));
//		 }
//		 cyl_pub.publish(tc);

	}

	// tc.table = table;
	// tc.objects = objects;
	// tc.objects_id = objects_id;

}
