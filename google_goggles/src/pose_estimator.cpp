#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <math.h>

#include <pcl/common/common_headers.h>
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/ros/conversions.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/range_image/range_image.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>

#include <LinearMath/btTransform.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "tracker/tracker_ros.h"
#include "tracker/table_config.h"

#include "tracker/cloud_ops.h"

#include "config.h"

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

/*
struct PoseEstimatorConfig : public benk::Config {
	
	bool remove_occluded;
	float max_correspondence;
	float ransac_threshold;

	PoseEstimatorConfig() : benk::Config() {
		addOption(remove_occluded,bool,true);
		addOption(max_correspondence,float,0.02);
		addOption(ransac_threshold,float,0.05);
	}
};

namespace ::Config {
static PoseEstimatorConfig PoseEstimator = PoseEstimatorConfig();
}

//Use Config::PoseEstimator.remove_occluded
*/

struct PoseEstimatorConfiguration : public Config {
	static bool remove_occluded;
	static float max_correspondence;
	static float ransac_threshold;
	PoseEstimatorConfiguration() : Config() {
		params.push_back(new Parameter<bool>("remove-occluded", &remove_occluded, ""));
		params.push_back(new Parameter<float>("max-correspondence", &max_correspondence, ""));
		params.push_back(new Parameter<float>("ransac-threshold", &ransac_threshold, ""));
	}
};

static PoseEstimatorConfiguration PEConfig = PoseEstimatorConfiguration();
bool PoseEstimatorConfiguration::remove_occluded = false;
float PoseEstimatorConfiguration::max_correspondence = 0.01;
float PoseEstimatorConfiguration::ransac_threshold = 0.05;


Eigen::Affine3f toEigenTransform(const btTransform& transform) {
	btVector3 transBullet = transform.getOrigin();
	btQuaternion quatBullet = transform.getRotation();
	Eigen::Translation3f transEig;
	transEig = Eigen::Translation3f(transBullet.x(), transBullet.y(), transBullet.z());
	Eigen::Matrix3f rotEig = Eigen::Quaternionf(quatBullet.w(),quatBullet.x(),quatBullet.y(),quatBullet.z()).toRotationMatrix();
	Eigen::Affine3f out = transEig*rotEig;
	return out;
}

Eigen::Affine3f toEigenTransform(const tf::Transform& transform) {
	tf::Vector3 transBullet = transform.getOrigin();
	tf::Quaternion quatBullet = transform.getRotation();
	Eigen::Translation3f transEig;
	transEig = Eigen::Translation3f(transBullet.x(), transBullet.y(), transBullet.z());
	Eigen::Matrix3f rotEig = Eigen::Quaternionf(quatBullet.w(),quatBullet.x(),quatBullet.y(),quatBullet.z()).toRotationMatrix();
	Eigen::Affine3f out = transEig*rotEig;
	return out;
}

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

pcl::PointIndicesPtr getUnoccluded(CloudPtr ref_cloud,CloudPtr ref_cloud_guess,Vector3f camera_pt,CloudPtr ref_cloud_hull,std::vector<pcl::Vertices>& ref_cloud_hull_polygons) {
	pcl::PointIndicesPtr keep_indices(new pcl::PointIndices);
	for (size_t pt_ind=0;pt_ind<ref_cloud_guess->points.size() && !ros::isShuttingDown();pt_ind++) {
		
		Vector3f pt;
		pt[0] = ref_cloud_guess->points[pt_ind].x;
		pt[1] = ref_cloud_guess->points[pt_ind].y;
		pt[2] = ref_cloud_guess->points[pt_ind].z;
		Vector3f cam_to_pt = pt - camera_pt;
		bool in_any = false;
		for (size_t i=0;i<ref_cloud_hull_polygons.size() && !ros::isShuttingDown();i++) {
			pcl::Vertices polygon = ref_cloud_hull_polygons[i];
			std::vector<Vector3f> polygon_v;
			std::vector<Vector3f> polygon_v_proj;
			for (size_t j=0;j<polygon.vertices.size();j++) {
				int v_ind = polygon.vertices[j];
				//Point hull_pt = ref_cloud_guess.points[v_ind];
				Vector3f hull_pt;
				hull_pt[0] = ref_cloud_guess->points[v_ind].x;
				hull_pt[1] = ref_cloud_guess->points[v_ind].y;
				hull_pt[2] = ref_cloud_guess->points[v_ind].z;
				
				Vector3f cam_to_hull_pt = hull_pt - camera_pt;
				Vector3f hull_pt_on_line = cam_to_pt.dot(cam_to_hull_pt) * cam_to_pt.normalized();
				Vector3f hull_pt_proj = cam_to_hull_pt - hull_pt_on_line;
				
				polygon_v.push_back(hull_pt);
				polygon_v_proj.push_back(hull_pt_proj);
			}
			
			if (polygon_v.size() > 3) {
				std::cerr << "size too big" << std::endl;
			}
			
			Vector3f I;
			Vector3f    u, v, n;             // triangle vectors
			Vector3f    dir, w0, w;          // ray vectors
			float     r, a, b;             // params to calc ray-plane intersect

			// get triangle edge vectors and plane normal
			u = polygon_v[1] - polygon_v[0];
			v = polygon_v[2] - polygon_v[0];
			n = u.cross(v);             // cross product
			
			//if (n == (Vector)0)            // triangle is degenerate
			//	return -1;                 // do not deal with this case

			dir = pt - camera_pt;             // ray direction vector
			w0 = camera_pt - polygon_v[0];
			a = -n.dot(w0);
			b = n.dot(dir);
			if (fabs(b) < 0.0001) {
				// ray is parallel to triangle plane
				continue;
			}
				
				
			
			// get intersect point of ray with triangle plane
			r = a / b;
			//if (r < 0.0)                   // ray goes away from triangle
			//	return 0;                  // => no intersect
			// for a segment, also test if (r > 1.0) => no intersect
			
			
			if ( r < 1 ) {
				continue;
			}
			
			I = camera_pt + r * dir;           // intersect point of ray and plane

			// is I inside T?
			float    uu, uv, vv, wu, wv, D;
			uu = u.dot(u);
			uv = u.dot(v);
			vv = v.dot(v);
			w = I - polygon_v[0];
			wu = w.dot(u);
			wv = w.dot(v);
			D = uv * uv - uu * vv;

			// get and test parametric coords
			float s, t;
			s = (uv * wv - vv * wu) / D;
			if (s < 0.0 || s > 1.0) {
				// I is outside T
				continue;
			} else {
				t = (uv * wu - uu * wv) / D;
				if (t < 0.0 || (s + t) > 1.0) {
					// I is outside T
					continue;
				} else {
					// I is in T
					in_any = true;
					break;
				}
			}
		}
		if (!in_any) {
			keep_indices->indices.push_back((int)pt_ind);
		}
	}
	
	std::cout << "keeping " << keep_indices->indices.size() << "/" << ref_cloud->points.size() << std::endl;
	
	return keep_indices;
	
// 	CloudPtr nonoccluded_ref_cloud(new Cloud());
// 	pcl::copyPointCloud(*ref_cloud,keep_indices,*nonoccluded_ref_cloud);
// 	return nonoccluded_ref_cloud;
}

pcl::RangeImage getRangeImage(CloudPtr pointCloud,Affine3f sensorPose) {
	// We now want to create a range image from the above point cloud, with a 1deg angular resolution
	float angularResolution = (float) (  0.5f * (M_PI/180.0f));  //   1.0 degree in radians
	float maxAngleWidth     = (float) (120.0f * (M_PI/180.0f));  // 360.0 degree in radians
	float maxAngleHeight    = (float) (120.0f * (M_PI/180.0f));  // 180.0 degree in radians
	//Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
	float noiseLevel=0.00;
	float minRange = 0.0f;
	int borderSize = 1;
	
	pcl::RangeImage rangeImage;
	rangeImage.createFromPointCloud(*pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,
									sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
	return rangeImage;
}

CloudPtr align(const sensor_msgs::PointCloud2& pc,geometry_msgs::PoseStamped& pose_out) {
	if (!tracker->initialized) { return CloudPtr(); }
	ROS_INFO("*****************GOT REF CLOUD*********");
	CloudPtr ref_cloud(new Cloud());
	pcl::fromROSMsg(pc, *ref_cloud);
	
	ros::Time align_start_time = ros::Time::now();
	
	tf::StampedTransform stamped_transform;
	ROS_INFO("Looking up tf");
	listener->lookupTransform("/camera_rgb_optical_frame", pc.header.frame_id, ros::Time(0), stamped_transform);
	ROS_INFO("Done");
	Affine3f kinect_tf = toEigenTransform(stamped_transform);
	
	Vector3f camera_pt = kinect_tf.translation();
	
	downsampleCloudInPlace(ref_cloud,TableConfig::VOXEL_SIZE);
	
	Vector3f ref_cloud_centroid;
	Vector4f ref_cloud_centroid4;
	pcl::compute3DCentroid(*ref_cloud,ref_cloud_centroid4);
	for (int i=0;i<3;i++) {	ref_cloud_centroid[i] = ref_cloud_centroid4[i]; }
	Affine3f ref_to_centered = (Affine3f) Translation3f(-ref_cloud_centroid);
	CloudPtr ref_cloud_centered(new Cloud());
	pcl::transformPointCloud(*ref_cloud,*ref_cloud_centered,ref_to_centered);
	
	CloudPtr ref_cloud_hull (new Cloud());
	std::vector<pcl::Vertices> ref_cloud_hull_polygons;
	pcl::ConvexHull<Point> chull;
	chull.setInputCloud(ref_cloud);
	chull.reconstruct(*ref_cloud_hull,ref_cloud_hull_polygons);
	
	CloudPtr latest_cloud(new Cloud());
	pcl::copyPointCloud(*(tracker->latest_cluster),*latest_cloud );
	
	CloudPtr aligned_cloud(new Cloud());
	CloudPtr nonoccluded_aligned_cloud(new Cloud());
	CloudPtr best_nonoccluded_aligned_cloud(new Cloud());
	
	Point min, max;
	pcl::getMinMax3D(*latest_cloud,min,max);
	
	Affine3f best_tf;
	
	{
		float best_score = 1000000;
		bool got_best = false;
		float best_angle = 5000;
		Vector3f best_axis(0,0,0);
		Quaternionf best_rotation;
		
		
		int pub_tf_num = -1;
		
		float angle;
		Vector3f axis;
		std::vector<Quaternionf> transforms;
		
		int num_angles = 360/5;
		for (int i=0;i<num_angles;i++) {
			angle = i * 2.0 * M_PI / num_angles;
			axis = Vector3f(0,0,1);
			Quaternionf rot(AngleAxisf(angle,axis.normalized()));
			
			transforms.push_back(rot);
			
			for (int j=-1;j<=1;j+=1) {
				for (int k=-1;k<=1;k+=1) {
					axis = Vector3f(j,k,0);
					Quaternionf rot2(AngleAxisf(M_PI_2,axis.normalized()));
					transforms.push_back(rot2 * rot);
				}
			}
			std::cout << "tf sz " << transforms.size() << std::endl;
		}
		
		/*
		int num_transforms = 1000;
		transforms.push_back(Quaternionf(AngleAxisf(0,Vector3f(0,0,1))));
		for (int i=1;i<=num_transforms;i++) {
			angle = rand() * 2.0 * M_PI / RAND_MAX;
			axis.setRandom();
			Quaternionf rot(AngleAxisf(angle,axis.normalized()));
			
			transforms.push_back(rot);
		}
		*/
		
		float first_score;
		for (size_t i=0;i<transforms.size() && !ros::isShuttingDown();i++) {
			std::cout << "testing " << i+1 << "/" << transforms.size() << "..." << std::endl;
			
			Quaternionf rot = transforms[i];
			
			//float angle = i * (2. * M_PI / num_angles);
			//std::cout << "testing " << i+1 << "/" << num_angles << "..." << std::endl;
			//Quaternionf rot(AngleAxisf(angle,Vector3f(0,0,1)));
			
			
			
			CloudPtr ref_cloud_centered_rot(new Cloud());
			pcl::transformPointCloud(*ref_cloud_centered,*ref_cloud_centered_rot,Vector3f(0,0,0),rot);
			
			CloudPtr ref_cloud_rot(new Cloud());
			pcl::transformPointCloud(*ref_cloud_centered_rot,*ref_cloud_rot,ref_to_centered.inverse());
			
			Point ref_min, ref_max;
			pcl::getMinMax3D(*ref_cloud_rot,ref_min,ref_max);
			
			float shift_x, shift_y, shift_z;
			shift_x = min.x - ref_min.x;
			shift_y = ((max.y+min.y) - (ref_max.y+ref_min.y)) / 2;
			//shift_z = max.z - ref_max.z; //align tops of clouds
			shift_z = tracker->table_height - ref_min.z; //align bottom of ref cloud with table
			shift_z = tracker->table_height; //align bottom of ref cloud with table
			
			Translation3f shift(shift_x,shift_y,shift_z);
			
			Affine3f guess = shift * ref_to_centered.inverse() * rot * ref_to_centered;
			
			//std::cout << "angle: " << 180 * angle/M_PI << " shift: (" << shift.x() << ", " << shift.y() << ", " << shift.z() << ")" << std::endl;
			
			CloudPtr ref_cloud_guess(new Cloud());
			pcl::transformPointCloud(*ref_cloud,*ref_cloud_guess,guess);
			
			if ((int)i == pub_tf_num) {
				ROS_INFO("Publishing ref cloud");
				CloudPtr ref_cloud_tranformed(new Cloud());
				pcl::transformPointCloud(*ref_cloud,*ref_cloud_tranformed,guess);
				sensor_msgs::PointCloud2 ref_cloud_msg;
				pcl::toROSMsg(*ref_cloud_tranformed,ref_cloud_msg);
				ref_cloud_msg.header.stamp = ros::Time::now();
				ref_cloud_msg.header.frame_id = tracker->map_frame;
				ref_cloud_pub.publish(ref_cloud_msg);
				ros::Duration(1).sleep();
			}
			
			bool remove_occluded = PEConfig.remove_occluded;
			//bool remove_occluded = false;
			
			
			CloudPtr input_cloud = ref_cloud;
			
			if (remove_occluded) {
				pcl::PointIndicesPtr inds = getUnoccluded(ref_cloud,ref_cloud_guess,camera_pt,ref_cloud_hull,ref_cloud_hull_polygons);
				input_cloud = CloudPtr(new Cloud());
				pcl::copyPointCloud(*ref_cloud,*inds,*input_cloud);
			} else {
				input_cloud = ref_cloud;
			}
			
			//icp
			pcl::IterativeClosestPoint<Point, Point> icp;
			//pcl::IterativeClosestPointNonLinear<Point, Point> icp;
			//icp.setInputCloud(ref_cloud);
			//icp.setInputCloud(nonoccluded_ref_cloud);
			icp.setInputCloud(input_cloud);
			
			if (remove_occluded) {
				//icp.setIndices(getUnoccluded(ref_cloud,ref_cloud_guess,camera_pt,ref_cloud_hull,ref_cloud_hull_polygons));
			}
			
			icp.setInputTarget(latest_cloud);
			
			//icp.setMaxCorrespondenceDistance(0.01);
			icp.setMaxCorrespondenceDistance(PEConfig.max_correspondence);
			//icp.setRANSACOutlierRejectionThreshold(0.05);
			icp.setRANSACOutlierRejectionThreshold(PEConfig.ransac_threshold);
			//std::cout << "ransac thresh: " << icp.getRANSACOutlierRejectionThreshold() << std::endl;
			
			CloudPtr output_cloud(new Cloud());
			
			//std::cout << "aligning..." << std::endl;
			//icp.align(*aligned_cloud,guess.matrix());
			icp.align(*output_cloud,guess.matrix());
			
			*nonoccluded_aligned_cloud = *output_cloud;
			
			double fitness = icp.getFitnessScore();
			//std::cout << "has converged:" << icp.hasConverged() << " score: " << fitness << std::endl;
			if (i==0) {
				first_score = fitness;
			}
			
			Affine3f tf;
			tf.matrix() = icp.getFinalTransformation();
			
			pcl::transformPointCloud(*ref_cloud,*aligned_cloud,tf);
			
			if (icp.hasConverged() && fitness < best_score) {
				got_best = true;
				best_score = fitness;
				best_angle = angle;
				best_axis = axis;
				best_rotation = rot;
				best_tf = tf;
				best_nonoccluded_aligned_cloud = nonoccluded_aligned_cloud;
			}
			
			if (false && icp.hasConverged()) {
				sensor_msgs::PointCloud2 this_aligned_cloud_msg;
				pcl::toROSMsg(*output_cloud,this_aligned_cloud_msg);
				this_aligned_cloud_msg.header.stamp = ros::Time::now();
				this_aligned_cloud_msg.header.frame_id = tracker->map_frame;
				ROS_INFO("Publishing this aligned cloud %d",(int)this_aligned_cloud_msg.width);
				aligned_cloud_pub.publish(this_aligned_cloud_msg);
				ros::Duration(1).sleep();
			}
		}
		
		if (!got_best) {
			ROS_ERROR("ICP failed!");
			return CloudPtr();
		}
		
		ROS_INFO_STREAM("Best: axis (" << best_axis.x() << "," << best_axis.y() << "," << best_axis.z() << ")" << " angle: " << 180 * best_angle/M_PI << " with score: " << best_score);
		ROS_INFO_STREAM("Identity score: " << first_score);
	}
	
	pcl::transformPointCloud(*ref_cloud,*aligned_cloud,best_tf);
	
	ros::Time align_end_time = ros::Time::now();
	
	ros::Duration align_time = align_end_time - align_start_time;
	ROS_INFO_STREAM("Alignment took " << align_time.toSec() << "s");
	
	sensor_msgs::PointCloud2 aligned_cloud_msg;
	pcl::toROSMsg(*aligned_cloud,aligned_cloud_msg);
	//pcl::toROSMsg(*best_nonoccluded_aligned_cloud,aligned_cloud_msg);
	//aligned_cloud_msg.header = msg.header;
	aligned_cloud_msg.header.stamp = ros::Time::now();
	aligned_cloud_msg.header.frame_id = tracker->map_frame;
	ROS_INFO("Publishing aligned cloud");
	aligned_cloud_pub.publish(aligned_cloud_msg);
	
	/**************** Publishing **********************/

	Affine3d tfd;
	tfd.matrix() = best_tf.matrix().cast<double>();

	tf::Transform ros_tf;
	tf::transformEigenToTF(tfd,ros_tf);

	geometry_msgs::PoseStamped pose;
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = tracker->map_frame;

	tf::pointTFToMsg(ros_tf.getOrigin(),pose.pose.position);
	tf::quaternionTFToMsg(ros_tf.getRotation(),pose.pose.orientation);

	//publish pose
	pose_pub.publish(pose);
	pose_out = pose;
	
	return aligned_cloud;
}

bool serviceCallbackObject(google_goggles_srvs::AlignObject::Request& request, google_goggles_srvs::AlignObject::Response& response) {
	ROS_INFO("AlignObject service called!");
	geometry_msgs::PoseStamped pose;
	pose.header.stamp = ros::Time(0);
	CloudPtr aligned_cloud = align(request.object.cloud,pose);
	if (pose.header.stamp == ros::Time(0)) {
		ROS_ERROR("Align call failed!");
		return false;
	}
	response.aligned_object = request.object;
	pcl::toROSMsg(*aligned_cloud,response.aligned_object.cloud);
	response.pose = pose;
	return true;
}

bool serviceCallbackCloud(google_goggles_srvs::AlignCloud::Request& request, google_goggles_srvs::AlignCloud::Response& response) {
	ROS_INFO("AlignObject service called!");
	geometry_msgs::PoseStamped pose;
	pose.header.stamp = ros::Time(0);
	CloudPtr aligned_cloud = align(request.cloud,pose);
	if (pose.header.stamp == ros::Time(0)) {
		ROS_ERROR("Align call failed!");
		return false;
	}
	pcl::toROSMsg(*aligned_cloud,response.aligned_cloud);
	response.pose = pose;
	return true;
}

void callback(const google_goggles_msgs::ObjectReferenceData& msg) {
	ROS_INFO("Received object reference data");
	geometry_msgs::PoseStamped pose;
	align(msg.object.cloud,pose);
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "pose_estimator");
	
	Parser parser;
	parser.addGroup(TableConfig());
	parser.addGroup(PEConfig);
	parser.read(argc, argv);
	
	std::cout << "ro " << PEConfig.remove_occluded << " md " << PEConfig.max_correspondence << " rt " << PEConfig.ransac_threshold << std::endl;
	
	bool latch_aligned_cloud = false;
	for (int i=1;i<argc;i++) {
		std::string arg(argv[i]);
		latch_aligned_cloud |= (arg == "--latch-cloud");
	}
	
	ros::NodeHandle nh;
	tracker = new TabletopTrackerROS(nh);
	ROS_INFO("Tracker created");
	
	listener = new tf::TransformListener();
	
	//ref_cloud_sub = nh.subscribe("/google_goggles/ref_cloud",1,&callback);
	ref_cloud_sub = nh.subscribe("object_ref_data",1,&callback);
	
	ref_cloud_pub     = nh.advertise<sensor_msgs::PointCloud2>("ref_object",1,true);
	aligned_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("aligned_object",1,latch_aligned_cloud);
	
	pose_pub = nh.advertise<geometry_msgs::PoseStamped>("object_pose",1,false);
	//grasp_pub = nh.advertise<geometry_msgs::PoseStamped>("grasp_pose",1,true);
	
	object_data_pub = nh.advertise<google_goggles_msgs::ObjectData>("aligned_object_data",1,true);
	
	testGraspsService = new ros::ServiceClient(nh.serviceClient<graspit_srvs::TestGrasps>("test_grasps"));
	
	alignObjectService = new ros::ServiceServer(nh.advertiseService("align_object", serviceCallbackObject));
	alignCloudService = new ros::ServiceServer(nh.advertiseService("align_cloud", serviceCallbackCloud));
	
	ROS_INFO("Pose estimator ready");

	ros::Duration(1).sleep();

	ros::Time last_pub = ros::Time(0);
	bool first_msg = false;
	while (ros::ok()) {
		for (int i=0; i < 10; i++) { ros::spinOnce(); }
		ros::Duration max_pub_interval = ros::Duration(1);
		ros::Time now = ros::Time::now();
		first_msg |= tracker->hasPendingMessage;
		if (tracker->hasPendingMessage || (first_msg && now-last_pub > max_pub_interval)) {
			//ROS_INFO("tracker has pending message. updating");
			try {
				tracker->updateAll();
				tracker->publish();
				tracker->hasPendingMessage = false;
				last_pub = now;
			} catch (std::runtime_error err) {
				ROS_ERROR_STREAM("error while updating tracker: " <<err.what());
				ros::Duration(.1).sleep();
			}
		} else {
			ros::Duration(0.01).sleep();
		}
	}
}
