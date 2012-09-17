#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <rosbag/bag.h>

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>

#include <boost/foreach.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>

#include <pcl/point_types.h>
#include <pcl/common/pca.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl/common/eigen.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/concave_hull.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/PolygonMesh.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/UInt16MultiArray.h>

#include <graspit_srvs/GenerateGrasps.h>
#include <graspit_srvs/TestGrasps.h>

#include <google_goggles_srvs/CreateMesh.h>
#include <google_goggles_msgs/ObjectReferenceData.h>

using namespace std;
using namespace Eigen;

static pcl::visualization::PCLVisualizer* viewer = 0;
static pcl::PolygonMesh* mesh_to_view = 0;

static Affine3f base_tf = Affine3f::Identity();

static vector<Affine3f> object_rotations;

static ros::ServiceClient* generateGraspsService = 0;
static ros::ServiceClient* testGraspsService = 0;
static int num_grasps = 7;

static ros::Publisher* cloud_pub = 0;
static ros::Publisher* grasps_pub = 0;

static bool use_bag = false;
static rosbag::Bag* bag = 0;
static std::string bag_filename;


static bool mls_polynomialFit = true;
static float mls_searchRadius = 0.03;
static int normal_k = 20;

static float mesh_searchRadius = 0.025;

// Set typical values for the parameters
static float mesh_mu = 2.5;
static float mesh_maximumNearestNeighbors = 100;
static float mesh_maximumSurfaceAngle = M_PI/4; // 45 degrees
static float mesh_minimumAngle = M_PI/18; // 10 degrees
static float mesh_maximumAngle = 2*M_PI/3; // 120 degrees
static float mesh_normalConsistency = false;


/*
bool mls_polynomialFit = true;
float mls_searchRadius = 0.03;
int normal_k = 20;

float mesh_searchRadius = 0.025;

// Set typical values for the parameters
float mesh_mu = 2.5;
float mesh_maximumNearestNeighbors = 100;
float mesh_maximumSurfaceAngle = M_PI/4; // 45 degrees
float mesh_minimumAngle = M_PI/18; // 10 degrees
float mesh_maximumAngle = 2*M_PI/3; // 120 degrees
float mesh_normalConsistency = false;
*/

bool point_cloud_to_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,pcl::PolygonMesh& mesh) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::transformPointCloud(*cloud_in,*cloud,base_tf);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>());
	
	*cloud2 = *cloud;
	
	float max_z = 0;
	BOOST_FOREACH(pcl::PointXYZ& pt, cloud2->points) {
		if (pt.z > max_z) {
			max_z = pt.z;
		}
		//pt.x = -pt.x;
		//pt.y = -pt.y;
		pt.x = pt.y;
		pt.y = pt.x;
	}
	//*cloud += *cloud2;
	
	// Create a KD-Tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree0 (new pcl::search::KdTree<pcl::PointXYZ>);

	// Output has the PointNormal type in order to store the normals calculated by MLS
	//pcl::PointCloud<pcl::PointNormal> cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr mls_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	
	//mls.setComputeNormals (true);
	//mls.setOutputNormals(cloud_with_normals);

	// Set parameters
	mls.setInputCloud (cloud);
	mls.setPolynomialFit (true);
	mls.setSearchMethod (tree0);
	//mls.setSearchRadius (mls_searchRadius);
	mls.setSearchRadius (0.03);
	
	// Reconstruct
	ROS_INFO("Preprocessing point cloud");
	//mls.process (*cloud_with_normals);
	mls.reconstruct(*mls_cloud);
	
	*cloud = *mls_cloud;
	
	/*
	ROS_INFO("Creating concave hull");
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ConcaveHull<pcl::PointXYZ> chull;
	chull.setInputCloud(mls_cloud);
	chull.setAlpha(0.1);
	chull.reconstruct(*cloud_hull);
	
	*cloud = *cloud_hull;
	*/
	
	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZ>);
	tree1->setInputCloud (cloud);
	n.setInputCloud (cloud);
	n.setSearchMethod (tree1);
	//n.setKSearch (normal_k);
	n.setKSearch (20);
	ROS_INFO("Computing normals");
	n.compute (*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud (cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	//pcl::PolygonMesh mesh;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius (mesh_searchRadius);

	gp3.setMu (mesh_mu);
	gp3.setMaximumNearestNeighbors (mesh_maximumNearestNeighbors);
	gp3.setMaximumSurfaceAngle(mesh_maximumSurfaceAngle); // 45 degrees
	gp3.setMinimumAngle(mesh_minimumAngle); // 10 degrees
	gp3.setMaximumAngle(mesh_maximumAngle); // 120 degrees
	gp3.setNormalConsistency(mesh_normalConsistency);
	/*
	gp3.setMu (2.5);
	gp3.setMaximumNearestNeighbors (100);
	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(false);
	*/
	
	
	
	// Get result
	gp3.setInputCloud (cloud_with_normals);
	//gp3.setInputCloud(cloud_hull);
	gp3.setSearchMethod (tree2);
	ROS_INFO("Constructing mesh");
	gp3.reconstruct (mesh);

	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();
	
	{
		std::vector<double> v_new;
		std::vector<int> t_new;
		
		sensor_msgs::PointCloud2 pc = mesh.cloud;
		std::vector<pcl::Vertices> polygons = mesh.polygons;

		if (pc.height != 1) {
			ROS_ERROR("2D point cloud!");
			return false;
		} else if (pc.width <= 0) {
			ROS_ERROR("Invalid width!");
			return false;
		}

		ROS_INFO("converting vertices");
		for (uint32_t i=0;i<pc.width;i++) {
			if (pc.fields[0].name != "x" || pc.fields[1].name != "y" || pc.fields[2].name != "z") {
				ROS_ERROR("point cloud has bad fields!");
				return false;
			} else if (pc.fields[0].datatype != sensor_msgs::PointField::FLOAT32) {
				ROS_ERROR("point cloud is not float32!");
				return false;
			}
			uint32_t offset = pc.point_step * i;
			for (int j=0;j<3;j++) {
				float val;
				char data[4];
				for (int k=0;k<4;k++) { data[k] = pc.data[offset+j*4+k]; }
				memcpy(&val,data,4);
				v_new.push_back(val);
			}

		}

		ROS_INFO("converting triangles");
		for (size_t i=0;i<polygons.size();i++) {
			t_new.push_back(polygons[i].vertices[0]);
			t_new.push_back(polygons[i].vertices[1]);
			t_new.push_back(polygons[i].vertices[2]);
		}
		ROS_INFO("Done");
		
		if (v_new.size() != 3*cloud->points.size()) {
			ROS_ERROR("clouds have different size!");
			return false;
		}
		
		float avg_diff=0;
		for (size_t i=0;i<cloud->points.size();i++) {
			pcl::PointXYZ p = cloud->points[i];
			float x = v_new[3*i];
			float y = v_new[3*i+1];
			float z = v_new[3*i+2];
			avg_diff = sqrt(pow(p.x - x,2) + pow(p.y - y,2) + pow(p.z - z,2));
		}
		avg_diff = avg_diff / cloud->points.size();
		//std::cout << "avg diff: " << avg_diff << std::endl;
		
		for (size_t i=0;i<t_new.size();i++) {
			if ((int)polygons[i/3].vertices[i%3] != t_new[i]) {
				ROS_ERROR_STREAM("vertex screwed up at " << i << ": " << t_new[i] << " " << polygons[i/3].vertices[i%3]);
				break;
			}
		}
	}
	
	if (viewer) {
		pcl::PolygonMesh* new_mesh_to_view = new pcl::PolygonMesh();
		*new_mesh_to_view = mesh;
		
		mesh_to_view = new_mesh_to_view;
		
		/*
		viewer->setBackgroundColor (0, 0, 0);
		//pcl::visualization::PointCloudColorHandlerRGBField<ColorPoint> rgb(cloud);
		//viewer->addPointCloud<ColorPoint>(cloud, rgb, "sample cloud");
		//viewer->addPointCloud<Point> (cloud, "sample cloud");
		//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
		viewer->addPolygonMesh(mesh,"mesh");
		viewer->addCoordinateSystem(0.3);
		viewer->initCameraParameters();
		//viewer->showCloud(cloud);
		
		viewer->spinOnce();
		*/
	}
	
	return true;
}


bool service_callback(google_goggles_srvs::CreateMesh::Request& request, google_goggles_srvs::CreateMesh::Response& response) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromROSMsg(request.point_cloud, *cloud);
	
	return point_cloud_to_mesh(cloud,response.mesh);
}


void sub_callback(const sensor_msgs::PointCloud2& msg) {
	ROS_INFO("Callback!");
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromROSMsg(msg, *cloud);
	
	pcl::PolygonMesh mesh;
	point_cloud_to_mesh(cloud,mesh);
	
	if (cloud_pub) {
		ROS_INFO("Publishing cloud");
		cloud_pub->publish(mesh.cloud);
	}
	
	if (!generateGraspsService && use_bag) {
		ROS_INFO("Writing object mesh to bag file");
		if (!bag) {
			bag = new rosbag::Bag();
			bag->open(bag_filename,rosbag::bagmode::Write);
		}
		bag->write("object", msg.header.stamp, mesh);
	} else if (generateGraspsService) {
		ROS_INFO("Generating grasps...");
		
		geometry_msgs::PoseArray object_rotation_array;
		std_msgs::UInt16MultiArray num_grasps_array;
		
		geometry_msgs::PoseArray pose_array;
		pose_array.header = msg.header;
		
		google_goggles_msgs::ObjectReferenceData data;
		data.object = mesh;
		
		
		for (size_t rot_ind=0;rot_ind<object_rotations.size();rot_ind++) {
			if (rot_ind != 0) {
				ros::Duration(1).sleep();
			}
			printf("Generating for rotation %u/%u\n",(unsigned int)rot_ind+1,(unsigned int)object_rotations.size());
			graspit_srvs::GenerateGrasps::Request greq;
			
			greq.object = mesh;
			greq.object_pose.position.x = 0; greq.object_pose.position.y = 0; greq.object_pose.position.z = 0;
			greq.object_pose.orientation.x = 0; greq.object_pose.orientation.y = 0; greq.object_pose.orientation.z = 0; greq.object_pose.orientation.w = 1;
			
			Affine3f object_rotation_tf = object_rotations[rot_ind];
			
			geometry_msgs::Pose or_msg;
			Vector3f or_p = object_rotation_tf.translation();
			or_msg.position.x = or_p.x();
			or_msg.position.y = or_p.y();
			or_msg.position.z = or_p.z();
			
			Quaternionf or_q(object_rotation_tf.rotation());
			or_msg.orientation.x = or_q.x();
			or_msg.orientation.y = or_q.y();
			or_msg.orientation.z = or_q.z();
			or_msg.orientation.w = or_q.w();
			
			object_rotation_array.poses.push_back(or_msg);
			
			pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud1(new pcl::PointCloud<pcl::PointXYZ>());
			pcl::fromROSMsg(greq.object.cloud, *mesh_cloud1);
			pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud2(new pcl::PointCloud<pcl::PointXYZ>());
			pcl::transformPointCloud(*mesh_cloud1,*mesh_cloud2,object_rotation_tf);
			
			float min_z = 10000000;
			for (pcl::PointCloud<pcl::PointXYZ>::iterator itr=mesh_cloud2->begin();itr!=mesh_cloud2->end();itr++) {
				if (itr->z < min_z) {
					min_z = itr->z;
				}
			}
			
			Translation3f shift_up(Vector3f(0, 0, -min_z));
			
			pcl::transformPointCloud(*mesh_cloud2,*mesh_cloud1,Affine3f(shift_up));
			
			pcl::toROSMsg(*mesh_cloud1,greq.object.cloud);
			object_rotation_tf = shift_up * object_rotation_tf;
		
			greq.num_grasps = num_grasps;
		
			graspit_srvs::GenerateGrasps::Response gres;
			
			generateGraspsService->call(greq,gres);
			
			if (gres.grasps.empty()) {
				ROS_WARN("No grasps returned");
				return;
			}
			
			std::cout << " Num grasps " << gres.grasps.size() << std::endl;
			
			num_grasps_array.data.push_back(gres.grasps.size());
		
			//unrotate poses
			for (size_t i=0;i<gres.grasps.size();i++) {
				geometry_msgs::Pose gp = gres.grasps[i].grasp_pose;
				Affine3f gp_tf = Translation3f(gp.position.x,gp.position.y,gp.position.z)
						* Quaternionf(gp.orientation.w,gp.orientation.x,gp.orientation.y,gp.orientation.z);
				Affine3f new_gp_tf = object_rotation_tf.inverse() * gp_tf;
				//Affine3f new_gp_tf = object_rotation_tf * gp_tf;
				
				Vector3f new_p = new_gp_tf.translation();
				gp.position.x = new_p.x();
				gp.position.y = new_p.y();
				gp.position.z = new_p.z();
				
				Quaternionf new_q(new_gp_tf.rotation());
				gp.orientation.x = new_q.x();
				gp.orientation.y = new_q.y();
				gp.orientation.z = new_q.z();
				gp.orientation.w = new_q.w();
				
				gres.grasps[i].grasp_pose = gp;
			}
			
			geometry_msgs::PoseArray this_pose_array;
			this_pose_array.header = msg.header;
			this_pose_array.header.stamp = ros::Time::now();
			
			for (size_t i=0;i<gres.grasps.size();i++) {
				pose_array.poses.push_back(gres.grasps[i].grasp_pose);
				this_pose_array.poses.push_back(gres.grasps[i].grasp_pose);
			}
			
			data.grasps.insert(data.grasps.end(),gres.grasps.begin(),gres.grasps.end());
			
			if (grasps_pub) {
				ROS_INFO("Publishing grasps");
				this_pose_array.poses = pose_array.poses;
				grasps_pub->publish(this_pose_array);
			}
		}
			
		if (grasps_pub) {
			ROS_INFO("Publishing grasps");
			geometry_msgs::PoseArray this_pose_array;
			this_pose_array = pose_array;
			this_pose_array.header.stamp = ros::Time::now();
			grasps_pub->publish(this_pose_array);
		}
			
		if (use_bag) {
			ROS_INFO("Writing object mesh and grasps to bag file");
			
			if (!bag) {
				bag = new rosbag::Bag();
				bag->open(bag_filename,rosbag::bagmode::Write);
			}
			
			
			bag->write("object_ref_data",msg.header.stamp,data);
			bag->write("object_grasps",msg.header.stamp,pose_array);
			
			bag->write("object_rotations",msg.header.stamp,object_rotation_array);
			bag->write("object_num_grasps",msg.header.stamp,num_grasps_array);
			
			//std::cout << data.object.cloud.header << std::endl;
		}
		
		graspit_srvs::TestGrasps::Request treq;
		treq.object = mesh;
		treq.grasps = data.grasps;
		
		graspit_srvs::TestGrasps::Response tres;
		
		if (testGraspsService) {
		
			ros::Duration(1).sleep();
			
			ROS_INFO("Testing grasps...");
			testGraspsService->call(treq,tres);
			
			ROS_INFO("Got result!");
			std::cout << " Num grasps " << tres.qualities.size() << std::endl;
			for (size_t i=0;i<tres.qualities.size();i++) {
				std::cout << "Grasp #" << i << " quality: " << tres.qualities[i] << std::endl;
			}
			
			if (use_bag) {
				ROS_INFO("Writing object mesh and tested grasps to bag file");
				if (!bag) {
					bag = new rosbag::Bag();
					bag->open(bag_filename,rosbag::bagmode::Write);
				}
				google_goggles_msgs::ObjectReferenceData data;
				data.object = mesh;
				data.grasps = tres.grasps;
				bag->write("object_grasps_tested",msg.header.stamp,data);
				//std::cout << data.grasps[0] << std::endl;
			}
		}
	}
	
	/*if (bag) {
		ROS_INFO("Shutting down");
		bag->close();
		ros::shutdown();
	}
	*/
}

#ifndef BENK_STRINGIFY
#define BENK_STRINGIFY(X) BENK_STRINGIFY_HELPER(X)
#define BENK_STRINGIFY_HELPER(X) #X
#endif

#define SAVE_OPT_ARG(ARGNAME,VARNAME) \
if (arg == "--" BENK_STRINGIFY(ARGNAME)) { \
	arg_ind++; \
	std::stringstream ss;\
	ss << argv[arg_ind]; \
	ss >> VARNAME; \
}

#define PRINT_VALUE(VARNAME) std::cout << BENK_STRINGIFY(VARNAME) << ": " << VARNAME << std::endl

int main(int argc, char* argv[]) {
	ros::init(argc,argv,"point_cloud_to_mesh_repub",ros::init_options::AnonymousName);
	
	bool use_viewer = true;
	bool viewer_axes = true;
	
	bool use_service = true;
	bool generate_grasps = false;
	bool test_grasps = false;
	bool save = false;
	std::string save_prefix = "object_data";
	
	
	bool publish_cloud = false;
	bool publish_grasps = false;
	
	object_rotations.push_back(Affine3f::Identity());
	
	
	std::vector<std::string> args;
	for (int arg_ind = 0;arg_ind < argc; arg_ind++) {
		std::string arg(argv[arg_ind]);
		
		if (arg[0] != '-') {
			args.push_back(arg);
			continue;
		}
		
		if (arg == "--base-rotation" || arg == "--base-tb") {
			float yaw,pitch,roll;
			std::stringstream ss1;
			arg_ind++;
			ss1 << argv[arg_ind];
			ss1 >> yaw;
			yaw = yaw * M_PI / 180;
			
			std::stringstream ss2;
			arg_ind++;
			ss2 << argv[arg_ind];
			ss2 >> pitch;
			pitch = pitch * M_PI / 180;
			
			std::stringstream ss3;
			arg_ind++;
			ss3 << argv[arg_ind];
			ss3 >> roll;
			roll = roll * M_PI / 180;
			
			AngleAxisf yaw_tf(yaw, Vector3f(0,0,1));
			AngleAxisf pitch_tf(pitch, Vector3f(0,1,0));
			AngleAxisf roll_tf(roll, Vector3f(1,0,0));
			
			base_tf = roll_tf * pitch_tf * yaw_tf;
			
			continue;
		}
		
		if (arg == "--object-rotation" || arg == "--tb"
				|| arg == "--add-object-rotation" || arg == "--add-tb") {
			if (arg == "--object-rotation" || arg == "--tb") {
				object_rotations.clear();
			}
			float yaw,pitch,roll;
			std::stringstream ss1;
			arg_ind++;
			ss1 << argv[arg_ind];
			ss1 >> yaw;
			yaw = yaw * M_PI / 180;
			
			std::stringstream ss2;
			arg_ind++;
			ss2 << argv[arg_ind];
			ss2 >> pitch;
			pitch = pitch * M_PI / 180;
			
			std::stringstream ss3;
			arg_ind++;
			ss3 << argv[arg_ind];
			ss3 >> roll;
			roll = roll * M_PI / 180;
			
			AngleAxisf yaw_tf(yaw, Vector3f(0,0,1));
			AngleAxisf pitch_tf(pitch, Vector3f(0,1,0));
			AngleAxisf roll_tf(roll, Vector3f(1,0,0));
			
			Affine3f the_tf;
			the_tf = roll_tf * pitch_tf * yaw_tf;
			object_rotations.push_back(the_tf);
			
			continue;
		}
		
		use_viewer &= (arg != "--no-viz");
		viewer_axes &= (arg != "--axes-off");
		
		
		use_service &= (arg != "--no-service");
		
		generate_grasps |= (arg == "--generate-grasps");
		generate_grasps |= (arg == "-g");
		
		SAVE_OPT_ARG(num-grasps,num_grasps);
		
		test_grasps |= (arg == "--test-grasps");
		
		save |= (arg == "--save");
		if (arg == "--save-as") {
			save = true;
			arg_ind++;
			save_prefix = argv[arg_ind];
		}
		
		publish_cloud |= (arg == "--publish-cloud");
		publish_grasps |= (arg == "--publish-grasps");
		
		publish_cloud |= (arg == "--publish");
		publish_grasps |= (arg == "--publish");
		
		SAVE_OPT_ARG(mls-polynomial-fit,mls_polynomialFit);
		SAVE_OPT_ARG(mls-search-radius,mls_searchRadius);
		SAVE_OPT_ARG(normal-k,normal_k);
		SAVE_OPT_ARG(mesh-mu,mesh_mu);
		SAVE_OPT_ARG(mesh-maximum-nearest-neighbors,mesh_maximumNearestNeighbors);
		if (arg == "--mesh-maximum-surface-angle") {
			arg_ind++;
			std::stringstream ss;
			ss << argv[arg_ind];
			ss >> mesh_maximumSurfaceAngle;
			mesh_maximumSurfaceAngle = mesh_maximumSurfaceAngle * M_PI / 180;
		}
		if (arg == "--mesh-maximum-surface-angle") {
			arg_ind++;
			std::stringstream ss;
			ss << argv[arg_ind];
			ss >> mesh_maximumSurfaceAngle;
			mesh_maximumSurfaceAngle = mesh_maximumSurfaceAngle * M_PI / 180;
		}
		if (arg == "--mesh-minimum-angle") {
			arg_ind++;
			std::stringstream ss;
			ss << argv[arg_ind];
			ss >> mesh_minimumAngle;
			mesh_minimumAngle = mesh_minimumAngle * M_PI / 180;
		}
		
		if (arg == "--mesh-maximum-angle") {
			arg_ind++;
			std::stringstream ss;
			ss << argv[arg_ind];
			ss >> mesh_maximumAngle;
			mesh_maximumAngle = mesh_maximumAngle * M_PI / 180;
		}
		//SAVE_OPT_ARG(mesh-maximumSurfaceAngle,mesh_maximumSurfaceAngle);
		//SAVE_OPT_ARG(mesh-minimumAngle,mesh_minimumAngle);
		//SAVE_OPT_ARG(mesh-maximumAngle,mesh_maximumAngle);
		SAVE_OPT_ARG(mesh-normalConsistency,mesh_normalConsistency);
	}
	
	PRINT_VALUE(mls_polynomialFit);
	PRINT_VALUE(mls_searchRadius);
	PRINT_VALUE(normal_k);

	PRINT_VALUE(mesh_searchRadius);

	// Set typical values for the parameters
	PRINT_VALUE(mesh_mu);
	PRINT_VALUE(mesh_maximumNearestNeighbors);
	std::cout << "mesh_maximumSurfaceAngle: " << mesh_maximumSurfaceAngle * 180 / M_PI << std::endl;
	std::cout << "mesh_minimumAngle: " << mesh_minimumAngle * 180 / M_PI << std::endl;
	std::cout << "mesh_maximumAngle: " << mesh_maximumAngle * 180 / M_PI << std::endl;
	PRINT_VALUE(mesh_normalConsistency);
	
	ros::CallbackQueue* cb_queue = new ros::CallbackQueue();

	ros::NodeHandle nh("");
	nh.setCallbackQueue(cb_queue);
	
	if (use_viewer) {
		viewer = new pcl::visualization::PCLVisualizer("3D Viewer");
		viewer->setBackgroundColor (0, 0, 0);
		
		if (viewer_axes) {
			viewer->addCoordinateSystem(0.3);
		}
		viewer->initCameraParameters();
	}
	
	ros::Duration service_wait_timeout(10);
	
	ros::ServiceServer* service = 0;
	if (use_service) {
		service = new ros::ServiceServer(nh.advertiseService("create_mesh", service_callback));
	}
	
	if (generate_grasps || test_grasps) {
		generateGraspsService = new ros::ServiceClient(nh.serviceClient<graspit_srvs::GenerateGrasps>("generate_grasps"));
		if (!generateGraspsService->waitForExistence(service_wait_timeout)) {
			ROS_FATAL("Generate grasp service not found!");
			return 1;
		}
		if (test_grasps) {
			testGraspsService = new ros::ServiceClient(nh.serviceClient<graspit_srvs::TestGrasps>("test_grasps"));
			if (!testGraspsService->waitForExistence(service_wait_timeout)) {
				ROS_FATAL("Test grasp service not found!");
			return 1;
			}
		}
	}
	
	if (save) {
		time_t rawtime;
		struct tm* timeinfo;
		char buffer[80];

		time(&rawtime);
		timeinfo = localtime(&rawtime);

		strftime(buffer,80,"%Y-%m-%d-T%H-%M-%S",timeinfo);
		std::string time_string(buffer);
		bag_filename = save_prefix + "_" + time_string + ".bag";
		
		use_bag = true;
		ROS_INFO("Writing object data to %s",bag_filename.c_str());
	}
	
	if (publish_cloud) {
		cloud_pub = new ros::Publisher(nh.advertise<sensor_msgs::PointCloud2>("object_mesh",1,true));
	}
	
	if (publish_grasps) {
		grasps_pub = new ros::Publisher(nh.advertise<geometry_msgs::PoseArray>("object_grasps",1,true));
	}
	
	ros::Subscriber sub = nh.subscribe("cloud_pcd", 1, &sub_callback);
	
	ROS_INFO("Ready");
	
	ros::AsyncSpinner spinner(1,cb_queue);
	spinner.start();
	
	ros::Rate rate(5);
	while (!viewer->wasStopped() && ros::ok()) {
		//ros::spinOnce();
		
		if (mesh_to_view) {
			pcl::PolygonMesh* m = mesh_to_view;
			std::stringstream ss;
			ss << "mesh" << rand();
			std::string mesh_name;
			ss >> mesh_name;
			viewer->addPolygonMesh(*m,mesh_name);
			mesh_to_view = 0;
		}
		
		viewer->spinOnce(100);
		rate.sleep();
	}
	
	if (bag) {
		ROS_INFO_STREAM("Closing bag " << bag_filename);
		bag->close();
	} else {
		std::cout << "Shutting down" << std::endl;
	}
}