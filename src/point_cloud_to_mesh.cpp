#include <ros/ros.h>
#include <iostream>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PolygonMesh.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl/common/common_headers.h>
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/foreach.hpp>
#include <sstream>
#include <vector>
#include <math.h>

#include <graspit_srvs/GenerateGrasps.h>
#include <graspit_srvs/TestGrasps.h>

#include "google_goggles/CreateMesh.h"

using namespace std;
using namespace Eigen;

static pcl::visualization::PCLVisualizer* viewer = 0;
static ros::ServiceClient* generateGraspsService = 0;
static ros::ServiceClient* testGraspsService = 0;

bool point_cloud_to_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PolygonMesh& mesh) {
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
	pcl::PointCloud<pcl::PointXYZ> mls_cloud;//(new pcl::PointCloud<Point>);

	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	
	//mls.setComputeNormals (true);
	//mls.setOutputNormals(cloud_with_normals);

	// Set parameters
	mls.setInputCloud (cloud);
	mls.setPolynomialFit (true);
	mls.setSearchMethod (tree0);
	mls.setSearchRadius (0.03);
	
	// Reconstruct
	//mls.process (*cloud_with_normals);
	mls.reconstruct(mls_cloud);
	
	*cloud = mls_cloud;
	
	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZ>);
	tree1->setInputCloud (cloud);
	n.setInputCloud (cloud);
	n.setSearchMethod (tree1);
	n.setKSearch (20);
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
	gp3.setSearchRadius (0.025);

	// Set typical values for the parameters
	gp3.setMu (2.5);
	gp3.setMaximumNearestNeighbors (100);
	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud (cloud_with_normals);
	gp3.setSearchMethod (tree2);
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
		std::cout << "avg diff: " << avg_diff << std::endl;
		
		for (size_t i=0;i<t_new.size();i++) {
			if ((int)polygons[i/3].vertices[i%3] != t_new[i]) {
				ROS_ERROR_STREAM("vertex screwed up at " << i << ": " << t_new[i] << " " << polygons[i/3].vertices[i%3]);
				break;
			}
		}
	}
	
	if (viewer) {
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
	}
	
	return true;
}


bool service_callback(google_goggles::CreateMesh::Request& request, google_goggles::CreateMesh::Response& response) {
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
	
	if (generateGraspsService) {
		graspit_srvs::GenerateGrasps::Request greq;
		greq.object = mesh;
		greq.num_grasps = 7;
	
		graspit_srvs::GenerateGrasps::Response gres;
		
		ROS_INFO("Generating grasps...");
		generateGraspsService->call(greq,gres);
		
		std::cout << " Num grasps " << gres.grasps.size() << std::endl;
		
		graspit_srvs::TestGrasps::Request treq;
		treq.object = mesh;
		treq.grasps = gres.grasps;
		
		graspit_srvs::TestGrasps::Response tres;
		
		ros::Duration(1).sleep();
		
		ROS_INFO("Testing grasps...");
		testGraspsService->call(treq,tres);
		
		ROS_INFO("Got result!");
		for (size_t i=0;i<tres.qualities.size();i++) {
			std::cout << "Grasp #" << i << " quality: " << tres.qualities[i] << std::endl;
		}
	}
}


int main(int argc, char* argv[]) {
	ros::init(argc,argv,"point_cloud_to_mesh_repub");
	
	bool use_viewer = true;
	bool use_service = true;
	bool test = false;
	
	
	for (int arg_ind = 0;arg_ind < argc; arg_ind++) {
		std::string arg(argv[arg_ind]);
		if (arg == "--no-viz") {
			use_viewer = false;
		} else if (arg == "--no-service") {
			use_service = false;
		} else if (arg == "--test") {
			test = true;
		}
	}

	ros::NodeHandle nh("");
	
	if (use_viewer) {
		viewer = new pcl::visualization::PCLVisualizer("3D Viewer");
	}
	
	ros::ServiceServer* service = 0;
	if (use_service) {
		service = new ros::ServiceServer(nh.advertiseService("create_mesh", service_callback));
	}
	
	if (test) {
		generateGraspsService = new ros::ServiceClient(nh.serviceClient<graspit_srvs::GenerateGrasps>("graspit/generate_grasps"));
		testGraspsService = new ros::ServiceClient(nh.serviceClient<graspit_srvs::TestGrasps>("graspit/test_grasps"));
	}	
	
	ros::Subscriber sub = nh.subscribe("cloud_pcd", 1, &sub_callback);
	
	ROS_INFO("Ready");
	
	ros::Rate rate(5);
	while (!viewer->wasStopped() && ros::ok()) {
		ros::spinOnce();
		viewer->spinOnce(100);
		rate.sleep();
	}
}