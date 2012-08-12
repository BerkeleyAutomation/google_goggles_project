#pragma once
#include "utils_pcl.h"
#include <pcl/Vertices.h>

class TabletopTracker {
public:
	enum Mode { TRACK, COMBINE };
	Mode mode;
	
	bool initialized;

	Eigen::Affine3f map_transform;
	Eigen::Affine3f map_to_base_transform;
	
	ColorCloudPtr latest_cloud;
	ColorCloudPtr map_transformed_cloud;

	ColorCloudPtr table_hull;
	float xmin, xmax, ymin, ymax;
	float table_height;
	std::vector<pcl::Vertices> table_polygons;

	ColorCloudPtr latest_cluster;
	ColorCloudPtr combined_cluster;
	Eigen::Vector4f combinedClusterCentroid;
	Eigen::Affine3f combined_transform;

	std::vector<ColorCloudPtr> clusters;
	std::vector<int> ids;
	int smallest_unused_id;
	std::vector<Eigen::VectorXf> cylinder_params;
	Eigen::MatrixXf circle_centers;

	TabletopTracker(Mode theMode=TRACK) : mode(theMode), initialized(false) {}
	void setLatest(ColorCloudPtr);
	virtual void updateTransform() = 0; // set transform using latest cloud
	void updateCloud(); // update transformed_cloud using transform
	void updateTable(); // find table
	void updateClusters();
	void updateCylinders();
	virtual void updateAll();
	virtual void reset();
	
	void write(const std::string& filename);
};

