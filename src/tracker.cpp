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
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace Eigen;

void TabletopTracker::setLatest(ColorCloudPtr cloud) {
	latest_cloud = cloud;
};

void TabletopTracker::updateCloud() {
	map_transformed_cloud = transformPointCloud1(downsampleCloud(latest_cloud, TableConfig::VOXEL_SIZE), map_transform);
}

void TabletopTracker::updateTable() {
	ROS_DEBUG_STREAM(map_transform.matrix());
	table_height = getTableHeight(map_transformed_cloud);
	ROS_INFO_STREAM("table_height " << table_height);
	ColorCloudPtr in_table = getTablePoints(map_transformed_cloud, table_height);
	in_table = getBiggestCluster(in_table, TableConfig::TABLE_CLUSTERING_TOLERANCE);
	table_hull = findConvexHull(in_table, table_polygons);
	getTableBounds(in_table, xmin, xmax, ymin, ymax);
	//	fixZ(table_hull, table_height);

}

std::vector<int> arange(int lo, int hi, int step) {
	int n = (hi - lo) / step;
	std::vector<int> out(n);
	for (int i=0; i < n; i++) out[i] = lo + step*i;
	return out;
}


vector<ColorCloudPtr> mergeOverlappingCircles(vector<ColorCloudPtr> clu_list) {
	int n_clu = clu_list.size();
	vector<VectorXf> params_list(n_clu);
	vector<int> merge_to(n_clu);

	merge_to = arange(0, n_clu, 1);

	for (int i=0; i < n_clu; i++) {
		params_list[i] = getEnclosingCircle(clu_list[i]);
	}

	for (int i=0; i < n_clu; i++) {
		for (int j=i+1; j < n_clu; j++) {
				float dist = (params_list[i].block(0,0,2,1) - params_list[j].block(0,0,2,1)).norm();
				float rad_sum = params_list[i][3] + params_list[j][3];
				if (dist < rad_sum) merge_to[j] = merge_to[i];
			}
	}


	vector<ColorCloudPtr> merges(n_clu);
	for (int i = 0; i < n_clu; i++) {
		if(!merges[merge_to[i]]) merges[merge_to[i]].reset(new ColorCloud());
		*merges[merge_to[i]] += *clu_list[i];
	}


	vector<ColorCloudPtr> final;
	BOOST_FOREACH(ColorCloudPtr cloud, merges) if (cloud) final.push_back(cloud);

	return final;

}

vector<VectorXf> getCircleParams(vector<ColorCloudPtr> clu_list) {
	vector<VectorXf> params_list(clu_list.size());
	for (size_t i=0; i < clu_list.size(); i++) {
		params_list[i] = getEnclosingCircle(clu_list[i]);
	}
	return params_list;
}


void TabletopTracker::updateClusters() {
	ColorCloudPtr on_table = getPointsOnTableHull(map_transformed_cloud, table_hull, table_polygons, table_height+TableConfig::ABOVE_TABLE_CUTOFF);
	
	//ColorCloudPtr on_table = filterXYZ(map_transformed_cloud, xmin, xmax, ymin, ymax, table_height+TableConfig::ABOVE_TABLE_CUTOFF, 1000);
	
	if (on_table->size() < 30) {
		stringstream ss;
		ss << "not enough points (" << on_table->size() << ") on table";
		throw runtime_error(ss.str());
	}
	//cout << "on table: " << on_table->size() << endl;
	
// 	latest_cluster = on_table;
// 	if (!combined_cluster) {
// 		combined_cluster = on_table;
// 	} else {
// 		icp_combine(combined_cluster,on_table,TableConfig::VOXEL_SIZE);
// 	}
	
	vector< vector<int> > cluster_inds = findClusters(on_table,TableConfig::OBJECT_CLUSTERING_TOLERANCE,TableConfig::OBJECT_CLUSTER_MIN_SIZE);
	if (cluster_inds.size() == 0) throw runtime_error("no reasonably big clusters found on table");	

	
	clusters.clear();
	BOOST_FOREACH(vector<int>& inds, cluster_inds) {
		ColorCloudPtr cluster = extractInds(on_table, inds);
		float minHeight = 10000;
		float maxHeight = -10000;
		BOOST_FOREACH(ColorPoint& pt, cluster->points) {
			float height = pt.z - table_height;
			if (height > maxHeight) {
				maxHeight = height;
			} else if (height < minHeight) {
				minHeight = height;
			}
		}
		//ROS_DEBUG_STREAM(fabs(maxHeight-minHeight));
		if (fabs(maxHeight-minHeight) > 0.023) {
			clusters.push_back(cluster);
		}
	}
	
	clusters = mergeOverlappingCircles(clusters);
	
	size_t min_ind = -1;
	float min_x_dist = 100000;
	for (size_t i=0;i<clusters.size();i++) {
		Eigen::Vector4f centroid;
		float dist;
		if (!combined_cluster) {
			ColorCloudPtr base_cloud = transformPointCloud1(clusters[i], map_to_base_transform);
			pcl::compute3DCentroid(*(clusters[i]),centroid);
			dist = fabs(centroid[0]);
		} else {
			pcl::compute3DCentroid(*(clusters[i]),centroid);
			dist = sqrt(pow(centroid[0]-combinedClusterCentroid[0],2)+pow(centroid[1]-combinedClusterCentroid[1],2));
		}
		if (dist < min_x_dist) {
			min_x_dist = dist;
			min_ind = i;
		}
		float minHeight = 10000;
		float maxHeight = -10000;
		BOOST_FOREACH(ColorPoint& pt, clusters[i]->points) {
			float height = pt.z - table_height;
			if (height > maxHeight) {
				maxHeight = height;
			} else if (height < minHeight) {
				minHeight = height;
			}
		}
		//std::cout << "cluster height: " << maxHeight - minHeight << " (" << minHeight << "," << maxHeight << ")" << std::endl;
		//std::cout << "cluster centroid: (" << centroid[0] << "," << centroid[1] << "," << centroid[2] << ") " << dist << std::endl;
	}

	latest_cluster = clusters[min_ind];
	if (!combined_cluster || mode == TRACK) {
		combined_cluster = latest_cluster;
	} else {
		
		icp_combine(combined_cluster,latest_cluster,TableConfig::VOXEL_SIZE);
		//*(combined_cluster) += *(latest_cluster);
		//combined_cluster = downsampleCloud(combined_cluster, TableConfig::VOXEL_SIZE);
		
	}
	pcl::compute3DCentroid(*(combined_cluster),combinedClusterCentroid);
}

void TabletopTracker::updateCylinders() {

	MatrixXf new_circle_centers(clusters.size(), 2);
	cylinder_params = getCircleParams(clusters);
	for (size_t i=0; i < clusters.size(); i++) {
		new_circle_centers(i,0) = cylinder_params[i](0);
		new_circle_centers(i,1) = cylinder_params[i](1);
	}

	if (ids.size() == 0) { // first time 
		ids = arange(0, clusters.size(), 1);
		smallest_unused_id = ids.size();
	} else {
		MatrixXf dists = pairwiseSquareDist(new_circle_centers, circle_centers).array().sqrt();
		vector<int> new2old = argminAlongRows(dists);
		vector<int> newids(clusters.size());
		for (size_t i=0; i < clusters.size(); i++) {
			if (dists(i, new2old[i]) < TableConfig::OBJECT_MATCH_TOLERANCE)	{
				newids[i] = ids[new2old[i]];
			} else {
				ROS_INFO("lost a cluster!");
				newids[i] = smallest_unused_id;
				smallest_unused_id++;
			}
		}
		ids = newids;
	}
	circle_centers = new_circle_centers;

	stringstream ss;
	ss << "ids: ";
	BOOST_FOREACH(int i, ids) ss << i << " ";
	//ROS_INFO_STREAM(ss.str());
}


void TabletopTracker::updateAll() {

	updateTransform();
	updateCloud();
	if (!initialized) {
		updateTable();
		updateClusters();
		updateCylinders();
		initialized = true;
	}
	else {
		updateClusters();
		updateCylinders();
	}

}

void TabletopTracker::reset() {
	clusters = vector<ColorCloudPtr>();
	ids = std::vector<int>();
	smallest_unused_id = 0;
	circle_centers = Eigen::MatrixXf();
}

void TabletopTracker::write(const std::string& filename) {
	ColorCloudPtr base_transformed_cloud = transformPointCloud1(combined_cluster, map_to_base_transform);
	std::cout << "Writing to " << filename << std::endl;
	pcl::io::savePCDFileASCII(filename, *base_transformed_cloud);
}
