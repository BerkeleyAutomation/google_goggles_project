#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "utils_pcl.h"
#include <pcl/Vertices.h>
#include <opencv2/core/core.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>


std::vector< std::vector<int> > findClusters(ColorCloudPtr cloud, float tol=.02, float minSize=100);
ColorCloudPtr downsampleCloud(const ColorCloudPtr in, float sz);
void downsampleCloudInPlace(ColorCloudPtr in, float sz);
ColorCloudPtr removeOutliers(const ColorCloudPtr in, float thresh=1, int k=15);
ColorCloudPtr findConcaveHull(ColorCloudPtr in, std::vector<pcl::Vertices>& polygons);
ColorCloudPtr findConvexHull(ColorCloudPtr in, std::vector<pcl::Vertices>& polygons);
ColorCloudPtr cropToHull(const ColorCloudPtr in, ColorCloudPtr hull, std::vector<pcl::Vertices>& polygons);
ColorCloudPtr projectOntoPlane(const ColorCloudPtr in, Eigen::Vector4f& coeffs);
ColorCloudPtr filterZ(ColorCloudPtr in, float low, float high);
ColorCloudPtr filterY(ColorCloudPtr in, float low, float high);
ColorCloudPtr filterX(ColorCloudPtr in, float low, float high);
ColorCloudPtr filterXYZ(ColorCloudPtr in, float xlo, float xhi, float ylo, float yhi, float zlo, float zhi);
Eigen::VectorXf getCircle(ColorCloudPtr cloud);
Eigen::VectorXf getEnclosingCircle(ColorCloudPtr cloud);
ColorCloudPtr getBiggestCluster(ColorCloudPtr in, float tol);
ColorCloudPtr maskCloud(const ColorCloudPtr in, const cv::Mat& mask);
ColorCloudPtr maskCloud(const ColorCloudPtr in, const VectorXb& mask);
ColorCloudPtr removeZRange(const ColorCloudPtr in, float minZ, float maxZ);
void labelCloud(ColorCloudPtr in, const cv::Mat& labels);
ColorCloudPtr hueFilter(ColorCloudPtr in, uint8_t minHue, uint8_t maxHue, uint8_t minSat);

void icp_combine(ColorCloudPtr curr,ColorCloudPtr in,int downsampleSize = -1);
void combine(ColorCloudPtr curr,ColorCloudPtr in);
