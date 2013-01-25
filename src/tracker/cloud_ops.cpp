#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "cloud_ops.h"
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/surface/convex_hull.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <algorithm>

typedef ColorPoint PointT;

using namespace std;
using namespace Eigen;
using namespace pcl;

CloudPtr stripColor(ColorCloudPtr in) {
	CloudPtr out(new Cloud());
	BOOST_FOREACH(ColorPoint& pt, in->points) out->push_back(Point(pt.x, pt.y, pt.z));
	out->width = in->width;
	out->height = in->height;
	out->is_dense = in->is_dense;
	return out;
}

vector< vector<int> > findClusters(ColorCloudPtr cloud, float tol, float minSize) {
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<Point> ec;
	CloudPtr cloud2 = stripColor(cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<Point>);
	ec.setClusterTolerance (tol);
	ec.setMinClusterSize (minSize);
	ec.setMaxClusterSize (2500000);
	ec.setSearchMethod (tree);
	ec.setInputCloud(cloud2);
	ec.extract (cluster_indices);

	vector< vector<int> > out;

	for (size_t i=0; i < cluster_indices.size(); i++) {
		out.push_back(cluster_indices[i].indices);
	}
	return out;
}

ColorCloudPtr downsampleCloud(const ColorCloudPtr in, float sz) {
	pcl::PointCloud<ColorPoint>::Ptr out(new pcl::PointCloud<ColorPoint>());
	pcl::VoxelGrid<ColorPoint> vg;
	vg.setInputCloud(in);
	vg.setLeafSize(sz,sz,sz);
	vg.filter(*out);
	return out;
}

void downsampleCloudInPlace(ColorCloudPtr in, float sz) {
	pcl::VoxelGrid<ColorPoint> vg;
	vg.setInputCloud(in);
	vg.setLeafSize(sz,sz,sz);
	vg.filter(*in);
}

void downsampleCloudInPlace(CloudPtr in, float sz) {
	pcl::VoxelGrid<Point> vg;
	vg.setInputCloud(in);
	vg.setLeafSize(sz,sz,sz);
	vg.filter(*in);
}

ColorCloudPtr removeOutliers(const ColorCloudPtr in, float thresh, int k) {
	ColorCloudPtr out(new ColorCloud());
	pcl::StatisticalOutlierRemoval<ColorPoint> sor;
	sor.setInputCloud (in);
	sor.setMeanK (k);
	sor.setStddevMulThresh (thresh);
	sor.filter (*out);
	cout << "removeOutliers: removed " << (in->size() - out->size()) << " of " << in->size() << endl;
	return out;
}

ColorCloudPtr projectOntoPlane(const ColorCloudPtr in, Eigen::Vector4f& coeffs) {
	ColorCloudPtr cloud_projected (new ColorCloud());

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	coefficients->values.resize (4);
	coefficients->values[0] = coefficients->values[1] = 0;
	coefficients->values[2] = 1.0;
	coefficients->values[3] = 0;

	// Create the filtering object
	pcl::ProjectInliers<ColorPoint> proj;
	proj.setModelType (pcl::SACMODEL_PLANE);
	proj.setInputCloud (in);
	proj.setModelCoefficients (coefficients);
	proj.filter (*cloud_projected);
	return cloud_projected;
}

ColorCloudPtr findConcaveHull(ColorCloudPtr in, std::vector<pcl::Vertices>& polygons) {
	ColorCloudPtr out(new ColorCloud());
	pcl::ConcaveHull<PointT> chull;
	chull.setInputCloud (in);
	chull.setAlpha (0.1);
	chull.reconstruct (*out, polygons);
	return out;
}

ColorCloudPtr findConvexHull(ColorCloudPtr in, std::vector<pcl::Vertices>& polygons) {
	ColorCloudPtr out(new ColorCloud());
	pcl::ConvexHull<PointT> chull;
	chull.setInputCloud (in);
	chull.setDimension(2);
	chull.reconstruct (*out, polygons);
	return out;
}

ColorCloudPtr cropToHull(const ColorCloudPtr in, ColorCloudPtr hull_cloud, std::vector<pcl::Vertices>& polygons) {
	ColorCloudPtr out(new ColorCloud());
	pcl::CropHull<PointT> crop_filter;
	crop_filter.setInputCloud (in);
	crop_filter.setHullCloud (hull_cloud);
	crop_filter.setHullIndices (polygons);
	crop_filter.setDim (2);
	crop_filter.filter (*out);
	return out;
}

ColorCloudPtr filterX(ColorCloudPtr in, float low, float high) {
	pcl::PassThrough<ColorPoint> pass;
	pass.setInputCloud (in);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (low, high);

	ColorCloudPtr out(new ColorCloud());
	pass.filter (*out);
	return out;
}
ColorCloudPtr filterY(ColorCloudPtr in, float low, float high) {
	pcl::PassThrough<ColorPoint> pass;
	pass.setInputCloud (in);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (low, high);

	ColorCloudPtr out(new ColorCloud());
	pass.filter (*out);
	return out;
}
ColorCloudPtr filterZ(ColorCloudPtr in, float low, float high) {
	pcl::PassThrough<ColorPoint> pass;
	pass.setInputCloud (in);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (low, high);

	ColorCloudPtr out(new ColorCloud());
	pass.filter (*out);
	return out;
}
ColorCloudPtr filterXYZ(ColorCloudPtr in, float xlo, float xhi, float ylo, float yhi, float zlo, float zhi) {
	ColorCloudPtr out(new ColorCloud());
	BOOST_FOREACH(ColorPoint& pt, in->points) {
		if (
				pt.x >= xlo && pt.x <= xhi &&
				pt.y >= ylo && pt.y <= yhi &&
				pt.z >= zlo && pt.z <= zhi
				) {
			out->push_back(pt);
		}
	}
	return out;
}



VectorXf getCircle(ColorCloudPtr cloud) {
	ColorCloudPtr cloud_hull (new ColorCloud());
	pcl::ConvexHull<ColorPoint> chull;
	chull.setInputCloud (cloud);
	chull.setDimension(2);
	chull.reconstruct (*cloud_hull);

	boost::shared_ptr<pcl::SampleConsensusModelCircle2D<ColorPoint> > model(new pcl::SampleConsensusModelCircle2D<ColorPoint>(cloud_hull));
	pcl::RandomSampleConsensus<ColorPoint> sac(model, .02);
	/*bool result = */sac.computeModel(2);
	VectorXf zs = toEigenMatrix(cloud_hull).col(2);
	Eigen::VectorXf xyr;
	sac.getModelCoefficients (xyr);

	VectorXf coeffs(7);
	coeffs(0) = xyr(0);
	coeffs(1) = xyr(1);
	coeffs(2) = zs.minCoeff();
	coeffs(3) = 0;
	coeffs(4) = 0;
	coeffs(5) = zs.maxCoeff() - zs.minCoeff();
	coeffs(6) = xyr(2);

	return coeffs;

}

VectorXf getEnclosingCircle(ColorCloudPtr cloud) {

	MatrixXf xyz = toEigenMatrix(cloud);
	VectorXf zs = xyz.col(2);

	vector<cv::Point2f> points;
	for (int i=0; i < xyz.rows(); i++) {
		points.push_back(cv::Point2f(1000*xyz(i,0), 1000*xyz(i,1)));
	}
	cv::Mat mat(points);

	cv::Point2f center;
	float radius;
	cv::minEnclosingCircle(mat, center, radius);

	VectorXf coeffs(5);
	coeffs(0) = center.x/1000;
	coeffs(1) = center.y/1000;
	coeffs(2) = (zs.minCoeff() + zs.maxCoeff())/2;
	coeffs(3) = radius/1000;
	coeffs(4) = (zs.maxCoeff() - zs.minCoeff())/2;

	return coeffs;

}


ColorCloudPtr getBiggestCluster(ColorCloudPtr in, float tol) {

	vector< vector<int> > cluster_inds = findClusters(in, tol, 10);
	if (cluster_inds.size() == 0) throw runtime_error("didn't find any clusters");
	VectorXi sizes(cluster_inds.size());
	for (size_t i=0; i < cluster_inds.size(); i++) sizes(i) = cluster_inds[i].size();
	int iBest;
	sizes.maxCoeff(&iBest);
	//cout << "sizes: " << sizes.transpose() << endl;
	//cout << iBest << endl;
	return extractInds(in, cluster_inds[iBest]);

}


ColorCloudPtr maskCloud(const ColorCloudPtr in, const cv::Mat& mask) {
	assert(mask.elemSize() == 1);
	assert(mask.rows == (int)in->height);
	assert(mask.cols == (int)in->width);

	boost::shared_ptr< vector<int> > indicesPtr(new vector<int>());

	cv::MatConstIterator_<bool> it = mask.begin<bool>(), it_end = mask.end<bool>();
	for (int i=0; it != it_end; ++it, ++i) {
		if (*it > 0) indicesPtr->push_back(i);
	}

	ColorCloudPtr out(new ColorCloud());
	pcl::ExtractIndices<ColorPoint> ei;
	ei.setNegative(false);
	ei.setInputCloud(in);
	ei.setIndices(indicesPtr);
	ei.filter(*out);
	return out;
}


ColorCloudPtr maskCloud(const ColorCloudPtr in, const VectorXb& mask) {

	ColorCloudPtr out(new ColorCloud());
	int nOut = mask.sum();
	out->reserve(nOut);
	out->header=in->header;
	out->width = nOut;
	out->height = 1;
	out->is_dense = false;

	int i = 0;
	BOOST_FOREACH(const ColorPoint& pt, in->points) {
		if (mask(i)) out->push_back(pt);
		++i;
	}

	return out;
}

void labelCloud(ColorCloudPtr in, const cv::Mat& labels) {
	std::cerr << "labelCloud() doesn't work no more" << std::endl;
	ColorCloudPtr out(new ColorCloud());
	MatrixXi uv = xyz2uv(toEigenMatrix(in));
	for (size_t i=0; i < in->size(); i++) {
		//in->points[i]._unused = labels.at<uint8_t>(uv(i,0), uv(i,1));
	}
}

#include <opencv2/highgui/highgui.hpp>
ColorCloudPtr hueFilter(const ColorCloudPtr in, uint8_t minHue, uint8_t maxHue, uint8_t minSat) {
	MatrixXu bgr = toBGR(in);
	//size_t nPts = in->size();
	cv::Mat cvmat(in->height,in->width, CV_8UC3, bgr.data());
	cv::cvtColor(cvmat, cvmat, CV_BGR2HSV);
	vector<cv::Mat> hsvChannels;
	cv::split(cvmat, hsvChannels);

	cv::Mat& h = hsvChannels[0];
	cv::Mat& s = hsvChannels[1];
	cv::Mat& l = hsvChannels[2];
	cv::Mat hueMask = (minHue < maxHue) ?
		(h > minHue) & (h < maxHue) :
		(h > minHue) | (h < maxHue);
	cv::Mat satMask = (s > minSat);
	cv::Mat lumMask = (l > 100);
	cv::Mat mask = hueMask & satMask;
	//cv::imwrite("/tmp/blah.bmp", mask);
	return maskCloud(in, hueMask & satMask & lumMask);
}


void icp_combine(ColorCloudPtr curr,ColorCloudPtr in,int downsampleSize) {
	pcl::IterativeClosestPoint<ColorPoint, ColorPoint> icp;
	icp.setInputCloud(in);
	icp.setInputTarget(curr);
	ColorCloudPtr final(new ColorCloud());
	std::cout << "aligning..." << std::endl;
	icp.align(*final);
	
	double fitness = icp.getFitnessScore();
	std::cout << "has converged:" << icp.hasConverged() << " score: " << fitness << std::endl;
	
	Eigen::Affine3f tf;
	tf.matrix() = icp.getFinalTransformation();
	
	//Eigen::AngleAxis aa(tf.rotation());
	
	if (fitness > 0.001) {
		std::cout << "bad points, skipping" << std::endl;
	} else {
		if (tf.matrix()(0,0) < 0.9 || tf.matrix()(1,1) < 0.9 || tf.matrix()(2,2) < 0.9) {
			std::cout << icp.getFinalTransformation() << std::endl;
		}
		//*curr += *final;
		combine(curr,final);
		if (downsampleSize > 0) {
			downsampleCloudInPlace(curr,downsampleSize);
		}
	}
	transformPointCloudInPlace(curr, tf.inverse());
}

void combine(ColorCloudPtr curr,ColorCloudPtr in) {
	static const float maxAllowableDist = 0.00001;
	pcl::KdTreeFLANN<ColorPoint> tree;
	int nn = 10;
	std::vector<int> nn_indices(nn);
	std::vector<float> nn_dists(nn);
	
	tree.setInputCloud(curr);
	
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices ());
	pcl::ExtractIndices<ColorPoint> extract;
	
	float max = -1;
	float avg = 0;
	float avg2 = 0;
	float max_nndiff = -1;
	float min_nndiff = 1000000;
	for (size_t i = 0; i < in->points.size(); ++i) {
		tree.nearestKSearch (in->points[i], nn, nn_indices, nn_dists);
		float dist = nn_dists[0];
		if (dist > max) {
			max = dist;
		}
		avg += dist;
		if (dist < maxAllowableDist) {
			inliers->indices.push_back(i);
			avg2 += dist;
		}
		float nndiff = *std::max_element(nn_dists.begin(),nn_dists.end()) - *std::min_element(nn_dists.begin(),nn_dists.end());
		if (nndiff > max_nndiff) {
			max_nndiff = nndiff;
		} else if (nndiff < min_nndiff) {
			min_nndiff = nndiff;
		}
	}
	avg = avg / in->points.size();
	avg2 = avg / inliers->indices.size();
	
	std::cout << "Removed " << in->points.size() - inliers->indices.size() << " out of " << in->points.size() << std::endl;
	std::cout << "Max " << max << "\tavg " << avg << "\tavg2 " << avg2 << std::endl;
	std::cout << "Diff max " << max_nndiff << "\tmin " << min_nndiff << std::endl;
	
	ColorCloudPtr out(new ColorCloud());
	extract.setInputCloud(in);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter(*out);
	
	*curr += *out;
}
