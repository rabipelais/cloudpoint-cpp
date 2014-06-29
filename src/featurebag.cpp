#include "featurebag.hpp"

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/surface/convex_hull.h>

#include <stdlib.h>
#include <algorithm>


const int MAX_SAMPLES = 100000;
const int MAX_PAIRS = 100000;
const int MAX_TRIOS = 100000;

FeatureBag::FeatureBag() {

}

FeatureBag::FeatureBag(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
	: cloud(cloud)
{
	//Calculate features
}

FeatureBag::~FeatureBag() {

}

std::vector<double> makeHistogram(int bins, std::vector<double> values) {
	std::vector<double> histogram = std::vector<double>(bins);
	double width = *std::max_element(values.begin(), values.end()) / (double) bins;

	for(int i = 0; i < values.size(); i++) {
		histogram[std::min((int) (values.at(i) / width), bins - 1)]++;
	}
}

double distance(pcl::PointXYZ a, pcl::PointXYZ b) {
	return sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));
}

double norm(pcl::PointXYZ a) {
	return sqrt(std::pow(a.x, 2) + std::pow(a.y, 2) + std::pow(a.z, 2));
}

double dot(pcl::PointXYZ a, pcl::PointXYZ b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

pcl::PointXYZ sub(pcl::PointXYZ a, pcl::PointXYZ b){
	pcl::PointXYZ c;
	c.x = a.x - b.x;
	c.y = a.y - b.y;
	c.z = a.z - b.z;
	return c;
}

double angle(pcl::PointXYZ a, pcl::PointXYZ b, pcl::PointXYZ c) {
	pcl::PointXYZ v1 = sub(a, b);
	pcl::PointXYZ v2 = sub(c, b);
	double d1 = norm(v1);
	double d2 = norm(v2);
	double prod = dot(v1, v2);

	return acos(prod / (d1 * d2));
}

void FeatureBag::calculateD2(int bins) {
	int numPoints = cloud->points.size();
	std::vector<int> indices = std::vector<int>(std::min(numPoints, MAX_SAMPLES));

	if(numPoints < MAX_SAMPLES) {
		//Take all the points
		for(int i = 0; i < numPoints; i++) {
			indices[i] = i;
		}
	} else {
		//Sample enough points
		for(int i = 0; i < MAX_SAMPLES; i++) {
			indices[i] = rand() % numPoints;
		}
	}

	std::vector<double> distances = std::vector<double>(MAX_PAIRS);
	int first;
	int second;
	int numIndices = indices.size();
	//Now make the pairs and calculate the distances
	for(int i = 0; i < MAX_PAIRS; i++) {
		first = rand() % numIndices;
		//Sample a `different` point
		do {
			second = rand() % numIndices;
		} while(first == second);
		//Calculate the distance
		distances[i] = distance(cloud->points[indices[first]], cloud->points[indices[second]]);
	}
	d2Histogram = makeHistogram(bins, distances);
}

void FeatureBag::calculateA3(int bins) {
	int numPoints = cloud->points.size();
	std::vector<int> indices = std::vector<int>(std::min(numPoints, MAX_SAMPLES));

	if(numPoints < MAX_SAMPLES) {
		//Take all the points
		for(int i = 0; i < numPoints; i++) {
			indices[i] = i;
		}
	} else {
		//Sample enough points
		for(int i = 0; i < MAX_SAMPLES; i++) {
			indices[i] = rand() % numPoints;
		}
	}

	std::vector<double> angles = std::vector<double>(MAX_TRIOS);
	int first;
	int second;
	int third;
	int numIndices = indices.size();
	//Now make the trios and calculate the angles
	for(int i = 0; i < MAX_TRIOS; i++) {
		first = rand() % numIndices;
		//Sample a `different` point
		do {
			second = rand() % numIndices;
		} while(first == second);

		do {
			third = rand() % numIndices;
		} while(first == third || second == third);

		//Calculate the angle
		angles[i] = angle(cloud->points[indices[first]], cloud->points[indices[second]], cloud->points[indices[third]]);
	}
	a3Histogram = makeHistogram(bins, angles);
}

void FeatureBag::calculateAreaAndVolume() {
	pcl::ConvexHull<pcl::PointXYZ> cHull;
	pcl::PointCloud<pcl::PointXYZ> cHull_points;
	cHull.setComputeAreaVolume(true);
	std::vector<pcl::Vertices> polygons;

	cHull.setInputCloud(cloud);
	cHull.reconstruct (cHull_points, polygons);

	area = cHull.getTotalArea();
	volume = cHull.getTotalVolume();
}
