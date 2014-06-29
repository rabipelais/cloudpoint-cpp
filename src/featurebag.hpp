#ifndef FEATUREBAG_HPP
#define FEATUREBAG_HPP

#include <vector>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>

class FeatureBag {
public:
	FeatureBag();
	FeatureBag(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	~FeatureBag();

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

	std::vector<double> d2Histogram;
	std::vector<double> a3Histogram;
	double area;
	double volume;

	void calculateD2(int bins);
	void calculateA3(int bins);
	void calculateAreaAndVolume();
};

#endif
