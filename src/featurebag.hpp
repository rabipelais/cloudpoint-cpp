#ifndef FEATUREBAG_HPP
#define FEATUREBAG_HPP

#include <vector>

#include <pcl/point_types.h>
#include <pcl/common/centroid.h>

#include "mongo/bson/bson.h"

using mongo::BSONObj;

struct Features {
	double area;
	double volume;

	std::vector<double> d2Histogram;
	std::vector<double> a3Histogram;
};

class FeatureBag {
public:
	FeatureBag();
	FeatureBag(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	~FeatureBag();


	std::vector<double> getD2Histogram() {return features.d2Histogram;}
	std::vector<double> getA3Histogram() {return features.a3Histogram;}
	double getArea() {return features.area;}
	double getVolume() {return features.volume;}

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

	Features features;

	void calculateD2(int bins);
	void calculateA3(int bins);
	void calculateAreaAndVolume();
};

namespace features {
	Features fromBSONObj(BSONObj obj);
}

#endif
