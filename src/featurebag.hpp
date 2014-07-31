#ifndef FEATUREBAG_HPP
#define FEATUREBAG_HPP

#include <vector>
#include <Eigen/Core>

#include <pcl/point_types.h>
#include <pcl/common/centroid.h>

#include "mongo/client/dbclient.h"
#include "mongo/bson/bson.h"

struct Features {
	double area;
	double volume;

	double bbVolume;
	double bbLong;
	double bbMedian;
	double bbShort;
	double bbLongShort;
	double bbMedianShort;

	std::vector<double> d2Histogram;
	std::vector<double> a3Histogram;
};

class FeatureBag {
public:
	FeatureBag();
	FeatureBag(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::vector<Eigen::Vector3f>& points);
	~FeatureBag();


	std::vector<double> getD2Histogram() {return features.d2Histogram;}
	std::vector<double> getA3Histogram() {return features.a3Histogram;}
	double getArea() {return features.area;}
	double getVolume() {return features.volume;}

	double getBBVolume() {return features.bbVolume;}
	double getBBLong() {return features.bbLong;}
	double getBBMedian() {return features.bbMedian;}
	double getBBShort() {return features.bbShort;}
	double getBBLongShort() {return features.bbLongShort;}
	double getBBMedianShort() {return features.bbMedianShort;}

	double compareD2(std::vector<double> vals);
	double compareA3(std::vector<double> vals);

	Features features;

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	std::vector<Eigen::Vector3f> points;

	void calculateD2(int bins);
	void calculateA3(int bins);
	void calculateAreaAndVolume();
	void calculateBB();
};

namespace features {
	Features fromBSONObj(mongo::BSONObj obj);

	void compareToDB(FeatureBag features, mongo::DBClientConnection &c);
	void saveToDB(FeatureBag features, std::string name, mongo::DBClientConnection &c);


	std::vector<double> normalize(std::vector<double> x);
	double compareHistograms(std::vector<double> a, std::vector<double> b);
}

#endif
