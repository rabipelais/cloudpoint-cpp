#include "featurebag.hpp"
#include "convexHull.hpp"

#include "gdiam.h"
#include "point.hpp"

#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <limits>
using std::cout;
using std::endl;

const int MAX_SAMPLES = 100000;
const int MAX_PAIRS = 100000;
const int MAX_TRIOS = 100000;

using mongo::BSONArray;
using mongo::BSONArrayBuilder;
using mongo::BSONObj;
using mongo::BSONObjIterator;
using mongo::BSONElement;

FeatureBag::FeatureBag() {

}

FeatureBag::FeatureBag(const std::vector<Eigen::Vector3f>& points)
	: points(points), features()
{
	int bins = 15;
	//Calculate features
	calculateD2(bins);
	calculateA3(bins);
	calculateAreaAndVolume();
	calculateBB();
}

FeatureBag::~FeatureBag() {

}

std::vector<double> makeHistogram(int bins, std::vector<double> values) {
	std::vector<double> histogram(bins);
	double width = *std::max_element(values.begin(), values.end()) / (double) bins;

	for(int i = 0; i < values.size(); i++) {
		histogram[std::min((int) (values.at(i) / width), bins - 1)]++;
	}
	return histogram;
}

void FeatureBag::calculateD2(int bins) {
	cout << "Calculating D2..." << endl;
	int numPoints = points.size();
	std::vector<int> indices = std::vector<int>(std::min(numPoints, MAX_SAMPLES));

	if(numPoints < MAX_SAMPLES) {
		//Take all the points
		for(int i = 0; i < indices.size(); i++) {
			indices[i] = i;
		}
	} else {
		//Sample enough points
		for(int i = 0; i < indices.size(); i++) {
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
		distances[i] = distance(points[indices[first]], points[indices[second]]);
	}
	features.d2Histogram = makeHistogram(bins, distances);
}

void FeatureBag::calculateA3(int bins) {
	cout << "Calculating A3..." << endl;
	int numPoints = points.size();
	std::vector<int> indices = std::vector<int>(std::min(numPoints, MAX_SAMPLES));

	if(numPoints < MAX_SAMPLES) {
		//Take all the points
		for(int i = 0; i < indices.size(); i++) {
			indices[i] = i;
		}
	} else {
		//Sample enough points
		for(int i = 0; i < indices.size(); i++) {
			indices[i] = rand() % numPoints;
		}
	}

	std::vector<double> angles(MAX_TRIOS);
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
		angles[i] = angle(points[indices[first]], points[indices[second]], points[indices[third]]);
	}
	features.a3Histogram = makeHistogram(bins, angles);
}

void FeatureBag::calculateAreaAndVolume() {
	// cout << "Calculating area and volume..." << endl;
	// pcl::ConvexHull<pcl::PointXYZ> cHull;
	// pcl::PointCloud<pcl::PointXYZ> cHull_points;
	// cHull.setComputeAreaVolume(true);
	// std::vector<pcl::Vertices> polygons;

	// cHull.setInputCloud(cloud);
	// cHull.reconstruct (cHull_points, polygons);

	// features.area = cHull.getTotalArea();
	// features.volume = cHull.getTotalVolume();
}

void FeatureBag::calculateBB() {
	cout << "Calculating bounding box..." << endl;
	double eps = 0.005;
	//Convert to gdiam format
	int size = points.size();
	gdiam_real *tmpVals = new gdiam_real[size * 3];
	for(int i = 0; i < size * 3; i += 3) {
		tmpVals[i + 0] = points[i / 3][0];
		tmpVals[i + 1] = points[i / 3][1];
		tmpVals[i + 2] = points[i / 3][2];
	}
	gdiam_point *tmpPoints = gdiam_convert(tmpVals, size);
	gdiam_bbox bbox = gdiam_approx_mvbb(tmpPoints, size, eps);

	double len1 = ( bbox.high_1 - bbox.low_1 );
	double len2 = ( bbox.high_2 - bbox.low_2 );
	double len3 = ( bbox.high_3 - bbox.low_3 );

	double l = std::max(len1, std::max(len2, len3));
	double s = std::min(len1, std::min(len2, len3));
	double m = len1 * len2 * len3 / (l * s);

	features.bbVolume = bbox.volume();
	features.bbLong = l;
	features.bbShort = s;
	features.bbMedian = m;
	features.bbLongShort = l/s;
	features.bbMedianShort = m/s;

	delete[] tmpVals;
	delete[] tmpPoints;
}

double FeatureBag::compareD2(std::vector<double> vals) {
	return features::compareHistograms(vals, this->features.d2Histogram);
}

double FeatureBag::compareA3(std::vector<double> vals) {
	return features::compareHistograms(vals, this->features.a3Histogram);
}

Features features::fromBSONObj(BSONObj obj) {
	return Features();
}

std::vector<double> features::normalize(std::vector<double> x) {
	std::vector<double> n(x.size());
	double max = *std::max_element(x.begin(), x.end());
	for(int i = 0; i < n.size(); i++) {
		n[i] = x[i] / max;
	}
	return n;
}

double features::compareHistograms(std::vector<double> a, std::vector<double> b) {
	double diff = 0.0;
	std::vector<double> an = features::normalize(a);
	std::vector<double> bn = features::normalize(b);

	for(int i = 0; i < an.size(); i++) {
		diff += std::pow(an[i] - bn[i], 2);
	}
	return diff;
}



std::auto_ptr<mongo::DBClientCursor> findObjectsWithFeatures(mongo::DBClientConnection &c) {
	std::auto_ptr<mongo::DBClientCursor> cursor = c.query("tesis.objects", QUERY("features" << BSON("$exists" << true)));
	return cursor;
}

void features::compareToDB(FeatureBag features, mongo::DBClientConnection &c) {
	std::auto_ptr<mongo::DBClientCursor> cursor = findObjectsWithFeatures(c);
	double minD2 = std::numeric_limits<double>::max();
	double minA3 = std::numeric_limits<double>::max();
	double minArea = std::numeric_limits<double>::max();
	double minVolume = std::numeric_limits<double>::max();

	double minBBVolume = std::numeric_limits<double>::max();
	double minBBLong = std::numeric_limits<double>::max();
	double minBBMedian = std::numeric_limits<double>::max();
	double minBBShort = std::numeric_limits<double>::max();
	double minBBLongShort = std::numeric_limits<double>::max();
	double minBBMedianShort = std::numeric_limits<double>::max();

	std::string mArea, mVolume, mD2, mA3, mBBVol, mBBL, mBBS, mBBM, mBBLS, mBBMS;
	//Iterate over each object in the DB
	while (cursor->more()) {
		BSONObj p = cursor->next();
		std::string name = p["name"].toString();
		cout << "Comparing to " << name << endl;
		std::vector<BSONElement> fs;
		//fs will now be an array of features
		BSONForEach(e, p.getObjectField("features")) {
			fs.push_back(e);
		}

		//Iterate over each feature of the DB object
		//and try to compare it to the new object
		for(int i = 0; i < fs.size(); i++) {
			std::string type;
			fs[i]["type"].Val(type);

			if(type == "d2") {
				int bins = fs[i]["params"]["bins"].numberInt();

				//Get the values array
				std::vector<double> vals;
				BSONForEach(e, fs[i]["vals"].Obj()) {
					vals.push_back(e.numberDouble());
				}
				if(bins == 15) {
					double diff = features.compareD2(vals);
					cout << "D2 histogram: " << diff << endl;

					if(diff < minD2) {
						minD2 = diff;
						mD2 = name;
					}
				}
			} else if(type == "a3") {
				int bins = fs[i]["params"]["bins"].numberInt();

				//Get the values array
				std::vector<double> vals;
				BSONForEach(e, fs[i]["vals"].Obj()) {
					vals.push_back(e.numberDouble());
				}
				if(bins == 15) {
					double diff = features.compareA3(vals);
					cout << "A3 histogram: " << diff << endl;

					if(diff < minA3) {
						minA3 = diff;
						mA3 = name;
					}
				}
			} else if(type == "ch") {
				double area = fs[i]["area"].numberDouble();
				double volume = fs[i]["volume"].numberDouble();
				double diffArea = fabs(area - features.getArea());
				double diffVolume = fabs(volume - features.getVolume());
				cout << "Area: " << diffArea << endl;
				cout << "Volume: " << diffVolume << endl;

				if(diffArea < minArea) {
					minArea = diffArea;
					mArea = name;
				}
				if(diffVolume < minVolume) {
					minVolume = diffVolume;
					mVolume = name;
				}
			} else if(type == "bb") {
				double bbVolume = fs[i]["bbVolume"].numberDouble();
				double bbLong = fs[i]["bbLong"].numberDouble();
				double bbMedian = fs[i]["bbMedian"].numberDouble();
				double bbShort = fs[i]["bbShort"].numberDouble();
				double bbLongShort = fs[i]["bbLongShort"].numberDouble();
				double bbMedianShort = fs[i]["bbMedianShort"].numberDouble();

				double diffBBVolume = fabs(bbVolume - features.getBBVolume());
				double diffBBLong = fabs(bbLong - features.getBBLong());
				double diffBBMedian = fabs(bbMedian - features.getBBMedian());
				double diffBBShort = fabs(bbShort - features.getBBShort());
				double diffBBLongShort = fabs(bbLongShort - features.getBBLongShort());
				double diffBBMedianShort = fabs(bbMedianShort - features.getBBMedianShort());

				cout << "BBVolume: " << bbVolume << endl;
				cout << "BBLong: " << bbLong << endl;
				cout << "BBMedian: " << bbMedian << endl;
				cout << "BBShort: " << bbShort << endl;
				cout << "BBLongShort: " << bbLongShort << endl;
				cout << "BBMedianShort: " << bbMedianShort << endl;

				if(diffBBVolume < minBBVolume) {
					minBBVolume = diffBBVolume;
					mBBVol = name;
				}
				if(diffBBLong < minBBLong) {
					minBBLong = diffBBLong;
					mBBL = name;
				}
				if(diffBBMedian < minBBMedian) {
					minBBMedian = diffBBMedian;
					mBBM = name;
				}
				if(diffBBShort < minBBShort) {
					minBBShort = diffBBShort;
					mBBS = name;
				}
				if(diffBBLongShort < minBBLongShort) {
					minBBLongShort = diffBBLongShort;
					mBBLS = name;
				}
				if(diffBBMedianShort < minBBMedianShort) {
					minBBMedianShort = diffBBMedianShort;
					mBBMS = name;
				}
			}
		}
		cout << endl;
	}
	cout << "Minimal D2: " << mD2 << endl
	     << "Minimal A3: " << mA3 << endl
	     << "Minimal Area: " << mArea << endl
	     << "Minimal Volume: " << mVolume << endl
	     << "Minimal BB-Volume: " << mBBVol << endl
	     << "Minimal Long/Short ratio: " << mBBLS << endl
	     << "Minimal Median/Short ratio: " << mBBMS << endl;
}

void features::saveToDB(FeatureBag features, std::string name, mongo::DBClientConnection &c) {
	cout << "Saving " << name << " to DB" << endl;

	Features fs = features.features;

	//Set the value array of the histograms
	BSONArrayBuilder d2Arr;
	d2Arr.append(fs.d2Histogram);

	BSONArrayBuilder a3Arr;
	a3Arr.append(fs.a3Histogram);

	//Serialize into BSON object
	BSONObj p = BSON( mongo::GENOID << "name" << name << "features" <<
		BSON_ARRAY(
			BSON("type" << "d2" << "params" << BSON("bins" << (unsigned int)fs.d2Histogram.size())
			     << "vals" << d2Arr.arr()[0]) <<
			BSON("type" << "a3" << "params" << BSON("bins" << (unsigned int)fs.a3Histogram.size())
			     << "vals" << a3Arr.arr()[0]) <<
			BSON("type" << "ch" << "area" << fs.area << "volume" << fs.volume) <<
			BSON("type" << "bb" << "bbVolume" << fs.bbVolume << "bbLong" << fs.bbLong
			     << "bbShort" << fs.bbShort << "bbMedian" << fs.bbMedian
			     << "bbLongShort" << fs.bbLongShort << "bbMedianShort" << fs.bbMedianShort)
			)
		);

	//Insert into DB
	c.insert("tesis.objects", p);
}
