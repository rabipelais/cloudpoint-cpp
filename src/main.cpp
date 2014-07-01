
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>

#include "mongo/client/dbclient.h"
#include "mongo/bson/bson.h"
#include <iostream>
#include <stdio.h>

#include "featurebag.hpp"

using std::cout;
using std::endl;

using mongo::BSONArray;
using mongo::BSONObj;
using mongo::BSONObjIterator;
using mongo::BSONElement;

BSONObj findMatchingObjects(mongo::DBClientConnection &c);
std::auto_ptr<mongo::DBClientCursor> findObjectsWithFeatures(mongo::DBClientConnection &c);
void compareToDB(FeatureBag features, mongo::DBClientConnection &c);

int main(int argc, char* argv[])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	if(argc < 1) return 1;
	std::string fileName = argv[1];

	FILE* pFile = fopen(fileName.c_str(), "rt");

	if (pFile==NULL)
	{
		printf("\nCannot open object file: %s\n\n", fileName.c_str());
		return 1;
	}

	float x, y, z;
	while(fscanf(pFile, "%e %e %e", &x, &y, &z) != EOF) {
		cloud->push_back(pcl::PointXYZ(x, y, z));
	}


    std::cout << "Loaded "
	      << cloud->width * cloud->height
	      << " data points."
	      << std::endl;

    mongo::DBClientConnection mongoConnection;
    try {
	    mongoConnection.connect("localhost");
        cout << "Connected ok to mongoDB" << endl;
    } catch( const mongo::DBException &e ) {
        cout << "Caught " << e.what() << endl;
        return 1;
    }
    FeatureBag features(cloud);
    compareToDB(features, mongoConnection);
}

void compareToDB(FeatureBag features, mongo::DBClientConnection &c) {
	std::auto_ptr<mongo::DBClientCursor> cursor = findObjectsWithFeatures(c);
	double minD2 = 100000;
	double minA3 = 100000;
	double minArea = 100000;
	double minVolume = 100000;

	std::string mArea, mVolume, mD2, mA3;
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
			}
		}
		cout << endl;
	}
	cout << "Minimal D2: " << mD2 << endl
	     << "Minimal A3: " << mA3 << endl
	     << "Minimal Area: " << mArea << endl
	     << "Minimal Volume: " << mVolume << endl;
}

std::auto_ptr<mongo::DBClientCursor> findObjectsWithFeatures(mongo::DBClientConnection &c) {
	std::auto_ptr<mongo::DBClientCursor> cursor = c.query("tesis.objects", QUERY("features" << BSON("$exists" << true)));
	return cursor;
}

BSONObj findMatchingObjects(mongo::DBClientConnection &c) {
	std::string feature = "d2";
	BSONArray pipeline = BSON_ARRAY(
		BSON("$unwind" << "$features") <<
		BSON("$match" << BSON("features.type" << feature)) <<
		BSON("$group" << BSON("_id" << "$name" << "features" << BSON("$push" << "$features")))
		);
	BSONObj res;
	c.runCommand("tesis", BSON("aggregate" << "objects" << "pipeline" << pipeline), res);
	return res;
}
