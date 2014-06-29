
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

BSONObj findMatchingObjects(mongo::DBClientConnection &c);

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
	      << " data points from cube_100000.pcd with the following fields: "
	      << std::endl;
    for (size_t i = 0; i < 10; ++i)
	std::cout << "    " << cloud->points[i].x
		  << " "    << cloud->points[i].y
		  << " "    << cloud->points[i].z << std::endl;

    mongo::DBClientConnection mongoConnection;
    try {
	    mongoConnection.connect("localhost");
        cout << "Connected ok to mongoDB" << endl;
    } catch( const mongo::DBException &e ) {
        cout << "Caught " << e.what() << endl;
        return 1;
    }
    BSONObj res = findMatchingObjects(mongoConnection);
    cout << res.toString() << endl;
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
