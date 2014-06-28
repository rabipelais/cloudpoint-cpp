#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/centroid.h>

#include <iostream>
#include <stdio.h>

using std::cout;
using std::endl;

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

}
