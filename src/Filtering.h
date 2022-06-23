#ifndef FILTERING_H
#define FILTERING_H

#include<cmath>
#include<vector>
#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/kdtree/kdtree.h>

class Filters{

public:

	Filters();
	~Filters();

	//filtering based on distance thresholds
	static void DistanceFiltering(const pcl::PointCloud<pcl::PointXYZ> & vRawCloud, const pcl::PointXYZ & oBasePoint, 
		                          pcl::PointCloud<pcl::PointXYZ> & vFiltedCloud, float fCloseDis = 0.0, float fFarDis = 50.0);

	//compute the Euclidean distance
	float EuclideanDis(const pcl::PointXYZ & oPointA, const pcl::PointXYZ & oPointB);

private:


};



#endif
