#ifndef NORMALFEATURES_H
#define NORMALFEATURES_H

#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/kdtree/kdtree.h>

#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>

//Gaussian Mapping is to map the normal vectors to a Gaussian sphere
class GaussianMapping{

public:

	GaussianMapping();

	~GaussianMapping();


	//map to the surface of the sphere
	void MappingtoSphere(const pcl::PointNormal & oNormal);

	//compute distribution by using the covariance matrix
	float ComputeDistribution();

	//Spherical Points
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_pSphPoints;

private:


};



#endif