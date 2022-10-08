#include "NormalFeatures.h"


GaussianMapping::GaussianMapping() : m_pSphPoints(new pcl::PointCloud<pcl::PointXYZ>){

}

GaussianMapping::~GaussianMapping(){

}


//map to the surface of the sphere
void GaussianMapping::MappingtoSphere(const pcl::PointNormal & oNormal){

	//new the spherical point
	pcl::PointXYZ oPoint;

	//Convert normal vector to spherical point
	oPoint.x = oNormal.normal_x;
	oPoint.y = oNormal.normal_y;
	oPoint.z = oNormal.normal_z;

	m_pSphPoints->points.push_back(oPoint);

}

//compute distribution by using the covariance matrix
float GaussianMapping::ComputeDistribution(){

	//covariance decomposition in PCA
	pcl::PCA<pcl::PointXYZ> oCovariance;
	oCovariance.setInputCloud(m_pSphPoints);
	
	//get eigenvalues after covariance matrix decomposition
	Eigen::Vector3f vEigenValues = oCovariance.getEigenValues();

	//measure the dispersion
	float normalizationcoe = vEigenValues[0] + vEigenValues[1] + vEigenValues[2];
	float fCurvature = vEigenValues[0] / normalizationcoe;

	return fCurvature;

}

