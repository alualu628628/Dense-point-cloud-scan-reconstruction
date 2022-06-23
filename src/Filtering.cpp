#include "Filtering.h"


Filters::Filters(){

}


Filters::~Filters(){

}

//filtering based on distance thresholds
void Filters::DistanceFiltering(const pcl::PointCloud<pcl::PointXYZ> & vRawCloud, const pcl::PointXYZ & oBasePoint, pcl::PointCloud<pcl::PointXYZ> & vFiltedCloud, float fCloseDis, float fFarDis){

	//to each raw point
	for (int i = 0; i != vRawCloud.points.size(); ++i){
	
		float fx = vRawCloud.points[i].x - oBasePoint.x;
		float fy = vRawCloud.points[i].y - oBasePoint.y;
		float fz = vRawCloud.points[i].z - oBasePoint.z;
		
		//compute the distance 
		float fDis = sqrt(fx*fx + fy*fy + fz*fz);
		
		//get the filted point
		if (fDis >= fCloseDis && fDis <= fFarDis)
			vFiltedCloud.points.push_back(vRawCloud.points[i]);
	
	}


}

//compute the Euclidean distance between point A and B
float Filters::EuclideanDis(const pcl::PointXYZ & oPointA, const pcl::PointXYZ & oPointB){

	float fx = oPointA.x - oPointB.x;
	float fy = oPointA.y - oPointB.y;
	float fz = oPointA.z - oPointB.z;
	//compute the distance 
	return sqrt(fx*fx + fy*fy + fz*fz);

}