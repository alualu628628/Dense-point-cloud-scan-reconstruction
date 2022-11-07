#ifndef SIGNEDDISTANCE_H
#define SIGNEDDISTANCE_H

#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include "GHPR.h"
#include "Cell.h"
#include "ConvexHullOperation.h"
#include "MeshSample.h"
#include "Polar.h"
#include "Fusion.h"
#include "readtxt.h"




void SamplePoints(const pcl::PointCloud<pcl::PointXYZ> & vCloud, pcl::PointCloud<pcl::PointXYZ> & vNewCloud, int iSampleNum, bool bIntervalSamp = true);






class SignedDisParam{

public:

	SignedDisParam() :m_GHPRParam(2.5){
	
	
	};

	~SignedDisParam(){
	};

	float m_GHPRParam;
};

class SignedDistance :public SignedDisParam{

public:

	SignedDistance(){
	
	
	};

	~SignedDistance(){
	};

	//compute the signed distance based on convex hull operation - type 1
	std::vector<float> ConvexBasedGlance(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pRawCloud,
		                const pcl::PointXYZ & oViewPoint,
		                const pcl::PointCloud<pcl::PointXYZ>::Ptr & m_pVoxelCorner);

	//compute the signed distance based on ray cast (polar) - type 2
	std::vector<float> RaycastBasedGlance(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pRawCloud,
		                                  const pcl::PointXYZ & oViewPoint,
		                             const pcl::PointCloud<pcl::PointXYZ>::Ptr & m_pVoxelCorner);

	//compute the signed distance based on point - type 3
	std::vector<float> PointBasedGlance(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pRawCloud,
		                                const pcl::PointXYZ & oViewPoint,
		                                Voxelization & oVoxeler);


	//compute the signed distance based on point and normal - type 4
	//fThrCenterDis = 1.01f means that distance value is not applicable (leading to single-scale modeling)
	std::vector<float> NormalBasedGlance(const pcl::PointCloud<pcl::PointNormal>::Ptr & pCloudNormals,
										Voxelization & oVoxeler, float fThrCenterDis = 1.01f);

	//compute the nearest distance
	static std::vector<float> MinKDDis(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pCloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr & pQueryCloud);

	//compute the signed distance based on finding the nearest surface point - type 3
	std::vector<float> MinKDDSignedDis(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pNCloud, Voxelization & voxeler,
		const std::vector<FacePara> & vNormalPara);

	//compute the signed distance based on corners to plan in a voxel
	std::vector<float> PlanDistance(Voxelization & oVoxeler, const std::vector<FacePara> & vNormalPara);

	//=====data=====
	//
	std::vector<pcl::Vertices> m_vSurfaceIdxs;

	//
	std::vector<pcl::Vertices> m_vGlanceFaceIdxs;

};



#endif