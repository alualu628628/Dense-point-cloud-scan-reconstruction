#include "Fusion.h"


//double KalmanFilter(KalmanInfo* kalmanInfo, double lastMeasurement)
//{
//	//预测下一时刻的值
//	double predictValue = kalmanInfo->A* kalmanInfo->filterValue;   //x的先验估计由上一个时间点的后验估计值和输入信息给出，此处需要根据基站高度做一个修改
//
//	//求协方差
//	kalmanInfo->P = kalmanInfo->A*kalmanInfo->A*kalmanInfo->P + kalmanInfo->Q;  //计算先验均方差 p(n|n-1)=A^2*p(n-1|n-1)+q
//	double preValue = kalmanInfo->filterValue;  //记录上次实际坐标的值
//
//	//计算kalman增益
//	kalmanInfo->kalmanGain = kalmanInfo->P*kalmanInfo->H / (kalmanInfo->P*kalmanInfo->H*kalmanInfo->H + kalmanInfo->R);  //Kg(k)= P(k|k-1) H’ / (H P(k|k-1) H’ + R)
//	//修正结果，即计算滤波值
//	kalmanInfo->filterValue = predictValue + (lastMeasurement - predictValue)*kalmanInfo->kalmanGain;  //利用残余的信息改善对x(t)的估计，给出后验估计，这个值也就是输出  X(k|k)= X(k|k-1)+Kg(k) (Z(k)-H X(k|k-1))
//	//更新后验估计
//	kalmanInfo->P = (1 - kalmanInfo->kalmanGain*kalmanInfo->H)*kalmanInfo->P;//计算后验均方差  P[n|n]=(1-K[n]*H)*P[n|n-1]
//
//	return  kalmanInfo->filterValue;
//}



Fusion::Fusion(){


}
	
Fusion::~Fusion(){



}

/*=======================================
SetAccDisSize
Input: iNodeNum - node number (voxel corner number)
Output: m_vKinectWeight - a weight vector
Function:set the size of accumulated distance map 
========================================*/
void Fusion::SetAccDisSize(const int & iNodeNum, float fRawDis){

	//distances of each node in kinnect fusion mode
	m_vAccDis.clear();
	m_vAccDis.resize(iNodeNum, fRawDis);

}


/*=======================================
SetKinectWeight
Input: iNodeNum - node number (voxel corner number)
     fRawWeight - initial weight of each corner (node)
Output: m_vKinectWeight - a weight vector
Function:set node weight in kinect fusion mode
========================================*/
void Fusion::SetKinectMode(const int & iNodeNum, float fRawWeight){

	//weights of each node in kinnect fusion mode
	m_vKinectWeight.clear();
	m_vKinectWeight.resize(iNodeNum, fRawWeight);


}

/*=======================================
KinectFusion
Input: vCurrentDis - the newest signed distance value
          vWeights - the newest weight of each corner (the angle between the face and the ray as usual)
           vDisMap - the accumulated signed distance value (k times)
Output: m_vKinectWeight - a weight vector
               vDisMap - the accumulated signed distance value (k+1 times)
Function:fusion two frame or two glance of mesh using kinect fusion method
========================================*/
void Fusion::KinectFusion(const std::vector<float> & vCurrentDis, std::vector<float> & vCurrentW){

	//update the distance
	for (int i = 0; i != vCurrentDis.size(); ++i){

		//distance
		//d_c = (w_c*d_c+w_new*d_new)/(w_c+w_new)
		m_vAccDis[i] = (m_vAccDis[i] * m_vKinectWeight[i] + vCurrentDis[i] * vCurrentW[i]) / (vCurrentW[i] + m_vKinectWeight[i]);

		//weight
		//w_c = (w_c+w_new)/2.0
		//if vCurrentW[i] is constant 1, the m_vKinectWeight[i] would not change
		m_vKinectWeight[i] = (vCurrentW[i] + m_vKinectWeight[i]) / 2.0f;

	}//end for i

};

/*=======================================
ConvexBasedFusion
Input: f_viewpoint - a given viewpoint
Output: none
Function: set the given viewpoint to class as m_oViewPInWorld
========================================*/
void Fusion::ConvexBasedFusion(const std::vector<float> & vCurrentDis, std::vector<float> & vDisMap){

	//update the distance
	for (int i = 0; i != vCurrentDis.size(); ++i){

		//the case of a visible node
		if (vCurrentDis[i] > 0.0){
			//the case of finding new visiable map
			if (vDisMap[i] < 0.0){
				vDisMap[i] = vCurrentDis[i];
			}else{//end else
				if (vDisMap[i] > vCurrentDis[i])
					vDisMap[i] = vCurrentDis[i];
			}//end else

		//the case of a occluded node
		}else{
		
			if (vDisMap[i] < 0.0){
				if (vDisMap[i] < vCurrentDis[i])
					vDisMap[i] = vCurrentDis[i];
			}//end if vDisMap[i] >= 0.0
		
		}//end big else

	}//end for i


}


/*=======================================
ConvexBasedFusion
Input: f_viewpoint - a given viewpoint
Output: none
Function: set the given viewpoint to class as m_oViewPInWorld
========================================*/
void Fusion::UnionMinimalFusion(const std::vector<float> & vCurrentDis){

	//update the distance
	for (int i = 0; i != vCurrentDis.size(); ++i){

		//the case of a visible node
		if (vCurrentDis[i]){
			//
			if (m_vAccDis[i]){
				//
				if (abs(m_vAccDis[i]) > abs(vCurrentDis[i]))
					m_vAccDis[i] = vCurrentDis[i];
			
			
			}else{
				//
				m_vAccDis[i] = vCurrentDis[i];
			
			}//end else

		}//end big if (vCurrentDis[i])

	}//end for i


}


float Fusion::NormalFusion(const std::vector<int> & vPointIdx, const pcl::PointCloud<pcl::PointNormal> & vCloudNormal, pcl::PointNormal & oOutPointNormal){

	//Initialize output
	pcl::PointNormal oOnePN;
	oOnePN.x = 0.0f;
	oOnePN.y = 0.0f;
	oOnePN.z = 0.0f;
	oOnePN.normal_x = 0.0f;
	oOnePN.normal_y = 0.0f;
	oOnePN.normal_z = 0.0f;

	GaussianMapping oGaussSphere;

	//linear increase
	for (int i = 0; i != vPointIdx.size(); ++i){

		int iVoxelPIdx = vPointIdx[i];
	
		oOnePN.x = oOnePN.x + vCloudNormal.points[iVoxelPIdx].x;
		oOnePN.y = oOnePN.y + vCloudNormal.points[iVoxelPIdx].y;
		oOnePN.z = oOnePN.z + vCloudNormal.points[iVoxelPIdx].z;
		oOnePN.normal_x = oOnePN.normal_x + vCloudNormal.points[iVoxelPIdx].normal_x;
		oOnePN.normal_y = oOnePN.normal_y + vCloudNormal.points[iVoxelPIdx].normal_y;
		oOnePN.normal_z = oOnePN.normal_z + vCloudNormal.points[iVoxelPIdx].normal_z;

		//oGaussSphere.MappingtoSphere(vCloudNormal.points[iVoxelPIdx]);
	
	}

	
	float fNum = float(vPointIdx.size());

	if (vPointIdx.size()){
		//take the mean
		oOnePN.x = oOnePN.x / fNum;
		oOnePN.y = oOnePN.y / fNum;
		oOnePN.z = oOnePN.z / fNum;
		oOnePN.normal_x = oOnePN.normal_x / fNum;
		oOnePN.normal_y = oOnePN.normal_y / fNum;
		oOnePN.normal_z = oOnePN.normal_z / fNum;
	}

	oOutPointNormal.x = oOnePN.x;
	oOutPointNormal.y = oOnePN.y;
	oOutPointNormal.z = oOnePN.z;
	oOutPointNormal.normal_x = oOnePN.normal_x;
	oOutPointNormal.normal_y = oOnePN.normal_y;
	oOutPointNormal.normal_z = oOnePN.normal_z;

	//using pca based method
	//float fNormalEntropy = oGaussSphere.ComputeDistribution();

	//define an entropy indicating the confusion of the normal vectors
	//-1 means no value
	float fNormalEntropy = -1.0f;

	//if the input is not empty
	if (vPointIdx.size()){
		//give the distance for entropy
		fNormalEntropy = oOnePN.normal_x*oOnePN.normal_x + oOnePN.normal_y*oOnePN.normal_y + oOnePN.normal_z*oOnePN.normal_z;

		fNormalEntropy = sqrt(fNormalEntropy);

		oOutPointNormal.normal_x = oOutPointNormal.normal_x / fNormalEntropy;
		oOutPointNormal.normal_y = oOutPointNormal.normal_y / fNormalEntropy;
		oOutPointNormal.normal_z = oOutPointNormal.normal_z / fNormalEntropy;
	}


	return fNormalEntropy;


}

void Fusion::NormalFusion(const std::vector<std::vector<int>> & vPointIdxs, const std::vector<int> & vVoxelIdxs, const pcl::PointCloud<pcl::PointNormal> & vCloudNormal, pcl::PointNormal & oOutPointNormal){

	//Initialize output
	pcl::PointNormal oOnePN;
	oOnePN.x = 0.0f;
	oOnePN.y = 0.0f;
	oOnePN.z = 0.0f;
	oOnePN.normal_x = 0.0f;
	oOnePN.normal_y = 0.0f;
	oOnePN.normal_z = 0.0f;

	GaussianMapping oGaussSphere;

	float fNum = 0.0f;

	//linear increase
	for (int i = 0; i != vVoxelIdxs.size(); ++i){

		int iVoxelIdx = vVoxelIdxs[i];

		for (int j = 0; j != vPointIdxs[iVoxelIdx].size(); ++j){

			int iPointIdx = vPointIdxs[iVoxelIdx][j];

			oOnePN.x = oOnePN.x + vCloudNormal.points[iPointIdx].x;
			oOnePN.y = oOnePN.y + vCloudNormal.points[iPointIdx].y;
			oOnePN.z = oOnePN.z + vCloudNormal.points[iPointIdx].z;
			oOnePN.normal_x = oOnePN.normal_x + vCloudNormal.points[iPointIdx].normal_x;
			oOnePN.normal_y = oOnePN.normal_y + vCloudNormal.points[iPointIdx].normal_y;
			oOnePN.normal_z = oOnePN.normal_z + vCloudNormal.points[iPointIdx].normal_z;

			fNum = fNum + 1.0f;

		}

	}

	//mean
	if (fNum){
		//take the mean
		oOnePN.x = oOnePN.x / fNum;
		oOnePN.y = oOnePN.y / fNum;
		oOnePN.z = oOnePN.z / fNum;
		oOnePN.normal_x = oOnePN.normal_x / fNum;
		oOnePN.normal_y = oOnePN.normal_y / fNum;
		oOnePN.normal_z = oOnePN.normal_z / fNum;
		//output unit normal vector
		float fNorm = sqrt(oOnePN.normal_x*oOnePN.normal_x + oOnePN.normal_y*oOnePN.normal_y + oOnePN.normal_z*oOnePN.normal_z);
		oOnePN.normal_x = oOnePN.normal_x / fNorm;
		oOnePN.normal_y = oOnePN.normal_y / fNorm;
		oOnePN.normal_z = oOnePN.normal_z / fNorm;

	}

	oOutPointNormal.x = oOnePN.x;
	oOutPointNormal.y = oOnePN.y;
	oOutPointNormal.z = oOnePN.z;
	oOutPointNormal.normal_x = oOnePN.normal_x;
	oOutPointNormal.normal_y = oOnePN.normal_y;
	oOutPointNormal.normal_z = oOnePN.normal_z;

}

//reload 
//add the normal vectors of voxels for input and ouput, respectively
void Fusion::NormalFusion(const std::vector<int> & vVoxelIdxs, const std::vector<float> & vFeatures, pcl::PointCloud<pcl::PointNormal>::Ptr & pVoxelNormal){

	//Initialize output
	pcl::PointNormal oOnePN;
	oOnePN.x = 0.0f;
	oOnePN.y = 0.0f;
	oOnePN.z = 0.0f;
	oOnePN.normal_x = 0.0f;
	oOnePN.normal_y = 0.0f;
	oOnePN.normal_z = 0.0f;

	GaussianMapping oGaussSphere;

	float fNum = 0.0f;

	//linear increase
	for (int i = 0; i != vVoxelIdxs.size(); ++i){

		int iPointIdx = vVoxelIdxs[i];

		if (vFeatures[iPointIdx] >= 0){

			oOnePN.x = oOnePN.x + pVoxelNormal->points[iPointIdx].x;
			oOnePN.y = oOnePN.y + pVoxelNormal->points[iPointIdx].y;
			oOnePN.z = oOnePN.z + pVoxelNormal->points[iPointIdx].z;
			oOnePN.normal_x = oOnePN.normal_x + pVoxelNormal->points[iPointIdx].normal_x;
			oOnePN.normal_y = oOnePN.normal_y + pVoxelNormal->points[iPointIdx].normal_y;
			oOnePN.normal_z = oOnePN.normal_z + pVoxelNormal->points[iPointIdx].normal_z;

			fNum = fNum + 1.0f;

		}

	}

	//mean
	if (fNum){
		//take the mean
		oOnePN.x = oOnePN.x / fNum;
		oOnePN.y = oOnePN.y / fNum;
		oOnePN.z = oOnePN.z / fNum;
		oOnePN.normal_x = oOnePN.normal_x / fNum;
		oOnePN.normal_y = oOnePN.normal_y / fNum;
		oOnePN.normal_z = oOnePN.normal_z / fNum;
		//output unit normal vector
		float fNorm = sqrt(oOnePN.normal_x*oOnePN.normal_x + oOnePN.normal_y*oOnePN.normal_y + oOnePN.normal_z*oOnePN.normal_z);
		oOnePN.normal_x = oOnePN.normal_x / fNorm;
		oOnePN.normal_y = oOnePN.normal_y / fNorm;
		oOnePN.normal_z = oOnePN.normal_z / fNorm;

	}

	//the first member of vVoxelIdxs should be the query point
	pVoxelNormal->points[vVoxelIdxs[0]] = oOnePN;

}