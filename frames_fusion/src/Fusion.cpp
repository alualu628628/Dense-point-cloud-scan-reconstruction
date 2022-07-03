#include "Fusion.h"

#include "MeshOperation.h"

//double KalmanFilter(KalmanInfo* kalmanInfo, double lastMeasurement)
//{
//	//Ԥ����һʱ�̵�ֵ
//	double predictValue = kalmanInfo->A* kalmanInfo->filterValue;   //x�������������һ��ʱ���ĺ������ֵ��������Ϣ�������˴���Ҫ���ݻ�վ�߶���һ���޸�
//
//	//��Э����
//	kalmanInfo->P = kalmanInfo->A*kalmanInfo->A*kalmanInfo->P + kalmanInfo->Q;  //������������� p(n|n-1)=A^2*p(n-1|n-1)+q
//	double preValue = kalmanInfo->filterValue;  //��¼�ϴ�ʵ�������ֵ
//
//	//����kalman����
//	kalmanInfo->kalmanGain = kalmanInfo->P*kalmanInfo->H / (kalmanInfo->P*kalmanInfo->H*kalmanInfo->H + kalmanInfo->R);  //Kg(k)= P(k|k-1) H�� / (H P(k|k-1) H�� + R)
//	//����������������˲�ֵ
//	kalmanInfo->filterValue = predictValue + (lastMeasurement - predictValue)*kalmanInfo->kalmanGain;  //���ò������Ϣ���ƶ�x(t)�Ĺ��ƣ�����������ƣ����ֵҲ�������  X(k|k)= X(k|k-1)+Kg(k) (Z(k)-H X(k|k-1))
//	//���º������
//	kalmanInfo->P = (1 - kalmanInfo->kalmanGain*kalmanInfo->H)*kalmanInfo->P;//������������  P[n|n]=(1-K[n]*H)*P[n|n-1]
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


pcl::PointNormal Fusion::NormalFusion(const std::vector<int> & vPointIdx, const pcl::PointCloud<pcl::PointNormal> & vCloudNormal){

	//Initialize output
	pcl::PointNormal oOnePN;
	oOnePN.x = 0.0f;
	oOnePN.y = 0.0f;
	oOnePN.z = 0.0f;
	oOnePN.normal_x = 0.0f;
	oOnePN.normal_y = 0.0f;
	oOnePN.normal_z = 0.0f;

	//linear increase
	for (int i = 0; i != vPointIdx.size(); ++i){

		int iVoxelPIdx = vPointIdx[i];
	
		oOnePN.x = oOnePN.x + vCloudNormal.points[iVoxelPIdx].x;
		oOnePN.y = oOnePN.y + vCloudNormal.points[iVoxelPIdx].y;
		oOnePN.z = oOnePN.z + vCloudNormal.points[iVoxelPIdx].z;
		oOnePN.normal_x = oOnePN.normal_x + vCloudNormal.points[iVoxelPIdx].normal_x;
		oOnePN.normal_y = oOnePN.normal_y + vCloudNormal.points[iVoxelPIdx].normal_y;
		oOnePN.normal_z = oOnePN.normal_z + vCloudNormal.points[iVoxelPIdx].normal_z;
	
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

	return oOnePN;


}

pcl::PointNormal Fusion::NormalFusionWeighted(const std::vector<int> & vPointIdx, pcl::PointCloud<pcl::PointNormal> & vCloudNormal){

	//Initialize output
	pcl::PointNormal oOnePN;
	oOnePN.x = 0.0f;
	oOnePN.y = 0.0f;
	oOnePN.z = 0.0f;
	oOnePN.normal_x = 0.0f;
	oOnePN.normal_y = 0.0f;
	oOnePN.normal_z = 0.0f;
	float& all_weight = oOnePN.data_n[3];

	//linear increase
	for (int i = 0; i != vPointIdx.size(); ++i){

		int iVoxelPIdx = vPointIdx[i];
		float point_weight = vCloudNormal.points[iVoxelPIdx].data_n[3];
	
		oOnePN.x = oOnePN.x + vCloudNormal.points[iVoxelPIdx].x;
		oOnePN.y = oOnePN.y + vCloudNormal.points[iVoxelPIdx].y;
		oOnePN.z = oOnePN.z + vCloudNormal.points[iVoxelPIdx].z;
		oOnePN.normal_x = oOnePN.normal_x + point_weight * vCloudNormal.points[iVoxelPIdx].normal_x;
		oOnePN.normal_y = oOnePN.normal_y + point_weight * vCloudNormal.points[iVoxelPIdx].normal_y;
		oOnePN.normal_z = oOnePN.normal_z + point_weight * vCloudNormal.points[iVoxelPIdx].normal_z;
		all_weight = all_weight + point_weight;
	}

	
	float fNum = float(vPointIdx.size());

	if (vPointIdx.size()){
		//take the mean
		oOnePN.x = oOnePN.x / fNum;
		oOnePN.y = oOnePN.y / fNum;
		oOnePN.z = oOnePN.z / fNum;
		oOnePN.normal_x = oOnePN.normal_x / all_weight;
		oOnePN.normal_y = oOnePN.normal_y / all_weight;
		oOnePN.normal_z = oOnePN.normal_z / all_weight;
	}

	// spread back to the cloud normal
	for (int i = 0; i != vPointIdx.size(); ++i){

		int iVoxelPIdx = vPointIdx[i];
		float point_weight = vCloudNormal.points[iVoxelPIdx].data_n[3];
		vCloudNormal.points[iVoxelPIdx].normal_x = all_weight * oOnePN.normal_x + point_weight * vCloudNormal.points[iVoxelPIdx].normal_x;
		vCloudNormal.points[iVoxelPIdx].normal_y = all_weight * oOnePN.normal_y + point_weight * vCloudNormal.points[iVoxelPIdx].normal_y;
		vCloudNormal.points[iVoxelPIdx].normal_z = all_weight * oOnePN.normal_z + point_weight * vCloudNormal.points[iVoxelPIdx].normal_z;

		vCloudNormal.points[iVoxelPIdx].normal_x /= (all_weight + point_weight);
		vCloudNormal.points[iVoxelPIdx].normal_y /= (all_weight + point_weight);
		vCloudNormal.points[iVoxelPIdx].normal_z /= (all_weight + point_weight);

		MeshOperation m;
		m.VectorNormalization(vCloudNormal.points[iVoxelPIdx].normal_x, vCloudNormal.points[iVoxelPIdx].normal_y, vCloudNormal.points[iVoxelPIdx].normal_z);
	}

	return oOnePN;


}