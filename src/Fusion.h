#ifndef FUSION_H
#define FUSION_H

#include<cmath>
#include<vector>
#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/kdtree/kdtree.h>

#include "NormalFeatures.h"//2022.10.08

// һά�˲�����Ϣ�ṹ��
typedef  struct{
	double filterValue;  //k-1ʱ�̵��˲�ֵ������k-1ʱ�̵�ֵ
	double kalmanGain;   //   Kalamn����
	double A;   // x(n)=A*x(n-1)+u(n),u(n)~N(0,Q)
	double H;   // z(n)=H*x(n)+w(n),w(n)~N(0,R)
	double Q;   //Ԥ���������ƫ��ķ���
	double R;   //��������ƫ�(ϵͳ����Ժ�ͨ������ͳ��ʵ����)
	double P;   //�������Э����
}  KalmanInfo;
/**
* @brief Init_KalmanInfo   ��ʼ���˲����ĳ�ʼֵ
* @param info  �˲���ָ��
* @param Q Ԥ���������� ��ϵͳ�ⲿ�ⶨ����
* @param R ������������ ��ϵͳ�ⲿ�ⶨ����
*/
//void Init_KalmanInfo(KalmanInfo* info, double Q, double R)
//{
//	info->A = 1;  //����������
//	info->H = 1;  //
//	info->P = 10;  //����״̬����ֵ���ķ���ĳ�ʼֵ����ҪΪ0���ⲻ��
//	info->Q = Q;    //Ԥ�⣨���̣��������� Ӱ���������ʣ����Ը���ʵ���������
//	info->R = R;    //�������۲⣩�������� ����ͨ��ʵ���ֶλ��
//	info->filterValue = 0;// �����ĳ�ʼֵ
//};

//double KalmanFilter(KalmanInfo* kalmanInfo, double lastMeasurement);


class Fusion{

public:

	Fusion();
	~Fusion();

	//set node weight for kinect fusion mode
	void SetKinectMode(const int & iNodeNum, float fRawWeight = 1.0f);

	//set map size 
	void SetAccDisSize(const int & iNodeNum, float fRawDis = 0.0f);

	//fusion using kinect fusion mode - Weighted average
	//if vWeights and fRawWeight is the constant 1, respectively
	//then it is a mean least squares
	void KinectFusion(const std::vector<float> & vCurrentDis, std::vector<float> & vWeights);

	//
	void ConvexBasedFusion(const std::vector<float> & vCurrentDis, std::vector<float> & vDisMap);

	//
	void UnionMinimalFusion(const std::vector<float> & vCurrentDis);

	//fuses normal vector
	float NormalFusion(const std::vector<int> & vPointIdx, const pcl::PointCloud<pcl::PointNormal> & vCloudNormal, pcl::PointNormal & oOutPointNormal);
	void NormalFusion(const std::vector<std::vector<int>> & vPointIdxs, const std::vector<int> & vVoxelIdxs, const pcl::PointCloud<pcl::PointNormal> & vCloudNormal, pcl::PointNormal & oOutPointNormal);
	void NormalFusion(const std::vector<int> & vVoxelIdxs, const std::vector<float> & vFeatures, pcl::PointCloud<pcl::PointNormal>::Ptr & pVoxelNormal);

	//*******data********
	std::vector<float> m_vAccDis;

private:

	std::vector<float> m_vKinectWeight;

};



#endif