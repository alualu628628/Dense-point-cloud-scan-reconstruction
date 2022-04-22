#ifndef HPDPOINTCLOUDDISPLAY_H //defeat reconstruction of h.file
#define HPDPOINTCLOUDDISPLAY_H 
#include"LasOperator.h"
#include <string>
#include <vector>
#include <ctime>
#include <iostream>
#include <sstream>//for istringstream function

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>



//��ɫ��R,G,B
struct ColorRGB{
	float r;
	float g;
	float b;
	std::string name; 
};
/*====================================
һ��HPD��ʾ�����PCL�������δ��� 
Edited by huang Peng-di 2014.05.07
��Ҫ���ܣ����Ƶ�������ʾ
�ں���
1.������ʾ��2.�߳�ֵ��ʾ��3.����������ʾ��
4.������ʾ��5.����ֵ��ʾ��
����ͨ��Ϊ����las��pcd�ļ��Ĵ洢����
��Ӧ�����Ĳ������Ը���ʵ�������Ӧ����
����ο�ÿ��������˵�����ں��������Ϸ���
ע�⣺���๦�ܺ������ص���һ������ָ�룬����
��Ҫʵ�ֶ���һ��ָ�룬����ʾ����
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
HpdDisplay displayer;
viewer=displayer.Showclassification(point3d,"random");
������,����ʾ�������·���ֱ�ӿ�������
���ʹ�ã��⼸��������������Ӧ�Դ󲿷ֵ�����ʾҪ��
��ͨ������Ŀ�����ݣ�ת����ʽ���ں�����ת�����������ȣ�
====================================*/
class HpdDisplay{

public:
	//��
typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> PointCloudViewerPtr;
    //
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudXYZPtr;
	//******************��ʼ������**************************
HpdDisplay();
~HpdDisplay();
//������ɫ
bool Generatecolor();
//������ɫ�仯����
void HpdDisplay::Setcolorvariationrange(float hr=255.0,float hg=0.0,float hb=0.0,
	float tr=0.0,float tg=255.0,float tb=0.0);
//���ñ�����ɫ
void Setbackgroudcolor(float,float,float);
    //*******************���ܺ���***************************
//��ʾ���ƣ���ɫ��ʾ��
//��ʾ���Ƶĺ������е���ʾͬһ��ɫ����ɫ�ɵ�
PointCloudViewerPtr Showsimplecolor(PointCloudXYZPtr &,char *colorvalue="white");

//��ʾ�߳���Ϣ��Elevation��
//��ʾ���Ƶĸ߳���Ϣ���ɳ����������Խ��ٵ���������ʾ
PointCloudViewerPtr Showelevation(PointCloudXYZPtr & ,int sampleInterval=1);

//��ʾ�������
//������ʾ����ʾĳ���������Ϣ��������������ָ����
PointCloudViewerPtr Showneighboorcloud(PointCloudXYZPtr &, int type=1, float radius=0.3, int viewpoint=-1);
PointCloudViewerPtr Showneighboorcloud(PointCloudXYZPtr &, double f_x, double f_y, double f_z, float radius=0.3,int type=1 );

//��ʾ������Ϣ
//��ʾ���Ƶ���𣬼��۲���Ƶķ�����
PointCloudViewerPtr Showclassification(std::vector<Point3D> &, char *keyword="random");
PointCloudViewerPtr Showclassification(PointCloudXYZPtr &, std::vector<int> &, char *keyword = "random");
PointCloudViewerPtr Showclassification(pcl::PointCloud<pcl::PointXYZ> & vCloud, std::vector<int> & vClasses, char *keyword = "random");

//��ʾ��������
//����ÿ��ĵ�����ֵ����仯�ֲ���ʾ����ʾÿһ�����һ������ֵ����Ҫÿ���������ֵ����
//"gray","redgreen" ,"greenblue","redblue","free"����ģʽ
PointCloudViewerPtr Showfeatureresult(std::vector<Point3D> &,std::vector<double> &, char *keyword="redgreen");
PointCloudViewerPtr Showfeatureresult(PointCloudXYZPtr & , std::vector<float> & , char *keyword = "redgreen");
PointCloudViewerPtr Showfeatureresult(pcl::PointCloud<pcl::PointXYZ> &, std::vector<float> &, char *keyword = "redgreen");

//��ʾһЩ��������
PointCloudViewerPtr Creatgeometry(PointCloudXYZPtr &,Point3D &);

//show center points with lines
//L indicates lines with blue color 
//S indicates sphere (points) with red color
void AddLineWithPoints(boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer,
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & vSkelinePoints,
	float fRadius = 0.01, float fLr = 0.0, float fLg = 0.0, float fLb = 1.0,
	float fSr = 1.0, float fSg = 0.0, float fSb = 0.0);

void AddSphereAtPoints(boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer,
	pcl::PointCloud<pcl::PointXYZ>::Ptr & vSkelinePoints,
	float fRadius = 0.01, float fSr = 1.0, float fSg = 0.0, float fSb = 0.0);


//һ�������С��������������
double random(double,double);
void displaytable();

private:
//������ɫ���õĴ洢��
	std::vector<ColorRGB> colors;
	//colors������
	bool colorflag;
	bool colorvarflag;
	//������ɫ
	ColorRGB backgroudcolor;
	ColorRGB headcolor;
	ColorRGB tailcolor;
};
//��ɫ�仯��λ����
float Colorvar(float headnum,float tailnum,float proportion);
/*==================================Example,you can copy this down on your main cpp to run
One Example:
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
std::vector<Point3D> point3d;
HPDpointclouddataread("CornerXYZPCD.pcd",cloud, point3d,2);
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
HpdDisplay hpdisplay;
viewer=hpdisplay.Showsimplecolor(cloud,"yellow");
while (!viewer->wasStopped())
{
     viewer->spinOnce ();
}
=====================================*/

#endif
