//
////������͵�ͷ�ļ�
//#include <pcl/point_types.h>
////�����ļ�IO��pcd�ļ���ply�ļ���
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
////kd��
//#include <pcl/kdtree/kdtree_flann.h>
////������ȡ
//#include <pcl/features/normal_3d_omp.h>
//#include <pcl/features/normal_3d.h>
////�ع�
//#include <pcl/surface/gp3.h>
//#include <pcl/surface/poisson.h>
////���ӻ�
//#include <pcl/visualization/pcl_visualizer.h>
////���߳�
//#include <boost/thread/thread.hpp>
//#include <fstream>
//#include <iostream>
//#include <stdio.h>
//#include <string.h>
//#include <string>
//#include"GHPR.h"
//#include"HpdPointCloudDisplay.h"
//
//
//int main(int argc, char** argv)
//{
//	
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	//
//	std::vector<Point3D> point3d;//Point3DĿ��������������
//	HPDpointclouddataread("bunny.las",cloud,point3d,1);
//
//	// ���㷨����
//	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>); //���������ƶ���ָ��
//	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//���߹��ƶ���
//	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//�洢���Ƶķ��ߵ�ָ��
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//	tree->setInputCloud(cloud);
//	n.setInputCloud(cloud);
//	n.setSearchMethod(tree);
//	n.setKSearch(5);
//	n.compute(*normals); //���㷨�ߣ�����洢��normals��
//
//	//�����ƺͷ��߷ŵ�һ��
//	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
//
//	//����������
//	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
//	tree2->setInputCloud(cloud_with_normals);
//	//����Poisson���󣬲����ò���
//	pcl::Poisson<pcl::PointNormal> pn;
//	pn.setConfidence(false); //�Ƿ�ʹ�÷������Ĵ�С��Ϊ������Ϣ�����false�����з���������һ����
//	pn.setDegree(2); //���ò���degree[1,5],ֵԽ��Խ��ϸ����ʱԽ�á�
//	pn.setDepth(8); //���������ȣ����2^d x 2^d x 2^d������Ԫ�����ڰ˲�������Ӧ�����ܶȣ�ָ��ֵ��Ϊ�����ȡ�
//	pn.setIsoDivide(8); //������ȡISO��ֵ����㷨�����
//	pn.setManifold(false); //�Ƿ���Ӷ���ε����ģ�����������ǻ�ʱ�� �������б�־���������Ϊtrue����Զ���ν���ϸ�����ǻ�ʱ������ģ�����false�����
//	pn.setOutputPolygons(false); //�Ƿ������������񣨶��������ǻ��ƶ�������Ľ����
//	pn.setSamplesPerNode(3.0); //��������һ���˲�������е����������С��������������[1.0-5.0],������[15.-20.]ƽ��
//	pn.setScale(1.25); //���������ع���������ֱ���������߽�������ֱ���ı��ʡ�
//	pn.setSolverDivide(8); //����������Է������Gauss-Seidel�������������
//	//pn.setIndices();
//
//	//���������������������
//	pn.setSearchMethod(tree2);
//	pn.setInputCloud(cloud_with_normals);
//	//����������������ڴ洢���
//	pcl::PolygonMesh mesh;
//	//ִ���ع�
//	pn.performReconstruction(mesh);
//
//	//��������ͼ
//	pcl::io::savePLYFile("mesh_possion.ply", mesh);
//
//	// ��ʾ���ͼ
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer"));
//	viewer->setBackgroundColor(0, 0, 0);
//	viewer->addPolygonMesh(mesh, "my");
//	//viewer->addCoordinateSystem(50.0);
//	viewer->initCameraParameters();
//	while (!viewer->wasStopped()){
//		viewer->spinOnce(100);
//		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//	}
//
//	return 0;
//}
//
