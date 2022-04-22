//#include "stdafx.h"

#include "LasOperator.h"
#include <iostream>
#include <math.h> 

using namespace std;

/*===================================================================
Constructor of CLasOperator
Initialization.
===================================================================*/

 CLasOperator:: CLasOperator(){
	 isLas = false;
	 avail = false;
}

/*===================================================================
Destructor of CLasOperator
Free space.
===================================================================*/

 CLasOperator::~ CLasOperator(){

}

/*===================================================================
CLasOperator::readLasFile
1. Open an LAS file specified by the parameter file_name.
2. Read the data block corresponding to public header block and store it.
3. Read the data block corresponding to variable length record headers and store them.
4. Read the point data record and store them.
===================================================================*/

bool  CLasOperator::readLasFile(const char *file_name){

	clearData();
	filename.assign(file_name);

	// Open an Las File
	fstream file;
	file.open(file_name, ios::in | ios::binary);
	if(file.fail()) {
		file.close();
		return false;
	}

	// Read public header block
	if(!readPulicHeaderBlock(file)) {
		file.close();
		return false;
	}

	// Read variable length record headers
	if(!readVariableLengthRecords(file)) {
		file.close();
		return false;		
	}
	
	// Read point data records
	if(!readPointDataRecords(file)){
		file.close();
		return false;
	}

	file.close();
	isLas = true;
	avail = true;

	return true;
}

/*===================================================================
CLasOperator::readXYZFile
1. Open an XYZ file specified by the parameter file_name.
2. Read the data points with x,y,z coordinations.
3. Store the coordinations into point3d.
===================================================================*/

bool  CLasOperator::readXYZFile(const char *file_name){

	clearData();
	filename.assign(file_name);

	// Open an XYZ file
	fstream file;
	file.open(file_name,ios::in);
	if(file.fail()){
		file.close();
		return false;
	}

	Point3D point;
	// Read data points
	while(!file.eof()){
		file>>point.x;
		file>>point.y;
		file>>point.z;
		point3d.push_back(point);
	}
	point3d.pop_back();
	file.close();
	isLas = false;
	avail = true;

	return true;
}

/*===================================================================
CLasOperator::exportData

===================================================================*/

bool  CLasOperator::exportData(const char *file_name,
        bool RGB,                                 // RBG value
		bool intensity,                           // Intensity value
		bool returnNum,                           // Return number
		bool numOfReturns,                        // Number of returns (given pulse)
		bool scandir,                             // Scan direction
		bool flightline,                          // Edge of the flight line
		bool classification,                      // Classification
		bool GPS){                                // GPS time

	// Open a file
	fstream file;
	file.open(file_name,ios::out);
	if(file.fail()){
		file.close();
		return false;
	}

	for(size_t i=0;i<pHeader.pointRecordNum;++i){

		file<<fixed<<point3d[i].x<<"\t"<<point3d[i].y<<"\t"<<point3d[i].z;

		if(RGB)
			file<<fixed<<"\t"<<pointData[i].red<<"\t"<<pointData[i].green<<"\t"<<pointData[i].blue;
		if(intensity)
			file<<fixed<<"\t"<<pointData[i].intensity;
		if(returnNum)
			file<<fixed<<"\t"<<(pointData[i].mask & 0x07);
		if(numOfReturns)
			file<<fixed<<"\t"<<((pointData[i].mask & 0x38)>>3);
		if(scandir)
			file<<fixed<<"\t"<<((pointData[i].mask & 0x40)>>6);
		if(flightline)
			file<<fixed<<"\t"<<((pointData[i].mask & 0x80)>>7);
		if(classification)
			file<<fixed<<"\t"<<(short)pointData[i].classification;
		if(GPS)
			file<<fixed<<"\t"<<pointData[i].GPS;
		file<<endl;
	}

	file.close();
	return true;
}

/*===================================================================
CLasOperator::lasSampling
Sample the point cloud data specified by parameter inter which is the
sampling interval and store into a las file.
===================================================================*/

bool CLasOperator::lasSampling(const char *file_name, unsigned long inter){

	// Open a file
	fstream file;
	file.open(file_name,ios::out|ios::binary);
	if(file.fail()){
		file.close();
		return false;
	}

	PublicHeaderBlock ph;

	memcpy(&ph,&pHeader,sizeof(PublicHeaderBlock));
	ph.pointRecordNum = pHeader.pointRecordNum/inter; // Set the sampled point record number
	ph.maxX = pointData[0].x;
	ph.minX = pointData[0].x;
	ph.maxY = pointData[0].y;
	ph.minY = pointData[0].y;
	ph.maxZ = pointData[0].z;
	ph.minZ = pointData[0].z;
	for(size_t i=inter;i<pointData.size();i+=inter){ // Compute the min and max of x,y and z
		if(ph.maxX<pointData[i].x)
			ph.maxX = pointData[i].x;
		if(ph.minX>pointData[i].x)
			ph.minX = pointData[i].x;
		if(ph.maxY<pointData[i].y)
			ph.maxY = pointData[i].y;
		if(ph.minY>pointData[i].y)
			ph.minY = pointData[i].y;
		if(ph.maxZ<pointData[i].z)
			ph.maxZ = pointData[i].z;
		if(ph.minZ>pointData[i].z)
			ph.minZ = pointData[i].z;
	}
	// Compute the offsetToData
	ph.offsetToData = ph.headerSize+varHeader.size()*sizeof(VariableLengthRecordHeader);
	// Write the public header block into file
	file.write((const char *)&ph,sizeof(PublicHeaderBlock));
	// Write the variable length headers into file
	for(size_t i=0;i<varHeader.size();++i){
		file.write((const char *)&varHeader[i],sizeof(VariableLengthRecordHeader));
	}
	// Write the data point record into file
	for(size_t i=0;i<pointData.size();i+=inter){
		file.write((const char *)&pointData[i],20);
		//Distinguish different point data record format
		switch(pHeader.pointRecordLen){
		case 28:
			file.write((const char *)&pointData[i].GPS,sizeof(double));
			break;
		case 26:
			file.write((const char *)&pointData[i].red, 3*sizeof(unsigned short));
			break;
		case 34:
			file.write((const char *)&pointData[i].GPS, sizeof(double)+3*sizeof(unsigned short));
			break;
		}
	}
	file.close();
	return true;
}

/*===================================================================
CLasOperator::xyzSampling
Sample the point cloud data specified by parameter inter which is the
sampling interval and store into a xyz file.
===================================================================*/

bool CLasOperator::xyzSampling(const char *file_name, unsigned long inter){
	// Open a file
	fstream file;
	file.open(file_name,ios::out);
	if(file.fail()){
		file.close();
		return false;
	}
	for(size_t i=0;i<point3d.size();i+=inter){
		file<<fixed<<point3d[i].x<<"\t"<<point3d[i].y<<"\t"<<point3d[i].z<<endl;
	}
	file.close();
	return true;
}

/*===================================================================
CLasOperator::pointNum
Return the number of the data points.
===================================================================*/

unsigned long CLasOperator::pointNum(){ 
	return point3d.size();
}

/*===================================================================
CLasOperator::getPointData
Return the vector containing the data point coordinations.
===================================================================*/

std::vector<Point3D>& CLasOperator::getPointData(){                
	return point3d;
}

/*===================================================================
CLasOperator::getPointRecords
Return the vector containing the data point records.
===================================================================*/

std::vector<PointDataRecord>& CLasOperator::getPointRecords(){  
	return pointData;
}

/*===================================================================
CLasOperator::getPublicHeader
Return the structure of the public header block.
===================================================================*/

PublicHeaderBlock& CLasOperator::getPublicHeader(){
	return pHeader;
}

/*===================================================================
CLasOperator::isLasFile
Return a boolean value indicating whether the imported file is an 
las file or an XYZ file.
===================================================================*/

bool CLasOperator::isLasFile(){
	return isLas;
}

/*===================================================================
CLasOperator::available
Return a boolean value indicating whether an las file or an XYZ file
has been imported into the structure.
===================================================================*/

bool CLasOperator::available(){
	return avail;
}

/*===================================================================
CLasOperator::clearData
Clear all the information about the imported file.
===================================================================*/

void CLasOperator::clearData(){
	varHeader.clear();
	pointData.clear();
	point3d.clear();
	isLas = false;
	avail = false;
}

/*===================================================================
CLasOperator::readPublicHeaderBlock
1. Read the data block corresponding to public header block.
2. Store the information into pHeader
3. Check the validity of the LAS file
===================================================================*/

bool  CLasOperator::readPulicHeaderBlock(fstream &file){

	char *pHeaderBuf = new char[sizeof(PublicHeaderBlock)];
	file.read(pHeaderBuf, sizeof(PublicHeaderBlock));
	memcpy(&pHeader, pHeaderBuf,sizeof(PublicHeaderBlock));
	delete [] pHeaderBuf;
	//if(strcmp(pHeader.fileSign, "LASF") != 0) {
		//return false;
	//}
	return true;
}

/*===================================================================
CLasOperator::readVariableLengthRecords
1. Read the data block corresponding to variable length record headers iteratively
2. Store them into varHeader
===================================================================*/

bool  CLasOperator::readVariableLengthRecords(fstream &file){

	size_t varHeaderSize = sizeof(VariableLengthRecordHeader);
	char *varHeaderBuf = new char[varHeaderSize];
	VariableLengthRecordHeader pvarHeader;
	unsigned long startPoint = pHeader.headerSize;

	for(unsigned long i=0;i<pHeader.varRecordNum;++i){
		file.seekg(startPoint);
		file.read(varHeaderBuf, varHeaderSize);
		memcpy(&pvarHeader, varHeaderBuf, varHeaderSize);
		startPoint += pvarHeader.recordLength + varHeaderSize;
		varHeader.push_back(pvarHeader);
	}

	delete [] varHeaderBuf;
	return true;
}

/*===================================================================
CLasOperator::readPointDataRecords

===================================================================*/

bool  CLasOperator::readPointDataRecords(fstream &file){

	file.seekg(pHeader.offsetToData);
	PointDataRecord pointRecord;
	Point3D point;
	char *pRecord = new char[pHeader.pointRecordLen];

	for (unsigned long i=0;i<pHeader.pointRecordNum;++i){
		file.read(pRecord,pHeader.pointRecordLen);
		memcpy(&pointRecord,pRecord,20);

		//Distinguish different point data record format
		switch(pHeader.pointRecordLen){
		case 28:
			memcpy(&pointRecord.GPS, &pRecord[20],sizeof(double));
			break;
		case 26:
			memcpy(&pointRecord.red, &pRecord[20],3*sizeof(unsigned short));
			break;
		case 34:
			memcpy(&pointRecord.GPS, &pRecord[20],sizeof(double)+3*sizeof(unsigned short));
			break;
		}
		point.x = pointRecord.x*pHeader.xScale+pHeader.xOffset;
		point.y = pointRecord.y*pHeader.yScale+pHeader.yOffset;
		point.z = pointRecord.z*pHeader.zScale+pHeader.zOffset;
		pointData.push_back(pointRecord);
		point3d.push_back(point);
	}
	return true;
}
/*================================
HPDpointclouddataread�������
���ܣ�ĳ����������Ϣ�Ŀ��ӻ�����ɫ����
�����β�1��const std::string nameΪ�����ļ���
�����β�4��ѡ����las�ļ�����pcd�ļ�������las�ļ�Ϊ1ѡ�Ĭ�ϣ�������Ϊpcb
�����β�2��һ������ָ�����ڴ洢pointXYZ���͵�����(float)
�����β�3��һ�������洢point3d��������(double)
�������״̬��double�ͺ�float���������ڼ�¼����ÿ����ĵѿ���ϵ����λ��
=================================*/
void HPDpointclouddataread(const std::string name,pcl::PointCloud<pcl::PointXYZ>::Ptr & las_cloud,std::vector<Point3D> & point3d,int type){
	if (type==1)//��ȡ����las�ļ�
   {
	CLasOperator lasptr;
	//����las��ȡ����
	if(!lasptr.readLasFile(name.c_str())){
		std::cout<<"Read file failed!"<<std::endl;
	exit(0);
	}
	std::cout<<"Read file succeed!"<<std::endl;
	//��ȡ�����еĵ�����
	int num = lasptr.pointNum();
	point3d = lasptr.getPointData();
	//++++++++++++++++++++++++++תΪpcd++++++++++++++++++++++++++
// ����PCL����ƺ�PCD��ʽ
las_cloud->width=num;
las_cloud->height=1;
las_cloud->is_dense=false;
las_cloud->points.resize(las_cloud->width*las_cloud->height);
//��las���ݸ��Ƹ�pcl��
for(size_t i=0;i<num;++i)
{
las_cloud->points[i].x=float(point3d[i].x);
las_cloud->points[i].y=float(point3d[i].y);
las_cloud->points[i].z=float(point3d[i].z);
}
	}
	else//��PCD�ļ�
	{
		//*��PCD�ļ�	
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(name,*las_cloud)==-1){
PCL_ERROR("Couldn't read file test_pcd.pcd\n");
exit(0);
}
//����ʱֻҪpoint3d��������pcl��Ҫpointxyz��תΪpoint3d��ΪpointXYZ���Ȳ�������Ҫdouble�͵�
point3d.resize(las_cloud->points.size());
//��las���ݸ��Ƹ�point3d
for(size_t i=0;i!=point3d.size();++i)
{
point3d[i].x=double(las_cloud->points[i].x);
point3d[i].y=double(las_cloud->points[i].y);
point3d[i].z=double(las_cloud->points[i].z);
}//end for
   }//end small if
	std::cout<<"finish reading."<<std::endl;
}
/*======================================
Tranlasorpcdfile�������
pcl::PointCloud<pcl::PointXYZ>::Ptr��Point3D��ʽ��ת
ת������"PCLPointXYZtoPoint3D"��"Point3DtoPCLPointXYZ"
=======================��ҹ��������==================*/
void Tranlasorpcdfile(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,std::vector<Point3D> & point3d,char *formname){
	if (!strcmp(formname,"PCLPointXYZtoPoint3D")){
		point3d.clear();
		point3d.resize(cloud->points.size());
//��las���ݸ��Ƹ�point3d
for(size_t i=0;i!=point3d.size();++i)
{
point3d[i].x=double(cloud->points[i].x);
point3d[i].y=double(cloud->points[i].y);
point3d[i].z=double(cloud->points[i].z);
}//end for
}else if(!strcmp(formname,"Point3DtoPCLPointXYZ")){
cloud->clear();
cloud->width=point3d.size();
cloud->height=1;
cloud->is_dense=false;
cloud->points.resize(cloud->width*cloud->height);
//��las���ݸ��Ƹ�pcl��
for(size_t i=0;i<cloud->width;++i)
{
cloud->points[i].x=float(point3d[i].x);
cloud->points[i].y=float(point3d[i].y);
cloud->points[i].z=float(point3d[i].z);
}
	}else{
		std::cout<<"Something wrong happened on calling Tranlasorpcdfile function"<<std::endl;
		std::cin.get();
		exit(0);
	}
}
/*======================================
HPDpointcloudallread�������
���룺const std::string name �Ӻ�׺���ļ���
      pcl::PointCloud<pcl::PointXYZI>::Ptr & las_cloud PCD��ʽ�ĵ��� pcl��
	  std::vector<Point3DI> & pointall Point3DI��ʽ�ĵ��� las��
	  int type ��ȡpcd����las�ļ�
�������ǿ����Ϣ�ĵ���
=========================================*/
void HPDpointcloudallread(const std::string name,pcl::PointCloud<pcl::PointXYZI>::Ptr & las_cloud,std::vector<Point3DI> & pointall,int type){
	if (type==1)//��ȡ����las�ļ�
   {
	CLasOperator lasptr;
	//����las��ȡ����
	if(!lasptr.readLasFile(name.c_str())){
		std::cout<<"Read file failed!"<<std::endl;
	exit(0);
	}
	std::cout<<"Read file succeed!"<<std::endl;
	//���
	pointall.clear();
	//��ȡ�����еĵ�����
	int num = lasptr.pointNum();
	//��ȡPointDataRecord��ʽ�µ�intensity��ע�ø�ʽ�µ����꾫�ȼ���
	std::vector<PointDataRecord> pointrecords;
	pointrecords = lasptr.getPointRecords();
	//��ȡpoint3d�µ���ά����ֵ
	std::vector<Point3D> point3d;
	point3d=lasptr.getPointData();
	//�϶�Ϊһ
	Point3DI onepoint3di={0.0,0.0,0.0,0,0};
	for(size_t i=0;i!=point3d.size();i++){
		onepoint3di.x=point3d[i].x;
		onepoint3di.y=point3d[i].y;
		onepoint3di.z=point3d[i].z;
		onepoint3di.intensity=pointrecords[i].intensity;
		pointall.push_back(onepoint3di);
	}
	//+++++++++++++תΪpcd
    // ����PCL����ƺ�PCD��ʽ
    las_cloud->width=num;
    las_cloud->height=1;
    las_cloud->is_dense=false;
    las_cloud->points.resize(las_cloud->width*las_cloud->height);
    //��las���ݸ��Ƹ�pcl��
    for(size_t i=0;i<num;++i){
    las_cloud->points[i].x=float(pointall [i].x);
    las_cloud->points[i].y=float(pointall [i].y);
    las_cloud->points[i].z=float(pointall [i].z);
    las_cloud->points[i].intensity=float(pointall[i].intensity);
	}//end for
	
	//*************************��PCD�ļ���ģʽ**************************
	}else{
		//*��PCD�ļ�	
    if(pcl::io::loadPCDFile<pcl::PointXYZI>(name,*las_cloud)==-1){
       PCL_ERROR("Couldn't read file test_pcd.pcd\n");
       exit(0);
      }
//����ʱֻҪpointall��������pcl��ҪpointxyzI��תΪpointall��ΪpointXYZI���Ȳ�������Ҫdouble�͵�
       pointall.resize(las_cloud->points.size());
//��las���ݸ��Ƹ�pointall
       for(size_t i=0;i!=pointall.size();++i){
           pointall[i].x=double(las_cloud->points[i].x);
           pointall[i].y=double(las_cloud->points[i].y);
           pointall[i].z=double(las_cloud->points[i].z);
           pointall[i].intensity=unsigned short(las_cloud->points[i].intensity);}//end for
       }//end small if
	std::cout<<"finish reading."<<std::endl;
}