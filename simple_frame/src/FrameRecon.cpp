#include "FrameRecon.h"
#include "OutputUtils.h"

#include <iostream>
#include <sstream>

/*************************************************
Function: FrameRecon
Description: constrcution function for FrameRecon class
Calls: all member functions
Called By: main function of project
Table Accessed: none
Table Updated: none
Input: node - a ros node class
     nodeHandle - a private ros node class
*************************************************/
FrameRecon::FrameRecon(ros::NodeHandle & node,
                       ros::NodeHandle & nodeHandle):
                       m_iTrajFrameNum(0),m_dAverageReconstructTime(0),m_iReconstructFrameNum(0),m_dMaxReconstructTime(0),node(node),nodeHandle(nodeHandle){
}

void FrameRecon::LazyLoading() {

	//read parameters
	ReadLaunchParams(nodeHandle);

	//***subscriber related*** 
	//subscribe (hear) the odometry information (trajectory)
	m_oOdomSuber = nodeHandle.subscribe(m_sInOdomTopic, 2, &FrameRecon::HandleTrajectory, this);	//记录运动信息到 m_vOdomHistory 的循环数组之中，并且 m_iTrajCount++

	//subscribe (hear) the point cloud topic 
	m_oCloudSuber = nodeHandle.subscribe(m_sInCloudTopic, 1, &FrameRecon::HandlePointClouds, this);	//在m_vMapPCN记录法向点集，发布Mesh主题

	//***publisher related*** 
	//publish point cloud after processing
	m_oCloudPublisher = nodeHandle.advertise<sensor_msgs::PointCloud2>(m_sOutCloudTopic, 1, true);	//暂无发布

  	//publish polygon constructed from one frame point cloud
	m_oMeshPublisher = nodeHandle.advertise<visualization_msgs::Marker>(m_sOutMeshTopic, 1);		//在接受到点云重建完之后， 被 PublishMeshs() 函数调用

    m_oAdditionalPointPublisher = nodeHandle.advertise<sensor_msgs::PointCloud2>(m_sAdditionalPointTopic, 1, true); //发布补充的点云
}

/*************************************************
Function: ~FrameRecon
Description: deconstrcution function for FrameRecon class
Calls: all member functions
Called By: main function of project
Table Accessed: none
Table Updated: none
Input: none
Output: a file storing the point clouds with correct normal for accurate reconstruction
*************************************************/

FrameRecon::~FrameRecon() {

	std::cout << std::format_yellow 
		<< "Reconstructed frame numbers: " << m_iReconstructFrameNum << ";\tTotal frame numbers : " << m_iTotalFrameNum << std::endl
		<< "Average recontime per frame: " << m_dAverageReconstructTime / m_iReconstructFrameNum << "ms"
		<< ";\t Max frame time: " << m_dMaxReconstructTime << "ms"
		<< std::format_white << std::endl;
	/*
	//define ouput ply file name
	m_sOutPCNormalFileName << m_sFileHead << "Map_PCNormal.ply"; 

    //output to the screen
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << "********************************************************************************" << std::endl;
	std::cout << "Please do not force closing the programe, the process is writing output PLY file." << std::endl;
	std::cout << "It may take times (Writing 500M file takes about 20 seconds in usual)." << std::endl;
	std::cout << "The output file is " << m_sOutPCNormalFileName.str() << std::endl;

	//output point clouds with computed normals to the files when the node logs out
	pcl::io::savePLYFileASCII(m_sOutPCNormalFileName.str(), m_vMapPCN);

	std::cout << "Output is complete! The process will be automatically terminated. Thank you for waiting. " << std::endl;
	*/

}



/*************************************************
Function: ReadLaunchParams
Description: read the parameter value from ros launch file (mapping.launch)
Calls: all member functions
Called By: main function of project
Table Accessed: none
Table Updated: none
Input: nodeHandle - a private ros node class
Output: the individual parameter value for system
Return: none
Others: none
*************************************************/

bool FrameRecon::ReadLaunchParams(ros::NodeHandle & nodeHandle) {

  //output file name
  nodeHandle.param("file_outputpath", m_sFileHead, std::string("./"));

  //input odom topic
  nodeHandle.param("odom_in_topic", m_sInOdomTopic, std::string("/odometry/filtered"));

  //input point cloud topic
  nodeHandle.param("cloud_in_topic", m_sInCloudTopic, std::string("/cloud_points"));


  //input odom topic
  nodeHandle.param("cloud_out_topic", m_sOutCloudTopic, std::string("/processed_clouds"));

  //input point cloud topic
  nodeHandle.param("outcloud_tf_id", m_sOutCloudTFId, std::string("camera_init"));

  //input odom topic
  nodeHandle.param("polygon_out_topic", m_sOutMeshTopic, std::string("/processed_clouds"));

  //input point cloud topic
  nodeHandle.param("polygon_tf_id", m_sOutMeshTFId, std::string("camera_init"));

  //point cloud sampling number
  nodeHandle.param("sample_pcframe_num", m_iFrameSmpNum, 1);

  //point cloud sampling number
  nodeHandle.param("sample_inputpoints_num", m_iSampleInPNum, 1);

  //height of viewpoint
  double dViewZOffset;
  nodeHandle.param("viewp_zoffset", dViewZOffset, 0.0);
  m_fViewZOffset = float(dViewZOffset);
  
  //explicit reconstruction related
  //number of sectors
  nodeHandle.param("sector_num", m_iSectorNum, 1);
  m_oExplicitBuilder.HorizontalSectorSize(m_iSectorNum);

  //count processed point cloud frame
  m_iPCFrameCount = 0;

  //count processed odom frame
  m_iTrajCount = 0;

  //true indicates the file has not been generated
  m_bOutPCFileFlag = true;

  return true;

}


/*************************************************
Function: PublishPointCloud
Description: publish point clouds (mainly used for display and test)
Calls: none
Called By: ComputeConfidence()
Table Accessed: none
Table Updated: none
Input: vCloud - a point clouds to be published
Output: none
Return: none
Others: none
*************************************************/
void FrameRecon::PublishPointCloud(const pcl::PointCloud<pcl::PointXYZI> & vCloud){
  
	//publish obstacle points
	sensor_msgs::PointCloud2 vCloudData;

	pcl::toROSMsg(vCloud, vCloudData);

	vCloudData.header.frame_id = m_sOutCloudTFId;

	vCloudData.header.stamp = ros::Time::now();

	m_oCloudPublisher.publish(vCloudData);

}


/*************************************************
Function: PublishPointCloud
Description: publish point clouds (mainly used for display and test)
Calls: none
Called By: ComputeConfidence()
Table Accessed: none
Table Updated: none
Input: vCloud - a point clouds with its corresponding normals for publication
Output: none
Return: none
Others: none
*************************************************/
void FrameRecon::PublishPointCloud(const pcl::PointCloud<pcl::PointNormal> & vCloudNormal){

    //convert to pc2 message
	sensor_msgs::PointCloud2 vCloudData;

	pcl::toROSMsg(vCloudNormal, vCloudData);

	//other informations
	vCloudData.header.frame_id = m_sOutCloudTFId;

	vCloudData.header.stamp = ros::Time::now();

	//publish
	m_oCloudPublisher.publish(vCloudData);

}

/*************************************************
Function: PublishPointCloud
Description: publish point clouds (mainly used for display and test)
Calls: none
Called By: ComputeConfidence()
Table Accessed: none
Table Updated: none
Input: vCloud - a point clouds to be published
Output: none
Return: none
Others: none
*************************************************/
void FrameRecon::PublishPointCloud(const pcl::PointCloud<pcl::PointXYZI> & vCloud, const std::vector<float> & vFeatures){
  
	//get colors
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr  pColorClouds (new pcl::PointCloud<pcl::PointXYZRGB>);

	//to each point
	for (int i = 0; i <  vCloud.points.size(); ++i){

		pcl::PointXYZRGB oColorP;
		oColorP.x = vCloud.points[i].x;
		oColorP.y = vCloud.points[i].y;
		oColorP.z = vCloud.points[i].z;

		oColorP.r =  (1.0-vFeatures[i])*255.0f;
		oColorP.g =  vFeatures[i]*255.0f;
		oColorP.b =  (1.0-vFeatures[i])*255.0f; 
		pColorClouds->points.push_back(oColorP);
	}

    pColorClouds->width = 1;

    pColorClouds->height = vCloud.points.size();

    //convert to pc2 message
	sensor_msgs::PointCloud2 vCloudData;

	pcl::toROSMsg(*pColorClouds, vCloudData);

	//other informations
	vCloudData.header.frame_id = m_sOutCloudTFId;

	vCloudData.header.stamp = ros::Time::now();

	//publish
	m_oCloudPublisher.publish(vCloudData);

}

template<class T>
void FrameRecon::PublishPointCloud(pcl::PointCloud<T>& pointcloud, ros::Publisher& publisher)
{
    sensor_msgs::PointCloud2 vCloudData;

	pcl::toROSMsg(pointcloud, vCloudData);

	vCloudData.header.frame_id = m_sOutCloudTFId;

	vCloudData.header.stamp = ros::Time::now();

	publisher.publish(vCloudData);
}
template void FrameRecon::PublishPointCloud(pcl::PointCloud<pcl::PointXYZI>& pointcloud, ros::Publisher& publisher);
template void FrameRecon::PublishPointCloud(pcl::PointCloud<pcl::PointNormal>& pointcloud, ros::Publisher& publisher);

/*************************************************
Function: PublishPointCloud
Description: publish point clouds (mainly used for display and test)
Calls: none
Called By: ComputeConfidence()
Table Accessed: none
Table Updated: none
Input: vCloud - a point clouds to be published
Output: none
Return: none
Others: none
*************************************************/
void FrameRecon::PublishMeshs(){
  	
  	//new a visual message
	visualization_msgs::Marker oMeshMsgs;
	
	//define header of message
	oMeshMsgs.header.frame_id = m_sOutMeshTFId;
	oMeshMsgs.header.stamp = ros::Time::now();
	oMeshMsgs.type = visualization_msgs::Marker::TRIANGLE_LIST;
	oMeshMsgs.action = visualization_msgs::Marker::ADD;

	oMeshMsgs.scale.x = 1.0;
	oMeshMsgs.scale.y = 1.0;
	oMeshMsgs.scale.z = 1.0;

	oMeshMsgs.pose.position.x = 0.0;
	oMeshMsgs.pose.position.y = 0.0;
	oMeshMsgs.pose.position.z = 0.0;

	oMeshMsgs.pose.orientation.x = 0.0;
	oMeshMsgs.pose.orientation.y = 0.0;
	oMeshMsgs.pose.orientation.z = 0.0;
	oMeshMsgs.pose.orientation.w = 1.0;

	std_msgs::ColorRGBA color;
	color.a = 1;
	color.r = 1.0;
	color.g = 1.0;
	color.b = 1.0;

	//repeatable vertices
	pcl::PointCloud<pcl::PointXYZI> vMeshVertices;

	//get the reconstruted mesh
	m_oExplicitBuilder.OutputAllMeshes(vMeshVertices);

	//convert to publishable message
	for (int k = 0; k < vMeshVertices.points.size(); ++k){

		//temp point
    	geometry_msgs::Point oPTemp;
        oPTemp.x = vMeshVertices.points[k].x;
        oPTemp.y = vMeshVertices.points[k].y;
        oPTemp.z = vMeshVertices.points[k].z;

        //color
        oMeshMsgs.points.push_back(oPTemp);
        oMeshMsgs.color = color;

	}//end k

	m_oMeshPublisher.publish(oMeshMsgs);

}

std::ostream& operator<<(std::ostream& out, const sensor_msgs::PointCloud2::_header_type& header) {
	out << header.frame_id << ", " << header.seq << ", " << header.stamp;
	return out;
}
/*************************************************
Function: HandleRightLaser
Description: a callback function in below:
node.subscribe(m_sLaserTopic, 5, &GroundExtraction::HandlePointClouds, this);
Calls: CheckTruthPoint
Called By: TransformLaserInOdom, which is the construction function
Table Accessed: none
Table Updated: none
Input: rawpoint, a 3d point with pcl point type
Output: a point clouds are almost the same with raw point clouds but only their timestamp values are modified
Return: none
Others: none
*************************************************/
void FrameRecon::HandlePointClouds(const sensor_msgs::PointCloud2 & vLaserData)
{

	if (!(m_iPCFrameCount % m_iFrameSmpNum)){ //根据帧采样频率记录

		std::cout << "Now frame count is: " << m_iPCFrameCount << ";\t"
			<< "header is: {" << vLaserData.header << "}";
		m_iTotalFrameNum = vLaserData.header.seq;

		//开始算法计时
		clock_t start_time = clock();

		////a point clouds in PCL type
		pcl::PointCloud<pcl::PointXYZI>::Ptr pRawCloud(new pcl::PointCloud<pcl::PointXYZI>);
		////message from ROS type to PCL type
		pcl::fromROSMsg(vLaserData, *pRawCloud);

		//if have corresponding trajectory point (viewpoint)
		pcl::PointXYZI oCurrentViewP;

		if (m_vOdomHistory.size() /*&& vLaserData.header.stamp <= m_vOdomHistory.last().oTimeStamp*/){
			//
			oCurrentViewP = ComputeQueryTraj(vLaserData.header.stamp);	//当前点云对应的观测位置（Odom与frame并非一一对应，因此需要计算插值）

			std::cout << "\tView: " << oCurrentViewP;
		
		//else waiting for sync
		}else{

			std::cout << std::format_red << " Error: No odom matched!" << std::format_white << std::endl;

			return;
		}

		std::cout << ";\tsize: " << pRawCloud->size();
		
		pcl::PointCloud<pcl::PointXYZI>::Ptr pSceneCloud(new pcl::PointCloud<pcl::PointXYZI>);
		SamplePoints(*pRawCloud, *pSceneCloud, m_iSampleInPNum);
		
		pcl::PointCloud<pcl::PointNormal>::Ptr pFramePNormal(new pcl::PointCloud<pcl::PointNormal>);

		m_oExplicitBuilder.setWorkingFrameCount(m_iPCFrameCount);
		m_oExplicitBuilder.SetViewPoint(oCurrentViewP, m_fViewZOffset);
		m_oExplicitBuilder.FrameReconstruction(*pSceneCloud, *pFramePNormal);	//得到带法向的点云

		//************output value******************
		// for(int i=0;i!=pFramePNormal->points.size();++i)
		// 	m_vMapPCN.points.push_back(pFramePNormal->points[i]);
		
		//************additional points**************
        pcl::PointCloud<pcl::PointNormal> vAdditionalPoints;
        pcl::PointCloud<pcl::PointXYZI> vDisplayAdditionalPoints;
		for(int i = 0; i < m_oExplicitBuilder.m_vAllSectorClouds.size(); ++i) {
			MeshSample::GetAdditionalPointCloud(
				*(m_oExplicitBuilder.m_vAllSectorClouds[i]) , m_oExplicitBuilder.m_vAllSectorFaces[i], 
				m_oExplicitBuilder.m_vFaceWeight[i], m_oExplicitBuilder.m_vMatNormal[i],
				vAdditionalPoints, vDisplayAdditionalPoints
			);
		}

		PublishPointCloud(vDisplayAdditionalPoints, m_oAdditionalPointPublisher);

		// *pFramePNormal += vAdditionalPoints;
        vAdditionalPoints.is_dense = false;
        PublishPointCloud(vAdditionalPoints, m_oCloudPublisher);

		PublishMeshs();	//发布 m_oExplicitBuilder 中建立的 mesh

		PublishPointCloud(*pFramePNormal);

		/*output the points and normals
		{
			std::stringstream ss;
			ss << "../Dense_ROS/save/FramePNormal_" << m_iPCFrameCount << ".ply";
			pcl::io::savePLYFileASCII(ss.str(), *pFramePNormal);
		}
		//*/

		//clear this frame result
		m_oExplicitBuilder.ClearData();

		//结束算法计时并记录执行时间
		clock_t frame_reconstruct_time = 1000.0 * (clock() - start_time) / CLOCKS_PER_SEC;
		std::cout << ";\ttime:" << frame_reconstruct_time << "ms" << std::endl;
		m_dAverageReconstructTime += frame_reconstruct_time;
		m_dMaxReconstructTime = frame_reconstruct_time > m_dMaxReconstructTime ? frame_reconstruct_time : m_dMaxReconstructTime;
		++m_iReconstructFrameNum;
	}

	//count
	m_iPCFrameCount++;

	return;

}



/*************************************************
Function: HandleTrajectory
Description: a callback function in below:
m_oOdomSuber = node.subscribe(m_sOdomTopic, 5, &GroundExtraction::HandleTrajectory, this);
Calls: none
Called By: TransformLaserInOdom, which is the construction function
Table Accessed: none
Table Updated: none
Input: rawpoint, a 3d point with pcl point type
Output: a point clouds are almost the same with raw point clouds but only their timestamp values are modified
Return: none
Others: none
*************************************************/	
void FrameRecon::HandleTrajectory(const nav_msgs::Odometry & oTrajectory)
{

	std::cout << std::format_yellow << "Now Odome count is: " << m_iTrajCount << ";\t"
		<< "header is: {" << oTrajectory.header << "}" << "\tPose: (" << oTrajectory.pose.pose.position.x << ","
		<< oTrajectory.pose.pose.position.y << "," << oTrajectory.pose.pose.position.z << ")" << std::format_white << std::endl;

	//count input frames
	//m_iTrajPointNum++;

	//save the into the memory
	//save the position of trajectory
	RosTimePoint oOdomPoint;
	oOdomPoint.oLocation.x = oTrajectory.pose.pose.position.x;
	oOdomPoint.oLocation.y = oTrajectory.pose.pose.position.y;
	oOdomPoint.oLocation.z = oTrajectory.pose.pose.position.z;

	//save record time
	oOdomPoint.oTimeStamp = oTrajectory.header.stamp;

	m_vOdomHistory.push(oOdomPoint);

	m_iTrajCount++;

}

/*************************************************
Function: InterpolateTraj
Description: a callback function in below:
m_oOdomSuber = node.subscribe(m_sOdomTopic, 5, &GroundExtraction::HandleTrajectory, this);
Calls: none
Called By: TransformLaserInOdom, which is the construction function
Table Accessed: none
Table Updated: none
Input: rawpoint, a 3d point with pcl point type
Output: a point clouds are almost the same with raw point clouds but only their timestamp values are modified
Return: none
Others: none
*************************************************/
void FrameRecon::InterpolateTraj(const RosTimePoint & oCurrent, const RosTimePoint & oPast, const float& fRatio,
	pcl::PointXYZI & oInter){


	//The ratio is from the interpolated value to oCurrent value 
	//Complementary ratio
	float fCompRatio = 1 - fRatio;
	//p+(c-p)(1-r)
	oInter.x = oCurrent.oLocation.x * fCompRatio + oPast.oLocation.x * fRatio;
	oInter.y = oCurrent.oLocation.y * fCompRatio + oPast.oLocation.y * fRatio;
	oInter.z = oCurrent.oLocation.z * fCompRatio + oPast.oLocation.z * fRatio;

}

/*************************************************
Function: ComputeQueryTraj
Description: a callback function in below:
m_oOdomSuber = node.subscribe(m_sOdomTopic, 5, &GroundExtraction::HandleTrajectory, this);
Calls: none
Called By: TransformLaserInOdom, which is the construction function
Table Accessed: none
Table Updated: none
Input: rawpoint, a 3d point with pcl point type
Output: a point clouds are almost the same with raw point clouds but only their timestamp values are modified
Return: none
Others: none
*************************************************/
pcl::PointXYZI FrameRecon::ComputeQueryTraj(const ros::Time & oQueryTime){

	pcl::PointXYZI oResTraj;
	//clear the output
	oResTraj.x = 0.0;
	oResTraj.y = 0.0;
	oResTraj.z = 0.0;
	//index
	int iTrajIdx = 0;
	//time different
	double timeDiff = (oQueryTime - m_vOdomHistory[iTrajIdx].oTimeStamp).toSec();
	//search the most recent time
	while (iTrajIdx < m_vOdomHistory.size() - 1 && timeDiff > 0) {
		//increase index
		iTrajIdx++;
		//time different
		timeDiff = (oQueryTime - m_vOdomHistory[iTrajIdx].oTimeStamp).toSec();
	}

	//if the querytime is out of the stored time section 
	if (iTrajIdx == 0 || timeDiff > 0) {
		//turn back zero
		oResTraj.x = m_vOdomHistory[iTrajIdx].oLocation.x;
		oResTraj.y = m_vOdomHistory[iTrajIdx].oLocation.y;
		oResTraj.z = m_vOdomHistory[iTrajIdx].oLocation.z;

	}else {//if it is between two stored times
		//get the ratio
		//ROS_INFO("Trajtime between: %f and %f", m_vOdomHistory[iTrajIdx].oTimeStamp.toSec(), m_vOdomHistory[iTrajIdx - 1].oTimeStamp.toSec());

		float ratio = -timeDiff / (m_vOdomHistory[iTrajIdx].oTimeStamp - m_vOdomHistory[iTrajIdx - 1].oTimeStamp).toSec();
		//interpolate an accuracy value
		InterpolateTraj(m_vOdomHistory[iTrajIdx], m_vOdomHistory[iTrajIdx - 1], ratio, oResTraj);
	}

	return oResTraj;

}
  

/*=======================================
SamplePoints
Input: vCloud - the raw point clouds
vNewCloud - the sampled point clouds
iSampleNum - interval number or the maximum number
bIntervalSamp - interval number model (true) or the maximum number model (false)
Output: vNewCloud - the sampled point clouds
Function: Sample a point clouds. Note that tthe given maximum total number guarantes the number of sampled point is smaller than it
However, it is not guaranteed that the number of sampled point could be equal to the given maximum total number
========================================*/
void FrameRecon::SamplePoints(const pcl::PointCloud<pcl::PointXYZI> & vCloud, pcl::PointCloud<pcl::PointXYZI> & vNewCloud, int iSampleNum, bool bIntervalSamp){

	vNewCloud.clear();

	//sample by interval number
	if (bIntervalSamp){

		for (int i = 0; i < vCloud.points.size(); i = i + iSampleNum)
			vNewCloud.push_back(vCloud.points[i]);

		//over the function and output	
		return;

	}//end if

	//Sampling according to the given maximum total number
	//get the interval point number - how muny points should be skipped
	int iInterval = ceil(float(vCloud.points.size()) / float(iSampleNum));
	//sample
	for (int i = 0; i < vCloud.points.size(); i = i + iInterval)
		vNewCloud.push_back(vCloud.points[i]);

	//output
	return;

}


/*************************************************
Function: OutputScannedPCFile
Description: output scanned point clouds in a txt file
Calls: none
Called By: HandleTrajectory
Table Accessed: none
Table Updated: none
Input: vCloud - one frame scanning point cloud data
Output: a point cloud txt file
Return: none
Others: none
*************************************************/
void FrameRecon::OutputPCFile(const pcl::PointCloud<pcl::PointXYZI> & vCloud, bool bAllRecord){
  
    //generate a output file if possible
	if( m_bOutPCFileFlag || bAllRecord){

		m_sOutPCFileName.clear();
	    //set the current time stamp as a file name
		//m_sOutPCFileName << m_sFileHead << "PC_" << ros::Time::now() << ".txt"; 

		//set the count as a file name
		m_sOutPCFileName << m_sFileHead << "PC_" << m_iPCFrameCount << ".txt"; 

		m_bOutPCFileFlag = false;

        //print output file generation message
		std::cout << "[*] Attention, a point cloud recording file is created in " << m_sOutPCFileName.str() << std::endl;
	}

    //output
	m_oOutPCFile.open(m_sOutPCFileName.str(), std::ios::out | std::ios::app);

	//output in a txt file
	//the storage type of output file is x y z time frames 
    //record the point clouds
    for(int i = 0; i != vCloud.size(); ++i ){

        //output in a txt file
        //the storage type of output file is x y z time frames right/left_sensor
        m_oOutPCFile << vCloud.points[i].x << " "
                  << vCloud.points[i].y << " "
                  << vCloud.points[i].z << " " 
                  << m_iPCFrameCount << " " 
                  << std::endl;
    }//end for         

    m_oOutPCFile.close();

    //count new point cloud input (plus frame) 

}


/*************************************************
Function: OutputScannedPCFile
Description: output scanned point clouds in a txt file
Calls: none
Called By: HandleTrajectory
Table Accessed: none
Table Updated: none
Input: vCloud - one frame scanning point cloud data
Output: a point cloud txt file
Return: none
Others: none
*************************************************/
void FrameRecon::OutputPCFile(const pcl::PointCloud<pcl::PointXYZI> & vCloud, const std::vector<float> & vFeatures, bool bAllRecord){
  
    //generate a output file if possible
	if( m_bOutPCFileFlag || bAllRecord){

		m_sOutPCFileName.clear();
	    //set the current time stamp as a file name
		//m_sOutPCFileName << m_sFileHead << "PC_" << ros::Time::now() << ".txt"; 

		//set the count as a file name
		m_sOutPCFileName << m_sFileHead << "PC_" << m_iPCFrameCount << ".txt"; 

		m_bOutPCFileFlag = false;

        //print output file generation message
		std::cout << "[*] Attention, a point cloud recording file is created in " << m_sOutPCFileName.str() << std::endl;
	}

    //output
	m_oOutPCFile.open(m_sOutPCFileName.str(), std::ios::out | std::ios::app);

	//output in a txt file
	//the storage type of output file is x y z time frames 
    //record the point clouds
    for(int i = 0; i != vCloud.size(); ++i ){

        //output in a txt file
        //the storage type of output file is x y z time frames right/left_sensor
        m_oOutPCFile << vCloud.points[i].x << " "
                  << vCloud.points[i].y << " "
                  << vCloud.points[i].z << " " 
                  << vFeatures[i] << " "
                  << m_iPCFrameCount << " " 
                  << std::endl;
    }//end for         

    m_oOutPCFile.close();

    //count new point cloud input (plus frame) 

}




