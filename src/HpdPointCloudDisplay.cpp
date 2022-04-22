#include "HpdPointCloudDisplay.h"
//------------------------------------------------------------------------------------------
typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> PointCloudViewerPtr;

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudXYZPtr;
/*================================
HpdDisplay���캯�����
���ܣ���ʼ��ĳЩ����
backgroudcolor��ʼ��������ɫ
colorflag����
Generatecolor()���ڳ�ʼ����ɫ��
=================================*/
HpdDisplay::HpdDisplay(){
	colorflag=false;
	colorflag=Generatecolor();
	//Ĭ�ϱ�����ɫ
	backgroudcolor.r=colors[0].r;
	backgroudcolor.g=colors[0].g;
	backgroudcolor.b=colors[0].b;
	Setcolorvariationrange();
	colorvarflag=false;
}
/*================================
~HpdDisplay()�����������
���ܣ�������
=================================*/
HpdDisplay::~HpdDisplay(){

}
/*================================
Setcolorvariationrange�����������
���ܣ�������
=================================*/
void HpdDisplay::Setcolorvariationrange(float hr,float hg,float hb,
	float tr,float tg,float tb){
	headcolor.r=hr;
	headcolor.g=hg;
	headcolor.b=hb;
	tailcolor.r=tr;
	tailcolor.g=tg;
	tailcolor.b=tb;
	colorvarflag=true;
}
/*================================
Generatecolor()�������
���ܣ���ʼ����ɫ�岢��ֵ
�ܹ�27����Ҫ��ɫ������ʱ����
�õ���ɫ�ĵط�ֱ�ӵ�����Щ�����ڵ�ֵ
=================================*/
bool HpdDisplay::Generatecolor(){
	//�ɰ��ĵ�ɫ���
colors.resize(27);
//�Ҷ�ϵ
colors[0].r=255;colors[0].g=255;colors[0].b=255;colors[0].name="����";
colors[1].r=0;colors[1].g=0;colors[1].b=0;colors[1].name="����";
colors[2].r=185;colors[2].g=185;colors[2].b=185;colors[2].name="��ɫ";
//��ɫϵ
colors[3].r=0;colors[3].g=255;colors[3].b=0;colors[3].name="����";
colors[4].r=46;colors[4].g=139;colors[4].b=87;colors[4].name="������";
colors[5].r=0;colors[5].g=128;colors[5].b=0;colors[5].name="����";
colors[6].r=124;colors[6].g=252;colors[6].b=0;colors[6].name="��ƺ��";
//��ɫϵ
colors[7].r=0;colors[7].g=255;colors[7].b=255;colors[7].name="��ɫ";
colors[8].r=32;colors[8].g=178;colors[8].b=170;colors[8].name="ǳ������";
colors[9].r=0;colors[9].g=139;colors[9].b=139;colors[9].name="����ɫ";
//��ɫϵ
colors[10].r=255;colors[10].g=255;colors[10].b=0;colors[10].name="����";
colors[11].r=189;colors[11].g=183;colors[11].b=107;colors[11].name="������ŵ";
colors[12].r=255;colors[12].g=215;colors[12].b=0;colors[12].name="��ɫ";
colors[13].r=255;colors[13].g=165;colors[13].b=0;colors[13].name="��ɫ";
colors[14].r=255;colors[14].g=222;colors[14].b=173;colors[14].name="Ƥ����";
colors[15].r=255;colors[15].g=140;colors[15].b=0;colors[15].name="���ɫ";
colors[16].r=210;colors[16].g=105;colors[16].b=30;colors[16].name="�ɿ���ɫ";
//��ɫϵ
colors[17].r=255;colors[17].g=69;colors[17].b=0;colors[17].name="�Ⱥ�ɫ";
colors[18].r=255;colors[18].g=0;colors[18].b=0;colors[18].name="����";
colors[19].r=139;colors[19].g=9;colors[19].b=0;colors[19].name="���ɫ";
//��ɫϵ
colors[20].r=128;colors[20].g=0;colors[20].b=128;colors[20].name="��ɫ";
colors[21].r=148;colors[21].g=0;colors[21].b=211;colors[21].name="����ɫ";
colors[22].r=255;colors[22].g=105;colors[22].b=180;colors[22].name="�ۺ�ɫ";
//��ɫϵ
colors[23].r=0;colors[23].g=0;colors[23].b=255;colors[23].name="��ɫ";
colors[24].r=0;colors[24].g=0;colors[24].b=139;colors[24].name="����ɫ";
colors[25].r=30;colors[25].g=144;colors[25].b=255;colors[25].name="������";
colors[26].r=0;colors[26].g=191;colors[26].b=255;colors[26].name="����ɫ";

return true;
}
/*================================
Setbackgroudcolor�������
���ܣ����ñ�����ɫ
�β�1,2,3�ֱ���r,g,b��ɫֵ
��Ҫ��Ϊ�˷������������ɫ
����1��r,g,b��255��ʽ����
=================================*/
void HpdDisplay::Setbackgroudcolor(float f_r,float f_g,float f_b){
	//��ֹ����
	if(f_r*f_g*f_b<0){
	backgroudcolor.r=0;
	backgroudcolor.g=0;
	backgroudcolor.b=0;
	}
	//����1�����������255��ʽ��rgb
	if(f_r>1)
	backgroudcolor.r=f_r/255;
	if(f_g>1)
	backgroudcolor.g=f_g/255;
	if(f_b>1)
	backgroudcolor.b=f_b/255;
}
/*================================
Showsimplecolor�������
���ܣ���ʾ����
�����β�1��һ��ָ��pointXYZ���͵�ָ��
�����β�2��һ��char�����飬ֻ����yellow,white,black��red
������򵥵ĵ�����ʾ���ڼ�����
=================================*/
PointCloudViewerPtr HpdDisplay::Showsimplecolor(PointCloudXYZPtr & pointxyz,char *colorchoose){
	//���жϵ�ɫ���Ƿ��Ѿ�����
	while(!colorflag){
		colorflag=Generatecolor();
	}
	//����
	int i;
	//��ʾ����
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("��ɫ������ʾ����"));
	//һ���ַ����жϸ��ڶ����β�,Ĭ�ϰ�ɫ
	if (!strcmp(colorchoose,"white"))
	i=0;
	else if(!strcmp(colorchoose,"red"))
	i=18;
	else if(!strcmp(colorchoose,"yellow"))
	i=10;
	else if(!strcmp(colorchoose,"black"))
	i=1;
	else if(!strcmp(colorchoose,"grey"))
	i=2;
	else if(!strcmp(colorchoose,"free")){
	std::cout<<"���ʴ��ˣ���Ҫ��ʲô��ɫ? ���������"<<std::endl;
	displaytable();
	std::cin>>i;
	}
	//��ɫ��ֵ
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color (pointxyz, colors[i].r, colors[i].g, colors[i].b);
	viewer->addPointCloud(pointxyz,color,"Simple Cloud");
	//����
	viewer->setBackgroundColor(backgroudcolor.r, backgroudcolor.g, backgroudcolor.b);
	return(viewer);
}//end Showsimplecolor

/*================================
Showelevation�������
���ܣ���ʾ�̶߳ȣ����Ǹ߶���Ϣ�Ŀ��ӻ�����ɫ����
�����β�1��һ��ָ��pointXYZ���͵�ָ��
�����β�2��һ���������ڸ�sampleInterval���������1Ϊȫ�����㶮��
�������߶ȣ����z��ֵ���仯�Ĳ�ɫ���ƣ��߶�������z����˲�����
=================================*/
PointCloudViewerPtr HpdDisplay::Showelevation(PointCloudXYZPtr & pointxyz,int sampleInterval){

	//һ����ɫ�������ڱ�����ɫ��Ϣ
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb(new pcl::PointCloud<pcl::PointXYZRGB>);

	//PCL XYZ��ʽ
	unsigned long pointNum = pointxyz->size()/sampleInterval;
    cloudrgb->width = pointNum;
	cloudrgb->height = 1;
	cloudrgb->is_dense = false;
	cloudrgb->points.resize(cloudrgb->width*cloudrgb->height);
    //����z�������Сֵ�������һ�������¸�ֵ
	double minElevation = pointxyz->points[0].z, maxElevation = pointxyz->points[0].z;
	for(size_t i=1;i<pointNum;++i){
		if(pointxyz->points[i].z<minElevation)
			minElevation = pointxyz->points[i].z;
		if( pointxyz->points[i].z>maxElevation)
			maxElevation = pointxyz->points[i].z;
	}
	//�Եȼ�������ķ�ʽѡ�����
	for(size_t i=0,j=0;i<cloudrgb->points.size();++i,j+=sampleInterval){
		cloudrgb->points[i].x = pointxyz->points[j].x;
		cloudrgb->points[i].y = pointxyz->points[j].y;
		cloudrgb->points[i].z = pointxyz->points[j].z;
		//ͬʱ���ݹ�һ����ʽ�ȱ�������ͼ��ĸ߳�ֵ
		//��������ɫ������˭������ǰ����ɫʵ���ҵ���,�һ���㽲�ǵ�˹��ͼ����ɫ���ѷֱ���
		cloudrgb->points[i].r = Colorvar(headcolor.r,tailcolor.r,(pointxyz->points[j].z-minElevation)/(maxElevation-minElevation));
	    cloudrgb->points[i].g = Colorvar(headcolor.g,tailcolor.g,(pointxyz->points[j].z-minElevation)/(maxElevation-minElevation));
	    cloudrgb->points[i].b = Colorvar(headcolor.b,tailcolor.b,(pointxyz->points[j].z-minElevation)/(maxElevation-minElevation));
		
	}
	//����һ��PCL����ʾ����
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("�̵߳�����ʾ����"));
	
	//PCL��ɫ
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloudrgb);
	viewer->addPointCloud(cloudrgb,rgb,"Elevation Cloud");	
	//����
	viewer->setBackgroundColor(backgroudcolor.r, backgroudcolor.g, backgroudcolor.b);
	//���ش���
    return(viewer);
	
}//end Showelevation
/*================================
Showneighboorcloud�������
���ܣ�ĳ����������Ϣ�Ŀ��ӻ�����ɫ����
�����β�1��һ��ָ��pointXYZ���͵�ָ��
�����β�2��һ���������ڶ������ͣ���ֵ1Ϊ��������������ֵΪ������ʾ����
�����β�3��float�͵İ뾶���������Ҫ
�����β�4���ӵ㣬���������ѡ�񣬱�����10000���㣬��ѡ���3021����
��˲�����ѡһ�㣬��������֪������ţ���ѡ�Ļ������һ��
��������ӻ��ĵ���������
=================================*/
//��ʾ����ĺ���
PointCloudViewerPtr HpdDisplay::Showneighboorcloud(PointCloudXYZPtr & pointxyz, int type, float radius, int viewpoint){
	
	//һ����ɫ�������ڱ�����ɫ��Ϣ
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointrgb(new pcl::PointCloud<pcl::PointXYZRGB>);

	//	
	if (viewpoint==-1){
	    srand(unsigned(time(0)));//��ϵͳʱ����Ϊ�������
	    for(int icnt = 0; icnt != 2; ++icnt)//�����������������Ϸ�ɢ
			viewpoint=int(random(0,pointxyz->size()-1)); 
	}
	else if (viewpoint>=pointxyz->size()){
		std::cout<<"Out of the points number range!!!!"<<std::endl;
	    exit(0);
	}

	//����kdtree������
	pcl::KdTreeFLANN<pcl::PointXYZ>kdtree;
    kdtree.setInputCloud (pointxyz);
	//������������
    pcl::PointXYZ searchPoint;
    searchPoint.x=pointxyz->points[viewpoint].x;
    searchPoint.y=pointxyz->points[viewpoint].y;
    searchPoint.z=pointxyz->points[viewpoint].z;
    //+++++++���������뾶++++++important
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
//ִ������
if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) >0 )
  {
	  //+++++++++++++++++++++++++++++++++++++++++++����ģʽ++++++++++++++++++++++
	  //+++++++++++++++++++++++++++++ģʽ1ȫ����ʾ����������Ĭ�ϣ�
	  if(type==1){
		  //��ͳһ�����Ƹ��ϰ�ɫ
	unsigned long pointNum = pointxyz->size();
    pointrgb->width = pointNum;
	pointrgb->height = 1;
	pointrgb->is_dense = false;
	pointrgb->points.resize(pointrgb->width*pointrgb->height);
	for(size_t i=0;i<pointxyz->points.size();++i){
		pointrgb->points[i].x = pointxyz->points[i].x;
		pointrgb->points[i].y = pointxyz->points[i].y;
		pointrgb->points[i].z = pointxyz->points[i].z;
		pointrgb->points[i].r = 150;
		pointrgb->points[i].g = 150;
		pointrgb->points[i].b = 150;
	}
	//�������϶�Ӧ����ɫ
for (size_t i=0; i<pointIdxRadiusSearch.size (); ++i){
pointrgb->points[ pointIdxRadiusSearch[i] ].r=255;  
pointrgb->points[ pointIdxRadiusSearch[i] ].g=255;
pointrgb->points[ pointIdxRadiusSearch[i] ].b=0;
}//end for
	  }//end second if
	  else{
		  //++++++++++++++++++++++++++++++++++++++++����������õڶ���ģʽ
		  //+++++++++++++++++++++++++ֻ��ʾ����ĵ�
		  //��ͳһ�����Ƹ��ϰ�ɫ
	unsigned long pointNum =(unsigned long)pointIdxRadiusSearch.size();
    pointrgb->width = pointNum;
	pointrgb->height = 1;
	pointrgb->is_dense = false;
	pointrgb->points.resize(pointrgb->width*pointrgb->height);
	for(size_t i=0;i!=pointIdxRadiusSearch.size();++i){
		//ֻ��ʾ������ֵ
		pointrgb->points[i].x= pointxyz->points[ pointIdxRadiusSearch[i] ].x ;
		pointrgb->points[i].y= pointxyz->points[ pointIdxRadiusSearch[i] ].y;
		pointrgb->points[i].z= pointxyz->points[ pointIdxRadiusSearch[i] ].z;
        pointrgb->points[i].r=255;  
        pointrgb->points[i].g=255;
        pointrgb->points[i].b=0;
              }//end for
	  }//end else
}//end first if
//
//����һ��PCL����ʾ����
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("������ƿ��ӻ�"));
	//
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointrgb);
	viewer->addPointCloud(pointrgb,rgb,"Neighboor Cloud Show");	
	//����
	viewer->setBackgroundColor(backgroudcolor.r, backgroudcolor.g, backgroudcolor.b);
	//holdס����
	return(viewer);
}
PointCloudViewerPtr HpdDisplay::Showneighboorcloud(PointCloudXYZPtr & pointxyz, 
	double f_x, double f_y, double f_z, float radius,int type){
	//һ����ɫ�������ڱ�����ɫ��Ϣ
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointrgb(new pcl::PointCloud<pcl::PointXYZRGB>);

	//����kdtree������
	pcl::KdTreeFLANN<pcl::PointXYZ>kdtree;
    kdtree.setInputCloud (pointxyz);
	//������������
    pcl::PointXYZ searchPoint;
    searchPoint.x=float(f_x);
    searchPoint.y=float(f_y);
    searchPoint.z=float(f_z);
    //+++++++���������뾶++++++important
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
//ִ������
if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) >0 )
  {
	  //+++++++++++++++++++++++++++++++++++++++++++����ģʽ++++++++++++++++++++++
	  //+++++++++++++++++++++++++++++ģʽ1ȫ����ʾ����������Ĭ�ϣ�
	  if(type==1){
		  //��ͳһ�����Ƹ��ϰ�ɫ
	unsigned long pointNum = pointxyz->size();
    pointrgb->width = pointNum;
	pointrgb->height = 1;
	pointrgb->is_dense = false;
	pointrgb->points.resize(pointrgb->width*pointrgb->height);
	for(size_t i=0;i<pointxyz->points.size();++i){
		pointrgb->points[i].x = pointxyz->points[i].x;
		pointrgb->points[i].y = pointxyz->points[i].y;
		pointrgb->points[i].z = pointxyz->points[i].z;
		pointrgb->points[i].r = 150;
		pointrgb->points[i].g = 150;
		pointrgb->points[i].b = 150;
	}
	//�������϶�Ӧ����ɫ
for (size_t i=0; i<pointIdxRadiusSearch.size (); ++i){
pointrgb->points[ pointIdxRadiusSearch[i] ].r=255;  
pointrgb->points[ pointIdxRadiusSearch[i] ].g=255;
pointrgb->points[ pointIdxRadiusSearch[i] ].b=0;
}//end for
	  }//end second if
	  else{
		  //++++++++++++++++++++++++++++++++++++++++����������õڶ���ģʽ
		  //+++++++++++++++++++++++++ֻ��ʾ����ĵ�
		  //��ͳһ�����Ƹ��ϰ�ɫ
	unsigned long pointNum =(unsigned long)pointIdxRadiusSearch.size();
    pointrgb->width = pointNum;
	pointrgb->height = 1;
	pointrgb->is_dense = false;
	pointrgb->points.resize(pointrgb->width*pointrgb->height);
	for(size_t i=0;i!=pointIdxRadiusSearch.size();++i){
		//ֻ��ʾ������ֵ
		pointrgb->points[i].x= pointxyz->points[ pointIdxRadiusSearch[i] ].x ;
		pointrgb->points[i].y= pointxyz->points[ pointIdxRadiusSearch[i] ].y;
		pointrgb->points[i].z= pointxyz->points[ pointIdxRadiusSearch[i] ].z;
        pointrgb->points[i].r=255;  
        pointrgb->points[i].g=255;
        pointrgb->points[i].b=0;
              }//end for
	  }//end else
}//end first if
//
//����һ��PCL����ʾ����
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("������ƿ��ӻ�"));
	//
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointrgb);
	viewer->addPointCloud(pointrgb,rgb,"Neighboor Cloud Show");	
	//����
	viewer->setBackgroundColor(backgroudcolor.r, backgroudcolor.g, backgroudcolor.b);
	//holdס����
	return(viewer);
}
/*================================
HpdDisplay::Showclassification�������
���ܣ���ʾ������ȡ�����ʶ��Ľ��
�����β�1��һ��Point3D���͵ĵ������ݣ�һ����ָ�뿪�ٵ��ڴ�����
�����β�2��һ���ַ��������ж���������ָ��"assign", ���"random"���ַ�ʽ
ָ�����ǽ���ʽѡ����ɫ������������ɫ
ע�⣺�������������ԣ���1,2,3,4��������ţ�����1,13,44,89������Ծ������
�����������ͼ
=================================*/
PointCloudViewerPtr HpdDisplay::Showclassification(std::vector<Point3D> & point3d,char *keyword){
	//���жϵ�ɫ���Ƿ��Ѿ�����
	while(!colorflag){
		colorflag=Generatecolor();
	}
	//����ģʽ
char *dictionary[2] = {"assign", "random"};//ָ������
int keynumber;
for(keynumber=0;keynumber<2;keynumber++)
{
  if(!strcmp(keyword,dictionary[keynumber]))
	  break;
}
int maxclassification=-INT_MAX,minclassification=INT_MAX;
//�ҵ��������
for(size_t i=0;i!=point3d.size();i++){
	if(point3d[i].classification> maxclassification)
		maxclassification=point3d[i].classification;
	if(point3d[i].classification< minclassification)
		minclassification=point3d[i].classification;
}
//���
int classnumber=maxclassification-minclassification;
//��ʼ����ɫ����
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb(new pcl::PointCloud<pcl::PointXYZRGB>);

unsigned long pointNum = point3d.size();
cloudrgb->width = pointNum;
cloudrgb->height = 1;
cloudrgb->is_dense = false;
cloudrgb->points.resize(cloudrgb->width*cloudrgb->height);

switch(keynumber){
   //ָ��ģʽ�����Ƚ��٣����������۷�����
case 0:{
//�ֶ��ƶ���ɫ������ʾһЩ����
	std::vector<int> colorindices(classnumber+1); 
	std::cout<<"��"<<classnumber+1<<"��"<<std::endl;
	std::cout<<"���ʴ��ˣ���Ҫ��ʲô��ɫ? ���������"<<std::endl;
	displaytable();
   //�ֶ���ֵ
	for(int i=0;i<classnumber+1;i++){
		std::cout<<"��"<<i+1<<"��ɫ��ţ������벢�س�"<<std::endl;
	    std::cin>> colorindices[i];
	}
   //��ɫ
for(size_t i=0;i<cloudrgb->points.size();++i){
	cloudrgb->points[i].x = point3d[i].x;
	cloudrgb->points[i].y = point3d[i].y;
	cloudrgb->points[i].z = point3d[i].z;
	//ͨ������������
	cloudrgb->points[i].r = colors[colorindices[point3d[i].classification-minclassification]].r;
	cloudrgb->points[i].g = colors[colorindices[point3d[i].classification-minclassification]].g;
	cloudrgb->points[i].b = colors[colorindices[point3d[i].classification-minclassification]].b;
}
break;}
   //���ģʽ,���ڵ������Ƚ϶�����
case 1:{
	//���ڸ���װ��ɫ
	std::vector<ColorRGB> classcolors(classnumber+1);
	//��ϵͳʱ����Ϊ�������
	srand(unsigned(time(0)));
	for(size_t n=0;n!=classnumber+1;n++){
	    for(int icnt = -1; icnt != 3; ++icnt){//����ʱ����
			if(icnt==0)
			classcolors[n].r=float(random(0,255)); 
		    if(icnt==1)
			classcolors[n].g=float(random(0,255)); 
			if(icnt==2)
			classcolors[n].g=float(random(0,255)); 
	}
}
	int indices=0;
//�Եȼ�������ķ�ʽѡ�����
    for(size_t i=0;i<cloudrgb->points.size();++i){
	cloudrgb->points[i].x = point3d[i].x;
	cloudrgb->points[i].y = point3d[i].y;
	cloudrgb->points[i].z = point3d[i].z;
	//�ҵ������Ӧ�������ɫֵ
	indices=point3d[i].classification-minclassification;
	cloudrgb->points[i].r = classcolors[indices].r;
	cloudrgb->points[i].g = classcolors[indices].g;
	cloudrgb->points[i].b = classcolors[indices].b;
}
	break;}
//Ĭ��ģʽ
default:{
	std::cout<<"������·�㲻�ߣ�����������ƫ����ģʽѡ�����"<<std::endl;
	std::cin.get();
	exit(0);
	break;}
}//end switch

	//����һ��PCL����ʾ����
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloudrgb);
	viewer->addPointCloud<pcl::PointXYZRGB> (cloudrgb, rgb, "classified cloud");
	
	viewer->setBackgroundColor (backgroudcolor.r, backgroudcolor.g, backgroudcolor.b);
	//holdס����
	return(viewer);

}//end Classification


/*================================
HpdDisplay::Showclassification�������
���ܣ���ʾ������ȡ�����ʶ��Ľ��
�����β�1��һ��Point3D���͵ĵ������ݣ�һ����ָ�뿪�ٵ��ڴ�����
�����β�2��һ���ַ��������ж���������ָ��"assign", ���"random"���ַ�ʽ
ָ�����ǽ���ʽѡ����ɫ������������ɫ
ע�⣺�������������ԣ���1,2,3,4��������ţ�����1,13,44,89������Ծ������
�����������ͼ
=================================*/
PointCloudViewerPtr HpdDisplay::Showclassification(PointCloudXYZPtr & pCloud, std::vector<int> & vClasses, char *keyword){
	
		//���жϵ�ɫ���Ƿ��Ѿ�����
		while (!colorflag){
			colorflag = Generatecolor();
		}
		//����ģʽ
		char *dictionary[2] = { "assign", "random" };//ָ������
		int keynumber;
		for (keynumber = 0; keynumber<2; keynumber++)
		{
			if (!strcmp(keyword, dictionary[keynumber]))
				break;
		}
		int maxclassification = -INT_MAX, minclassification = INT_MAX;
		//�ҵ��������
		for (size_t i = 0; i != vClasses.size(); i++){
			if (vClasses[i] > maxclassification)
				maxclassification = vClasses[i];
			if (vClasses[i] < minclassification)
				minclassification = vClasses[i];
		}
		//���
		int classnumber = maxclassification - minclassification;
		//��ʼ����ɫ����
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb(new pcl::PointCloud<pcl::PointXYZRGB>);

		unsigned long pointNum = pCloud->points.size();
		cloudrgb->width = pointNum;
		cloudrgb->height = 1;
		cloudrgb->is_dense = false;
		cloudrgb->points.resize(cloudrgb->width*cloudrgb->height);

		switch (keynumber){
			//ָ��ģʽ�����Ƚ��٣����������۷�����
		case 0:{
			//�ֶ��ƶ���ɫ������ʾһЩ����
			std::vector<int> colorindices(classnumber + 1);
			std::cout << "��" << classnumber + 1 << "��" << std::endl;
			std::cout << "���ʴ��ˣ���Ҫ��ʲô��ɫ? ���������" << std::endl;
			displaytable();
			//�ֶ���ֵ
			for (int i = 0; i<classnumber + 1; i++){
				std::cout << "��" << i + 1 << "��ɫ��ţ������벢�س�" << std::endl;
				std::cin >> colorindices[i];
			}
			//��ɫ
			for (size_t i = 0; i<cloudrgb->points.size(); ++i){
				cloudrgb->points[i].x = pCloud->points[i].x;
				cloudrgb->points[i].y = pCloud->points[i].y;
				cloudrgb->points[i].z = pCloud->points[i].z;
				//ͨ������������
				cloudrgb->points[i].r = colors[colorindices[vClasses[i] - minclassification]].r;
				cloudrgb->points[i].g = colors[colorindices[vClasses[i] - minclassification]].g;
				cloudrgb->points[i].b = colors[colorindices[vClasses[i] - minclassification]].b;
			}
			break; }
			//���ģʽ,���ڵ������Ƚ϶�����
		case 1:{
			//���ڸ���װ��ɫ
			std::vector<ColorRGB> classcolors(classnumber + 1);
			//��ϵͳʱ����Ϊ�������
			srand(unsigned(time(0)));
			for (size_t n = 0; n != classnumber + 1; n++){
				for (int icnt = -1; icnt != 3; ++icnt){//����ʱ����
					if (icnt == 0)
						classcolors[n].r = float(random(0, 255));
					if (icnt == 1)
						classcolors[n].g = float(random(0, 255));
					if (icnt == 2)
						classcolors[n].g = float(random(0, 255));
				}
			}
			int indices = 0;
			//�Եȼ�������ķ�ʽѡ�����
			for (size_t i = 0; i<cloudrgb->points.size(); ++i){
				cloudrgb->points[i].x = pCloud->points[i].x;
				cloudrgb->points[i].y = pCloud->points[i].y;
				cloudrgb->points[i].z = pCloud->points[i].z;
				//�ҵ������Ӧ�������ɫֵ
				indices = vClasses[i] - minclassification;
				cloudrgb->points[i].r = classcolors[indices].r;
				cloudrgb->points[i].g = classcolors[indices].g;
				cloudrgb->points[i].b = classcolors[indices].b;
			}
			break; }
			//Ĭ��ģʽ
		default:{
			std::cout << "������·�㲻�ߣ�����������ƫ����ģʽѡ�����" << std::endl;
			std::cin.get();
			exit(0);
			break; }
		}//end switch

		//����һ��PCL����ʾ����
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloudrgb);
		viewer->addPointCloud<pcl::PointXYZRGB>(cloudrgb, rgb, "classified cloud");

		viewer->setBackgroundColor(backgroudcolor.r, backgroudcolor.g, backgroudcolor.b);
		//holdס����
		return(viewer);

}//end Classification


/*================================
HpdDisplay::Showclassification�������
���ܣ���ʾ������ȡ�����ʶ��Ľ��
�����β�1��һ��Point3D���͵ĵ������ݣ�һ����ָ�뿪�ٵ��ڴ�����
�����β�2��һ���ַ��������ж���������ָ��"assign", ���"random"���ַ�ʽ
ָ�����ǽ���ʽѡ����ɫ������������ɫ
ע�⣺�������������ԣ���1,2,3,4��������ţ�����1,13,44,89������Ծ������
�����������ͼ
=================================*/
PointCloudViewerPtr HpdDisplay::Showclassification(pcl::PointCloud<pcl::PointXYZ> & vCloud, std::vector<int> & vClasses, char *keyword){

	//���жϵ�ɫ���Ƿ��Ѿ�����
	while (!colorflag){
		colorflag = Generatecolor();
	}
	//����ģʽ
	char *dictionary[2] = { "assign", "random" };//ָ������
	int keynumber;
	for (keynumber = 0; keynumber<2; keynumber++)
	{
		if (!strcmp(keyword, dictionary[keynumber]))
			break;
	}
	int maxclassification = -INT_MAX, minclassification = INT_MAX;
	//�ҵ��������
	for (size_t i = 0; i != vClasses.size(); i++){
		if (vClasses[i] > maxclassification)
			maxclassification = vClasses[i];
		if (vClasses[i] < minclassification)
			minclassification = vClasses[i];
	}
	//���
	int classnumber = maxclassification - minclassification;
	//��ʼ����ɫ����
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb(new pcl::PointCloud<pcl::PointXYZRGB>);

	unsigned long pointNum = vCloud.points.size();
	cloudrgb->width = pointNum;
	cloudrgb->height = 1;
	cloudrgb->is_dense = false;
	cloudrgb->points.resize(cloudrgb->width*cloudrgb->height);

	switch (keynumber){
		//ָ��ģʽ�����Ƚ��٣����������۷�����
	case 0:{
		//�ֶ��ƶ���ɫ������ʾһЩ����
		std::vector<int> colorindices(classnumber + 1);
		std::cout << "��" << classnumber + 1 << "��" << std::endl;
		std::cout << "���ʴ��ˣ���Ҫ��ʲô��ɫ? ���������" << std::endl;
		displaytable();
		//�ֶ���ֵ
		for (int i = 0; i<classnumber + 1; i++){
			std::cout << "��" << i + 1 << "��ɫ��ţ������벢�س�" << std::endl;
			std::cin >> colorindices[i];
		}
		//��ɫ
		for (size_t i = 0; i<cloudrgb->points.size(); ++i){
			cloudrgb->points[i].x = vCloud.points[i].x;
			cloudrgb->points[i].y = vCloud.points[i].y;
			cloudrgb->points[i].z = vCloud.points[i].z;
			//ͨ������������
			cloudrgb->points[i].r = colors[colorindices[vClasses[i] - minclassification]].r;
			cloudrgb->points[i].g = colors[colorindices[vClasses[i] - minclassification]].g;
			cloudrgb->points[i].b = colors[colorindices[vClasses[i] - minclassification]].b;
		}
		break; }
		//���ģʽ,���ڵ������Ƚ϶�����
	case 1:{
		//���ڸ���װ��ɫ
		std::vector<ColorRGB> classcolors(classnumber + 1);
		//��ϵͳʱ����Ϊ�������
		srand(unsigned(time(0)));
		for (size_t n = 0; n != classnumber + 1; n++){
			for (int icnt = -1; icnt != 3; ++icnt){//����ʱ����
				if (icnt == 0)
					classcolors[n].r = float(random(0, 255));
				if (icnt == 1)
					classcolors[n].g = float(random(0, 255));
				if (icnt == 2)
					classcolors[n].g = float(random(0, 255));
			}
		}
		int indices = 0;
		//�Եȼ�������ķ�ʽѡ�����
		for (size_t i = 0; i<cloudrgb->points.size(); ++i){
			cloudrgb->points[i].x = vCloud.points[i].x;
			cloudrgb->points[i].y = vCloud.points[i].y;
			cloudrgb->points[i].z = vCloud.points[i].z;
			//�ҵ������Ӧ�������ɫֵ
			indices = vClasses[i] - minclassification;
			cloudrgb->points[i].r = classcolors[indices].r;
			cloudrgb->points[i].g = classcolors[indices].g;
			cloudrgb->points[i].b = classcolors[indices].b;
		}
		break; }
		//Ĭ��ģʽ
	default:{
		std::cout << "������·�㲻�ߣ�����������ƫ����ģʽѡ�����" << std::endl;
		std::cin.get();
		exit(0);
		break; }
	}//end switch

	//����һ��PCL����ʾ����
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloudrgb);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloudrgb, rgb, "classified cloud");

	viewer->setBackgroundColor(backgroudcolor.r, backgroudcolor.g, backgroudcolor.b);
	//holdס����
	return(viewer);

}//end Classification

/*================================
Showfeatureresult�������
���ܣ���ʾ���������ֲ�
����ѡ��������ɫģʽ���ֱ��Ǻ��̣��Ҷȣ�����Ŀ����ʾ
ͨ��ĳһ�㷨�������ĳ�����ϻ���һ�����ϵ�����ֵ���ú���ֻ��ʾһ������ֵ
���ĸ�����Ҳ������ͬʱ��ʾ������1����������ʾ
�����β�1��һ��point3d��������
�����β�2��һ��double�͵������洢����ֵ
ע�⣺�����ǰ�ÿ��Ϊ��λ��
����������ֲ�ͼ
"gray","redgreen" ,"greenblue","redblue","free"����ģʽ
=================================*/
PointCloudViewerPtr HpdDisplay::Showfeatureresult(std::vector<Point3D> & point3d,std::vector<double> & features,char *keyword){
	//���ж��ǲ���ͬһ�����Ƽ���
	if(point3d.size()!=features.size()){
		std::cout<<"Error:���Ƽ���������������ƥ�䣡"<<std::endl;
		std::cout<<"�밴�س���������ȷ����������"<<std::endl;
		std::cin.get();
		exit(0);
	}
	//********************�ж������Ѹ�������ֵ�����������¸���***************************
	if(colorvarflag){//����ǰ���õ�ѡ
		//ʲô�����ɣ�ǰ���Ѿ�����
    }else{//��������ʽѡ
	//��ɫѡ�����
	char *dictionary[5] = {"gray","redgreen" ,"greenblue","redblue","free"};//ָ������
    int keynumber;
    for(keynumber=0;keynumber<5;keynumber++)
    {
    if(!strcmp(keyword,dictionary[keynumber]))
	  break;
    }
		 
switch(keynumber){
	//******************ģʽ1���̱仯��ʾ
case 0:{
	//������ɫ
	headcolor=colors[0];
	tailcolor=colors[1];
	//��Ӧ������ɫ
	backgroudcolor=colors[4];
	break;
	   }
   //******************ģʽ2�̺�
case 1:{
	//������ɫ
	headcolor.r=0.0;
	headcolor.g=255.0;
	headcolor.b=0.0;
	tailcolor.r=255.0;
	tailcolor.g=0.0;
	tailcolor.b=0.0;
	//��Ӧ������ɫ
	backgroudcolor=colors[0];
	break;
	   }
	//******************ģʽ3������ʾ
case 2:{
	//������ɫ
	headcolor.r=189.0;
	headcolor.g=183.0;
	headcolor.b=107.0;
	tailcolor.r=0.0;
	tailcolor.g=107.0;
	tailcolor.b=139.0;
	//��ɫ
	backgroudcolor=colors[0];
	break;
	   }
	  //******************ģʽ4������ʾ
case 3:{
	//������ɫ
	headcolor.r=9.0;
	headcolor.g=50.0;
	headcolor.b=204.0;
	tailcolor.r=249.0;
	tailcolor.g=50.0;
	tailcolor.b=204.0;
	//������ϲ���ı�����ɫ
	backgroudcolor.r=0.33;backgroudcolor.g=0.67;backgroudcolor.b=0.58;
	break;
	   }
	   //******************ģʽ4����ѡ����ɫ
case 4:{
	int cc;
	displaytable();
	std::cout<<"���ʴ��ˣ�ͷ��ɫ��ľ�����? ���������"<<std::endl;
	std::cin>>cc;
	headcolor=colors[cc];
	std::cout<<"���ʴ��ˣ�β��ɫ��ľ�����? ���������"<<std::endl;
	std::cin>>cc;
	tailcolor=colors[cc];
	//������ϲ���ı�����ɫ
	std::cout<<"�������ʣ�������ɫ������~~~��Ҫѡ���ĸ�?(\\^0^)// ���������,"<<std::endl;
	std::cin>>cc;
	backgroudcolor=colors[cc];
	break;
	   }
default:{
	std::cout<<"Interesting but inpossible! You choose something wrong!"<<std::endl;
	std::cin.get();
	exit(0);
	break;
	}
       }//end switch
    }//end super big if

   //��������ֵ�ķ�Χ�������һ��
	double minvalue = features[0], maxvalue = features[0];
	for(size_t i=1;i<features.size();++i){
		if(features[i]<minvalue)
		minvalue = features[i];
		if(features[i]> maxvalue)
		maxvalue = features[i];
	}
	//һ����ɫ�������ڱ�����ɫ��Ϣ
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointxyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);

	//PCL RGB��ʽ
	unsigned long pointNum = point3d.size();
    pointxyzrgb->width = pointNum;
	pointxyzrgb->height = 1;
	pointxyzrgb->is_dense = false;
	pointxyzrgb->points.resize(pointxyzrgb->width*pointxyzrgb->height);
    //��ɫ
    for(size_t i=0;i<pointxyzrgb->points.size();i++){
		pointxyzrgb->points[i].x = point3d[i].x;
		pointxyzrgb->points[i].y = point3d[i].y;
		pointxyzrgb->points[i].z = point3d[i].z;
		//��������ֵ�ı仯����
		//�����ɫ�����Ƚ��ʺ���ɫ����
	    pointxyzrgb->points[i].r = Colorvar(headcolor.r,tailcolor.r,(features[i]-minvalue)/(maxvalue-minvalue));
	    pointxyzrgb->points[i].g = Colorvar(headcolor.g,tailcolor.g,(features[i]-minvalue)/(maxvalue-minvalue));
	    pointxyzrgb->points[i].b = Colorvar(headcolor.b,tailcolor.b,(features[i]-minvalue)/(maxvalue-minvalue));
	}
	//����һ��PCL����ʾ����
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("�̵߳�����ʾ����"));
	//��ɫ
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointxyzrgb);
	viewer->addPointCloud(pointxyzrgb,rgb,"Features Show");	
	//����
	viewer->setBackgroundColor(backgroudcolor.r, backgroudcolor.g, backgroudcolor.b);
	//���ش���
    return(viewer);
	
}//end 
//reload with float type
PointCloudViewerPtr HpdDisplay::Showfeatureresult(PointCloudXYZPtr & pCloud, std::vector<float> & features, char *keyword) {
	//���ж��ǲ���ͬһ�����Ƽ���
	if (pCloud->points.size() != features.size()) {
		std::cout << "Error:���Ƽ���������������ƥ�䣡" << std::endl;
		std::cout << "�밴�س���������ȷ����������" << std::endl;
		std::cin.get();
		exit(0);
	}
	//********************�ж������Ѹ�������ֵ�����������¸���***************************
	if (colorvarflag) {//����ǰ���õ�ѡ
					   //ʲô�����ɣ�ǰ���Ѿ�����
	}
	else {//��������ʽѡ
		  //��ɫѡ�����
		char *dictionary[5] = { "grey","redgreen" ,"greenblue","redblue","free" };//ָ������
		int keynumber;
		for (keynumber = 0; keynumber<5; keynumber++)
		{
			if (!strcmp(keyword, dictionary[keynumber]))
				break;
		}

		switch (keynumber) {
			//******************ģʽ1���̱仯��ʾ
		case 0: {
			//������ɫ
			headcolor = colors[0];
			tailcolor = colors[1];
			//��Ӧ������ɫ
			backgroudcolor = colors[4];
			break;
		}
				//******************ģʽ2�̺�
		case 1: {
			//������ɫ
			headcolor.r = 0.0;
			headcolor.g = 255.0;
			headcolor.b = 0.0;
			tailcolor.r = 255.0;
			tailcolor.g = 0.0;
			tailcolor.b = 0.0;
			//��Ӧ������ɫ
			backgroudcolor = colors[0];
			break;
		}
				//******************ģʽ3������ʾ
		case 2: {
			//������ɫ
			headcolor.r = 189.0;
			headcolor.g = 183.0;
			headcolor.b = 107.0;
			tailcolor.r = 0.0;
			tailcolor.g = 107.0;
			tailcolor.b = 139.0;
			//��ɫ
			backgroudcolor = colors[0];
			break;
		}
				//******************ģʽ4������ʾ
		case 3: {
			//������ɫ
			headcolor.r = 9.0;
			headcolor.g = 50.0;
			headcolor.b = 204.0;
			tailcolor.r = 249.0;
			tailcolor.g = 50.0;
			tailcolor.b = 204.0;
			//������ϲ���ı�����ɫ
			backgroudcolor.r = 0.33; backgroudcolor.g = 0.67; backgroudcolor.b = 0.58;
			break;
		}
				//******************ģʽ4����ѡ����ɫ
		case 4: {
			int cc;
			displaytable();
			std::cout << "���ʴ��ˣ�ͷ��ɫ��ľ�����? ���������" << std::endl;
			std::cin >> cc;
			headcolor = colors[cc];
			std::cout << "���ʴ��ˣ�β��ɫ��ľ�����? ���������" << std::endl;
			std::cin >> cc;
			tailcolor = colors[cc];
			//������ϲ���ı�����ɫ
			std::cout << "�������ʣ�������ɫ������~~~��Ҫѡ���ĸ�?(\\^0^)// ���������," << std::endl;
			std::cin >> cc;
			backgroudcolor = colors[cc];
			break;
		}
		default: {
			std::cout << "Interesting but inpossible! You choose something wrong!" << std::endl;
			std::cin.get();
			exit(0);
			break;
		}
		}//end switch
	}//end super big if

	 //��������ֵ�ķ�Χ�������һ��
	double minvalue = features[0], maxvalue = features[0];
	for (size_t i = 1; i<features.size(); ++i) {
		if (features[i]<minvalue)
			minvalue = features[i];
		if (features[i]> maxvalue)
			maxvalue = features[i];
	}
	//һ����ɫ�������ڱ�����ɫ��Ϣ
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointxyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);

	//PCL RGB��ʽ
	unsigned long pointNum = pCloud->points.size();
	pointxyzrgb->width = pointNum;
	pointxyzrgb->height = 1;
	pointxyzrgb->is_dense = false;
	pointxyzrgb->points.resize(pointxyzrgb->width*pointxyzrgb->height);
	//��ɫ
	for (size_t i = 0; i<pointxyzrgb->points.size(); i++) {
		pointxyzrgb->points[i].x = pCloud->points[i].x;
		pointxyzrgb->points[i].y = pCloud->points[i].y;
		pointxyzrgb->points[i].z = pCloud->points[i].z;
		//��������ֵ�ı仯����
		//�����ɫ�����Ƚ��ʺ���ɫ����
		pointxyzrgb->points[i].r = Colorvar(headcolor.r, tailcolor.r, (features[i] - minvalue) / (maxvalue - minvalue));
		pointxyzrgb->points[i].g = Colorvar(headcolor.g, tailcolor.g, (features[i] - minvalue) / (maxvalue - minvalue));
		pointxyzrgb->points[i].b = Colorvar(headcolor.b, tailcolor.b, (features[i] - minvalue) / (maxvalue - minvalue));
	}
	//����һ��PCL����ʾ����
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("�̵߳�����ʾ����"));
	//��ɫ
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointxyzrgb);
	viewer->addPointCloud(pointxyzrgb, rgb, "Features Show");
	//����
	viewer->setBackgroundColor(backgroudcolor.r, backgroudcolor.g, backgroudcolor.b);
	//���ش���
	return(viewer);

}//end 

PointCloudViewerPtr HpdDisplay::Showfeatureresult(pcl::PointCloud<pcl::PointXYZ> & vCloud, std::vector<float> & features, char *keyword) {
	//���ж��ǲ���ͬһ�����Ƽ���
	if (vCloud.points.size() != features.size()) {
		std::cout << "Error:���Ƽ���������������ƥ�䣡" << std::endl;
		std::cout << "�밴�س���������ȷ����������" << std::endl;
		std::cin.get();
		exit(0);
	}
	//********************�ж������Ѹ�������ֵ�����������¸���***************************
	if (colorvarflag) {//����ǰ���õ�ѡ
		//ʲô�����ɣ�ǰ���Ѿ�����
	}
	else {//��������ʽѡ
		//��ɫѡ�����
		char *dictionary[5] = { "grey", "redgreen", "greenblue", "redblue", "free" };//ָ������
		int keynumber;
		for (keynumber = 0; keynumber<5; keynumber++)
		{
			if (!strcmp(keyword, dictionary[keynumber]))
				break;
		}

		switch (keynumber) {
			//******************ģʽ1���̱仯��ʾ
		case 0: {
			//������ɫ
			headcolor = colors[0];
			tailcolor = colors[1];
			//��Ӧ������ɫ
			backgroudcolor = colors[4];
			break;
		}
			//******************ģʽ2�̺�
		case 1: {
			//������ɫ
			headcolor.r = 0.0;
			headcolor.g = 255.0;
			headcolor.b = 0.0;
			tailcolor.r = 255.0;
			tailcolor.g = 0.0;
			tailcolor.b = 0.0;
			//��Ӧ������ɫ
			backgroudcolor = colors[0];
			break;
		}
			//******************ģʽ3������ʾ
		case 2: {
			//������ɫ
			headcolor.r = 189.0;
			headcolor.g = 183.0;
			headcolor.b = 107.0;
			tailcolor.r = 0.0;
			tailcolor.g = 107.0;
			tailcolor.b = 139.0;
			//��ɫ
			backgroudcolor = colors[0];
			break;
		}
			//******************ģʽ4������ʾ
		case 3: {
			//������ɫ
			headcolor.r = 9.0;
			headcolor.g = 50.0;
			headcolor.b = 204.0;
			tailcolor.r = 249.0;
			tailcolor.g = 50.0;
			tailcolor.b = 204.0;
			//������ϲ���ı�����ɫ
			backgroudcolor.r = 0.33; backgroudcolor.g = 0.67; backgroudcolor.b = 0.58;
			break;
		}
			//******************ģʽ4����ѡ����ɫ
		case 4: {
			int cc;
			displaytable();
			std::cout << "���ʴ��ˣ�ͷ��ɫ��ľ�����? ���������" << std::endl;
			std::cin >> cc;
			headcolor = colors[cc];
			std::cout << "���ʴ��ˣ�β��ɫ��ľ�����? ���������" << std::endl;
			std::cin >> cc;
			tailcolor = colors[cc];
			//������ϲ���ı�����ɫ
			std::cout << "�������ʣ�������ɫ������~~~��Ҫѡ���ĸ�?(\\^0^)// ���������," << std::endl;
			std::cin >> cc;
			backgroudcolor = colors[cc];
			break;
		}
		default: {
			std::cout << "Interesting but inpossible! You choose something wrong!" << std::endl;
			std::cin.get();
			exit(0);
			break;
		}
		}//end switch
	}//end super big if

	//��������ֵ�ķ�Χ�������һ��
	double minvalue = features[0], maxvalue = features[0];
	for (size_t i = 1; i<features.size(); ++i) {
		if (features[i]<minvalue)
			minvalue = features[i];
		if (features[i]> maxvalue)
			maxvalue = features[i];
	}
	//һ����ɫ�������ڱ�����ɫ��Ϣ
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointxyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);

	//PCL RGB��ʽ
	unsigned long pointNum = vCloud.points.size();
	pointxyzrgb->width = pointNum;
	pointxyzrgb->height = 1;
	pointxyzrgb->is_dense = false;
	pointxyzrgb->points.resize(pointxyzrgb->width*pointxyzrgb->height);
	//��ɫ
	for (size_t i = 0; i<pointxyzrgb->points.size(); i++) {
		pointxyzrgb->points[i].x = vCloud.points[i].x;
		pointxyzrgb->points[i].y = vCloud.points[i].y;
		pointxyzrgb->points[i].z = vCloud.points[i].z;
		//��������ֵ�ı仯����
		//�����ɫ�����Ƚ��ʺ���ɫ����
		pointxyzrgb->points[i].r = Colorvar(headcolor.r, tailcolor.r, (features[i] - minvalue) / (maxvalue - minvalue));
		pointxyzrgb->points[i].g = Colorvar(headcolor.g, tailcolor.g, (features[i] - minvalue) / (maxvalue - minvalue));
		pointxyzrgb->points[i].b = Colorvar(headcolor.b, tailcolor.b, (features[i] - minvalue) / (maxvalue - minvalue));
	}
	//����һ��PCL����ʾ����
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("�̵߳�����ʾ����"));
	//��ɫ
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointxyzrgb);
	viewer->addPointCloud(pointxyzrgb, rgb, "Features Show");
	//����
	viewer->setBackgroundColor(backgroudcolor.r, backgroudcolor.g, backgroudcolor.b);
	//���ش���
	return(viewer);

}//end 

PointCloudViewerPtr  HpdDisplay::Creatgeometry(PointCloudXYZPtr & cloud,Point3D & centerp){
	//���Բ��
	pcl::PointXYZ o;
	o.x=centerp.x;
	o.y=centerp.y;
	o.z=centerp.z;
	//����һ��PCL����ʾ����
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("�̵߳�����ʾ����"));
	//PCL��ɫ
	viewer->addPointCloud(cloud,"Key points");	
	viewer->addSphere(o,0.1,"sphere",0);
	//����
	viewer->setBackgroundColor(backgroudcolor.r, backgroudcolor.g, backgroudcolor.b);
	//���ش���
    return(viewer);
}
/*================================
random�������
���ܣ��������Ҫ����������ʾ�����ĵ����ѡ����ʾ�ͷ����������ɫ�������ʾ
=================================*/
//һ������������ĺ��������Թ涨�������Χ
double HpdDisplay::random(double start, double end)
{
    return start+(end-start)*rand()/(RAND_MAX + 1.0);
}

//
/*================================
displaytable()�������
���ܣ�������
=================================*/
void HpdDisplay::displaytable(){
	std::cout<<"      ��ɫ�����£�"<<std::endl;
	for(size_t i=0;i<colors.size();i++)
		std::cout<<"���"<<i<<"��"<<colors[i].name<<std::endl;
}
/*================================
Colorvar()�������
���ܣ����㽥����ɫ�ж�Ӧ����ɫֵ
���룺headnum ������ɫ��ͷ��ɫ��ֵ��������r,g,b
      tailnum β��ɫ
	  proportion ����ֵ��ԭ�����е�λ�ñ���
�����������ɫֵ
=================================*/
float Colorvar(float headnum,float tailnum,float proportion){
	return headnum+(tailnum-headnum)*proportion;
}

/*================================
AddLineWithPoints
=================================*/
void HpdDisplay::AddLineWithPoints(boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer,
		                           std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & vSkelinePoints,
	                               float fSRadius, float fLr, float fLg, float fLb,
	                               float fSr, float fSg, float fSb) {
	
	for (int k = 0; k != vSkelinePoints.size(); ++k) {

		for (int i = 0; i != vSkelinePoints[k]->points.size() - 1; ++i) {
			
			std::stringstream sLinestream;
			sLinestream << k << "_" << i << "_" << i + 1 << "_Line";
			std::string sLineName;
			sLinestream >> sLineName;

			viewer->addLine<pcl::PointXYZ>(vSkelinePoints[k]->points[i], vSkelinePoints[k]->points[i + 1], fLr, fLg, fLb, sLineName);
			
			std::stringstream sSpherestream;
			sSpherestream << k << "_" << i << "_Sphere";
			std::string sSphereName;
			sSpherestream >> sSphereName;
			viewer->addSphere(vSkelinePoints[k]->points[i], fSRadius, fSr, fSg, fSb, sSphereName);
		}//end for i

		int iTailNum = vSkelinePoints[k]->points.size() - 1;
		std::stringstream sTailStream;
		sTailStream << k << "_" << iTailNum << "_Sphere";
		std::string sTailName;
		sTailStream >> sTailName;
		viewer->addSphere(vSkelinePoints[k]->points[iTailNum], fSRadius, fSr, fSg, fSb, sTailName);

	}//end for k

}

/*================================
AddSphereAtPoints
=================================*/
void HpdDisplay::AddSphereAtPoints(boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer,
	pcl::PointCloud<pcl::PointXYZ>::Ptr & vSkelinePoints,
	float fSRadius, float fSr, float fSg, float fSb) {

	for (int k = 0; k != vSkelinePoints->points.size(); ++k) {

			std::stringstream sSpherestream;
			sSpherestream << k << "_ball";
			std::string sSphereName;
			sSpherestream >> sSphereName;
			viewer->addSphere(vSkelinePoints->points[k], fSRadius, fSr, fSg, fSb, sSphereName);

	}//end for k

}