#include"readtxt.h"

/*================================================
ReadMatrix ����
���� ��ȡtxt����洢�ľ���
���룺�ļ�����Ҫ���̵�����
��������̵�������ֵ
=================================================*/
void ReadMatrix(string fileName,std::vector<std::vector<double>> & feamatrix,int n){
	feamatrix.clear();
	//���ļ�
  ifstream ifile(fileName.c_str(),ios::in);
  double tmp;//�м���
  string line;//һ�е�ԭʼ��
  vector<double> row;//һ�е��м���
  //**��ķ���
  int linenum=0;
  //go
  while(getline(ifile,line)){//��ȡһ�У�һ��һ�и��Ҷ�
       istringstream istr(line);//�����ո����˺�ɼ���
	   //��һ����һ��
       while(istr>>tmp){
       row.push_back(tmp);
       }//end small while
	   //�����м����ƽ�����ֱ������
  if(!(linenum%n))//�Ƿ������n=1��������Ĭ�ϣ�
  feamatrix.push_back(row);
  row.clear();
  istr.clear();
  line.clear();
  linenum++;
  }//end big while
//���Ӳ���
  ifile.close();
}

/*================================================
����Extlabelatend
���� ��������е�����������������ڱ��������������һ�����
���룺���̵��������������
ע��ԭʼ������������������Ϊ���һ��Ԫ��
��������������ֵ����������ɾ�����Ԫ��
=================================================*/
void Dividefeandclass::Extlabelatend(std::vector<std::vector<double>> & set_feature, 
	std::vector<int> & set_classes ){
		//������ֵ
	for(size_t i=0;i!=set_feature.size();i++){
		set_classes.push_back(set_feature[i][set_feature[i].size()-1]);
		set_feature[i].pop_back();
	}//end for
}
/*================================================
����Extlabelatbegin
���� ȥ��iccv�ڵ�����м���һ��
���룺iccv���������
�����iccv_feature iccv����
=================================================*/
void Dividefeandclass::Extlabelatbegin(std::vector<std::vector<double>> & set_feature,
	std::vector<int> & set_classes){
	//perfect
	vector<double>::iterator ite;
	for(size_t i=0;i!=set_feature.size();i++){
		ite=set_feature[i].begin();
		set_classes.push_back(*ite);
		set_feature[i].erase(ite);
		//over
	}//end for

}
void Dividefeandclass::Extlabelatbegin(std::vector<std::vector<double>> & set_feature,
	std::vector<double> & ext_feature){
	//perfect
	vector<double>::iterator ite;
	for(size_t i=0;i!=set_feature.size();i++){
		ite=set_feature[i].begin();
		ext_feature.push_back(*ite);
		set_feature[i].erase(ite);
		//over
	}//end for

}
/*================================================
����Removesomefeature
���� ֻ������������
���룺��������vec_feature ����������������retaindices
������µ���������vec_feature
=================================================*/
void Dividefeandclass::Removesomefeature(std::vector<std::vector<double>> &vec_feature, 
	std::vector<int> & retaindices){
    //һ���м����
	std::vector<double> oneres(retaindices.size(),0.0);
	std::vector<std::vector<double>> midvec;
	//�洢����
	for(size_t i=0;i!=vec_feature.size();i++){
		for(size_t j=0;j!=retaindices.size();j++){
		oneres[j]=vec_feature[i][retaindices[j]];
		}//over
		midvec.push_back(oneres);
	}//end for
	vec_feature.clear();
	vec_feature=midvec;
}
/*================================================
����Sampling
���� ����
���룺std::vector<std::vector<double>> & feamatrix double�͵Ķ�ά����
      n ������
������µ�����������Ȼ������feamatrix
=================================================*/
void Sampling(std::vector<std::vector<double>> & feamatrix,int n){
	//perfect new
	std::vector<std::vector<double>> newfea;
	//count i
	for(int i=0;i!=feamatrix.size();i=i+n)
	newfea.push_back(feamatrix[i]);
	
	//rebuild feamatrix
	feamatrix.clear();
	feamatrix=newfea;
}



void WritePointCloudTxt(string fileName, pcl::PointCloud<pcl::PointXYZ> & vCloud, std::vector<float> & vFeatures){

	std::ofstream oOutFile;
	oOutFile.open(fileName, std::ios::ate | std::ios::out);

	//output
	for (int i = 0; i != vCloud.size(); ++i){
		oOutFile << vCloud.points[i].x << " "
			          << vCloud.points[i].y << " "
			          << vCloud.points[i].z << " "
			          << vFeatures[i] << " "
			          << std::endl;
	}

	oOutFile.close();

}

void WritePointCloudTxt(string fileName, pcl::PointCloud<pcl::PointXYZ> & vCloud, std::vector<int> & vClasses){

	std::ofstream oOutFile;
	oOutFile.open(fileName, std::ios::ate | std::ios::out);

	//output
	for (int i = 0; i != vCloud.size(); ++i){
		oOutFile << vCloud.points[i].x << " "
			<< vCloud.points[i].y << " "
			<< vCloud.points[i].z << " "
			<< vClasses[i] << " "
			<< std::endl;
	}

	oOutFile.close();

}

void WritePointCloudTxt(string fileName, pcl::PointCloud<pcl::PointXYZ> & vCloud){

	std::ofstream oOutFile;
	oOutFile.open(fileName, std::ios::ate | std::ios::out);

	//output
	for (int i = 0; i != vCloud.size(); ++i){
		oOutFile << vCloud.points[i].x << " "
			<< vCloud.points[i].y << " "
			<< vCloud.points[i].z << " "
			<< std::endl;
	}

	oOutFile.close();

}