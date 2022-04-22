/*========================
Huang pengdi����ѧ��ʽ����ͷ�ļ�
����һЩ���õ�����ѧ��ʽ���������
Firstly edited in 09.12.2014 secondly edited 05.24.2015
I mean large scale edition time
�Ǻ�
==========================*/
#ifndef HPDMATH_H //���ظ�����ͷ�ļ�
#define HPDMATH_H
#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <algorithm>
#include <pcl/common/eigen.h>
#define PI 3.1415926
using namespace std;
//����õ���
struct convector{
   float* pStart;
   int length;
}; 
//wavelet
class Hpdwavelet{

public:
//*******************��ʼ����********************
Hpdwavelet();
void clear(char* keyword="same");
void Setemplv(float f_rm,int);
//*******************���ܺ���********************
//��ֺ���
void Diff(std::vector<float> &);
//С�����ɺ�����ֻ��MarrС������ΪҪ�õ���
void Conswavtemplate(int num=6,float m=-2.0,char* funame="marr");
//�����������㺯��
float Normvector(std::vector<float> & nvect,int p=2);
//�������ĺ���
std::vector<float> Conv(std::vector<float> &,std::vector<float> &);
//�����źų���
std::vector<float> wkeep( const std::vector<float> &v, int length,
                    char* direction="center" );
//��ȡ���
int Findsigularity(std::vector<float> & orisignals);
//����
std::vector<float> moduleresults;//ģֵ���
private:
//�洢����
    std::vector<float> diffarray;//��ֱ���
//ģ��
    int ln;//ģ�峤��
	float rm;
	bool flagm;
    std::vector<float> waveletemplate;
//������
    std::vector<float> convresult;

};
//*****************************************ȫ�ֺ���**********************************
/*===========================
Maxofall����
���ã���ȡ�����ڵ����ֵ������
============================*/ 
int Maxofall(std::vector<float> & absvector);
int Minofall(std::vector<float> & vec);
/*===========================
Normvector����
���ã���ȡ������ģֵ
============================*/ 
double Normvector(std::vector<double> & nvect,int p);
/*===========================
Removerepetitionr����
���ã�ȥ����������ͬ��Ԫ��
============================*/ 
void Removerepetition();
/*===========================
Gomputedis����
���ã���ȡ�����ڵ����ֵ���������ֵ������
============================*/ 
template <typename T>
T Getmax(std::vector<T> & vec){
	T max;
	max=vec[0];
	for(size_t i=0;i!=vec.size();i++){
	if(vec[i]>max)
	max=vec[i];
	}
	return max;
}
template <typename T>
std::vector<T> Getmaxmin(std::vector<T> & vec){
	std::vector<T> maxmin;
	T max;
	T min;
	//if non-empty
	if(vec.size()){
	    max=vec[0];
	    min=vec[0];
	   for(size_t i=0;i!=vec.size();i++){
	   if(vec[i]>max)
	      max=vec[i];
	   if(vec[i]<min)
		  min=vec[i];
	}//end for
	    maxmin.push_back(max);
		maxmin.push_back(min);
	}else{//if empty
		maxmin.push_back(0.0);
		maxmin.push_back(0.0);
	}
		return maxmin;
}
/*===========================
Gomputedis����
���ã���������֮���ŷʽ����
============================*/
double Gomputedis(double &, double &,double &, double &, double &,double &);
double Gomputedis(double &, double &, double &, double &);
/*===============================
Trangletoradian����
�Ƕ�ת�ɻ���
===================================*/
double Trangletoradian(double &);
/*===============================
computeanglecosine����
�Ƕ�ת�ɻ���
===================================*/
double Computeanglecosine(Eigen::Vector3f & vecone,Eigen::Vector3f & vectwo);
double Computeanglecosine(Eigen::Vector2d & vecone,Eigen::Vector2d & vectwo);
/*=======================================
Removevectormember����
����:ȥ���ƶ���vector��Ԫ��
����:����vector��׼��ȥ������������val��indices
========================================*/
template <typename T>
void Removevectormember(vector<T>& v,int val){
    vector<T>::iterator ite=v.begin();
    while(ite!=v.end()){
        if(*ite==val)
            ite=v.erase(ite);
        else
            ++ite;
    }
}
//����������indices����ʽ
template <typename T>
void Removevectormember(vector<T>& v,std::vector<bool> indices){
    vector<T>::iterator ite=v.begin();
	int n=0;
    while(ite!=v.end()){
        if(!indices[n])
            ite=v.erase(ite);
        else
            ++ite;
		n++;
    }
}
/*===============================
Numofsamenber����
Ѱ������������ͬԪ�صĸ���
ע������������Ԫ�ض����ظ�
===================================*/
double Numofsamenber(std::vector<int> &,std::vector<int> &);




#endif