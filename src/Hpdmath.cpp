#include"Hpdmath.h"
//******Edited by Huang Pengdi 2014.12.09 Xiamen Unversity******
/*===================================================
���캯��Hpdwavelet()

=================================================*/
Hpdwavelet::Hpdwavelet(){
	flagm=false;
	ln=0;
	rm=0.0;
}
/*===================================================
����Setscalem���
���ó߶�m
=================================================*/
void Hpdwavelet::Setemplv(float f_rm,int f_ln){
	ln=f_ln;
	rm=f_rm;
	flagm=true;
}
/*=================================================
����Diff()��飺�����źŵĲ�ֽ��
���룺ԭʼ�ź� std::vector<float> & orifun
���������ź� diffarray
==================================================*/
void Hpdwavelet::Diff(std::vector<float> & orifun){
	diffarray.clear();
	//ǰ���־���ô��
	for(size_t i=1;i<orifun.size();i++){
		diffarray.push_back(orifun[i]-orifun[i-1]);
	}
	//
};
/*==============================================
����clear��飺�������ڵ�ֵ
���룺��������������ڵ�ֵ
һ���ò���
===============================================*/
void Hpdwavelet::clear(char* keyword){
	diffarray.clear();//��ֱ���
    moduleresults.clear();
	convresult.clear();//������
	if (!strcmp(keyword,"all"))
	waveletemplate.clear();
}
/*==============================================
����Conswavtemplate��飺С�����ɺ���
���룺numС��ģ���С,m�ֱ���,funameС������
�����waveletemplateС���ź�
===============================================*/
void Hpdwavelet::Conswavtemplate(int num,float m,char* funame){
	if(flagm){
	m=rm;num=ln;
	}
	//��������
	if (!strcmp(funame,"marr")){
	waveletemplate.clear();
	//��߶�
		float delta=pow(2,m);
	//ģ
    //���ȣ�num���ȣ�
	float amplitude=-1/sqrt(2*PI);
	//ģ��
	float x;
	for(int n=0;n!=num;n++){
    //�����˹������ƫ����
	x=float(n+1)-float(num+1)/2;
	waveletemplate.push_back(amplitude*(x/pow(delta,2))*exp(-pow(x,2)/(2*pow(delta,2))));
	//
	      }//end for
	float normvect=Normvector(waveletemplate);
	for(int i=0;i!=num;i++){
	waveletemplate[i]=waveletemplate[i]/normvect;
	   }//end for
	}//end if
	if(!strcmp(funame,"harr")){
	/*����ʩ������������*/
	}
}
/*===============================================
����Normvector��飺������������
���룺std::vector<float> & nvect������p������Ĭ��2������
�����sum����ֵ
===============================================*/
float Hpdwavelet::Normvector(std::vector<float> & nvect,int p){
	if(p<0)
	p=2;//Υ����ʽǿ���û�
	float sum=0.0;
	//p����
	for(size_t i=0;i!=nvect.size();i++){
	sum=pow(fabs(nvect[i]),float(p))+sum;
	}
	sum=pow(sum,1/float(p));
	return sum;
}
/*===============================================
����Conv��飺���
���룺std::vector<float> & nvect������p������Ĭ��2������
�����sum����ֵ
===============================================*/
std::vector<float> Hpdwavelet::Conv(std::vector<float> & avector,std::vector<float> & bvector){
	//����������ļ��㵥Ԫ
 struct convector a,b,y;
 a.length =avector.size();
 b.length =bvector.size();
 a.pStart =new float [a.length];
 b.pStart =new float [b.length];
   //���ֵ
 for(size_t i=0;i!=avector.size();i++){
 *(a.pStart+i)=avector[i];
 }
 for(size_t i=0;i!=bvector.size();i++){
 *(b.pStart+i)=bvector[i];
 }
	//��ʼ��
 std::vector<float> yvector;
 float aa,bb;
 y.length =a.length +b.length-1 ;
 y.pStart =new float [y.length];

 for (int i=0;i<y.length ;i++)
 *(y.pStart +i)=0;

 if (a.length >=b.length ){  //���a�ĳ��ȴ��ڻ����b�ĳ���
 for (int i=0;i<b.length;i++){
    for (int j=0;j<=i;j++){
     aa=*(a.pStart+i-j);
     bb=*(b.pStart +j);
     *(y.pStart +i)=*(y.pStart +i)+aa*bb;
    }
 }

 for (int i=b.length;i<a.length;i++){
    for (int j=0;j<b.length;j++){
        aa=*(a.pStart+i-j);
        bb=*(b.pStart +j);
        *(y.pStart +i)=*(y.pStart +i)+aa*bb;
     }
 }

 for (int i=a.length;i<a.length+b.length;i++){
    for (int j=i-a.length+1;j<b.length;j++){
      aa=*(a.pStart+i-j);
      bb=*(b.pStart +j);
      *(y.pStart +i)=*(y.pStart +i)+aa*bb;
     }
    }

 }//end if
 else  //���b�ĳ��ȴ��ڻ����a�ĳ���
{
for(int i=0;i<a.length;i++){
     for (int j=0;j<=i;j++){
     bb=*(b.pStart+i-j);
     aa=*(a.pStart +j);
     *(y.pStart +i)=*(y.pStart +i)+aa*bb;
    }
 }

 for (int i=a.length;i<b.length;i++){
    for (int j=0;j<a.length;j++){
       bb=*(b.pStart+i-j);
       aa=*(a.pStart +j);
       *(y.pStart +i)=*(y.pStart +i)+aa*bb;
    }
 }

 for (int i=b.length;i<b.length+a.length-1;i++){
     for (int j=i-b.length+1;j<a.length;j++){
       bb=*(b.pStart+i-j);
       aa=*(a.pStart +j);
       *(y.pStart+i)=*(y.pStart +i)+aa*bb;
    }
 }
 }//end else
 //�����ֵ
 for(int i=0;i<y.length;i++)
	 yvector.push_back(*(y.pStart+i));
 //�鷳���ƺ���
 delete [] a.pStart ;
 delete [] b.pStart ;
 delete [] y.pStart ;
 return(yvector);
 } 
/*===============================================
/**�����ҵ�wkeep���־������ȷ���ȵĺ���
 * Keep part of vector.
 * For a vector, w = wkeep(v,L,opt) extracts the vector w from the vector v.
 * The length of w is L. If direction = "center" ("left", "rigth",
 * respectively), w is the central (left, right, respectively) part of v.
 * w = wkeep(x,L) is equivalent to w = wkeep(v,L,"center").
 * w = wkeep(v,L,first) returns the vector v[first] to v[first+L-1].
===============================================*/
std::vector<float> Hpdwavelet::wkeep( const std::vector<float> &v, int length,
                    char* direction ){
    int lv = v.size();
	std::vector<float> tmp(length);

    if( ( 0 <= length ) && ( length <= lv ) )
    {
        if(!strcmp(direction,"right")) 
            for( int i=0; i<length; ++i )
                tmp[i] = v[lv-length+i];
        else if(!strcmp(direction,"left")) 
            for( int i=0; i<length; ++i )
                tmp[i] = v[i];
        else
        {
            int first = (lv-length)/2;
            for( int i=0; i<length; ++i )
                tmp[i] = v[first+i];
        }

        return tmp;
    }
    else
    {
        cerr << "Invalid length input." << endl;
        return tmp;
    }
}
/*==========================
����Maxofall
�β�1��std::vector<float>������
�����������ֵ��������ע������������
==========================*/
int Maxofall(std::vector<float> & absvector){
   int pos=0;
   float max=-DBL_MAX;
   for(int i=0;i!=absvector.size();i++){
       if(absvector[i]>max){
          max=absvector[i];
          pos=i;//������
       }
   }
   return pos;
}
/*==========================
����Minofall
�β�1��std::vector<float>������
����������С��������ע������������
==========================*/
int Minofall(std::vector<float> & vec){
   int pos=0;
   float min=DBL_MAX;
   for(int i=0;i!=vec.size();i++){
       if(vec[i]<min){
          min=vec[i];
          pos=i;//������
       }
   }
   return pos;
}
/*==========================
����Findsigularity
�β�1��std::vector<float>������
�����������ֵ��������ע������������
==========================*/
int Hpdwavelet::Findsigularity(std::vector<float> & orisignals){
	//�Ȳ�ֻ�ȡһ�׵���ͻ�亯��
	Diff(orisignals);
	//����С������
	Conswavtemplate();
	//�������С������
	convresult.clear();
	//���ǵ���һ��ֵͻ��̫����
	float veczero=diffarray[0];
	for(size_t i=0;i!=diffarray.size();i++)
    diffarray[i]=diffarray[i]-veczero;
	//���coven...
	convresult=Conv(diffarray,waveletemplate);
	//�˶Ե�ԭ�����ź���
	moduleresults.clear();
	moduleresults=wkeep(convresult,diffarray.size());
	//ȡģֵ
	for(size_t i=0;i!=moduleresults.size();i++){
	moduleresults[i]=fabs(moduleresults[i]);
	}
	//��ȡ���ֵλ��
	int numres=Maxofall(moduleresults);
	return numres+1;//������Ϊ��ֺ�����ԭ������һ��ֵ����һ��ֵ��
}
/*==========================
����Normvector������ģֵ������Ǻܳ��õ�Ŷ
�β�1��std::vector<float>������
�����������ֵ��������ע������������
==========================*/
double Normvector(std::vector<double> & nvect,int p){
	if(p<0)
	p=2;//Υ����ʽǿ���û�
	double sum=0.0;
	//p����
	for(size_t i=0;i!=nvect.size();i++){
	sum=pow(fabs(nvect[i]),double(p))+sum;
	}
	sum=pow(sum,1/double(p));
	return sum;
}
/*==========================
����Removerepetition
ȥ����������ͬ��Ԫ��
==========================*/
void Removerepetition(std::vector<int> & iv){
//����
sort(iv.begin(),iv.end());  
//ɾ�����ڵ���ֵͬ
vector<int>::iterator ix=unique(iv.begin(),iv.end()); 
iv.erase(ix,iv.end()); 
}
/*==========================
����Gomputedis������ģֵ������Ǻܳ��õ�Ŷ
�β�1������1��x,y,zֵ
�β�2������2��x,y,zֵ
�����ŷʽ���루ע������������
����Ϊ�����ά��
==========================*/
double Gomputedis(double & x1, double & y1,double & z1, double & x2, double & y2,double & z2){
	return sqrt(pow(x1-x2,2.0)+pow(y1-y2,2.0)+pow(z1-z2,2.0));
}
double Gomputedis(double & x1, double & y1, double & x2, double & y2){
	return sqrt(pow(x1-x2,2.0)+pow(y1-y2,2.0));
}
/*===============================
Trangletoradian����
�Ƕ�ת�ɻ���
===================================*/
double Trangletoradian(double & angle){
	return angle*PI/180.0;
}
/*===============================
Trangletoradian����
�Ƕ�ת�ɻ���
===================================*/
double Computvanglecosine(Eigen::Vector3f & vecone,Eigen::Vector3f & vectwo){
	double dotres=vecone.dot(vectwo);
	return dotres/(vecone.norm()*vectwo.norm());
}
double Computeanglecosine(Eigen::Vector2d & vecone,Eigen::Vector2d & vectwo){
	double dotres=vecone.dot(vectwo);
	return dotres/(vecone.norm()*vectwo.norm());
}

double Numofsamenber(std::vector<int> & tarvec,std::vector<int> & modvec){
	int n=0;
	for(int i=0;i!=tarvec.size();i++){
		for(int j=0;j!=modvec.size();j++){
			if(tarvec[i]==modvec[j]){
			n++;
			break;
			}
	    }
	}
	return double(n)/double(tarvec.size());
}