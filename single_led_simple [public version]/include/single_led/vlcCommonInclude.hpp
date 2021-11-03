#ifndef vlcCommonInclude_COMMON_INCLUDES_hpp_
#define vlcCommonInclude_COMMON_INCLUDES_hpp_
//-----------------------------------��ͷ�ļ��������֡�------------------------------ 
//      ����������������������ͷ�ļ�
//----------------------------------------------------------------------------------------------  
#include <ros/ros.h>
#include <iostream> //C++��׼���������  
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>
//roi����
#include <opencv2/opencv.hpp>
#include <time.h>
#define PI 3.1415926

//-----------------------------------�������ռ������֡�------------------------------ 
//      ����������������ʹ�õ������ռ�  
//----------------------------------------------------------------------------------------------  
using namespace cv;
using namespace std;

//----------------------------------�����ṹ�塿--------------------------------------------
//      ������������ֽṹ��  
//----------------------------------------------------------------------------------------------- 

struct position{// LED��λ�ã���Ӧ��ͬλ�õĵƾ�
	int max;	// ID_max,���������Ŀ 	
	int min;	// ID_min����С������Ŀ
	double X;	// LED�ƾߵ���ʵλ��,x����
	double Y;	// LED�ƾߵ���ʵλ��,y����
	};


struct LED{	// LED������̵Ľṹ�壬���ڴ��ͼ��������е���Ϣ�Լ�������
	int ID;								//	ID,������Ŀ
	double imgLocalX, imgLocalY;	// LED��ͼ���ϵ���������λ�ã�,x����,y����
	double X, Y; 						// LED�ƾߵ���ʵλ��,x����,y����
	Mat imgNext, matBinary;			
	int X_min, X_max, Y_min, Y_max;
	Mat imgCut;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	int num;
	};
#endif


#ifndef vlcMainInclude_
#define vlcMainInclude_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <image_transport/image_transport.h> /*image_transport ͷ�ļ�������ROSϵͳ�еĻ����Ϸ����Ͷ���ͼ����Ϣ */ 
#include <cv_bridge/cv_bridge.h>  
#include <sensor_msgs/image_encodings.h> /* ROSͼ�����͵ı��뺯�� */ 
#include <std_msgs/String.h>

/*���Ա����ʽ  message_filters::Synchronizer��ʱ��ͬ����*/
#include <message_filters/subscriber.h>
/*��Ϣͬ����װ�������õ�ͷ�ļ�*/
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
/*��Ϣͬ����ȫ�ֱ�������ʽ���õ�ͷ�ļ�*/
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#endif