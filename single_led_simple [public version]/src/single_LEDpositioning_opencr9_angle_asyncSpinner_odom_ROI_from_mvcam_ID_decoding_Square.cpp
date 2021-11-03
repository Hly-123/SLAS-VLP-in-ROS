
//-----------------------------------��ͷ�ļ��������֡�------------------------------
//      ����������������������ͷ�ļ�   
//---------------------------------------------------------------------------------------------- 
#include<single_led/vlcCommonInclude.hpp>
#include <single_led/imgProcess_ID.hpp>
#include<sensor_msgs/Imu.h>
#include<sensor_msgs/MagneticField.h>
#include <stdio.h>
#include <string>
#include <math.h>
#include <iostream>
#include <fstream>
#include <boost/thread.hpp>
#include "tf/transform_datatypes.h"
#include <nav_msgs/Odometry.h> 
// #include <geometry_msgs/Point.h>
// #include <geometry_msgs/Quaternion.h> 
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

//��֪���Ҳ��ҵõ�
//�ó��������װ�ڱ��˵�����slo_vlp_msg����Ϊsingle_led
// #include "single_led/Rect_image.h"
#include "slo_vlp_msg/Rect_image.h"
//#include <single_led/sensor_angle.h> //�Զ��廰����Ϣ����ʵû��Ҫ��std_msgs::float64Ҳû��Ҫ��ֱ��double��

//-----------------------------------������������MySyncPolicy��------------------------------ 
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Imu, sensor_msgs::MagneticField> MySyncPolicy;
// const double PI=acos(-1.0);

ros::Duration d(0.01);

//-----------------------------------�������ռ������֡�------------------------------ 
//      ����������������ʹ�õ������ռ�  
//----------------------------------------------------------------------------------------------  
using namespace cv;
using namespace std;

double getThreshVal_Otsu_8u(const cv::Mat& _src);
void ls_LED(const Mat& _img, int& X_min, int& X_max, int& Y_min, int& Y_max, Mat& img_next);
void LED_contour(const Mat& _img, vector<Point> &myPoints, vector<int> &myRadius);
void para_set(double& X, double& Y, double& D_LED, double& f, double& led_x, double& led_y, double& all_H);
double get_similarity(Mat&a,Mat&b);
Rect find_minrect(Mat frame,Rect selected,int thresh);
Rect ls_LED_sample(Mat img, int& X_min, int& X_max, int& Y_min, int& Y_max, cv::Mat& imgNext,int sample);
void thinImage(Mat &srcimage);

//ID�������׺���
cv::Mat_<float>::Mat ImagePreProcessing(cv::Mat imgLED, int backgroundThreshold,int &backgroundCompensation);
cv::Mat matShift(cv::Mat frame, int shiftCol, int shiftRow);
cv::Mat LEDMeanRowCrestsTroughs(const cv::Mat_<float>::Mat imgRow, int BlurSize);
cv::Mat LEDMeanRowThreshold(cv::Mat imgRow, cv::Mat NonZeroLocations, int backgroundThreshold, int &backgroundCompensation);
cv::Mat convertPxielRowToBit(cv::Mat row,int continuousnum);
cv::Mat getMsgDate(cv::Mat imgRow, cv::Mat headerStamp,int idlength);//
int subregions(cv::Mat row);
int countstripes(cv::Mat imageLED, int backgroundThreshold,int &backgroundCompensation,int Blursize);
cv::Mat MsgProcess(cv::Mat imageLED, cv::Mat headerStamp,int backgroundThreshold,int &backgroundCompensation,int Blursize,int continuousnum,int idlength);//

/*
���壺һ��ת����sensor_msgs::Iamge-->cv::Mat�����ࣺʵ��ͼ��ת����ͬʱ����ͼ��λ����
���룺ͬʱ�����ͼƬsensor_msgs::Image�͵شżƽǶ�single_led::sensor_angle
*/
class IMAGE_LISTENER_and_LOCATOR
{
private:
	ros::NodeHandle nh_; //// ����һ���ڵ�������ROSϵͳ����ͨ��
	image_transport::ImageTransport it_; //����һ��image_transportʵ��  
	image_transport::Subscriber image_sub_; //����ROSͼ�������  
	ros::Subscriber point_info_sub;
	image_transport::Publisher image_pub_;

	//���������ߣ����涨��
	ros::Publisher msgPointPub;
	ros::Publisher ekfmsgPointPub;//����ekf������
	//����������
	ros::Subscriber odom_sub;//odom�Ķ�����
	ros::Subscriber roi_sub;//ROI�Ķ�����

	//-------------�����ӵ�-------------
	ros::Subscriber mag_sub;//�شŵĶ�����

	//ȫ�ֵ���Ԫ��
	nav_msgs::Odometry odom_msg_q;

	//д���ļ���(д��Ƕȵ�)
	ofstream fout_angle;

	//������ROI�����ı�־λ,�ж��Ƿ�������ǰ��ID
	int flag_roi;
	//����LED�ṹ��
	struct LED unknown;

	tf::Quaternion quat;
	double roll;
	double pitch;
	double yaw;
	float q0;
	float q1;
	float q2;
	float q3;

	float mag_x;
	float mag_y;
	float mag_z;

	//---------�����ӵ�-------------
	double mag_yaw;

	cv_bridge::CvImagePtr cv_ptr;

	int tarwidth, tarheight;
	bool firstframe;

	//ͼ����LED����������
	double Img_local_X;
	double Img_local_Y;

	//ͼ����LED��ֱ��;
	double D;
	double width_from_mvcam;
	double height_from_mvcam;

	geometry_msgs::Point point;
	Mat IDdata;
	//�ļ�������
    ofstream id_result;

//--------------------------�����ӵ�------------------------------
    cv::Point leftside,rightside,upside,downside;
	double testangle;
	double smallangle;
	vector<double> findangles;
	double realangle;
    Rect selectrect;
//--------------------------------------------------------------

	//����޵��ж�����
	// Mat imgNext;
	// Mat grayimage;

	// //��λ��ص���������
	// //�����ͼ����������(��ǰ�ǻ����˵�������Ķ�׼���·�)
	// double Center_X;
	// double Center_Y;
	// //LED�ƾߵ���ʵֱ��
	// double D_LED;
	// //����
	// double f;
	// //LED����������ϵ�е�ˮƽλ������
	// double led_x;
	// double led_y;
	// //LED�������Ĵ�ֱ�߶�
	// double all_H;

	//д���ļ���
	// ofstream fout_mag;

public:
	IMAGE_LISTENER_and_LOCATOR()
		:it_(nh_) //���캯��  
	{	
		/*---------------����1-------------------*/
		// Center_X = 575;//685;//674;//������Ķ�׼�����·�670;//640;//429;
		// Center_Y = 499;//495;//447;//486;//480;//334.5;
		// D_LED = 180;//LED�ƾߵ���ʵֱ��
		// f = 2.917*312.5;//���ݸ߶Ƚ����Ľ���(2.6389)//3.111;//����ͼƬ��С��ͬ,���в�ͬ�ĵ�Ч����(1280*960��Ӧ2.5)
		// led_x = -800;
		// led_y = 0;
		// all_H = 2.85;

		/*---------------����2-------------------*/
		// Center_X = 653;//685;//674;//������Ķ�׼�����·�670;//640;//429;
		// Center_Y = 498;//495;//447;//486;//480;//334.5;
		// D_LED = 103;//LED�ƾߵ���ʵֱ��
		// f = 2.596*312.5;//���ݸ߶Ƚ����Ľ���(2.6389)//3.111;//����ͼƬ��С��ͬ,���в�ͬ�ĵ�Ч����(1280*960��Ӧ2.5)
		// led_x = 0;
		// led_y = 0;
		// all_H = 1.57;
		fout_angle.open("odom_angle.txt");
		id_result.open("id_result.txt");

		fout_angle << "angle" << endl;
		firstframe=true;

		//�����߽���
		// image_sub_ = it_.subscribe("/mvcam/image", 1, &IMAGE_LISTENER_and_LOCATOR::convert_callback, this);
		roi_sub = nh_.subscribe("/mvcam1/image", 1, &IMAGE_LISTENER_and_LOCATOR::convert_callback, this);
		odom_sub = nh_.subscribe("odom", 1, &IMAGE_LISTENER_and_LOCATOR::odom_callback, this);
		
		//-----------�����ӵ�--------------
		mag_sub = nh_.subscribe("magnetic_field", 1, &IMAGE_LISTENER_and_LOCATOR::mag_callback, this);

		//���巢���Ļ���
		msgPointPub = nh_.advertise<geometry_msgs::PointStamped>("/location_info", 1000); //�����λ������
		image_pub_ = it_.advertise("/location/image_show", 1); //����ROSͼ�󷢲���
		//ekf������ȥ�Ļ���
		//����������"/slovlp_ekf_info"
		//��Ϣ�ļ������߶��У�queue���Ĵ�С����Ϊ1000
		//��Ӧ�����ݸ�ʽ��ҪΪ��geometry_msgs::PoseWithCovarianceStamped����Ӧ����Ϣ���ݣ�
		// ������Ҫ��PoseWithCovarianceStampedl��Ϣ�ļ���ʽ����һ����Ϣ
		ekfmsgPointPub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/slovlp_ekf_info", 1000); //�������ekf���
		// �������Ϣ������Ӧ������ekf�����ġ�
		// ��ekf�У�����ѡ��pose��ΪVLC�����롣��ROS�е�pose message�����֣�
		//1��geometry_msgs/Pose.msg��2��geometry_msgs/PoseWithCovarianceStamped.msg
		//�ο����ӣ�http://docs.ros.org/en/melodic/api/robot_localization/html/preparing_sensor_data.html
		//��֪����Ӧ��Ϊgeometry_msgs/PoseWithCovarianceStamped
		

		unknown.ID=0;
		// fout_mag.open("odom_angle.txt");
	}

	~IMAGE_LISTENER_and_LOCATOR() //��������
	{
		// delete image_sub;
		// delete imu_sub;
		// delete mag_sub;
		// delete sync;
		// fout_mag << endl;
		// fout_mag.close();
		fout_angle.close();
		id_result.close();
		cout << "close file success!" << endl;
		ROS_INFO("OUT CLASS!!");
	}


	//----------------------------��ROS��OpenCV�ĸ�ʽת���ص�������--------------------------------------
	//      ����������һ��ROS��OpenCV�ĸ�ʽת���ص���������ͼ���ʽ��sensor_msgs/Image  --->  cv::Mat 
	//--------------------------------------------------------------------------------------------------------------------------

	void convert_callback(const slo_vlp_msg::Rect_image::ConstPtr& rect_msg) {
		// cv_bridge::CvImagePtr cv_ptr;  // ����һ��CvImageָ���ʵ��

		// cv::Mat image_show;
		// ROS_INFO("I am coming CLASS in image_callback!");
		try {
			slo_vlp_msg::Rect_image rect_msg1 = *rect_msg;
			sensor_msgs::Image msg = rect_msg1.image_roi;
			Img_local_X = rect_msg1.Img_local_X;
			Img_local_Y = rect_msg1.Img_local_Y;
			D = max(rect_msg1.width, rect_msg1.height);
			width_from_mvcam = rect_msg1.width;
			height_from_mvcam = rect_msg1.height;

			//----------------------�����ӵ�---------------------
			selectrect.x=rect_msg1.Img_local_X-0.5*rect_msg1.width;
			selectrect.y=rect_msg1.Img_local_Y-0.5*rect_msg1.height;
			selectrect.width=rect_msg1.width;
			selectrect.height=rect_msg1.height;

			flag_roi = rect_msg1.flag_roi;
			cout << "flag_roi=" << flag_roi << endl;
			cout << "D=" << D << endl;
			// ��ROS��Ϣ�е�ͼ����Ϣ��ȡ��������cv���͵�ͼ�󣬸��Ƹ�CvImageָ��
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);//BGR8
			image_pub_.publish(msg);
		}
		catch (cv_bridge::Exception& e) {  // �쳣����
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		// global_image = cv_ptr ->image;

		// ROS_INFO("nowtime_yaw:%.5f", yaw);
		image_process(cv_ptr ->image,leftside,rightside,upside,downside,smallangle,realangle);
		d.sleep();
		//�õ���cv::Mat���͵�ͼ����CvImageָ���image�У���������͸�������
		//image_process(cv_ptr->image);

		//cv::flip(cv_ptr->image, image_show, 1);	//��ת��1���ң�0���£�-1��������ͬʱ
		// sensor_msgs::ImagePtr msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_show).toImageMsg();
		// image_pub_.publish(msg_image);//�������յ���ԭͼ	

	}

	void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
	{
		nav_msgs::Odometry odom_msg1 = *odom_msg;
		odom_msg_q=*odom_msg;
		// ROS_INFO("I am coming CLASS in odom_callback!");

		tf::quaternionMsgToTF(odom_msg1.pose.pose.orientation, quat);

		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
		yaw = yaw * 57.29578f;	//(-180,180)����ϵ
		// ROS_INFO("yaw_from_odom:%.5f", yaw);

		// fout_mag << yaw << "\n";
		
		d.sleep();
	}

	//-----------------�����ӵļ���ش�ƫ�ǵĺ���----------------------
		void mag_callback(const sensor_msgs::MagneticField::ConstPtr& mag_msg)
	{
		sensor_msgs::MagneticField mag_msg1 = *mag_msg;
		ROS_INFO("I am coming CLASS in magnetic_field_callback!");
		mag_x = mag_msg1.magnetic_field.x;
		mag_y = mag_msg1.magnetic_field.y;
		mag_z = mag_msg1.magnetic_field.z;
		mag_yaw = atan2(mag_y, mag_x);
		mag_yaw = mag_yaw * 57.29578f + 180;	//(-180,180)����ϵ
		// global_yaw = yaw;
		ROS_INFO("YAW_from_mag:%.5f", mag_yaw);
		d.sleep();
	} 

	//----------------------------------ͼ������------------------------------------------
	//      ������ͼ�������
	//-----------------------------------------------------------------------------------------------

	void image_process(cv::Mat img, Point& leftside, Point& rightside, Point& upside, Point& downside, double& smallangle, double& realangle)
	{
		ROS_INFO("image process start!~~");
		// cv::Mat img_out;
		ros::Rate loop_rate(1);
		int count = 0;

		std_msgs::String msg;
		geometry_msgs::PointStamped msgPointStamped;//
		//��PoseWithCovarianceStamped��Ϣ�ļ���ʽ����һ�� ����msgPWCStamped����Ϣ
		//��Ϣ��������ekfmsgPointPub������ȷ����
		geometry_msgs::PoseWithCovarianceStamped msgPWCStamped;//�������꼰��Ԫ��
		//����������ϢӦ������Ҫ�Ⱦ���tf�任����Ӧ������ϵ�ϵ�
		//����Ǹ�IMU�ںϵĻ���������ľ��Ǳ任����imuһ�µ�����ϵ��
		//�������ٱ任��geometry_msgs/PoseWithCovarianceStamped��
		//��geometry_msgs/PoseWithCovarianceStamped��Handled in the same fashion as the pose data in the Odometry message
		//�ʴ�Ӧ��ת�Ƶ�odom��

		std::stringstream sss;
		// cv::Mat image_show;

		testangle=mag_yaw;
		realangle=AngleCorrection(img,selectrect,testangle,smallangle);
		msgPointStamped.point = Get_coordinate(leftside,rightside,upside,downside,smallangle,realangle);
		// sss << '\n' << msgPointStamped.point.x*100
		// 	<< '\n' << msgPointStamped.point.y*100
		// 	<< '\n' << msgPointStamped.point.z*100 << count;
		// msg.data = sss.str();
		msgPointStamped.header.stamp = ros::Time::now();
		msgPointStamped.header.frame_id = "map";
		// msgPointPub.publish(msgPointStamped);//�������Ķ�λ���������ȥ

		//��������ӵ�msgPWCStamped��
		msgPWCStamped.pose.pose.orientation = odom_msg_q.pose.pose.orientation;//����odom����Ԫ��
		msgPWCStamped.pose.pose.position =msgPointStamped.point;
	
	// *****************tf�任
    geometry_msgs::PointStamped mvcam_map;
	mvcam_map.header.frame_id = "base_vlc_mvcam";
	mvcam_map.header.stamp = ros::Time::now();
	mvcam_map.point.x=-msgPWCStamped.pose.pose.position.x;
	mvcam_map.point.y=-msgPWCStamped.pose.pose.position.y;
	mvcam_map.point.z=-msgPWCStamped.pose.pose.position.z;
    geometry_msgs::PointStamped map_base;
    try
    {
        // listener.transformPose(self_odom, tmp_tf_stamped, odom_to_map);//"robot_1/odom"
        tf::TransformListener listener;
        ros::Duration(0.3).sleep();
        listener.transformPoint("base_link", mvcam_map, map_base);//"robot_1/odom"
        }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ROS_DEBUG("Failed to subtract base to odom transform");
        //return;
    } 
	// *****************
		// msgPWCStamped.pose.pose.position.x=-map_base.point.x*100;
		// msgPWCStamped.pose.pose.position.y=-map_base.point.y*100;
		// msgPWCStamped.pose.pose.position.z=-map_base.point.z*100;
		msgPWCStamped.pose.pose.position.x=-map_base.point.x;
		msgPWCStamped.pose.pose.position.y=-map_base.point.y;
		msgPWCStamped.pose.pose.position.z=-map_base.point.z;

		msgPointStamped.point.x = -map_base.point.x;
		msgPointStamped.point.y = -map_base.point.y;
		msgPointStamped.point.z = -map_base.point.z;

		sss << '\n' << msgPointStamped.point.x*100
			<< '\n' << msgPointStamped.point.y*100
			<< '\n' << msgPointStamped.point.z*100 << count;
		msg.data = sss.str();
		//Э�������
		msgPWCStamped.pose.covariance={0.7223,  0.0927,  0.1170,  0,  0,  0.1171,
									   0.0927,  1.1779, -0.2214,  0,  0,  0.4157,
									   0.1770, -0.2214,  9.0631,  0,  0,  0.5067,
									   0     ,  0     ,  0     ,  0,  0,  0     ,
									   0     ,  0     ,  0     ,  0,  0,  0     ,
									   0.1171,  0.4157,  0.5067,  0,  0,  3.4576};
		msgPWCStamped.header.stamp = ros::Time::now();
		msgPWCStamped.header.frame_id = "map";
		if(unknown.ID==1)
		{
		ekfmsgPointPub.publish(msgPWCStamped);//����ekf
		}
		else
		{
		ROS_INFO("Remain the last coordinate, BUT NOT PUBLISHED!!");
		
		}


		msgPointPub.publish(msgPointStamped);
		//ekfmsgPointPub.publish(msgPWCStamped);//����ekf


		// cout << "coordinate_out_time:\t" << msgPointStamped.header.stamp << "secs" << endl;
		ROS_INFO("%s", msg.data.c_str());
		//ROS_INFO("angle:%.5f", angle);
		// cv::flip(img, image_show, 1);
		//sensor_msgs::ImagePtr msg_image = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_show).toImageMsg();//�Ҷ�ͼ����
		// sensor_msgs::ImagePtr msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_show).toImageMsg();
		// image_pub_.publish(msg_image);
		++count;

	}


geometry_msgs::Point Get_coordinate(Point& leftside, Point& rightside, Point& upside, Point& downside, double& smallangle, double angle)
{
	// the key code temporarily is hid
	return point;
}


double AngleCorrection(Mat ROI,Rect minrect,double &testangle,double &smallangle)
{
    cv::Mat LED_edge;
    int minIdx[2] = {}, maxIdx[2] = {};
    double minVal,maxVal;
    int i,j,val;
    int bright=0;
    int dark=0;
    double angle;

    for(i=0;i<ROI.cols;i++)
    {
        for(j=0;j<ROI.rows;j++)
        {
            val=ROI.at<uchar>(j,i);
            if(val==255)
                bright++;
            else
                dark++;
        }
    }

    if((double)bright/(bright+dark)>0.95)
    {
        angle=0;
        smallangle=0;
        cout<<"angle="<<angle<<endl;
        cout<<"possible="<<270+angle<<";"<<180+angle<<";"<<90+angle<<";"<<angle<<endl;
        findangles.push_back(270+angle);
        findangles.push_back(180+angle);
        findangles.push_back(90+angle);
        findangles.push_back(angle);
        leftside.x=minrect.x;
        leftside.y=minrect.y;
        upside.x=minrect.x+minrect.width-1;
        upside.y=minrect.y;
        rightside.x=minrect.x+minrect.width-1;
        rightside.y=minrect.y+minrect.height-1;
        downside.x=minrect.x;
        downside.y=minrect.y+minrect.height-1;
        cout<<"x="<<(rightside.x+leftside.x)/2<<endl;
        cout<<"y="<<(upside.y+downside.y)/2<<endl;  
        goto getangle;
    }

//-------------------------------------------------------------------------------------
    LED_edge=ROI.col(0);
    minMaxIdx(LED_edge, &minVal, &maxVal, minIdx, maxIdx);//Ѱ����������ڵ��������Сֵ
    leftside.x=minrect.x;
    leftside.y=minrect.y+maxIdx[0];

    LED_edge=ROI.col(ROI.cols-1);
    minMaxIdx(LED_edge, &minVal, &maxVal, minIdx, maxIdx);//Ѱ����������ڵ��������Сֵ
    rightside.x=minrect.x+minrect.width-1;
    rightside.y=minrect.y+maxIdx[0];

    LED_edge=ROI.row(0);
    minMaxIdx(LED_edge, &minVal, &maxVal, minIdx, maxIdx);//Ѱ����������ڵ��������Сֵ
    upside.x=minrect.x+maxIdx[1];
    upside.y=minrect.y;    

    LED_edge=ROI.row(ROI.rows-1);
    minMaxIdx(LED_edge, &minVal, &maxVal, minIdx, maxIdx);//Ѱ����������ڵ��������Сֵ
    downside.x=minrect.x+maxIdx[1];
    downside.y=minrect.y+minrect.height-1;


    // rightup.x=(upside.x+rightside.x)/2;
    // rightup.y=(upside.y+rightside.y)/2;
    // leftdown.x=(leftside.x+downside.x)/2;
    // leftdown.y=(leftside.y+downside.y)/2;
    // angle=Computeangle(rightup,leftdown);
    // smallangle=angle;
    // findangles.push_back(angle);
    // findangles.push_back(270+angle);
    // findangles.push_back(180+angle);
    // findangles.push_back(90+angle);
//------------------------------------------------------------------------------------------

    angle=Computeangle(rightside,downside);
    smallangle=angle;
    findangles.push_back(360+angle);
    findangles.push_back(270+angle);
    findangles.push_back(180+angle);
    findangles.push_back(90+angle);
    // cout<<"angle1="<<angle<<endl;
    // cout<<"possible="<<270+angle<<";"<<180+angle<<";"<<90+angle<<";"<<angle<<endl;

    angle=Computeangle(leftside,downside);
    findangles.push_back(360+angle);
    findangles.push_back(270+angle);
    findangles.push_back(180+angle);
    findangles.push_back(90+angle);
    // cout<<"angle2="<<angle<<endl;
    // cout<<"possible="<<360+angle<<";"<<270+angle<<";"<<180+angle<<";"<<90+angle<<endl;

    angle=Computeangle(upside,leftside);
    findangles.push_back(360+angle);
    findangles.push_back(270+angle);
    findangles.push_back(180+angle);
    findangles.push_back(90+angle);
    // cout<<"angle3="<<angle<<endl;
    // cout<<"possible="<<270+angle<<";"<<180+angle<<";"<<90+angle<<";"<<angle<<endl;

    angle=Computeangle(upside,rightside);
    findangles.push_back(360+angle);
    findangles.push_back(270+angle);
    findangles.push_back(180+angle);
    findangles.push_back(90+angle);
    // cout<<"angle4="<<angle<<endl;
    // cout<<"possible="<<360+angle<<";"<<270+angle<<";"<<180+angle<<";"<<90+angle<<endl;

//-----------------------------------------------------------------------------------------------

    getangle:

    // line(frame, leftside, downside, Scalar(0, 0, 255), 10);
    // line(frame, downside, rightside, Scalar(0, 0, 255), 10);
    // line(frame, rightside, upside, Scalar(0, 0, 255), 10);
    // line(frame, upside, leftside, Scalar(0, 0, 255), 10);
    // imwrite(str2,frame);
    // imshow("ROI_show",frame);
    // cvWaitKey(0);

    double diff=100;
    double realangle;
    for(i=0;i<findangles.size();i++)
    {
        if(findangles.at(i)>=360)
        {
            findangles.at(i)=findangles.at(i)-360;
        }
        if(abs(testangle-findangles.at(i))<diff||360-abs(testangle-findangles.at(i))<diff)
        {
            realangle=findangles.at(i);
            diff=abs(testangle-findangles.at(i));
        }
    }
    return realangle;
}

//------------�����ӵ�------------
double diff(double x,double y,double z)
{
    double d=min(abs(x-y),abs(y-z));
    d=max(d,abs(z-x));
    return d;
}

double Computeangle(Point a,Point b)
{
    Point c;
    c.x=b.x+500;
    c.y=b.y;
    double angle;
    double k1;
    double k2;
    if(diff(a.x,b.x,c.x)<0.001||diff(a.y,b.y,c.y)<0.001)
    {
        return 180.0;
    }
    if(a.x==b.x)
    {
        a.x+=0.01;
    }
    if(c.x==b.x)
    {
        c.x+=0.01;
    }
    Point testpoint;
    k1=(double)(a.y-b.y)/(a.x-b.x);
    k2=(double)(c.y-b.y)/(c.x-b.x);
    angle=atan((k2-k1)/(1+k1*k2))*180.0/PI;
    return angle;
}

};
//-----------------------------------��Ϣͬ���Ļص�����-------------------------------  
//      ������  ����ͬ��ʱ�����ͼƬ�ͽǶ���Ϣ
//-----------------------------------------------------------------------------------------------  

// void callback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::Imu::ConstPtr& imu_msg, const sensor_msgs::MagneticField::ConstPtr& mag_msg)
// {
// 	ROS_INFO("I am coming in message_filters_callback!");
// 	IMAGE_LISTENER_and_LOCATOR obj(msg, imu_msg, mag_msg);
// }

//-----------------------------------main( )����--------------------------------------------  
//      ��������������������λ��Ϣ��ͼ����Ϣ
//      rosrun single_led single_LEDpositioning
//	  ����˼·���������ȴ���������͵شżƽڵ��topic��ͼƬ���Ƕȣ���
//						  ����ص������������ĵ������ݴ�����ʵ�����Ķ���
//						  ���нǶ����ݵ���ȡ��ͼƬ���͵�ת����imageprocess������
//						  ������õ����ݴ��붨λ������㺯����Get_coordinate������
//						  ����λ�����������/location_info��topic����ӡ���ն�
//-----------------------------------------------------------------------------------------------  

int main(int argc, char** argv)
{
	ros::init(argc, argv, "single_LEDpositioning_odom");//��ʼ���ڵ������

	
	IMAGE_LISTENER_and_LOCATOR obj;
	
	//�첽���߳�
	ros::AsyncSpinner s(2);
	s.start();
	
	ros::Rate r(5);
	while (ros::ok())
	{
		ROS_INFO_STREAM("Main thread [" << boost::this_thread::get_id() << "].");
		// obj.image_process(global_image, global_yaw);
		r.sleep();
		// obj.image_process(global_image, global_yaw);
	}

	//ͬ�����߳�
	// ros::MultiThreadedSpinner s(2);
	// ros::spin(s);
	return 0;
}