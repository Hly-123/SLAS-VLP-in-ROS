
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


//�ó��������װ�ڱ��˵�����slo_vlp_msg����Ϊsingle_led
// #include "single_led/Rect_image.h"
#include "slo_vlp_msg/Rect_image.h"


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
	ros::Subscriber imu_sub;//imu�Ķ�����
	ros::Subscriber roi_sub;//ROI�Ķ�����
	ros::Subscriber mag_sub;//mag�Ķ�����

	//ȫ�ֵ���Ԫ��
	sensor_msgs::Imu imu_msg_q;

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

	cv_bridge::CvImagePtr cv_ptr;

	int tarwidth, tarheight;
	Rect selected1;
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
		fout_angle.open("imuMag_angle.txt");
		id_result.open("imuMag_id_result.txt");

		fout_angle << "angle" << endl;
		firstframe=true;

		//�����߽���
		// image_sub_ = it_.subscribe("/mvcam/image", 1, &IMAGE_LISTENER_and_LOCATOR::convert_callback, this);
		roi_sub = nh_.subscribe("/mvcam1/image", 1, &IMAGE_LISTENER_and_LOCATOR::convert_callback, this);
		imu_sub = nh_.subscribe("imu", 1, &IMAGE_LISTENER_and_LOCATOR::imu_callback, this);
		mag_sub = nh_.subscribe("magnetic_field", 1, &IMAGE_LISTENER_and_LOCATOR::mag_callback, this);


		//���巢���Ļ���
		msgPointPub = nh_.advertise<geometry_msgs::PointStamped>("/imu_mag_location_info", 1000); //�����λ������
		image_pub_ = it_.advertise("/location/image_show", 1); //����ROSͼ�󷢲���
		//ekf������ȥ�Ļ���
		//����������"/slovlp_ekf_info"
		//��Ϣ�ļ������߶��У�queue���Ĵ�С����Ϊ1000
		//��Ӧ�����ݸ�ʽ��ҪΪ��geometry_msgs::PoseWithCovarianceStamped����Ӧ����Ϣ���ݣ�
		// ������Ҫ��PoseWithCovarianceStampedl��Ϣ�ļ���ʽ����һ����Ϣ
		ekfmsgPointPub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/imu_mag_vlp_ekf_info", 1000); //�������ekf���
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
		image_process(cv_ptr ->image, yaw);
		d.sleep();
		//�õ���cv::Mat���͵�ͼ����CvImageָ���image�У���������͸�������
		//image_process(cv_ptr->image);

		//cv::flip(cv_ptr->image, image_show, 1);	//��ת��1���ң�0���£�-1��������ͬʱ
		// sensor_msgs::ImagePtr msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_show).toImageMsg();
		// image_pub_.publish(msg_image);//�������յ���ԭͼ	

	}

	void mag_callback(const sensor_msgs::MagneticField::ConstPtr& mag_msg)
	{
		sensor_msgs::MagneticField mag_msg1 = *mag_msg;
		// ROS_INFO("I am coming CLASS in magnetic_field_callback!");
		mag_x = mag_msg1.magnetic_field.x;
		mag_y = mag_msg1.magnetic_field.y;
		mag_z = mag_msg1.magnetic_field.z;
		// yaw = atan2(mag_y, mag_x);
		// global_yaw = yaw;
		// ROS_INFO("YAW_from_mag:%.5f", yaw);
		d.sleep();
	} 

	void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg)
	{
		sensor_msgs::Imu imu_msg1 = *imu_msg;
		imu_msg_q=*imu_msg;
		// ROS_INFO("I am coming CLASS in odom_callback!");

		q0 = imu_msg1.orientation.w;
		q1 = imu_msg1.orientation.x;
		q2 = imu_msg1.orientation.y;
		q3 = imu_msg1.orientation.z;

		update(imu_msg1.angular_velocity.x, imu_msg1.angular_velocity.y, imu_msg1.angular_velocity.z, imu_msg1.linear_acceleration.x, imu_msg1.linear_acceleration.y, imu_msg1.linear_acceleration.z, mag_x, mag_y, mag_z);

		imu_msg_q.orientation.w = q0;
		imu_msg_q.orientation.x = q1;
		imu_msg_q.orientation.y = q2;
		imu_msg_q.orientation.z = q3;

		tf::quaternionMsgToTF(imu_msg_q.orientation, quat);

		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
		yaw = yaw * 57.29578f;	//(-180,180)����ϵ
		// ROS_INFO("yaw_from_odom:%.5f", yaw);

		// fout_mag << yaw << "\n";
		
		d.sleep();
	}

	//----------------------------------ͼ������------------------------------------------
	//      ������ͼ�������
	//-----------------------------------------------------------------------------------------------

	void image_process(cv::Mat img, double angle)
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

		msgPointStamped.point = Get_coordinate(img, angle);
		// sss << '\n' << msgPointStamped.point.x*100
		// 	<< '\n' << msgPointStamped.point.y*100
		// 	<< '\n' << msgPointStamped.point.z*100 << count;
		// msg.data = sss.str();
		msgPointStamped.header.stamp = ros::Time::now();
		msgPointStamped.header.frame_id = "map";
		// msgPointPub.publish(msgPointStamped);//�������Ķ�λ���������ȥ

		//��������ӵ�msgPWCStamped��
		msgPWCStamped.pose.pose.orientation = imu_msg_q.orientation;//����odom����Ԫ��
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
		ROS_INFO("angle:%.5f", angle);
		// cv::flip(img, image_show, 1);
		//sensor_msgs::ImagePtr msg_image = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_show).toImageMsg();//�Ҷ�ͼ����
		// sensor_msgs::ImagePtr msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_show).toImageMsg();
		// image_pub_.publish(msg_image);
		++count;

	}


	geometry_msgs::Point Get_coordinate(cv::Mat srcImage, float angle)
	{
		// the key code temporarily is hid
		return point;
	}

//������AHRS�㷨����Mag��IMU��������
	void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
		float recipNorm;
		float s0, s1, s2, s3;
		float qDot1, qDot2, qDot3, qDot4;
		float hx, hy;
		float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

		float aRes = 8.0/32768.0;      // 8g
  		float gRes = 2000.0/32768.0;   // 2000dps
		//mRes = 10.*4912./8190.;  // 14BIT
  		float mRes = 10.*4912./32760.; // 16BIT
		float beta = 0.1f;
		float invSampleFreq = 1.0f / 512.0f;

		gx = gx * gRes;
		gy = gy * gRes;
		gz = gz * gRes;
		ax = ax *aRes;
		ay = ay *aRes;
		az = az *aRes;
		mx = mx * mRes / 15e-8;
		my = my * mRes / 15e-8;
		mz = mz * mRes / 15e-8;

		// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
		if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
			updateIMU(gx, gy, gz, ax, ay, az);
			return;
		}

		// Convert gyroscope degrees/sec to radians/sec
		gx *= 0.0174533f;
		gy *= 0.0174533f;
		gz *= 0.0174533f;

		// Rate of change of quaternion from gyroscope
		qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
		qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
		qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
		qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

		// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

			// Normalise accelerometer measurement
			recipNorm = invSqrt(ax * ax + ay * ay + az * az);
			ax *= recipNorm;
			ay *= recipNorm;
			az *= recipNorm;

			// Normalise magnetometer measurement
			recipNorm = invSqrt(mx * mx + my * my + mz * mz);
			mx *= recipNorm;
			my *= recipNorm;
			mz *= recipNorm;

			// Auxiliary variables to avoid repeated arithmetic
			_2q0mx = 2.0f * q0 * mx;
			_2q0my = 2.0f * q0 * my;
			_2q0mz = 2.0f * q0 * mz;
			_2q1mx = 2.0f * q1 * mx;
			_2q0 = 2.0f * q0;
			_2q1 = 2.0f * q1;
			_2q2 = 2.0f * q2;
			_2q3 = 2.0f * q3;
			_2q0q2 = 2.0f * q0 * q2;
			_2q2q3 = 2.0f * q2 * q3;
			q0q0 = q0 * q0;
			q0q1 = q0 * q1;
			q0q2 = q0 * q2;
			q0q3 = q0 * q3;
			q1q1 = q1 * q1;
			q1q2 = q1 * q2;
			q1q3 = q1 * q3;
			q2q2 = q2 * q2;
			q2q3 = q2 * q3;
			q3q3 = q3 * q3;

			// Reference direction of Earth's magnetic field
			hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
			hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
			_2bx = sqrtf(hx * hx + hy * hy);
			_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
			_4bx = 2.0f * _2bx;
			_4bz = 2.0f * _2bz;

			// Gradient decent algorithm corrective step
			s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
			s0 *= recipNorm;
			s1 *= recipNorm;
			s2 *= recipNorm;
			s3 *= recipNorm;

			// Apply feedback step
			qDot1 -= beta * s0;
			qDot2 -= beta * s1;
			qDot3 -= beta * s2;
			qDot4 -= beta * s3;
		}

		// Integrate rate of change of quaternion to yield quaternion
		q0 += qDot1 * invSampleFreq;
		q1 += qDot2 * invSampleFreq;
		q2 += qDot3 * invSampleFreq;
		q3 += qDot4 * invSampleFreq;

		// Normalise quaternion
		recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
		q0 *= recipNorm;
		q1 *= recipNorm;
		q2 *= recipNorm;
		q3 *= recipNorm;
		char anglesComputed = 0;
	}

	float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
	}
	
	void updateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
	float invSampleFreq = 1.0f / 512.0f;
	float beta = 0.1f;

	// Convert gyroscope degrees/sec to radians/sec
	gx *= 0.0174533f;
	gy *= 0.0174533f;
	gz *= 0.0174533f;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * invSampleFreq;
	q1 += qDot2 * invSampleFreq;
	q2 += qDot3 * invSampleFreq;
	q3 += qDot4 * invSampleFreq;

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	char anglesComputed = 0;
}


//������AHRS�㷨����Mag��IMU��������

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
	ros::init(argc, argv, "single_LEDpositioning_imu_mag");//��ʼ���ڵ������

	
	IMAGE_LISTENER_and_LOCATOR obj;
	
	//�첽���߳�
	ros::AsyncSpinner s(3);
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