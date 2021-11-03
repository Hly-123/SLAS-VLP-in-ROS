
//-----------------------------------【头文件包含部分】------------------------------
//      描述：包含程序所依赖的头文件   
//---------------------------------------------------------------------------------------------- 
#include<single_led/vlcCommonInclude.hpp>
#include <single_led/imgProcess.hpp>
#include<sensor_msgs/Imu.h>
#include<sensor_msgs/MagneticField.h>
#include <stdio.h>
#include <string>
#include <math.h>
#include <iostream>
#include <fstream>
#include <nav_msgs/Odometry.h>
#include "tf/transform_datatypes.h"  
//#include <single_led/sensor_angle.h> //自定义话题消息（其实没必要，std_msgs::float64也没必要，直接double）
void para_set(double& X, double& Y, double& D_LED, double& f, double& led_x, double& led_y, double& all_H);

//-----------------------------------【定义新类型MySyncPolicy】------------------------------ 
typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Imu, sensor_msgs::MagneticField> MySyncPolicy;
// const double PI=acos(-1.0);
//-----------------------------------【命名空间声明分】------------------------------ 
//      描述：包含程序所使用的命名空间  
//----------------------------------------------------------------------------------------------  
using namespace cv;
using namespace std;

/*
定义：一个转换（sensor_msgs::Iamge-->cv::Mat）的类：实现图像转换的同时进行图像定位处理
输入：同时间戳的图片sensor_msgs::Image和地磁计角度single_led::sensor_angle
*/
class IMAGE_LISTENER_and_LOCATOR
{
private:
	ros::NodeHandle nh_; //定义ROS句柄  
	image_transport::ImageTransport it_; //定义一个image_transport实例  
	image_transport::Subscriber image_sub_; //定义ROS图象接收器  
	ros::Subscriber point_info_sub;
	// image_transport::Publisher image_pub_;
	ros::Publisher msgPointPub;
	//同步
	// message_filters::Subscriber<sensor_msgs::Image>* image_sub;
	message_filters::Subscriber<nav_msgs::Odometry>* odom_sub;
	message_filters::Subscriber<sensor_msgs::Imu>* imu_sub;
	message_filters::Subscriber<sensor_msgs::MagneticField>* mag_sub;
	message_filters::Synchronizer<MySyncPolicy>* sync;

	tf::Quaternion quat;
	double roll;
	double pitch;
	double yaw;
	float q0;
	float q1;
	float q2;
	float q3;

	//定位相关的数据设置
	//相机的图像中心坐标(当前是机器人的相机中心对准正下方)
	// double Center_X;
	// double Center_Y;
	// //LED灯具的真实直径
	// double D_LED;
	// //焦距
	// double f;
	// //LED在世界坐标系中的水平位置坐标
	// double led_x;
	// double led_y;
	// //LED距离地面的垂直高度
	// double all_H;

	//写入文件流
	ofstream fout_mag;

public:
	IMAGE_LISTENER_and_LOCATOR()
		:it_(nh_) //构造函数  
	{	
		/*---------------场地1-------------------*/
		// Center_X = 575;//685;//674;//相机中心对准灯正下方670;//640;//429;
		// Center_Y = 499;//495;//447;//486;//480;//334.5;
		// D_LED = 180;//LED灯具的真实直径
		// f = 2.917*312.5;//依据高度矫正的焦距(2.6389)//3.111;//根据图片大小不同,会有不同的等效焦距(1280*960对应2.5)
		// led_x = -800;
		// led_y = 0;
		// all_H = 2.85;

		/*---------------场地2-------------------*/
		// Center_X = 653;//685;//674;//相机中心对准灯正下方670;//640;//429;
		// Center_Y = 498;//495;//447;//486;//480;//334.5;
		// D_LED = 103;//LED灯具的真实直径
		// f = 2.596*312.5;//依据高度矫正的焦距(2.6389)//3.111;//根据图片大小不同,会有不同的等效焦距(1280*960对应2.5)
		// led_x = 0;
		// led_y = 0;
		// all_H = 1.57;

		// image_sub= new message_filters::Subscriber<sensor_msgs::Image>(nh_, "/mvcam/image", 1);            	//topic1 输入
		odom_sub= new message_filters::Subscriber<nav_msgs::Odometry>(nh_, "odom", 1);            	//topic1 输入
		imu_sub=new message_filters::Subscriber<sensor_msgs::Imu>(nh_, "imu", 1);   //topic2 输入
		mag_sub=new message_filters::Subscriber<sensor_msgs::MagneticField>(nh_, "magnetic_field", 1);	//topic3输入
		sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *odom_sub, *imu_sub, *mag_sub);//10
		sync->registerCallback(boost::bind(&IMAGE_LISTENER_and_LOCATOR::callback,this, _1, _2, _3)); 

		// image_pub_ = it_.advertise("/location/image_show", 1); //定义ROS图象发布
		msgPointPub = nh_.advertise<geometry_msgs::PointStamped>("/location_info", 1000); //输出定位坐标结果

		fout_mag.open("angle_comparison.txt");

		// image_pub_ = it_.advertise("/location/image_show", 1); //定义ROS图象发布器
		// msgPointPub = nh_.advertise<geometry_msgs::PointStamped>("/location_info", 1000); //输出定位坐标结果
		// Mat srcImage = imread("/home/hly/catkin_ws/src/single_led/single_LED.bmp");
		// if (srcImage.empty())
		// {
		// 	ROS_INFO("image fail!!");
		// }
		//image_process(srcImage);
	}

	void callback(const nav_msgs::Odometry::ConstPtr& odom_msg, const sensor_msgs::Imu::ConstPtr& imu_msg, const sensor_msgs::MagneticField::ConstPtr& mag_msg)
	{
		nav_msgs::Odometry odom_msg1 = *odom_msg;
		sensor_msgs::Imu imu_msg1 = *imu_msg;
		sensor_msgs::MagneticField mag_msg1 = *mag_msg;

		ROS_INFO("I am coming CLASS in message_filters_callback!");

		q0 = imu_msg1.orientation.w;
		q1 = imu_msg1.orientation.x;
		q2 = imu_msg1.orientation.y;
		q3 = imu_msg1.orientation.z;

		/*对地磁计角度数据的提取*/
		//------------获得角度-------------
		// ROS_INFO("imu.quartation:(%.5f, %.5f, %.5f, %.5f)", imu_msg1.orientation.w, imu_msg1.orientation.x, imu_msg1.orientation.y, imu_msg1.orientation.z);
		// ROS_INFO("mag.xyz:(%.5f, %.5f, %.5f)", mag_msg1.magnetic_field.x, mag_msg1.magnetic_field.y, mag_msg1.magnetic_field.z);

		//imu
		yaw = atan2f(q1*q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3);
		yaw = yaw * 57.29578f;	//(-180,180)右手系
		// fout_mag << mag_msg1.magnetic_field.x << "\t" << mag_msg1.magnetic_field.y << "\t" << mag_msg1.magnetic_field.z << "\n";
		fout_mag << "imu:" << yaw << "\t";
		ROS_INFO("YAW_from_imu:%.5f", yaw);


		//mag	
		yaw = atan2(mag_msg1.magnetic_field.y, mag_msg1.magnetic_field.x);
		//广州的地磁偏角1°09'
		yaw = yaw * 57.29578f - 1;	//(-180,180)右手系
		fout_mag << "mag:" << yaw << "\t";
		ROS_INFO("YAW_from_mag:%.5f", yaw);

		//imu+mag
		update(imu_msg1.angular_velocity.x, imu_msg1.angular_velocity.y, imu_msg1.angular_velocity.z, imu_msg1.linear_acceleration.x, imu_msg1.linear_acceleration.y, imu_msg1.linear_acceleration.z, mag_msg1.magnetic_field.x, mag_msg1.magnetic_field.y, mag_msg1.magnetic_field.z);
		// roll = atan2f(q0*q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2);
		// pitch = asinf(-2.0f * (q1*q3 - q0 * q2));
		yaw = atan2f(q1*q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3);
		yaw = yaw * 57.29578f;	//(-180,180)右手系
		fout_mag << "imu+mag:" << yaw << "\t";
		ROS_INFO("YAW_from_imu+mag:%.5f", yaw);

		//odom
		tf::quaternionMsgToTF(odom_msg1.pose.pose.orientation, quat);
		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
		yaw = yaw * 57.29578f;	//(-180,180)右手系
		fout_mag << "odom:" << yaw << "\n";
		ROS_INFO("YAW_from_odom:%.5f", yaw);

		// roll = roll * 57.29578f;
		// pitch = pitch * 57.29578f;
		//yaw = yaw * 57.29578f + 180.0f;	//(0,360)
		
		// yaw = yaw * 57.29578f * (-1.0f);	//(-180,180)左手系

		// ROS_INFO("YAW:%.5f", yaw);
		/*对图像的转换和处理*/
		cv_bridge::CvImagePtr cv_ptr;  // 声明一个CvImage指针的实例
		// cv::Mat image_show;

		// try 
		// {
		// 	// 将ROS消息中的图象信息提取，生成新cv类型的图象，复制给CvImage指针
		// 	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		// 	// image_pub_.publish(msg);
		// }
		// catch (cv_bridge::Exception& e)
		// {  // 异常处理
		// 	ROS_ERROR("cv_bridge exception: %s", e.what());
		// 	return;
		// }

		//得到了cv::Mat类型的图象和地磁计角度，在CvImage指针的image中，将结果传送给处理函数
		// image_process(cv_ptr->image, yaw);
	}

	~IMAGE_LISTENER_and_LOCATOR() //析构函数
	{
		delete odom_sub;
		delete imu_sub;
		delete mag_sub;
		delete sync;
		// fout_mag << endl;
		// fout_mag.close();
		ROS_INFO("OUT CLASS!!");
	}

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

	//----------------------------【ROS和OpenCV的格式转换回调函数】--------------------------------------
	//      描述：这是一个ROS和OpenCV的格式转换回调函数，将图象格式从sensor_msgs/Image  --->  cv::Mat 
	//--------------------------------------------------------------------------------------------------------------------------

	// void convert_callback(const sensor_msgs::ImageConstPtr& msg) {
	// 	cv_bridge::CvImagePtr cv_ptr;  // 声明一个CvImage指针的实例

	// 	cv::Mat image_show;

	// 	try {
	// 		// 将ROS消息中的图象信息提取，生成新cv类型的图象，复制给CvImage指针
	// 		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	// 		image_pub_.publish(msg);
	// 	}
	// 	catch (cv_bridge::Exception& e) {  // 异常处理
	// 		ROS_ERROR("cv_bridge exception: %s", e.what());
	// 		return;
	// 	}

	// 	//得到了cv::Mat类型的图象，在CvImage指针的image中，将结果传送给处理函数
	// 	//image_process(cv_ptr->image);

	// 	//cv::flip(cv_ptr->image, image_show, 1);	//翻转，1左右；0上下；-1左右上下同时
	// 	sensor_msgs::ImagePtr msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_show).toImageMsg();
	// 	image_pub_.publish(msg_image);//发布接收到的原图	

	// }


	//----------------------------------图像处理函数------------------------------------------
	//      描述：图像处理程序
	//-----------------------------------------------------------------------------------------------

	void image_process(cv::Mat img, float angle)
	{
		ROS_INFO("image process start!~~");
		// cv::Mat img_out;
		ros::Rate loop_rate(1);
		int count = 0;

		std_msgs::String msg;
		geometry_msgs::PointStamped msgPointStamped;
		std::stringstream sss;
		// cv::Mat image_show;

		msgPointStamped.point = Get_coordinate(img, angle);
		sss << '\n' << msgPointStamped.point.x*100
			<< '\n' << msgPointStamped.point.y*100
			<< '\n' << msgPointStamped.point.z*100 << count;
		msg.data = sss.str();
		msgPointStamped.header.stamp = ros::Time::now();
		msgPointStamped.header.frame_id = "map";
		msgPointPub.publish(msgPointStamped);

		ROS_INFO("%s", msg.data.c_str());
		ROS_INFO("angle:%.5f", angle);
		++count;
	}

	geometry_msgs::Point Get_coordinate(cv::Mat srcImage, float angle)
	{
	//进行高斯滤波
	Mat resGauImage;//高斯滤波结果
	GaussianBlur(srcImage, resGauImage, Size(5, 5), 0, 0);
	//imshow("resGauImage", resGauImage);

	//将图像转换为灰度图
	Mat grayImage;//灰度图
	cvtColor(resGauImage, grayImage, COLOR_BGR2GRAY);
	//imshow("grayImage", grayImage);

	//将图像进行二值化
	double m_threshold;//二值化阈值
	Mat matBinary;//二值化图像
	m_threshold = getThreshVal_Otsu_8u(grayImage);//获取自动阈值
	threshold(grayImage, matBinary, m_threshold, 255, 0); // 二值化    
	//imshow("matBinary", matBinary);

	//先膨胀后腐蚀,闭运算
	Mat element = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));//定义结构元素,size不可以太大
	morphologyEx(matBinary, matBinary, MORPH_CLOSE, element);

	int X_min, X_max, Y_min, Y_max;//分割LED
	Mat img_next;//分割LED后存储的图像

	//定位相关的数据设置
	//相机的图像中心坐标(当前是机器人的相机中心对准正下方)
	double Center_X;
	double Center_Y;
	//LED灯具的真实直径
	double D_LED;
	//焦距
	double f;
	//LED在世界坐标系中的水平位置坐标
	double led_x;
	double led_y;
	//LED距离地面的垂直高度
	double all_H;

	para_set(Center_X, Center_Y, D_LED, f, led_x, led_y, all_H);

	//下面方案二选一（均是获取LED在图像中的半径和圆心坐标）

		//------------------------------------------1.对应is_LED函数的定位计算--------------------------------//
		//分割LED
	ls_LED(matBinary, X_min, X_max, Y_min, Y_max, img_next);
	//imshow("img_next", img_next);
	//cout << "X_min=" << X_min << '\n';
	//cout << "X_max=" << X_max << '\n';
	//cout << "Y_min=" << Y_min << '\n';
	//cout << "Y_max=" << Y_max << '\n';

	// imwrite("test.png",matBinary);
	//获得LED像素中心的位置
	double Img_local_X = (X_max + X_min) / 2;
	double Img_local_Y = (Y_max + Y_min) / 2;
	cout << "Img_local_X=" << Img_local_X << '\n';
	cout << "Img_local_Y=" << Img_local_Y << '\n';

	double D = max(abs(X_max - X_min), abs(Y_max - Y_min));//LED图像直径,像素的物理大小3.2e-3
	double H = f * D_LED / D;//计算出高度
	
	//one
	// double X = (Img_local_X - Center_X)*H / f;//相似三角形算出相机在相机坐标系的坐标,像素的物理大小3.2e-3
	// double Y = (Img_local_Y - Center_Y)*H / f;

	//two
	//由于图像是以左上角为原点建立的左手系坐标,而机器人是右手系(Z向上,X向前,Y由右手系确定),因此XY对调
	// double Y = (Img_local_X - Center_X)*H / f;//相似三角形算出相机在相机坐标系的坐标,像素的物理大小3.2e-3
	// double X = (Img_local_Y - Center_Y)*H / f;

	// double Y = (Center_X - Img_local_X)*H / f;//相似三角形算出相机在相机坐标系的坐标,像素的物理大小3.2e-3
	// double X = (Center_Y - Img_local_Y)*H / f;

	double Y = (Img_local_X - Center_X)*H / f;
	double X = (Center_Y - Img_local_Y)*H / f;
/*-----------------调试debug时显示的信息--------------------------*/
	// cout << "D=" << D << '\n';
	// cout << "H=" << H << '\n';

	// namedWindow("src", 0);
	// imshow("src", img_next);
	// waitKey(100);
	
	//------------------------------------------2.对应LED_contours函数的定位计算-----------------------//
	/*
	//获得LED像素中心和图像中的半径
	LED_contour(matBinary, myPoints, myRadius);
	double D = myRadius[0] * 2 * 3.2e-3;
	double D_LED = 103;//LED灯具的真实直径
	double f = 3.111;//焦距
	double H = f * D_LED / D;//计算出高度
	double X = (myPoints[0].x - Center_X)*3.2e-3*H / f;//相似三角形算出相机在相机坐标系的坐标,像素的物理大小3.2e-3
	double Y = (myPoints[0].y - Center_Y)*3.2e-3*H / f;
	*/

	//获取图像的行列
	double rowB = matBinary.rows;//二值化图像的行数
	double colB = matBinary.cols;//二值化图像的列数


	//------------------------------------------地磁传感器输入角度------------------------------------------//
	//计算方向角（用图片测试阶段——赋值将传进来的角度架空处理）
	//angle = 0;
	// 接收到树莓派传回来的角度信息时，或者接收到opencr发来的数据时,注释掉该句
	//yaw偏航角的范围：-180°~180°，相当于atan2(y,x)函数
	//------------------------------------------地磁传感器输入角度------------------------------------------//

	//测试角度的输出是不是有问题,如果还是打点失败就说明是代码问题---(直线实验的调试)
	//angle=0;
	//将坐标系转化为与世界坐标系平行
	angle = angle * PI /180.0;
	double XX = X * cos(angle) - Y * sin(angle);
	double YY = X * sin(angle) + Y * cos(angle);

	//根据LED的实际位置坐标进行原点迁移
	XX = XX + led_x;
	YY = YY + led_y;

	//mm转换成cm
	geometry_msgs::Point point;
	point.x = XX / 1000;
	point.y = YY / 1000;
	//深圳(远程控制)那边实验室的高度
	//point.z = 189 - H / 10;
	point.z = all_H - H / 1000;
	return point;
	}


};

//-----------------------------------消息同步的回调函数-------------------------------  
//      描述：  接收同个时间戳的图片和角度信息
//-----------------------------------------------------------------------------------------------  

// void callback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::Imu::ConstPtr& imu_msg, const sensor_msgs::MagneticField::ConstPtr& mag_msg)
// {
// 	ROS_INFO("I am coming in message_filters_callback!");
// 	IMAGE_LISTENER_and_LOCATOR obj(msg, imu_msg, mag_msg);
// }

//-----------------------------------main( )函数--------------------------------------------  
//      描述：主函数，发布定位信息和图像信息
//      rosrun single_led single_LEDpositioning
//	  程序思路：主函数等待接收相机和地磁计节点的topic（图片，角度）；
//						  进入回调函数，将订阅到的数据传入类实例化的对象
//						  进行角度数据的提取和图片类型的转换（imageprocess函数）
//						  将处理好的数据传入定位坐标计算函数（Get_coordinate函数）
//						  将定位结果发布到“/location_info”topic，打印于终端
//-----------------------------------------------------------------------------------------------  

int main(int argc, char** argv)
{
	ros::init(argc, argv, "single_LEDpositioning");

	IMAGE_LISTENER_and_LOCATOR obj;
	ros::spin();
	return 0;
}