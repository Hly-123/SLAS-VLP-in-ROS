
//-----------------------------------【头文件包含部分】------------------------------
//      描述：包含程序所依赖的头文件   
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

//不知道找不找得到
//该程序包若安装在别人电脑则将slo_vlp_msg都改为single_led
// #include "single_led/Rect_image.h"
#include "slo_vlp_msg/Rect_image.h"
//#include <single_led/sensor_angle.h> //自定义话题消息（其实没必要，std_msgs::float64也没必要，直接double）

//-----------------------------------【定义新类型MySyncPolicy】------------------------------ 
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Imu, sensor_msgs::MagneticField> MySyncPolicy;
// const double PI=acos(-1.0);

ros::Duration d(0.01);

//-----------------------------------【命名空间声明分】------------------------------ 
//      描述：包含程序所使用的命名空间  
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

//ID解码配套函数
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
定义：一个转换（sensor_msgs::Iamge-->cv::Mat）的类：实现图像转换的同时进行图像定位处理
输入：同时间戳的图片sensor_msgs::Image和地磁计角度single_led::sensor_angle
*/
class IMAGE_LISTENER_and_LOCATOR
{
private:
	ros::NodeHandle nh_; //// 声明一个节点句柄来与ROS系统进行通信
	image_transport::ImageTransport it_; //定义一个image_transport实例  
	image_transport::Subscriber image_sub_; //定义ROS图象接收器  
	ros::Subscriber point_info_sub;
	image_transport::Publisher image_pub_;

	//声明发布者，后面定义
	ros::Publisher msgPointPub;
	ros::Publisher ekfmsgPointPub;//定义ekf发布者
	//声明订阅者
	ros::Subscriber imu_sub;//odom的订阅者
	ros::Subscriber roi_sub;//ROI的订阅者

	//全局的四元素
	sensor_msgs::Imu imu_msg_q;

	//写入文件流(写入角度的)
	ofstream fout_angle;

	//接收由ROI传来的标志位,判断是否锁定当前的ID
	int flag_roi;
	//定义LED结构体
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

	//图像中LED的中心坐标
	double Img_local_X;
	double Img_local_Y;

	//图像中LED的直径;
	double D;
	double width_from_mvcam;
	double height_from_mvcam;

	geometry_msgs::Point point;
	Mat IDdata;
	//文件流定义
    ofstream id_result;

	//解决无灯中断问题
	// Mat imgNext;
	// Mat grayimage;

	// //定位相关的数据设置
	// //相机的图像中心坐标(当前是机器人的相机中心对准正下方)
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
	// ofstream fout_mag;

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
		fout_angle.open("imu_angle.txt");
		id_result.open("imu_id_result.txt");

		fout_angle << "angle" << endl;
		firstframe=true;

		//订阅者接受
		// image_sub_ = it_.subscribe("/mvcam/image", 1, &IMAGE_LISTENER_and_LOCATOR::convert_callback, this);
		roi_sub = nh_.subscribe("/mvcam1/image", 1, &IMAGE_LISTENER_and_LOCATOR::convert_callback, this);
		imu_sub = nh_.subscribe("imu", 1, &IMAGE_LISTENER_and_LOCATOR::imu_callback, this);
		
		//定义发布的话题
		msgPointPub = nh_.advertise<geometry_msgs::PointStamped>("/imu_location_info", 1000); //输出定位坐标结果
		image_pub_ = it_.advertise("/location/image_show", 1); //定义ROS图象发布器
		//ekf发布出去的话题
		//话题名称是"/slovlp_ekf_info"
		//消息文件发布者队列（queue）的大小设置为1000
		//对应的数据格式需要为：geometry_msgs::PoseWithCovarianceStamped（对应的消息内容）
		// 下面需要以PoseWithCovarianceStampedl消息文件格式声明一个消息
		ekfmsgPointPub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/imu_vlp_ekf_info", 1000); //输出用于ekf结果
		// 这里的消息的类型应该是由ekf决定的。
		// 在ekf中，我们选用pose作为VLC的输入。但ROS中的pose message有两种：
		//1、geometry_msgs/Pose.msg和2、geometry_msgs/PoseWithCovarianceStamped.msg
		//参考连接：http://docs.ros.org/en/melodic/api/robot_localization/html/preparing_sensor_data.html
		//可知道，应该为geometry_msgs/PoseWithCovarianceStamped
		

		unknown.ID=0;
		// fout_mag.open("odom_angle.txt");
	}

	~IMAGE_LISTENER_and_LOCATOR() //析构函数
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


	//----------------------------【ROS和OpenCV的格式转换回调函数】--------------------------------------
	//      描述：这是一个ROS和OpenCV的格式转换回调函数，将图象格式从sensor_msgs/Image  --->  cv::Mat 
	//--------------------------------------------------------------------------------------------------------------------------

	void convert_callback(const slo_vlp_msg::Rect_image::ConstPtr& rect_msg) {
		// cv_bridge::CvImagePtr cv_ptr;  // 声明一个CvImage指针的实例

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
			// 将ROS消息中的图象信息提取，生成新cv类型的图象，复制给CvImage指针
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);//BGR8
			image_pub_.publish(msg);
		}
		catch (cv_bridge::Exception& e) {  // 异常处理
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		// global_image = cv_ptr ->image;

		// ROS_INFO("nowtime_yaw:%.5f", yaw);
		image_process(cv_ptr ->image, yaw);
		d.sleep();
		//得到了cv::Mat类型的图象，在CvImage指针的image中，将结果传送给处理函数
		//image_process(cv_ptr->image);

		//cv::flip(cv_ptr->image, image_show, 1);	//翻转，1左右；0上下；-1左右上下同时
		// sensor_msgs::ImagePtr msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_show).toImageMsg();
		// image_pub_.publish(msg_image);//发布接收到的原图	

	}

	void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg)
	{
		sensor_msgs::Imu imu_msg1 = *imu_msg;
		imu_msg_q=*imu_msg;
		// ROS_INFO("I am coming CLASS in odom_callback!");

		tf::quaternionMsgToTF(imu_msg1.orientation, quat);

		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
		yaw = yaw * 57.29578f;	//(-180,180)右手系
		// ROS_INFO("yaw_from_odom:%.5f", yaw);

		// fout_mag << yaw << "\n";
		
		d.sleep();
	}

	//----------------------------------图像处理函数------------------------------------------
	//      描述：图像处理程序
	//-----------------------------------------------------------------------------------------------

	void image_process(cv::Mat img, double angle)
	{
		ROS_INFO("image process start!~~");
		// cv::Mat img_out;
		ros::Rate loop_rate(1);
		int count = 0;

		std_msgs::String msg;
		geometry_msgs::PointStamped msgPointStamped;//
		//以PoseWithCovarianceStamped消息文件格式声明一个 叫做msgPWCStamped的消息
		//消息的类型在ekfmsgPointPub话题中确定了
		geometry_msgs::PoseWithCovarianceStamped msgPWCStamped;//发布坐标及四元素
		//而发布的消息应该是需要先经过tf变换到对应的坐标系上的
		//如果是跟IMU融合的化，最基本的就是变换到跟imu一致的坐标系上
		//或者至少变换到geometry_msgs/PoseWithCovarianceStamped上
		//而geometry_msgs/PoseWithCovarianceStamped是Handled in the same fashion as the pose data in the Odometry message
		//故此应该转移到odom上

		std::stringstream sss;
		// cv::Mat image_show;

		msgPointStamped.point = Get_coordinate(img, angle);
		// sss << '\n' << msgPointStamped.point.x*100
		// 	<< '\n' << msgPointStamped.point.y*100
		// 	<< '\n' << msgPointStamped.point.z*100 << count;
		// msg.data = sss.str();
		msgPointStamped.header.stamp = ros::Time::now();
		msgPointStamped.header.frame_id = "map";
		// msgPointPub.publish(msgPointStamped);//把正常的定位结果发布出去

		//把内容添加到msgPWCStamped中
		msgPWCStamped.pose.pose.orientation = imu_msg_q.orientation;//加入odom的四元素
		msgPWCStamped.pose.pose.position =msgPointStamped.point;
	
	// *****************tf变换
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
		//协方差矩阵
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
		ekfmsgPointPub.publish(msgPWCStamped);//发给ekf
		}
		else
		{
		ROS_INFO("Remain the last coordinate, BUT NOT PUBLISHED!!");
		
		}


		msgPointPub.publish(msgPointStamped);
		//ekfmsgPointPub.publish(msgPWCStamped);//发给ekf


		// cout << "coordinate_out_time:\t" << msgPointStamped.header.stamp << "secs" << endl;
		ROS_INFO("%s", msg.data.c_str());
		ROS_INFO("angle:%.5f", angle);
		// cv::flip(img, image_show, 1);
		//sensor_msgs::ImagePtr msg_image = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_show).toImageMsg();//灰度图传入
		// sensor_msgs::ImagePtr msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_show).toImageMsg();
		// image_pub_.publish(msg_image);
		++count;

	}


	geometry_msgs::Point Get_coordinate(cv::Mat srcImage, float angle)
	{
		// the key code temporarily is hid
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
	ros::init(argc, argv, "single_LEDpositioning_imu");//初始化节点的名称

	
	IMAGE_LISTENER_and_LOCATOR obj;
	
	//异步多线程
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

	//同步多线程
	// ros::MultiThreadedSpinner s(2);
	// ros::spin(s);
	return 0;
}