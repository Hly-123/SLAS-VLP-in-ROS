
//-----------------------------------【头文件包含部分】------------------------------
//      描述：包含程序所依赖的头文件   
//---------------------------------------------------------------------------------------------- 
#include "ros/ros.h" // ROS默认头文件
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
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

//-----------------------------------【命名空间声明分】------------------------------ 
//      描述：包含程序所使用的命名空间  
//---------------------------------------------------------------------------------------------- 

using namespace cv;
using namespace std;


//全局的消息
nav_msgs::Odometry ekf_msg;
ros::Duration d(0.01);
ros::Subscriber ekf_sub;
ros::Publisher ekf_result_pub;

// 这是一个消息后台函数，
// 此函数在收到一个下面设置的名为"/odometry/filtered"的话题时候被调用。
void msgCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    ekf_msg=*msg;
    d.sleep();
}



int main(int argc, char **argv)
{

    ros::init(argc, argv, "single_LEDpositioning1");//初始化节点的名称
    ros::NodeHandle nh; // 声明用于ROS系统和通信的节点句柄


    while(ros::ok())
    {
        // 声明订阅者，创建一个订阅者ekf_sub
        // 话题名称是"/odometry/filtered"，订阅者队列（queue)的大小设为1。
        ekf_sub = nh.subscribe("/odometry/filtered", 1, msgCallback);

        //获得了ekf_msg后，再把它发布出去
        //声明并定义发布者
        ekf_result_pub=nh.advertise<geometry_msgs::PointStamped>("/slovlp_ekf_info1", 1000);

        ROS_INFO("EKF subscribe!~~");
        ros::Rate loop_rate(1);
        int count = 0;

        std_msgs::String msg;
        geometry_msgs::PointStamped msgPointStamped;
        std::stringstream sss;

        msgPointStamped.point.x=ekf_msg.pose.pose.position.x;
        msgPointStamped.point.y=ekf_msg.pose.pose.position.y;
        msgPointStamped.point.z=ekf_msg.pose.pose.position.z;

        sss << '\n' << msgPointStamped.point.x*100
			<< '\n' << msgPointStamped.point.y*100
			<< '\n' << msgPointStamped.point.z*100 << count;

        msg.data = sss.str();
        msgPointStamped.header.stamp = ros::Time::now();
        msgPointStamped.header.frame_id = "map";

        ekf_result_pub.publish(msgPointStamped);//把ekf的定位结果发布出去

        ROS_INFO("%s", msg.data.c_str());
        ++count;
    }
 
    ros::spin();
    return 0;

}


