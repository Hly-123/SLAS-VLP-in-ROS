#include<ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <iostream>
#include <fstream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
// #include <boost/bind.hpp> 
// #include <stdio.h>

using namespace std;

class point_time
{
private:
ros::NodeHandle n;
double t_past;
double t_now;
int flag_past;
ros::Subscriber point_info_sub;
ros::Publisher point_info_pub;
double t;
geometry_msgs::PointStamped ekf_point;
geometry_msgs::PointStamped ekf_point_temp;
//写入文件流(写入时间的)
ofstream fout_time;

//写入文件流(写入坐标的)--用统一的文件流就好
// ofstream fout_point;


public:
point_time(geometry_msgs::PointStamped &vlc_global)
{
	t_past = 0;
	t_now = 0;
	flag_past = 1;
	t = 0;
	//总共要做这几组数据(同步方式包括三种:多线程,松同步,手动同步)
	//-----------每次解注释一个-----------
	//多线程(四个)
	// fout_time.open("time_imu_multi_1.txt");	//代码ok

	// fout_time.open("time_imu_mag_multi_1.txt");	//代码ok

	// fout_time.open("time_imu_mag_filter_multi_2.txt");	//代码ok

	fout_time.open("time_location_odom_xyz11_8.txt");	//代码ok

	//松同步(三个)
	// fout_time.open("time_imu_message.txt");	

	// fout_time.open("time_imu_mag_message.txt");

	// fout_time.open("time_odom_message.txt");

	fout_time << "x" << "\t" << "y" << "\t"<< "z" << "\t"<< "time" << endl;
	
	ekf_point_temp.point.x = vlc_global.point.x;
	ekf_point_temp.point.y = vlc_global.point.y;
	ekf_point_temp.point.z = vlc_global.point.z;
	ROS_INFO("ekf_point_temp_x=%.5f, y=%.5f, z=%.5f", ekf_point_temp.point.x*100, ekf_point_temp.point.y*100, ekf_point_temp.point.z*100);

	point_info_sub = n.subscribe<nav_msgs::Odometry>("/odom", 1000, &point_time::pointCallback, this);
	point_info_pub=n.advertise<geometry_msgs::PointStamped>("/location_odom_info", 1000);
}

void pointCallback(const nav_msgs::Odometry::ConstPtr &msgPointStamped)
{
	nav_msgs::Odometry msgPoseStamped = *msgPointStamped;
	ekf_point.point.x = msgPoseStamped.pose.pose.position.x + ekf_point_temp.point.x;
	ekf_point.point.y = msgPoseStamped.pose.pose.position.y + ekf_point_temp.point.y;
	ekf_point.point.z = msgPoseStamped.pose.pose.position.z + ekf_point_temp.point.z;
	ekf_point.header.frame_id ="map";
	ekf_point.header.stamp = ros::Time::now();
	ROS_INFO("(%.5f, %.5f, %.5f)", ekf_point.point.x*100, ekf_point.point.y*100, ekf_point.point.z*100);
	cout << "coordinate_out_time:\t" << msgPoseStamped.header.stamp << "secs" << endl;
	// cout << "coordinate_out_time:\t" << msgPointStamped1.header.stamp.toSec() << "secs" << endl;

	fout_time << msgPoseStamped.pose.pose.position.x*100 <<"\t"<< msgPoseStamped.pose.pose.position.y*100 <<"\t"<< msgPoseStamped.pose.pose.position.z*100 << "\t";
	point_info_pub.publish(ekf_point);

	// double t = 0;
	// double t = 5;
	if(flag_past == 1)
	{
		t_past = msgPoseStamped.header.stamp.toSec();
		flag_past = 0;
		// fout_time << t << endl;
	}
	else
	{
		t_now = msgPoseStamped.header.stamp.toSec();
		flag_past = 1;
	}

	if (t_now != 0)
	{
		t = t_now - t_past;
		if (t_now > t_past)
		{
			t = t;
		}
		else
		{
			t = -t;
		}
		cout << "t_now - t_past = " << t << endl;
		fout_time << t << endl;
	}
	else 
	{
		t=0;
		cout << "t_now - t_past = " << t << endl;
		fout_time << t << endl;
	}
}

~point_time()
{
	fout_time.close();
	cout << "close file success!" << endl;
}

};

void vlcCallback(const geometry_msgs::PointStamped::ConstPtr &vlcPointStamped, geometry_msgs::PointStamped &vlc_global)
{
	vlc_global = *vlcPointStamped;
	ROS_INFO("vlc_init_x=%.5f, y=%.5f, z=%.5f", vlc_global.point.x*100, vlc_global.point.y*100, vlc_global.point.z*100);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "odom_translater");
	ros::NodeHandle nh;
	geometry_msgs::PointStamped point_vlc;

	//方法一(这个不行，还是会持续订阅)
	// ros::Subscriber odom_sub=nh.subscribe<geometry_msgs::PointStamped>("/location_info",100, boost::bind(vlcCallback,_1,point_vlc));
	
	// ros::Rate loop_rate(10);

	// if(ros::ok())
	// {
	// 	ros::spinOnce();
	// 	loop_rate.sleep();
	// 	ROS_INFO("point_vlc_x=%.5f, y=%.5f, z=%.5f", point_vlc.point.x*100, point_vlc.point.y*100, point_vlc.point.z*100);
	// }
	
	//方法二（这个可以，只订阅一次）
	boost::shared_ptr<geometry_msgs::PointStamped const> point_vlc_ptr;
	point_vlc_ptr = ros::topic::waitForMessage<geometry_msgs::PointStamped>("/location_info", ros::Duration(5));
	if(point_vlc_ptr != NULL)
	{
	point_vlc = *point_vlc_ptr;
	ROS_INFO("point_vlc_x=%.5f, y=%.5f, z=%.5f", point_vlc.point.x*100, point_vlc.point.y*100, point_vlc.point.z*100);
	}
	else
	{
	ROS_INFO("no topic location/info!!");
	}

	//进行odom的接收
	// if(point_vlc.point.x*100!=0 || point_vlc.point.y*100!=0 || point_vlc.point.z*100!=0)
	point_time recordT(point_vlc);
	

	// ros::Subscriber point_info_sub = n.subscribe<geometry_msgs::PointStamped>("/location_info", 1000, boost::bind(pointCallback,_1, t_past, t_now, flag_past));
	ros::spin();
	return 0;
}

