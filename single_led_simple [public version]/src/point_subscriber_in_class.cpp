#include<ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <iostream>
#include <fstream>
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
double t;
//写入文件流(写入时间的)
ofstream fout_time;

//写入文件流(写入坐标的)--用统一的文件流就好
// ofstream fout_point;


public:
point_time()
{
	t_past = 0;
	t_now = 0;
	flag_past = 1;
	t = 0;
	//总共要做这几组数据(同步方式包括三种:多线程,松同步,手动同步)
	//-----------每次解注释一个-----------
	//多线程(四个)
	// fout_time.open("time_imu_multi_2.txt");	//代码ok

	// fout_time.open("time_imu_mag_multi_1.txt");	//代码ok

	// fout_time.open("time_imu_mag_filter_multi_2.txt");	//代码ok

	fout_time.open("time_odom_multi_xyz_location_vlp11_8.txt");	//代码ok

	//松同步(三个)
	// fout_time.open("time_imu_message2.txt");	

	// fout_time.open("time_imu_mag_message.txt");

	// fout_time.open("time_odom_message.txt");

	fout_time << "x" << "\t" << "y" << "\t"<< "z" << "\t"<< "time" << endl;

	point_info_sub = n.subscribe<geometry_msgs::PointStamped>("/location_info", 1000, &point_time::pointCallback, this);
}

void pointCallback(const geometry_msgs::PointStamped::ConstPtr &msgPointStamped)
{
	geometry_msgs::PointStamped msgPointStamped1 = *msgPointStamped;
	ROS_INFO("(%.5f, %.5f, %.5f)", msgPointStamped1.point.x*100, msgPointStamped1.point.y*100, msgPointStamped1.point.z*100);
	cout << "coordinate_out_time:\t" << msgPointStamped1.header.stamp << "secs" << endl;
	// cout << "coordinate_out_time:\t" << msgPointStamped1.header.stamp.toSec() << "secs" << endl;

	fout_time << msgPointStamped1.point.x*100 <<"\t"<< msgPointStamped1.point.y*100 <<"\t"<< msgPointStamped1.point.z*100 << "\t";

	// double t = 0;
	// double t = 5;
	if(flag_past == 1)
	{
		t_past = msgPointStamped1.header.stamp.toSec();
		flag_past = 0;
		// fout_time << t << endl;
	}
	else
	{
		t_now = msgPointStamped1.header.stamp.toSec();
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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "point_subscriber_in_class");
	// ros::NodeHandle n;

	point_time recordT;

	// ros::Subscriber point_info_sub = n.subscribe<geometry_msgs::PointStamped>("/location_info", 1000, boost::bind(pointCallback,_1, t_past, t_now, flag_past));
	ros::spin();
	return 0;
}

