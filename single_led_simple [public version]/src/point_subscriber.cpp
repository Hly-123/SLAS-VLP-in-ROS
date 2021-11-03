#include<ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <iostream>
// #include <boost/bind.hpp> 
// #include <stdio.h>

using namespace std;

void pointCallback(const geometry_msgs::PointStamped::ConstPtr &msgPointStamped, double& t_past, double& t_now, int& flag_past)
{
	geometry_msgs::PointStamped msgPointStamped1 = *msgPointStamped;
	ROS_INFO("(%.5f, %.5f, %.5f)", msgPointStamped1.point.x*100, msgPointStamped1.point.y*100, msgPointStamped1.point.z*100);
	cout << "coordinate_out_time:\t" << msgPointStamped1.header.stamp << "secs" << endl;
	// cout << "coordinate_out_time:\t" << msgPointStamped1.header.stamp.toSec() << "secs" << endl;

	double t = 0;
	if(flag_past == 1)
	{
		t_past = msgPointStamped1.header.stamp.toSec();
		flag_past = 0;
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
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "point_subscriber");
	ros::NodeHandle n;

	double t_past = 0;
	double t_now = 0;

	int flag_past = 1;
	// int flag_now = 0;


	ros::Subscriber point_info_sub = n.subscribe<geometry_msgs::PointStamped>("/location_info", 1000, boost::bind(pointCallback,_1, t_past, t_now, flag_past));
	ros::spin();
	return 0;
}

