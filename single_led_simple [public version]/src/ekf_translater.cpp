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
	//����������˽�г�Ա
	private:
		ros::NodeHandle n;//// ��������ROSϵͳ��ͨ�ŵĽڵ���


		double t;
		double t_past;
		double t_now;
		int flag_past;

		//���嶩����
		ros::Subscriber point_info_sub;

		//���巢�����붩����
		ros::Publisher point_info_pub;
		geometry_msgs::PointStamped ekf_point;

		//д���ļ���(д��ʱ���)
		ofstream fout_time;

		//д���ļ���(д�������)--��ͳһ���ļ����ͺ�
		// ofstream fout_point;


	//���г�Ա
	public:
		//�����乹�캯��
		point_time()
		{
			t_past = 0;
			t_now = 0;
			flag_past = 1;
			t = 0;
			

			fout_time.open("time_location_ekf_xyz11_8.txt");	//��һ���������ļ�

			fout_time << "x" << "\t" << "y" << "\t"<< "z" << "\t"<< "time" << endl;

			//��������/odometry/filtered�Ļ���
			//��Ϣ����Ϊnav_msgs::Odometry
			//ͨ������pointCallback������
			point_info_sub = n.subscribe<nav_msgs::Odometry>("/odometry/filtered", 1000, &point_time::pointCallback, this);
			
			//����һ����Ϊ/location_ekf_info�Ļ���
			//��Ϣ����Ϊgeometry_msgs::PointStamped
			point_info_pub=n.advertise<geometry_msgs::PointStamped>("/location_ekf_info", 1000);
		}

		void pointCallback(const nav_msgs::Odometry::ConstPtr &msgPointStamped)
		{
			nav_msgs::Odometry msgPoseStamped = *msgPointStamped;
			//���յ���Ϣ�󣬸��������ߣ���������˽�г�Ա��
			ekf_point.point.x = msgPoseStamped.pose.pose.position.x;
			ekf_point.point.y = msgPoseStamped.pose.pose.position.y;
			ekf_point.point.z = msgPoseStamped.pose.pose.position.z;

			//�������ߵļ���frameid��ʱ���
			ekf_point.header.frame_id ="map";
			ekf_point.header.stamp = ros::Time::now();
			//���ն���ʾ��λ�����ʱ��
			ROS_INFO("(%.5f, %.5f, %.5f)", msgPoseStamped.pose.pose.position.x*100, msgPoseStamped.pose.pose.position.y*100, msgPoseStamped.pose.pose.position.z*100);
			cout << "coordinate_out_time:\t" << msgPoseStamped.header.stamp << "secs" << endl;
			
			//���浽�ļ��С�
			fout_time << msgPoseStamped.pose.pose.position.x*100 <<"\t"<< msgPoseStamped.pose.pose.position.y*100 <<"\t"<< msgPoseStamped.pose.pose.position.z*100 << "\t";
			point_info_pub.publish(ekf_point);//������ȥ

			//��ʱ����������
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

		//��������
		~point_time()
		{
			fout_time.close();
			cout << "close file success!" << endl;
		}

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ekf_translater");//��ʼ����ǰ�ڵ�����ƣ��������Ӧ����Ҫ��cmake.list���õ�һ����
	// ros::NodeHandle n;

	point_time recordT;//������һ����

	// ros::Subscriber point_info_sub = n.subscribe<geometry_msgs::PointStamped>("/location_info", 1000, boost::bind(pointCallback,_1, t_past, t_now, flag_past));
	ros::spin();
	return 0;
}

