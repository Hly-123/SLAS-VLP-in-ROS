#include<ros/ros.h>
#include<sensor_msgs/Imu.h>
#include<sensor_msgs/MagneticField.h>
#include<math.h>

void imu_processCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
	ROS_INFO("-------------------I am coming into imu_processCallback--------------------");
	sensor_msgs::Imu imu_msg1 = *imu_msg;
	ROS_INFO("imu.quartation:(%.5f, %.5f, %.5f, %.5f)",imu_msg1.orientation.w, imu_msg1.orientation.x, imu_msg1.orientation.y, imu_msg1.orientation.z);
	float q0 = imu_msg1.orientation.w;
	float q1 = imu_msg1.orientation.x;
	float q2 = imu_msg1.orientation.y;
	float q3 = imu_msg1.orientation.z;
	float roll = atan2f(q0*q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2);
	float pitch = asinf(-2.0f * (q1*q3 - q0 * q2));
	float yaw = atan2f(q1*q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3);

	roll = roll * 57.29578f;
	pitch = pitch * 57.29578f;
	//yaw = yaw * 57.29578f + 180.0f;	//(0,360)
	//yaw = yaw * 57.29578f;	//(-180,180)����ϵ
	yaw = yaw * 57.29578f * (-1.0f);	//(-180,180)����ϵ
	ROS_INFO("imu.rpy:(%.5f, %.5f, %.5f)", roll, pitch, yaw);
	//����ͨ������sensor_msgs��float���ݽ��д���
	//����������ڳ�ʼλ��
}

void mag_processCallback(const sensor_msgs::MagneticField::ConstPtr& mag_msg)
{
	ROS_INFO("-------------------I am coming into mag_processCallback--------------------");
	sensor_msgs::MagneticField mag_msg1 = *mag_msg;
	ROS_INFO("mag.xyz��(%.5f, %.5f, %.5f)",mag_msg1.magnetic_field.x, mag_msg1.magnetic_field.y, mag_msg1.magnetic_field.z);
	//����������ڳ�ʼλ��
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "imu_subscriber");
	ros::NodeHandle n;
	//���ĵĶ��г�����Ϊ1��ʹ�ö��ĵ������������·���������imu��Ϣ
	ros::Subscriber imu_sub = n.subscribe("imu", 1, imu_processCallback);
	ros::Subscriber mag_sub = n.subscribe("mag", 1, mag_processCallback);
	ros::spin();
	return 0;
}