#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h> 
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include<opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

int imgInfoCallback(const sensor_msgs::ImageConstPtr &msg)
{
	try {
		imshow("img", cv_bridge::toCvShare(msg, "bgr8")->image);
		waitKey(30);
	}
	catch (cv_bridge::Exception &e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
	return 0;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "img_subscriber");
	
	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	
	//image_transport::ImageTransport it_; //����һ��image_transportʵ��  

	/*������ʾ����*/
	namedWindow("img", WINDOW_NORMAL);
	cvResizeWindow("img", 800, 600);
	/*�򿪴����Ĵ���*/
	//startWindowThread();

	image_transport::Subscriber img_info = it.subscribe("/location/image_show", 1, imgInfoCallback);
	ros::spin();

	/*�رմ���*/
	//waitKey(0);
	destroyWindow("img");
	return 0;
}
