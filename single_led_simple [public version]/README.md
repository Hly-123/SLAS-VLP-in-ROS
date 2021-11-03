# SLAS-VLP-in-ROS

操作流程：
ssh ubuntu@192.168.1.111或 ssh ubuntu@192.168.1.100
roslaunch turtlebot3_bringup turtlebot3.launch

ssh ubuntu@192.168.1.111或 ssh ubuntu@192.168.1.100
rosrun mvcam mvcam_ROI_max_flag

//---------------------本机运行---------------------------
//单灯定位的节点
rosrun single_led_simple single_led_locator_multi_spin_odom_ROI_from_mvcam_ID_decoding_simple
rosrun single_led_simple point_subscriber_in_class_simple

//下面是允许ekf需要的命令
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/catkin_ws/src/single_led/map/map.yaml

roslaunch robot_localization ekf_template.launch

rosrun single_led_simple ekf_translater_simple


控制命令：
rosrun image_view image_view image:=/mvcam/image

roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
