# single_led_vlc-ROS

单灯定位文件夹(single_led_simple)

PS:注意事项
当电脑运行太久卡顿时,无法通过Ctrl+C结束时,Ctrl+Alt+T重开一个终端
查看指定进程
ps -ef|grep 关键词
如: ps -ef|grep single_led_locator
查看到当前运行的进程号 xxx
再通过下述命令杀掉对应的进程
kill -9 xxx

相机功能包（mvcam）有以下节点:
mvcam节点----实时获取拍摄到的图像

单灯定位功能包（single_led)有以下节点:
point_subscriber  ----订阅显示接收到的定位坐标结果(坐标单位是cm)

//多线程
single_led_locator_multi_spin_imu_simple----订阅接收到的角度和图像信息,进行定位计算(imu+image)
single_led_locator_multi_spin_imu_mag_simple----订阅接收到的角度和图像信息,进行定位计算(imu+mag+image)
single_led_locator_multi_spin_odom_simple----订阅接收到的角度和图像信息,进行定位计算(odom+image)
single_led_locator_multi_spin_imu_mag_filter_simple----订阅接收到的角度（松同步）和图像信息,进行定位计算(imu+mag+image)
rosrun single_led single_led_locator_multi_spin_mag_simple----订阅接收到的角度和图像信息,进行定位计算(mag)

//角度对比节点
single_led_locator_imu_mag_odom_simple

//img_subscriber    ----订阅显示接收到的图像(测试节点,实际操作时无需运行)
//imu_subscriber    ----订阅显示接收到的imu数据 (测试节点,实际操作时无需运行)

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
